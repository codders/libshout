/* -*- c-basic-offset: 8; -*- */
/* aac.c: libshout AAC format handler
 *
 *  Copyright (C) 2010 Aupeo GmbH, Arthur Taylor <arthur@aupeo.com>
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * AAC frame handling courtesy Arthur Taylor
 */

#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <shout/shout.h>
#include "shout_private.h"

#define ADTS_HEADER_SIZE 8 /* Actual ADTS Header size -> 56 bits */

#define min(a,b) (a<b ? a : b)

#undef TRACE
#ifdef TRACE
#define trace(x, ...) fprintf(stderr, x, ##__VA_ARGS__)
#else
#define trace(x, ...) ;
#endif

#define DEBUG
#ifdef DEBUG
#define debug(x, ...) fprintf(stderr, x, ##__VA_ARGS__)
#else
#define debug(x, ...) ;
#endif

typedef struct {
    int version;
    int channels;
    int sampling_rate;
    int bitrate;
    int length;
    int object_type;
    int headertype;
} faadAACInfo;

static int sample_rates[] = {96000,88200,64000,48000,44100,32000,24000,22050,16000,12000,11025,8000};

typedef enum {
  SEEK,
  PARSE_HEADER,
  READ_FRAME
} ParseState_e;

struct aac_data {
  faadAACInfo *info;
  ParseState_e state;
  unsigned char *buffer;
  int buffer_position;
  int buffer_length;
  int buffer_allocated;
  int frame_length;
  int frames_sent;
  float frames_per_second;
};
typedef struct aac_data aac_data_t;

/* -- static prototypes -- */
static int process_local_buffer(shout_t *self, aac_data_t *data);
static int send_aac(shout_t *self, const unsigned char *data, size_t len);
static void close_aac(shout_t *self);

int shout_open_aac(shout_t *self)
{
  aac_data_t *aac_data;
  if (!(aac_data = (aac_data_t *)calloc(1, sizeof(aac_data_t))))
    return SHOUTERR_MALLOC;
  self->format_data = aac_data;

  memset(aac_data, 0, sizeof(aac_data));

  aac_data->info = (faadAACInfo *)calloc(1, sizeof(faadAACInfo));
  aac_data->state = SEEK;

  self->send = send_aac;
  self->close = close_aac;

  return SHOUTERR_SUCCESS;
}

static void copy_to_local_buffer(aac_data_t *data, const unsigned char *buf, size_t offset, size_t length) {
  trace("%s\n", __FUNCTION__);
  data->buffer = malloc(length);
  memcpy(data->buffer, buf + offset, length);
  data->buffer_position = 0;
  data->buffer_length = length;
  data->buffer_allocated = length;
}

static void append_to_local_buffer(aac_data_t *data, const unsigned char *buf, size_t length) {
  trace("%s\n", __FUNCTION__);
  if (data->buffer_allocated - data->buffer_length < length) {
    data->buffer = realloc(data->buffer, data->buffer_length + length);
    data->buffer_allocated = data->buffer_length + length;
  }
  memcpy(data->buffer + data->buffer_length, buf, length);
  data->buffer_length += length;
}

static void read_header_data(aac_data_t *data) {
  trace("%s\n", __FUNCTION__);
  int version_id = data->buffer[1] & 0x08;
  data->info->object_type = (data->buffer[2]&0xC0)>>6;
  int sr_idx = (data->buffer[2]&0x3C)>>2;
  data->info->channels = ((data->buffer[2]&0x01)<<2)|((data->buffer[3]&0xC0)>>6);

  data->frames_per_second = sample_rates[sr_idx] / 1024.f;
  trace("Frames per sec: %f\n", data->frames_per_second);

  if (version_id == 0) {
    data->info->version = 4;
  } else { /* MPEG-2 */
    data->info->version = 2;
  }
// XXX Bail if frame is too long
  data->frame_length = ((((unsigned int)data->buffer[3] & 0x3)) << 11)
                      | (((unsigned int)data->buffer[4]) << 3) | (data->buffer[5] >> 5);
}

static void shift_data_left(aac_data_t *data, size_t amount) {
  trace("%s\n", __FUNCTION__);
  size_t shift = min(amount, data->buffer_length);
  memmove(data->buffer, data->buffer + shift, data->buffer_length - shift);
  data->buffer_length -= shift;
}

static int send_frame(shout_t *self, aac_data_t *data) {
  trace("%s\n", __FUNCTION__);
  int ret;
  if (data->buffer_length < data->frame_length)
   return SHOUTERR_SUCCESS;

  data->frames_sent++;
  self->senttime = (int64_t)((double)data->frames_sent * 1000000/(double)data->frames_per_second);

  ret = shout_send_raw(self, data->buffer, data->frame_length);
  if (ret != data->frame_length)
    return SHOUTERR_SOCKET;

  shift_data_left(data, data->frame_length);
  data->state = PARSE_HEADER;
  return process_local_buffer(self, data);
}

static int valid_header_bytes(const unsigned char *buf) {
  return buf[0] == 0xFF && (buf[1] & 0xF6) == 0xF0;
}

static int process_local_buffer(shout_t *self, aac_data_t *data) {
  trace("%s\n", __FUNCTION__);
  if (!data->buffer) {
    return SHOUTERR_SUCCESS;
  }
  switch (data->state) {
    case PARSE_HEADER:
      if (data->buffer_length < ADTS_HEADER_SIZE) {
        return SHOUTERR_SUCCESS;
      }
      if (!valid_header_bytes(data->buffer)) {
        fprintf(stderr, "Invalid header bytes detected. Quittung\n");
        exit(1);
      }
      read_header_data(data);
      data->state = READ_FRAME;
      return process_local_buffer(self, data);
    case READ_FRAME:
      return send_frame(self, data);
    case SEEK:
      fprintf(stderr, "Shouldn't get here\n");
      exit(1);
      break;
  }
  return SHOUTERR_SUCCESS;
}

static int send_aac(shout_t* self, const unsigned char* buf, size_t len)
{
  aac_data_t *data = (aac_data_t *)self->format_data;
  int i = 0;
  switch (data->state) {
    case SEEK:
      debug("Looking for frame header\n");
      while (i<len-2) {
        if (valid_header_bytes(buf + i)) {
          debug("Found Frame Header\n");
          copy_to_local_buffer(data, buf, i, len - i);
          data->state = PARSE_HEADER;
          if ((self->error = process_local_buffer(self, data)) != SHOUTERR_SUCCESS)
            return self->error;
          break;
        }
        i++;
      }
      break;
    default:
      debug("Copying data to end of existing buffer\n");
      append_to_local_buffer(data, buf, len);
      return self->error = process_local_buffer(self, data);
  }
  return self->error = SHOUTERR_SUCCESS;
}

static void close_aac(shout_t *self) {
  aac_data_t *aac_data = (aac_data_t *)self->format_data;
  free(aac_data->buffer);
  free(aac_data->info);
  free(aac_data);
}


