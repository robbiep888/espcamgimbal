#pragma once

#include <stdint.h>

/* ASCII "OCAM" when written in network byte order. */
#define STREAM_MAGIC 0x4f43414dU
#define STREAM_PROTOCOL_VERSION 1U

/*
 * Every JPEG is preceded by this fixed 32-byte header. Multi-byte fields are
 * sent in network byte order, so the desktop code is portable.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;
    uint32_t sequence;
    uint64_t captured_at_us;
    uint32_t jpeg_bytes;
    uint16_t width;
    uint16_t height;
    uint32_t dropped_frames;
} stream_frame_header_t;

_Static_assert(sizeof(stream_frame_header_t) == 32,
               "The desktop and firmware protocol headers must stay in sync");
