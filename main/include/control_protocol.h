#pragma once

#include <stdint.h>

/*
 * Tracking control channel, TCP port CONTROL_TCP_PORT.
 *
 * Every exchange is one fixed-size command answered by one fixed-size status
 * message, so there is no framing state to get out of step and a late reply
 * cannot desynchronise the stream.
 *
 * A command carries one of two things in its pan and tilt fields:
 *
 *  - With GIMBAL_FLAG_MEASUREMENT, where the hand was: its angle from the
 *    camera axis in hundredths of a degree, as seen in the video frame the
 *    board stamped captured_at_ms. Positive pan is right of centre in the
 *    displayed image, positive tilt is below it. The board works out where
 *    the hand is now and decides how to get there (see target_tracker.h).
 *
 *  - Without it, a manual speed in Q15 fixed point: -32767..32767 maps to
 *    -1.0..+1.0 of the proportional range. The jog and test tools use this.
 */

#define GIMBAL_COMMAND_MAGIC 0x4743544cU /* "GCTL" */
#define GIMBAL_STATUS_MAGIC 0x47535441U  /* "GSTA" */
/* 2 added measurements. A version 1 desktop is refused rather than having its
 * speeds misread as angles. */
#define GIMBAL_PROTOCOL_VERSION 2U

#define GIMBAL_Q15_ONE 32767
#define GIMBAL_CENTIDEGREES 100.0f

/* Command flags, desktop -> board. */
#define GIMBAL_FLAG_TRACKING (1U << 0)     /* a closed hand is visible: follow */
#define GIMBAL_FLAG_HAND_VISIBLE (1U << 1) /* any hand is visible (telemetry) */
#define GIMBAL_FLAG_RECENTER (1U << 2)     /* return both axes to home */
#define GIMBAL_FLAG_MOTORS_OFF (1U << 3)   /* de-energise the stepper now */
#define GIMBAL_FLAG_MEASUREMENT (1U << 4)  /* pan/tilt are a measured hand angle */

/* State flags, board -> desktop. */
#define GIMBAL_STATE_STEPPER_ENABLED (1U << 0)
#define GIMBAL_STATE_TRACKING (1U << 1)
#define GIMBAL_STATE_WATCHDOG_STOP (1U << 2)
#define GIMBAL_STATE_PAN_AT_LIMIT (1U << 3)
#define GIMBAL_STATE_TILT_AT_LIMIT (1U << 4)
#define GIMBAL_STATE_RECENTERING (1U << 5)
#define GIMBAL_STATE_PREDICTIVE (1U << 6) /* following measurements */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t message_bytes;
    uint32_t sequence;
    int16_t pan;
    int16_t tilt;
    uint16_t flags;
    /* The frame's captured_at_us from the stream header, in milliseconds,
     * modulo 65536. Only meaningful with GIMBAL_FLAG_MEASUREMENT. */
    uint16_t captured_at_ms;
} gimbal_command_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t message_bytes;
    uint32_t sequence; /* echoes the command this answers */
    int32_t pan_steps; /* estimated position from home, in microsteps */
    int16_t pan_rate;  /* signed step rate currently commanded, steps/s */
    int16_t tilt_angle_decidegrees;
    uint16_t state_flags;
    /* Capture to arrival for the newest measurement, in milliseconds: how old
     * the board's picture of the hand is. 0 until one arrives. */
    uint16_t vision_latency_ms;
} gimbal_status_t;

_Static_assert(sizeof(gimbal_command_t) == 20,
               "gimbal_command_t must match desktop/control_protocol.py");
_Static_assert(sizeof(gimbal_status_t) == 24,
               "gimbal_status_t must match desktop/control_protocol.py");
