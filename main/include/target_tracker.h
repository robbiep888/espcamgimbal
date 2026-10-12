#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Predictive tracking maths, free of ESP-IDF so it can be unit tested and
 * simulated on a desktop (see test/target_tracker_test.c).
 *
 * A frame reaches the board roughly 150-200 ms after it was exposed, and the
 * gimbal keeps moving in the meantime. Steering on "the hand is 10 degrees
 * right" from wherever the camera points NOW applies that 10 degrees twice; it
 * is the delayed feedback that made the plain proportional loop hunt as soon as
 * its gain was raised. So each axis records where it pointed, every measurement
 * becomes an absolute angle using where the camera pointed when that frame was
 * exposed, those absolute angles are filtered, and the estimate is predicted
 * forward to the present. The gimbal's own motion drops out of the loop.
 */

/* Just over a second at the 100 Hz control rate: far longer than any frame
 * that is still worth acting on. */
#define AXIS_HISTORY_LENGTH 128

typedef struct {
    int64_t time_us[AXIS_HISTORY_LENGTH];
    float degrees[AXIS_HISTORY_LENGTH];
    uint16_t newest;
    uint16_t count;
} axis_history_t;

void axis_history_clear(axis_history_t *history);

/* Record where the axis points at time_us. Times must not go backwards. */
void axis_history_record(axis_history_t *history, int64_t time_us, float degrees);

/* Where the axis pointed at time_us, interpolated between records. Later than
 * the newest record gives the newest; false if the history does not reach back
 * that far. */
bool axis_history_at(const axis_history_t *history, int64_t time_us, float *degrees);

typedef struct {
    /* Share of each surprise believed immediately (0..1). Higher follows
     * faster; lower rejects more landmark jitter. */
    float alpha;
    /* Share of each surprise turned into a speed correction. Higher picks up
     * a moving hand sooner; lower keeps a still hand's speed nearer zero. */
    float beta;
    /* A surprise bigger than this is the hand jumping somewhere new, not
     * moving fast: take the new position and forget the speed, or the filter
     * would read the jump as a burst of speed and carry the gimbal past it. */
    float jump_degrees;
    /* Estimated speeds this close to zero are landmark jitter. Only speed
     * beyond it is acted on, so a still hand never drags the gimbal around. */
    float speed_noise_deg_s;
    float max_speed_deg_s;
    /* A gap this long between measurements means the hand was lost; the next
     * measurement starts afresh instead of implying a huge speed. */
    int64_t reacquire_gap_us;
    /* How far past the last measurement a prediction may run. */
    int64_t max_extrapolation_us;
} target_filter_config_t;

typedef struct {
    bool valid;
    int64_t time_us;
    float degrees;
    float speed_deg_s;
} target_estimate_t;

void target_estimate_clear(target_estimate_t *estimate);

/* Fold in one measured absolute angle taken at time_us (an alpha-beta filter).
 * Repeated or out-of-order measurements are ignored. */
void target_estimate_update(target_estimate_t *estimate,
                            const target_filter_config_t *config,
                            int64_t time_us,
                            float measured_degrees);

/* The part of the estimated speed that is real motion rather than jitter;
 * zero for a still target. */
float target_estimate_speed(const target_estimate_t *estimate,
                            const target_filter_config_t *config);

/* Where the target is estimated to be at time_us. */
float target_estimate_predict(const target_estimate_t *estimate,
                              const target_filter_config_t *config,
                              int64_t time_us);

/*
 * The speed (degrees/s) that closes `error` degrees as quickly as `max_accel`
 * permits without overshooting: proportional to the error near the target and
 * the square-root braking curve beyond, so the axis always has room to stop.
 * The two regions meet with matching value and slope.
 */
float approach_speed(float error, float gain, float max_accel);

/*
 * Hysteresis for holding still. Returns true while the axis should not move:
 * once the error is inside stop_degrees and the target is not moving it
 * settles, and it stays settled until the error passes start_degrees or the
 * target starts moving. Without the gap the axis twitches on every frame's
 * worth of landmark jitter.
 */
bool axis_hold_still(bool *settled,
                     float error_degrees,
                     bool target_moving,
                     float start_degrees,
                     float stop_degrees);
