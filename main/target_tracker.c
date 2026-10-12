#include "target_tracker.h"

#include <math.h>

void axis_history_clear(axis_history_t *history)
{
    history->newest = 0;
    history->count = 0;
}

void axis_history_record(axis_history_t *history, int64_t time_us, float degrees)
{
    const uint16_t slot = history->count == 0
                              ? 0
                              : (uint16_t)((history->newest + 1) % AXIS_HISTORY_LENGTH);
    history->time_us[slot] = time_us;
    history->degrees[slot] = degrees;
    history->newest = slot;
    if (history->count < AXIS_HISTORY_LENGTH) {
        history->count++;
    }
}

bool axis_history_at(const axis_history_t *history, int64_t time_us, float *degrees)
{
    if (history->count == 0) {
        return false;
    }

    uint16_t later = history->newest;
    if (time_us >= history->time_us[later]) {
        *degrees = history->degrees[later];
        return true;
    }

    /* Walk back from the newest record. Frames are rarely more than a few
     * hundred milliseconds old, so this stops after a few dozen steps. */
    for (uint16_t back = 1; back < history->count; ++back) {
        const uint16_t earlier =
            (uint16_t)((history->newest + AXIS_HISTORY_LENGTH - back) % AXIS_HISTORY_LENGTH);
        if (time_us >= history->time_us[earlier]) {
            const int64_t span = history->time_us[later] - history->time_us[earlier];
            const float fraction =
                span > 0 ? (float)(time_us - history->time_us[earlier]) / (float)span : 0.0f;
            *degrees = history->degrees[earlier] +
                       (history->degrees[later] - history->degrees[earlier]) * fraction;
            return true;
        }
        later = earlier;
    }
    return false;
}

void target_estimate_clear(target_estimate_t *estimate)
{
    estimate->valid = false;
    estimate->time_us = 0;
    estimate->degrees = 0.0f;
    estimate->speed_deg_s = 0.0f;
}

void target_estimate_update(target_estimate_t *estimate,
                            const target_filter_config_t *config,
                            int64_t time_us,
                            float measured_degrees)
{
    if (estimate->valid && time_us <= estimate->time_us) {
        return;
    }

    if (!estimate->valid || time_us - estimate->time_us > config->reacquire_gap_us) {
        estimate->valid = true;
        estimate->time_us = time_us;
        estimate->degrees = measured_degrees;
        estimate->speed_deg_s = 0.0f;
        return;
    }

    const float elapsed = (float)(time_us - estimate->time_us) / 1e6f;
    const float predicted = estimate->degrees + estimate->speed_deg_s * elapsed;
    const float surprise = measured_degrees - predicted;

    if (fabsf(surprise) > config->jump_degrees) {
        estimate->time_us = time_us;
        estimate->degrees = measured_degrees;
        estimate->speed_deg_s = 0.0f;
        return;
    }

    float speed = estimate->speed_deg_s + (config->beta / elapsed) * surprise;
    if (speed > config->max_speed_deg_s) {
        speed = config->max_speed_deg_s;
    } else if (speed < -config->max_speed_deg_s) {
        speed = -config->max_speed_deg_s;
    }

    estimate->degrees = predicted + config->alpha * surprise;
    estimate->speed_deg_s = speed;
    estimate->time_us = time_us;
}

float target_estimate_speed(const target_estimate_t *estimate,
                            const target_filter_config_t *config)
{
    const float beyond_noise = fabsf(estimate->speed_deg_s) - config->speed_noise_deg_s;
    if (beyond_noise <= 0.0f) {
        return 0.0f;
    }
    return estimate->speed_deg_s < 0.0f ? -beyond_noise : beyond_noise;
}

float target_estimate_predict(const target_estimate_t *estimate,
                              const target_filter_config_t *config,
                              int64_t time_us)
{
    int64_t ahead = time_us - estimate->time_us;
    if (ahead > config->max_extrapolation_us) {
        ahead = config->max_extrapolation_us;
    } else if (ahead < -config->max_extrapolation_us) {
        ahead = -config->max_extrapolation_us;
    }
    return estimate->degrees +
           target_estimate_speed(estimate, config) * (float)ahead / 1e6f;
}

float approach_speed(float error, float gain, float max_accel)
{
    if (gain <= 0.0f) {
        return 0.0f;
    }
    if (max_accel <= 0.0f) {
        return gain * error;
    }

    /* Beyond this distance a proportional speed could not be braked away in
     * time; below it the proportional speed already decelerates gently enough. */
    const float linear_limit = max_accel / (gain * gain);
    if (error > linear_limit) {
        return sqrtf(2.0f * max_accel * (error - 0.5f * linear_limit));
    }
    if (error < -linear_limit) {
        return -sqrtf(2.0f * max_accel * (-error - 0.5f * linear_limit));
    }
    return gain * error;
}

bool axis_hold_still(bool *settled,
                     float error_degrees,
                     bool target_moving,
                     float start_degrees,
                     float stop_degrees)
{
    const float distance = fabsf(error_degrees);
    if (*settled) {
        if (distance > start_degrees || target_moving) {
            *settled = false;
        }
    } else if (distance < stop_degrees && !target_moving) {
        *settled = true;
    }
    return *settled;
}
