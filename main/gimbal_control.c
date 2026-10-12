#include "gimbal_control.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gimbal_config.h"
#include "servo_driver.h"
#include "stepper_driver.h"
#include "target_tracker.h"

static const char *TAG = "gimbal";

#define CONTROL_PERIOD_TICKS (pdMS_TO_TICKS(1000 / GIMBAL_CONTROL_HZ))
#define PAN_STEPS_PER_DEGREE \
    ((GIMBAL_STEPPER_STEPS_PER_REV * GIMBAL_PAN_GEAR_RATIO) / 360.0f)
#define PAN_LIMIT_STEPS ((int32_t)(GIMBAL_PAN_LIMIT_DEGREES * PAN_STEPS_PER_DEGREE))

/* Deceleration the predictive pan approach brakes with, in deg/s^2. */
#define PAN_TRACK_ACCEL_DEG_S2 \
    (GIMBAL_TRACK_ACCEL_SHARE * GIMBAL_PAN_ACCEL_STEPS_PER_S2 / PAN_STEPS_PER_DEGREE)

/* Tilt in tracking terms: increasing it moves the view toward a positive
 * (below-centre) tilt measurement, whichever way the servo is mounted. */
#define TILT_AXIS_SIGN (GIMBAL_TILT_INVERT ? -1.0f : 1.0f)

#define TWO_PI_F 6.28318531f

/* Close enough to home that another correction would only chatter. */
#define PAN_HOME_TOLERANCE_STEPS 4
#define TILT_HOME_TOLERANCE_DEGREES 0.5f

/* A single long scheduling gap must not integrate into a large jump. */
#define MAX_TICK_SECONDS 0.05f

typedef struct {
    float pan_error;
    float tilt_error;
    uint16_t flags;
    int64_t received_at_us;
    /* The newest measurement, when the desktop is sending them. */
    bool has_measurement;
    float pan_offset_degrees;
    float tilt_offset_degrees;
    int64_t captured_at_us;
} tracking_target_t;

static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

/* Written by the control server task, read by the control loop. */
static tracking_target_t pending_target;

/* Written by the control loop, read by the control server task for replies.
 * The position is published rather than read from the stepper driver directly:
 * that is a 64-bit double written on CPU 1 while the server runs on CPU 0, and
 * an unlocked read can catch it half-written. */
static int32_t published_pan_steps;
static float published_pan_rate;
static float published_tilt_degrees;
static uint16_t published_state_flags;

/* Written and read by the control server task, under the same lock. */
static uint16_t published_latency_ms;

/* Owned exclusively by the control loop. */
static float pan_rate_command;
static float pan_rate_demand;
static float tilt_target_degrees = GIMBAL_TILT_CENTER_DEGREES;
static float tilt_smoothed_degrees = GIMBAL_TILT_CENTER_DEGREES;
static bool tilt_at_limit;
static bool recentering;
static int64_t recentering_started_us;

/* Predictive tracking, also owned by the control loop. */
static const target_filter_config_t TRACK_FILTER = {
    .alpha = GIMBAL_TRACK_FILTER_ALPHA,
    .beta = GIMBAL_TRACK_FILTER_BETA,
    .jump_degrees = GIMBAL_TRACK_JUMP_DEGREES,
    .speed_noise_deg_s = GIMBAL_TRACK_SPEED_NOISE_DEG_S,
    .max_speed_deg_s = GIMBAL_TRACK_MAX_SPEED_DEG_S,
    .reacquire_gap_us = (int64_t)GIMBAL_TRACK_REACQUIRE_GAP_MS * 1000,
    .max_extrapolation_us = (int64_t)GIMBAL_TRACK_MAX_EXTRAPOLATION_MS * 1000,
};
static axis_history_t pan_history;
static axis_history_t tilt_history;
static target_estimate_t pan_estimate;
static target_estimate_t tilt_estimate;
static int64_t consumed_capture_us;
static int64_t measurement_arrived_us;
static bool pan_settled = true;
static bool tilt_settled = true;

static float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    return value > high ? high : value;
}

/*
 * Errors inside the deadband become exactly zero, and what is left is rescaled
 * across the remaining range. Without the rescale the response would jump from
 * stopped to a fraction of full speed the moment the hand crosses the band.
 */
static float apply_deadband(float error)
{
    const float magnitude = fabsf(error);
    if (magnitude <= GIMBAL_ERROR_DEADBAND) {
        return 0.0f;
    }
    const float rescaled =
        (magnitude - GIMBAL_ERROR_DEADBAND) / (1.0f - GIMBAL_ERROR_DEADBAND);
    return error < 0.0f ? -rescaled : rescaled;
}

static float slew_limit(float current, float desired, float max_change)
{
    const float change = desired - current;
    if (change > max_change) {
        return current + max_change;
    }
    if (change < -max_change) {
        return current - max_change;
    }
    return desired;
}

static float pan_rate_for_tracking(float pan_error)
{
    const float rate = GIMBAL_PAN_GAIN_STEPS_PER_S * apply_deadband(pan_error);
    return clampf(rate, -GIMBAL_PAN_MAX_STEP_RATE, GIMBAL_PAN_MAX_STEP_RATE);
}

static float pan_rate_for_recentering(int32_t position_steps)
{
    if (position_steps > -PAN_HOME_TOLERANCE_STEPS &&
        position_steps < PAN_HOME_TOLERANCE_STEPS) {
        return 0.0f;
    }
    /* Proportional approach so the axis eases into home instead of arriving at
     * full speed. */
    float rate = -GIMBAL_RECENTER_GAIN * (float)position_steps;

    /* Close to home the proportional rate falls below the slowest pulse train
     * the driver will emit, which would stop the axis a few steps short and
     * leave the latched recentring state unable to finish. Hold the floor. */
    if (fabsf(rate) < GIMBAL_PAN_MIN_STEP_RATE) {
        rate = position_steps > 0 ? -GIMBAL_PAN_MIN_STEP_RATE
                                  : GIMBAL_PAN_MIN_STEP_RATE;
    }
    return clampf(rate, -GIMBAL_PAN_MAX_STEP_RATE, GIMBAL_PAN_MAX_STEP_RATE);
}

/* Refuse to drive further into a soft limit, but always allow motion back out. */
static float enforce_pan_limits(float rate, int32_t position_steps, bool *at_limit)
{
    *at_limit = false;
    if (position_steps >= PAN_LIMIT_STEPS && rate > 0.0f) {
        *at_limit = true;
        return 0.0f;
    }
    if (position_steps <= -PAN_LIMIT_STEPS && rate < 0.0f) {
        *at_limit = true;
        return 0.0f;
    }
    return rate;
}

static void update_tilt(float tilt_error, float elapsed_seconds, bool driving)
{
    if (driving) {
        float rate = GIMBAL_TILT_GAIN_DEG_PER_S * apply_deadband(tilt_error);
        rate = clampf(rate,
                      -GIMBAL_TILT_MAX_RATE_DEG_PER_S,
                      GIMBAL_TILT_MAX_RATE_DEG_PER_S);
#if GIMBAL_TILT_INVERT
        rate = -rate;
#endif
        const float requested = tilt_target_degrees + rate * elapsed_seconds;
        tilt_target_degrees = clampf(requested,
                                     GIMBAL_TILT_MIN_DEGREES,
                                     GIMBAL_TILT_MAX_DEGREES);
        tilt_at_limit = requested != tilt_target_degrees;
    }

    /* One-pole low-pass, evaluated exactly for whatever this tick's length was
     * rather than assuming a fixed period. */
    const float alpha =
        1.0f - expf(-TWO_PI_F * GIMBAL_TILT_SMOOTHING_HZ * elapsed_seconds);
    tilt_smoothed_degrees += (tilt_target_degrees - tilt_smoothed_degrees) * alpha;
    servo_driver_set_angle(tilt_smoothed_degrees);
}

/*
 * Turn the newest measurement into an absolute angle for each axis, using
 * where the camera pointed when its frame was exposed, and fold it into the
 * estimates. The desktop repeats a measurement until the next frame, so each
 * frame is used once.
 */
static void consume_measurement(const tracking_target_t *target)
{
    if (!target->has_measurement || target->captured_at_us == consumed_capture_us) {
        return;
    }
    consumed_capture_us = target->captured_at_us;
    measurement_arrived_us = target->received_at_us;

    const int64_t exposed_us =
        target->captured_at_us - (int64_t)GIMBAL_CAMERA_EXPOSURE_DELAY_MS * 1000;

    float pan_then = 0.0f;
    if (axis_history_at(&pan_history, exposed_us, &pan_then)) {
        target_estimate_update(&pan_estimate, &TRACK_FILTER, exposed_us,
                               pan_then + target->pan_offset_degrees);
    }

    float tilt_then = 0.0f;
    if (axis_history_at(&tilt_history,
                        exposed_us - (int64_t)GIMBAL_TILT_SERVO_LAG_MS * 1000,
                        &tilt_then)) {
        target_estimate_update(&tilt_estimate, &TRACK_FILTER, exposed_us,
                               tilt_then + target->tilt_offset_degrees);
    }
}

/* A closed hand seen again after a pause starts from its new position. */
static void forget_estimates(void)
{
    target_estimate_clear(&pan_estimate);
    target_estimate_clear(&tilt_estimate);
    pan_settled = true;
    tilt_settled = true;
}

/* Pan speed, in steps/s, that brings the camera onto where the hand is now. */
static float pan_rate_for_prediction(int64_t now_us, float pan_now_degrees)
{
    if (!pan_estimate.valid) {
        return 0.0f;
    }
    const float error =
        target_estimate_predict(&pan_estimate, &TRACK_FILTER, now_us) - pan_now_degrees;
    const float hand_speed = target_estimate_speed(&pan_estimate, &TRACK_FILTER);
    if (axis_hold_still(&pan_settled, error, hand_speed != 0.0f,
                        GIMBAL_TRACK_START_DEGREES, GIMBAL_TRACK_STOP_DEGREES)) {
        return 0.0f;
    }
    const float degrees_per_second =
        approach_speed(error, GIMBAL_TRACK_PAN_GAIN, PAN_TRACK_ACCEL_DEG_S2) +
        GIMBAL_TRACK_FEEDFORWARD * hand_speed;
    return clampf(degrees_per_second * PAN_STEPS_PER_DEGREE,
                  -GIMBAL_PAN_MAX_STEP_RATE, GIMBAL_PAN_MAX_STEP_RATE);
}

/* The servo closes its own position loop, so tilt is sent straight toward
 * where the hand is now, limited only by the slew rate. */
static void update_tilt_for_prediction(int64_t now_us, float elapsed_seconds)
{
    if (tilt_estimate.valid) {
        const float current = TILT_AXIS_SIGN * tilt_smoothed_degrees;
        const float error =
            target_estimate_predict(&tilt_estimate, &TRACK_FILTER, now_us) - current;
        const float hand_speed = target_estimate_speed(&tilt_estimate, &TRACK_FILTER);
        if (!axis_hold_still(&tilt_settled, error, hand_speed != 0.0f,
                             GIMBAL_TRACK_START_DEGREES, GIMBAL_TRACK_STOP_DEGREES)) {
            const float step = GIMBAL_TILT_MAX_RATE_DEG_PER_S * elapsed_seconds;
            const float requested = TILT_AXIS_SIGN * (current + clampf(error, -step, step));
            tilt_smoothed_degrees =
                clampf(requested, GIMBAL_TILT_MIN_DEGREES, GIMBAL_TILT_MAX_DEGREES);
            tilt_at_limit = requested != tilt_smoothed_degrees;
        }
    }
    /* Whatever follows prediction must carry on from here, not snap back to a
     * target left over from before. */
    tilt_target_degrees = tilt_smoothed_degrees;
    servo_driver_set_angle(tilt_smoothed_degrees);
}

static void gimbal_control_task(void *unused)
{
    (void)unused;
    ESP_LOGI(TAG, "Gimbal control loop is running on CPU %d at %d Hz",
             xPortGetCoreID(), GIMBAL_CONTROL_HZ);

    TickType_t next_wake = xTaskGetTickCount();
    int64_t previous_us = esp_timer_get_time();
    int64_t last_motion_us = previous_us;

    while (true) {
        xTaskDelayUntil(&next_wake, CONTROL_PERIOD_TICKS);

        const int64_t now_us = esp_timer_get_time();
        const float elapsed_seconds = clampf(
            (float)(now_us - previous_us) / 1000000.0f, 0.0f, MAX_TICK_SECONDS);
        previous_us = now_us;

        tracking_target_t target;
        portENTER_CRITICAL(&state_lock);
        target = pending_target;
        portEXIT_CRITICAL(&state_lock);

        /* Where both axes point right now. The stepper's count only advances
         * when its rate is next set, so include the pulses emitted since; tilt
         * records the angle last sent to the servo. */
        const float pan_now_degrees = stepper_driver_position_at(now_us) / PAN_STEPS_PER_DEGREE;
        axis_history_record(&pan_history, now_us, pan_now_degrees);
        axis_history_record(&tilt_history, now_us, TILT_AXIS_SIGN * tilt_smoothed_degrees);

        const bool ever_commanded = target.received_at_us != 0;
        const bool stale =
            !ever_commanded ||
            (now_us - target.received_at_us) > ((int64_t)GIMBAL_COMMAND_TIMEOUT_MS * 1000);
        const bool motors_off = (target.flags & GIMBAL_FLAG_MOTORS_OFF) != 0;
        const bool tracking =
            !stale && !motors_off && (target.flags & GIMBAL_FLAG_TRACKING) != 0;

        /* Follow measurements only while tracking and while new frames keep
         * arriving; commands alone do not count, since the desktop repeats the
         * last one until the next frame. */
        const bool measuring = tracking && (target.flags & GIMBAL_FLAG_MEASUREMENT) != 0;
        if (measuring) {
            consume_measurement(&target);
        }
        const bool predictive =
            measuring && measurement_arrived_us != 0 &&
            (now_us - measurement_arrived_us) <= (int64_t)GIMBAL_MEASUREMENT_TIMEOUT_MS * 1000;
        if (!predictive) {
            forget_estimates();
        }

        /* Recentering is latched so a single keypress completes the move, but
         * the watchdog still outranks it: losing the desktop stops everything. */
        if ((target.flags & GIMBAL_FLAG_RECENTER) != 0 && !recentering) {
            recentering = true;
            recentering_started_us = now_us;
        }
        if (stale || motors_off) {
            recentering = false;
        } else if (recentering &&
                   (now_us - recentering_started_us) >
                       ((int64_t)GIMBAL_RECENTER_TIMEOUT_MS * 1000)) {
            /* Belt and braces: recentring outranks tracking, so it must never
             * be able to stay latched and leave the gimbal unresponsive. */
            ESP_LOGW(TAG, "Recentring did not finish in %d ms; giving up",
                     GIMBAL_RECENTER_TIMEOUT_MS);
            recentering = false;
        }

        const bool following = predictive && !recentering;
        const int32_t position_steps = stepper_driver_position_steps();

        float desired_rate = 0.0f;
        if (recentering) {
            desired_rate = pan_rate_for_recentering(position_steps);
            tilt_target_degrees = GIMBAL_TILT_CENTER_DEGREES;
            if (desired_rate == 0.0f &&
                fabsf(tilt_smoothed_degrees - GIMBAL_TILT_CENTER_DEGREES) <
                    TILT_HOME_TOLERANCE_DEGREES) {
                recentering = false;
            }
        } else if (following) {
            desired_rate = pan_rate_for_prediction(now_us, pan_now_degrees);
        } else if (tracking) {
            desired_rate = pan_rate_for_tracking(target.pan_error);
        }

        bool pan_at_limit = false;
        desired_rate = enforce_pan_limits(desired_rate, position_steps, &pan_at_limit);

        if (following) {
            /* The approach curve already starts and stops smoothly; a low-pass
             * here would only put back the lag the prediction took out. */
            pan_rate_demand = desired_rate;
        } else {
            /* Smooth the demand before limiting how fast it may change. The two
             * do different jobs: the filter stops the axis chasing per-frame
             * jitter in the hand centre, the acceleration limit stops the motor
             * skipping when the demand makes a genuine jump. */
            const float rate_alpha = 1.0f - expf(-TWO_PI_F *
                                                 GIMBAL_PAN_RATE_SMOOTHING_HZ *
                                                 elapsed_seconds);
            pan_rate_demand += (desired_rate - pan_rate_demand) * rate_alpha;
        }

        /* The acceleration limit is what keeps a NEMA 17 from skipping when a
         * hand appears at the edge of frame: the rate can only change by this
         * much per tick, however large the demand is. */
        pan_rate_command = slew_limit(pan_rate_command,
                                      pan_rate_demand,
                                      GIMBAL_PAN_ACCEL_STEPS_PER_S2 * elapsed_seconds);

        const bool wants_motion = fabsf(pan_rate_command) >= GIMBAL_PAN_MIN_STEP_RATE;
        if (motors_off) {
            /* An explicit stop must not be eased into by the filter. */
            pan_rate_demand = 0.0f;
            pan_rate_command = 0.0f;
            stepper_driver_set_enabled(false);
        } else if (wants_motion) {
            last_motion_us = now_us;
            if (!stepper_driver_is_enabled()) {
                stepper_driver_set_enabled(true);
            }
        } else if (GIMBAL_IDLE_DISABLE_MS > 0 && stepper_driver_is_enabled() &&
                   (now_us - last_motion_us) >
                       ((int64_t)GIMBAL_IDLE_DISABLE_MS * 1000)) {
            /* The pan axis is vertical, so nothing back-drives it. Dropping the
             * coils keeps motor and driver cool through long idle periods. */
            ESP_LOGI(TAG, "Pan idle; de-energising the stepper");
            stepper_driver_set_enabled(false);
        }

        stepper_driver_set_rate(pan_rate_command);

        if (following) {
            update_tilt_for_prediction(now_us, elapsed_seconds);
        } else {
            /* Tilt holds its last angle unless it is actively being driven. */
            update_tilt(target.tilt_error,
                        elapsed_seconds,
                        !motors_off && (tracking || recentering));
        }

        uint16_t flags = 0;
        if (stepper_driver_is_enabled()) {
            flags |= GIMBAL_STATE_STEPPER_ENABLED;
        }
        if (tracking) {
            flags |= GIMBAL_STATE_TRACKING;
        }
        if (stale && ever_commanded) {
            flags |= GIMBAL_STATE_WATCHDOG_STOP;
        }
        if (pan_at_limit) {
            flags |= GIMBAL_STATE_PAN_AT_LIMIT;
        }
        if (tilt_at_limit) {
            flags |= GIMBAL_STATE_TILT_AT_LIMIT;
        }
        if (recentering) {
            flags |= GIMBAL_STATE_RECENTERING;
        }
        if (following) {
            flags |= GIMBAL_STATE_PREDICTIVE;
        }

        const int32_t pan_steps_now = stepper_driver_position_steps();
        portENTER_CRITICAL(&state_lock);
        published_pan_steps = pan_steps_now;
        published_pan_rate = pan_rate_command;
        published_tilt_degrees = tilt_smoothed_degrees;
        published_state_flags = flags;
        portEXIT_CRITICAL(&state_lock);
    }
}

esp_err_t gimbal_control_start(void)
{
    esp_err_t error = stepper_driver_init();
    if (error != ESP_OK) {
        return error;
    }
    error = servo_driver_init();
    if (error != ESP_OK) {
        return error;
    }

    /* Wherever the gimbal happens to be pointing at power-on is home. */
    stepper_driver_zero_position();
    axis_history_clear(&pan_history);
    axis_history_clear(&tilt_history);
    forget_estimates();

    const BaseType_t created = xTaskCreatePinnedToCore(gimbal_control_task,
                                                       "gimbal_control",
                                                       GIMBAL_TASK_STACK_BYTES,
                                                       NULL,
                                                       GIMBAL_TASK_PRIORITY,
                                                       NULL,
                                                       GIMBAL_TASK_CORE);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Pan soft limits at +/-%" PRId32 " steps (%.0f degrees)",
             (int32_t)PAN_LIMIT_STEPS, (double)GIMBAL_PAN_LIMIT_DEGREES);
    return ESP_OK;
}

void gimbal_control_submit(const gimbal_command_t *command)
{
    const int64_t now_us = esp_timer_get_time();
    const bool measurement = (command->flags & GIMBAL_FLAG_MEASUREMENT) != 0;

    /* Both times are the board's own clock: the desktop echoes the stream
     * header's capture time cut to 16 bits of milliseconds, and unsigned
     * subtraction undoes the wrap. The capture time is rebuilt from whole
     * milliseconds so every repeat of one frame yields exactly the same value;
     * mixing in this call's sub-millisecond remainder would make each repeat
     * look like a new frame a fraction of a millisecond after the last. */
    const int64_t now_ms = now_us / 1000;
    const uint16_t age_ms = (uint16_t)((uint16_t)now_ms - command->captured_at_ms);
    const bool usable = measurement && age_ms <= GIMBAL_MEASUREMENT_MAX_AGE_MS;

    portENTER_CRITICAL(&state_lock);
    pending_target.flags = command->flags;
    pending_target.received_at_us = now_us;
    if (measurement) {
        pending_target.pan_error = 0.0f;
        pending_target.tilt_error = 0.0f;
        pending_target.has_measurement = usable;
        if (usable) {
            pending_target.pan_offset_degrees = (float)command->pan / GIMBAL_CENTIDEGREES;
            pending_target.tilt_offset_degrees = (float)command->tilt / GIMBAL_CENTIDEGREES;
            pending_target.captured_at_us = (now_ms - age_ms) * 1000;
        }
        published_latency_ms = age_ms;
    } else {
        const float pan = (float)command->pan / (float)GIMBAL_Q15_ONE;
        const float tilt = (float)command->tilt / (float)GIMBAL_Q15_ONE;
        pending_target.pan_error = clampf(pan, -1.0f, 1.0f);
        pending_target.tilt_error = clampf(tilt, -1.0f, 1.0f);
        pending_target.has_measurement = false;
    }
    portEXIT_CRITICAL(&state_lock);
}

void gimbal_control_fill_status(uint32_t sequence, gimbal_status_t *status)
{
    memset(status, 0, sizeof(*status));
    status->magic = GIMBAL_STATUS_MAGIC;
    status->version = GIMBAL_PROTOCOL_VERSION;
    status->message_bytes = sizeof(*status);
    status->sequence = sequence;

    portENTER_CRITICAL(&state_lock);
    status->pan_steps = published_pan_steps;
    const float rate = published_pan_rate;
    const float tilt = published_tilt_degrees;
    status->state_flags = published_state_flags;
    status->vision_latency_ms = published_latency_ms;
    portEXIT_CRITICAL(&state_lock);

    status->pan_rate = (int16_t)lroundf(clampf(rate, -32767.0f, 32767.0f));
    status->tilt_angle_decidegrees = (int16_t)lroundf(tilt * 10.0f);
}

void gimbal_control_stop_motion(void)
{
    /* Leave the timestamp fresh and simply withdraw permission to move, so the
     * loop ramps down through the acceleration limit instead of cutting the
     * pulse train dead and risking a skipped step. */
    portENTER_CRITICAL(&state_lock);
    pending_target.pan_error = 0.0f;
    pending_target.tilt_error = 0.0f;
    pending_target.flags = 0;
    pending_target.has_measurement = false;
    pending_target.received_at_us = esp_timer_get_time();
    portEXIT_CRITICAL(&state_lock);
}
