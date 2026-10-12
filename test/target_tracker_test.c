/*
 * Host-side tests for main/target_tracker.c. No board, no ESP-IDF:
 *
 *   cc -std=c11 -Wall -Wextra -Imain/include main/target_tracker.c \
 *      test/target_tracker_test.c -lm -o /tmp/target_tracker_test && /tmp/target_tracker_test
 */
#include <math.h>
#include <stdio.h>

#include "target_tracker.h"

static int failures;

#define EXPECT_NEAR(actual, expected, tolerance)                                  \
    do {                                                                          \
        const double a_ = (actual), e_ = (expected);                              \
        if (fabs(a_ - e_) > (tolerance)) {                                        \
            printf("  FAIL %s:%d: %s = %.4f, expected %.4f\n", __func__, __LINE__, \
                   #actual, a_, e_);                                              \
            failures++;                                                           \
        }                                                                         \
    } while (0)

#define EXPECT_TRUE(condition)                                                    \
    do {                                                                          \
        if (!(condition)) {                                                       \
            printf("  FAIL %s:%d: %s\n", __func__, __LINE__, #condition);         \
            failures++;                                                           \
        }                                                                         \
    } while (0)

/* The plain filter, with the jitter threshold off so its maths is visible. */
static const target_filter_config_t FILTER = {
    .alpha = 0.6f,
    .beta = 0.4f,
    .jump_degrees = 7.0f,
    .speed_noise_deg_s = 0.0f,
    .max_speed_deg_s = 180.0f,
    .reacquire_gap_us = 300000,
    .max_extrapolation_us = 120000,
};

static void history_interpolates_between_records(void)
{
    axis_history_t history;
    axis_history_clear(&history);
    axis_history_record(&history, 0, 0.0f);
    axis_history_record(&history, 10000, 1.0f);
    axis_history_record(&history, 20000, 3.0f);

    float degrees = 0.0f;
    EXPECT_TRUE(axis_history_at(&history, 5000, &degrees));
    EXPECT_NEAR(degrees, 0.5, 1e-5);
    EXPECT_TRUE(axis_history_at(&history, 15000, &degrees));
    EXPECT_NEAR(degrees, 2.0, 1e-5);
    EXPECT_TRUE(axis_history_at(&history, 99000, &degrees));
    EXPECT_NEAR(degrees, 3.0, 1e-5);
    EXPECT_TRUE(!axis_history_at(&history, -1, &degrees));
}

static void history_forgets_what_it_cannot_hold(void)
{
    axis_history_t history;
    axis_history_clear(&history);
    for (int tick = 0; tick < 300; ++tick) {
        axis_history_record(&history, tick * 10000LL, (float)tick);
    }

    float degrees = 0.0f;
    /* The newest 128 ticks are 172..299. */
    EXPECT_TRUE(axis_history_at(&history, 250 * 10000LL + 5000, &degrees));
    EXPECT_NEAR(degrees, 250.5, 1e-4);
    EXPECT_TRUE(axis_history_at(&history, 172 * 10000LL, &degrees));
    EXPECT_NEAR(degrees, 172.0, 1e-4);
    EXPECT_TRUE(!axis_history_at(&history, 171 * 10000LL, &degrees));
}

static void empty_history_answers_nothing(void)
{
    axis_history_t history;
    axis_history_clear(&history);
    float degrees = 0.0f;
    EXPECT_TRUE(!axis_history_at(&history, 0, &degrees));
}

static void a_still_target_settles_with_no_speed(void)
{
    target_estimate_t estimate;
    target_estimate_clear(&estimate);
    for (int frame = 0; frame < 40; ++frame) {
        target_estimate_update(&estimate, &FILTER, frame * 59000LL, 12.0f);
    }
    EXPECT_NEAR(estimate.degrees, 12.0, 1e-4);
    EXPECT_NEAR(estimate.speed_deg_s, 0.0, 1e-4);
}

static void a_moving_target_is_followed_without_lag(void)
{
    /* A hand sweeping at 30 deg/s. After the filter settles, the prediction
     * for "now" must sit on the hand, not trail it. */
    target_estimate_t estimate;
    target_estimate_clear(&estimate);
    const double speed = 30.0;
    int64_t time_us = 0;
    for (int frame = 0; frame < 60; ++frame) {
        time_us = frame * 59000LL;
        target_estimate_update(&estimate, &FILTER, time_us, (float)(speed * time_us / 1e6));
    }
    EXPECT_NEAR(estimate.speed_deg_s, speed, 0.05);

    const int64_t later = time_us + 80000;
    EXPECT_NEAR(target_estimate_predict(&estimate, &FILTER, later), speed * later / 1e6, 0.05);
}

static void prediction_does_not_run_away(void)
{
    target_estimate_t estimate = {
        .valid = true, .time_us = 0, .degrees = 10.0f, .speed_deg_s = 100.0f,
    };
    /* Two seconds without a frame must not become 200 degrees of prediction. */
    EXPECT_NEAR(target_estimate_predict(&estimate, &FILTER, 2000000), 22.0, 1e-4);
}

static void a_long_gap_starts_again_instead_of_inventing_speed(void)
{
    target_estimate_t estimate;
    target_estimate_clear(&estimate);
    target_estimate_update(&estimate, &FILTER, 0, 0.0f);
    target_estimate_update(&estimate, &FILTER, 59000, 0.0f);
    /* Hand lost for a second, then seen 40 degrees away. */
    target_estimate_update(&estimate, &FILTER, 1059000, 40.0f);
    EXPECT_NEAR(estimate.degrees, 40.0, 1e-4);
    EXPECT_NEAR(estimate.speed_deg_s, 0.0, 1e-4);
}

static void a_jump_is_a_new_position_not_a_burst_of_speed(void)
{
    target_estimate_t estimate;
    target_estimate_clear(&estimate);
    for (int frame = 0; frame < 10; ++frame) {
        target_estimate_update(&estimate, &FILTER, frame * 59000LL, 0.0f);
    }
    /* The hand moves 20 degrees between two frames. */
    target_estimate_update(&estimate, &FILTER, 10 * 59000LL, 20.0f);
    EXPECT_NEAR(estimate.degrees, 20.0, 1e-4);
    EXPECT_NEAR(estimate.speed_deg_s, 0.0, 1e-4);
}

static void repeated_and_old_measurements_are_ignored(void)
{
    target_estimate_t estimate;
    target_estimate_clear(&estimate);
    target_estimate_update(&estimate, &FILTER, 100000, 5.0f);
    target_estimate_update(&estimate, &FILTER, 100000, 6.0f);
    target_estimate_update(&estimate, &FILTER, 50000, 6.0f);
    EXPECT_NEAR(estimate.degrees, 5.0, 1e-5);
    EXPECT_NEAR(estimate.speed_deg_s, 0.0, 1e-5);
}

static void jitter_sized_speed_is_not_acted_on(void)
{
    target_filter_config_t config = FILTER;
    config.speed_noise_deg_s = 8.0f;
    const target_estimate_t jittering = {
        .valid = true, .time_us = 0, .degrees = 3.0f, .speed_deg_s = -5.0f,
    };
    EXPECT_NEAR(target_estimate_speed(&jittering, &config), 0.0, 1e-6);
    EXPECT_NEAR(target_estimate_predict(&jittering, &config, 60000), 3.0, 1e-6);

    const target_estimate_t moving = {
        .valid = true, .time_us = 0, .degrees = 3.0f, .speed_deg_s = -30.0f,
    };
    EXPECT_NEAR(target_estimate_speed(&moving, &config), -22.0, 1e-4);
}

static void approach_speed_meets_its_braking_curve_smoothly(void)
{
    const float gain = 16.0f;
    const float accel = 720.0f;
    const float boundary = accel / (gain * gain);
    EXPECT_NEAR(approach_speed(boundary * 0.99999f, gain, accel),
                approach_speed(boundary * 1.00001f, gain, accel), 0.005);
    EXPECT_NEAR(approach_speed(-3.0f, gain, accel), -approach_speed(3.0f, gain, accel), 1e-5);
    EXPECT_NEAR(approach_speed(0.0f, gain, accel), 0.0, 1e-6);
}

static void approach_speed_can_always_stop_in_time(void)
{
    /* Braking at max_accel from the commanded speed must never need more
     * distance than is left, or the axis overshoots the target. */
    const float gain = 16.0f;
    const float accel = 720.0f;
    for (float error = 0.1f; error < 90.0f; error *= 1.3f) {
        const float speed = approach_speed(error, gain, accel);
        EXPECT_TRUE(speed * speed / (2.0f * accel) <= error + 1e-3f);
    }
}

static void holding_still_needs_a_real_reason_to_move(void)
{
    bool settled = true;
    /* Jitter inside the start threshold does not wake a settled axis. */
    EXPECT_TRUE(axis_hold_still(&settled, 0.6f, false, 0.8f, 0.25f));
    /* A real offset does. */
    EXPECT_TRUE(!axis_hold_still(&settled, 1.2f, false, 0.8f, 0.25f));
    /* Once moving it keeps going past the start threshold, down to stop. */
    EXPECT_TRUE(!axis_hold_still(&settled, 0.6f, false, 0.8f, 0.25f));
    EXPECT_TRUE(axis_hold_still(&settled, 0.2f, false, 0.8f, 0.25f));
    /* A moving target wakes it however small the offset. */
    EXPECT_TRUE(!axis_hold_still(&settled, 0.0f, true, 0.8f, 0.25f));
}

int main(void)
{
    history_interpolates_between_records();
    history_forgets_what_it_cannot_hold();
    empty_history_answers_nothing();
    a_still_target_settles_with_no_speed();
    a_moving_target_is_followed_without_lag();
    prediction_does_not_run_away();
    a_long_gap_starts_again_instead_of_inventing_speed();
    a_jump_is_a_new_position_not_a_burst_of_speed();
    repeated_and_old_measurements_are_ignored();
    jitter_sized_speed_is_not_acted_on();
    approach_speed_meets_its_braking_curve_smoothly();
    approach_speed_can_always_stop_in_time();
    holding_still_needs_a_real_reason_to_move();

    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("target_tracker: all checks passed\n");
    return 0;
}
