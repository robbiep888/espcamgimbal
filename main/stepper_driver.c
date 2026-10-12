#include "stepper_driver.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "gimbal_config.h"
#include "gimbal_pins.h"

static const char *TAG = "stepper";

/* LEDC low-speed timers on the ESP32-S3 divide the 80 MHz APB clock by
 * (2^resolution * divider). The divider's integer part is 10 bits wide, which
 * fixes the range of step rates the hardware can actually produce. */
#define STEPPER_SOURCE_HZ 80000000u
#define STEPPER_DUTY_MAX (1u << GIMBAL_STEPPER_DUTY_RESOLUTION)
#define STEPPER_DUTY_HALF (STEPPER_DUTY_MAX / 2u)
#define STEPPER_HW_MAX_HZ (STEPPER_SOURCE_HZ / STEPPER_DUTY_MAX)
#define STEPPER_HW_MIN_HZ ((STEPPER_SOURCE_HZ / STEPPER_DUTY_MAX / 1023u) + 1u)

_Static_assert((int)GIMBAL_PAN_MAX_STEP_RATE <= (int)STEPPER_HW_MAX_HZ,
               "GIMBAL_PAN_MAX_STEP_RATE exceeds what the LEDC timer can emit "
               "at this duty resolution");
_Static_assert((int)GIMBAL_PAN_MIN_STEP_RATE >= (int)STEPPER_HW_MIN_HZ,
               "GIMBAL_PAN_MIN_STEP_RATE is below the LEDC timer's slowest "
               "achievable frequency at this duty resolution");

/* Position when the rate last changed, and when that was. Only the gimbal task
 * touches these; the control server reads a copy published under its lock. */
static double position_steps;
static int64_t rate_changed_us;
static float commanded_rate;
static uint32_t programmed_frequency;
static bool moving_forward = true;
static bool coils_enabled;
static bool pulses_running;

/*
 * Fold the pulses emitted at the current rate since it was set into the
 * position, using the real time that passed. The control loop's own tick is
 * clamped so one late tick cannot jolt the motion maths, but the hardware kept
 * pulsing through all of it: integrating the clamped time dropped those steps
 * from the estimate, and so did cutting the motors mid-move.
 */
static void account_for_pulses(int64_t now_us)
{
    position_steps += (double)commanded_rate * (double)(now_us - rate_changed_us) / 1e6;
    rate_changed_us = now_us;
}

static void set_pulse_output(bool running)
{
    if (running == pulses_running) {
        return;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  GIMBAL_STEPPER_LEDC_CHANNEL,
                                  running ? STEPPER_DUTY_HALF : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     GIMBAL_STEPPER_LEDC_CHANNEL));
    pulses_running = running;
}

esp_err_t stepper_driver_init(void)
{
    /*
     * Park ENABLE at its inactive level in the output register BEFORE the pin
     * becomes an output. gpio_config() would otherwise drive it from the
     * register's reset value of 0 -- which on an active-low driver is
     * "enabled" -- energising the coils for the microseconds until the level
     * is corrected.
     *
     * None of this helps during the ~885 ms of boot before app_main runs, when
     * every pin is high-impedance and the driver's own internal pull-down
     * enables it. Only an external pull-up on ENABLE covers that window; see
     * gimbal_pins.h.
     */
    gpio_set_level(GIMBAL_PIN_STEPPER_ENABLE,
                   GIMBAL_STEPPER_ENABLE_ACTIVE_LOW ? 1 : 0);

    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << GIMBAL_PIN_STEPPER_DIR) |
                        (1ULL << GIMBAL_PIN_STEPPER_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&outputs);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure DIR/ENABLE: %s", esp_err_to_name(error));
        return error;
    }

    /* Weakly hold ENABLE inactive if anything ever leaves the pin undriven. */
    gpio_set_pull_mode(GIMBAL_PIN_STEPPER_ENABLE,
                       GIMBAL_STEPPER_ENABLE_ACTIVE_LOW ? GPIO_PULLUP_ONLY
                                                        : GPIO_PULLDOWN_ONLY);

    /* De-energised until the control loop asks for motion. */
    stepper_driver_set_enabled(false);
    gpio_set_level(GIMBAL_PIN_STEPPER_DIR, GIMBAL_PAN_INVERT ? 0 : 1);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = GIMBAL_STEPPER_LEDC_TIMER,
        .duty_resolution = GIMBAL_STEPPER_DUTY_RESOLUTION,
        /* Any legal starting point; the control loop reprograms this. */
        .freq_hz = (uint32_t)GIMBAL_PAN_MIN_STEP_RATE,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    error = ledc_timer_config(&timer);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure the step-pulse timer: %s",
                 esp_err_to_name(error));
        return error;
    }
    programmed_frequency = (uint32_t)GIMBAL_PAN_MIN_STEP_RATE;

    const ledc_channel_config_t channel = {
        .gpio_num = GIMBAL_PIN_STEPPER_STEP,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = GIMBAL_STEPPER_LEDC_CHANNEL,
        .timer_sel = GIMBAL_STEPPER_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0, /* no pulses yet */
        .hpoint = 0,
    };
    error = ledc_channel_config(&channel);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure the step-pulse channel: %s",
                 esp_err_to_name(error));
        return error;
    }

    ESP_LOGI(TAG, "Pan direction: %s (GIMBAL_PAN_INVERT=%d)",
             GIMBAL_PAN_INVERT ? "REVERSED" : "normal", GIMBAL_PAN_INVERT);
    ESP_LOGI(TAG,
             "Pan axis ready on STEP=%d DIR=%d EN=%d, %d steps/rev, "
             "%.0f-%.0f steps/s",
             GIMBAL_PIN_STEPPER_STEP,
             GIMBAL_PIN_STEPPER_DIR,
             GIMBAL_PIN_STEPPER_ENABLE,
             GIMBAL_STEPPER_STEPS_PER_REV,
             GIMBAL_PAN_MIN_STEP_RATE,
             GIMBAL_PAN_MAX_STEP_RATE);
    return ESP_OK;
}

void stepper_driver_set_enabled(bool enabled)
{
    if (!enabled) {
        account_for_pulses(esp_timer_get_time());
        set_pulse_output(false);
        commanded_rate = 0.0f;
    }
#if GIMBAL_STEPPER_ENABLE_ACTIVE_LOW
    gpio_set_level(GIMBAL_PIN_STEPPER_ENABLE, enabled ? 0 : 1);
#else
    gpio_set_level(GIMBAL_PIN_STEPPER_ENABLE, enabled ? 1 : 0);
#endif
    coils_enabled = enabled;
}

bool stepper_driver_is_enabled(void)
{
    return coils_enabled;
}

void stepper_driver_set_rate(float steps_per_second)
{
    /* Account for the pulses the timer emitted while the previous rate was in
     * force, before changing anything. */
    account_for_pulses(esp_timer_get_time());

    const float magnitude = fabsf(steps_per_second);
    if (!coils_enabled || magnitude < GIMBAL_PAN_MIN_STEP_RATE) {
        set_pulse_output(false);
        commanded_rate = 0.0f;
        return;
    }

    const bool want_forward = steps_per_second > 0.0f;
    if (want_forward != moving_forward) {
        /* Never change direction under a live pulse train: a step edge landing
         * inside the driver's direction setup window is a lost or doubled
         * microstep. */
        set_pulse_output(false);
        gpio_set_level(GIMBAL_PIN_STEPPER_DIR,
                       want_forward != (GIMBAL_PAN_INVERT != 0) ? 1 : 0);
        esp_rom_delay_us(GIMBAL_STEPPER_DIR_SETUP_US);
        moving_forward = want_forward;
    }

    uint32_t frequency = (uint32_t)lroundf(magnitude);
    if (frequency > (uint32_t)GIMBAL_PAN_MAX_STEP_RATE) {
        frequency = (uint32_t)GIMBAL_PAN_MAX_STEP_RATE;
    }
    if (frequency < STEPPER_HW_MIN_HZ) {
        frequency = STEPPER_HW_MIN_HZ;
    }

    if (frequency != programmed_frequency) {
        const esp_err_t error = ledc_set_freq(
            LEDC_LOW_SPEED_MODE, GIMBAL_STEPPER_LEDC_TIMER, frequency);
        if (error != ESP_OK) {
            /* Refuse to guess: leave the previous, known-good rate running. */
            ESP_LOGW(TAG, "Rejected step rate %" PRIu32 " Hz: %s",
                     frequency, esp_err_to_name(error));
            return;
        }
        programmed_frequency = frequency;
    }

    set_pulse_output(true);
    commanded_rate = want_forward ? (float)frequency : -(float)frequency;
}

float stepper_driver_rate(void)
{
    return commanded_rate;
}

int32_t stepper_driver_position_steps(void)
{
    return (int32_t)llround(position_steps);
}

float stepper_driver_position_at(int64_t now_us)
{
    return (float)(position_steps +
                   (double)commanded_rate * (double)(now_us - rate_changed_us) / 1e6);
}

void stepper_driver_zero_position(void)
{
    position_steps = 0.0;
    rate_changed_us = esp_timer_get_time();
}
