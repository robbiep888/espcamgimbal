#include "servo_driver.h"

#include <stdbool.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gimbal_config.h"
#include "gimbal_pins.h"

static const char *TAG = "servo";

#define SERVO_DUTY_MAX (1u << GIMBAL_SERVO_DUTY_RESOLUTION)
#define SERVO_FRAME_US (1000000 / GIMBAL_SERVO_FRAME_HZ)

/* A 14-bit duty across a 20 ms frame needs a divider of 80e6/(50*16384) ~ 98,
 * comfortably inside the timer's 10-bit integer divider. */
_Static_assert(SERVO_FRAME_US > GIMBAL_SERVO_MAX_PULSE_US,
               "The servo pulse cannot be longer than the frame that holds it");

static float current_angle = GIMBAL_TILT_CENTER_DEGREES;
static bool clipped_by_limit;

static uint32_t pulse_microseconds_to_duty(float pulse_us)
{
    const float fraction = pulse_us / (float)SERVO_FRAME_US;
    return (uint32_t)(fraction * (float)SERVO_DUTY_MAX + 0.5f);
}

static void write_angle(float degrees)
{
    const float span_us =
        (float)(GIMBAL_SERVO_MAX_PULSE_US - GIMBAL_SERVO_MIN_PULSE_US);
    const float pulse_us = (float)GIMBAL_SERVO_MIN_PULSE_US +
                           span_us * (degrees / GIMBAL_SERVO_RANGE_DEGREES);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  GIMBAL_SERVO_LEDC_CHANNEL,
                                  pulse_microseconds_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     GIMBAL_SERVO_LEDC_CHANNEL));
}

esp_err_t servo_driver_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = GIMBAL_SERVO_LEDC_TIMER,
        .duty_resolution = GIMBAL_SERVO_DUTY_RESOLUTION,
        .freq_hz = GIMBAL_SERVO_FRAME_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    esp_err_t error = ledc_timer_config(&timer);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure the servo timer: %s",
                 esp_err_to_name(error));
        return error;
    }

    const ledc_channel_config_t channel = {
        .gpio_num = GIMBAL_PIN_SERVO_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = GIMBAL_SERVO_LEDC_CHANNEL,
        .timer_sel = GIMBAL_SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };
    error = ledc_channel_config(&channel);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure the servo channel: %s",
                 esp_err_to_name(error));
        return error;
    }

    current_angle = GIMBAL_TILT_CENTER_DEGREES;
    write_angle(current_angle);

    /* Read the duty back out of the peripheral rather than trusting that the
     * write took. If this matches the requested pulse width then the LEDC
     * channel really is driving the pin, and a servo that still does not move
     * is a wiring, ground or supply problem rather than a firmware one.
     *
     * The register read here is duty_read, the LIVE counter, which is only
     * meaningful once the hardware has completed a PWM cycle. At 50 Hz that is
     * 20 ms, so wait three periods first -- reading immediately after
     * ledc_update_duty() returns a misleading zero. */
    vTaskDelay(pdMS_TO_TICKS(60));
    const float span_us =
        (float)(GIMBAL_SERVO_MAX_PULSE_US - GIMBAL_SERVO_MIN_PULSE_US);
    const float expected_us =
        (float)GIMBAL_SERVO_MIN_PULSE_US +
        span_us * (current_angle / GIMBAL_SERVO_RANGE_DEGREES);
    const uint32_t duty =
        ledc_get_duty(LEDC_LOW_SPEED_MODE, GIMBAL_SERVO_LEDC_CHANNEL);
    const float actual_us = (float)duty * (float)SERVO_FRAME_US / (float)SERVO_DUTY_MAX;
    ESP_LOGI(TAG,
             "Servo output check: duty register %u/%u -> %.0f us pulse "
             "(wanted %.0f us). A multimeter on GPIO %d vs GND should read "
             "about %.2f V",
             (unsigned)duty, (unsigned)SERVO_DUTY_MAX, (double)actual_us,
             (double)expected_us, GIMBAL_PIN_SERVO_PWM,
             (double)(3.3f * actual_us / (float)SERVO_FRAME_US));

    ESP_LOGI(TAG,
             "Tilt axis ready on GPIO %d, centred at %.0f deg, travel %.0f-%.0f deg",
             GIMBAL_PIN_SERVO_PWM,
             (double)GIMBAL_TILT_CENTER_DEGREES,
             (double)GIMBAL_TILT_MIN_DEGREES,
             (double)GIMBAL_TILT_MAX_DEGREES);
    return ESP_OK;
}

void servo_driver_set_angle(float degrees)
{
    clipped_by_limit = false;
    if (degrees < GIMBAL_TILT_MIN_DEGREES) {
        degrees = GIMBAL_TILT_MIN_DEGREES;
        clipped_by_limit = true;
    } else if (degrees > GIMBAL_TILT_MAX_DEGREES) {
        degrees = GIMBAL_TILT_MAX_DEGREES;
        clipped_by_limit = true;
    }

    current_angle = degrees;
    write_angle(current_angle);
}

float servo_driver_angle(void)
{
    return current_angle;
}

bool servo_driver_at_limit(void)
{
    return clipped_by_limit;
}
