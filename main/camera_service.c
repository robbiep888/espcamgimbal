#include "camera_service.h"

#include <stdbool.h>

#include "app_config.h"
#include "board_pins.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "sensor.h"

static const char *TAG = "camera";

esp_err_t camera_service_start(void)
{
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM is not initialized; check the N16R8 sdkconfig settings");
        return ESP_FAIL;
    }

    const camera_config_t config = {
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,
        .pin_d7 = CAMERA_PIN_D7,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,
        .xclk_freq_hz = CAMERA_XCLK_HZ,
        /* LEDC timer 0 and channel 0 belong to XCLK. The gimbal uses timers
         * 1 and 2; see gimbal_config.h. */
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = CAMERA_FRAME_SIZE,
        .jpeg_quality = CAMERA_JPEG_QUALITY,
        .fb_count = CAMERA_FRAME_BUFFER_COUNT,
        .fb_location = CAMERA_FB_IN_PSRAM,
        /* Completed frames remain owned by the application until returned. */
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    ESP_LOGI(TAG, "Starting camera with %u bytes of PSRAM free",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    esp_err_t error = esp_camera_init(&config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s (0x%x)",
                 esp_err_to_name(error), (unsigned)error);
        ESP_LOGE(TAG, "The most common cause is an incorrect pin table in board_pins.h");
        ESP_LOGE(TAG, "On this build, also check that nothing in gimbal_pins.h was "
                      "moved onto a camera pin");
        return error;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Camera initialised but no sensor handle was returned");
        esp_camera_deinit();
        return ESP_ERR_NOT_FOUND;
    }

    const bool is_ov3660 = sensor->id.PID == OV3660_PID;
    if (!is_ov3660) {
        ESP_LOGW(TAG,
                 "Detected sensor PID 0x%04x, not the OV3660 (0x%04x) this "
                 "project was tuned for. Streaming anyway with stock timing.",
                 sensor->id.PID, OV3660_PID);
    }

    /* Keep the external XCLK at 16 MHz so it does not spoil Wi-Fi, then tune
     * only the sensor's internal JPEG/pixel pipeline. */
#if CAMERA_USE_FAST_16MHZ_PLL
    if (is_ov3660 && sensor->set_pll(sensor,
                                     0,
                                     CAMERA_PLL_MULTIPLIER,
                                     1,
                                     0,
                                     3,
                                     0,
                                     1,
                                     CAMERA_PLL_PIXEL_DIVIDER) != 0) {
        ESP_LOGE(TAG, "Could not apply the fast OV3660 PLL profile");
        esp_camera_deinit();
        return ESP_FAIL;
    }
#endif

    /* A physical 180-degree rotation requires both vertical flip and mirror. */
#if CAMERA_ROTATE_180
    if (sensor->set_vflip(sensor, 1) != 0 ||
        sensor->set_hmirror(sensor, 1) != 0) {
        ESP_LOGE(TAG, "Could not rotate the camera image");
        esp_camera_deinit();
        return ESP_FAIL;
    }
#endif

    /* The OV3660 defaults can look a little flat; these values match
     * Espressif's camera example. */
    if (sensor->set_brightness(sensor, 1) != 0 ||
        sensor->set_saturation(sensor, -2) != 0) {
        ESP_LOGW(TAG, "Could not apply the image settings; continuing with defaults");
    }

    ESP_LOGI(TAG,
             "Camera ready: PID=0x%04x, JPEG quality=%d, buffers=%d, rotated=%s",
             sensor->id.PID,
             CAMERA_JPEG_QUALITY,
             CAMERA_FRAME_BUFFER_COUNT,
             CAMERA_ROTATE_180 ? "yes" : "no");
    return ESP_OK;
}
