#include "app_config.h"
#include "camera_service.h"
#include "control_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "gimbal_control.h"
#include "stream_server.h"
#include "wifi_manager.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 hand-tracking gimbal camera starting");

    /*
     * Bring the axes up first, before Wi-Fi and the camera. The stepper starts
     * de-energised and the servo is parked at centre, so this is the earliest
     * point at which the hardware is in a known, safe state -- important when
     * the motors share the 5 V supply with the board.
     */
#if GIMBAL_DISABLED_DIAGNOSTIC
    ESP_LOGW(TAG, "Gimbal diagnostic mode: motors are not initialised");
#else
    ESP_ERROR_CHECK(gimbal_control_start());
#endif

    /* Wi-Fi before camera DMA. Besides making network failures easier to
     * diagnose, this keeps camera activity out of the WPA handshake. */
    ESP_ERROR_CHECK(wifi_manager_connect());

#if WIFI_ONLY_DIAGNOSTIC
    ESP_LOGW(TAG, "Wi-Fi-only diagnostic mode: camera and stream are disabled");
#elif CAMERA_IDLE_DIAGNOSTIC
    ESP_ERROR_CHECK(camera_service_start());
    ESP_LOGW(TAG, "Camera-idle diagnostic mode: stream task is disabled");
#else
    ESP_ERROR_CHECK(camera_service_start());
    ESP_ERROR_CHECK(stream_server_start());
#endif

#if !GIMBAL_DISABLED_DIAGNOSTIC
    ESP_ERROR_CHECK(control_server_start());
#endif

    ESP_LOGI(TAG, "Startup complete: video on port %d, tracking on port %d",
             STREAM_TCP_PORT, CONTROL_TCP_PORT);
}
