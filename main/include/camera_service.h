#pragma once

#include "esp_camera.h"
#include "esp_err.h"

/* Configure the camera and its continuous JPEG frame-buffer pipeline. */
esp_err_t camera_service_start(void);
