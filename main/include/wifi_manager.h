#pragma once

#include "esp_err.h"

/* Join Wi-Fi and wait until the board owns an IP address. */
esp_err_t wifi_manager_connect(void);
