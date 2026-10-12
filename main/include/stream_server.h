#pragma once

#include "esp_err.h"

/* Start the capture and TCP video streaming tasks. */
esp_err_t stream_server_start(void);
