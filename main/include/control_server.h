#pragma once

#include "esp_err.h"

/* Start the TCP listener that receives tracking commands from the desktop. */
esp_err_t control_server_start(void);
