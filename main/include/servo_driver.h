#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * MG996R tilt axis on a 50 Hz LEDC channel. The servo closes its own position
 * loop, so this module only converts degrees to a pulse width and clamps the
 * result to the travel configured in gimbal_config.h.
 */
esp_err_t servo_driver_init(void);

/* Command an absolute angle in degrees; clamped to the configured travel. */
void servo_driver_set_angle(float degrees);

float servo_driver_angle(void);

/* True when the last commanded angle was clipped by a travel limit. */
bool servo_driver_at_limit(void);
