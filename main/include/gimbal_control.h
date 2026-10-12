#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control_protocol.h"
#include "esp_err.h"

/* Bring up both axes and start the fixed-rate motion loop. */
esp_err_t gimbal_control_start(void);

/*
 * Hand one decoded command to the motion loop. Safe to call from the control
 * server task; the loop picks it up on its next tick.
 */
void gimbal_control_submit(const gimbal_command_t *command);

/* Fill in the current axis state for the reply to the desktop. */
void gimbal_control_fill_status(uint32_t sequence, gimbal_status_t *status);

/* Ramp to a stop immediately, used when the control connection drops. */
void gimbal_control_stop_motion(void);
