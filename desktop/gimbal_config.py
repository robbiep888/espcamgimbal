"""The few values from main/include/gimbal_config.h the desktop has to know.

The board reports pan as a microstep count and tilt as the raw servo angle, and
the jog and test tools command step rates through the firmware's gain and
deadband. Turning either into degrees needs the same numbers the firmware was
built with.

Each tool used to keep its own copy, and three of them went stale when the
driver moved from 1/8 to 1/16 microstepping and the gain was retuned:
axis_selftest.py asked for 150 steps/s and got about 790, sweeping the pan axis
into its soft limits, and jog.py showed every angle doubled. Now there is one
copy, and test_gimbal_config.py reads gimbal_config.h and fails if anything
here drifts from it. Change the header, then this file.
"""

from __future__ import annotations

FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
PAN_GEAR_RATIO = 1.0
PAN_STEPS_PER_DEGREE = FULL_STEPS_PER_REV * MICROSTEPS * PAN_GEAR_RATIO / 360.0

# The proportional path, which the jog and test tools drive.
PAN_GAIN_STEPS_PER_S = 1050.0
ERROR_DEADBAND = 0.040

# The servo angle the tilt axis powers up at, and whether increasing the angle
# moves the view up (inverted) rather than down.
TILT_CENTER_DEGREES = 90.0
TILT_INVERT = False

# Where a frame's exposure sits relative to its capture timestamp, and how far
# the servo trails the angle it was sent: the board looks up where the camera
# pointed with these, and so does the hand lock.
CAMERA_EXPOSURE_DELAY_MS = 55
TILT_SERVO_LAG_MS = 45


def pan_degrees(pan_steps: int) -> float:
    """A status report's pan position in degrees from home."""
    return pan_steps / PAN_STEPS_PER_DEGREE


def tilt_degrees_from_center(servo_degrees: float) -> float:
    """A status report's servo angle as degrees from home, positive when the
    view has moved down (the same sense as a hand below centre)."""
    sign = -1.0 if TILT_INVERT else 1.0
    return sign * (servo_degrees - TILT_CENTER_DEGREES)


def error_for_step_rate(steps_per_second: float) -> float:
    """The proportional error that makes the firmware command this step rate:
    gimbal_control.c's deadband and gain, run backwards."""
    return ERROR_DEADBAND + (steps_per_second / PAN_GAIN_STEPS_PER_S) * (1.0 - ERROR_DEADBAND)
