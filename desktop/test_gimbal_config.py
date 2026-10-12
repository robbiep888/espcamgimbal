"""The desktop's copy of gimbal_config.h must match the header. No board."""

from __future__ import annotations

import pathlib
import re
import unittest

import gimbal_config

HEADER = pathlib.Path(__file__).resolve().parent.parent / "main" / "include" / "gimbal_config.h"


def header_value(name: str) -> float:
    match = re.search(rf"^#define {name} ([-0-9.]+)f?\b", HEADER.read_text(), re.MULTILINE)
    if match is None:
        raise AssertionError(f"{name} not found in {HEADER}")
    return float(match.group(1))


class MirrorTest(unittest.TestCase):
    def test_every_mirrored_value_matches_the_header(self) -> None:
        pairs = {
            "GIMBAL_STEPPER_FULL_STEPS_PER_REV": gimbal_config.FULL_STEPS_PER_REV,
            "GIMBAL_STEPPER_MICROSTEPS": gimbal_config.MICROSTEPS,
            "GIMBAL_PAN_GEAR_RATIO": gimbal_config.PAN_GEAR_RATIO,
            "GIMBAL_PAN_GAIN_STEPS_PER_S": gimbal_config.PAN_GAIN_STEPS_PER_S,
            "GIMBAL_ERROR_DEADBAND": gimbal_config.ERROR_DEADBAND,
            "GIMBAL_TILT_CENTER_DEGREES": gimbal_config.TILT_CENTER_DEGREES,
            "GIMBAL_TILT_INVERT": float(gimbal_config.TILT_INVERT),
            "GIMBAL_CAMERA_EXPOSURE_DELAY_MS": gimbal_config.CAMERA_EXPOSURE_DELAY_MS,
            "GIMBAL_TILT_SERVO_LAG_MS": gimbal_config.TILT_SERVO_LAG_MS,
        }
        for name, mirrored in pairs.items():
            with self.subTest(name=name):
                self.assertAlmostEqual(header_value(name), float(mirrored), places=6)

    def test_step_rate_round_trips_through_the_firmware_formula(self) -> None:
        """gimbal_control.c: rate = gain * (|e| - deadband) / (1 - deadband)."""
        error = gimbal_config.error_for_step_rate(150.0)
        rate = (gimbal_config.PAN_GAIN_STEPS_PER_S * (error - gimbal_config.ERROR_DEADBAND)
                / (1.0 - gimbal_config.ERROR_DEADBAND))
        self.assertAlmostEqual(rate, 150.0, places=6)

    def test_home_is_zero_on_both_axes(self) -> None:
        self.assertEqual(gimbal_config.pan_degrees(0), 0.0)
        self.assertEqual(gimbal_config.tilt_degrees_from_center(gimbal_config.TILT_CENTER_DEGREES), 0.0)


if __name__ == "__main__":
    unittest.main()
