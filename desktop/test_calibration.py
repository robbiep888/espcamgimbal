"""Tests for the calibration maths in calibrate_fov.py. No board, no camera."""

from __future__ import annotations

import math
import unittest

import cv2
import numpy as np

from calibrate_fov import fit_focal, measure_shift


def textured_scene(seed: int = 3) -> np.ndarray:
    rng = np.random.default_rng(seed)
    scene = cv2.GaussianBlur(rng.uniform(0, 255, (900, 1100)).astype(np.float32), (0, 0), 3)
    for _ in range(40):
        x, y = rng.integers(0, 1000), rng.integers(0, 800)
        cv2.rectangle(scene, (int(x), int(y)),
                      (int(x + rng.integers(10, 80)), int(y + rng.integers(10, 80))),
                      float(rng.uniform(0, 255)), -1)
    return scene


def view(scene: np.ndarray, left: int, top: int) -> np.ndarray:
    """A 640 x 480 frame of the scene through a lens that vignettes hard, the
    way this camera does: the dark corners stay put while the scene moves."""
    yy, xx = np.mgrid[0:480, 0:640]
    radius = ((xx - 320) / 320) ** 2 + ((yy - 240) / 240) ** 2
    vignette = np.clip(1.25 - 0.9 * radius, 0.05, 1.0).astype(np.float32)
    frame = (scene[top:top + 480, left:left + 640] * vignette).clip(0, 255).astype(np.uint8)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


class ShiftMeasurementTest(unittest.TestCase):
    def test_finds_the_scene_shift_through_a_fixed_vignette(self) -> None:
        """The vignette matches itself perfectly at zero shift; it must not
        win over the scene, which is what fooled the old self-test."""
        scene = textured_scene()
        # Camera turns right and up, so the scene moves left and down.
        dx, dy, response = measure_shift(view(scene, 200, 150), view(scene, 237, 129))
        self.assertAlmostEqual(dx, -37.0, delta=0.5)
        self.assertAlmostEqual(dy, 21.0, delta=0.5)
        self.assertGreater(response, 0.3)

    def test_a_still_scene_reports_no_shift(self) -> None:
        scene = textured_scene()
        dx, dy, response = measure_shift(view(scene, 200, 150), view(scene, 200, 150))
        self.assertAlmostEqual(dx, 0.0, delta=0.1)
        self.assertAlmostEqual(dy, 0.0, delta=0.1)
        self.assertGreater(response, 0.9)

    def test_a_blank_wall_is_not_trusted(self) -> None:
        rng = np.random.default_rng(1)
        wall = [np.full((480, 640, 3), 128, np.uint8) + rng.integers(0, 3, (480, 640, 3), dtype=np.uint8)
                for _ in range(2)]
        _, _, response = measure_shift(wall[0], wall[1])
        self.assertLess(response, 0.08)


class FocalFitTest(unittest.TestCase):
    def samples(self, focal: float) -> list[tuple[float, float]]:
        return [(angle, -focal * math.tan(math.radians(angle)))
                for angle in (5.0, -10.0, 10.0, -5.0)]

    def test_recovers_the_focal_length(self) -> None:
        focal, residual = fit_focal(self.samples(515.0))
        self.assertAlmostEqual(focal, 515.0, places=3)
        self.assertLess(residual, 1e-6)

    def test_a_reversed_axis_fits_a_negative_focal_length(self) -> None:
        """Positive angles moving the scene the SAME way means the axis turns
        away from where it is told to go; calibrate_fov refuses to save."""
        focal, _ = fit_focal([(angle, -shift) for angle, shift in self.samples(515.0)])
        self.assertLess(focal, 0.0)


if __name__ == "__main__":
    unittest.main()
