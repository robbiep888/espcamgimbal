#!/usr/bin/env python3
"""Measure how many degrees a pixel is, and check both axes turn the right way.

    python desktop/calibrate_fov.py espgimbal.local

Needs motor power and a still, textured scene -- a bookshelf or a cluttered
desk, not a blank wall, and nobody walking through it. Each axis is turned a
few degrees at a time; the picture's shift is measured by phase correlation and
set against the angle the board reports. The result is written to
desktop/camera_calibration.json, which tracker.py loads at startup.

Why it matters: predictive tracking aims at where the hand IS, converting its
position in the picture to an angle. A focal length 15% off makes the gimbal
land 15% short or long of every move, and a reversed axis makes it run away.
"""

from __future__ import annotations

import argparse
import datetime
import math
import statistics
import sys
import time

import cv2
import numpy as np

from camera_model import CameraModel
import gimbal_config
from protocol import newest_jpeg_offset
from tracker import GimbalLink, LatestFrame, StreamReader

# Relative moves per axis, in degrees; they sum to zero so the axis ends where
# it started. Small enough that most of the picture still overlaps.
MOVES = (5.0, -10.0, 10.0, -5.0)
MIN_RESPONSE = 0.08
SETTLE_S = 0.9


def prepare(image: np.ndarray) -> np.ndarray:
    """Central half of the frame, grey, with slow shading removed.

    The lens vignettes hard, and the vignette is fixed to the sensor: left in,
    it correlates perfectly with itself and reports no shift at all however far
    the camera turned. Keeping the middle and subtracting a heavy blur leaves
    only the scene's own detail.
    """
    height, width = image.shape[:2]
    middle = image[height // 4: height * 3 // 4, width // 4: width * 3 // 4]
    grey = cv2.cvtColor(middle, cv2.COLOR_BGR2GRAY).astype(np.float32) if middle.ndim == 3 \
        else middle.astype(np.float32)
    return grey - cv2.GaussianBlur(grey, (0, 0), sigmaX=8)


def measure_shift(before: np.ndarray, after: np.ndarray) -> tuple[float, float, float]:
    """How far the scene moved from `before` to `after`, in pixels, and how
    confident the match is (phase correlation peak, 0..1)."""
    a, b = prepare(before), prepare(after)
    window = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(a, b, window)
    return dx, dy, response


def fit_focal(samples: list[tuple[float, float]]) -> tuple[float, float]:
    """Least-squares focal length (px) from (degrees turned, pixels shifted).

    Turning toward positive angles moves the scene the other way in the
    picture, so a correctly signed axis fits a POSITIVE focal length from
    shift = -focal * tan(angle). Returns (focal, rms residual in px)."""
    tangents = np.array([math.tan(math.radians(angle)) for angle, _ in samples])
    shifts = np.array([shift for _, shift in samples])
    focal = float(-np.dot(shifts, tangents) / np.dot(tangents, tangents))
    residual = float(np.sqrt(np.mean((shifts + focal * tangents) ** 2)))
    return focal, residual


class Rig:
    def __init__(self, host: str) -> None:
        self.mailbox = LatestFrame()
        self.reader = StreamReader(host, 5000, self.mailbox)
        self.link = GimbalLink(host, 5001)
        self.reader.start()
        self.link.start()
        deadline = time.monotonic() + 10
        while self.link.last_status is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.link.last_status is None:
            raise SystemExit(f"no control connection: {self.link.status} "
                             "(is the tracker still running?)")

    def frame(self) -> np.ndarray:
        """Average of a few fresh frames, to beat sensor and JPEG noise."""
        images = []
        deadline = time.monotonic() + 5
        while len(images) < 3 and time.monotonic() < deadline:
            packet = self.mailbox.get(timeout=2.0)
            start = newest_jpeg_offset(packet.jpeg)
            image = cv2.imdecode(np.frombuffer(packet.jpeg, np.uint8, offset=start),
                                 cv2.IMREAD_COLOR)
            if image is not None:
                images.append(image.astype(np.float32))
        if not images:
            raise SystemExit("no video frames arriving")
        return (sum(images) / len(images)).astype(np.uint8)

    def angle(self, axis: str) -> float:
        status = self.link.last_status
        return gimbal_config.pan_degrees(status.pan_steps) if axis == "pan" else status.tilt_degrees

    def settle(self) -> None:
        time.sleep(SETTLE_S)

    def turn(self, axis: str, degrees: float) -> None:
        """Jog until the board reports the axis has turned about `degrees`."""
        start = self.angle(axis)
        speed = math.copysign(0.18 if axis == "pan" else 0.3, degrees)
        deadline = time.monotonic() + 6
        while abs(self.angle(axis) - start) < abs(degrees) * 0.95:
            if time.monotonic() > deadline:
                raise SystemExit(f"{axis} did not move: is motor power on?")
            self.link.jog(speed if axis == "pan" else 0.0, speed if axis == "tilt" else 0.0)
            time.sleep(0.03)
        self.link.jog(0.0, 0.0, hold=0.2)
        self.settle()

    def close(self) -> None:
        self.link.request_recenter()
        time.sleep(3)
        self.link.stop()
        self.reader.stop()


def calibrate_axis(rig: Rig, axis: str) -> tuple[list[tuple[float, float]], list[float]]:
    samples, cross = [], []
    rig.settle()
    before_image, before_angle = rig.frame(), rig.angle(axis)
    for move in MOVES:
        rig.turn(axis, move)
        after_image, after_angle = rig.frame(), rig.angle(axis)
        dx, dy, response = measure_shift(before_image, after_image)
        turned = after_angle - before_angle
        along, across = (dx, dy) if axis == "pan" else (dy, dx)
        verdict = "ok" if response >= MIN_RESPONSE else "REJECTED (scene too plain or moving)"
        print(f"  {axis} turned {turned:+6.2f} deg -> picture moved {along:+7.1f} px "
              f"(sideways {across:+5.1f}), match {response:.2f} {verdict}")
        if response >= MIN_RESPONSE:
            samples.append((turned, along))
            cross.append(across)
        before_image, before_angle = after_image, after_angle
    return samples, cross


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate the camera's degrees per pixel")
    parser.add_argument("host")
    parser.add_argument("--dry-run", action="store_true", help="measure but do not save")
    arguments = parser.parse_args()

    rig = Rig(arguments.host)
    width = height = None
    results: dict[str, tuple[float, float, int]] = {}
    try:
        print("recentring ...")
        rig.link.request_recenter()
        time.sleep(4)
        probe = rig.frame()
        height, width = probe.shape[:2]
        _, _, stillness = measure_shift(probe, rig.frame())
        print(f"scene check: two still frames match at {stillness:.2f}")
        if stillness < 0.2:
            raise SystemExit("the scene is too plain or something is moving; point the "
                             "camera at a still, detailed scene and run again")

        for axis in ("pan", "tilt"):
            print(f"\n{axis}:")
            samples, cross = calibrate_axis(rig, axis)
            if len(samples) < 3:
                raise SystemExit(f"only {len(samples)} usable {axis} moves; nothing saved")
            focal, residual = fit_focal(samples)
            results[axis] = (focal, residual, len(samples))
            print(f"  focal length {abs(focal):.0f} px, fit residual {residual:.1f} px, "
                  f"sideways drift {statistics.mean(abs(c) for c in cross):.1f} px per move")
    finally:
        rig.close()

    reversed_axes = [axis for axis, (focal, _, _) in results.items() if focal < 0]
    for axis in reversed_axes:
        flag = "GIMBAL_PAN_INVERT" if axis == "pan" else "GIMBAL_TILT_INVERT"
        print(f"\n!!! {axis.upper()} IS REVERSED: turning toward a hand would move the "
              f"picture the wrong way and tracking would run away. Flip {flag} in "
              f"gimbal_config.h (or press {'p' if axis == 'pan' else 't'} in the tracker).")

    model = CameraModel(focal_x=abs(results["pan"][0]) / width,
                        focal_y=abs(results["tilt"][0]) / height)
    horizontal, vertical = model.field_of_view
    print(f"\nfield of view {horizontal:.1f} x {vertical:.1f} deg "
          f"(was {CameraModel.load().field_of_view[0]:.1f} x {CameraModel.load().field_of_view[1]:.1f})")
    if arguments.dry_run or reversed_axes:
        print("not saved" + (" (fix the axis direction first)" if reversed_axes else ""))
        sys.exit(1 if reversed_axes else 0)
    model.save(
        measured=datetime.datetime.now().isoformat(timespec="seconds"),
        frame=[width, height],
        residual_px={axis: round(r[1], 2) for axis, r in results.items()},
        moves={axis: r[2] for axis, r in results.items()},
    )
    print("saved to desktop/camera_calibration.json; restart the tracker to use it")


if __name__ == "__main__":
    main()
