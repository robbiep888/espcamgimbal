#!/usr/bin/env python3
"""Decide whether each axis physically moves, without anyone watching it.

The camera rides on both axes, so if an axis moves the picture changes. Earlier
attempts compared a frame before a move with one after, which failed on this
build: a large move leaves no overlapping scene, and the lens vignette is a
fixed pattern on the sensor that correlation locks onto instead.

This measures something far harder to fool. It samples frame-to-frame change
while the axis is idle to establish a noise floor, then samples the same
quantity while the axis is commanded to move. A camera that is moving makes
consecutive frames differ far more than sensor noise does. No scene texture,
overlap or correlation is required -- only that something in view changes.

    python desktop/axis_selftest.py espgimbal.local
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

import cv2
import numpy as np

import gimbal_config
from hand_tracker import HandReading
from protocol import newest_jpeg_offset
from tracker import GimbalLink, LatestFrame, StreamReader

ROI = (0.1, 0.1, 0.9, 0.9)

# A moving camera must beat the idle noise floor by this much to count. Chosen
# well above the JPEG-compression flicker seen on a static scene.
MOTION_RATIO = 3.0


def central(image: np.ndarray) -> np.ndarray:
    """The bright middle of the frame; the vignetted edges carry no signal."""
    height, width = image.shape[:2]
    view = image[int(height * 0.25):int(height * 0.75),
                 int(width * 0.25):int(width * 0.75)]
    return cv2.cvtColor(view, cv2.COLOR_BGR2GRAY).astype(np.float32)


class Sampler:
    def __init__(self, mailbox: LatestFrame) -> None:
        self.mailbox = mailbox

    def frame(self) -> np.ndarray | None:
        try:
            packet = self.mailbox.get(timeout=2.0)
        except Exception:
            return None
        start = newest_jpeg_offset(packet.jpeg)
        image = cv2.imdecode(np.frombuffer(packet.jpeg, np.uint8, offset=start),
                             cv2.IMREAD_COLOR)
        return None if image is None else central(image)

    def change_rate(self, seconds: float, drive=None) -> float:
        """How much consecutive frames differ, as a high percentile.

        A percentile rather than a median because an axis does not move for the
        whole window: tilt reaches its limit in under two seconds and then holds
        still, so a median over the window is dominated by stationary frames and
        reports no motion even when the axis moved perfectly well.

        `drive` is called each iteration so the caller can keep the axis moving
        (reversing it as needed) for the duration of the sample.
        """
        previous, differences = None, []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if drive is not None:
                drive()
            current = self.frame()
            if current is None:
                continue
            if previous is not None and previous.shape == current.shape:
                differences.append(float(np.mean(np.abs(current - previous))))
            previous = current
        if not differences:
            return 0.0
        differences.sort()
        return differences[int(len(differences) * 0.9) - 1] if len(differences) >= 4 \
            else max(differences)


def main() -> None:
    parser = argparse.ArgumentParser(description="Check each gimbal axis for real motion")
    parser.add_argument("host")
    parser.add_argument("--rate", type=float, default=150.0, help="pan step rate to test")
    arguments = parser.parse_args()

    mailbox = LatestFrame()
    reader = StreamReader(arguments.host, 5000, mailbox)
    reader.start()
    link = GimbalLink(arguments.host, 5001)
    link.start()

    for _ in range(40):
        if link.last_status is not None:
            break
        time.sleep(0.25)
    if link.last_status is None:
        raise SystemExit(f"no control connection: {link.status}")

    sampler = Sampler(mailbox)
    for _ in range(40):
        if sampler.frame() is not None:
            break
        time.sleep(0.25)

    def measure_floor(label: str) -> float:
        link.publish(HandReading(visible=False))
        time.sleep(0.8)
        value = sampler.change_rate(5.0)
        print(f"  idle frame-to-frame change ({label}): {value:.3f} grey levels")
        return value

    # The floor is sensor and JPEG noise, which is constant. Anything moving in
    # front of the camera only ever ADDS to it, so measuring twice and keeping
    # the quieter reading rejects a person walking through the shot. A large
    # gap between the two means the scene was not still and no verdict from
    # this run can be trusted.
    print("measuring the idle noise floor (nothing commanded) ...")
    floor_before = measure_floor("before")

    results = {}
    for axis, label in (("pan", "PAN (stepper)"), ("tilt", "TILT (servo)")):
        link.request_recenter()
        time.sleep(4.0)
        before = link.last_status

        # Reverse periodically so the axis keeps moving for the whole sample
        # instead of parking against a limit part way through.
        offset = gimbal_config.error_for_step_rate(arguments.rate) if axis == "pan" else 0.6
        started = time.monotonic()
        period = 2.0 if axis == "pan" else 1.4

        def drive() -> None:
            direction = 1.0 if int((time.monotonic() - started) / period) % 2 == 0 else -1.0
            value = direction * offset
            if axis == "pan":
                link.publish(HandReading(visible=True, following=True,
                                         center_x=0.5 + value * 0.4,
                                         center_y=0.5, roi=ROI))
            else:
                link.publish(HandReading(visible=True, following=True, center_x=0.5,
                                         center_y=0.5 + value * 0.4, roi=ROI))

        print(f"{label}: commanding motion back and forth for 8 s ...")
        moving = sampler.change_rate(8.0, drive)
        link.publish(HandReading(visible=False))
        time.sleep(1.0)
        after = link.last_status

        commanded = ("swept back and forth; ended at "
                     + (f"{after.pan_steps} steps" if axis == "pan"
                        else f"{after.tilt_degrees:.0f} deg"))

        results[axis] = moving
        print(f"  firmware commanded: {commanded}")
        print(f"  picture change while moving: {moving:.3f}\n")

    link.publish(HandReading(visible=False))
    link.request_recenter()
    time.sleep(4.0)

    print("re-measuring the idle noise floor ...")
    floor_after = measure_floor("after")
    link.stop()
    reader.stop()

    unstable = max(floor_before, floor_after) > 2.0 * min(floor_before, floor_after)
    print("=" * 62)
    if unstable:
        print("  SCENE NOT STILL: the two idle measurements disagree by more")
        print(f"  than 2x ({floor_before:.3f} vs {floor_after:.3f}). Something was")
        print("  moving in front of the camera, so these verdicts mean nothing.")
        print("  Point the camera at a static scene and run it again.")
        print("=" * 62)
    floor = min(floor_before, floor_after)
    verdict = {}
    for axis, label in (("pan", "PAN (stepper)"), ("tilt", "TILT (servo)")):
        ratio = results[axis] / floor if floor > 1e-6 else float("inf")
        verdict[axis] = ratio >= MOTION_RATIO
        print(f"  {label:<16} {results[axis]:6.3f}  = {ratio:5.1f}x floor  "
              f"{'WORKING' if verdict[axis] else 'NOT MOVING'}")
    results = verdict
    if not results["pan"]:
        print("\nPan not moving: check the driver's current-limit pot, the coil")
        print("pairing (A1/A2 must be one coil, B1/B2 the other), and that STEP")
        print("is on GPIO 1 and DIR on GPIO 2 rather than swapped.")
    if not results["tilt"]:
        print("\nTilt not moving: the boot log confirms the pulse is present on")
        print("GPIO 41, so check the ground bond between the ESP32 and the servo")
        print("supply, the signal wire itself, and whether the servo still holds")
        print("position stiffly when turned by hand.")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
