#!/usr/bin/env python3
"""A stand-in for the gimbal, for working on the desktop side without hardware.

It speaks both protocols the firmware speaks: synthetic JPEG frames on the video
port, and the command/status exchange on the control port. The motion model is a
simplified copy of gimbal_control.c -- enough to see the pan position move and
the failsafe engage, not a simulation of the real dynamics.

    python fake_board.py
    python tracker.py 127.0.0.1
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time

import cv2
import numpy as np

import control_protocol as ctrl
from protocol import HEADER, MAGIC, VERSION

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# Degrees per unit of proportional error: half of a 0.8 working area of ~66 deg.
HALF_ROI_DEGREES = 26.4


def render_frame(sequence: int, pan_steps: int, tilt_degrees: float) -> np.ndarray:
    """A moving target plus the simulated axis readout drawn into the picture."""
    image = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 24, dtype=np.uint8)

    for x in range(0, FRAME_WIDTH, 80):
        cv2.line(image, (x, 0), (x, FRAME_HEIGHT), (44, 44, 44), 1)
    for y in range(0, FRAME_HEIGHT, 80):
        cv2.line(image, (0, y), (FRAME_WIDTH, y), (44, 44, 44), 1)

    angle = sequence / 25.0
    center = (
        int(FRAME_WIDTH / 2 + math.sin(angle) * 180),
        int(FRAME_HEIGHT / 2 + math.cos(angle * 0.7) * 110),
    )
    cv2.circle(image, center, 46, (70, 200, 255), -1, cv2.LINE_AA)
    cv2.putText(
        image,
        f"frame {sequence}  pan {pan_steps:+d}  tilt {tilt_degrees:.1f}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    return image


class SimulatedGimbal:
    """A cut-down gimbal_control.c: deadband, P gain, slew limit, failsafe."""

    DEADBAND = 0.06
    GAIN_STEPS_PER_S = 900.0
    MAX_STEP_RATE = 1200.0
    MIN_STEP_RATE = 25.0
    ACCEL = 4000.0
    COMMAND_TIMEOUT = 0.4
    TILT_GAIN_DEG_PER_S = 45.0
    TILT_MIN, TILT_MAX, TILT_CENTER = 50.0, 130.0, 90.0

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pan_steps = 0.0
        self.pan_rate = 0.0
        self.tilt_degrees = self.TILT_CENTER
        self.state_flags = 0
        self._pan_error = 0.0
        self._tilt_error = 0.0
        self._flags = 0
        self._received_at = 0.0
        self._recentering = False

    def submit(self, pan_error: float, tilt_error: float, flags: int) -> None:
        with self.lock:
            self._pan_error = pan_error
            self._tilt_error = tilt_error
            self._flags = flags
            self._received_at = time.monotonic()
            if flags & ctrl.FLAG_RECENTER:
                self._recentering = True

    @staticmethod
    def _deadband(error: float) -> float:
        magnitude = abs(error)
        if magnitude <= SimulatedGimbal.DEADBAND:
            return 0.0
        rescaled = (magnitude - SimulatedGimbal.DEADBAND) / (1 - SimulatedGimbal.DEADBAND)
        return -rescaled if error < 0 else rescaled

    def tick(self, elapsed: float) -> None:
        with self.lock:
            stale = (
                self._received_at == 0.0
                or (time.monotonic() - self._received_at) > self.COMMAND_TIMEOUT
            )
            motors_off = bool(self._flags & ctrl.FLAG_MOTORS_OFF)
            tracking = (
                not stale and not motors_off and bool(self._flags & ctrl.FLAG_TRACKING)
            )
            if stale or motors_off:
                self._recentering = False

            desired = 0.0
            if self._recentering:
                if abs(self.pan_steps) < 4:
                    desired = 0.0
                    self._recentering = False
                else:
                    # Mirrors GIMBAL_RECENTER_GAIN plus the minimum-rate floor:
                    # below MIN_STEP_RATE no pulses are emitted at all.
                    desired = -3.0 * self.pan_steps
                    if abs(desired) < self.MIN_STEP_RATE:
                        desired = math.copysign(self.MIN_STEP_RATE, desired)
                    desired = max(-self.MAX_STEP_RATE, min(self.MAX_STEP_RATE, desired))
                self.tilt_degrees += (self.TILT_CENTER - self.tilt_degrees) * min(
                    1.0, elapsed * 6.0
                )
            elif tracking:
                desired = self.GAIN_STEPS_PER_S * self._deadband(self._pan_error)
                desired = max(-self.MAX_STEP_RATE, min(self.MAX_STEP_RATE, desired))
                rate = self.TILT_GAIN_DEG_PER_S * self._deadband(self._tilt_error)
                self.tilt_degrees = max(
                    self.TILT_MIN, min(self.TILT_MAX, self.tilt_degrees + rate * elapsed)
                )

            change = max(-self.ACCEL * elapsed,
                         min(self.ACCEL * elapsed, desired - self.pan_rate))
            self.pan_rate += change
            if abs(self.pan_rate) < self.MIN_STEP_RATE:
                self.pan_rate = 0.0
            self.pan_steps += self.pan_rate * elapsed

            flags = 0
            if not motors_off and self.pan_rate != 0.0:
                flags |= ctrl.STATE_STEPPER_ENABLED
            if tracking:
                flags |= ctrl.STATE_TRACKING
            if stale and self._received_at != 0.0:
                flags |= ctrl.STATE_WATCHDOG_STOP
            if self._recentering:
                flags |= ctrl.STATE_RECENTERING
            self.state_flags = flags

    def snapshot(self) -> tuple[int, int, float, int]:
        with self.lock:
            return (
                int(round(self.pan_steps)),
                int(round(self.pan_rate)),
                self.tilt_degrees,
                self.state_flags,
            )


def serve_video(gimbal: SimulatedGimbal, port: int, fps: float, stop: threading.Event) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", port))
        server.listen(1)
        server.settimeout(0.5)
        print(f"fake board: video on port {port}")

        sequence = 0
        while not stop.is_set():
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            print(f"fake board: viewer connected from {address[0]}")
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                period = 1.0 / fps
                try:
                    while not stop.is_set():
                        started = time.monotonic()
                        pan, _rate, tilt, _flags = gimbal.snapshot()
                        ok, encoded = cv2.imencode(
                            ".jpg", render_frame(sequence, pan, tilt),
                            [int(cv2.IMWRITE_JPEG_QUALITY), 70],
                        )
                        if not ok:
                            continue
                        jpeg = encoded.tobytes()
                        header = HEADER.pack(
                            MAGIC, VERSION, HEADER.size, sequence,
                            int(time.monotonic() * 1e6), len(jpeg),
                            FRAME_WIDTH, FRAME_HEIGHT, 0,
                        )
                        connection.sendall(header + jpeg)
                        sequence += 1
                        remaining = period - (time.monotonic() - started)
                        if remaining > 0:
                            time.sleep(remaining)
                except (OSError, ConnectionError):
                    print("fake board: viewer disconnected")


def serve_control(gimbal: SimulatedGimbal, port: int, stop: threading.Event) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", port))
        server.listen(1)
        server.settimeout(0.5)
        print(f"fake board: control on port {port}")

        while not stop.is_set():
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            print(f"fake board: tracker connected from {address[0]}")
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                connection.settimeout(2.0)
                try:
                    while not stop.is_set():
                        raw = ctrl.receive_exactly(connection, ctrl.COMMAND.size)
                        (magic, version, size, sequence,
                         pan_raw, tilt_raw, flags, captured_at_ms) = ctrl.COMMAND.unpack(raw)
                        if magic != ctrl.COMMAND_MAGIC or version != ctrl.VERSION:
                            print("fake board: malformed command; closing")
                            break
                        latency_ms = 0
                        if flags & ctrl.FLAG_MEASUREMENT:
                            # Not the board's predictive controller: angles are
                            # mapped back onto the proportional model, enough to
                            # exercise the desktop without hardware.
                            pan_error = pan_raw / ctrl.CENTIDEGREES / HALF_ROI_DEGREES
                            tilt_error = tilt_raw / ctrl.CENTIDEGREES / HALF_ROI_DEGREES
                            latency_ms = (int(time.monotonic() * 1000) - captured_at_ms) & 0xFFFF
                        else:
                            pan_error = pan_raw / ctrl.Q15_ONE
                            tilt_error = tilt_raw / ctrl.Q15_ONE
                        gimbal.submit(pan_error, tilt_error, flags)
                        pan, rate, tilt, state = gimbal.snapshot()
                        connection.sendall(
                            ctrl.STATUS.pack(
                                ctrl.STATUS_MAGIC, ctrl.VERSION, ctrl.STATUS.size,
                                sequence, pan, rate, int(round(tilt * 10)), state,
                                latency_ms,
                            )
                        )
                except (OSError, ConnectionError, struct.error):
                    print("fake board: tracker disconnected")


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulate the gimbal board")
    parser.add_argument("--stream-port", type=int, default=5000)
    parser.add_argument("--control-port", type=int, default=5001)
    parser.add_argument("--fps", type=float, default=22.0)
    arguments = parser.parse_args()

    gimbal = SimulatedGimbal()
    stop = threading.Event()

    def motion_loop() -> None:
        previous = time.monotonic()
        while not stop.is_set():
            time.sleep(0.01)
            now = time.monotonic()
            gimbal.tick(min(0.05, now - previous))
            previous = now

    threads = [
        threading.Thread(target=motion_loop, daemon=True),
        threading.Thread(
            target=serve_video, args=(gimbal, arguments.stream_port, arguments.fps, stop),
            daemon=True,
        ),
        threading.Thread(
            target=serve_control, args=(gimbal, arguments.control_port, stop), daemon=True
        ),
    ]
    for thread in threads:
        thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nfake board: stopping")
        stop.set()


if __name__ == "__main__":
    main()
