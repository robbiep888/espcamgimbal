#!/usr/bin/env python3
"""Drive the gimbal by hand from the terminal. No camera, no hand detection.

    python desktop/jog.py espgimbal.local

    arrows      pan / tilt for as long as the key is held
    , .         slower / faster
    p / t       reverse pan / tilt direction, live
    c           recentre both axes
    m           motors off / on
    q or Ctrl-C quit

Useful when an axis is misbehaving and the vision pipeline would only add
variables. Releasing the keys stops the gimbal on its own: a jog expires shortly
after the last keypress, and the board's own failsafe halts the axes 400 ms
after commands stop arriving.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

import gimbal_config
from hand_tracker import HandReading
from tracker import GimbalLink

SPEEDS = [0.20, 0.35, 0.55, 0.80, 1.00]


_pending = ""


def read_keys(timeout: float) -> list[str]:
    """Return any keys waiting on stdin, decoding arrow escape sequences.

    Reads with os.read rather than sys.stdin.read. sys.stdin is buffered: it
    pulls the whole three-byte arrow sequence (ESC [ C) into Python's own
    buffer, where select() -- which watches the raw file descriptor -- cannot
    see it. The loop then exits holding a bare ESC, the sequence never
    reassembles, and every arrow press is silently dropped.

    Anything left mid-sequence is carried over, because a keystroke can arrive
    split across two reads.
    """
    global _pending
    if select.select([sys.stdin], [], [], timeout)[0]:
        _pending += os.read(sys.stdin.fileno(), 1024).decode("utf-8", "replace")

    arrows = {"A": "up", "B": "down", "C": "right", "D": "left"}
    keys: list[str] = []
    while _pending:
        if _pending[0] == "\x1b":
            if len(_pending) < 3:
                break                       # rest of the sequence not here yet
            if _pending[1] == "[":
                keys.append(arrows.get(_pending[2], ""))
                _pending = _pending[3:]
            else:
                _pending = _pending[1:]
            continue
        keys.append(_pending[0])
        _pending = _pending[1:]
    return [k for k in keys if k]


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual gimbal control")
    parser.add_argument("host")
    arguments = parser.parse_args()

    link = GimbalLink(arguments.host, 5001)
    link.start()
    for _ in range(40):
        if link.last_status is not None:
            break
        time.sleep(0.25)
    if link.last_status is None:
        raise SystemExit(f"no control connection: {link.status}")

    speed_index = 2
    settings = termios.tcgetattr(sys.stdin)
    print("arrows jog   , . speed   p/t reverse   c recentre   m motors   q quit\n")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            pan = tilt = 0.0
            for key in read_keys(0.05):
                if key in ("q", "\x03"):
                    return
                if key == "left":
                    pan = -SPEEDS[speed_index]
                elif key == "right":
                    pan = +SPEEDS[speed_index]
                elif key == "up":
                    tilt = -SPEEDS[speed_index]
                elif key == "down":
                    tilt = +SPEEDS[speed_index]
                elif key == ",":
                    speed_index = max(0, speed_index - 1)
                elif key == ".":
                    speed_index = min(len(SPEEDS) - 1, speed_index + 1)
                elif key == "c":
                    link.request_recenter()
                elif key == "m":
                    link.toggle_motors()
                elif key == "p":
                    sign = link.flip_pan()
                    print(f"\npan {'NORMAL' if sign > 0 else 'REVERSED'} -> set "
                          f"GIMBAL_PAN_INVERT to "
                          f"{'0' if sign > 0 else '1'} relative to the flashed value")
                elif key == "t":
                    sign = link.flip_tilt()
                    print(f"\ntilt {'NORMAL' if sign > 0 else 'REVERSED'} -> set "
                          f"GIMBAL_TILT_INVERT to "
                          f"{'0' if sign > 0 else '1'} relative to the flashed value")

            if pan or tilt:
                link.jog(pan, tilt)

            status = link.last_status
            if status is not None:
                sys.stdout.write(
                    f"\rpan {status.pan_steps:+5d} steps "
                    f"({gimbal_config.pan_degrees(status.pan_steps):+6.1f} deg) "
                    f"rate {status.pan_rate:+5d}   "
                    f"tilt {status.tilt_degrees:5.1f} deg   "
                    f"speed {SPEEDS[speed_index]:.2f}   "
                    f"[{status.summary()}]          "
                )
                sys.stdout.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        link.publish(HandReading(visible=False))
        time.sleep(0.4)
        link.stop()
        print("\nstopped")


if __name__ == "__main__":
    main()
