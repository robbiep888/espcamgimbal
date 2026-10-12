#!/usr/bin/env python3
"""Drive one axis slowly and repeatedly so it can be watched, listened to and
felt. Use when an axis is not moving and it is not yet clear whether the cause
is electrical (the motor never turns) or mechanical (the motor turns but the
load does not follow).

    python desktop/motor_test.py espgimbal.local --axis pan
    python desktop/motor_test.py espgimbal.local --axis tilt

Ctrl-C stops it; the board's own failsafe also stops the axes within 400 ms of
the commands ceasing.
"""
from __future__ import annotations

import argparse
import time

import control_protocol as ctrl
import gimbal_config
from hand_tracker import HandReading
from tracker import GimbalLink

ROI = (0.1, 0.1, 0.9, 0.9)


def sweep_pan(link: GimbalLink, rate: float, seconds: float) -> None:
    error = gimbal_config.error_for_step_rate(rate)
    for direction, name in ((+1, "RIGHT"), (-1, "LEFT ")):
        start = link.last_status.pan_steps if link.last_status else 0
        print(f"  driving {name} at {rate:.0f} steps/s "
              f"({gimbal_config.pan_degrees(rate):.0f} deg/s) for {seconds:.0f}s ... ", end="", flush=True)
        link.publish(HandReading(visible=True, following=True,
                                 center_x=0.5 + direction * error * 0.4,
                                 center_y=0.5, roi=ROI))
        time.sleep(seconds)
        link.publish(HandReading(visible=False))
        time.sleep(0.8)
        status = link.last_status
        moved = (status.pan_steps - start) if status else 0
        print(f"firmware counted {moved:+d} steps "
              f"({gimbal_config.pan_degrees(moved):+.0f} deg), coils "
              f"{'ON' if status and status.describes(ctrl.STATE_STEPPER_ENABLED) else 'OFF'}")


def sweep_tilt(link: GimbalLink, seconds: float) -> None:
    for direction, name in ((+1, "DOWN"), (-1, "UP  ")):
        start = link.last_status.tilt_degrees if link.last_status else 0.0
        print(f"  tilting {name} for {seconds:.0f}s ... ", end="", flush=True)
        link.publish(HandReading(visible=True, following=True, center_x=0.5,
                                 center_y=0.5 + direction * 0.35, roi=ROI))
        time.sleep(seconds)
        link.publish(HandReading(visible=False))
        time.sleep(0.8)
        now = link.last_status.tilt_degrees if link.last_status else 0.0
        print(f"firmware angle {start:.0f} -> {now:.0f} deg")


def main() -> None:
    parser = argparse.ArgumentParser(description="Exercise one gimbal axis")
    parser.add_argument("host")
    parser.add_argument("--axis", choices=("pan", "tilt"), default="pan")
    parser.add_argument("--rate", type=float, default=80.0,
                        help=f"pan step rate; 80 steps/s is {gimbal_config.pan_degrees(80):.0f} deg/s")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--cycles", type=int, default=6)
    arguments = parser.parse_args()

    link = GimbalLink(arguments.host, ctrl_port := 5001)
    link.start()
    for _ in range(40):
        if link.last_status is not None:
            break
        time.sleep(0.25)
    if link.last_status is None:
        raise SystemExit(f"could not reach the gimbal on {arguments.host}:{ctrl_port} "
                         f"({link.status}). Is the tracker still running?")

    print(f"connected to {arguments.host}\n")
    print("WATCH THE MOTOR while this runs:")
    print("  shaft turns, camera does not  -> coupler or grub screw")
    print("  shaft buzzes or hums, no turn -> coil wiring or driver current")
    print("  shaft silent and free to turn -> no power, or ENABLE never asserts")
    print("  shaft locked but will not turn -> STEP pulses not arriving\n")

    try:
        for cycle in range(arguments.cycles):
            print(f"cycle {cycle + 1}/{arguments.cycles}")
            if arguments.axis == "pan":
                sweep_pan(link, arguments.rate, arguments.seconds)
            else:
                sweep_tilt(link, arguments.seconds)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        link.publish(HandReading(visible=False))
        time.sleep(0.5)
        link.stop()


if __name__ == "__main__":
    main()
