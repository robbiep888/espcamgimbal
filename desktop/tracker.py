#!/usr/bin/env python3
"""Watch the ESP32-S3 gimbal stream and drive it to follow your hand.

Show your hand and press r: the gimbal locks onto that hand and follows it in
any pose, ignoring every other hand. Hold a stop sign (palm open, fingers up,
thumb out) for three seconds and it lets go; press r to lock on again.

Three things run at once:

  * a network thread that drains TCP so decoding can never build a backlog,
  * the main thread, which decodes, finds the hand, draws and displays
    (macOS requires all GUI calls on the main thread),
  * a control thread that sends the hand's position to the board at a steady
    100 Hz, whatever the video is doing.

Splitting the control channel off matters: the board's failsafe stops the axes
if commands stop arriving, so the command rate must not sag just because a
frame took a long time to decode.
"""

from __future__ import annotations

import argparse
from collections import deque
import queue
import socket
import threading
import time

import cv2
import numpy as np

from camera_model import CameraModel
from camera_pose import BoardClock, Pose, PoseHistory
import control_protocol as ctrl
import gimbal_config
from hand_lock import STOP_HOLD_S
from hand_tracker import HandReading, HandTracker, draw_hand
from protocol import CameraFrame, newest_jpeg_offset, receive_frame


STREAM_PORT = 5000
CONTROL_PORT = 5001
# Matched to the board's 100 Hz control loop. Every reply also reports where
# the axes point, which the hand lock uses to take the gimbal's own motion out
# of where hands appear in the picture.
COMMAND_HZ = 100
# The pan axis is rate-controlled: an error is a speed, not a position. Re-sending
# the last reading while the video stalls keeps the gimbal turning toward where
# the hand WAS, and the board's own 400 ms watchdog never fires because commands
# are still arriving. A one-second Wi-Fi stall became a wild overshoot, and the
# correction after it another. Past this age a reading no longer grants motion.
READING_MAX_AGE_S = 0.25
WINDOW_TITLE = "ESP32-S3 gimbal - r locks onto a hand, hold a stop sign 3 s to let go"


def connect_ipv4(host: str, port: int, timeout: float) -> socket.socket:
    """Open a TCP connection, resolving IPv4 only.

    socket.create_connection() looks up both A and AAAA records. The board's
    mDNS responder answers only A, so "espgimbal.local" leaves the AAAA query
    unanswered and the resolver waits out its own timeout -- about five seconds
    on macOS. That is longer than the connect timeout, so the connection fails
    and looks exactly like an unreachable board. Asking for IPv4 explicitly
    skips the wait, and keeps the timeout meaning what it says: time to connect,
    not time to connect plus time to resolve.
    """
    addresses = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_STREAM)
    if not addresses:
        raise ConnectionError(f"could not resolve {host}")

    family, kind, protocol, _canonical_name, address = addresses[0]
    connection = socket.socket(family, kind, protocol)
    try:
        connection.settimeout(timeout)
        connection.connect(address)
    except BaseException:
        connection.close()
        raise
    return connection


class LatestFrame:
    """A one-frame mailbox: putting a new frame replaces an unviewed old one."""

    def __init__(self) -> None:
        self._queue: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)
        self.replaced_frames = 0

    def put(self, frame: CameraFrame) -> None:
        try:
            self._queue.put_nowait(frame)
            return
        except queue.Full:
            pass

        try:
            self._queue.get_nowait()
            self.replaced_frames += 1
        except queue.Empty:
            pass
        self._queue.put_nowait(frame)

    def get(self, timeout: float) -> CameraFrame:
        return self._queue.get(timeout=timeout)


class StreamReader(threading.Thread):
    """Continuously drains TCP so decoding/display can never cause a backlog."""

    def __init__(self, host: str, port: int, mailbox: LatestFrame) -> None:
        super().__init__(name="camera-network-reader", daemon=True)
        self.host = host
        self.port = port
        self.mailbox = mailbox
        self.stop_requested = threading.Event()
        self.status = "starting"
        self.network_fps = 0.0
        self.megabits_per_second = 0.0
        # Fed by every frame, including ones the mailbox later drops, so it
        # sees the quickest deliveries.
        self.clock = BoardClock()

    def run(self) -> None:
        while not self.stop_requested.is_set():
            try:
                self._read_one_connection()
            except (ConnectionError, OSError, ValueError) as error:
                self.status = f"reconnecting: {error}"
                self.stop_requested.wait(1.0)

    def _read_one_connection(self) -> None:
        self.status = f"connecting to {self.host}:{self.port}"
        with connect_ipv4(self.host, self.port, 3.0) as connection:
            connection.settimeout(3.0)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # A smaller receive queue prevents several old JPEGs from hiding in
            # the operating system after a brief Wi-Fi slowdown.
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024)
            self.status = "connected"

            sample_started = time.monotonic()
            sample_frames = 0
            sample_bytes = 0

            while not self.stop_requested.is_set():
                frame = receive_frame(connection)
                self.clock.observe(frame.captured_at_us, frame.received_at)
                self.mailbox.put(frame)
                sample_frames += 1
                sample_bytes += len(frame.jpeg)

                now = time.monotonic()
                elapsed = now - sample_started
                if elapsed >= 1.0:
                    self.network_fps = sample_frames / elapsed
                    self.megabits_per_second = sample_bytes * 8 / elapsed / 1_000_000
                    sample_started = now
                    sample_frames = 0
                    sample_bytes = 0

    def stop(self) -> None:
        self.stop_requested.set()


class GimbalLink(threading.Thread):
    """Owns the control socket and sends at a fixed rate, come what may."""

    def __init__(self, host: str, port: int, camera: CameraModel | None = None) -> None:
        super().__init__(name="gimbal-control", daemon=True)
        self.host = host
        self.port = port
        self.camera = camera if camera is not None else CameraModel.load()
        self.stop_requested = threading.Event()

        self._lock = threading.Lock()
        self._reading = HandReading()
        self._reading_at = float("-inf")
        # The board's capture time for the frame the reading came from. With it
        # the board can steer predictively; without it (the test tools) the
        # reading is sent as a plain proportional speed.
        self._captured_at_us: int | None = None
        self._new_reading = threading.Event()
        self._tracking_armed = True
        self._motors_off = False
        self._recenter_pending = False
        # Negating the reported error is equivalent to reversing the axis, so
        # the direction can be found in seconds here and only then baked into
        # GIMBAL_PAN_INVERT / GIMBAL_TILT_INVERT in the firmware.
        self._pan_sign = 1.0
        self._tilt_sign = 1.0
        # Manual jog. A keypress is a single event -- there is no key-release -
        # so a jog stays live for JOG_HOLD seconds and the OS key-repeat renews
        # it. Hold an arrow and the axis runs; let go and it stops by itself.
        self._jog: tuple[float, float] | None = None
        self._jog_expires = 0.0

        self.status = "starting"
        self.last_status: ctrl.GimbalStatus | None = None
        self.command_fps = 0.0
        # Where the board says the axes point, against this computer's clock.
        self.poses = PoseHistory()

    # ----- called from the main thread -------------------------------------

    def publish(self, reading: HandReading, captured_at_us: int | None = None) -> None:
        with self._lock:
            if (not reading.visible and reading.locked
                    and self._reading.visible and self._reading.following):
                # The locked hand was missed in this frame -- motion blur, an
                # awkward pose -- but it is still locked. Keep the last sighting
                # until it goes stale instead of withdrawing permission to move
                # at once: the board takes that as the hand being gone, forgets
                # how fast it was moving and brakes, so every missed frame used
                # to be a stutter. Repeating a measurement is safe because the
                # board uses each capture time once, and it stops following by
                # itself when no new frame arrives.
                return
            self._reading = reading
            self._reading_at = time.monotonic()
            self._captured_at_us = captured_at_us
        self._new_reading.set()

    def toggle_tracking(self) -> bool:
        with self._lock:
            self._tracking_armed = not self._tracking_armed
            return self._tracking_armed

    def toggle_motors(self) -> bool:
        with self._lock:
            self._motors_off = not self._motors_off
            return not self._motors_off

    def request_recenter(self) -> None:
        with self._lock:
            self._recenter_pending = True

    def jog(self, pan: float, tilt: float, hold: float = 0.35) -> None:
        with self._lock:
            self._jog = (pan, tilt)
            self._jog_expires = time.monotonic() + hold

    def flip_pan(self) -> float:
        with self._lock:
            self._pan_sign = -self._pan_sign
            return self._pan_sign

    def flip_tilt(self) -> float:
        with self._lock:
            self._tilt_sign = -self._tilt_sign
            return self._tilt_sign

    def camera_pose(self, exposed_at: float) -> Pose | None:
        """Where the camera pointed at `exposed_at` on this computer's clock,
        with the p and t overrides applied the same way they are applied to
        measurements. None until the board has replied at least once.

        The servo trails the angle it was sent, so tilt is looked up that much
        earlier, as the board does."""
        pan = self.poses.at(exposed_at)
        tilt = self.poses.at(exposed_at - gimbal_config.TILT_SERVO_LAG_MS / 1000.0)
        if pan is None or tilt is None:
            return None
        with self._lock:
            pan_sign, tilt_sign = self._pan_sign, self._tilt_sign
        return Pose(pan_sign * pan.pan_degrees, tilt_sign * tilt.tilt_degrees)

    @property
    def signs(self) -> tuple[float, float]:
        with self._lock:
            return self._pan_sign, self._tilt_sign

    @property
    def tracking_armed(self) -> bool:
        with self._lock:
            return self._tracking_armed

    @property
    def motors_off(self) -> bool:
        with self._lock:
            return self._motors_off

    # ----- control thread ---------------------------------------------------

    def _next_command(self, sequence: int) -> bytes:
        with self._lock:
            reading = self._reading
            fresh = time.monotonic() - self._reading_at <= READING_MAX_AGE_S
            captured_at_us = self._captured_at_us
            armed = self._tracking_armed
            motors_off = self._motors_off
            recenter = self._recenter_pending
            pan_sign, tilt_sign = self._pan_sign, self._tilt_sign
            jog = self._jog if time.monotonic() < self._jog_expires else None
            # A recentre is a one-shot: clear it as soon as it is put on the
            # wire so a dropped reply cannot make the gimbal home twice.
            self._recenter_pending = False

        flags = 0
        if recenter:
            flags |= ctrl.FLAG_RECENTER
        if motors_off:
            flags |= ctrl.FLAG_MOTORS_OFF

        if jog is not None:
            # Manual jog outranks the hand and works while tracking is
            # disarmed, because it is an explicit instruction rather than an
            # inference. It still respects the motors-off switch.
            if not motors_off:
                flags |= ctrl.FLAG_TRACKING
            return ctrl.encode_command(
                sequence, pan_sign * jog[0], tilt_sign * jog[1], flags
            )

        if reading.visible and fresh:
            flags |= ctrl.FLAG_HAND_VISIBLE
        # The locked hand, seen recently, is followed. This is the only thing
        # that grants motion; when the video stalls the board ramps the axes down.
        if reading.visible and reading.following and fresh and armed and not motors_off:
            flags |= ctrl.FLAG_TRACKING

        if reading.visible and fresh and captured_at_us is not None:
            pan, tilt = self.camera.angles(reading.center_x, reading.center_y)
            return ctrl.encode_measurement(
                sequence, pan_sign * pan, tilt_sign * tilt, flags, captured_at_us
            )
        return ctrl.encode_command(
            sequence,
            pan_sign * reading.pan_error,
            tilt_sign * reading.tilt_error,
            flags,
        )

    def run(self) -> None:
        sequence = 0
        while not self.stop_requested.is_set():
            try:
                sequence = self._run_one_connection(sequence)
            except (ConnectionError, OSError, ValueError) as error:
                self.status = f"reconnecting: {error}"
                self.last_status = None
                # The board may have rebooted and re-homed; old angles would
                # no longer line up with new ones.
                self.poses.clear()
                self.stop_requested.wait(1.0)

    def _run_one_connection(self, sequence: int) -> int:
        self.status = f"connecting to {self.host}:{self.port}"
        with connect_ipv4(self.host, self.port, 3.0) as connection:
            connection.settimeout(3.0)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.status = "connected"

            period = 1.0 / COMMAND_HZ
            next_send = time.monotonic()
            sample_started = next_send
            sample_commands = 0

            while not self.stop_requested.is_set():
                # Cleared before the reading is read, so one published while
                # this command is being built still wakes the next wait.
                self._new_reading.clear()
                connection.sendall(self._next_command(sequence))
                sequence += 1
                status = ctrl.receive_status(connection)
                self.poses.record(time.monotonic(),
                                  gimbal_config.pan_degrees(status.pan_steps),
                                  gimbal_config.tilt_degrees_from_center(status.tilt_degrees))
                self.last_status = status

                sample_commands += 1
                now = time.monotonic()
                if now - sample_started >= 1.0:
                    self.command_fps = sample_commands / (now - sample_started)
                    sample_started = now
                    sample_commands = 0

                # Absolute schedule, so a slow reply shortens the next wait
                # instead of letting the command rate drift downward. A new
                # frame's reading cuts the wait short: the board should hear
                # about a frame when it is processed, not up to a tick later.
                next_send += period
                sleep_for = next_send - time.monotonic()
                if sleep_for > 0:
                    if self._new_reading.wait(sleep_for):
                        next_send = time.monotonic()
                else:
                    next_send = time.monotonic()
        return sequence

    def stop(self) -> None:
        self.stop_requested.set()


def exposure_pose(frame: CameraFrame, clock: BoardClock, link: GimbalLink) -> Pose | None:
    """Where the camera pointed while this frame was being exposed, or None if
    that cannot be worked out yet."""
    captured_at = clock.to_desktop(frame.captured_at_us)
    if captured_at is None:
        return None
    return link.camera_pose(captured_at - gimbal_config.CAMERA_EXPOSURE_DELAY_MS / 1000.0)


def draw_status(image: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]) -> None:
    """Draw readable stats without pulling in a GUI toolkit."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    for row, (line, color) in enumerate(lines):
        origin = (12, 25 + row * 23)
        cv2.putText(image, line, origin, font, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, line, origin, font, 0.55, color, 1, cv2.LINE_AA)


def draw_working_area(
    image: np.ndarray, roi: tuple[float, float, float, float], deadband: float
) -> None:
    """Outline the working area, its centre, and the deadband inside it.

    When the box is smaller than the frame, everything outside it is dimmed:
    on a lens that vignettes hard it is the part of the picture the tracker
    cannot use, and seeing it makes the region adjustable by eye instead of by
    measurement. At the full-frame default nothing is dimmed.
    """
    height, width = image.shape[:2]
    left, top = int(roi[0] * width), int(roi[1] * height)
    right, bottom = int(roi[2] * width), int(roi[3] * height)

    if (left, top, right, bottom) != (0, 0, width, height):
        shaded = image.copy()
        cv2.rectangle(shaded, (0, 0), (width, height), (0, 0, 0), -1)
        cv2.rectangle(shaded, (left, top), (right, bottom), (255, 255, 255), -1)
        outside = shaded[:, :, 0] == 0
        image[outside] = (image[outside] * 0.45).astype(image.dtype)
        cv2.rectangle(image, (left, top), (right, bottom), (150, 150, 150), 1)

    center = ((left + right) // 2, (top + bottom) // 2)
    cv2.drawMarker(image, center, (200, 200, 200), cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)
    half_width = (right - left) / 2.0
    half_height = (bottom - top) / 2.0
    cv2.rectangle(
        image,
        (int(center[0] - deadband * half_width), int(center[1] - deadband * half_height)),
        (int(center[0] + deadband * half_width), int(center[1] + deadband * half_height)),
        (120, 120, 120),
        1,
    )


def overlay_lines(
    frame: CameraFrame,
    reading: HandReading,
    link: GimbalLink | None,
    mailbox: LatestFrame,
    reader: StreamReader,
    display_fps: float,
    merged_frames: int,
    tracker: HandTracker | None = None,
) -> list[tuple[str, tuple[int, int, int]]]:
    white = (235, 235, 235)
    green = (80, 255, 80)
    amber = (60, 190, 255)
    red = (80, 80, 255)

    lines = [
        (
            f"{frame.width}x{frame.height}  recv {reader.network_fps:.1f} fps  "
            f"show {display_fps:.1f} fps  {reader.megabits_per_second:.2f} Mbit/s",
            white,
        ),
        (
            f"dropped: camera {frame.dropped_on_camera}  viewer {mailbox.replaced_frames}"
            f"  |  merged by camera {merged_frames}",
            white,
        ),
    ]

    if link is None:
        lines.append(("tracking disabled (--no-track)", white))
        return lines

    others = len(reading.others)
    plural = "" if others == 1 else "s"
    if not reading.locked:
        seen = f"{others} hand{plural} in view" if others else "no hands in view"
        lines.append((f"NOT FOLLOWING - {seen}; press r to lock onto one", amber))
    else:
        ignored = f"  ({others} other hand{plural} ignored)" if others else ""
        if not reading.visible:
            lines.append(("locked hand out of sight - holding; r picks again" + ignored, amber))
        elif reading.stop_progress > 0.0:
            held = reading.stop_progress * STOP_HOLD_S
            lines.append((f"STOP SIGN {held:.1f} / {STOP_HOLD_S:.0f} s" + ignored, amber))
        else:
            lines.append(("FOLLOWING locked hand" + ignored, green))
    if tracker is not None:
        lines.append((tracker.detection_summary(), white))

    if not link.tracking_armed:
        lines.append(("TRACKING DISARMED - press space to arm", red))
    if link.motors_off:
        lines.append(("MOTORS OFF - press m to enable", red))

    status = link.last_status
    if status is None:
        lines.append((f"gimbal: {link.status}", red))
    else:
        lines.append(
            (
                f"gimbal: {status.summary()}  pan {status.pan_steps:+d} steps "
                f"@ {status.pan_rate:+d}/s  tilt {status.tilt_degrees:.1f} deg  "
                f"vision {status.vision_latency_ms} ms old",
                green if status.describes(ctrl.STATE_TRACKING) else white,
            )
        )
        lines.append((f"commands {link.command_fps:.0f}/s", white))

    pan_sign, tilt_sign = link.signs
    if pan_sign < 0 or tilt_sign < 0:
        lines.append(
            (
                f"direction overridden: pan {'REVERSED' if pan_sign < 0 else 'normal'}, "
                f"tilt {'REVERSED' if tilt_sign < 0 else 'normal'}",
                amber,
            )
        )
    lines.append(("arrows jog manually", white))
    lines.append(
        ("r lock/release hand  space arm  m motors  c recentre  p/t flip  [ ] area  q quit",
         white)
    )
    return lines


def run(
    host: str,
    stream_port: int,
    control_port: int,
    track: bool,
    deadband: float,
    roi: tuple[float, float],
) -> None:
    mailbox = LatestFrame()
    reader = StreamReader(host, stream_port, mailbox)
    reader.start()

    link: GimbalLink | None = None
    tracker: HandTracker | None = None
    if track:
        link = GimbalLink(host, control_port)
        # No smoothing here: the board filters the hand's position after
        # taking its own motion out, where smoothing costs no lag. Smoothing
        # the raw picture would also smooth the gimbal's movement, which the
        # board then subtracts at full strength.
        tracker = HandTracker(roi_width=roi[0], roi_height=roi[1], smoothing=1.0,
                              camera=link.camera)
        horizontal, vertical = link.camera.field_of_view
        print(f"camera model: {link.camera.source}, "
              f"{horizontal:.1f} x {vertical:.1f} deg field of view")
        link.start()

    display_times: deque[float] = deque(maxlen=60)
    merged_frames = 0
    # Home until the board reports otherwise, and the last known angle through
    # any gap in its reports: stale is far nearer the truth than home.
    pose = Pose()
    last_status_print = 0.0
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    try:
        while True:
            try:
                frame = mailbox.get(timeout=0.25)
            except queue.Empty:
                now = time.monotonic()
                if now - last_status_print >= 1.0:
                    print(reader.status)
                    last_status_print = now
                if handle_keys(link, tracker) is False:
                    break
                continue

            start = newest_jpeg_offset(frame.jpeg)
            if start:
                merged_frames += 1
            image = cv2.imdecode(
                np.frombuffer(frame.jpeg, dtype=np.uint8, offset=start), cv2.IMREAD_COLOR
            )
            if image is None and start:
                # The newest frame was damaged; the older ones still beat nothing.
                image = cv2.imdecode(
                    np.frombuffer(frame.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
            if image is None:
                print(f"Could not decode frame {frame.sequence}")
                continue

            reading = HandReading()
            if tracker is not None and link is not None:
                pose = exposure_pose(frame, reader.clock, link) or pose
                reading = tracker.process(image, pose=pose)
                if reading.stopped:
                    print("stop sign held: let go of the hand, gimbal holding "
                          "(press r to lock on again)")
                link.publish(reading, frame.captured_at_us)

            now = time.monotonic()
            display_times.append(now)
            display_fps = 0.0
            if len(display_times) > 1:
                display_fps = (len(display_times) - 1) / (
                    display_times[-1] - display_times[0]
                )

            draw_working_area(
                image, tracker.roi if tracker is not None else (0.0, 0.0, 1.0, 1.0), deadband
            )
            draw_hand(image, reading)
            draw_status(
                image,
                overlay_lines(
                    frame, reading, link, mailbox, reader, display_fps, merged_frames, tracker
                ),
            )
            cv2.imshow(WINDOW_TITLE, image)

            if handle_keys(link, tracker) is False:
                break
    finally:
        reader.stop()
        if link is not None:
            link.stop()
        if tracker is not None:
            tracker.close()
        cv2.destroyAllWindows()


# Extended key codes differ per platform/backend, so accept every common set.
ARROW_LEFT = {63234, 65361, 2424832, 81}
ARROW_RIGHT = {63235, 65363, 2555904, 83}
ARROW_UP = {63232, 65362, 2490368, 82}
ARROW_DOWN = {63233, 65364, 2621440, 84}


def handle_keys(link: GimbalLink | None, tracker: "HandTracker | None" = None,
                jog_amount: float = 0.55) -> bool:
    """Returns False when the user asked to quit."""
    raw = cv2.waitKeyEx(1)
    key = raw & 0xFF if raw != -1 else 255
    if key in (ord("q"), 27):
        return False

    if link is not None and raw != -1:
        pan = tilt = 0.0
        if raw in ARROW_LEFT:
            pan = -jog_amount
        elif raw in ARROW_RIGHT:
            pan = +jog_amount
        elif raw in ARROW_UP:
            tilt = -jog_amount
        elif raw in ARROW_DOWN:
            tilt = +jog_amount
        if pan or tilt:
            link.jog(pan, tilt)
            return True

    if tracker is not None and key in (ord("["), ord("]"), ord("{"), ord("}")):
        width, height = tracker.roi_size
        step = 0.02
        if key == ord("["):
            tracker.set_roi(width - step, height - step)
        elif key == ord("]"):
            tracker.set_roi(width + step, height + step)
        elif key == ord("{"):
            tracker.set_roi(width - step, height)
        else:
            tracker.set_roi(width + step, height)
        print("working area {:.2f} x {:.2f}".format(*tracker.roi_size))

    if tracker is not None and key == ord("r"):
        print(tracker.toggle_lock())

    if link is None:
        return True
    if key == ord(" "):
        print("tracking armed" if link.toggle_tracking() else "tracking disarmed")
    elif key == ord("m"):
        print("motors enabled" if link.toggle_motors() else "motors off")
    elif key == ord("c"):
        link.request_recenter()
        print("recentring")
    elif key == ord("p"):
        sign = link.flip_pan()
        print(f"pan direction {'NORMAL' if sign > 0 else 'REVERSED'} "
              f"-> GIMBAL_PAN_INVERT {'0' if sign > 0 else '1'} (with the flashed value)")
    elif key == ord("t"):
        sign = link.flip_tilt()
        print(f"tilt direction {'NORMAL' if sign > 0 else 'REVERSED'} "
              f"-> GIMBAL_TILT_INVERT {'0' if sign > 0 else '1'} (with the flashed value)")
    return True


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="View the ESP32-S3 gimbal stream and drive hand tracking"
    )
    parser.add_argument("host", help="board IP printed by idf.py monitor")
    parser.add_argument("--stream-port", type=int, default=STREAM_PORT)
    parser.add_argument("--control-port", type=int, default=CONTROL_PORT)
    parser.add_argument(
        "--no-track",
        action="store_true",
        help="watch the stream only; never connect to the control port",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=gimbal_config.ERROR_DEADBAND,
        help="drawn deadband (default: GIMBAL_ERROR_DEADBAND from gimbal_config.h)",
    )
    parser.add_argument(
        "--roi",
        default="1.0,1.0",
        metavar="W,H",
        help="working area as fractions of the frame (default: the whole "
        "frame). Shrink it if a lens leaves the corners dark, e.g. 0.8,0.8. "
        "Adjust live with [ and ], or { and } for width alone.",
    )
    return parser.parse_args()


def parse_roi(text: str) -> tuple[float, float]:
    parts = text.split(",")
    if len(parts) != 2:
        raise SystemExit("--roi wants two comma-separated fractions, e.g. 0.8,0.75")
    try:
        width, height = (float(part) for part in parts)
    except ValueError:
        raise SystemExit(f"--roi values must be numbers, got {text!r}") from None
    if not (0.2 <= width <= 1.0 and 0.2 <= height <= 1.0):
        raise SystemExit("--roi fractions must be between 0.2 and 1.0")
    return width, height


if __name__ == "__main__":
    arguments = parse_arguments()
    run(
        arguments.host,
        arguments.stream_port,
        arguments.control_port,
        not arguments.no_track,
        arguments.deadband,
        parse_roi(arguments.roi),
    )
