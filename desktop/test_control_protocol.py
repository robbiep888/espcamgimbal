"""Wire-format and control-link tests that need no board and no camera.

Hand gestures and the hand lock are tested in test_hand_lock.py.
"""

from __future__ import annotations

import socket
import threading
import unittest
from unittest import mock

from camera_model import CameraModel
import control_protocol as ctrl
from hand_tracker import HandReading


class CommandEncodingTest(unittest.TestCase):
    def test_matches_the_firmware_struct_size(self) -> None:
        # main/include/control_protocol.h static-asserts these same numbers.
        self.assertEqual(ctrl.COMMAND.size, 20)
        self.assertEqual(ctrl.STATUS.size, 24)

    def test_round_trips_a_negative_error(self) -> None:
        packet = ctrl.encode_command(7, -0.5, 0.25, ctrl.FLAG_TRACKING)
        magic, version, size, sequence, pan, tilt, flags, _ = ctrl.COMMAND.unpack(packet)

        self.assertEqual(magic, ctrl.COMMAND_MAGIC)
        self.assertEqual(version, ctrl.VERSION)
        self.assertEqual(size, ctrl.COMMAND.size)
        self.assertEqual(sequence, 7)
        self.assertEqual(flags, ctrl.FLAG_TRACKING)
        self.assertAlmostEqual(pan / ctrl.Q15_ONE, -0.5, places=4)
        self.assertAlmostEqual(tilt / ctrl.Q15_ONE, 0.25, places=4)

    def test_clamps_an_out_of_range_error(self) -> None:
        """A vision bug must not be able to widen the commanded range."""
        packet = ctrl.encode_command(0, 9.0, -9.0, 0)
        _, _, _, _, pan, tilt, _, _ = ctrl.COMMAND.unpack(packet)
        self.assertEqual(pan, ctrl.Q15_ONE)
        self.assertEqual(tilt, -ctrl.Q15_ONE)

    def test_rejects_a_status_with_the_wrong_magic(self) -> None:
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)

        sender.sendall(ctrl.STATUS.pack(0xDEADBEEF, 1, 24, 0, 0, 0, 0, 0, 0))
        with self.assertRaises(ValueError):
            ctrl.receive_status(receiver)

    def test_receives_a_fragmented_status(self) -> None:
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)

        packet = ctrl.STATUS.pack(
            ctrl.STATUS_MAGIC, ctrl.VERSION, ctrl.STATUS.size,
            11, -200, -350, 875, ctrl.STATE_PAN_AT_LIMIT, 0,
        )
        # Tiny writes reproduce the fragmentation TCP is allowed to do.
        def dribble() -> None:
            for start in range(0, len(packet), 3):
                sender.sendall(packet[start : start + 3])

        thread = threading.Thread(target=dribble)
        thread.start()
        status = ctrl.receive_status(receiver)
        thread.join()

        self.assertEqual(status.pan_steps, -200)
        self.assertEqual(status.pan_rate, -350)
        self.assertAlmostEqual(status.tilt_degrees, 87.5)
        self.assertTrue(status.describes(ctrl.STATE_PAN_AT_LIMIT))
        self.assertIn("pan limit", status.summary())

    def test_a_measurement_carries_angles_and_capture_time(self) -> None:
        packet = ctrl.encode_measurement(9, 12.34, -5.5, ctrl.FLAG_TRACKING, 123_456_789)
        magic, version, _, sequence, pan, tilt, flags, captured_ms = ctrl.COMMAND.unpack(packet)
        self.assertEqual((magic, version, sequence), (ctrl.COMMAND_MAGIC, 2, 9))
        self.assertEqual((pan, tilt), (1234, -550))
        self.assertEqual(flags, ctrl.FLAG_TRACKING | ctrl.FLAG_MEASUREMENT)
        self.assertEqual(captured_ms, 123_456 & 0xFFFF)

    def test_capture_time_wraps_where_the_board_expects(self) -> None:
        """The board undoes the wrap with 16-bit subtraction, so only the low
        16 bits of milliseconds may be sent."""
        packet = ctrl.encode_measurement(0, 0.0, 0.0, 0, 65_537_000)
        self.assertEqual(ctrl.COMMAND.unpack(packet)[7], 1)

    def test_angles_are_clamped_to_the_field(self) -> None:
        packet = ctrl.encode_measurement(0, 999.0, -999.0, 0, 0)
        _, _, _, _, pan, tilt, _, _ = ctrl.COMMAND.unpack(packet)
        self.assertEqual((pan, tilt), (32767, -32767))

    def test_a_manual_command_never_claims_to_be_a_measurement(self) -> None:
        packet = ctrl.encode_command(0, 0.5, 0.0, ctrl.FLAG_MEASUREMENT | ctrl.FLAG_TRACKING)
        self.assertEqual(ctrl.COMMAND.unpack(packet)[6], ctrl.FLAG_TRACKING)

    def test_status_reports_how_old_the_boards_picture_is(self) -> None:
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)
        sender.sendall(ctrl.STATUS.pack(
            ctrl.STATUS_MAGIC, ctrl.VERSION, ctrl.STATUS.size, 1, 0, 0, 900,
            ctrl.STATE_TRACKING | ctrl.STATE_PREDICTIVE, 142,
        ))
        status = ctrl.receive_status(receiver)
        self.assertEqual(status.vision_latency_ms, 142)
        self.assertIn("predictive", status.summary())

    def test_a_version_1_board_is_refused_by_name(self) -> None:
        sender, receiver = socket.socketpair()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)
        sender.sendall(ctrl.STATUS.pack(ctrl.STATUS_MAGIC, 1, 24, 0, 0, 0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, "reflash"):
            ctrl.receive_status(receiver)


class HandReadingTest(unittest.TestCase):
    def test_centre_of_frame_is_zero_error(self) -> None:
        reading = HandReading(visible=True, center_x=0.5, center_y=0.5)
        self.assertEqual(reading.pan_error, 0.0)
        self.assertEqual(reading.tilt_error, 0.0)

    def test_error_signs_follow_the_firmware_convention(self) -> None:
        """Right of centre is positive pan; below centre is positive tilt."""
        right_and_low = HandReading(visible=True, center_x=1.0, center_y=1.0)
        self.assertAlmostEqual(right_and_low.pan_error, 1.0)
        self.assertAlmostEqual(right_and_low.tilt_error, 1.0)

        left_and_high = HandReading(visible=True, center_x=0.0, center_y=0.0)
        self.assertAlmostEqual(left_and_high.pan_error, -1.0)
        self.assertAlmostEqual(left_and_high.tilt_error, -1.0)


class WorkingAreaTest(unittest.TestCase):
    """The tracking error is measured against the usable part of the frame."""

    def test_full_frame_roi_matches_the_old_behaviour(self) -> None:
        reading = HandReading(visible=True, center_x=0.75, center_y=0.25)
        self.assertAlmostEqual(reading.pan_error, 0.5)
        self.assertAlmostEqual(reading.tilt_error, -0.5)

    def test_edge_of_the_working_area_is_full_error(self) -> None:
        """With a 0.8 ROI the hand reaches full error at x=0.9, not x=1.0 --
        the point is that x=1.0 is black and a hand can never be seen there."""
        roi = (0.1, 0.1, 0.9, 0.9)
        self.assertAlmostEqual(
            HandReading(visible=True, center_x=0.9, center_y=0.5, roi=roi).pan_error, 1.0
        )
        self.assertAlmostEqual(
            HandReading(visible=True, center_x=0.1, center_y=0.5, roi=roi).pan_error, -1.0
        )
        self.assertAlmostEqual(
            HandReading(visible=True, center_x=0.5, center_y=0.9, roi=roi).tilt_error, 1.0
        )

    def test_roi_centre_is_still_zero_error(self) -> None:
        roi = (0.1, 0.125, 0.9, 0.875)
        reading = HandReading(visible=True, center_x=0.5, center_y=0.5, roi=roi)
        self.assertAlmostEqual(reading.pan_error, 0.0)
        self.assertAlmostEqual(reading.tilt_error, 0.0)

    def test_a_narrow_roi_reaches_full_speed_sooner(self) -> None:
        """Shrinking the working area must make a given hand position a LARGER
        error, otherwise the gimbal would never reach full speed on this lens."""
        wide = HandReading(visible=True, center_x=0.8, roi=(0.0, 0.0, 1.0, 1.0))
        narrow = HandReading(visible=True, center_x=0.8, roi=(0.2, 0.2, 0.8, 0.8))
        self.assertGreater(narrow.pan_error, wide.pan_error)

    def test_outside_the_working_area_saturates_rather_than_wrapping(self) -> None:
        """A hand just past the ROI edge must stay a full-speed command in the
        same direction; the Q15 encoder is what clamps it."""
        roi = (0.2, 0.2, 0.8, 0.8)
        reading = HandReading(visible=True, center_x=0.95, center_y=0.5, roi=roi)
        self.assertGreater(reading.pan_error, 1.0)
        packet = ctrl.encode_command(0, reading.pan_error, reading.tilt_error, 0)
        _, _, _, _, pan, _, _, _ = ctrl.COMMAND.unpack(packet)
        self.assertEqual(pan, ctrl.Q15_ONE)


class CameraModelTest(unittest.TestCase):
    def test_the_centre_of_the_picture_is_on_axis(self) -> None:
        self.assertEqual(CameraModel().angles(0.5, 0.5), (0.0, 0.0))

    def test_signs_match_the_board(self) -> None:
        """Right of centre is positive pan; below centre is positive tilt."""
        pan, tilt = CameraModel().angles(0.8, 0.9)
        self.assertGreater(pan, 0.0)
        self.assertGreater(tilt, 0.0)
        left, high = CameraModel().angles(0.2, 0.1)
        self.assertAlmostEqual(left, -pan)
        self.assertAlmostEqual(high, -tilt)

    def test_the_default_matches_the_measurement_it_came_from(self) -> None:
        """A 13.1 degree tilt moved the picture 120 px of 480."""
        _, tilt = CameraModel().angles(0.5, 0.5 + 120 / 480)
        self.assertAlmostEqual(tilt, 13.1, places=1)

    def test_calibration_survives_a_round_trip(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            path = f"{folder}/calibration.json"
            CameraModel(focal_x=0.9, focal_y=1.2).save(path, samples=3)
            loaded = CameraModel.load(path)
        self.assertEqual((loaded.focal_x, loaded.focal_y), (0.9, 1.2))

    def test_no_calibration_file_means_the_default(self) -> None:
        self.assertEqual(CameraModel.load("/nonexistent/calibration.json"), CameraModel())


class StaleReadingTest(unittest.TestCase):
    """A reading the video has stopped refreshing must not keep the gimbal moving."""

    def command_flags(self, link, at: float) -> int:
        import tracker

        with mock.patch.object(tracker.time, "monotonic", return_value=at):
            _, _, _, _, _, _, flags, _ = ctrl.COMMAND.unpack(link._next_command(0))
        return flags

    def test_a_followed_hand_stops_granting_motion_once_the_video_stalls(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0)
        with mock.patch.object(tracker.time, "monotonic", return_value=100.0):
            link.publish(HandReading(visible=True, following=True, center_x=0.9))

        fresh = self.command_flags(link, 100.0 + tracker.READING_MAX_AGE_S / 2)
        stale = self.command_flags(link, 100.0 + tracker.READING_MAX_AGE_S * 2)

        self.assertTrue(fresh & ctrl.FLAG_TRACKING)
        self.assertFalse(stale & ctrl.FLAG_TRACKING)
        self.assertFalse(stale & ctrl.FLAG_HAND_VISIBLE)

    def test_nothing_published_yet_is_stale(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0)
        self.assertFalse(self.command_flags(link, 0.0) & ctrl.FLAG_TRACKING)

    def test_a_reading_with_a_capture_time_is_sent_as_a_measurement(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        with mock.patch.object(tracker.time, "monotonic", return_value=100.0):
            link.publish(HandReading(visible=True, following=True, center_x=0.75, center_y=0.5),
                         captured_at_us=5_000_000)
            packet = link._next_command(0)
        _, _, _, _, pan, tilt, flags, captured_ms = ctrl.COMMAND.unpack(packet)
        expected_pan, _ = CameraModel().angles(0.75, 0.5)
        self.assertTrue(flags & ctrl.FLAG_MEASUREMENT)
        self.assertTrue(flags & ctrl.FLAG_TRACKING)
        self.assertAlmostEqual(pan / ctrl.CENTIDEGREES, expected_pan, places=2)
        self.assertEqual(tilt, 0)
        self.assertEqual(captured_ms, 5000)

    def test_a_reading_without_a_capture_time_stays_a_manual_speed(self) -> None:
        """motor_test.py and axis_selftest.py drive the axes this way."""
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        with mock.patch.object(tracker.time, "monotonic", return_value=100.0):
            link.publish(HandReading(visible=True, following=True, center_x=0.75))
            packet = link._next_command(0)
        flags = ctrl.COMMAND.unpack(packet)[6]
        self.assertFalse(flags & ctrl.FLAG_MEASUREMENT)
        self.assertTrue(flags & ctrl.FLAG_TRACKING)

    def test_a_stale_measurement_is_not_resent_as_one(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        with mock.patch.object(tracker.time, "monotonic", return_value=100.0):
            link.publish(HandReading(visible=True, following=True, center_x=0.75), captured_at_us=1)
        flags = self.command_flags(link, 100.0 + tracker.READING_MAX_AGE_S * 2)
        self.assertFalse(flags & (ctrl.FLAG_MEASUREMENT | ctrl.FLAG_TRACKING))

    def test_a_manual_jog_is_not_affected(self) -> None:
        """Jogging is an explicit keypress, not an inference from old video."""
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0)
        with mock.patch.object(tracker.time, "monotonic", return_value=100.0):
            link.jog(0.5, 0.0)
        self.assertTrue(self.command_flags(link, 100.1) & ctrl.FLAG_TRACKING)


class MissedFrameTest(unittest.TestCase):
    """One frame without the locked hand must not stop the board following."""

    def publish_at(self, link, at: float, reading: HandReading, captured_at_us: int | None) -> None:
        import tracker

        with mock.patch.object(tracker.time, "monotonic", return_value=at):
            link.publish(reading, captured_at_us)

    def command_at(self, link, at: float) -> tuple[int, int]:
        import tracker

        with mock.patch.object(tracker.time, "monotonic", return_value=at):
            unpacked = ctrl.COMMAND.unpack(link._next_command(0))
        return unpacked[6], unpacked[7]

    def test_the_last_sighting_is_repeated_through_a_missed_frame(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        seen = HandReading(visible=True, following=True, locked=True, center_x=0.7)
        self.publish_at(link, 100.00, seen, 5_000_000)
        self.publish_at(link, 100.06, HandReading(locked=True), 5_058_000)

        flags, captured_ms = self.command_at(link, 100.07)
        self.assertTrue(flags & ctrl.FLAG_TRACKING)
        self.assertTrue(flags & ctrl.FLAG_MEASUREMENT)
        # The same capture time, so the board does not count it twice.
        self.assertEqual(captured_ms, 5000)

    def test_the_sighting_still_goes_stale(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        self.publish_at(link, 100.0, HandReading(visible=True, following=True, locked=True), 5_000_000)
        for frame in range(1, 10):
            self.publish_at(link, 100.0 + frame * 0.06, HandReading(locked=True), 5_000_000 + frame)
        flags, _ = self.command_at(link, 100.0 + tracker.READING_MAX_AGE_S + 0.05)
        self.assertFalse(flags & (ctrl.FLAG_TRACKING | ctrl.FLAG_MEASUREMENT))

    def test_letting_go_stops_following_at_once(self) -> None:
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        self.publish_at(link, 100.00, HandReading(visible=True, following=True, locked=True), 5_000_000)
        self.publish_at(link, 100.06, HandReading(locked=False), 5_058_000)
        flags, _ = self.command_at(link, 100.07)
        self.assertFalse(flags & ctrl.FLAG_TRACKING)


class CameraPoseLookupTest(unittest.TestCase):
    def test_reports_become_poses_with_the_direction_overrides(self) -> None:
        import gimbal_config
        import tracker

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        nine_degrees = int(round(9 * gimbal_config.PAN_STEPS_PER_DEGREE))
        for tick in range(20):
            at = 50.0 + tick * 0.01
            link.poses.record(at, gimbal_config.pan_degrees(tick * nine_degrees),
                              gimbal_config.tilt_degrees_from_center(90.0 + tick))
        pose = link.camera_pose(50.10)
        self.assertAlmostEqual(pose.pan_degrees, 90.0, places=3)
        # Tilt is read the servo's lag earlier.
        lag_ticks = gimbal_config.TILT_SERVO_LAG_MS / 10.0
        self.assertAlmostEqual(pose.tilt_degrees, 10.0 - lag_ticks, places=3)

        link.flip_pan()
        link.flip_tilt()
        flipped = link.camera_pose(50.10)
        self.assertAlmostEqual(flipped.pan_degrees, -pose.pan_degrees)
        self.assertAlmostEqual(flipped.tilt_degrees, -pose.tilt_degrees)

    def test_nothing_before_the_board_has_replied(self) -> None:
        import tracker

        self.assertIsNone(tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel()).camera_pose(1.0))

    def test_a_frame_is_looked_up_at_its_exposure(self) -> None:
        import gimbal_config
        import tracker
        from camera_pose import BoardClock
        from protocol import CameraFrame

        link = tracker.GimbalLink("127.0.0.1", 0, camera=CameraModel())
        for tick in range(100):
            link.poses.record(200.0 + tick * 0.01, tick * 0.5, 0.0)  # 50 deg/s
        clock = BoardClock()
        clock.observe(captured_at_us=7_000_000, arrived_at=200.5)  # board 7 s = desktop 200.5
        frame = CameraFrame(0, 7_300_000, 640, 480, 0, b"", 200.9)
        pose = tracker.exposure_pose(frame, clock, link)
        exposed_at = 200.8 - gimbal_config.CAMERA_EXPOSURE_DELAY_MS / 1000.0
        self.assertAlmostEqual(pose.pan_degrees, (exposed_at - 200.0) * 50.0, places=2)


if __name__ == "__main__":
    unittest.main()
