"""Clock mapping and pose lookup for the hand lock. No board, no camera."""

from __future__ import annotations

import unittest

from camera_model import CameraModel
from camera_pose import BoardClock, Pose, PoseHistory


class BoardClockTest(unittest.TestCase):
    def test_nothing_is_known_before_a_frame(self) -> None:
        self.assertIsNone(BoardClock().to_desktop(1_000_000))

    def test_the_quickest_delivery_sets_the_offset(self) -> None:
        clock = BoardClock()
        # Board time 10 s is desktop time 500 s; frames take 12-60 ms to arrive.
        for board_s, delay in ((10.00, 0.060), (10.05, 0.012), (10.10, 0.045), (10.15, 0.030)):
            clock.observe(int(board_s * 1e6), 490.0 + board_s + delay)
        self.assertAlmostEqual(clock.to_desktop(11_000_000), 501.012, places=3)

    def test_a_slow_frame_never_moves_it_later(self) -> None:
        clock = BoardClock()
        clock.observe(0, 100.010)
        clock.observe(50_000, 100.060 + 0.400)  # held up 400 ms in transit
        self.assertAlmostEqual(clock.to_desktop(0), 100.010, places=3)

    def test_a_reboot_starts_the_estimate_again(self) -> None:
        clock = BoardClock()
        clock.observe(3_600_000_000, 5000.010)  # an hour of uptime
        clock.observe(2_000_000, 5010.010)  # rebooted: 2 s of uptime
        self.assertAlmostEqual(clock.to_desktop(2_000_000), 5010.010, places=3)


class PoseHistoryTest(unittest.TestCase):
    def test_nothing_reported_answers_nothing(self) -> None:
        self.assertIsNone(PoseHistory().at(1.0))

    def test_interpolates_between_reports(self) -> None:
        history = PoseHistory()
        history.record(1.00, 0.0, 10.0)
        history.record(1.01, 1.0, 10.0)
        history.record(1.02, 3.0, 8.0)
        pose = history.at(1.015)
        self.assertAlmostEqual(pose.pan_degrees, 2.0)
        self.assertAlmostEqual(pose.tilt_degrees, 9.0)

    def test_beyond_either_end_gives_the_nearest_report(self) -> None:
        history = PoseHistory()
        history.record(1.0, 5.0, -1.0)
        history.record(2.0, 7.0, -2.0)
        self.assertEqual(history.at(9.0), Pose(7.0, -2.0))
        self.assertEqual(history.at(0.5), Pose(5.0, -1.0))

    def test_keeps_only_the_newest_reports(self) -> None:
        history = PoseHistory(capacity=3)
        for tick in range(10):
            history.record(float(tick), float(tick), 0.0)
        self.assertEqual(history.at(0.0), Pose(7.0, 0.0))


class CameraModelInverseTest(unittest.TestCase):
    def test_position_undoes_angles(self) -> None:
        camera = CameraModel(focal_x=0.85, focal_y=0.95)
        for x, y in ((0.5, 0.5), (0.1, 0.9), (0.73, 0.21)):
            pan, tilt = camera.angles(x, y)
            back_x, back_y = camera.position(pan, tilt)
            self.assertAlmostEqual(back_x, x, places=9)
            self.assertAlmostEqual(back_y, y, places=9)


if __name__ == "__main__":
    unittest.main()
