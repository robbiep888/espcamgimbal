"""Hand signs and the hand lock, on synthetic hands. No board, no camera."""

from __future__ import annotations

import math
import unittest

import numpy as np

from camera_model import CameraModel
from camera_pose import Pose
from hand_lock import (MATCH_DISTANCE_PALMS, STOP_GRACE_S, STOP_HOLD_S, HandLock)
from hand_tracker import (ZOOM_MIN_PIXELS, count_curled_fingers, is_stop_sign, observe_hand,
                          palm_size_degrees, zoom_box)

ASPECT = 4 / 3
FRAME = 1.0 / 17  # seconds between frames at this camera's rate
CAMERA = CameraModel()


def build_hand(curled: frozenset[int] = frozenset(), thumb_out: bool = True,
               lean_degrees: float = 0.0, center: tuple[float, float] = (0.5, 0.62),
               scale: float = 1.0, tip_toward_camera: float = 1.0) -> np.ndarray:
    """A synthetic 21-landmark right hand, palm to the camera, fingers up.

    Its proportions follow an adult hand: knuckle span about 1/1.4 of palm
    length. Named fingers (0 index .. 3 little) are curled with their tips
    pulled back below the middle joint. `lean_degrees` rotates the hand about
    the wrist, in picture terms, so 90 is a hand pointing sideways.
    `tip_toward_camera` foreshortens everything along the hand's length, as
    tipping the fingers toward the lens does: 0.4 leaves 40% of the length.
    """
    points = np.zeros((21, 2), dtype=np.float32)
    points[0] = (0.50, 0.90)  # wrist
    for slot, finger_x in enumerate((0.452, 0.484, 0.516, 0.548)):
        mcp, pip, dip, tip = 5 + 4 * slot, 6 + 4 * slot, 7 + 4 * slot, 8 + 4 * slot
        points[mcp] = (finger_x, 0.72)
        points[pip] = (finger_x, 0.62)
        points[dip] = (finger_x, 0.55)
        points[tip] = (finger_x, 0.70) if slot in curled else (finger_x, 0.48)
    points[1] = (0.42, 0.85)  # thumb joints, then the tip out or folded in
    points[2] = (0.38, 0.80)
    points[3] = (0.35, 0.76)
    points[4] = (0.32, 0.72) if thumb_out else (0.50, 0.78)
    points[:, 1] = points[0, 1] + (points[:, 1] - points[0, 1]) * tip_toward_camera

    # Rotate about the wrist in pixel space, then scale and move the hand.
    angle = math.radians(lean_degrees)
    offset = (points - points[0]) * np.array([ASPECT, 1.0], dtype=np.float32)
    rotated = np.stack([
        offset[:, 0] * math.cos(angle) + offset[:, 1] * math.sin(angle),
        -offset[:, 0] * math.sin(angle) + offset[:, 1] * math.cos(angle),
    ], axis=1) / np.array([ASPECT, 1.0], dtype=np.float32)
    palm_centre = points[[0, 5, 9, 13, 17]].mean(axis=0) - points[0]
    placed = rotated * scale + np.array(center, dtype=np.float32) - palm_centre * scale
    return placed.astype(np.float32)


def observe(points: np.ndarray, handedness: str = "Right", score: float = 0.95,
            pose: Pose | None = None):
    return observe_hand(points, handedness, score, ASPECT, CAMERA, pose)


def in_room(pan: float, tilt: float, camera_at: Pose, **hand) -> np.ndarray:
    """A hand fixed in the room at (pan, tilt) degrees, as the camera sees it
    while pointing at camera_at."""
    x, y = CAMERA.position(pan - camera_at.pan_degrees, tilt - camera_at.tilt_degrees)
    return build_hand(center=(x, y), **hand)


class HandSignTest(unittest.TestCase):
    def test_a_fist_counts_four_curled_fingers(self) -> None:
        self.assertEqual(count_curled_fingers(build_hand(frozenset({0, 1, 2, 3})), ASPECT), 4)

    def test_an_open_hand_counts_none(self) -> None:
        self.assertEqual(count_curled_fingers(build_hand(), ASPECT), 0)

    def test_the_count_is_scale_invariant(self) -> None:
        near = build_hand(frozenset({0, 1, 2}))
        far = build_hand(frozenset({0, 1, 2}), scale=0.4)
        self.assertEqual(count_curled_fingers(near, ASPECT), count_curled_fingers(far, ASPECT))

    def test_a_raised_open_palm_is_a_stop_sign(self) -> None:
        self.assertTrue(is_stop_sign(build_hand(), ASPECT))
        self.assertTrue(is_stop_sign(build_hand(scale=0.5), ASPECT))
        self.assertTrue(is_stop_sign(build_hand(lean_degrees=30), ASPECT))

    def test_a_fist_is_not(self) -> None:
        self.assertFalse(is_stop_sign(build_hand(frozenset({0, 1, 2, 3})), ASPECT))

    def test_one_curled_finger_is_not(self) -> None:
        self.assertFalse(is_stop_sign(build_hand(frozenset({3})), ASPECT))

    def test_a_folded_thumb_is_not(self) -> None:
        self.assertFalse(is_stop_sign(build_hand(thumb_out=False), ASPECT))

    def test_a_hand_pointing_sideways_or_down_is_not(self) -> None:
        self.assertFalse(is_stop_sign(build_hand(lean_degrees=70), ASPECT))
        self.assertFalse(is_stop_sign(build_hand(lean_degrees=180), ASPECT))


class PalmSizeTest(unittest.TestCase):
    def test_turning_the_hand_barely_changes_its_size(self) -> None:
        flat = palm_size_degrees(build_hand(), CAMERA)
        tipped = palm_size_degrees(build_hand(tip_toward_camera=0.4), CAMERA)
        sideways = palm_size_degrees(build_hand(lean_degrees=80), CAMERA)
        self.assertAlmostEqual(tipped / flat, 1.0, delta=0.15)
        self.assertAlmostEqual(sideways / flat, 1.0, delta=0.15)

    def test_a_further_hand_is_smaller(self) -> None:
        self.assertAlmostEqual(palm_size_degrees(build_hand(scale=0.5), CAMERA)
                               / palm_size_degrees(build_hand(), CAMERA), 0.5, delta=0.05)


class HandLockTest(unittest.TestCase):
    def lock_onto(self, *hands) -> tuple[HandLock, float]:
        lock = HandLock()
        message = lock.toggle(list(hands), now=10.0)
        self.assertTrue(lock.locked, message)
        return lock, 10.0

    def test_r_with_no_hand_in_view_locks_nothing(self) -> None:
        lock = HandLock()
        self.assertIn("no hand", lock.toggle([], now=1.0))
        self.assertFalse(lock.locked)
        update = lock.update([observe(build_hand())], now=1.1)
        self.assertIsNone(update.hand)

    def test_r_locks_onto_the_hand_nearest_the_centre(self) -> None:
        edge = observe(build_hand(center=(0.15, 0.5)))
        middle = observe(build_hand(center=(0.55, 0.5)), handedness="Left")
        lock, now = self.lock_onto(edge, middle)
        update = lock.update([edge, middle], now + FRAME)
        self.assertIs(update.hand, middle)
        self.assertEqual(update.others, (edge,))

    def test_only_the_locked_hand_is_followed_as_it_moves(self) -> None:
        mine = observe(build_hand(center=(0.5, 0.5)))
        lock, now = self.lock_onto(mine)
        stranger = observe(build_hand(center=(0.12, 0.3)))
        for step in range(1, 30):
            now += FRAME
            moved = observe(build_hand(center=(0.5 + 0.01 * step, 0.5)))
            update = lock.update([stranger, moved], now)
            self.assertIs(update.hand, moved)

    def test_another_hand_is_ignored_while_the_locked_one_is_out_of_sight(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.7, 0.5))))
        stranger = observe(build_hand(center=(0.15, 0.5)))
        for _ in range(60):  # over three seconds of only the stranger in view
            now += FRAME
            self.assertIsNone(lock.update([stranger], now).hand)
        self.assertTrue(lock.locked)

    def test_a_hand_that_turns_over_is_kept_when_its_label_flips(self) -> None:
        """A left palm looks like a right back, so MediaPipe confidently calls
        a turning hand the other side. That must not lose it."""
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5)), handedness="Right"))
        for step in range(1, 10):
            now += FRAME
            lock.update([observe(build_hand(center=(0.5 + 0.002 * step, 0.5)))], now)
        for step in range(10):
            now += FRAME
            turned = observe(build_hand(center=(0.52, 0.5)), handedness="Left", score=0.97)
            self.assertIs(lock.update([turned], now).hand, turned)

    def test_a_close_contest_goes_to_the_usual_side(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5), scale=0.4)))
        for _ in range(5):
            now += FRAME
            lock.update([observe(build_hand(center=(0.5, 0.5), scale=0.4))], now)
        # Two hands appear equally near where it was; only one is a right hand.
        right = observe(build_hand(center=(0.53, 0.5), scale=0.4), handedness="Right")
        left = observe(build_hand(center=(0.47, 0.5), scale=0.4), handedness="Left", score=0.97)
        self.assertIs(lock.update([left, right], now + FRAME).hand, right)

    def test_an_unsure_handedness_call_does_not_lose_the_hand(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5)), handedness="Right"))
        flickered = observe(build_hand(center=(0.51, 0.5)), handedness="Left", score=0.55)
        self.assertIs(lock.update([flickered], now + FRAME).hand, flickered)

    def test_a_hand_tipping_toward_the_camera_is_kept(self) -> None:
        """Tipping foreshortens the palm length to 40%. Measured along that line
        alone, the hand halved in size in one frame and was refused."""
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5), scale=0.5)))
        tipped = observe(build_hand(center=(0.5, 0.5), scale=0.5, tip_toward_camera=0.4))
        self.assertIs(lock.update([tipped], now + FRAME).hand, tipped)

    def test_a_wrong_size_estimate_cannot_shut_the_hand_out_for_good(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5), scale=0.9)))
        # Seen again at under half the size: refused at first...
        small = observe(build_hand(center=(0.5, 0.5), scale=0.4))
        now += FRAME
        self.assertIsNone(lock.update([small], now).hand)
        # ...but taken once it has been out of sight a moment.
        for _ in range(8):
            now += FRAME
            if lock.update([small], now).hand is small:
                break
        else:
            self.fail("the hand was never taken back")

    def test_a_briefly_lost_hand_is_found_again_near_where_it_went(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5))))
        for _ in range(5):
            now += FRAME
            self.assertIsNone(lock.update([], now).hand)
        back = observe(build_hand(center=(0.62, 0.55)))
        self.assertIs(lock.update([back], now + FRAME).hand, back)

    def test_the_hand_is_kept_while_the_gimbal_swings_toward_it(self) -> None:
        """A hand about two metres away (palm ~3 degrees), 20 degrees right.
        The gimbal turns to it on its braking curve, moving the hand across the
        picture up to 7 degrees a frame -- over two palm lengths -- though the
        hand itself never moves."""
        swing = [0.0, 2.0, 8.0, 15.0, 19.0, 20.0, 20.0]
        lock, now = self.lock_onto(observe(in_room(20.0, 0.0, Pose(), scale=0.3)))
        for pan in swing[1:]:
            now += FRAME
            camera_at = Pose(pan, 0.0)
            seen = observe(in_room(20.0, 0.0, camera_at, scale=0.3), pose=camera_at)
            self.assertIs(lock.update([seen], now).hand, seen, f"lost at pan {pan}")

        # Matched by picture position alone, the same swing loses the hand.
        blind, now = self.lock_onto(observe(in_room(20.0, 0.0, Pose(), scale=0.3)))
        lost = 0
        for pan in swing[1:]:
            now += FRAME
            seen = observe(in_room(20.0, 0.0, Pose(pan, 0.0), scale=0.3))
            lost += blind.update([seen], now).hand is None
        self.assertGreater(lost, 0)

    def test_the_other_hand_is_not_taken_when_the_locked_one_vanishes_beside_it(self) -> None:
        mine = observe(build_hand(center=(0.46, 0.5), scale=0.4))
        other = observe(build_hand(center=(0.58, 0.5), scale=0.4), handedness="Left")
        lock, now = self.lock_onto(mine, other)
        self.assertLess(other.degrees_to(mine.pan_degrees, mine.tilt_degrees)
                        / mine.size_degrees, MATCH_DISTANCE_PALMS + 0.5,
                        "the other hand must be close enough to be mistaken")
        for _ in range(3):
            now += FRAME
            self.assertIs(lock.update([mine, other], now).hand, mine)
        for _ in range(20):  # a second with only the other hand in view
            now += FRAME
            self.assertIsNone(lock.update([other], now).hand)
        now += FRAME
        self.assertIs(lock.update([other, mine], now).hand, mine)

    def test_hands_side_by_side_keep_their_places_as_they_move_together(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.44, 0.5), scale=0.4)),
                                   observe(build_hand(center=(0.57, 0.5), scale=0.4)))
        for step in range(1, 12):
            now += FRAME
            mine = observe(build_hand(center=(0.44 + 0.008 * step, 0.5), scale=0.4))
            other = observe(build_hand(center=(0.57 - 0.002 * step, 0.5), scale=0.4))
            self.assertIs(lock.update([other, mine], now).hand, mine)

    def test_holding_a_stop_sign_for_three_seconds_lets_go(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(frozenset({0, 1, 2, 3}))))
        stop = observe(build_hand())
        self.assertTrue(stop.stop_sign)
        released_at = None
        for _ in range(int(4 * 17)):
            now += FRAME
            update = lock.update([stop], now)
            if update.released_by_stop:
                released_at = now
                break
            self.assertTrue(lock.locked)
        self.assertIsNotNone(released_at)
        self.assertAlmostEqual(released_at - 10.0, STOP_HOLD_S + FRAME, delta=2 * FRAME)
        self.assertFalse(lock.locked)
        self.assertIsNone(lock.update([stop], now + FRAME).hand)

    def test_a_stop_sign_still_lets_go_while_the_gimbal_centres_it(self) -> None:
        """Held still in the room, the sign crosses the picture as the gimbal
        turns to it; that is not the hand moving."""
        lock, now = self.lock_onto(observe(in_room(12.0, 4.0, Pose(), scale=0.5)))
        for frame in range(int(4 * 17)):
            now += FRAME
            camera_at = Pose(min(12.0, frame * 1.5), min(4.0, frame * 0.5))
            update = lock.update([observe(in_room(12.0, 4.0, camera_at, scale=0.5),
                                          pose=camera_at)], now)
            if update.released_by_stop:
                break
        self.assertFalse(lock.locked)

    def test_moving_an_open_hand_around_is_not_a_stop_sign(self) -> None:
        """A raised open hand is the stop sign's shape and the natural way to
        hold a hand up to be followed. Waving it about must not let go."""
        lock, now = self.lock_onto(observe(build_hand(center=(0.3, 0.5), scale=0.5)))
        for frame in range(int(6 * 17)):
            now += FRAME
            x = 0.5 + 0.2 * math.sin(frame * 0.25)
            update = lock.update([observe(build_hand(center=(x, 0.5), scale=0.5))], now)
            self.assertFalse(update.released_by_stop)
            self.assertLess(update.stop_progress, 0.5)
        self.assertTrue(lock.locked)

    def test_a_flicker_does_not_restart_the_stop_count(self) -> None:
        lock, now = self.lock_onto(observe(build_hand()))
        stop, fist = observe(build_hand()), observe(build_hand(frozenset({0, 1, 2, 3})))
        now += FRAME
        lock.update([stop], now)
        now += 1.5
        lock.update([stop], now)
        now += STOP_GRACE_S / 2  # one misread frame
        lock.update([fist], now)
        now += FRAME
        self.assertGreater(lock.update([stop], now).stop_progress, 0.45)

    def test_dropping_the_stop_sign_starts_the_count_again(self) -> None:
        lock, now = self.lock_onto(observe(build_hand()))
        stop, fist = observe(build_hand()), observe(build_hand(frozenset({0, 1, 2, 3})))
        lock.update([stop], now + FRAME)
        now += 2.0
        lock.update([stop], now)
        now += STOP_GRACE_S * 2
        self.assertEqual(lock.update([fist], now).stop_progress, 0.0)

    def test_r_again_releases_a_hand_being_followed(self) -> None:
        mine = observe(build_hand())
        lock, now = self.lock_onto(mine)
        lock.update([mine], now + FRAME)
        self.assertIn("released", lock.toggle([mine], now + 2 * FRAME))
        self.assertFalse(lock.locked)

    def test_r_after_losing_the_hand_locks_on_again_instead_of_releasing(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.8, 0.5))))
        now += 5.0
        lock.update([], now)
        returned = observe(build_hand(center=(0.3, 0.5)))
        self.assertIn("locked", lock.toggle([returned], now))
        self.assertIs(lock.update([returned], now + FRAME).hand, returned)

    def test_find_changes_nothing(self) -> None:
        lock, now = self.lock_onto(observe(build_hand(center=(0.5, 0.5))))
        before = lock.last_seen
        moved = observe(build_hand(center=(0.55, 0.5)))
        self.assertIs(lock.find([moved], now + FRAME), moved)
        self.assertEqual(lock.last_seen, before)


class ZoomBoxTest(unittest.TestCase):
    def test_centred_on_where_the_hand_should_be_now(self) -> None:
        """Seen 10 degrees right with the camera at home; the camera has since
        turned 4 degrees right, so the hand is 6 degrees right of centre."""
        size = 3.0
        left, top, right, bottom = zoom_box(640, 480, CAMERA, Pose(4.0, 0.0), (10.0, 0.0, size))
        expected_x, expected_y = CAMERA.position(6.0, 0.0)
        self.assertAlmostEqual((left + right) / 2 / 640, expected_x, delta=2 / 640)
        self.assertAlmostEqual((top + bottom) / 2 / 480, expected_y, delta=2 / 480)
        self.assertEqual(right - left, bottom - top)

    def test_stays_inside_the_frame(self) -> None:
        left, top, right, bottom = zoom_box(640, 480, CAMERA, Pose(), (40.0, -30.0, 4.0))
        self.assertGreaterEqual(left, 0)
        self.assertGreaterEqual(top, 0)
        self.assertLessEqual(right, 640)
        self.assertLessEqual(bottom, 480)

    def test_never_smaller_than_the_detector_input_nor_larger_than_the_frame(self) -> None:
        tiny = zoom_box(640, 480, CAMERA, Pose(), (0.0, 0.0, 0.2))
        huge = zoom_box(640, 480, CAMERA, Pose(), (0.0, 0.0, 40.0))
        self.assertEqual(tiny[2] - tiny[0], ZOOM_MIN_PIXELS)
        self.assertEqual(huge[3] - huge[1], 480)


if __name__ == "__main__":
    unittest.main()
