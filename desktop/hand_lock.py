"""Which hand the gimbal follows, and when it lets go.

Several hands can be in view -- your other hand, someone walking past -- and
MediaPipe reports them in no particular order. Pressing r locks onto one: the
hand nearest the centre of the picture. From then on only that hand is
followed.

Each frame the locked hand is picked out again as the hand nearest where it was
last seen. "Where" is an angle fixed to the room: where the camera pointed when
the frame was exposed, plus where in the picture the hand was (camera_pose.py).
Matching by position in the picture instead lost the hand whenever the gimbal
swung toward it, which moves the hand across the picture several palm widths a
frame -- far more than a hand moves by itself.

Other hands seen at the same time as the locked one are remembered and followed
too, and every frame the hands are shared out nearest pair first. So a hand
known to be someone else's is never taken for the locked one, even while the
locked one is out of sight right next to it. A locked hand that vanishes is
only picked up again near where it went.

Two cues were rules and are now only tie-breakers, because each lost the hand
for good in ordinary use. MediaPipe's left/right call flips, confidently, when
a hand turns over: a left palm and a right back look alike. And a palm measured
along one line shrinks when the hand tips toward the camera, and a size
estimate that only updated on a match never caught up once the hand had been
rejected.

Holding a stop sign -- palm open, fingers up, thumb out -- in place for
STOP_HOLD_S releases the lock, and the gimbal stops following until r is
pressed again.

Nothing here needs MediaPipe, so it is tested with synthetic hands.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

# How long a stop sign must be held to let go of the hand.
STOP_HOLD_S = 3.0
# A stop sign that drops out for less than this -- a blurred frame, a missed
# detection -- still counts as held, rather than restarting the count.
STOP_GRACE_S = 0.4
# A raised open hand is a stop sign by shape, and it is also the most natural
# way to hold up a hand to be followed. Moving it around therefore restarts the
# count: only a stop sign kept within this many palm lengths of where it started
# lets go.
STOP_MAX_DRIFT_PALMS = 1.0

# How far the locked hand may be from where it was last seen, in palm lengths.
# The gimbal's own motion is already taken out; what is left is the hand's own
# movement plus a couple of degrees of error in lining frames up with the
# gimbal's reported angles while it swings.
MATCH_DISTANCE_PALMS = 2.0
# While the hand is out of sight that radius widens, so a hand that vanished
# mid-movement -- motion blur takes it out for a frame or two -- is found where
# it stopped. It stops widening at MAX_SEARCH_PALMS: any hand further than that
# from where the locked one vanished is never taken for it, however long it has
# been gone. Press r to lock onto a hand elsewhere.
SEARCH_GROWTH_PALMS_PER_S = 4.0
MAX_SEARCH_PALMS = 3.5

# A hand twice or half the size of the locked one is a different hand: moving
# that much nearer takes far longer than a frame. The limit relaxes while the
# hand is out of sight, since it may come back nearer or further, and so that a
# bad size estimate can never shut the right hand out for good.
SIZE_RATIO_LIMIT = 2.0
SIZE_RATIO_GROWTH_PER_S = 2.0
# Share of each frame's size measurement believed; the rest is the estimate.
SIZE_SMOOTHING = 0.3

# MediaPipe's left/right call is only counted when it is this sure. A hand whose
# call disagrees with the locked hand's usual one is ranked this many palm
# lengths further away: it loses a close contest, but it is never refused.
HANDEDNESS_CONFIDENT = 0.8
HANDEDNESS_TIE_BREAK_PALMS = 1.0

# Hands seen alongside the locked one are remembered this long after they were
# last seen.
OTHER_HAND_MEMORY_S = 0.6
# A detection this close to the locked hand is MediaPipe reporting it twice, not
# a second hand.
DUPLICATE_PALMS = 1.0

# A lock counts as seeing its hand if the hand was matched this recently.
SEEN_RECENTLY_S = 0.5


@dataclass(frozen=True, slots=True)
class HandObservation:
    """One hand found in one frame."""

    center_x: float  # palm centre, as fractions of the frame
    center_y: float
    handedness: str
    handedness_score: float
    curled_fingers: int
    stop_sign: bool
    aspect_ratio: float  # frame width / height
    # The palm centre as an angle fixed to the room: where the camera pointed
    # when the frame was exposed plus the palm's angle from the camera axis.
    pan_degrees: float
    tilt_degrees: float
    # Palm size in degrees, measured so that turning the hand barely changes it.
    size_degrees: float
    points: np.ndarray | None = field(default=None, compare=False)

    def image_distance_to(self, x: float, y: float) -> float:
        """Distance in the picture from a point given as frame fractions, in
        frame heights."""
        return math.hypot((self.center_x - x) * self.aspect_ratio, self.center_y - y)

    def degrees_to(self, pan_degrees: float, tilt_degrees: float) -> float:
        return math.hypot(self.pan_degrees - pan_degrees, self.tilt_degrees - tilt_degrees)


@dataclass(frozen=True, slots=True)
class LockUpdate:
    hand: HandObservation | None  # the locked hand, if it was seen this frame
    others: tuple[HandObservation, ...]  # every hand that is not being followed
    stop_progress: float  # 0..1 of the stop sign hold
    released_by_stop: bool  # the stop sign was held long enough this frame


@dataclass(frozen=True, slots=True)
class _OtherHand:
    pan_degrees: float
    tilt_degrees: float
    size_degrees: float
    seen_at: float


# Stands for the locked hand in the assignment, beside the others' indices.
_LOCKED = -1


class HandLock:
    def __init__(self, stop_hold_s: float = STOP_HOLD_S) -> None:
        self.stop_hold_s = stop_hold_s
        self.locked = False
        self._pan = 0.0
        self._tilt = 0.0
        self._size = 1.0
        self._votes: dict[str, float] = {}
        self._last_seen = 0.0
        self._stop_since: float | None = None
        self._stop_last_seen = 0.0
        self._stop_anchor: tuple[float, float] | None = None
        self._others: list[_OtherHand] = []

    @property
    def last_seen(self) -> tuple[float, float, float]:
        """Where the locked hand was last seen: pan and tilt degrees fixed to the
        room, and its palm size in degrees."""
        return self._pan, self._tilt, self._size

    def seen_recently(self, now: float) -> bool:
        return self.locked and now - self._last_seen <= SEEN_RECENTLY_S

    def out_of_sight_s(self, now: float) -> float:
        return max(0.0, now - self._last_seen) if self.locked else 0.0

    def release(self) -> None:
        self.locked = False
        self._votes = {}
        self._stop_since = None
        self._stop_anchor = None
        self._others = []

    def toggle(self, observations: list[HandObservation], now: float) -> str:
        """The r key. Releases a hand that is being followed; otherwise locks
        onto the hand nearest the centre of the picture. A locked hand that has
        gone out of sight is replaced rather than released, so one press is
        always enough to get following again."""
        if self.seen_recently(now):
            self.release()
            return "released the hand: gimbal holding"
        self.release()
        if not observations:
            return "no hand in view to lock onto"

        chosen = min(observations, key=lambda hand: hand.image_distance_to(0.5, 0.5))
        self.locked = True
        self._pan, self._tilt = chosen.pan_degrees, chosen.tilt_degrees
        self._size = chosen.size_degrees
        self._last_seen = now
        self._vote(chosen)
        self._others = [
            _OtherHand(hand.pan_degrees, hand.tilt_degrees, hand.size_degrees, now)
            for hand in observations
            if hand is not chosen and not self._duplicate_of(hand, chosen)
        ]
        others = len(observations) - 1
        return (f"locked onto the hand nearest the centre"
                + (f", ignoring {others} other" if others else "")
                + ": following")

    def find(self, observations: list[HandObservation], now: float) -> HandObservation | None:
        """The locked hand among these observations, if it is there. Changes
        nothing, so it can be asked before deciding whether to look harder."""
        return self._assign(observations, now)[0] if self.locked else None

    def update(self, observations: list[HandObservation], now: float) -> LockUpdate:
        if not self.locked:
            return LockUpdate(None, tuple(observations), 0.0, False)

        hand, other_matches = self._assign(observations, now)
        self._remember_others(observations, hand, other_matches, now)
        others = tuple(other for other in observations if other is not hand)

        if hand is None:
            if self._stop_since is not None and now - self._stop_last_seen > STOP_GRACE_S:
                self._stop_since = None
                self._stop_anchor = None
            return LockUpdate(None, others, self._stop_progress(now), False)

        # Follow the hand's drift in size (it moves nearer and further) and
        # position, and count its left/right call.
        self._pan, self._tilt = hand.pan_degrees, hand.tilt_degrees
        self._size += (hand.size_degrees - self._size) * SIZE_SMOOTHING
        self._last_seen = now
        self._vote(hand)

        if hand.stop_sign:
            drifted = (self._stop_anchor is not None and
                       hand.degrees_to(*self._stop_anchor) > STOP_MAX_DRIFT_PALMS * self._size)
            if self._stop_since is None or drifted:
                self._stop_since = now
                self._stop_anchor = (hand.pan_degrees, hand.tilt_degrees)
            self._stop_last_seen = now
            if now - self._stop_since >= self.stop_hold_s:
                self.release()
                return LockUpdate(hand, others, 1.0, True)
        elif self._stop_since is not None and now - self._stop_last_seen > STOP_GRACE_S:
            self._stop_since = None
            self._stop_anchor = None

        return LockUpdate(hand, others, self._stop_progress(now), False)

    def _stop_progress(self, now: float) -> float:
        if self._stop_since is None:
            return 0.0
        return min(1.0, (now - self._stop_since) / self.stop_hold_s)

    def _assign(self, observations: list[HandObservation],
                now: float) -> tuple[HandObservation | None, dict[int, HandObservation]]:
        """Share the observations out between the locked hand and the other
        hands being remembered, nearest pairs first, each used at most once.
        Distances are in palm lengths of the hand being matched."""
        out_of_sight = max(0.0, now - self._last_seen)
        radius = min(MAX_SEARCH_PALMS,
                     MATCH_DISTANCE_PALMS + SEARCH_GROWTH_PALMS_PER_S * out_of_sight)
        size_limit = SIZE_RATIO_LIMIT + SIZE_RATIO_GROWTH_PER_S * out_of_sight
        usual_side = self._usual_handedness()

        pairs: list[tuple[float, int, int]] = []
        for index, hand in enumerate(observations):
            palms = hand.degrees_to(self._pan, self._tilt) / self._size
            ratio = hand.size_degrees / self._size
            if palms <= radius and 1.0 / size_limit <= ratio <= size_limit:
                if (usual_side is not None and hand.handedness_score >= HANDEDNESS_CONFIDENT
                        and hand.handedness != usual_side):
                    palms += HANDEDNESS_TIE_BREAK_PALMS
                pairs.append((palms, _LOCKED, index))
            for slot, other in enumerate(self._others):
                other_palms = hand.degrees_to(other.pan_degrees, other.tilt_degrees) / other.size_degrees
                if other_palms <= MATCH_DISTANCE_PALMS:
                    pairs.append((other_palms, slot, index))

        pairs.sort()
        taken_hands: set[int] = set()
        taken_observations: set[int] = set()
        locked_hand: HandObservation | None = None
        other_matches: dict[int, HandObservation] = {}
        for _, owner, index in pairs:
            if owner in taken_hands or index in taken_observations:
                continue
            taken_hands.add(owner)
            taken_observations.add(index)
            if owner == _LOCKED:
                locked_hand = observations[index]
            else:
                other_matches[owner] = observations[index]
        return locked_hand, other_matches

    def _remember_others(self, observations: list[HandObservation],
                         hand: HandObservation | None,
                         other_matches: dict[int, HandObservation], now: float) -> None:
        remembered = []
        for slot, other in enumerate(self._others):
            match = other_matches.get(slot)
            if match is not None:
                other = _OtherHand(match.pan_degrees, match.tilt_degrees, match.size_degrees, now)
            elif now - other.seen_at > OTHER_HAND_MEMORY_S:
                continue
            if hand is not None and hand.degrees_to(other.pan_degrees, other.tilt_degrees) \
                    <= DUPLICATE_PALMS * hand.size_degrees:
                continue  # it has drifted onto the locked hand; the lock wins
            remembered.append(other)

        # A new hand counts as someone else's only when it is seen at the same
        # time as the locked hand. While the locked hand is out of sight, a hand
        # appearing may be it coming back.
        if hand is not None:
            matched = {id(match) for match in other_matches.values()}
            remembered.extend(
                _OtherHand(other.pan_degrees, other.tilt_degrees, other.size_degrees, now)
                for other in observations
                if other is not hand and id(other) not in matched
                and not self._duplicate_of(other, hand)
            )
        self._others = remembered

    @staticmethod
    def _duplicate_of(other: HandObservation, hand: HandObservation) -> bool:
        return other.degrees_to(hand.pan_degrees, hand.tilt_degrees) <= DUPLICATE_PALMS * hand.size_degrees

    def _vote(self, hand: HandObservation) -> None:
        """Older calls fade, so a hand MediaPipe keeps calling the other way
        for a while gets that as its usual side."""
        if hand.handedness_score < HANDEDNESS_CONFIDENT:
            return
        for side in self._votes:
            self._votes[side] *= 0.9
        self._votes[hand.handedness] = self._votes.get(hand.handedness, 0.0) + 1.0

    def _usual_handedness(self) -> str | None:
        if not self._votes:
            return None
        ranked = sorted(self._votes.items(), key=lambda item: item[1], reverse=True)
        top_side, top = ranked[0]
        runner_up = ranked[1][1] if len(ranked) > 1 else 0.0
        return top_side if top >= 2.0 and top >= 3.0 * runner_up else None
