"""Where the camera pointed when a frame was exposed, on the desktop's clock.

The hand lock recognises the locked hand from frame to frame by where it is. In
the picture, that position changes for two reasons: the hand moved, or the
gimbal turned. The second is the big one. Predictive tracking swings the pan
axis at up to 150 deg/s on the way to a hand, several palm widths of picture per
frame, so a lock that matched hands by picture position lost its hand exactly
when the gimbal was busy following it -- and the gimbal stopped, because a lost
hand grants no motion.

So the lock works in angles fixed to the room instead: where the camera pointed
when the frame was exposed, plus where in the picture the hand was. The board
does the same sum to steer; this is the desktop's copy of it.

Two clocks meet here. Frames carry the board's capture time. The board's pan and
tilt arrive in the control replies, stamped with the desktop's clock on arrival.
BoardClock maps one onto the other and PoseHistory answers "where was the camera
at this moment".
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading


@dataclass(frozen=True, slots=True)
class Pose:
    """Degrees from home: pan positive toward a hand right of centre, tilt
    positive toward a hand below it -- the sense the board steers in."""

    pan_degrees: float = 0.0
    tilt_degrees: float = 0.0


class BoardClock:
    """Maps the board's capture timestamps onto time.monotonic().

    A frame always arrives some time after it was captured, never before, so the
    smallest (arrival - capture) seen so far is the offset between the clocks
    plus the quickest delivery Wi-Fi managed. Only that lower edge is used: a
    frame that was held up in transit can never drag the estimate later.

    The estimate is allowed to creep later very slowly in case the two crystals
    drift apart, and it starts again if the board reboots, which restarts its
    clock from zero.
    """

    # Crystal drift is parts per million; this is a thousand times that.
    CREEP_S_PER_S = 0.001
    # A frame this much later than the estimate means the board's clock restarted.
    REBOOT_JUMP_S = 1.0

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._offset: float | None = None
        self._last_arrival = 0.0

    def observe(self, captured_at_us: int, arrived_at: float) -> None:
        sample = arrived_at - captured_at_us / 1e6
        with self._lock:
            if self._offset is None or sample - self._offset > self.REBOOT_JUMP_S:
                self._offset = sample
            else:
                crept = self._offset + self.CREEP_S_PER_S * max(0.0, arrived_at - self._last_arrival)
                self._offset = min(sample, crept)
            self._last_arrival = arrived_at

    def to_desktop(self, board_us: int) -> float | None:
        """The desktop time at which the board's clock read board_us, or None
        before the first frame has arrived."""
        with self._lock:
            return None if self._offset is None else board_us / 1e6 + self._offset


class PoseHistory:
    """The gimbal's reported angles against the desktop clock.

    The board replies to every command, a hundred times a second, so two
    seconds of records reach back well past the oldest frame worth matching.
    """

    def __init__(self, capacity: int = 256) -> None:
        self._lock = threading.Lock()
        self._records: deque[tuple[float, float, float]] = deque(maxlen=capacity)

    def record(self, at: float, pan_degrees: float, tilt_degrees: float) -> None:
        with self._lock:
            if self._records and at < self._records[-1][0]:
                return  # the desktop clock is monotonic; ignore anything else
            self._records.append((at, pan_degrees, tilt_degrees))

    def clear(self) -> None:
        with self._lock:
            self._records.clear()

    def at(self, when: float) -> Pose | None:
        """Where the gimbal pointed at `when`, interpolated between reports.

        Past the newest report the newest is returned and before the oldest the
        oldest, as the nearest thing known: a stale angle is far closer to the
        truth than none. None only when nothing has been reported yet."""
        with self._lock:
            if not self._records:
                return None
            newest = self._records[-1]
            if when >= newest[0]:
                return Pose(newest[1], newest[2])
            later = newest
            # Frames are a tenth of a second old, so this walks back ~10 records.
            for earlier in reversed(self._records):
                if earlier[0] <= when:
                    span = later[0] - earlier[0]
                    fraction = (when - earlier[0]) / span if span > 0 else 0.0
                    return Pose(
                        earlier[1] + (later[1] - earlier[1]) * fraction,
                        earlier[2] + (later[2] - earlier[2]) * fraction,
                    )
                later = earlier
            oldest = self._records[0]
            return Pose(oldest[1], oldest[2])
