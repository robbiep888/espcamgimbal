"""Find hands in a camera frame, keep hold of the locked one, and read its signs.

MediaPipe runs here on the computer rather than on the ESP32-S3. Landmark
inference needs far more compute and memory than the board has spare while it is
also capturing and streaming JPEGs, and the desktop is already receiving every
frame anyway. The board only ever receives where the followed hand is.

Which hand is followed, and when following stops, is decided in hand_lock.py.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import os
import pathlib
import time

import cv2
import numpy as np

from camera_model import CameraModel
from camera_pose import Pose
from hand_lock import DUPLICATE_PALMS, HandLock, HandObservation

# MediaPipe dropped the old mp.solutions.hands API. The Tasks API that replaced
# it keeps the model in a separate file rather than inside the wheel, so one has
# to be on disk before anything can run.
DEFAULT_MODEL = pathlib.Path(__file__).with_name("models") / "hand_landmarker.task"

# MediaPipe hand landmark indices.
WRIST = 0
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
FINGER_MCPS = (5, 9, 13, 17)  # index, middle, ring, pinky knuckles
FINGER_PIPS = (6, 10, 14, 18)
FINGER_TIPS = (8, 12, 16, 20)
MIDDLE_MCP = 9
MIDDLE_TIP = 12
PINKY_MCP = 17
PALM_POINTS = (0, 5, 9, 13, 17)  # wrist plus the four knuckles

# Palm length (wrist to middle knuckle) against knuckle span (index to little
# finger knuckle) on an adult hand. Tipping the fingers toward the camera
# foreshortens the first, turning the hand edge-on the second, and hardly any
# pose foreshortens both, so the larger of the two, scaled to match, is a palm
# size that stays put while the hand turns.
PALM_LENGTH_PER_KNUCKLE_SPAN = 1.4

# When the locked hand is missed, look again in a square this many palm lengths
# across, centred on where it should be. MediaPipe's palm detector sees the
# whole frame squeezed into 192 x 192 pixels, where a palm 1.2 m away is about
# a dozen pixels long and one 2 m away eight: landmark tracking can hold onto a
# hand that small, but once it lets go, the detector rarely finds it again. In this crop the same palm is two
# to three times larger. Measured on this Mac, the extra look costs 14 ms, about
# the same as the main pass, and only runs on frames that missed.
ZOOM_PALMS = 6.0
# Zooming in past the detector's own input size only shows it blur.
ZOOM_MIN_PIXELS = 192
# Keep looking this long after the locked hand was last seen.
ZOOM_SEARCH_S = 2.0

# Frames the detection-rate readout is averaged over: about three seconds.
STATS_FRAMES = 50


@dataclass(frozen=True, slots=True)
class HandReading:
    """The followed hand in one frame, and whether the gimbal should follow it.

    Coordinates are normalised against the whole frame so drawing is simple, but
    the proportional errors are measured against the region of interest. The
    region is the whole frame unless a lens whose image circle does not cover
    the sensor leaves the corners black: a hand can never be seen there, so the
    edge of the working area has to be pulled in to where it can.
    """

    visible: bool = False  # the locked hand was seen in this frame
    following: bool = False
    center_x: float = 0.5
    center_y: float = 0.5
    curled_fingers: int = 0
    confidence: float = 0.0
    points: np.ndarray | None = field(default=None, compare=False)
    # (x0, y0, x1, y1) as fractions of the frame. The default is the whole frame.
    roi: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)
    locked: bool = False  # a hand is locked, whether or not it was seen
    stop_progress: float = 0.0  # 0..1 of the stop sign hold
    stopped: bool = False  # the stop sign released the hand in this frame
    others: tuple[HandObservation, ...] = field(default=(), compare=False)

    @property
    def pan_error(self) -> float:
        """Positive when the hand is right of the ROI centre, in half-ROI units."""
        x0, _, x1, _ = self.roi
        half_width = max(1e-6, (x1 - x0) / 2.0)
        return (self.center_x - (x0 + x1) / 2.0) / half_width

    @property
    def tilt_error(self) -> float:
        """Positive when the hand is below the ROI centre (y grows downward)."""
        _, y0, _, y1 = self.roi
        half_height = max(1e-6, (y1 - y0) / 2.0)
        return (self.center_y - (y0 + y1) / 2.0) / half_height


def _distance(a: np.ndarray, b: np.ndarray, aspect_ratio: float) -> float:
    """Distance between two landmarks in frame heights. MediaPipe normalises x
    and y against different pixel counts, so x is rescaled first."""
    return math.hypot((a[0] - b[0]) * aspect_ratio, a[1] - b[1])


def count_curled_fingers(points: np.ndarray, aspect_ratio: float) -> int:
    """A finger is curled when its tip sits closer to the wrist than its middle
    joint does. The comparison is a ratio, so it holds at any distance from the
    camera."""
    wrist = points[WRIST]
    return sum(
        1
        for tip, pip in zip(FINGER_TIPS, FINGER_PIPS)
        if _distance(points[tip], wrist, aspect_ratio) < _distance(points[pip], wrist, aspect_ratio)
    )


def is_stop_sign(points: np.ndarray, aspect_ratio: float) -> bool:
    """A raised "stop": all four fingers straight and pointing up, the hand
    within 45 degrees of upright, and the thumb out from the palm.

    A hand held up open to be followed has this shape too, so hand_lock.py
    only counts a stop sign that is also held in place. A hand that is lying
    flat, pointing or folded fails at least one of these.
    """
    if count_curled_fingers(points, aspect_ratio) != 0:
        return False
    # Image y grows downward, so "up" is a smaller y.
    if any(points[tip][1] >= points[knuckle][1] for tip, knuckle in zip(FINGER_TIPS, FINGER_MCPS)):
        return False
    rise = points[WRIST][1] - points[MIDDLE_TIP][1]
    lean = abs(points[MIDDLE_TIP][0] - points[WRIST][0]) * aspect_ratio
    if rise <= lean:
        return False
    # An outstretched thumb's tip is further from the little finger's knuckle
    # than its own last joint is; a thumb folded across the palm is nearer.
    return (_distance(points[THUMB_TIP], points[PINKY_MCP], aspect_ratio)
            > _distance(points[THUMB_IP], points[PINKY_MCP], aspect_ratio))


def palm_size_degrees(points: np.ndarray, camera: CameraModel) -> float:
    """Palm length in degrees, from whichever of palm length and knuckle span
    the hand's pose foreshortens least."""
    wrist, index, middle, pinky = (camera.angles(*points[i]) for i in (WRIST, INDEX_MCP, MIDDLE_MCP, PINKY_MCP))
    length = math.hypot(middle[0] - wrist[0], middle[1] - wrist[1])
    span = math.hypot(pinky[0] - index[0], pinky[1] - index[1])
    return max(0.05, length, PALM_LENGTH_PER_KNUCKLE_SPAN * span)


def observe_hand(points: np.ndarray, handedness: str, handedness_score: float,
                 aspect_ratio: float, camera: CameraModel | None = None,
                 pose: Pose | None = None) -> HandObservation:
    """Summarise 21 landmarks, given as fractions of the whole frame, seen with
    the camera pointing at `pose` (home when not given)."""
    camera = camera if camera is not None else CameraModel()
    pose = pose if pose is not None else Pose()
    palm = points[list(PALM_POINTS)]
    center_x, center_y = float(palm[:, 0].mean()), float(palm[:, 1].mean())
    pan, tilt = camera.angles(center_x, center_y)
    return HandObservation(
        center_x=center_x,
        center_y=center_y,
        handedness=handedness,
        handedness_score=handedness_score,
        curled_fingers=count_curled_fingers(points, aspect_ratio),
        stop_sign=is_stop_sign(points, aspect_ratio),
        aspect_ratio=aspect_ratio,
        pan_degrees=pose.pan_degrees + pan,
        tilt_degrees=pose.tilt_degrees + tilt,
        size_degrees=palm_size_degrees(points, camera),
        points=points,
    )


def zoom_box(width: int, height: int, camera: CameraModel, pose: Pose | None,
             last_seen: tuple[float, float, float]) -> tuple[int, int, int, int]:
    """Pixel box (left, top, right, bottom) of a square around where a hand
    last seen at `last_seen` (room pan, tilt and palm size, in degrees) should
    appear with the camera at `pose`. Slid back inside the frame when that is
    near or past an edge, so the search looks as close as it can."""
    pose = pose if pose is not None else Pose()
    pan, tilt, size = last_seen
    x, y = camera.position(pan - pose.pan_degrees, tilt - pose.tilt_degrees)
    palm_pixels = math.tan(math.radians(size)) * camera.focal_y * height
    side = int(min(width, height, max(ZOOM_MIN_PIXELS, ZOOM_PALMS * palm_pixels)))
    left = int(min(max(0.0, x * width - side / 2), width - side))
    top = int(min(max(0.0, y * height - side / 2), height - side))
    return left, top, left + side, top + side


def _same_hand(a: HandObservation, b: HandObservation) -> bool:
    return a.degrees_to(b.pan_degrees, b.tilt_degrees) <= DUPLICATE_PALMS * max(a.size_degrees, b.size_degrees)


class HandTracker:
    """Wraps MediaPipe hand landmarks, the hand lock and centre smoothing."""

    def __init__(
        self,
        # The stream's full width. Inference cost is set by the model's fixed
        # input sizes, not by this: 320 and 640 both measured 13.7 ms per frame
        # here. What the larger picture buys is detail in the crop the landmark
        # model takes around a small, distant hand.
        inference_width: int = 640,
        # MediaPipe's own defaults. 0.6 was stricter than that for no measured
        # benefit: on this camera's view of the room, thresholds down to 0.3
        # found no phantom hands in whole frames or zoomed crops.
        detection_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
        smoothing: float = 0.58,
        model_path: str | os.PathLike[str] | None = None,
        roi_width: float = 1.0,
        roi_height: float = 1.0,
        num_hands: int = 2,
        camera: CameraModel | None = None,
        zoom_search: bool = True,
    ) -> None:
        try:
            import mediapipe as mp
            from mediapipe.tasks import python as mp_python
            from mediapipe.tasks.python import vision
        except ImportError as error:  # pragma: no cover - depends on the host
            raise RuntimeError(
                "mediapipe is not installed. Run:\n"
                "    python -m pip install -r desktop/requirements.txt\n"
                "or start the tracker with --no-track to watch the stream only."
            ) from error

        model = pathlib.Path(
            model_path or os.environ.get("HAND_LANDMARKER_MODEL") or DEFAULT_MODEL
        )
        if not model.is_file():
            raise RuntimeError(
                f"Hand landmark model not found at {model}.\n"
                "Download hand_landmarker.task from "
                "https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker"
                " and save it there, or point HAND_LANDMARKER_MODEL at a copy."
            )

        self._mp = mp

        def landmarker(mode: object) -> object:
            return vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=str(model)),
                running_mode=mode,
                # More than one, or a second hand in view could crowd out the
                # one being followed; the lock picks the right one out.
                num_hands=num_hands,
                min_hand_detection_confidence=detection_confidence,
                min_hand_presence_confidence=detection_confidence,
                min_tracking_confidence=tracking_confidence,
            ))

        # VIDEO mode carries tracking state between frames, so the model only
        # runs full detection when it has fewer hands than it is looking for.
        self._landmarker = landmarker(vision.RunningMode.VIDEO)
        # The zoomed second look is somewhere different every time, so it has
        # no tracking state worth carrying: IMAGE mode, and a separate instance
        # so it cannot disturb the main one's.
        self._zoom_landmarker = landmarker(vision.RunningMode.IMAGE) if zoom_search else None
        self.camera = camera if camera is not None else CameraModel()
        self._inference_width = inference_width
        self._smoothing = smoothing
        self._smoothed_center: tuple[float, float] | None = None
        self._last_observations: list[HandObservation] = []
        self.lock = HandLock()
        # (any hand seen, a hand locked, locked hand seen, seen only by zooming)
        self._frames: deque[tuple[bool, bool, bool, bool]] = deque(maxlen=STATS_FRAMES)
        # VIDEO mode rejects a timestamp that does not advance, so the clock is
        # kept here rather than trusting frame arrival times.
        self._last_timestamp_ms = -1
        self._roi_width = 1.0
        self._roi_height = 1.0
        self.set_roi(roi_width, roi_height)

    def set_roi(self, width_fraction: float, height_fraction: float) -> None:
        """Resize the working area, as fractions of the frame. Safe to call
        between frames, so the region can be dialled in against a live picture
        instead of being measured up front."""
        self._roi_width = min(1.0, max(0.2, width_fraction))
        self._roi_height = min(1.0, max(0.2, height_fraction))

    @property
    def roi_size(self) -> tuple[float, float]:
        """The working area as (width, height) fractions of the frame."""
        return (self._roi_width, self._roi_height)

    @property
    def roi(self) -> tuple[float, float, float, float]:
        """The working area as (x0, y0, x1, y1) fractions of the frame."""
        return (
            (1.0 - self._roi_width) / 2.0,
            (1.0 - self._roi_height) / 2.0,
            (1.0 + self._roi_width) / 2.0,
            (1.0 + self._roi_height) / 2.0,
        )

    def close(self) -> None:
        self._landmarker.close()
        if self._zoom_landmarker is not None:
            self._zoom_landmarker.close()

    def __enter__(self) -> "HandTracker":
        return self

    def __exit__(self, *_exception_details: object) -> None:
        self.close()

    def toggle_lock(self, now: float | None = None) -> str:
        """Lock onto the hand nearest the centre of the last frame, or release
        the one being followed. Returns what happened, for the console."""
        self._smoothed_center = None
        return self.lock.toggle(self._last_observations,
                                time.monotonic() if now is None else now)

    def detection_summary(self) -> str:
        """How reliably hands are being found, over the last few seconds."""
        frames = list(self._frames)
        if not frames:
            return "vision: no frames yet"
        found = sum(1 for any_hand, _, _, _ in frames if any_hand)
        text = f"vision: hands found in {100 * found / len(frames):.0f}% of frames"
        locked = [frame for frame in frames if frame[1]]
        if locked:
            seen = sum(1 for frame in locked if frame[2])
            text += f", locked hand in {100 * seen / len(locked):.0f}%"
            zoomed = sum(1 for frame in locked if frame[3])
            if zoomed:
                text += f" ({100 * zoomed / len(locked):.0f}% only by zooming in)"
        return text

    def _smooth(self, x: float, y: float) -> tuple[float, float]:
        if self._smoothed_center is None:
            self._smoothed_center = (x, y)
        else:
            previous_x, previous_y = self._smoothed_center
            alpha = self._smoothing
            self._smoothed_center = (
                previous_x + (x - previous_x) * alpha,
                previous_y + (y - previous_y) * alpha,
            )
        return self._smoothed_center

    def detect(self, bgr_image: np.ndarray, pose: Pose | None = None) -> list[HandObservation]:
        """Every hand in one frame. Call from a single thread only."""
        height, width = bgr_image.shape[:2]

        # Crop to the working area first; a no-op at the full-frame default.
        # When it is smaller, this matches the error scale to the part of the
        # frame a hand can actually appear in, keeps black vignetted corners
        # out of the model's input and makes the hand larger relative to what
        # it does see.
        roi = self.roi
        left = int(roi[0] * width)
        top = int(roi[1] * height)
        right = max(left + 1, int(roi[2] * width))
        bottom = max(top + 1, int(roi[3] * height))
        return self._detect_in(bgr_image, (left, top, right, bottom), pose, zoomed=False)

    def _detect_in(self, bgr_image: np.ndarray, box: tuple[int, int, int, int],
                   pose: Pose | None, zoomed: bool) -> list[HandObservation]:
        height, width = bgr_image.shape[:2]
        left, top, right, bottom = box
        cropped = bgr_image[top:bottom, left:right]
        crop_height, crop_width = cropped.shape[:2]

        # Only ever shrink, and only past the stream's own resolution: the
        # model resizes to its fixed input anyway, so this saves nothing below.
        scale = self._inference_width / float(crop_width)
        if scale < 1.0:
            small = cv2.resize(
                cropped,
                (self._inference_width, max(1, int(round(crop_height * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            small = cropped

        rgb = np.ascontiguousarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)

        if zoomed:
            result = self._zoom_landmarker.detect(image)
        else:
            timestamp_ms = max(int(time.monotonic() * 1000), self._last_timestamp_ms + 1)
            self._last_timestamp_ms = timestamp_ms
            result = self._landmarker.detect_for_video(image, timestamp_ms)

        observations = []
        for index, landmarks in enumerate(result.hand_landmarks):
            # Landmarks come back normalised against the crop; put them back
            # into whole-frame coordinates so drawing and the ROI maths agree.
            points = np.array([(point.x, point.y) for point in landmarks], dtype=np.float32)
            points = points * np.array(
                [crop_width / width, crop_height / height], dtype=np.float32
            ) + np.array([left / width, top / height], dtype=np.float32)

            handedness, score = "unknown", 0.0
            if index < len(result.handedness) and result.handedness[index]:
                category = result.handedness[index][0]
                handedness, score = category.category_name, float(category.score)
            observations.append(
                observe_hand(points, handedness, score, width / height, self.camera, pose)
            )
        return observations

    def process(self, bgr_image: np.ndarray, now: float | None = None,
                pose: Pose | None = None) -> HandReading:
        """Find the locked hand in one frame, taken with the camera pointing at
        `pose`. Call from a single thread only."""
        height, width = bgr_image.shape[:2]
        if width <= 0 or height <= 0:
            return HandReading()
        now = time.monotonic() if now is None else now

        observations = self.detect(bgr_image, pose)
        zoomed: list[HandObservation] = []
        if (self._zoom_landmarker is not None and self.lock.locked
                and self.lock.out_of_sight_s(now) <= ZOOM_SEARCH_S
                and self.lock.find(observations, now) is None):
            box = zoom_box(width, height, self.camera, pose, self.lock.last_seen)
            zoomed = [hand for hand in self._detect_in(bgr_image, box, pose, zoomed=True)
                      if not any(_same_hand(hand, seen) for seen in observations)]
            observations = observations + zoomed

        self._last_observations = observations
        was_locked = self.lock.locked
        update = self.lock.update(observations, now)
        hand = update.hand
        self._frames.append((
            bool(observations),
            was_locked,
            hand is not None,
            hand is not None and any(hand is extra for extra in zoomed),
        ))
        if hand is None:
            # Forget the smoothed centre so the hand is not re-acquired at a
            # stale position when it comes back somewhere else.
            self._smoothed_center = None
            return HandReading(roi=self.roi, locked=self.lock.locked,
                               stop_progress=update.stop_progress,
                               stopped=update.released_by_stop, others=update.others)

        center_x, center_y = self._smooth(hand.center_x, hand.center_y)
        return HandReading(
            visible=True,
            following=self.lock.locked,
            center_x=center_x,
            center_y=center_y,
            curled_fingers=hand.curled_fingers,
            confidence=hand.handedness_score,
            points=hand.points,
            roi=self.roi,
            locked=self.lock.locked,
            stop_progress=update.stop_progress,
            stopped=update.released_by_stop,
            others=update.others,
        )


def draw_hand(image: np.ndarray, reading: HandReading) -> None:
    """Overlay the hands. The followed hand is green, a locked hand that has
    just been released amber, and every ignored hand grey. A ring around the
    followed hand fills while a stop sign is held."""
    height, width = image.shape[:2]
    scale = np.array([width, height], dtype=np.float32)

    for other in reading.others:
        if other.points is not None:
            for x, y in (other.points * scale).astype(int):
                cv2.circle(image, (int(x), int(y)), 2, (140, 140, 140), -1, cv2.LINE_AA)

    if not reading.visible or reading.points is None:
        return

    color = (80, 255, 80) if reading.following else (60, 190, 255)
    for x, y in (reading.points * scale).astype(int):
        cv2.circle(image, (int(x), int(y)), 3, color, -1, cv2.LINE_AA)

    center = (int(reading.center_x * width), int(reading.center_y * height))
    cv2.circle(image, center, 14, color, 2, cv2.LINE_AA)
    cv2.line(image, (center[0] - 20, center[1]), (center[0] + 20, center[1]), color, 1)
    cv2.line(image, (center[0], center[1] - 20), (center[0], center[1] + 20), color, 1)
    if reading.stop_progress > 0.0:
        cv2.ellipse(image, center, (26, 26), -90, 0, 360 * reading.stop_progress,
                    (60, 190, 255), 3, cv2.LINE_AA)
