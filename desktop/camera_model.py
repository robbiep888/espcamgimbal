"""Turn a position in the picture into an angle from the camera axis.

The board steers by angles, so the desktop has to know how many degrees a
pixel is. That is the lens's focal length, stored here as a fraction of the
frame's width and height so it holds at any stream resolution. Run
calibrate_fov.py to measure it on this camera; until then the default comes
from an early measurement on this rig (a 13.1 degree tilt moved the picture
120 px of 480).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
import pathlib

DEFAULT_PATH = pathlib.Path(__file__).with_name("camera_calibration.json")

# 120 px / tan(13.1 deg) = 515 px, over a 640 x 480 frame.
DEFAULT_FOCAL = 515.0 / 640.0
DEFAULT_FOCAL_Y = 515.0 / 480.0


@dataclass(frozen=True, slots=True)
class CameraModel:
    # Focal length as a fraction of frame width (for pan) and height (for tilt).
    focal_x: float = DEFAULT_FOCAL
    focal_y: float = DEFAULT_FOCAL_Y
    source: str = "default"

    def angles(self, center_x: float, center_y: float) -> tuple[float, float]:
        """Degrees right of and below the camera axis for a point given as
        fractions of the frame, matching the board's sign convention."""
        pan = math.degrees(math.atan((center_x - 0.5) / self.focal_x))
        tilt = math.degrees(math.atan((center_y - 0.5) / self.focal_y))
        return pan, tilt

    def position(self, pan_degrees: float, tilt_degrees: float) -> tuple[float, float]:
        """Where in the picture, as frame fractions, a direction this far from
        the camera axis appears: angles() run backwards. Directions behind the
        camera are clamped to the edge of view rather than wrapping around."""
        pan = math.radians(max(-89.0, min(89.0, pan_degrees)))
        tilt = math.radians(max(-89.0, min(89.0, tilt_degrees)))
        return 0.5 + self.focal_x * math.tan(pan), 0.5 + self.focal_y * math.tan(tilt)

    @property
    def field_of_view(self) -> tuple[float, float]:
        """Full horizontal and vertical field of view, in degrees."""
        return (
            2.0 * math.degrees(math.atan(0.5 / self.focal_x)),
            2.0 * math.degrees(math.atan(0.5 / self.focal_y)),
        )

    @classmethod
    def load(cls, path: str | os.PathLike[str] = DEFAULT_PATH) -> "CameraModel":
        try:
            data = json.loads(pathlib.Path(path).read_text())
        except FileNotFoundError:
            return cls()
        return cls(focal_x=float(data["focal_x"]), focal_y=float(data["focal_y"]),
                   source=str(path))

    def save(self, path: str | os.PathLike[str] = DEFAULT_PATH, **extra: object) -> None:
        data = {k: v for k, v in asdict(self).items() if k != "source"}
        data.update(extra)
        pathlib.Path(path).write_text(json.dumps(data, indent=2) + "\n")
