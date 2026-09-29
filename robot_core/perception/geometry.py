"""Camera geometry: pixels -> angles, bbox size -> rough distance, IOU.

Conventions, used everywhere in this package:
  * bearings in DEGREES, positive = RIGHT (matches the turn() tool and the
    drivetrain's Turn action: positive degrees turn right)
  * tilt positive = UP
  * bboxes are (x1, y1, x2, y2) in the pixel space of the image they came from

Distance from bbox size is a pinhole estimate from an assumed real-world size.
It is what stands in for lidar range until the LD14P is fitted (Part 3): fine
for "about 2 m, keep going", not for anything that must not touch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

BBox = tuple[float, float, float, float]

# Rough physical heights (m) of COCO classes worth approaching. Only used for
# the pinhole distance estimate; unknown classes get no distance.
CLASS_HEIGHT_M = {
    "person": 1.70, "chair": 0.90, "couch": 0.85, "potted plant": 0.50,
    "dining table": 0.75, "bed": 0.60, "tv": 0.55, "laptop": 0.25,
    "bottle": 0.25, "cup": 0.10, "dog": 0.55, "cat": 0.30, "backpack": 0.45,
    "suitcase": 0.65, "refrigerator": 1.75, "toilet": 0.75, "bicycle": 1.00,
    "sports ball": 0.22, "teddy bear": 0.30, "book": 0.22, "vase": 0.30,
}
PERSON_SHOULDER_M = 0.45       # used when a person's box is cut by the frame edge


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    hfov_deg: float

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def fy(self) -> float:
        return self.fx        # square pixels

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan((self.height / 2.0) / self.fy))

    def scaled(self, width: int, height: int) -> "CameraModel":
        """Same lens at another resolution (same aspect ratio assumed)."""
        return CameraModel(width, height, self.hfov_deg)

    # ---------------------------------------------------------------- angles

    def bearing_deg(self, u: float) -> float:
        """Horizontal angle of pixel column u from the optical axis, + = right."""
        return math.degrees(math.atan2(u - self.width / 2.0, self.fx))

    def elevation_deg(self, v: float) -> float:
        """Vertical angle of pixel row v from the optical axis, + = up."""
        return -math.degrees(math.atan2(v - self.height / 2.0, self.fy))

    def bbox_bearing_deg(self, bbox: BBox) -> float:
        return self.bearing_deg((bbox[0] + bbox[2]) / 2.0)

    # -------------------------------------------------------------- distance

    def distance_m(self, bbox: BBox, cls: str) -> Optional[float]:
        """Pinhole estimate from the class's typical size. None if unknown.

        A box touching the top or bottom edge is truncated, so its height
        under-reports the object and the estimate would read too FAR — the
        dangerous direction. For people that case falls back to shoulder width;
        for anything else there is no estimate.
        """
        x1, y1, x2, y2 = bbox
        h_px, w_px = max(1.0, y2 - y1), max(1.0, x2 - x1)
        edge = 2.0
        cut_vertically = y1 <= edge or y2 >= self.height - edge
        if cls == "person":
            if not cut_vertically:
                return round(CLASS_HEIGHT_M["person"] * self.fy / h_px, 2)
            cut_sideways = x1 <= edge or x2 >= self.width - edge
            if cut_sideways:
                return None
            return round(PERSON_SHOULDER_M * self.fx / w_px, 2)
        real = CLASS_HEIGHT_M.get(cls)
        if real is None or cut_vertically:
            return None
        return round(real * self.fy / h_px, 2)


def to_base_bearing(cam_bearing_deg: float, pan_deg: float) -> float:
    """Camera-frame bearing -> robot-body bearing, given the head's pan at capture."""
    return wrap_deg(cam_bearing_deg + pan_deg)


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def scale_bbox(bbox: BBox, from_wh: tuple[int, int], to_wh: tuple[int, int]) -> BBox:
    sx, sy = to_wh[0] / from_wh[0], to_wh[1] / from_wh[1]
    return (bbox[0] * sx, bbox[1] * sy, bbox[2] * sx, bbox[3] * sy)


def clip_bbox(bbox: BBox, wh: tuple[int, int]) -> BBox:
    w, h = wh
    x1, y1, x2, y2 = bbox
    return (min(max(x1, 0.0), w), min(max(y1, 0.0), h), min(max(x2, 0.0), w), min(max(y2, 0.0), h))


def iou(a: BBox, b: BBox) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    area = lambda r: max(0.0, r[2] - r[0]) * max(0.0, r[3] - r[1])  # noqa: E731
    return inter / (area(a) + area(b) - inter)


def head_region(bbox: BBox, wh: tuple[int, int], frac: float = 0.35) -> BBox:
    """The top part of a person box, padded — what gets sent for a face embed."""
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    pad = 0.15 * w
    return clip_bbox((x1 - pad, y1 - 0.05 * h, x2 + pad, y1 + frac * h), wh)


def median(values: Sequence[float]) -> float:
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
