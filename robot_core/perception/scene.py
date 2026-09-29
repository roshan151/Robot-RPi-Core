"""Turning tracks and detections into what the behaviors and the agent need:
body-frame TargetViews, and one short sentence per scene.
"""

from __future__ import annotations

from collections import Counter
from typing import Callable, Iterable, Optional

from .behavior import TargetView
from .geometry import CameraModel, to_base_bearing
from .tracker import Track

PERSON_AIM_FRAC = 0.25        # aim the head a quarter of the way down a person box


def target_views(tracks: Iterable[Track], cam: CameraModel, pan_deg: float, tilt_deg: float,
                 identity: Callable[[int], Optional[str]]) -> list[TargetView]:
    """Tracks (in `cam` pixels, current frame) -> body-frame views."""
    out = []
    area = float(cam.width * cam.height)
    for t in tracks:
        x1, y1, x2, y2 = t.bbox
        u = (x1 + x2) / 2.0
        v = y1 + (y2 - y1) * (PERSON_AIM_FRAC if t.cls == "person" else 0.5)
        out.append(TargetView(
            track_id=t.id, cls=t.cls, identity=identity(t.id),
            bearing_deg=round(to_base_bearing(cam.bearing_deg(u), pan_deg), 1),
            aim_elevation_deg=round(tilt_deg + cam.elevation_deg(v), 1),
            distance_m=cam.distance_m(t.bbox, t.cls),
            lost=t.lost, anchor_t=t.last_anchor_t,
            bottom_v_frac=round(y2 / cam.height, 3),
            area_frac=round((x2 - x1) * (y2 - y1) / area, 4),
            confidence=t.confidence,
        ))
    return out


def _a(noun: str) -> str:
    return ("an " if noun[:1] in "aeiou" else "a ") + noun


def _where(bearing: float) -> str:
    a = abs(bearing)
    if a < 8:
        return "straight ahead"
    side = "right" if bearing > 0 else "left"
    if a < 25:
        return f"slightly {side}"
    return f"{a:.0f} degrees {side}"


def describe(views: list[TargetView], *, max_items: int = 6) -> str:
    """One sentence for the agent: what, where, how far, who."""
    views = [v for v in views if not v.lost]
    if not views:
        return "I don't see anything I recognise."
    views = sorted(views, key=lambda v: -v.area_frac)[:max_items]
    parts = []
    for v in views:
        name = v.identity if v.identity and v.identity != "unknown" else (
            "a person I don't know" if v.cls == "person" and v.identity == "unknown" else _a(v.cls))
        dist = f" about {v.distance_m:.1f} m away" if v.distance_m else ""
        parts.append(f"{name}{dist}, {_where(v.bearing_deg)}")
    counts = Counter(v.cls for v in views)
    head = ", ".join(f"{n} {c}{'s' if n > 1 else ''}" for c, n in counts.most_common())
    return f"I see {head}: " + "; ".join(parts) + "."
