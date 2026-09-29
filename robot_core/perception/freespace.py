"""Which way is clear? Monocular free-space from sparse depth samples.

There is no lidar yet, so the camera + /v1/depth is the only look-ahead the
robot has. The service's default model (Depth Anything V2) returns RELATIVE
inverse depth — the numbers order correctly but are not metres — so nothing
here compares a value to a distance. Instead it uses a ratio that survives any
unknown scale:

    For each column, compare nearness at a FAR floor row (higher in the image)
    with nearness at a NEAR floor row (bottom of the image).

    open floor:   the far row really is farther  -> ratio well below 1
    obstacle:     a vertical surface fills both rows at about the same
                  distance                          -> ratio near (or above) 1

`nearness` is the model's value when metric is False (inverse depth: bigger =
closer) and 1/d when metric is True. The ratio is dimensionless either way.

Assumptions, stated because they are where this breaks:
  * the camera is roughly level or tilted down, so the sampled rows are floor
    when nothing is in the way (analyze() refuses when tilt is above +5 deg)
  * glass, mirrors and featureless walls confuse monocular depth
  * low obstacles below the near row are invisible. This is a steering hint,
    not a safety system — the lidar is (Phase 5).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from .geometry import CameraModel

Point = tuple[float, float]


@dataclass(frozen=True)
class Layout:
    """Where the sample points are and how they group."""
    points: list[Point]
    columns: list[float]             # u of each column, pixels
    far_row: float
    near_row: float
    rows: list[float]


@dataclass(frozen=True)
class Sector:
    bearing_deg: float               # camera frame, + = right
    ratio: float                     # far/near nearness; ~1 = wall
    clear: bool


@dataclass(frozen=True)
class FreeSpace:
    sectors: list[Sector]
    best_bearing_deg: Optional[float]   # camera frame; None if nothing is clear
    ahead_clear: Optional[bool]         # None = could not tell
    note: str = ""

    def summary(self) -> str:
        if self.ahead_clear is None:
            return f"can't tell ({self.note})" if self.note else "can't tell"
        ahead = "clear ahead" if self.ahead_clear else "blocked ahead"
        if self.best_bearing_deg is None:
            return f"{ahead}; no clear direction in view"
        side = "right" if self.best_bearing_deg > 0 else "left"
        if abs(self.best_bearing_deg) < 5:
            return f"{ahead}; clearest straight ahead"
        return f"{ahead}; clearest {abs(self.best_bearing_deg):.0f} deg {side}"


def grid(width: int, height: int, n_columns: int = 7,
         rows: Sequence[float] = (0.62, 0.76, 0.90),
         u_range: tuple[float, float] = (0.06, 0.94)) -> Layout:
    """Sample points across the lower part of the frame, row-major."""
    lo, hi = u_range
    cols = [width * (lo + (hi - lo) * i / (n_columns - 1)) for i in range(n_columns)]
    rs = [height * r for r in rows]
    pts = [(u, v) for v in rs for u in cols]
    return Layout(points=pts, columns=cols, far_row=rs[0], near_row=rs[-1], rows=rs)


def corridor(width: int, height: int, *, top_v: Optional[float] = None,
             half_width_frac: float = 0.16, n_columns: int = 3) -> Optional[Layout]:
    """Just the floor straight ahead, between the robot and `top_v`.

    Used while following: the person IS an obstacle, so the whole-frame test
    would always say "blocked". Sampling only the floor below their feet asks
    the right question — is anything between us?
    Returns None when there is too little floor visible to judge.
    """
    near = height * 0.92
    far = max(height * 0.60, (top_v + 0.04 * height) if top_v is not None else 0.0)
    if near - far < 0.10 * height:
        return None
    lo, hi = 0.5 - half_width_frac, 0.5 + half_width_frac
    return grid(width, height, n_columns, rows=(far / height, (far + near) / 2 / height, near / height),
                u_range=(lo, hi))


def analyze(depths: Sequence[float], metric: bool, layout: Layout, cam: CameraModel, *,
            tilt_deg: float = 0.0, block_ratio: float = 0.80,
            ahead_half_width_deg: float = 12.0, center_penalty: float = 0.004) -> FreeSpace:
    if tilt_deg > 5.0:
        return FreeSpace([], None, None, note="camera tilted up; floor not in view")
    n_cols = len(layout.columns)
    if len(depths) != n_cols * len(layout.rows):
        return FreeSpace([], None, None, note="depth sample count mismatch")

    def near(v: float) -> float:
        if metric:
            return 1.0 / v if v > 1e-6 else float("inf")
        return v

    far_vals = [near(d) for d in depths[:n_cols]]
    near_vals = [near(d) for d in depths[-n_cols:]]
    sectors = []
    for i, u in enumerate(layout.columns):
        nv = near_vals[i]
        ratio = far_vals[i] / nv if nv > 1e-9 else 1.0
        sectors.append(Sector(round(cam.bearing_deg(u), 1), round(ratio, 3), ratio < block_ratio))

    ahead = [s for s in sectors if abs(s.bearing_deg) <= ahead_half_width_deg] or \
            [min(sectors, key=lambda s: abs(s.bearing_deg))]
    ahead_clear = all(s.clear for s in ahead)
    clear = [s for s in sectors if s.clear]
    best = None
    if clear:
        # Most open, with a mild preference for not turning.
        best = min(clear, key=lambda s: s.ratio + center_penalty * abs(s.bearing_deg)).bearing_deg
    return FreeSpace(sectors, best, ahead_clear)
