"""What is the floor ahead: flat, a step the tracks can climb, something in the way, or a drop.

One pure function over points the head measured (`Head.floor_profile`): no
hardware, no ROS. Each point is (x, y, h): metres ahead of the robot, metres to
its left, and height above the floor the robot stands on.

    flat       nothing higher than the noise, no hole
    climbable  a rise of at most h_max_m with flat ground after it (a door sill, a
               rug edge, a low platform)
    blocked    a rise higher than that, or one with no landing (a wall, a box, stairs up)
    drop       the floor falls away by more than max_drop_m (stairs down, a table edge)
    unknown    too little of the floor was seen to say (dark or shiny floor)

A point whose height is None is a ray that should have met the floor there and
came back with nothing. Beyond floor that WAS seen, that is a drop too deep to see
the bottom of (a stairwell); with no floor seen at all it is `unknown`.
`may_drive` is the one rule for what the robot drives onto: flat floor, and an
unknown one only if robot.yaml says so (drive_on_unknown_floor).

The numbers in `Params` are robot.yaml's (h_max_m, min_landing_m, max_drop_m);
h_max_m is the one to measure (plan step 0: boards of 1 to 4 cm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

FLAT, CLIMBABLE, BLOCKED, DROP, UNKNOWN = "flat", "climbable", "blocked", "drop", "unknown"

Point = Tuple[float, float, Optional[float]]     # x ahead, y left, height (None: no return where the floor should be)


@dataclass
class Params:
    h_max_m: float = 0.03          # highest step the tracks climb (MEASURE)
    min_landing_m: float = 0.20    # flat ground needed after a step
    max_drop_m: float = 0.03       # a fall deeper than this is a drop
    flat_tol_m: float = 0.015      # sensor noise: anything lower is floor
    corridor_m: float = 0.30       # width of the path that is checked (robot width plus a margin)
    near_m: float = 0.15           # nearer than this the sensor cannot see the floor
    cell_m: float = 0.05           # the path is judged in slices this long
    drive_unknown: bool = False    # drive onto a floor the sensor cannot read
    body_m: float = 0.30           # the robot's height: anything lower than this is in its way
    stop_short_m: float = 0.25     # a drive cut short by something ends with the robot's middle this far
                                   # before it (half the track length plus a gap; at least 0.25)
    width_m: float = 0.25          # the robot's width (local_plan keeps half of it, plus clearance_m, from hazards)
    clearance_m: float = 0.08      # gap between the robot's side and a hazard when planning a way round
    holes_are_drops: bool = True   # no return where the floor should be, beyond seen floor = a drop; False:
                                   # floor not seen there (driven up to, never onto; not planned round)
    auto_level: bool = False       # level each floor scan on the floor right in front (see level())
    detect_drops: bool = True      # detect drops from ToF when no dedicated TF-Luna drop guard is fitted
    outdoor_mode: bool = False     # uneven ground (grass/mulch/slopes): fits local slope plane & widens roughness tol
    max_slope_deg: float = 12.0    # steepest ground slope (pitch or roll) allowed in outdoor_mode before BLOCKED
    roughness_tol_m: float = 0.04  # surface roughness (grass tufts, mulch, gravel) tolerated in outdoor_mode


@dataclass(frozen=True)
class Terrain:
    kind: str                           # flat | climbable | blocked | drop | unknown
    distance_m: Optional[float] = None  # where it starts, ahead of the robot
    height_m: Optional[float] = None    # how high (or, for a drop, how deep: negative)

    @property
    def clear(self) -> bool:
        return self.kind == FLAT


def _high(values: Sequence[float]) -> float:
    """Near the top of a slice, but not its single highest reading."""
    s = sorted(values)
    return s[min(len(s) - 1, int(0.8 * len(s)))]


def _low(values: Sequence[float]) -> float:
    """Near the bottom of a slice, ignoring a single stray low reading."""
    s = sorted(values)
    return s[min(len(s) - 1, int(0.2 * len(s)))]


def classify(points: Sequence[Point], p: Params = Params(), reach_m: float = 0.5) -> Terrain:
    """The first thing that matters within `reach_m` ahead (how far the robot means to drive).

    The path is cut into slices `cell_m` long. The sensor's rows land further and
    further apart with distance, so most slices beyond half a metre are empty:
    only slices with readings are judged, and what lies between two of them is
    taken to be like the nearer one."""
    if p.outdoor_mode or p.auto_level:
        leveled, corr = level(points, max_deg=max(30.0, p.max_slope_deg))
        if corr is not None:
            _, slope_p, slope_r = corr
            if p.outdoor_mode and (abs(slope_p) > p.max_slope_deg or abs(slope_r) > p.max_slope_deg):
                return Terrain(BLOCKED, p.near_m, None)
            if abs(slope_p) <= p.max_slope_deg and abs(slope_r) <= p.max_slope_deg:
                points = leveled
    flat_tol = max(p.flat_tol_m, p.roughness_tol_m) if p.outdoor_mode else p.flat_tol_m
    cells: Dict[int, List[float]] = {}
    holes: Dict[int, int] = {}                       # slices where the floor should be and nothing came back
    for x, y, h in points:
        if abs(y) <= p.corridor_m / 2.0 and x >= p.near_m:
            i = int((x + 1e-9) / p.cell_m)
            if h is None:
                holes[i] = holes.get(i, 0) + 1
            else:
                cells.setdefault(i, []).append(h)
    last_i = int((reach_m + 1e-9) / p.cell_m)
    seen = sorted(cells)
    if sum(1 for i in seen if i <= last_i + 1) < 2:
        return Terrain(UNKNOWN)

    if p.detect_drops:
        # When TF-Luna is not fitted, ToF is the only drop guard: catch both visible drops
        # below -max_drop_m and multi-zone missing-return holes beyond seen floor (stairwells).
        drop_i = next((i for i in seen if i <= last_i and _low(cells[i]) < -p.max_drop_m), None)
        if drop_i is not None:
            return Terrain(DROP, drop_i * p.cell_m, _low(cells[drop_i]))
        if p.holes_are_drops and seen:
            hole_i = next((i for i in sorted(holes) if seen[0] < i <= last_i and holes[i] >= 2 and i not in cells), None)
            if hole_i is not None:
                return Terrain(DROP, hole_i * p.cell_m, None)

    rise = next((i for i in seen if i <= last_i and _high(cells[i]) > flat_tol), None)
    if rise is None:
        return Terrain(FLAT)
    landing_end = rise + int(math.ceil(p.min_landing_m / p.cell_m)) + 1
    over = [i for i in seen if rise <= i <= landing_end]
    top = max(_high(cells[i]) for i in over)
    if top > max(p.h_max_m, flat_tol) or len(over) < 2:
        return Terrain(BLOCKED, rise * p.cell_m, top)      # too high, or what follows cannot be seen
    return Terrain(CLIMBABLE, rise * p.cell_m, top)


LEVEL_REACH_M = 0.9       # the floor the ground plane is fitted to: right in front of the robot
LEVEL_INLIER_M = 0.02     # a point this close to the plane is floor
LEVEL_MAX_M = 0.07        # corrections bigger than these mean the fit found something else (a ramp,
LEVEL_MAX_DEG = 12.0      # a box filling the view): the scan is left as it was


def level(pts: Sequence[Point], max_deg: float = LEVEL_MAX_DEG) -> Tuple[List[Point], Optional[Tuple[float, float, float]]]:
    """Heights measured from the floor the robot stands on, whatever the head's real angle.

    The SG90 tilt servo stops a few degrees from where it was sent (backlash, the head's
    weight) and the chassis rocks after a drive, so the same flat floor reads sloped one
    way on one scan and the other way on the next. The floor right in front is where the
    robot is about to drive and almost always floor: fit a plane h = a + b*x + c*y to the
    points near it (refitting on the points close to the plane, so a box or a step does
    not count) and measure every height from that plane. Returns the points and
    (offset m, pitch deg, roll deg) of the correction, or None when there was too little
    floor to fit or the correction would be too large to believe."""
    cand = [(x, y, h) for x, y, h in pts if h is not None and x <= LEVEL_REACH_M and abs(h) <= 0.35]
    if len(cand) < 8 or max(q[0] for q in cand) - min(q[0] for q in cand) < 0.15:
        return list(pts), None
    use, fit = cand, None
    for _ in range(3):
        fit = _plane(use, roll=max(q[1] for q in use) - min(q[1] for q in use) >= 0.10)
        if fit is None:
            return list(pts), None
        keep = [q for q in cand if abs(q[2] - (fit[0] + fit[1] * q[0] + fit[2] * q[1])) <= LEVEL_INLIER_M]
        if len(keep) < 8:
            break
        use = keep
    a, b, c = fit
    pitch, roll = math.degrees(math.atan(b)), math.degrees(math.atan(c))
    if abs(a) > LEVEL_MAX_M or abs(pitch) > max_deg or abs(roll) > max_deg:
        return list(pts), None
    return [(x, y, None if h is None else h - (a + b * x + c * y)) for x, y, h in pts], (a, pitch, roll)


def _plane(pts, roll: bool = True) -> Optional[Tuple[float, float, float]]:
    """Least squares h = a + b*x (+ c*y), solving the normal equations by hand (no numpy)."""
    rows = [(1.0, x, y if roll else 0.0, h) for x, y, h in pts]
    m = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    v = [sum(r[i] * r[3] for r in rows) for i in range(3)]
    if not roll:
        m[2][2] = 1.0                                    # c fixed at 0: too narrow a view to judge roll
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if abs(det) < 1e-12:
        return None
    out = []
    for k in range(3):                                   # Cramer's rule
        mk = [row[:] for row in m]
        for i in range(3):
            mk[i][k] = v[i]
        out.append((mk[0][0] * (mk[1][1] * mk[2][2] - mk[1][2] * mk[2][1])
                    - mk[0][1] * (mk[1][0] * mk[2][2] - mk[1][2] * mk[2][0])
                    + mk[0][2] * (mk[1][0] * mk[2][1] - mk[1][1] * mk[2][0])) / det)
    return out[0], out[1], out[2]


def may_drive(t: Terrain, p: Params = Params()) -> bool:
    """The one rule for both follow and explore: flat floor only. A low step waits
    for the climb controller; an unreadable floor needs `drive_unknown`."""
    return t.kind == FLAT or (t.kind == UNKNOWN and p.drive_unknown)


def why_not(t: Terrain) -> str:
    """A few words for the screen and the log."""
    where = f" at {t.distance_m:.2f} m" if t.distance_m is not None else ""
    return {DROP: "drop", CLIMBABLE: "low step", UNKNOWN: "floor not seen"}.get(t.kind, "blocked") + where


@dataclass(frozen=True)
class Column:
    """One column of the grid (one bearing), as readings ordered near to far:
    (distance from the sensor, x, y, height or None) in base_link."""
    readings: Tuple[Tuple[float, float, float, Optional[float]], ...]


@dataclass
class Slice:
    """What one look from the level head adds to the map, in base_link."""
    rays: List[Tuple[float, float]]            # (x, y) of the nearest thing in the robot's way, per bearing
    low: List[Tuple[float, float]]             # those of them that are low (under knee height): furniture legs, sills
    drops: List[Tuple[float, float]]           # where the floor falls away
    clear: List[Tuple[float, float]] = None    # per bearing with nothing in the way: the furthest floor SEEN
                                               # (so it is free up to there; no guess from a missing return)

    def __post_init__(self) -> None:
        self.clear = [] if self.clear is None else self.clear


def scan_slice(columns: Sequence[Column], p: Params = Params(), low_m: float = 0.12) -> Slice:
    """Turn one grid into map data. Per bearing: the nearest reading between the
    floor and the robot's own height is an obstacle (what is higher, it drives
    under). No drops: those are the TF-Luna's (`drops` stays empty)."""
    out = Slice([], [], [])
    for col in columns:
        floor_at, hit = None, False
        for d, x, y, h in col.readings:
            if h is None:
                continue
            margin = 2.0 * p.flat_tol_m + 0.02 * d             # a tilt error grows with distance
            # In the way: lower than the robot is tall. The level row climbs with distance
            # (a wall 3 m off is met half a metre up), and that is still the wall.
            if margin < h < p.body_m + 0.08 * d:
                out.rays.append((x, y))
                if h < low_m:
                    out.low.append((x, y))
                hit = True
                break
            if p.detect_drops and h < -(margin + p.max_drop_m):
                out.drops.append((x, y))
                hit = True
                break
            if abs(h) <= margin:
                floor_at = (x, y)
        if not hit and floor_at is not None:
            out.clear.append(floor_at)
    return out


def params_from(p: dict) -> Params:
    """Params from robot.yaml's /explorer section; anything missing keeps its default.
    Enables ToF drop detection automatically when `tfluna_fitted` is False."""
    d = Params()
    width = float(p.get("robot_width_m", d.width_m))
    corridor = width + 0.05 if "robot_width_m" in p else float(p.get("path_width_m", d.corridor_m))
    detect_drops = bool(p.get("tof_drop_guard", not bool(p.get("tfluna_fitted", True))))
    return Params(width_m=width, corridor_m=corridor,
                  clearance_m=float(p.get("plan_clearance_m", d.clearance_m)),
                  holes_are_drops=bool(p.get("floor_holes_are_drops", d.holes_are_drops)),
                  auto_level=bool(p.get("floor_auto_level", d.auto_level)),
                  detect_drops=detect_drops,
                  outdoor_mode=bool(p.get("outdoor_mode", d.outdoor_mode)),
                  max_slope_deg=float(p.get("max_slope_deg", d.max_slope_deg)),
                  roughness_tol_m=float(p.get("roughness_tol_m", d.roughness_tol_m)),
                  h_max_m=float(p.get("h_max_m", d.h_max_m)),
                  flat_tol_m=float(p.get("flat_tol_m", d.flat_tol_m)),
                  min_landing_m=float(p.get("min_landing_m", d.min_landing_m)),
                  max_drop_m=float(p.get("max_drop_m", d.max_drop_m)),
                  drive_unknown=bool(p.get("drive_on_unknown_floor", d.drive_unknown)),
                  body_m=float(p.get("robot_height_m", d.body_m)),
                  stop_short_m=max(0.25, float(p.get("stop_short_m", d.stop_short_m))))
