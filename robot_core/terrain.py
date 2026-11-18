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
import statistics
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


def classify(points: Sequence[Point], p: Params = Params(), reach_m: float = 0.5) -> Terrain:
    """The first thing that matters within `reach_m` ahead (how far the robot means to drive).

    The path is cut into slices `cell_m` long. The sensor's rows land further and
    further apart with distance, so most slices beyond half a metre are empty:
    only slices with readings are judged, and what lies between two of them is
    taken to be like the nearer one."""
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

    # A drop. Either the rays that should have met the floor carry on and land lower
    # and further away, or they meet nothing at all. It starts somewhere after the
    # last slice that was still floor.
    prev = None
    for i in sorted(set(seen) | set(holes)):
        gone = holes.get(i, 0) > len(cells.get(i, ()))
        depth = statistics.median(cells[i]) if i in cells else None
        if (depth is not None and depth < -p.max_drop_m) or (gone and prev is not None):
            start = i if prev is None else prev + 1
            if start <= last_i:
                return Terrain(DROP, start * p.cell_m, depth)
            break
        if i in cells:
            prev = i

    rise = next((i for i in seen if i <= last_i and _high(cells[i]) > p.flat_tol_m), None)
    if rise is None:
        return Terrain(FLAT)
    landing_end = rise + int(math.ceil(p.min_landing_m / p.cell_m)) + 1
    over = [i for i in seen if rise <= i <= landing_end]
    top = max(_high(cells[i]) for i in over)
    if top > p.h_max_m or len(over) < 2:
        return Terrain(BLOCKED, rise * p.cell_m, top)      # too high, or what follows cannot be seen
    return Terrain(CLIMBABLE, rise * p.cell_m, top)


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
    under); a reading below the floor is a drop, and so is a missing floor beyond
    floor that was seen."""
    out = Slice([], [], [])
    for col in columns:
        floor_seen, floor_at, hit = False, None, False
        for d, x, y, h in col.readings:
            if h is None:
                if floor_seen:
                    out.drops.append((x, y))
                    break
                continue
            margin = 2.0 * p.flat_tol_m + 0.02 * d             # a tilt error grows with distance
            if h < -(p.max_drop_m + 0.02 * d):
                out.drops.append((x, y))
                break
            # In the way: lower than the robot is tall. The level row climbs with distance
            # (a wall 3 m off is met half a metre up), and that is still the wall.
            if margin < h < p.body_m + 0.08 * d:
                out.rays.append((x, y))
                if h < low_m:
                    out.low.append((x, y))
                hit = True
                break
            if abs(h) <= margin:
                floor_seen, floor_at = True, (x, y)
        if not hit and floor_at is not None:
            out.clear.append(floor_at)
    return out


def params_from(p: dict) -> Params:
    """Params from robot.yaml's /explorer section; anything missing keeps its default."""
    d = Params()
    return Params(h_max_m=float(p.get("h_max_m", d.h_max_m)),
                  flat_tol_m=float(p.get("flat_tol_m", d.flat_tol_m)),
                  min_landing_m=float(p.get("min_landing_m", d.min_landing_m)),
                  max_drop_m=float(p.get("max_drop_m", d.max_drop_m)),
                  corridor_m=float(p.get("path_width_m", d.corridor_m)),
                  drive_unknown=bool(p.get("drive_on_unknown_floor", d.drive_unknown)),
                  body_m=float(p.get("robot_height_m", d.body_m)),
                  stop_short_m=max(0.25, float(p.get("stop_short_m", d.stop_short_m))))
