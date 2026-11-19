"""Local path planning around what the ToF found: shared by follow and explore.

The floor check (terrain.py) says whether the straight drive ahead is safe. This
remembers what made a drive unsafe (a box, a step, a drop, floor it could not read)
in the odometry frame, so a known hazard is not looked at again, and plans a way
around it: A* over a small grid around the robot, with every hazard grown by the
robot's half-width, cut down to the fewest straight legs (each one a turn then a
drive, checked by the floor check before it is driven).

Unknown ground counts as free: the plan is only a direction to try, and the floor
check before every leg is what keeps the robot safe. No hardware, no ROS.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Iterable, List, Optional, Sequence, Tuple

from robot_core import terrain

XY = Tuple[float, float]
CELL_M = 0.05
BORDER_M = 1.0          # how far beyond the start and the goal the grid reaches
KEEP_S = 120.0          # a hazard is remembered this long (odometry drifts)
NEIGHBOUR_M = 0.12      # a hazard point counts only with another within this (one noisy zone is not a box)
MIN_GAP_M = 0.10        # never plan nearer a hazard than this, even to get out from beside it
SURE_M = 0.08           # only something this far above or below the floor is remembered: the floor
                        # itself reads up to ~5 cm off, which the drive check rightly refuses, but
                        # planning round floor noise sends the robot the wrong way


def hazards_from_floor(pts: Sequence[terrain.Point], p: terrain.Params,
                       sensor: Optional[Tuple[float, float]] = None) -> List[XY]:
    """The points of a floor scan (x, y, h in base_link) the robot must not drive over:
    higher than floor noise (and lower than the robot, which passes under a table), lower
    than a drop, or no return where the floor should be. `sensor` = (x, height) of the
    ToF: a drop ray is also marked where it should have met the floor, nearer the edge."""
    out, solid = [], []                               # solid: something standing there (its far side is unseen)
    for x, y, h in pts:
        if x < p.near_m:
            continue
        if h is not None and max(p.flat_tol_m, SURE_M) < h < p.body_m + 0.05:   # no drops: the TF-Luna's
            out.append((x, y))
            solid.append((x, y))
    keep = [a for i, a in enumerate(out)
            if any(j != i and math.dist(a, b) <= NEIGHBOUR_M for j, b in enumerate(out))]
    return keep + fill_behind([a for a in keep if a in solid], sensor[0] if sensor is not None else 0.0)


def fill_behind(pts: Sequence[XY], sensor_x: float = 0.0) -> List[XY]:
    """The ToF sees only the near face of what stands in the way: points 0.1 and 0.2 m
    further along each ray, so a way round does not cut the unseen far side."""
    out = []
    for x, y in pts:
        n = math.hypot(x - sensor_x, y) or 1.0
        out += [(x + (x - sensor_x) / n * d, y + y / n * d) for d in (0.1, 0.2)]
    return out


class Hazards:
    """Hazard points in the odometry frame, each kept for KEEP_S."""

    def __init__(self, keep_s: float = KEEP_S, clock=time.monotonic) -> None:
        self.keep_s, self._clock, self._pts = keep_s, clock, []      # (x, y, t)

    def add(self, pose: Sequence[float], pts_base: Iterable[XY], felt: bool = False) -> int:
        """pose: the robot's (x, y, heading) when it saw them; pts_base: (x ahead, y left).
        `felt`: found by bumping into it, not seen: forget_near (a view that looks clear) keeps it."""
        x0, y0, th = pose[0], pose[1], pose[2]
        c, s, now = math.cos(th), math.sin(th), self._clock()
        new = [(x0 + c * x - s * y, y0 + s * x + c * y, now, felt) for x, y in pts_base]
        self._pts = [q for q in self._pts if now - q[2] <= self.keep_s] + new
        return len(new)

    def points(self, max_age_s: Optional[float] = None) -> List[XY]:
        """The points still kept (only those seen in the last `max_age_s`, if given)."""
        now, keep = self._clock(), self.keep_s if max_age_s is None else min(self.keep_s, max_age_s)
        return [(x, y) for x, y, t, _ in self._pts if now - t <= keep]

    def forget_near(self, xy: XY, radius_m: float) -> None:
        self._pts = [q for q in self._pts if q[3] or math.hypot(q[0] - xy[0], q[1] - xy[1]) > radius_m]


def radius(p: terrain.Params) -> float:
    """How far the robot's middle keeps from a hazard point: half its width plus the
    clearance (robot.yaml robot_width_m, plan_clearance_m). What is behind an obstacle's
    near face is filled in by hazards_from_floor, so this need not allow for it."""
    return p.width_m / 2.0 + p.clearance_m


def line_clear(a: XY, b: XY, hazards: Sequence[XY], radius_m: float) -> bool:
    """Does the straight drive from a to b keep `radius_m` from every hazard? Starting
    inside that margin, it must at least never get nearer to one than it is at a."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    n2 = dx * dx + dy * dy
    r = min(radius_m, max(MIN_GAP_M, min((math.dist(a, h) for h in hazards), default=math.inf) - 1e-6))
    for hx, hy in hazards:
        t = 0.0 if n2 == 0 else max(0.0, min(1.0, ((hx - ax) * dx + (hy - ay) * dy) / n2))
        if math.hypot(ax + t * dx - hx, ay + t * dy - hy) < r:
            return False
    return True


def plan(start: XY, goal: XY, hazards: Sequence[XY], radius_m: float,
         cell_m: float = CELL_M, border_m: float = BORDER_M) -> Optional[List[XY]]:
    """Straight legs from start to goal keeping `radius_m` from every hazard, as
    waypoints [start, ..., goal]; None if there is no way round within the grid. A goal
    inside a hazard's margin moves to the nearest free cell within 0.5 m of it."""
    x0 = min(start[0], goal[0]) - border_m
    y0 = min(start[1], goal[1]) - border_m
    nx = int((max(start[0], goal[0]) + border_m - x0) / cell_m) + 1
    ny = int((max(start[1], goal[1]) + border_m - y0) / cell_m) + 1
    cell = lambda p: (int(round((p[0] - x0) / cell_m)), int(round((p[1] - y0) / cell_m)))   # noqa: E731
    centre = lambda c: (x0 + c[0] * cell_m, y0 + c[1] * cell_m)                             # noqa: E731
    r = int(math.ceil(radius_m / cell_m))
    near = {}                                         # cell -> distance to its nearest hazard, within radius
    for h in hazards:
        hc = cell(h)
        for i in range(hc[0] - r, hc[0] + r + 1):
            for j in range(hc[1] - r, hc[1] + r + 1):
                d = math.dist(centre((i, j)), h)
                if 0 <= i < nx and 0 <= j < ny and d < radius_m and d < near.get((i, j), math.inf):
                    near[(i, j)] = d
    # Already inside a margin (stopped short of something): it may move through cells no closer
    # to a hazard than it is now, to get out, but never nearer.
    d0 = max(MIN_GAP_M, min((math.dist(start, h) for h in hazards), default=math.inf))
    blocked = {c for c, d in near.items() if d < d0 - 1e-9}
    s, g = cell(start), cell(goal)
    blocked.discard(s)                                # the robot is where it is: it may always leave
    if g in blocked:
        free = [(i, j) for i in range(g[0] - 10, g[0] + 11) for j in range(g[1] - 10, g[1] + 11)
                if 0 <= i < nx and 0 <= j < ny and (i, j) not in blocked
                and math.dist(centre((i, j)), goal) <= 0.5]
        if not free:
            return None
        g = min(free, key=lambda c: math.dist(centre(c), goal))
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    came, cost, todo = {s: None}, {s: 0.0}, [(0.0, s)]
    while todo:
        _, c = heapq.heappop(todo)
        if c == g:
            break
        for di, dj, w in steps:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or n in blocked:
                continue
            nc = cost[c] + w
            if nc < cost.get(n, math.inf):
                cost[n], came[n] = nc, c
                heapq.heappush(todo, (nc + math.dist(n, g), n))
    if g not in came:
        return None
    cells = [g]
    while came[cells[-1]] is not None:
        cells.append(came[cells[-1]])
    cells.reverse()

    def sees(a, b) -> bool:                           # a straight leg on the grid avoids every blocked cell
        k = max(1, int(2 * math.dist(a, b)))
        return all((round(a[0] + (b[0] - a[0]) * t / k), round(a[1] + (b[1] - a[1]) * t / k)) not in blocked
                   for t in range(k + 1))
    legs, i = [cells[0]], 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not sees(cells[i], cells[j]):
            j -= 1
        legs.append(cells[j])
        i = j
    path = [start] + [centre(c) for c in legs[1:]]
    if legs[-1] == cell(goal):
        path[-1] = goal
    return path


def leg(pose: Sequence[float], to: XY) -> Tuple[float, float]:
    """From pose (x, y, heading CCW) to a point: (turn in degrees, + right as the drive
    tools take it; distance in metres)."""
    dx, dy = to[0] - pose[0], to[1] - pose[1]
    turn = math.degrees(math.atan2(math.sin(math.atan2(dy, dx) - pose[2]), math.cos(math.atan2(dy, dx) - pose[2])))
    return -turn, math.hypot(dx, dy)
