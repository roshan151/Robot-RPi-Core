"""Theta* on the rolling costmap: the Pi's own way round, when the Mac's plan is missing.

DWA alone looks two seconds ahead; facing a box head-on it slows and dithers instead of
committing to one side (a local minimum). The Mac plans globally (Theta* on its map), but
the network may drop, so the Pi can plan the same way over its 4 x 4 m window: Theta* is
A* whose paths may cut across cells in any direction wherever there is line of sight, so
they come out as a few straight legs, not a staircase. The goal, if off the window, is
replaced by the furthest point toward it that is still on the window.

Cells are free when they keep `radius + margin` from everything occupied; with no path it
tries once more with a thinner margin (a narrow door). Costs add a little for passing close
to things, so paths keep to the middle of gaps. Pure numpy + heapq; ~10 ms on a Pi 4.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

XY = Tuple[float, float]


def _los(free: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    """Every cell on the straight line a -> b free (supercover sampling)."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
    xs = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    ys = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(free[xs, ys].all())


def theta_star(free: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int],
               penalty: Optional[np.ndarray] = None) -> Optional[List[Tuple[int, int]]]:
    """Cells [start, ..., goal] or None. `free`: bool grid; `penalty`: extra cost per cell entered."""
    nx, ny = free.shape
    if not free[goal]:
        return None
    g = {start: 0.0}
    parent = {start: start}
    todo = [(math.dist(start, goal), start)]
    closed = set()
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while todo:
        _, c = heapq.heappop(todo)
        if c in closed:
            continue
        if c == goal:
            path = [c]
            while parent[path[-1]] != path[-1]:
                path.append(parent[path[-1]])
            return path[::-1]
        closed.add(c)
        for dx, dy in steps:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or not free[n] or n in closed:
                continue
            extra = 0.0 if penalty is None else float(penalty[n])
            p = parent[c]
            if p != c and _los(free, p, n):               # any-angle: straight from the grandparent
                cost = g[p] + math.dist(p, n) + extra
                via = p
            else:
                cost = g[c] + math.dist(c, n) + extra
                via = c
            if cost < g.get(n, math.inf):
                g[n], parent[n] = cost, via
                heapq.heappush(todo, (cost + math.dist(n, goal), n))
    return None


def local_path(costmap, start: XY, goal: XY, radius_m: float, margin_m: float = 0.12,
               thin_margin_m: float = 0.04) -> Optional[List[XY]]:
    """Waypoints from start to goal (or to the window's edge toward it) on the costmap, world xy;
    the first is the start. None: no way within the window."""
    with costmap._lock:
        field = costmap._distance_field().copy()
        ox, oy, res, n = costmap.ox, costmap.oy, costmap.res, costmap.n
    cell = lambda p: (int((p[0] - ox) / res), int((p[1] - oy) / res))     # noqa: E731
    centre = lambda c: (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res)  # noqa: E731
    s = cell(start)
    if not (0 <= s[0] < n and 0 <= s[1] < n):
        return None
    gx, gy = goal
    edge = n * res / 2.0 - 0.15                       # off the window: the furthest point toward it on it
    cx, cy = ox + n * res / 2.0, oy + n * res / 2.0
    far = max(abs(gx - cx), abs(gy - cy))
    if far > edge:
        k = 1.0
        for _ in range(30):
            px, py = start[0] + (gx - start[0]) * k, start[1] + (gy - start[1]) * k
            if max(abs(px - cx), abs(py - cy)) <= edge:
                break
            k *= 0.9
        gx, gy = px, py
    for margin in (margin_m, thin_margin_m):
        free = field >= radius_m + margin
        here = field[s] if 0 <= s[0] < n and 0 <= s[1] < n else 0.0
        if not free[s]:                                # starting inside the margin: let it leave
            free |= field >= max(0.0, here - 1e-3)
            free[s] = True
        g = cell((gx, gy))
        if not free[g]:                                # goal in the margin: nearest free cell within 0.5 m
            r = int(0.5 / res)
            ii, jj = np.meshgrid(np.arange(max(0, g[0] - r), min(n, g[0] + r + 1)),
                                 np.arange(max(0, g[1] - r), min(n, g[1] + r + 1)), indexing="ij")
            ok = free[ii, jj]
            if not ok.any():
                continue
            d = np.hypot(ii - g[0], jj - g[1])
            d[~ok] = np.inf
            k = np.unravel_index(np.argmin(d), d.shape)
            g = (int(ii[k]), int(jj[k]))
        penalty = np.clip((radius_m + 0.35 - field) / 0.35, 0.0, 1.0) * 2.0
        cells = theta_star(free, s, g, penalty)
        if cells:
            return [start] + [centre(c) for c in cells[1:]]
    return None
