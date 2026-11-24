"""Rolling local costmap: a 4 m x 4 m occupancy grid that moves with the robot.

The 8x8 ToF sees 60 degrees at a time from a gimbal that keeps moving, so what it saw
must be remembered once the head looks elsewhere. Every ToF grid is projected into the
robot's frame (Head.project, with the gimbal's angles at the moment of measurement), then
into the odometry frame (encoders for distance, the gyro for heading), and folded in here:

  * a reading that stops on something standing (higher than floor noise, up to `top_m`)
    adds evidence to its cell;
  * the cells the ray passed through on its way, and a ray that reached the floor, lose
    evidence, so a person who walked off, or a bad reading, is cleared by the next look;
  * evidence fades with time (`tau_s`): the odometry frame drifts.

The grid is re-centred on the robot in whole cells whenever it strays `recentre_m` from
the middle; what falls off the edge is forgotten. Each cell is one of three states: occupied
(lo > OCC), seen free (lo < FREE) or unknown (between). `clearance()` treats unknown as free,
which is right for DWA's short arcs (the gimbal sweeps, the reflex brakes, speed is capped where
the way is unseen) and WRONG for deciding a straight line is open: use `known_free_line()` for
that (Nov 19: twice, a straight line through an unseen wall was taken over the planner's way
round through a doorway).

`clearance(xs, ys)` is the distance (m) from each point to the nearest occupied cell (from
a distance transform, recomputed lazily after updates), which is what DWA scores against.
numpy only (cv2 for the distance transform when it is there). No ROS, no hardware.
"""

from __future__ import annotations

import math
import threading
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

XY = Tuple[float, float]


class RollingCostmap:
    HIT, MISS = 0.9, 0.4            # evidence per reading
    LONE_W = 0.5                    # a hit with no neighbouring zone on the same surface counts this much: two to draw
    #                                 it. One stray zone in ~2000 (a look at -10 deg) put a point inside a 0.70 m
    #                                 doorway that a one-hit map kept: the door mapped 0.41 m (Nov 20 door calibration).
    #                                 A real thing hits neighbouring zones at once (or sits on the grid's border, where a
    #                                 wall's edge has fewer neighbours) and is drawn from one glance (with the
    #                                 head on a person, one glance is all it gets: test_scenarios lookaway).
    TOP, BOTTOM = 3.5, -2.0
    OCC = 0.6                       # occupied above this (one solid hit, or two weak ones)
    FREE = -0.3                     # seen free below this; between FREE and OCC: unknown
    FAR = 9.0                       # "no obstacle anywhere near"
    LOW = 0.065                     # tan(half a zone): how far a zone's lower edge is below its centre, per metre
    SHARP_M = 0.6                   # hits nearer than this count in full; further, by SHARP_M / d (zone width)

    def __init__(self, size_m: float = 4.0, res_m: float = 0.05, tau_s: float = 60.0,
                 recentre_m: float = 0.5, flat_tol_m: float = 0.03, body_m: float = 0.24) -> None:
        self.n = int(round(size_m / res_m))
        self.res, self.size, self.tau, self.recentre = res_m, size_m, tau_s, recentre_m
        self.flat_tol, self.body = flat_tol_m, body_m
        self.lo = np.zeros((self.n, self.n), np.float32)      # evidence, [ix, iy]
        self.ox = self.oy = -size_m / 2.0                     # world xy of cell (0, 0)'s corner
        self._t: Optional[float] = None
        self._field: Optional[np.ndarray] = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------------ frame

    def _cells(self, x, y):
        return (np.floor((np.asarray(x) - self.ox) / self.res).astype(int),
                np.floor((np.asarray(y) - self.oy) / self.res).astype(int))

    def _inside(self, ix, iy):
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    def follow(self, x: float, y: float) -> None:
        """Keep (x, y) near the middle: shift the grid by whole cells, forgetting the far edge."""
        with self._lock:
            cx, cy = self.ox + self.size / 2.0, self.oy + self.size / 2.0
            dx = int(round((x - cx) / self.res)) if abs(x - cx) > self.recentre else 0
            dy = int(round((y - cy) / self.res)) if abs(y - cy) > self.recentre else 0
            if not dx and not dy:
                return
            out = np.zeros_like(self.lo)
            sx, sy = slice(max(0, dx), self.n + min(0, dx)), slice(max(0, dy), self.n + min(0, dy))
            tx, ty = slice(max(0, -dx), self.n + min(0, -dx)), slice(max(0, -dy), self.n + min(0, -dy))
            if abs(dx) < self.n and abs(dy) < self.n:
                out[tx, ty] = self.lo[sx, sy]
            self.lo = out
            self.ox += dx * self.res
            self.oy += dy * self.res
            self._field = None

    def _decay(self, t: float) -> None:
        if self._t is not None and t > self._t:
            self.lo *= math.exp(-(t - self._t) / self.tau)
        self._t = t

    # ----------------------------------------------------------------- update

    def integrate(self, pose: Sequence[float], zones, sensor_xy: XY = (0.0, 0.0), t: Optional[float] = None,
                  top_m: Optional[float] = None, max_d: Optional[float] = None) -> int:
        """One ToF grid. `zones` as Head.project gives them: (row, col, x, y, h, d) in the robot's
        frame at `pose` (x, y, heading in the odometry frame); h None = no return where the floor
        should have been (not known: neither hit nor free). A zone standing higher than floor
        noise and lower than `top_m` (default: the robot's height) is a hit. Rays that stay at
        the robot's height or lower clear the cells they pass through (one passing overhead
        proves nothing about the floor below it). Zones `max_d` or further away (a person being
        followed) are left out. Returns the number of hits."""
        if pose is None or not zones:
            return 0
        top = (self.body if top_m is None else top_m) + 0.05
        hx, hy, hw, rx, ry, clear_end = [], [], [], [], [], []
        # A zone reports the nearest floor in its cone too: its lower edge, which projected along the centre
        # stands d * 0.065 above the floor (6 cm a metre out: the scattered phantom cells on Nov 19's map).
        is_hit = lambda h, d: h is not None and self.flat_tol < h - self.LOW * d and h < top   # noqa: E731
        hits = {(r, c): d for r, c, _, _, h, d in zones if is_hit(h, d) and (max_d is None or d < max_d)}
        for r, c, x, y, h, d in zones:
            if h is None or (max_d is not None and d >= max_d):
                continue
            hit = is_hit(h, d)
            if hit:
                hx.append(x)
                hy.append(y)
                lone = 0 < r < 7 and 0 < c < 7 and not any(hits.get((r + i, c + j), 99.0) < 1.5 * d
                               for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j)
                hw.append(min(1.0, self.SHARP_M / max(d, 1e-3)) * (self.LONE_W if lone else 1.0))   # far: weaker
            if h <= self.body + 0.05:
                rx.append(x)
                ry.append(y)
                clear_end.append(h <= self.flat_tol)   # floor for sure; between that and a hit: leave the cell be
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = lambda bx, by: (pose[0] + c * np.asarray(bx) - s * np.asarray(by),      # noqa: E731
                                pose[1] + s * np.asarray(bx) + c * np.asarray(by))
        with self._lock:
            self.follow(pose[0], pose[1])
            if t is not None:
                self._decay(t)
            hit_ix, hit_iy = self._cells(*world(hx, hy)) if hx else (np.zeros(0, int), np.zeros(0, int))
            inside = self._inside(hit_ix, hit_iy)
            hit_w = {}
            for cell, wgt in zip((hit_ix * self.n + hit_iy)[inside].tolist(), np.asarray(hw)[inside].tolist()):
                hit_w[cell] = max(wgt, hit_w.get(cell, 0.0))
            hit_lin = set(hit_w)
            if rx:                                            # free space along the rays
                sx, sy = world([sensor_xy[0]], [sensor_xy[1]])
                ex, ey = world(rx, ry)
                lens = np.hypot(ex - sx[0], ey - sy[0])
                steps = np.maximum(1, (lens / (self.res / 2.0)).astype(int))
                k = int(steps.max())
                f = np.arange(k + 1)[None, :] / steps[:, None]                # fraction along each ray
                keep = f <= 1.0
                endcell = np.isclose(f, 1.0) & ~np.asarray(clear_end)[:, None]  # a hit's own cell is not free
                px = sx[0] + (ex - sx[0])[:, None] * f
                py = sy[0] + (ey - sy[0])[:, None] * f
                ix, iy = self._cells(px[keep & ~endcell], py[keep & ~endcell])
                ok = self._inside(ix, iy)
                lin = np.unique(ix[ok] * self.n + iy[ok])
                if hit_lin:
                    lin = lin[~np.isin(lin, list(hit_lin))]
                v = self.lo.reshape(-1)
                v[lin] = np.maximum(self.BOTTOM, v[lin] - self.MISS)
            if hit_lin:
                v = self.lo.reshape(-1)
                idx = np.fromiter(hit_w, int)
                v[idx] = np.minimum(self.TOP, v[idx] + self.HIT * np.fromiter(hit_w.values(), float))
            self._field = None
        return len(hx)

    def add_obstacle(self, pose: Sequence[float], pts_base: Iterable[XY], strength: float = TOP) -> None:
        """Something found without seeing it (a bump, being stuck): occupied at once."""
        pts = list(pts_base)
        if pose is None or not pts:
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        with self._lock:
            for bx, by in pts:
                ix, iy = self._cells(pose[0] + c * bx - s * by, pose[1] + s * bx + c * by)
                if self._inside(ix, iy):
                    self.lo[ix, iy] = max(self.lo[ix, iy], strength)
            self._field = None

    def clear_near(self, xy: XY, radius_m: float) -> None:
        """What was seen within radius_m of xy is the person being followed, not an obstacle: the floor
        they stand on is free. Marked SEEN FREE, not unknown, so the Mac's global map (which folds only
        known cells) drops their trail too (Nov 23: their trail blocked the way and the plan went 9 m
        back round)."""
        with self._lock:
            ix, iy = self._cells(xy[0], xy[1])
            r = int(math.ceil(radius_m / self.res))
            gx, gy = np.meshgrid(np.arange(ix - r, ix + r + 1), np.arange(iy - r, iy + r + 1), indexing="ij")
            near = self._inside(gx, gy) & (np.hypot((gx - ix) * self.res, (gy - iy) * self.res) <= radius_m)
            self.lo[gx[near], gy[near]] = np.minimum(self.lo[gx[near], gy[near]], self.FREE - 0.2)
            self._field = None

    # ------------------------------------------------------------------ query

    def occupied(self) -> np.ndarray:
        with self._lock:
            return self.lo > self.OCC

    def _distance_field(self) -> np.ndarray:
        if self._field is None:
            occ = self.lo > self.OCC
            if not occ.any():
                self._field = np.full(occ.shape, self.FAR, np.float32)
            else:
                try:
                    import cv2
                    d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5)
                except Exception:                                # noqa: BLE001 — no cv2: brute force
                    oi, oj = np.nonzero(occ)
                    gi, gj = np.indices(occ.shape)
                    d = np.full(occ.shape, np.inf, np.float32)
                    for a in range(0, len(oi), 256):
                        d = np.minimum(d, np.min(np.hypot(gi[..., None] - oi[a:a + 256], gj[..., None] - oj[a:a + 256]),
                                                 axis=-1))
                self._field = (d.astype(np.float32) * self.res)
        return self._field

    def clearance(self, xs, ys) -> np.ndarray:
        """Distance (m) from each point to the nearest occupied cell; FAR off the map."""
        with self._lock:
            field = self._distance_field()
            ix, iy = self._cells(xs, ys)
            ok = self._inside(ix, iy)
            out = np.full(np.shape(ix), self.FAR, np.float32)
            out[ok] = field[ix[ok], iy[ok]]
            return out

    def seen_free(self, pose, ahead_m: float) -> bool:
        """Every cell on the way straight ahead of `pose` for ahead_m was seen free (not just unknown)."""
        n = max(2, int(ahead_m / (self.res / 2.0)))
        f = np.linspace(0.15, ahead_m, n)
        ix, iy = self._cells(pose[0] + f * math.cos(pose[2]), pose[1] + f * math.sin(pose[2]))
        with self._lock:
            ok = self._inside(ix, iy)
            return bool(ok.all() and (self.lo[ix, iy] < self.FREE).all())

    def known_free_line(self, a: XY, b: XY) -> bool:
        """Every cell on the straight line a -> b was SEEN free (unknown is not free here)."""
        d = math.dist(a, b)
        return d < 0.2 or self.seen_free((a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0])), d)

    def line_clear(self, a: XY, b: XY, radius_m: float) -> bool:
        """Nothing KNOWN within radius_m of the line; unknown cells pass. Pair with known_free_line
        when the answer decides that a way is open."""
        n = max(2, int(math.dist(a, b) / (self.res / 2.0)) + 1)
        xs, ys = np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)
        return bool(np.all(self.clearance(xs, ys) >= radius_m))

    def snapshot(self) -> dict:
        """The whole window for the Mac's global map: its corner, resolution and size, and every
        cell as one byte (base64): 2 occupied, 1 seen free, 0 not known."""
        import base64
        with self._lock:
            cells = np.zeros(self.lo.shape, np.uint8)
            cells[self.lo < -0.3] = 1
            cells[self.lo > self.OCC] = 2
            return {"ox": round(self.ox, 3), "oy": round(self.oy, 3), "res": self.res, "n": self.n,
                    "cells": base64.b64encode(cells.tobytes()).decode()}

    def points(self) -> List[XY]:
        """World xy of every occupied cell's centre (for the Mac's global map, and debugging)."""
        with self._lock:
            ix, iy = np.nonzero(self.lo > self.OCC)
            return list(zip((self.ox + (ix + 0.5) * self.res).round(3).tolist(),
                            (self.oy + (iy + 0.5) * self.res).round(3).tolist()))
