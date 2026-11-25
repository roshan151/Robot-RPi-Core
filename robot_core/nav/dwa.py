"""Dynamic Window Approach: pick the (v, w) to drive for the next tick.

A tracked rover is non-holonomic: it can go forward and turn, not sideways, and it has
inertia. DWA respects that by only considering speeds it can actually reach within the
next moment (the dynamic window round the measured v and gyro w, given the acceleration
limits), simulating each constant (v, w) pair for `horizon_s`, throwing out the ones that
come within the robot's radius of anything in the costmap or could not stop in time, and
scoring the rest:

    progress  how much nearer the target the arc ends (normalised by the best possible)
    heading   how well the arc's end faces the target
    clearance the nearest obstacle along the arc (capped: far is as good as very far)
    speed     faster is better, which keeps it from creeping

Turning on the spot (v = 0) is always among the candidates, so a robot boxed in can still
turn to look for a way out. No admissible arc at all: (0, 0) and "blocked".
Vectorised numpy: ~300 arcs x 20 steps in about a millisecond on a Pi 4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class DwaParams:
    max_v: float = 0.25            # robot.yaml velocity_max_mps            # m/s (the velocity loop's limit is 0.25)
    max_w: float = 1.0             # rad/s
    acc_v: float = 0.4             # m/s^2, as the drivetrain's velocity_accel_mps2
    acc_w: float = 2.0             # rad/s^2
    window_s: float = 0.3          # the dynamic window: what is reachable this long from now
    horizon_s: float = 2.0         # how far ahead each arc is simulated
    dt: float = 0.1
    nv: int = 9
    nw: int = 25
    radius_m: float = 0.177        # circumscribed circle (half the 25 x 25 cm diagonal), for turn-in-place / fallback
    half_l_m: float = 0.125        # physical body half-length (along heading)
    half_w_m: float = 0.125        # physical body half-width (sideways)
    side_pad_m: float = 0.04       # extra body pad on flat sides (0.165 + 0.04 = 0.205 m sideways vs 0.217 m circle)
    margin_m: float = 0.04         # an arc must keep this clear beyond the body edge
    nose_m: float = 0.10           # ...and a point this far ahead of the centre must keep nose_clear_m clear, so
    nose_clear_m: float = 0.17     # head-on it stays 0.27 m from what is ahead (the reflex brakes at 0.25)
    clear_cap_m: float = 0.5       # clearance beyond radius + this counts no better
    w_progress: float = 1.0
    w_heading: float = 0.4
    w_clear: float = 0.5
    w_speed: float = 0.2
    slow_m: float = 0.4            # within this of the target, top speed falls with distance

    @property
    def body_radius_m(self) -> float:
        """Effective half-width of the oriented body capsule (shared by DWA, local Theta*, and Mac GlobalMap)."""
        return self.half_w_m + self.side_pad_m


@dataclass
class DwaResult:
    v: float
    w: float
    blocked: bool
    clearance_m: float             # along the chosen arc (from the robot's edge)
    candidates: int


def arcs(x: float, y: float, th: float, vs: np.ndarray, ws: np.ndarray, horizon: float, dt: float):
    """Positions along constant-(v, w) arcs: (len(vs), steps) arrays xs, ys, ths."""
    steps = max(1, int(round(horizon / dt)))
    t = dt * np.arange(1, steps + 1)[None, :]
    v, w = vs[:, None], ws[:, None]
    ths = th + w * t
    straight = np.abs(w) < 1e-6
    wsafe = np.where(straight, 1.0, w)
    xs = np.where(straight, x + v * t * math.cos(th), x + v / wsafe * (np.sin(ths) - math.sin(th)))
    ys = np.where(straight, y + v * t * math.sin(th), y - v / wsafe * (np.cos(ths) - math.cos(th)))
    return xs, ys, ths


def rect_clearance(xs: np.ndarray, ys: np.ndarray, ths: np.ndarray, clearance_fn,
                   half_l: float, half_w: float, side_pad: float = 0.04) -> np.ndarray:
    """Oriented body clearance (m) along heading: a 3-circle capsule of half-width
    `r_side = half_w + side_pad` (0.165 m -> 0.205 m with margin) and axial offset `d`
    chosen so `r_side + d*cos(45) >= diag` (0.177 m), covering all 4 square corners
    during turns while reclaiming ~1.2 cm/side when driving straight past walls and doors."""
    shape = xs.shape
    r_side = half_w + side_pad
    diag = math.hypot(half_l, half_w)
    d = max(0.0, (diag - r_side) / math.sqrt(0.5))      # ensures r_side + d*cos(45) >= diag (no corner clips)
    c, s = np.cos(ths), np.sin(ths)
    clear = clearance_fn(xs.ravel(), ys.ravel()).reshape(shape) - r_side
    if d > 1e-4:
        for sign in (-1.0, 1.0):
            px = xs + sign * d * c
            py = ys + sign * d * s
            clear = np.minimum(clear, clearance_fn(px.ravel(), py.ravel()).reshape(shape) - r_side)
    return clear


def plan(state: Sequence[float], target: Sequence[float], clearance_fn, p: DwaParams = DwaParams(),
         stop_at_m: float = 0.0) -> DwaResult:
    """state: (x, y, heading, v, w) with v, w measured (or last commanded). target: (x, y).
    clearance_fn(xs, ys) -> metres to the nearest obstacle (RollingCostmap.clearance).
    stop_at_m: the target is reached this far short of it (a person's standoff)."""
    x, y, th, v0, w0 = state
    dist = math.hypot(target[0] - x, target[1] - y) - stop_at_m
    vmax = p.max_v if dist >= p.slow_m else max(0.0, p.max_v * dist / p.slow_m)
    vlo, vhi = max(0.0, v0 - p.acc_v * p.window_s), min(vmax, v0 + p.acc_v * p.window_s)
    if vhi < vlo:                                   # slowing harder than the window allows: brake anyway
        vlo = vhi = max(0.0, vmax)
    wlo, whi = max(-p.max_w, w0 - p.acc_w * p.window_s), min(p.max_w, w0 + p.acc_w * p.window_s)
    vs1 = np.unique(np.concatenate([np.linspace(vlo, vhi, p.nv), [0.0]]))
    ws1 = np.unique(np.concatenate([np.linspace(wlo, whi, p.nw), [0.0] if wlo <= 0.0 <= whi else []]))
    V, W = (a.ravel() for a in np.meshgrid(vs1, ws1, indexing="ij"))
    xs, ys, ths = arcs(x, y, th, V, W, p.horizon_s, p.dt)
    clear = rect_clearance(xs, ys, ths, clearance_fn, p.half_l_m, p.half_w_m, p.side_pad_m)
    # The nose: head-on is where the reflex looks, so it must stay further from things ahead than to the side.
    nose = clearance_fn((xs + p.nose_m * np.cos(ths)).ravel(), (ys + p.nose_m * np.sin(ths)).ravel()).reshape(xs.shape)
    clear = np.minimum(clear, nose - p.nose_clear_m + p.margin_m)
    here = float(rect_clearance(np.array([x]), np.array([y]), np.array([th]),
                                clearance_fn, p.half_l_m, p.half_w_m, p.side_pad_m)[0])
    here = min(here, float(clearance_fn(np.array([x + p.nose_m * math.cos(th)]),
                                        np.array([y + p.nose_m * math.sin(th)]))[0]) - p.nose_clear_m + p.margin_m)
    # Starting inside the margin (stopped close to something): an arc may not get any closer.
    need = min(p.margin_m, here - 1e-3)
    minclear = clear.min(axis=1)
    # Admissible: never closer than `need`, and able to brake before the first point that is.
    travel = np.abs(V)[:, None] * p.dt * np.arange(1, xs.shape[1] + 1)[None, :]
    first_bad = np.where(clear < p.margin_m, travel, np.inf).min(axis=1)
    can_stop = V ** 2 <= 2.0 * p.acc_v * first_bad
    ok = (minclear >= need) & (can_stop | (V == 0.0))
    if not ok.any():
        return DwaResult(0.0, 0.0, True, here, 0)
    d0 = math.hypot(target[0] - x, target[1] - y)
    # Closest approach on the arc, not where it ends 2 s on: an end past the target scored as less progress, so near
    # the target it slowed at every waypoint (test_scenarios trap: 31.7 s, now 23 s).
    dend = np.hypot(target[0] - xs, target[1] - ys).min(axis=1)
    progress = d0 - dend
    bearing = np.arctan2(target[1] - ys[:, -1], target[0] - xs[:, -1]) - ths[:, -1]
    heading = 1.0 - np.abs(np.arctan2(np.sin(bearing), np.cos(bearing))) / math.pi
    clr = np.minimum(minclear, p.clear_cap_m) / p.clear_cap_m
    speed = V / max(p.max_v, 1e-6)

    def norm(a):
        a = np.where(ok, a, np.nan)
        lo, hi = np.nanmin(a), np.nanmax(a)
        return np.zeros_like(a) if hi - lo < 1e-9 else (a - lo) / (hi - lo)

    score = (p.w_progress * norm(progress) + p.w_heading * norm(heading)
             + p.w_clear * norm(clr) + p.w_speed * norm(speed))
    score = np.where(ok, score, -np.inf)
    i = int(np.argmax(score))
    return DwaResult(float(V[i]), float(W[i]), False, float(minclear[i]), int(ok.sum()))


def face(heading_err: float, max_w: float = 0.8, gain: float = 1.5, deadband: float = math.radians(10)) -> float:
    """Turn rate (rad/s, + left) to face something heading_err (rad, + left) off: standing still."""
    if abs(heading_err) < deadband:
        return 0.0
    return max(-max_w, min(max_w, gain * heading_err)) if abs(gain * heading_err) > 0.3 else math.copysign(0.3, heading_err)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def target_on_path(pose: Sequence[float], waypoints: Sequence[Sequence[float]], clear_fn,
                   radius_m: float, reached_m: float = 0.25, seen_fn=None) -> Optional[Tuple[float, float]]:
    """The furthest waypoint that can be seen in a straight line (clear of obstacles) from the
    robot, after dropping the ones already reached: DWA steers for it. None: no waypoints.
    seen_fn(a, b): the straight line was SEEN free, not just not known to be blocked; skipping
    ahead past the next waypoint needs it (Nov 19: with the head on the person the wall beside a
    doorway was never in the costmap, so the target flipped each tick between the doorway and the
    person straight through the wall, and the robot spun left-right until they were gone)."""
    pts = [tuple(w) for w in waypoints]
    while len(pts) > 1 and math.hypot(pts[0][0] - pose[0], pts[0][1] - pose[1]) < reached_m:
        pts.pop(0)
    if not pts:
        return None
    best = pts[0]
    for wp in pts[1:]:
        n = max(2, int(math.hypot(wp[0] - pose[0], wp[1] - pose[1]) / 0.05))
        xs, ys = np.linspace(pose[0], wp[0], n), np.linspace(pose[1], wp[1], n)
        if np.all(clear_fn(xs, ys) >= radius_m) and (seen_fn is None or seen_fn(tuple(pose[:2]), wp)):
            best = wp
        else:
            break
    return best
