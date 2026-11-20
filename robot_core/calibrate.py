"""Calibration maths: plain functions, so they are tested without hardware.
Monitoring-Service/calibrate.py collects the readings (through robot_core/calibrate_pi.py
on the Pi) and these turn them into robot.yaml values.

  pan_zero      head square-on to a flat wall, pan swept: the range is shortest where
                the head really points at the wall, r = D / cos(pan - p0). Linear in
                1/r = a cos(pan) + b sin(pan)  ->  p0 = atan2(b, a), D = 1 / hypot(a, b)
  tilt_zero     head straight, flat floor ahead, tilt swept down: the centre ray hits the
                floor at r = h / sin(-(tilt + e)) for the sensor's height h. Each reading
                gives e = -asin(h / r) - tilt; the median is the tilt error
  range_fit     target at known tape distances: measured = (true - offset) / scale, so
                true = scale * measured + offset by least squares
  tilt_and_height  the same floor sweep with the sensor's height unknown too: the tilt
                error is the one that makes every reading agree on one height
  drive_fit     backing away from a wall, two tape readings (start, end) and the ToF
                readings at each stop -> ToF scale/offset and ticks_per_cm
  turn_fit      a commanded turn and the gyro's measured one -> ticks_per_degree
  reverse_trim  how much the forward-running track led in turns -> REVERSE_TRIM_PCT
  centre_change the pulse change that moves a servo's zero by `deg`
  door_fit      the robot still before a doorway, the head stepped: the wall line through
                the ToF points, and along it the door (the widest stretch with no wall
                points that rays went through): each edge per look, its spread, the width
  door_turn_fit the same door seen after turns: the gyro scale that puts it in one place
"""

from __future__ import annotations

import math
import statistics
from typing import List, Optional, Sequence, Tuple

import numpy as np


def pan_zero(samples: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    """(pan_deg, range_m) pairs -> (p0_deg: where the head truly faces the wall, wall distance m)."""
    pts = [(math.radians(p), 1.0 / r) for p, r in samples if r and r > 0]
    if len(pts) < 3:
        raise ValueError("need at least 3 readings")
    scc = sum(math.cos(a) ** 2 for a, _ in pts)
    sss = sum(math.sin(a) ** 2 for a, _ in pts)
    scs = sum(math.cos(a) * math.sin(a) for a, _ in pts)
    syc = sum(y * math.cos(a) for a, y in pts)
    sys_ = sum(y * math.sin(a) for a, y in pts)
    det = scc * sss - scs * scs
    if abs(det) < 1e-12:
        raise ValueError("sweep too narrow")
    a = (syc * sss - sys_ * scs) / det
    b = (sys_ * scc - syc * scs) / det
    return math.degrees(math.atan2(b, a)), 1.0 / math.hypot(a, b)


def tilt_zero(samples: Sequence[Tuple[float, float]], height_m: float) -> Tuple[float, float]:
    """(tilt_deg as the head really points, i.e. after its built-in offset, range_m) pairs on a
    flat floor -> (error_deg: true = commanded + error, spread_deg)."""
    errs = [-math.degrees(math.asin(height_m / r)) - t for t, r in samples
            if r and r > height_m and t < -5.0]
    if len(errs) < 3:
        raise ValueError("need at least 3 floor readings with the head tilted down")
    return statistics.median(errs), max(errs) - min(errs)


def tilt_and_height(samples: Sequence[Tuple[float, float]]) -> Tuple[float, float, float]:
    """(true tilt as commanded, range) on a flat floor, height unknown -> (error_deg,
    height_m, spread_m): the error that makes h = r sin(-(t + e)) the same for every reading."""
    pts = [(t, r) for t, r in samples if r and t < -5.0]
    if len(pts) < 4 or max(t for t, _ in pts) - min(t for t, _ in pts) < 10.0:
        raise ValueError("need at least 4 floor readings over at least 10 degrees of tilt")
    best = None
    for k in range(-200, 201):
        e = k * 0.05
        hs = [r * math.sin(math.radians(-(t + e))) for t, r in pts]
        if min(hs) <= 0:
            continue
        m = statistics.mean(hs)
        cost = statistics.pstdev(hs) / m
        if best is None or cost < best[0]:
            best = (cost, e, m, max(hs) - min(hs))
    if best is None:
        raise ValueError("the readings do not look like a floor")
    return best[1], best[2], best[3]


def drive_fit(d0_m: float, d_end_m: float, readings: Sequence[float], moved_cmd_m: Sequence[float],
              ticks_per_cm: float) -> dict:
    """Backing away from a wall: `readings[i]` the ToF at stop i, `moved_cmd_m[i]` how far
    the robot was TOLD to have moved by then (0 first), `d0_m` / `d_end_m` the tape at the
    first and last stops. The tape says how far it really moved; in between it is taken
    in proportion. Returns ticks_per_cm and the ToF's range_scale / range_offset_m."""
    real = d_end_m - d0_m
    cmd = moved_cmd_m[-1]
    if cmd <= 0 or real <= 0.05:
        raise ValueError("the robot did not move away from the wall")
    true = [d0_m + real * c / cmd for c in moved_cmd_m]
    scale, off = range_fit(list(zip(true, readings)))
    return {"ticks_per_cm": round(ticks_per_cm * cmd / real, 2), "moved_m": round(real, 3),
            "tof_range_scale": round(scale, 4), "tof_range_offset_m": round(off, 4),
            "tof_error_cm": [round((r - t) * 100, 1) for t, r in zip(true, readings)]}


def turn_fit(commanded_deg: Sequence[float], measured_deg: Sequence[float], ticks_per_degree: float) -> dict:
    """Turns asked for and turned (gyro, absolute degrees) -> ticks_per_degree."""
    ratios = [c / m for c, m in zip(commanded_deg, measured_deg) if m > 10]
    if not ratios:
        raise ValueError("no turn measured")
    return {"ticks_per_degree": round(ticks_per_degree * statistics.mean(ratios), 3),
            "turned_deg": [round(m, 1) for m in measured_deg]}


def reverse_trim(fwd_bwd_ratios: Sequence[float], current_pct: float) -> dict:
    """Each turn's |forward track ticks| / |backward track ticks| -> REVERSE_TRIM_PCT that
    would even them (1.0 = even). Rounded to whole percent, kept within 0..40."""
    r = statistics.mean(fwd_bwd_ratios)
    pct = max(0, min(40, round(((1 + current_pct / 100.0) * r - 1) * 100)))
    return {"REVERSE_TRIM_PCT": pct, "forward_leads_by_pct": round((r - 1) * 100, 1)}


def range_fit(pairs: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    """(true_m, measured_m) pairs -> (scale, offset_m) with true = scale * measured + offset."""
    pts = [(t, m) for t, m in pairs if m]
    if len(pts) < 2:
        raise ValueError("need at least 2 distances")
    n = len(pts)
    mx = sum(m for _, m in pts) / n
    my = sum(t for t, _ in pts) / n
    sxx = sum((m - mx) ** 2 for _, m in pts)
    if sxx < 1e-9:
        raise ValueError("use distances at least 0.5 m apart")
    scale = sum((m - mx) * (t - my) for t, m in pts) / sxx
    return scale, my - scale * mx


def centre_change(deg: float, us_per_deg: float, invert: bool = False) -> float:
    """Microseconds to add to *_center_us so that the commanded zero moves by `deg`."""
    return (-1.0 if invert else 1.0) * deg * us_per_deg


def centre_zone(rows) -> Optional[float]:
    """Median of the 4 centre zones of an 8x8 grid (None if they are all empty)."""
    v = [rows[r][c] for r in (3, 4) for c in (3, 4) if rows[r][c] is not None]
    return statistics.median(v) if v else None


def door_fit(looks: Sequence[Sequence[Tuple[float, float]]], half_w_m: float = 0.125,
             wall_tol_m: float = 0.06, beyond_m: float = 0.25) -> dict:
    """looks: per head position, the ToF points (x forward, y left, metres, robot frame) on things standing up.
    The wall the door is in is the line x = a + b*y through most points (the robot roughly square to it). Along
    it, u (+ left; 0 where the robot's straight-ahead line meets it) and v (+ beyond the wall). The door is the
    widest stretch with no wall points that has points beyond it (rays went through). Each edge is also found
    look by look (a wall point next to points beyond it), so its spread shows how much one look can be off."""
    pts = np.array([p for look in looks for p in look], float).reshape(-1, 2)
    if len(pts) < 20:
        raise ValueError("too few ToF points: is the wall within 2 m and the head level?")
    x, y = pts[:, 0], pts[:, 1]
    a, b = float(np.percentile(x, 25)), 0.0              # the wall is the nearer surface (the room beyond: further)
    for tol in (0.15, 0.10, wall_tol_m, wall_tol_m):
        keep = np.abs(x - (a + b * y)) < tol
        if keep.sum() < 10:
            raise ValueError("no wall found: face the robot roughly square to the wall with the door")
        b, a = (float(v) for v in np.polyfit(y[keep], x[keep], 1))
    n = math.hypot(1.0, b)

    def uv(px, py):
        px, py = np.asarray(px, float), np.asarray(py, float)
        return ((px - a) * b + py) / n, (px - a - b * py) / n

    u, v = uv(x, y)
    wall, beyond = np.sort(u[np.abs(v) < wall_tol_m]), u[v > beyond_m]
    gaps = [(wall[i], wall[i + 1]) for i in range(len(wall) - 1) if wall[i + 1] - wall[i] > 0.2]
    gaps = [(g, int(((beyond > g[0]) & (beyond < g[1])).sum())) for g in gaps]
    gaps = [g for g in gaps if g[1] >= 3]
    if not gaps:
        raise ValueError("no door found: no stretch of the wall that rays went through")
    (u_r, u_l), _ = max(gaps, key=lambda g: (g[1], g[0][1] - g[0][0]))
    per = {"left": [], "right": []}
    for look in looks:                                   # each edge as one look sees it
        if len(look) < 3:
            continue
        lu, lv = uv(*np.array(look, float).T)
        w, bu = lu[np.abs(lv) < wall_tol_m], lu[lv > beyond_m]
        for side, sign, ref in (("left", 1.0, u_l), ("right", -1.0, u_r)):
            near = w[(np.abs(w - ref) < 0.3) & (sign * (w - (u_r + u_l) / 2.0) > 0)]
            if not len(near):
                continue
            edge = float(near.min() if sign > 0 else near.max())          # the wall point nearest the opening
            if ((sign * (edge - bu) > 0) & (sign * (edge - bu) < 0.25)).any():   # with the room beyond seen next to it
                per[side].append(edge)

    def stat(vals, ref):
        if not vals:
            return {"median_m": round(float(ref), 3), "spread_m": None, "looks": 0}
        vv = np.array(vals)
        return {"median_m": round(float(np.median(vv)), 3), "spread_m": round(float(np.percentile(vv, 90) -
                np.percentile(vv, 10)), 3), "looks": len(vv), "min_m": round(float(vv.min()), 3),
                "max_m": round(float(vv.max()), 3)}
    left, right = stat(per["left"], u_l), stat(per["right"], u_r)
    worst = (left.get("min_m", u_l) - right.get("max_m", u_r))     # what a map keeping every look's edge would hold
    ang = math.atan(b)
    foot = lambda uu: (a + b * uu / n, uu / n)       # noqa: E731 — the point on the wall at u
    return {"width_m": round(left["median_m"] - right["median_m"], 3), "width_worst_m": round(float(worst), 3),
            "left": left, "right": right, "centre_u_m": round((left["median_m"] + right["median_m"]) / 2.0, 3),
            "wall_m": round(a / n, 3), "wall_angle_deg": round(math.degrees(ang), 1),
            "clear_left_m": round(left["median_m"] - half_w_m / math.cos(ang), 3),
            "clear_right_m": round(-right["median_m"] - half_w_m / math.cos(ang), 3),
            "edge_xy": {"left": [round(c, 3) for c in foot(left["median_m"])],
                        "right": [round(c, 3) for c in foot(right["median_m"])]},
            "points": int(len(pts))}


def door_turn_fit(headings: Sequence[Tuple[float, Sequence[Sequence[Tuple[float, float]]]]],
                  half_w_m: float = 0.125) -> dict:
    """headings: (gyro yaw since the first, deg; the looks' points in the robot frame then). Rotated back by
    k * yaw, the door must land in one place: the k that makes it is the gyro's scale (1.0: right)."""
    def mismatch(k: float) -> Optional[float]:
        edges = []
        for yaw, looks in headings:
            t = math.radians(k * yaw)
            c, s = math.cos(t), math.sin(t)
            rot = [[(c * px - s * py, s * px + c * py) for px, py in look] for look in looks]
            try:
                f = door_fit(rot, half_w_m)
            except ValueError:
                return None
            edges.append(f["edge_xy"]["left"] + f["edge_xy"]["right"])
        e = np.array(edges)
        return float(np.mean(np.std(e, axis=0)))
    ks: List[Tuple[float, float]] = [(k, m) for k in np.arange(0.6, 1.41, 0.02) if (m := mismatch(float(k))) is not None]
    if not ks:
        raise ValueError("the door was not found after the turns")
    best = min(ks, key=lambda km: km[1])
    at1 = next((m for k, m in ks if abs(k - 1.0) < 1e-6), None)
    return {"gyro_scale": round(float(best[0]), 2), "spread_m": round(best[1], 3),
            "spread_at_1_m": None if at1 is None else round(at1, 3)}
