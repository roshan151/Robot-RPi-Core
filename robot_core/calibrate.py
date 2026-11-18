"""Calibration maths: plain functions, so they are tested without hardware.
Monitoring-Sensors/calibrate.py collects the readings (through robot_core/calibrate_pi.py
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
"""

from __future__ import annotations

import math
import statistics
from typing import Optional, Sequence, Tuple


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
