#!/usr/bin/env python3
"""
Replay a debug-mode session (Vision-And-Planning debug/<session>/) through follow's
safety checks, offline, to see where they would have stepped in.

  python replay_debug.py ../Vision-And-Planning/debug/20261119-101426
  python replay_debug.py <session> --strict     # exit 1 if any stuck / drive-into-obstacle moment

Every frame of a session carries the odometry pose, the robot's pitch and the raw 8x8 ToF
grid. For each one this prints when:

  STUCK      nav.controller.StuckWatch fires: the encoders gained distance while something near
             ahead stayed put (tracks slipping), or the nose rose CLIMB_DEG (climbing).
  INTO       the robot moved forward while the ToF's centre zones read nearer than the
             reflex's stop distance (25 cm): a drive into an obstacle that the reflex
             (nav/reflex.py) would have braked.

The head's angle during drives is not recorded, so INTO uses the centre zones as they
read; it is a guide, not the exact brake rule (Reflex.danger).
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

from robot_core.nav.controller import StuckWatch
from robot_core.nav.reflex import Reflex

BRAKE_M = Reflex.__init__.__defaults__[1]                     # stop_m: the reflex's 25 cm


def frames(session: str):
    for path in sorted(glob.glob(os.path.join(session, "[0-9]*.json"))):
        with open(path) as f:
            d = json.load(f)
        ex = d.get("extras") or {}
        rows = (d.get("tof") or {}).get("rows")
        if ex.get("pose") is None:
            continue
        yield d["n"], d["ts"], ex["pose"], ex.get("pitch_deg"), rows, d.get("note")


def centre(rows):
    if not rows:
        return None
    vals = sorted(rows[r][c] for r in range(2, 6) for c in range(3, 5) if rows[r][c] is not None)
    if not vals:
        return None
    mid = len(vals) // 2
    return vals[mid] if len(vals) % 2 else (vals[mid - 1] + vals[mid]) / 2.0


def t_of(ts: str) -> float:
    hh, mm, ss = ts[11:23].split(":")
    return int(hh) * 3600 + int(mm) * 60 + float(ss)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--strict", action="store_true")
    a = ap.parse_args()

    watch = StuckWatch()
    odo, last, base_pitch, hits = 0.0, None, None, 0
    for n, ts, pose, pitch, rows, note in frames(a.session):
        step = 0.0 if last is None else math.hypot(pose[0] - last[0], pose[1] - last[1])
        fwd = 0.0
        if last is not None and step > 0.005:   # forward along the heading (not a turn or a reverse)
            fwd = (pose[0] - last[0]) * math.cos(last[2]) + (pose[1] - last[1]) * math.sin(last[2])
        if step < 0.005 and pitch is not None:  # standing: the pitch a drive starts from
            base_pitch = pitch
            watch = StuckWatch()
        odo += step
        c = centre(rows)
        rise = None if pitch is None or base_pitch is None else pitch - base_pitch
        why = watch.update(t_of(ts), odo, c, rise)
        tag = ""
        if why:
            tag = f"STUCK  {why} (phantom {watch.phantom_m:.2f} m)"
        elif fwd > 0.01 and c is not None and c < BRAKE_M:
            tag = f"INTO   moved {fwd:.2f} m forward with the centre zones at {c:.2f} m"
        if tag:
            hits += 1
            print(f"{n:4d} {ts[11:23]} pose=({pose[0]:+.2f},{pose[1]:+.2f},{math.degrees(pose[2]):+.0f}deg) {tag}")
        if note and note.get("state") == "tracking":
            print(f"{n:4d} {ts[11:23]}   decision: bearing {note.get('bearing_deg')}, {note.get('distance_m')} m "
                  f"({note.get('distance_from')}), turn {note.get('turn_deg')}, drive {note.get('drive_m')}")
        last = pose
    print(f"{hits} moment(s) flagged")
    return 1 if a.strict and hits else 0


if __name__ == "__main__":
    sys.exit(main())
