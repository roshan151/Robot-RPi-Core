"""
Pan/tilt head check: moves each axis through its range so you can see the directions,
the limits and the trims. Needs the Pi, the servos wired, and the PWM overlay on.

Stop whatever owns the head first (it holds the PWM channels):
  sudo systemctl stop robot-voice

  python tests/hardware/check_head.py                 # guided walk-through
  python tests/hardware/check_head.py --pan 20 --tilt 10   # go to one pose, hold, park
  python tests/hardware/check_head.py --fake          # no hardware: just run the logic

Uses the channels, trims, inversion and speed from robot.yaml (/explorer), so it checks
what the robot will actually do. What to look for:
  * +pan turns the head LEFT, +tilt looks UP. If not, set pan_invert / tilt_invert.
  * 0/0 is level and straight ahead. If not, adjust pan_center_us / tilt_center_us.
  * A request past a limit stops AT the limit (pan +-45, tilt -30/+45), never beyond.
  * The head parks at 0/0 when the test ends.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))

from robot_core.sensors.gimbal import from_params  # noqa: E402

YAML = ROOT / "ros2_ws/src/robot_bringup/config/robot.yaml"


def params() -> dict:
    import yaml
    return yaml.safe_load(YAML.read_text())["/explorer"]["ros__parameters"]


def go(g, pan=None, tilt=None, note="") -> None:
    got = g.move_to(pan, tilt)                  # blocks until it arrives
    g.wait_settled()
    print(f"  -> asked pan={pan} tilt={tilt}   went to pan={got[0]:+.0f} tilt={got[1]:+.0f}   {note}")
    time.sleep(0.6)


def walk(g) -> int:
    bad = 0

    def expect(label, got, want):
        nonlocal bad
        ok = abs(got - want) < 0.5
        bad += not ok
        print(f"     {'ok ' if ok else 'BAD'} {label}: {got:+.0f} (want {want:+.0f})")

    print("Level, straight ahead (0/0):")
    go(g, 0, 0)
    print("PAN: watch the head. +45 should be LEFT, -45 RIGHT.")
    go(g, 45, None, "<- left?")
    go(g, -45, None, "<- right?")
    go(g, 0)
    print("TILT: +45 should look UP, -30 DOWN.")
    go(g, None, 45, "<- up?")
    go(g, None, -30, "<- down?")
    go(g, None, 0)
    print("LIMITS: asking for more than allowed must stop at the limit.")
    expect("pan 90", g.move_to(90, None)[0], 45)
    expect("pan -90", g.move_to(-90, None)[0], -45)
    expect("tilt 90", g.move_to(None, 90)[1], 45)
    expect("tilt -90", g.move_to(None, -90)[1], -30)
    g.wait_settled()
    print("Sweep (smoothness: no jerks, no buzzing at the ends):")
    go(g, -45, 0)
    go(g, 45, 0)
    go(g, 0, 0)
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pan", type=float)
    ap.add_argument("--tilt", type=float)
    ap.add_argument("--fake", action="store_true", help="no hardware")
    a = ap.parse_args()

    g = from_params(params(), backend="fake" if a.fake else "hardware")
    try:
        if a.pan is not None or a.tilt is not None:
            go(g, a.pan, a.tilt)
            input("Holding. Enter to park... ")
            return 0
        bad = walk(g)
        print("\nFAILED: a limit was not respected" if bad else "\nDone. Directions and trims are for you to judge by eye.")
        return 1 if bad else 0
    finally:
        g.close()                                # parks at 0/0


if __name__ == "__main__":
    sys.exit(main())
