"""Move the head by hand-picked angles. Edit the numbers in main(), run, watch.

  sudo systemctl stop robot-voice      # it holds the PWM channels
  python tests/hardware/check_head.py

Nothing moves, or it spins far more than asked? Run check_servo_pulse.py first:
it tells positional servos from continuous-rotation ones using tiny raw pulses.

Each look goes from level/centre to the angle and back. Limits still apply
(pan +-90, tilt +-45, and the pulse window around your centres - the
reachable range is printed at the start), so a bigger number just stops at the
limit. At the end the head parks at centre and KEEPS HOLDING it; set
RELEASE_AT_END to let go instead (the servos go limp and the head may flop).
"""

import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
from robot_core.sensors.gimbal import from_params  # noqa: E402

YAML = ROOT / "ros2_ws/src/robot_bringup/config/robot.yaml"


def look_up(g, deg):    go(g, tilt=+deg)
def look_down(g, deg):  go(g, tilt=-deg)
def look_left(g, deg):  go(g, pan=+deg)
def look_right(g, deg): go(g, pan=-deg)


def go(g, pan=None, tilt=None):
    got = g.move_to(pan, tilt)
    print(f"went to pan={got[0]:+.0f} tilt={got[1]:+.0f}   "
          f"pulses pan={g.pan.pulse():.0f}us tilt={g.tilt.pulse():.0f}us")
    time.sleep(1.0)
    g.home()                       # back to 0/0
    time.sleep(1.0)


def main():
    UP, DOWN, LEFT, RIGHT = 20, 20, 20, 20       # degrees: change these
    PAN_CENTER_US, TILT_CENTER_US = 2300, 1750   # None = the centres in robot.yaml; a number overrides, to hunt for a new one (11 us = 1 deg)
    FIT_HORNS = False    # True: just hold 0/0 so you can fit each horn at the middle of its travel
    RELEASE_AT_END = False   # False: keep holding centre when the script ends. True: let go (servos go limp)

    p = yaml.safe_load(YAML.read_text())["/explorer"]["ros__parameters"]
    if PAN_CENTER_US is not None:
        p["pan_center_us"] = PAN_CENTER_US
    if TILT_CENTER_US is not None:
        p["tilt_center_us"] = TILT_CENTER_US
    p["hold_on_close"] = not RELEASE_AT_END
    g = from_params(p)
    print(f"centres: pan {g.pan.cfg.center_us:.0f} us, tilt {g.tilt.cfg.center_us:.0f} us   "
          f"reachable: pan {g.pan.lo:+.0f}..{g.pan.hi:+.0f}, tilt {g.tilt.lo:+.0f}..{g.tilt.hi:+.0f} deg")
    try:
        if FIT_HORNS:
            input("Holding the centre pulses. Press Enter to release... ")
            return
        look_up(g, UP)
        look_down(g, DOWN)
        look_left(g, LEFT)
        look_right(g, RIGHT)
    finally:
        g.close()                  # parks at 0/0, then holds it (unless RELEASE_AT_END)


if __name__ == "__main__":
    main()
