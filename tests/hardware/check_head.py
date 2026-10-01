"""Move the head by hand-picked angles. Edit the numbers in main(), run, watch.

  sudo systemctl stop robot-voice      # it holds the PWM channels
  python tests/hardware/check_head.py

Each look goes from level/centre to the angle and back. Limits still apply
(pan +-45, tilt -30/+45), so a bigger number just stops at the limit.
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
    print(f"went to pan={got[0]:+.0f} tilt={got[1]:+.0f}")
    time.sleep(1.0)
    g.home()                       # back to 0/0
    time.sleep(1.0)


def main():
    UP, DOWN, LEFT, RIGHT = 20, 20, 20, 20       # degrees: change these

    g = from_params(yaml.safe_load(YAML.read_text())["/explorer"]["ros__parameters"])
    try:
        look_up(g, UP)
        look_down(g, DOWN)
        look_left(g, LEFT)
        look_right(g, RIGHT)
    finally:
        g.close()                  # parks at 0/0 and releases the PWM


if __name__ == "__main__":
    main()
