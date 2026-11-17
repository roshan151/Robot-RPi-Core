"""Wiring and mounting check for the 8x8 ToF (DFRobot SEN0628). Run on the Pi with robot-voice stopped:

    python tests/hardware/check_tof.py            the grid in cm, once a second for 30 s
    python tests/hardware/check_tof.py --wall     head square-on to a flat wall, 0.5 to 1 m away

Wiring and `i2cdetect` are in the README ("IMU and ToF sensors"). Bus, address and
mounting come from /explorer in robot.yaml (tof_port for USB, else tof_bus and tof_addr; tof_tilt,
tof_flip_rows, tof_flip_cols). On USB, nothing in 8 s: look at the raw stream with
`python -m serial.tools.miniterm <tof_port> 115200` and send a few lines of it.
Set tof_tilt first (the board turned on its side), then check the flips.

The grid is printed as the sensor sees it: top row first, the robot's left on the left.
    head level on the floor     the BOTTOM rows read short (the floor), the top rows long
                                -> if the top rows are the short ones, set tof_flip_rows: true
    your hand at the robot's LEFT, 30 cm out
                                the LEFT columns read ~30
                                -> if the right columns do, set tof_flip_cols: true
--wall says whether the ranges are distances to the sensor's face (all 64 about equal) or
along each ray (corners about 1.2 times the centre), which is tof_perpendicular.
Then measure head_height_m (floor to the sensor, head level) and set tof_fitted: true.
"""
import statistics
import sys
import time

import yaml

from robot_core.sensors import tof8x8
from robot_core.sensors.gimbal import ROBOT_YAML


def show(g) -> None:
    for row in g.rows:
        print("  " + " ".join(" --" if v is None else f"{v * 100:3.0f}" for v in row))


def main() -> None:
    p = yaml.safe_load(ROBOT_YAML.read_text())["/explorer"]["ros__parameters"]
    where = f"USB {p['tof_port']}" if p.get("tof_port") else \
        f"bus {p.get('tof_bus', 1)}, address 0x{int(p.get('tof_addr', 0x33)):02x}"
    print(f"{where}, "
          f"tilt {p.get('tof_tilt', 0)}, flip_rows {p.get('tof_flip_rows', False)}, flip_cols {p.get('tof_flip_cols', False)}")
    tof = tof8x8.from_params(p)
    try:
        g = tof.grid(newer_than=time.monotonic(), timeout=8.0)
        if g is None:
            raise SystemExit("no grid: is the board's switch on I2C, the address 0x33, and was it "
                             "power-cycled after changing either? (i2cdetect -y 1 should show 33)")
        if "--wall" in sys.argv:
            corners = [g.rows[r][c] for r in (0, 7) for c in (0, 7)]
            centre = g.centre()
            show(g)
            if centre is None or None in corners:
                raise SystemExit("the wall does not fill the view: move closer, square-on")
            ratio = statistics.mean(corners) / centre
            print(f"corners / centre = {ratio:.2f} -> tof_perpendicular: "
                  f"{'true' if ratio < 1.1 else 'false'}  (1.0 = to the face, 1.2 = along the ray)")
            return
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < 30:
            g = tof.grid(newer_than=g.t, timeout=2.0) or g
            n += 1
            print(f"-- {time.monotonic() - t0:4.1f} s, straight ahead {'--' if g.centre() is None else f'{g.centre() * 100:.0f} cm'}")
            show(g)
            time.sleep(1.0)
        print(f"{len(tof.since(t0)) / 30:.1f} grids a second")
    finally:
        tof.close()


if __name__ == "__main__":
    main()
