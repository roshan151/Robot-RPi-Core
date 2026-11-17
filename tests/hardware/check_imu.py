"""Wiring and mounting check for the IMU (Fermion 10 DOF, BMI323). Run on the Pi with robot-voice stopped:

    python tests/hardware/check_imu.py            angles twice a second for 30 s
    python tests/hardware/check_imu.py --drift    leave the robot still: yaw drift over a minute

Wiring and `i2cdetect` are in the README ("IMU and ToF sensors"). Bus, address and
axes come from /drivetrain in robot.yaml (imu_bus, imu_gyro_addr, imu_axes).

What to look for, robot on the floor, then tilted by hand:
    level                 pitch ~0, roll ~0
    lift the FRONT        pitch goes POSITIVE
    lift the LEFT side    roll goes POSITIVE (right side down)
    turn it to the LEFT   yaw goes POSITIVE, by about the angle you turned it
If an angle moves the wrong way, or the wrong one moves, fix imu_axes: it names the
board axis (printed on the board) that points robot-forward, robot-left and up,
with "-" for one that points the other way. Then set imu_fitted: true.
"""
import sys
import time

import yaml

from robot_core.sensors.gimbal import ROBOT_YAML
from robot_core.sensors.imu import Imu


def main() -> None:
    p = yaml.safe_load(ROBOT_YAML.read_text())["/drivetrain"]["ros__parameters"]
    axes = p.get("imu_axes", ["y", "x", "-z"])
    print(f"bus {p.get('imu_bus', 1)}, BMI323 0x{int(p.get('imu_gyro_addr', 0x69)):02x}, axes {axes}")
    imu = Imu(bus=int(p.get("imu_bus", 1)), axes=axes, gyro_addr=int(p.get("imu_gyro_addr", 0x69)))
    try:
        print("keep the robot still: measuring the gyro's zero...")
        if not imu.wait_calibrated(6.0):
            print("  not still enough to zero the gyro; yaw will drift until it is")
        if "--drift" in sys.argv:
            y0 = imu.state().yaw_deg
            for left in range(60, 0, -10):
                print(f"  {left} s...")
                time.sleep(10)
            d = imu.state().yaw_deg - y0
            print(f"yaw drifted {d:+.2f} deg in 60 s ({'ok' if abs(d) < 1.0 else 'too much: was it really still?'})")
            return
        for _ in range(60):
            s = imu.state()
            print(f"pitch {s.pitch_deg:+6.1f}  roll {s.roll_deg:+6.1f}  yaw {s.yaw_deg:+7.1f}  "
                  f"turning {s.yaw_rate_dps:+6.1f} deg/s  jolt {s.jolt_g:.2f} g")
            time.sleep(0.5)
    finally:
        imu.close()


if __name__ == "__main__":
    main()
