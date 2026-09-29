"""gimbal_node: the pan/tilt head. VISION-DESIGN.md Parts 4.4 and 4.5.

Lives in the `robot` process (light, latency-sensitive, touches no pixels).
Owns the two servos and a GimbalController, ticks at 20 Hz.

    sub  /gimbal/command   GimbalCommand   from behavior_node: track | sweep | hold | home
    pub  /gimbal/state     GimbalState     20 Hz, commanded angles + settled; perception
                                           stamps every frame with these
    srv  /aim              Aim             manual aim (the agent's look_around, or you)

If the servos can't be opened (not a Pi, overlay missing, rpi-hardware-pwm not
installed) it logs one clear error and runs with NullPanTilt, so the rest of the
graph still comes up — bearings then assume the head is straight ahead.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from robot_core import settings
from robot_core.perception.gimbal import GimbalConfig, GimbalController
from robot_core.servo import NullPanTilt, PanTilt, ServoConfig
from robot_interfaces.msg import GimbalCommand, GimbalState
from robot_interfaces.srv import Aim

TICK_HZ = 20.0


class GimbalNode(Node):
    def __init__(self) -> None:
        super().__init__("gimbal")
        s = settings
        cfg = GimbalConfig(
            pan_min=s.GIMBAL_PAN_MIN, pan_max=s.GIMBAL_PAN_MAX,
            tilt_min=s.GIMBAL_TILT_MIN, tilt_max=s.GIMBAL_TILT_MAX,
            max_rate_dps=s.GIMBAL_MAX_RATE_DPS, settle_s=s.GIMBAL_SETTLE_MS / 1000.0,
        )
        self.declare_parameter("enabled", True)
        self._ctl = GimbalController(cfg)
        self._mode = "home"
        self._hw = self._open_servos() if self.get_parameter("enabled").value else NullPanTilt()
        self._hw.set(self._ctl.pan, self._ctl.tilt)

        self._pub = self.create_publisher(GimbalState, "gimbal/state", 10)
        self.create_subscription(GimbalCommand, "gimbal/command", self._on_command, 10)
        self.create_service(Aim, "aim", self._on_aim)
        self._last = time.monotonic()
        self.create_timer(1.0 / TICK_HZ, self._tick)

    def _open_servos(self):
        s = settings
        try:
            hw = PanTilt(
                pan=ServoConfig(channel=1, min_deg=s.GIMBAL_PAN_MIN, max_deg=s.GIMBAL_PAN_MAX,
                                trim_deg=s.GIMBAL_PAN_TRIM, invert=s.GIMBAL_PAN_INVERT),
                tilt=ServoConfig(channel=0, min_deg=s.GIMBAL_TILT_MIN, max_deg=s.GIMBAL_TILT_MAX,
                                 trim_deg=s.GIMBAL_TILT_TRIM, invert=s.GIMBAL_TILT_INVERT),
                chip=s.GIMBAL_PWM_CHIP,
            )
            self.get_logger().info("pan/tilt servos on hardware PWM (GPIO13 pan, GPIO12 tilt)")
            return hw
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().error(
                f"servos unavailable ({type(exc).__name__}: {exc}). Check dtoverlay=pwm-2chan "
                f"in /boot/firmware/config.txt and `pip install rpi-hardware-pwm`. "
                f"Running without a head.")
            return NullPanTilt()

    def _on_command(self, msg: GimbalCommand) -> None:
        now = time.monotonic()
        mode = msg.mode
        if mode == "track":
            self._ctl.track(msg.pan_deg, msg.tilt_deg)
        elif mode == "sweep":
            if not self._ctl.sweeping or self._mode != "sweep":
                self._ctl.start_sweep(msg.pan_deg, now)
        elif mode == "home":
            self._ctl.home()
        else:
            self._ctl.hold()
            mode = "hold"
        self._mode = mode

    def _on_aim(self, req, res):
        self._ctl.track(req.pan_deg, req.tilt_deg)
        self._mode = "aim"
        c = self._ctl.cfg
        res.ok = True
        res.pan_deg = max(c.pan_min, min(c.pan_max, req.pan_deg))
        res.tilt_deg = max(c.tilt_min, min(c.tilt_max, req.tilt_deg))
        return res

    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last = now - self._last, now
        pan, tilt = self._ctl.tick(now, dt)
        self._hw.set(pan, tilt)
        m = GimbalState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.pan_deg, m.tilt_deg = float(pan), float(tilt)
        m.settled = self._ctl.settled(now)
        m.mode = self._mode
        self._pub.publish(m)

    def destroy_node(self) -> bool:
        try:
            self._hw.set(0.0, self._ctl.cfg.home_tilt)
            time.sleep(0.3)
            self._hw.relax()
        except Exception:                                 # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
