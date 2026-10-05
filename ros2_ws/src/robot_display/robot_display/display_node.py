"""Owns the OLED and the power watcher; draws whatever the other nodes publish."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_core import battery, robot_log, status
from robot_core.oled import Oled, open_device
from robot_core.power import PowerMonitor
from robot_display.link import TOPIC


class DisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("display")
        for name, default in (("dc_pin", 25), ("rst_pin", 17), ("spi_bus", 0), ("spi_cs", 0)):
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value                          # noqa: E731
        self.create_subscription(String, TOPIC, self._on_msg, 50)
        self.oled = Oled(opener=lambda: open_device(p("dc_pin"), p("rst_pin"), p("spi_bus"), p("spi_cs")))
        try:
            if not self.oled.start():
                self.get_logger().warning("no OLED found; still watching power")
        except Exception as e:                                             # noqa: BLE001 — the watcher matters more
            self.get_logger().warning(f"OLED failed ({e}); still watching power")
        self.power = PowerMonitor(battery_read=lambda: battery.read(retries=1, delay=0))
        self.power.start()

    def _on_msg(self, msg: String) -> None:
        try:
            status.BOARD.apply(json.loads(msg.data))
        except ValueError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    robot_log.setup(None)                       # power dips go to logs.json, screen or no screen
    robot_log.event("session.start", mode="display")
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.power.stop()
        node.oled.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
