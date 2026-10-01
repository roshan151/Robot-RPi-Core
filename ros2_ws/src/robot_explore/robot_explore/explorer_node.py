"""Explore mode's node: the head, the mission, and the glue to ROS.

    ros2 launch robot_bringup explore.launch.py      (or ./start_robot.sh explore)

What lives here (and why in this process)
-----------------------------------------
* The camera, TF-Luna and gimbal. PLAN rule #1 — pixels never cross a process
  boundary — puts the camera in the same process as everything that reads it,
  and a sweep pairs every range reading with the gimbal angle at that instant,
  so those two must share a clock and a process too.
* The mission thread (robot_core.explore.Explorer), which blocks on one step
  at a time.
* A 5 Hz visual-motion watcher that publishes /visual_motion whenever the head
  is still, so drivetrain_node's MotionHealth can tell stalled from slipping.

What does not: the drivetrain. Moves go through the /drive and /turn actions
like every other client, so the e-stop, the distance limit and stuck
detection stay with the actuator.

Frames: this node publishes map → odom (from the Mac's SLAM); drivetrain_node
publishes odom → base_link.

The head is parked at its start position on every exit path, including
Ctrl+C and systemd's SIGTERM, from destroy_node().
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_core.explore import ExploreConfig, Explorer
from robot_core.odometry import Pose2D
from robot_core.sensors.camera import Camera
from robot_core.sensors.gimbal import from_params
from robot_core.sensors.head import Head, HeadGeometry
from robot_core.sensors.tfluna import TFLuna
from robot_core.sensors.visual_motion import VisualMotionEstimator
from robot_core.vision_client import VisionClient
from robot_interfaces.action import Drive, Turn
from robot_interfaces.msg import MotionHealth, VisualMotion

STUCK_STATES = ("stalled", "slipping")


def _wait(future, timeout: float):
    ev = threading.Event()
    future.add_done_callback(lambda _f: ev.set())
    if not ev.wait(timeout):
        return None
    return future.result()


class RosBase:
    """robot_core.explore.Base over the drivetrain's actions and topics."""

    def __init__(self, node: "ExplorerNode") -> None:
        self.n = node
        self._odom: Optional[Pose2D] = None
        self._health: Optional[MotionHealth] = None
        self._stuck_at = 0.0
        self.m2o = Pose2D()
        cb = ReentrantCallbackGroup()
        self._drive = ActionClient(node, Drive, "drive", callback_group=cb)
        self._turn = ActionClient(node, Turn, "turn", callback_group=cb)
        self._reloc = node.create_client(Trigger, "odom/relocalized", callback_group=cb)
        node.create_subscription(Odometry, "odom", self._on_odom, 10, callback_group=cb)
        node.create_subscription(MotionHealth, "motion_health", self._on_health, 10, callback_group=cb)

    def _on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self._odom = Pose2D(m.pose.pose.position.x, m.pose.pose.position.y,
                            2.0 * math.atan2(q.z, q.w))

    def _on_health(self, m: MotionHealth) -> None:
        self._health = m
        if m.state in STUCK_STATES:
            self._stuck_at = time.monotonic()

    def wait_ready(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._odom is not None and self._drive.server_is_ready() and self._turn.server_is_ready():
                return True
            time.sleep(0.2)
        return False

    def odom(self) -> Pose2D:
        return self._odom or Pose2D()

    def _run(self, client, goal, timeout: float) -> str:
        t0 = time.monotonic()
        handle = _wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return "rejected"
        res = _wait(handle.get_result_async(), timeout)
        time.sleep(0.25)                   # let /odom catch up (10 Hz telemetry)
        if res is None:
            return "timeout"
        status = res.result.status
        if status != "done" and self._stuck_at >= t0:
            return "stuck"                 # drivetrain_node braked on motion health
        return status

    def drive(self, meters: float) -> str:
        return self._run(self._drive, Drive.Goal(meters=float(meters)), 30.0)

    def turn_ccw(self, radians: float) -> str:
        deg = math.degrees(radians)
        if abs(deg) < 1.0:
            return "done"
        return self._run(self._turn, Turn.Goal(degrees=float(-deg)), 30.0)   # Turn: + is RIGHT

    def set_map_to_odom(self, pose: Pose2D) -> None:
        self.m2o = pose

    def relocalized(self) -> None:
        if self._reloc.service_is_ready():
            self._reloc.call_async(Trigger.Request())


class ExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("explorer")
        p = self.declare_parameter
        p("vision_url", "")
        p("map_name", "home")
        p("continue_map", False)
        p("start_pose", [0.0, 0.0, 0.0])
        p("max_minutes", 60.0)
        p("explore_radius_m", 10.0)
        p("no_behind", False)
        p("orbit_radius_m", 0.7)
        p("orbit_views", 4)
        p("revisit_known", True)
        p("ask_before_drive", False)
        p("autostart", True)
        p("tfluna_port", "/dev/serial0")
        p("camera_main_size", [1920, 1080])
        p("camera_inverted", True)
        p("head_height_m", 0.25)
        p("pan_axis_x_m", 0.05)
        p("hfov_deg", 66.0)
        p("pwm_chip", 0)
        p("pan_channel", 1)
        p("tilt_channel", 0)
        p("pan_center_us", 1500.0)
        p("tilt_center_us", 1500.0)
        p("pan_invert", False)
        p("tilt_invert", False)
        p("servo_speed_dps", 30.0)
        v = lambda name: self.get_parameter(name).value

        self.gimbal = from_params({n: v(n) for n in (
            "pwm_chip", "pan_channel", "tilt_channel", "pan_center_us", "tilt_center_us",
            "pan_invert", "tilt_invert", "servo_speed_dps")})
        self.lidar = TFLuna(v("tfluna_port"))
        self.camera = Camera(main_size=tuple(v("camera_main_size")), inverted=bool(v("camera_inverted")))
        self.head = Head(self.gimbal, self.lidar, self.camera, HeadGeometry(
            height_m=float(v("head_height_m")), pan_axis_x_m=float(v("pan_axis_x_m")),
            hfov_deg=float(v("hfov_deg"))))
        self.vision = VisionClient(v("vision_url")) if v("vision_url") else VisionClient()

        self.base = RosBase(self)
        self._status = self.create_publisher(String, "explore/status", 10)
        self._vm_pub = self.create_publisher(VisualMotion, "visual_motion", 10)
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.1, self._publish_map_tf)
        self.create_service(Trigger, "explore/stop", self._on_stop)
        self.create_service(Trigger, "explore/start", self._on_start)

        self.explorer = Explorer(self.base, self.head, self.vision, ExploreConfig(
            map_name=v("map_name"), continue_map=bool(v("continue_map")),
            start_pose=tuple(float(x) for x in v("start_pose")),
            max_minutes=float(v("max_minutes")), explore_radius_m=float(v("explore_radius_m")),
            no_behind=bool(v("no_behind")), orbit_radius_m=float(v("orbit_radius_m")),
            orbit_views=int(v("orbit_views")), revisit_known=bool(v("revisit_known")),
            ask_before_drive=bool(v("ask_before_drive"))), on_status=self._say)

        self._stop = threading.Event()
        self.done = threading.Event()
        self.failed = False                  # exit code 1: the voice agent gestures "no" on it
        self._watcher = threading.Thread(target=self._watch_motion, name="visual-motion", daemon=True)
        self._watcher.start()
        self._mission = threading.Thread(target=self._run_mission, name="mission", daemon=True)
        if v("autostart"):
            self._mission.start()

    # ------------------------------------------------------------------ #

    def _say(self, text: str) -> None:
        self.get_logger().info(text)
        self._status.publish(String(data=text))

    def _run_mission(self) -> None:
        try:
            if not self.base.wait_ready():
                self._say("drivetrain not ready (no /odom or actions) — is firmware v5 flashed "
                          "and the robot process running?")
                self.failed = True
                return
            try:
                self._say(f"vision service: {self.vision.health().get('backend')} at {self.vision.base}")
            except Exception as exc:                        # noqa: BLE001
                self._say(f"vision service unreachable at {self.vision.base}: {exc}")
                self.failed = True
                return
            self.explorer.run()
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().error(f"mission crashed: {type(exc).__name__}: {exc}")
            self.failed = True
        finally:
            self.done.set()

    def _watch_motion(self) -> None:
        est = VisualMotionEstimator()
        while not self._stop.wait(0.2):
            if not self.gimbal.settled():
                est.reset()                 # head motion is not robot motion
                continue
            try:
                t, lores = self.camera.capture_lores()
            except Exception:                               # noqa: BLE001
                continue
            vm = est.update(lores)
            if vm is not None and self.gimbal.settled():
                self._vm_pub.publish(VisualMotion(
                    t_monotonic=t, moving=vm.moving, shift_px=vm.shift_px,
                    changed_fraction=vm.changed_fraction))

    def _publish_map_tf(self) -> None:
        m = self.base.m2o
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = m.x
        tf.transform.translation.y = m.y
        tf.transform.rotation.z = math.sin(m.theta / 2)
        tf.transform.rotation.w = math.cos(m.theta / 2)
        self._tf.sendTransform(tf)

    def _on_start(self, _req, resp):
        if self._mission.is_alive() or self.done.is_set():
            resp.success, resp.message = False, "mission already running or finished"
        else:
            self._mission.start()
            resp.success, resp.message = True, "exploring"
        return resp

    def _on_stop(self, _req, resp):
        self.explorer.stop()
        resp.success = True
        resp.message = "stopping after the current step; the map will be saved"
        return resp

    def destroy_node(self) -> bool:
        self._stop.set()
        self.explorer.stop()
        if self._mission.is_alive():
            self._mission.join(timeout=20.0)
        for close in (self.gimbal.close, self.lidar.close, self.camera.close):
            try:
                close()                    # gimbal.close() parks the head first
            except Exception as exc:                        # noqa: BLE001
                self.get_logger().error(f"shutdown: {type(exc).__name__}: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = ExplorerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # Spin until the mission ends (then exit, which ends explore mode —
        # the launch file shuts everything down with it) or until Ctrl+C.
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(1 if node.failed else 0)


if __name__ == "__main__":
    main()
