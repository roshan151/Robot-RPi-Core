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

import logging
import math
import os
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

from robot_core import robot_log, status
from robot_core.explore import ExploreConfig, Explorer
from robot_core.odometry import Pose2D
from robot_core.sensors.camera import Camera
from robot_core.sensors.gimbal import from_params
from robot_core.sensors.head import Head, HeadGeometry, parse_self_zones
from robot_core import terrain
from robot_core.sensors import tof8x8
from robot_core.sensors.tfluna import TFLuna
from robot_core.sensors.visual_motion import VisualMotionEstimator
from robot_core.vision_client import VisionClient
from robot_display.link import connect
from robot_interfaces.action import Drive, Turn
from robot_interfaces.msg import MotionHealth, VisualMotion

log = logging.getLogger("robot_explore")       # robot* loggers: INFO and up reach logs.json
STUCK_STATES = ("stalled", "slipping", "tilted", "drop")    # drop: the TF-Luna guard braked


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
            self._stuck_why = m.state

    def wait_ready(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._odom is not None and self._drive.server_is_ready() and self._turn.server_is_ready():
                return True
            time.sleep(0.2)
        return False

    def odom(self) -> Pose2D:
        return self._odom or Pose2D()

    def _run(self, client, goal, timeout: float, watch=None) -> str:
        """`watch()`: called while the move runs (it paces itself, e.g. one ToF grid);
        True cancels the move and the answer is "blocked"."""
        t0 = time.monotonic()
        handle = _wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return "rejected"
        fut = handle.get_result_async()
        blocked = False
        while watch is not None and not fut.done() and time.monotonic() - t0 < timeout:
            if watch():
                blocked = True
                _wait(handle.cancel_goal_async(), 2.0)
                break
        res = _wait(fut, max(1.0, timeout - (time.monotonic() - t0)))
        time.sleep(0.25)                   # let /odom catch up (10 Hz telemetry)
        if blocked:
            return "blocked"
        if res is None:
            return "timeout"
        status = res.result.status
        if status != "done" and self._stuck_at >= t0:
            return "drop" if getattr(self, "_stuck_why", "") == "drop" else "stuck"   # braked on motion health
        return status

    def drive(self, meters: float, watch=None) -> str:
        return self._run(self._drive, Drive.Goal(meters=float(meters)), 30.0, watch)

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
        p("orbit_views", 8)
        p("revisit_known", True)
        p("ask_before_drive", False)
        p("autostart", True)
        p("tfluna_port", "/dev/serial0")
        p("tfluna_fitted", True)
        p("outdoor_mode", False)
        p("max_slope_deg", 12.0)
        p("roughness_tol_m", 0.04)
        # 8x8 ToF on the head (robot_core/sensors/tof8x8.py): replaces the TF-Luna as the
        # range sensor and gives the floor check its grid. Wiring: README.
        p("tof_fitted", False)
        p("tof_port", "")
        p("tof_bus", 1)
        p("tof_addr", 0x33)
        p("tof_tilt", 0)
        p("tof_flip_rows", False)
        p("tof_flip_cols", False)
        p("tof_perpendicular", True)
        p("drive_tilt_deg", -30.0)
        p("head_tilt_offset_deg", 0.0)
        p("h_max_m", 0.03)
        p("flat_tol_m", 0.015)
        p("min_landing_m", 0.20)
        p("max_drop_m", 0.03)
        p("path_width_m", 0.30)
        p("robot_width_m", 0.25)
        p("tof_from_right_m", 0.125)
        p("plan_clearance_m", 0.08)
        p("floor_auto_level", False)
        p("floor_holes_are_drops", True)
        p("drive_on_unknown_floor", False)
        p("robot_height_m", 0.30)
        p("stop_short_m", 0.25)
        p("camera_main_size", [1920, 1080])
        p("camera_inverted", True)
        p("camera_backend", "auto")
        p("camera_index", 0)
        p("head_height_m", 0.25)
        p("pan_axis_x_m", 0.05)
        p("tof_self_zones", "")
        p("hfov_deg", 66.0)
        p("pwm_chip", 0)
        p("pan_channel", 1)
        p("tilt_channel", 0)
        p("pan_center_us", 1500.0)
        p("tilt_center_us", 1500.0)
        p("pan_us_per_deg", 2000.0 / 180.0)
        p("tilt_us_per_deg", 2000.0 / 180.0)
        p("pan_invert", False)
        p("tilt_invert", False)
        p("servo_speed_dps", 30.0)
        p("servo_fast_dps", 120.0)
        p("servo_min_us", 500.0)
        p("servo_max_us", 2400.0)
        p("hold_on_close", True)
        p("servo_power_gpio", 0)
        v = lambda name: self.get_parameter(name).value

        if not (v("tof_fitted") or v("tfluna_fitted")):
            # The map, plant ranging and the floor check before every step all
            # come from the range sensor; without one the robot would drive blind.
            raise SystemExit("explore needs a range sensor (robot.yaml tof_fitted / tfluna_fitted are both false)")
        self.gimbal = from_params({n: v(n) for n in (
            "pwm_chip", "pan_channel", "tilt_channel", "pan_center_us", "tilt_center_us",
            "pan_us_per_deg", "tilt_us_per_deg", "pan_invert", "tilt_invert", "servo_speed_dps", "servo_fast_dps",
            "servo_min_us", "servo_max_us", "hold_on_close", "servo_power_gpio")})
        if v("tof_fitted"):
            self.lidar = tof8x8.from_params({n: v(n) for n in ("tof_port", "tof_bus", "tof_addr", "tof_tilt", "tof_flip_rows", "tof_flip_cols")})
        else:
            self.lidar = TFLuna(v("tfluna_port"))
        self.camera = Camera(main_size=tuple(v("camera_main_size")), inverted=bool(v("camera_inverted")),
                             backend=v("camera_backend"), index=int(v("camera_index")))
        self.head = Head(self.gimbal, self.lidar, self.camera, HeadGeometry(
            height_m=float(v("head_height_m")), pan_axis_x_m=float(v("pan_axis_x_m")),
            hfov_deg=float(v("hfov_deg")), drive_tilt_deg=float(v("drive_tilt_deg")),
            tof_perpendicular=bool(v("tof_perpendicular")), tilt_offset_deg=float(v("head_tilt_offset_deg")),
            tof_pan_offset_deg=float(v("tof_pan_offset_deg") if self.has_parameter("tof_pan_offset_deg") else 0.0),
            tof_tilt_offset_deg=float(v("tof_tilt_offset_deg") if self.has_parameter("tof_tilt_offset_deg") else 0.0),
            pan_axis_y_m=float(v("tof_from_right_m")) - float(v("robot_width_m")) / 2.0,
            self_zones=parse_self_zones(v("tof_self_zones"))),
            pitch=lambda: self._pitch(),
            terrain_params=terrain.params_from({n: v(n) for n in ("h_max_m", "flat_tol_m", "min_landing_m", "max_drop_m", "robot_width_m",
                                                             "plan_clearance_m", "floor_auto_level", "floor_holes_are_drops",
                                                             "drive_on_unknown_floor", "robot_height_m", "stop_short_m", "tfluna_fitted",
                                                             "outdoor_mode", "max_slope_deg", "roughness_tol_m")}))
        self.vision = VisionClient(v("vision_url")) if v("vision_url") else VisionClient()

        self.base = RosBase(self)
        self._debug_upload(v)
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

    def _report(self, text: str, level: int = logging.INFO) -> None:
        """The node's own messages (the mission logs its own: Explorer._say), to logs.json too."""
        log.log(level, "explore: %s", text)
        self._say(text)

    def _run_mission(self) -> None:
        try:
            if not self.base.wait_ready():
                self._report("drivetrain not ready (no /odom or actions) — is firmware v5 flashed "
                             "and the robot process running?", logging.ERROR)
                self.failed = True
                return
            try:
                self._report(f"vision service: {self.vision.health().get('backend')} at {self.vision.base}")
            except Exception as exc:                        # noqa: BLE001
                self._report(f"vision service unreachable at {self.vision.base}: {exc}", logging.ERROR)
                self.failed = True
                return
            self.explorer.run()
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().error(f"mission crashed: {type(exc).__name__}: {exc}")
            log.error("explore: mission crashed: %s: %s", type(exc).__name__, exc, exc_info=True)
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

    def _debug_upload(self, v) -> None:
        """robot.yaml /voice debug_mode: camera frames, ToF grids and the mission's steps go to the Mac as a
        debug session, like follow's (Vision-And-Planning vision_service/navigation_debugging.py shows them)."""
        import yaml
        from ament_index_python.packages import get_package_share_directory
        from robot_core import debug_upload
        with open(os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")) as f:
            if not (yaml.safe_load(f).get("/voice") or {}).get("ros__parameters", {}).get("debug_mode"):
                return
        debug_upload.configure(
            True, self.vision.base,
            geometry={"camera_hfov_deg": float(v("hfov_deg")), "tof_fov_deg": 60.0, "tof_on": "gimbal",
                      "head_tilt_offset_deg": float(v("head_tilt_offset_deg")), "tilt": "real"},
            extras=lambda: {"pose": [round(a, 4) for a in (self.base.odom().x, self.base.odom().y, self.base.odom().theta)],
                            "pitch_deg": self._pitch()})

    def _pitch(self) -> Optional[float]:
        """The robot's nose-up angle from the IMU (via /motion_health), None without one."""
        m = getattr(getattr(self, "base", None), "_health", None)
        return m.pitch_deg if m is not None and m.imu_known else None

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
    # A child process of the voice node (voice_node._explore): without this, everything the mission
    # says (each keyframe, each drive, "exploration complete") reached only the console (Nov 21).
    robot_log.setup(None)
    robot_log.event("session.start", mode="explore", pid=os.getpid())
    node = ExplorerNode()
    connect(node)                        # status + warnings go to the OLED (after setup: it resets handlers)
    status.task("explore")
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
