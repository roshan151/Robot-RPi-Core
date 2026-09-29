"""behavior_node: look / follow / approach. VISION-DESIGN.md Parts 4.6-4.8, 7.

Lives in the `robot` process. Thin: robot_core.perception.behavior decides,
this node feeds it and carries the decision out.

    sub  /tracks               Tracks            what perception believes, TRACK_HZ
    sub  /perception_status    PerceptionStatus  DEGRADED (or silence > 3 s) stops everything
    pub  /gimbal/command       GimbalCommand     while a behavior is active
    pub  /vision/events        std_msgs/String   short sentences for the agent
    pub  /vision/behavior_status BehaviorStatus  1 Hz
    srv  /vision/start         StartBehavior     look | follow | approach
    srv  /vision/stop_behavior std_srvs/Trigger  stop, and cancel the step in flight
    uses /drive, /turn (actions) and /vision/clear_path (service)

Motion is one short goal at a time through the drivetrain's own action servers,
so every limit it enforces (max distance, gesture rules, e-stop) still applies.
This node never calls /estop and never overrides one.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_core import settings
from robot_core.perception.approach import ApproachConfig
from robot_core.perception.behavior import BehaviorConfig, Mode, TargetView, VisionBehavior
from robot_core.perception.follow import FollowConfig
from robot_core.perception.freespace import FreeSpace
from robot_core.perception.motion_types import Move
from robot_interfaces.action import Drive, Turn
from robot_interfaces.msg import BehaviorStatus, GimbalCommand, PerceptionStatus, Tracks
from robot_interfaces.srv import ClearPath, StartBehavior

TICK_HZ = 10.0
STATUS_STALE_S = 3.0
CLEAR_RETRY_S = 1.0


class BehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_behavior")
        s = settings
        self.declare_parameter("speed_pct", s.FOLLOW_SPEED_PCT)
        self._speed = float(self.get_parameter("speed_pct").value)
        self._b = VisionBehavior(BehaviorConfig(
            reacquire_s=s.VISION_REACQUIRE_S,
            follow=FollowConfig(stop_m=s.FOLLOW_STOP_M, standoff_m=s.FOLLOW_STANDOFF_M,
                                max_step_m=s.FOLLOW_MAX_STEP_M),
            approach=ApproachConfig(standoff_m=s.APPROACH_STANDOFF_M, max_total_m=s.MAX_APPROACH_M),
        ))
        self._lock = threading.Lock()
        self._tracks: list[TargetView] = []
        self._status_state, self._status_t = "NO_DATA", 0.0
        self._move: Optional[Move] = None
        self._goal_handle = None
        self._clear_pending, self._clear_last_t = False, 0.0
        self._was_active = False

        cb = ReentrantCallbackGroup()
        self._drive = ActionClient(self, Drive, "drive", callback_group=cb)
        self._turn = ActionClient(self, Turn, "turn", callback_group=cb)
        self._clear = self.create_client(ClearPath, "vision/clear_path", callback_group=cb)

        self._pub_gimbal = self.create_publisher(GimbalCommand, "gimbal/command", 10)
        self._pub_events = self.create_publisher(String, "vision/events", 10)
        self._pub_status = self.create_publisher(BehaviorStatus, "vision/behavior_status", 5)

        self.create_subscription(Tracks, "tracks", self._on_tracks, 5, callback_group=cb)
        self.create_subscription(PerceptionStatus, "perception_status", self._on_status, 5,
                                 callback_group=cb)
        self.create_service(StartBehavior, "vision/start", self._srv_start, callback_group=cb)
        self.create_service(Trigger, "vision/stop_behavior", self._srv_stop, callback_group=cb)

        tick = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / TICK_HZ, self._tick, callback_group=tick)
        self.create_timer(1.0, self._publish_status, callback_group=tick)

    # -------------------------------------------------------------- inputs

    def _on_tracks(self, msg: Tracks) -> None:
        now = time.monotonic()
        views = []
        for o in msg.tracks:
            views.append(TargetView(
                track_id=int(o.id), cls=o.cls, identity=o.identity or None,
                bearing_deg=float(o.bearing_deg), aim_elevation_deg=float(o.aim_elevation_deg),
                distance_m=None if math.isnan(o.distance_m) else float(o.distance_m),
                lost=bool(o.lost), anchor_t=now - float(o.anchor_age_s),
                bottom_v_frac=float(o.bottom_v), area_frac=float(o.area_frac),
                confidence=float(o.confidence),
            ))
        with self._lock:
            self._tracks = views

    def _on_status(self, msg: PerceptionStatus) -> None:
        self._status_state, self._status_t = msg.state, time.monotonic()

    # ------------------------------------------------------------ services

    def _srv_start(self, req, res):
        now = time.monotonic()
        kind, target = req.behavior.strip().lower(), req.target.strip().lower()
        with self._lock:
            if kind == "follow":
                self._b.follow(target or None, now)
            elif kind == "approach":
                if not target:
                    res.ok, res.message = False, "approach needs an object, e.g. 'chair'"
                    return res
                self._b.approach(target, now)
            elif kind == "look":
                self._b.look(target or "person", now)
            else:
                res.ok, res.message = False, f"unknown behavior {req.behavior!r}"
                return res
        res.ok, res.message = True, f"{kind} {target or 'whoever is there'}: started"
        return res

    def _srv_stop(self, _req, res):
        with self._lock:
            self._b.stop("stopped on request")
        self._cancel_goal()
        self._pub_gimbal.publish(GimbalCommand(mode="hold"))
        res.success, res.message = True, "stopped"
        return res

    # ---------------------------------------------------------------- tick

    def _tick(self) -> None:
        now = time.monotonic()
        degraded = self._status_state != "OK" or now - self._status_t > STATUS_STALE_S
        with self._lock:
            active_before = self._b.mode != Mode.IDLE
            d = self._b.tick(list(self._tracks), motion_busy=self._move is not None,
                             degraded=degraded and active_before, now=now)
            active = self._b.mode != Mode.IDLE

        for text in d.events:
            self.get_logger().info(f"event: {text}")
            self._pub_events.publish(String(data=text))

        if active or d.gimbal[0] == "home":
            g = d.gimbal
            cmd = GimbalCommand(mode=g[0])
            if g[0] == "track":
                cmd.pan_deg, cmd.tilt_deg = float(g[1]), float(g[2])
            elif g[0] == "sweep":
                cmd.pan_deg = float(g[1])
            self._pub_gimbal.publish(cmd)
        elif self._was_active:
            self._pub_gimbal.publish(GimbalCommand(mode="hold"))
        self._was_active = active

        if d.want_clearance and not self._clear_pending and now - self._clear_last_t > CLEAR_RETRY_S:
            self._request_clearance(d.want_clearance, d.corridor_top_v)
        if d.move is not None and self._move is None:
            self._send(d.move)

    # ------------------------------------------------------------ clearance

    def _request_clearance(self, mode: str, top_v: Optional[float]) -> None:
        if not self._clear.service_is_ready():
            return
        self._clear_pending = True
        req = ClearPath.Request(mode=mode, corridor_top_v=float(top_v or 0.0))
        self._clear.call_async(req).add_done_callback(self._on_clearance)

    def _on_clearance(self, fut) -> None:
        now = time.monotonic()
        try:
            r = fut.result()
            fs = FreeSpace([], float(r.best_bearing_deg) if r.has_best else None,
                           bool(r.ahead_clear) if (r.ok and r.known) else None,
                           note=r.summary)
        except Exception as exc:                          # noqa: BLE001
            fs = FreeSpace([], None, None, note=str(exc))
        with self._lock:
            self._b.set_clearance(fs, now)
        self._clear_pending, self._clear_last_t = False, now

    # --------------------------------------------------------------- motion

    def _send(self, move: Move) -> None:
        if move.kind == "turn":
            client, goal = self._turn, Turn.Goal(degrees=float(move.value), speed_pct=self._speed)
        elif move.kind == "drive":
            client, goal = self._drive, Drive.Goal(meters=float(move.value), speed_pct=self._speed)
        else:
            return
        if not client.server_is_ready():
            self._finish(move, "failed")
            return
        self._move = move
        self.get_logger().info(f"step: {move}")
        client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal(f, move))

    def _on_goal(self, fut, move: Move) -> None:
        try:
            handle = fut.result()
        except Exception:                                 # noqa: BLE001
            self._finish(move, "failed")
            return
        if not handle.accepted:
            self._finish(move, "rejected")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, move))

    def _on_result(self, fut, move: Move) -> None:
        try:
            status = fut.result().result.status or "done"
        except Exception:                                 # noqa: BLE001
            status = "failed"
        self._finish(move, status)

    def _finish(self, move: Move, status: str) -> None:
        with self._lock:
            self._b.move_finished(move, status, time.monotonic())
        self._move, self._goal_handle = None, None

    def _cancel_goal(self) -> None:
        h = self._goal_handle
        if h is not None:
            h.cancel_goal_async()

    # --------------------------------------------------------------- status

    def _publish_status(self) -> None:
        with self._lock:
            s = self._b.status
        m = BehaviorStatus(mode=s["mode"], target=s["target"] or "",
                           track_id=int(s["track_id"]) if s["track_id"] is not None else -1,
                           travelled_m=float(s["travelled_m"]), last_reason=s["last_reason"])
        self._pub_status.publish(m)


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = BehaviorNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
