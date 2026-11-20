"""Follow an enrolled person, as a job the voice agent runs with its session closed.

Two loops, at their own rates:

  * perception (this module, as fast as the Mac answers, ~2-4 Hz): the newest camera frame
    (nav.stream, which also streams every frame to the Mac for visual SLAM) goes to
    POST /v1/follow/step; the Mac finds the person and says where they are (bearing in the
    robot's frame, distance with how sure it is). `PersonTracker` places them in the
    odometry frame and smooths that; the result becomes the nav goal ("stop FOLLOW_KEEP_M
    short of them") and the head's focus.
  * navigation (nav.LocalNav, 20 Hz, on its own): rolling costmap from every ToF grid, DWA
    toward the goal along the Mac's Theta* waypoints, the 50 Hz reflex, and the gimbal kept on
    the person with a little look-ahead and sweep so the costmap keeps learning the way.
    See docs/FOLLOW-DESIGN.md and robot_core/nav/.

Identity, locking and searching are the Mac's (vision_service/follow.py). Lost: the robot
goes to where they were last seen (once per loss), then stands while the head does the
Mac's search sweep. "near" (they fill the frame) means close: the robot stands and faces
them. Ends when the Mac says the person is gone, after `seconds`, or when the task is
cancelled ("stop" to the cancel-only voice session, or the task/stop service). Returns True
if it locked onto the person at all.

Needs the 8x8 ToF on the head and the drivetrain's velocity mode (firmware -vel): without
them there is no safe way to drive among obstacles, and follow says so and stops.
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from typing import Callable, Optional, Tuple

from robot_core import debug_upload, settings, status, task_cancel
from robot_core.nav.controller import LocalNav
from robot_core.nav.stream import FrameStream
from robot_core.sensors.camera import jpeg
from robot_core.sensors.tof8x8 import range_in_box
from robot_core.vision_client import VisionClient, VisionError

log = logging.getLogger(__name__)

LAST_SEEN_S = 30.0              # where they were last seen is worth going to / a search hint this long
LOST_KEEP_M = 0.3               # lost: go to this far short of where they were last seen
LOST_GO_S = 10.0                # ...for at most this long before the head search takes over
LOST_AHEAD_M = (0.5, 1.2)       # walking when lost: go this much further along their way (by their speed)
LOST_FACE_S = 2.0               # then turn to face the way they went, for at most this long
NEAR_M = 0.7                    # seen within FOLLOW_KEEP_M + this, but the way to them goes round something: face, stand
WALK_MPS = 0.6                  # how fast a tracked person may have moved, for the tracker's doubt
PERSON_CLEAR_M = 0.35           # round the person: their legs, not an obstacle
RANGE_JUMP_M = 0.4              # a ToF range this much nearer than the tracked person is something in front of them
NO_POSE_S = 3.0                 # no fresh odometry this long: stop following rather than steer blind

Pose = Tuple[float, float, float]   # x, y (m), heading (rad, CCW) in the odometry frame


class PersonTracker:
    """Where the person stands, in the odometry frame, from the Mac's measurements.

    Each tracking answer places them from the robot's pose when the frame was taken:
    `distance_m` along `bearing_deg`. Readings are combined by how sure each is
    (distance_sigma_m: a ToF range counts for far more than a box-size guess), and the
    estimate's doubt grows by WALK_MPS for every second it is not seen, so a person who
    walked on is taken at their new place. A reading far outside the doubt (a lookalike, a
    bad range) is ignored once; twice running, it is believed."""

    def __init__(self) -> None:
        self.xy: Optional[Tuple[float, float]] = None
        self.var = 0.0
        self.t = 0.0
        self.rejected = 0
        self.odd: Optional[Tuple[float, float]] = None
        self.trail: deque = deque(maxlen=30)              # (t, xy) after each reading: which way they walk

    def walking(self) -> Optional[Tuple[float, float, float]]:
        """(unit x, unit y, m/s) of how they moved over their last ~2 s seen, or None if standing."""
        pts = [p for p in self.trail if self.trail and self.trail[-1][0] - p[0] <= 2.0]
        if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.5:
            return None
        dx, dy = pts[-1][1][0] - pts[0][1][0], pts[-1][1][1] - pts[0][1][1]
        dist, dt = math.hypot(dx, dy), pts[-1][0] - pts[0][0]
        return (dx / dist, dy / dist, dist / dt) if dist / dt > 0.15 else None

    def update(self, here: Pose, bearing_deg: float, dist_m: float, sigma_m: Optional[float],
               t: float) -> None:
        a = here[2] + math.radians(bearing_deg)
        z = (here[0] + dist_m * math.cos(a), here[1] + dist_m * math.sin(a))
        r = max(0.03, sigma_m if sigma_m else 0.3 * dist_m) ** 2
        if self.xy is None:
            self.xy, self.var, self.t = z, r, t
            self.trail.append((t, z))
            return
        var = self.var + (WALK_MPS * max(0.0, t - self.t)) ** 2
        gap = math.hypot(z[0] - self.xy[0], z[1] - self.xy[1])
        if gap > 3.0 * math.sqrt(var + r) + 0.3:
            # Twice running, at the same new place: it is real, start again there. Odd readings that
            # disagree with each other (a range flipping between the person and the wall) are not.
            if self.rejected < 1 or math.hypot(z[0] - self.odd[0], z[1] - self.odd[1]) > 0.3 + 0.2 * dist_m:
                self.rejected, self.odd = 1, z
                return
            self.xy, self.var, self.t, self.rejected = z, r, t, 0
            self.trail.clear()                            # somewhere new: no walk across the jump
            self.trail.append((t, z))
            return
        k = var / (var + r)
        self.xy = (self.xy[0] + k * (z[0] - self.xy[0]), self.xy[1] + k * (z[1] - self.xy[1]))
        self.var, self.t, self.rejected = (1.0 - k) * var, t, 0
        self.trail.append((t, self.xy))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _planner(vision: VisionClient, radius_m: float):
    """The Mac's global planner as LocalNav wants it: (pose, goal, local costmap) -> waypoints."""
    def plan(pose, goal, local):
        return vision.nav_plan(pose, goal, radius_m, local)
    return plan


def _nav_tilt(head) -> float:
    """The head angle while driving: FOLLOW_NAV_TILT_DEG above the real horizon (the head
    looks tilt_offset_deg up at its straight position). Level, the camera sees the person's
    body and the ToF the floor from ~0.4 m out, up past the robot's height."""
    return settings.FOLLOW_NAV_TILT_DEG - getattr(head.geom, "tilt_offset_deg", 0.0)


def follow(camera, vision: VisionClient, motion, name: str = "", head=None,
           seconds: float = settings.FOLLOW_SECONDS,
           clock: Callable[[], float] = time.monotonic,
           pose: Optional[Callable[[], Optional[Pose]]] = None,
           bumped: Optional[Callable[[], float]] = None,
           twist: Optional[Callable[[], Optional[Tuple[float, float]]]] = None) -> bool:
    """Follow `name` (empty: whichever enrolled person is seen first).

    `pose()`: the robot's (x, y, heading) in the odometry frame, or None (unknown).
    `bumped()`: time.monotonic() of the last bump/stall/slip/climb the drivetrain reported.
    `twist()`: the measured (v m/s, w rad/s), or None."""
    pose = pose or (lambda: None)
    if not (head and getattr(head, "has_grid", False) and getattr(motion, "can_glide", lambda: False)()):
        why = "needs the 8x8 ToF and velocity firmware"
        log.warning("follow failed: %s", why)
        status.error("F02", why)
        status.result("follow failed")
        return False
    if not getattr(head.gimbal, "powered", lambda: True)():
        # Nov 19: servo supply off, the head stayed frozen, the ToF saw the person and the wall in turn and the
        # robot stalled into things 5 times in 12 s. Without the head there is no way to see the path.
        why = "head servos have no power"
        log.warning("follow failed: %s", why)
        status.error("F02", why)
        status.result("follow failed")
        return False
    if vision.follow_start(name) is None:
        why = f"{name or 'nobody'} not enrolled"
        log.warning("follow failed: %s", why)
        status.error("F01", why)
        status.detail(why)
        status.result("follow failed")
        return False
    nav = LocalNav(head, motion, pose, twist=twist, bumped=bumped, tilt_deg=_nav_tilt(head))
    nav.planner = _planner(vision, nav.params.radius_m)
    stream = FrameStream(camera, head.gimbal, nav.pose, send=getattr(vision, "nav_frame", None),
                         tilt_offset_deg=head.geom.tilt_offset_deg, tof=head.lidar.grid).start()
    nav.start()
    nav.look(0.0, settings.FOLLOW_TILT_DEG)              # face height until locked on
    status.timer(f"follow {name}".strip(), seconds)
    res: dict = {}
    why = ""
    tracker = PersonTracker()
    last_seen: Optional[Tuple[float, float]] = None       # (heading of the person in the odom frame, when)
    lost_go: Optional[float] = None                       # when it set off to where they were last seen
    lost_done = False                                     # went there already, this loss
    lost_xy = lost_dir = None                             # where to go when lost, and which way they walked
    lost_face: Optional[float] = None                     # when it began turning to face their way
    t0, seq, last_frame = clock(), 0, 0
    pose_ok_t = clock()                                   # when there last was a fresh odometry pose
    try:
        while clock() - t0 < seconds and not task_cancel.requested():
            frame, here = stream.latest(newer_than=last_frame)
            last_frame = frame.seq
            seq += 1
            if here is not None:
                pose_ok_t = clock()
            elif clock() - pose_ok_t > NO_POSE_S:
                # Nov 19: /odom stopped, the pose froze, and nav kept "driving" toward a person it placed from a
                # heading that no longer changed: the head sat at its +81 deg limit while the body never turned.
                raise RuntimeError(f"no odometry for {NO_POSE_S:.0f} s (drivetrain telemetry stopped)")
            h, w = frame.main.shape[:2]
            grid = head.lidar.grid()
            rng = range_in_box(grid, res.get("bbox"), w, h, head.geom.hfov_deg,
                               head.geom.tof_pan_offset_deg, head.geom.tof_tilt_offset_deg) if grid else None
            hint = None                                   # where they were last seen, from here (+ left)
            if last_seen and here and time.monotonic() - last_seen[1] < LAST_SEEN_S:
                hint = round(math.degrees(_wrap(last_seen[0] - here[2])), 1)
            img = jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE)
            res = vision.follow_step(
                img, seq, pan_deg=frame.pan_deg, hfov_deg=head.geom.hfov_deg,
                tilt_deg=frame.tilt_deg + head.geom.tilt_offset_deg,     # the REAL tilt: the head looks up at 0
                camera_height_m=head.geom.height_m, range_m=rng,
                tof_grid=grid.rows if grid is not None else None, hint_deg=hint, pose=here,
                tof_pan_offset_deg=head.geom.tof_pan_offset_deg,
                tof_tilt_offset_deg=head.geom.tof_tilt_offset_deg)
            state = res["state"]
            gated = None                                  # a ToF range the Pi refused this step (debug record)
            log.info("follow %d: %s %s, bearing %s, %s m (%s); nav %s %s%s", seq, state, res.get("target") or "-",
                     res.get("bearing_deg"), res.get("distance_m"), res.get("distance_from"), nav.state, nav.detail,
                     f"; mac: {res['range_note']}" if res.get("range_note") else "")
            status.detail(f"{state} {res.get('distance_m') or ''}")
            if state == "gone":
                break
            if state == "tracking" and here and res.get("bearing_deg") is not None:
                b, d = res["bearing_deg"], res.get("distance_m") or rng
                if d and tracker.xy is not None and res.get("distance_from") in ("tof", "range") \
                        and d < math.dist(here[:2], tracker.xy) - RANGE_JUMP_M:
                    gated = (round(d, 2), round(math.dist(here[:2], tracker.xy), 2))
                    log.info("follow %d: %s %.2f m refused: tracked at %.2f m (something in front of them)",
                             seq, res.get("distance_from"), *gated)
                    # The ToF zones under last frame's box hit something between us and them (a chair leg, the
                    # door frame): nobody steps 0.4 m nearer in one frame. Believed twice, it moved the person onto
                    # the obstacle, nav "arrived" 0.2 m from it and the person-ignore circle erased it from the
                    # costmap (Nov 19: 39 such readings in one run). Skip it; the next box/floor range will do.
                    d = None
                last_seen = (here[2] + math.radians(b), time.monotonic())
                lost_go, lost_done, lost_face = None, False, None
                nav.release_head()
                if res.get("distance_from") == "near":    # they fill the frame: close enough, face them
                    a = here[2] + math.radians(b)
                    spot = (here[0] + math.cos(a), here[1] + math.sin(a))
                    nav.set_goal(spot, keep_m=1.0)
                    nav.set_focus(spot, PERSON_CLEAR_M)
                elif d:
                    tracker.update(here, b, d, res.get("distance_sigma_m"), time.monotonic())
                    # Stand and face only a person standing still beside something; one walking on (through a
                    # door, round a corner) is followed the long way round, or they are lost behind it.
                    nav.set_goal(tracker.xy, keep_m=settings.FOLLOW_KEEP_M,
                                 near_m=NEAR_M if tracker.walking() is None else 0.0)
                    nav.set_focus(tracker.xy, PERSON_CLEAR_M)
            elif state == "searching" and res.get("locked") and not lost_done and tracker.xy is not None \
                    and last_seen and time.monotonic() - last_seen[1] < LAST_SEEN_S:
                # Lost them: go to where they were last seen (round a corner, through a door) first.
                # Walking when lost (round a corner): further on, the way they were going; the head looks
                # there. Then turn to face that way, and only then hand over to the head search (Nov 19:
                # it stopped where they were last seen and stood, while they had walked on round the wall).
                now = time.monotonic()
                if lost_go is None:
                    lost_go, lost_dir = now, tracker.walking()
                    ahead = min(LOST_AHEAD_M[1], max(LOST_AHEAD_M[0], lost_dir[2] * 1.5)) if lost_dir else 0.0
                    lost_xy = (tracker.xy[0] + (lost_dir[0] * ahead if lost_dir else 0.0),
                               tracker.xy[1] + (lost_dir[1] * ahead if lost_dir else 0.0))
                    log.info("follow %d: lost, going to where they were last seen%s", seq,
                             f" and {ahead:.1f} m on, the way they walked" if lost_dir else "")
                    status.detail("going where last seen")
                    nav.release_head()
                    nav.set_focus(lost_xy, 0.1)
                if lost_face is None and (nav.arrived() or now - lost_go > LOST_GO_S):
                    lost_face = now
                    nav.set_focus(None)
                if lost_face is None:
                    nav.set_goal(lost_xy, keep_m=LOST_KEEP_M, face=False)
                elif lost_dir and here and now - lost_face < LOST_FACE_S and abs(
                        _wrap(math.atan2(lost_dir[1], lost_dir[0]) - here[2])) > math.radians(15):
                    nav.set_goal((here[0] + lost_dir[0], here[1] + lost_dir[1]), keep_m=1.0)   # turn, stay
                else:
                    lost_done = True
                    nav.set_goal(None)
            elif state == "searching":
                look = res.get("head") or [0, 0]          # the Mac's search sweep: the robot stands, the head looks
                nav.set_goal(None)
                if res.get("turn_deg") and here:          # they were behind it: turn that way (+ right)
                    a = here[2] - math.radians(res["turn_deg"])
                    nav.set_goal((here[0] + math.cos(a), here[1] + math.sin(a)), keep_m=1.0)
                nav.look(look[0], settings.FOLLOW_TILT_DEG + look[1])
            debug_upload.offer_event(img, {                 # debug mode: this frame, the Mac's answer, the decision
                "task": "follow", "seq": seq, "state": state, "target": res.get("target"),
                "bearing_deg": res.get("bearing_deg"), "distance_m": res.get("distance_m"),
                "distance_from": res.get("distance_from"), "range_m": rng, "pose": here, "pose_now": nav.pose(),
                "goal": nav.goal, "nav": nav.state, "nav_detail": nav.detail, "cmd": nav.cmd,
                "waypoints": nav.waypoints[:6], "reflex_trips": nav.reflex.trips, "head": res.get("head"),
                "range_note": res.get("range_note"), "gated": gated, "near_m_used": nav.near_m,
                "tracker": None if tracker.xy is None else {"xy": [round(v, 3) for v in tracker.xy],
                                                            "sigma": round(math.sqrt(tracker.var), 3),
                                                            "walking": tracker.walking()},
                "nav_source": nav.source, "marked": nav.marked[-3:]},
                grid.rows if grid is not None else None, frame.seq)
    except (VisionError, RuntimeError) as exc:       # RuntimeError: camera or drivetrain gave up
        log.warning("follow ended: %s", exc)
        why = f"vision error: {exc}" if isinstance(exc, VisionError) else str(exc)
    finally:
        nav.stop()
        stream.stop()
        try:
            vision.follow_stop()
        except VisionError:
            pass
    followed = bool(res.get("followed"))
    if task_cancel.requested():
        log.info("follow cancelled (%s)", task_cancel.by())
        status.result("follow cancelled")
        return followed
    if not followed:
        if not why:
            who = name or "anyone"
            why = (f"searched, {who} not found" if res.get("state") == "gone"
                   else f"time up, {who} not found" if res else "no answer from vision")
        log.warning("follow failed: %s", why)
        status.error("F01", why)
        status.detail(why)
    status.result(f"followed {res.get('target')}" if followed else "follow failed")
    return followed
