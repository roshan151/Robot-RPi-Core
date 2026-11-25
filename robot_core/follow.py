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

from robot_core import debug_upload, health, settings, status, task_cancel
from robot_core.rules import rules
from robot_core.nav.controller import GLOBAL_FRESH_S, LocalNav
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
TOF_PAIR_S = 0.35               # a ToF grid further than this in time from the camera frame does not range its boxes
WALK_MAX_MPS = 1.5              # nobody moves faster: a tracked jump beyond this (+0.5 m) is rule goal_jump

# Part B (head_still_while_driving): four-phase follow (Watch / Align / Go / Search)
GO_CONE_DEG = 15.0              # Watch goes straight to Go inside this; Align outside it
GO_EXIT_DEG = 30.0              # Go stops to look when person is further off-centre
STRAIGHT_MPS = 0.2              # minimum speed for "walking in a steady direction"
STRAIGHT_TURN_DEG = 25.0        # maximum direction change over STRAIGHT_S
STRAIGHT_S = 1.5                # window for steady-walk check
STRAIGHT_MIN_S = 0.8            # minimum track duration for steady-walk check
TURN_EXIT_DEG = 30.0            # Go stops when person changes direction this much in TURN_EXIT_S
TURN_EXIT_S = 1.0
GO_MIN_S = 1.0                  # shortest Go before bearing/turn exits trigger (safety stops immediate)
ALIGN_DONE_DEG = 10.0           # Align hands over to Go within this heading error
LOST_UNSEEN_S = 0.5             # Go hands over to lost-ahead and then Search after this long unseen
GO_BLOCKED_S = 3.0              # Go stops for "blocked" only once it lasts this long: until then the nav is turning
                                # toward the way or backing off, and clearing its goal cancelled that recovery
ALIGN_MAX_S = 4.0               # Align that has not lined up by then (the turn is blocked) goes back to Watch

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

    def straight(self, window_s: float = STRAIGHT_S,
                 min_s: float = STRAIGHT_MIN_S) -> Optional[Tuple[float, float, float]]:
        """(speed_mps, turn_deg, span_s) comparing first and second halves of the last `window_s` of trail.
        None if the trail spans less than `min_s` or has fewer than 3 points."""
        if not self.trail:
            return None
        t_end = self.trail[-1][0]
        pts = [p for p in self.trail if t_end - p[0] <= window_s]
        if len(pts) < 3:
            return None
        span = pts[-1][0] - pts[0][0]
        if span < min_s:
            return None
        mid = len(pts) // 2
        d1x, d1y = pts[mid][1][0] - pts[0][1][0], pts[mid][1][1] - pts[0][1][1]
        d2x, d2y = pts[-1][1][0] - pts[mid][1][0], pts[-1][1][1] - pts[mid][1][1]
        dx, dy = pts[-1][1][0] - pts[0][1][0], pts[-1][1][1] - pts[0][1][1]
        speed = math.hypot(dx, dy) / max(span, 1e-3)
        if math.hypot(d1x, d1y) < 0.05 or math.hypot(d2x, d2y) < 0.05:
            turn_deg = 0.0
        else:
            turn_deg = abs(math.degrees(_wrap(math.atan2(d2y, d2x) - math.atan2(d1y, d1x))))
        return speed, turn_deg, span

    def turn_rate(self, window_s: float = TURN_EXIT_S) -> float:
        """Direction change (deg) between the first and second halves of the last `window_s` of trail."""
        st = self.straight(window_s=window_s, min_s=0.4)
        return st[1] if (st is not None and st[0] >= 0.15) else 0.0

    def still_s(self) -> float:
        """Seconds the person has stayed within 0.15 m of their latest tracked position."""
        if not self.trail or self.xy is None:
            return 0.0
        t_end, cur = self.trail[-1][0], self.xy
        t0 = t_end
        for t, xy in reversed(self.trail):
            if math.dist(xy, cur) > 0.15:
                break
            t0 = t
        return max(0.0, t_end - t0)

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


def refuse_range(tracker: PersonTracker, here: Optional[Pose], d: Optional[float],
                 source: Optional[str]) -> Optional[str]:
    """Why the Pi will not take range `d` (m, found by `source`) as the person's distance, or None.
    A ToF/range reading RANGE_JUMP_M nearer than where they are tracked hit something between the
    robot and them: nobody steps 0.4 m nearer in one frame (Nov 19: believed twice, it moved the
    person onto a chair leg). Monitoring-Service test_replay.py runs recorded sessions through this."""
    if d is None or here is None or tracker.xy is None or source not in ("tof", "range"):
        return None
    td = math.dist(here[:2], tracker.xy)
    if d < td - RANGE_JUMP_M:
        return f"{source} {d:.2f} m refused: tracked at {td:.2f} m (something in front of them)"
    return None


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _planner(vision: VisionClient, radius_m: float):
    """The Mac's global planner as LocalNav wants it: (pose, goal, local costmap, target) -> waypoints."""
    def plan(pose, goal, local, target=None):
        return vision.nav_plan(pose, goal, radius_m, local, target=target)
    return plan


def _nav_tilt(head) -> float:
    """The head angle while driving: FOLLOW_NAV_TILT_DEG above the real horizon (the head
    looks tilt_offset_deg up at its straight position). Level, the camera sees the person's
    body and the ToF the floor from ~0.4 m out, up past the robot's height."""
    return settings.FOLLOW_NAV_TILT_DEG - getattr(head.geom, "tilt_offset_deg", 0.0)


def _align_point(nav, person):
    """Where Align turns to: the start of the Mac's path while it is fresh, else the person."""
    fresh = nav.clock() - getattr(nav, "_wp_t", -1e9) < GLOBAL_FRESH_S
    return nav.waypoints[1] if fresh and len(nav.waypoints) > 1 else person


def follow(camera, vision: VisionClient, motion, name: str = "", head=None,
           seconds: float = settings.FOLLOW_SECONDS,
           clock: Callable[[], float] = time.monotonic,
           pose: Optional[Callable[[], Optional[Pose]]] = None,
           bumped: Optional[Callable[[], float]] = None,
           twist: Optional[Callable[[], Optional[Tuple[float, float]]]] = None,
           head_still_while_driving: bool = False,
           on_cmd: Optional[Callable[[float, float], None]] = None) -> bool:
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
    # Everything it steers by, checked at once before anything moves (Nov 19: it started with the servos
    # unpowered, and twice with odometry dead after an Arduino reset). robot_core/health.py.
    bad = health.failed(health.preflight(head=head, pose=pose, vision=vision))
    if bad:
        log.warning("follow failed: %s", bad)
        status.error("F02", bad)
        status.detail(bad)
        status.result("follow failed")
        return False
    rules.reset()
    if vision.follow_start(name) is None:
        why = f"{name or 'nobody'} not enrolled"
        log.warning("follow failed: %s", why)
        status.error("F01", why)
        status.detail(why)
        status.result("follow failed")
        return False
    nav_tilt = _nav_tilt(head)
    nav = LocalNav(head, motion, pose, twist=twist, bumped=bumped, tilt_deg=nav_tilt,
                   head_still_while_driving=head_still_while_driving, on_cmd=on_cmd)
    nav.planner = _planner(vision, nav.plan_radius_m)
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
    phase = "search"                                      # Part B: watch | align | go | search
    phase_why = "start"
    go_since = align_since = 0.0
    blocked_since: Optional[float] = None
    unseen_since: Optional[float] = None
    search_turn_target: Optional[float] = None            # latched world heading (rad) for Search body turns
    search_turn_since = 0.0
    search_checked_last = False                           # turned toward last-seen proximity once per loss
    t0, seq, last_frame = clock(), 0, 0
    watchdog = health.Watchdog(head, nav.pose)
    try:
        while clock() - t0 < seconds and not task_cancel.requested():
            frame, here = stream.latest(newer_than=last_frame)
            last_frame = frame.seq
            seq += 1
            h, w = frame.main.shape[:2]
            grid = head.lidar.grid()
            pair_s = None if grid is None else round(grid.t - frame.t_capture, 3)
            if pair_s is not None and abs(pair_s) > TOF_PAIR_S:
                rules.broke("grid_unpaired", pair_s=pair_s)   # the boxes are in this frame, the ranges from another moment
                grid = None
            rng = range_in_box(grid, res.get("bbox"), w, h, head.geom.hfov_deg,
                               head.geom.tof_pan_offset_deg, head.geom.tof_tilt_offset_deg) if grid else None
            hint = None                                   # where they were last seen, from here (+ left)
            if last_seen and here and time.monotonic() - last_seen[1] < LAST_SEEN_S:
                hint = round(math.degrees(_wrap(last_seen[0] - here[2])), 1)
            img = jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE)
            t_call = time.monotonic()
            res = vision.follow_step(
                img, seq, pan_deg=frame.pan_deg, hfov_deg=head.geom.hfov_deg,
                tilt_deg=frame.tilt_deg + head.geom.tilt_offset_deg,     # the REAL tilt: the head looks up at 0
                camera_height_m=head.geom.height_m, range_m=rng,
                tof_grid=grid.rows if grid is not None else None, hint_deg=hint, pose=here,
                tof_pan_offset_deg=head.geom.tof_pan_offset_deg,
                tof_tilt_offset_deg=head.geom.tof_tilt_offset_deg)
            mac_s = time.monotonic() - t_call
            if mac_s > health.MAC_SLOW_S:
                rules.broke("mac_slow", s=round(mac_s, 2))
            mode, reasons = watchdog.update(mac_s)
            if mode == "stop":                            # odometry gone, servos off: say why and end
                raise RuntimeError("; ".join(reasons))
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
                why_r = refuse_range(tracker, here, d, res.get("distance_from"))
                if why_r:
                    gated = (round(d, 2), round(math.dist(here[:2], tracker.xy), 2))
                    log.info("follow %d: %s", seq, why_r)
                    rules.broke("range_refused", d=gated[0], tracked=gated[1], source=res.get("distance_from"))
                    # The ToF zones under last frame's box hit something between us and them (a chair leg, the
                    # door frame): nobody steps 0.4 m nearer in one frame. Believed twice, it moved the person onto
                    # the obstacle, nav "arrived" 0.2 m from it and the person-ignore circle erased it from the
                    # costmap (Nov 19: 39 such readings in one run). Skip it; the next box/floor range will do.
                    d = None
                last_seen = (here[2] + math.radians(b), time.monotonic())
                lost_go, lost_done, lost_face, unseen_since = None, False, None, None
                search_turn_target, search_checked_last = None, False
                if res.get("distance_from") == "near":    # they fill the frame: close enough, face them
                    a = here[2] + math.radians(b)
                    spot = (here[0] + math.cos(a), here[1] + math.sin(a))
                    if head_still_while_driving:
                        phase, phase_why = "watch", "near: standing"
                        nav.release_head()
                        nav.stop_here()
                        nav.set_focus(spot, PERSON_CLEAR_M)
                    else:
                        nav.release_head()
                        nav.set_goal(spot, keep_m=1.0)
                        nav.set_focus(spot, PERSON_CLEAR_M)
                elif d:
                    now_t = time.monotonic()
                    before, t_before = tracker.xy, tracker.t
                    tracker.update(here, b, d, res.get("distance_sigma_m"), now_t)
                    if before is not None:
                        jump, dt = math.dist(before, tracker.xy), max(0.0, tracker.t - t_before)
                        if jump > WALK_MAX_MPS * dt + 0.5:
                            rules.broke("goal_jump", jump_m=round(jump, 2), dt_s=round(dt, 2),
                                        source=res.get("distance_from"))
                    dist_m = math.dist(here[:2], tracker.xy)
                    if not head_still_while_driving:
                        nav.release_head()
                        nav.set_goal(tracker.xy, keep_m=settings.FOLLOW_KEEP_M,
                                     near_m=NEAR_M if tracker.walking() is None else 0.0)
                        nav.set_focus(tracker.xy, PERSON_CLEAR_M)
                    else:
                        # Four-phase follow: head tracks only while stopped (Watch / Search);
                        # body drives with head frozen at pan 0 (Align / Go).
                        if phase == "search":
                            phase, phase_why = "watch", "found"
                        if phase == "watch":
                            st = tracker.straight()
                            steady = st is not None and st[0] >= STRAIGHT_MPS and st[1] <= STRAIGHT_TURN_DEG
                            stood_far = tracker.still_s() >= 1.0 and dist_m > settings.FOLLOW_KEEP_M + 0.5
                            if abs(b) <= GO_CONE_DEG and dist_m > settings.FOLLOW_KEEP_M + 0.3:
                                phase, phase_why, go_since = "go", f"watch->go: ahead ({b:+.0f} deg)", now_t
                            elif abs(b) > GO_CONE_DEG and (steady or stood_far):
                                phase, align_since = "align", now_t
                                phase_why = f"watch->align: {'steady walk' if steady else 'still far'} ({b:+.0f} deg)"
                        elif phase == "align":
                            path_pt = _align_point(nav, tracker.xy)
                            align_err = abs(math.degrees(_wrap(math.atan2(path_pt[1] - here[1], path_pt[0] - here[0]) - here[2])))
                            if align_err <= ALIGN_DONE_DEG:
                                phase, phase_why, go_since = "go", f"align->go: aimed ({align_err:.0f} deg)", now_t
                            elif now_t - align_since > ALIGN_MAX_S:
                                phase, phase_why = "watch", f"align->watch: not lined up in {ALIGN_MAX_S:.0f} s ({align_err:.0f} deg)"
                        elif phase == "go":
                            tr = tracker.turn_rate()
                            blocked_since = (blocked_since or now_t) if nav.state == "blocked" else None
                            if dist_m <= settings.FOLLOW_KEEP_M + 0.15:
                                phase, phase_why = "watch", f"go->watch: reached ({dist_m:.2f} m)"
                            elif nav.state == "tripped":
                                phase, phase_why = "watch", "go->watch: nav tripped"
                            elif blocked_since is not None and now_t - blocked_since >= GO_BLOCKED_S:
                                phase, phase_why = "watch", f"go->watch: blocked {GO_BLOCKED_S:.0f} s"
                            elif now_t - go_since >= GO_MIN_S and abs(b) > GO_EXIT_DEG:
                                phase, phase_why = "watch", f"go->watch: {b:+.0f} deg off-centre"
                            elif now_t - go_since >= GO_MIN_S and tr > TURN_EXIT_DEG:
                                phase, phase_why = "watch", f"go->watch: turned {tr:.0f} deg"

                        if phase == "watch":
                            nav.release_head()
                            nav.stop_here()
                            nav.set_focus(tracker.xy, PERSON_CLEAR_M)
                        elif phase == "align":
                            nav.hold_head(0.0, nav_tilt)
                            path_pt = _align_point(nav, tracker.xy)
                            a = math.atan2(path_pt[1] - here[1], path_pt[0] - here[0])
                            nav.set_goal((here[0] + math.cos(a), here[1] + math.sin(a)), keep_m=1.0)
                        elif phase == "go":
                            nav.hold_head(0.0, nav_tilt)
                            nav.set_goal(tracker.xy, keep_m=settings.FOLLOW_KEEP_M,
                                         near_m=NEAR_M if tracker.walking() is None else 0.0)
            elif state == "searching" and res.get("locked") and not lost_done and tracker.xy is not None \
                    and last_seen and time.monotonic() - last_seen[1] < LAST_SEEN_S:
                # Lost them: go to where they were last seen (round a corner, through a door) first.
                now = time.monotonic()
                unseen_since = unseen_since or now
                if head_still_while_driving and phase == "go" and now - unseen_since < LOST_UNSEEN_S:
                    nav.hold_head(0.0, nav_tilt)
                    nav.set_goal(tracker.xy, keep_m=settings.FOLLOW_KEEP_M, face=False)
                else:
                    if lost_go is None:
                        lost_go, lost_dir = now, tracker.walking()
                        ahead = min(LOST_AHEAD_M[1], max(LOST_AHEAD_M[0], lost_dir[2] * 1.5)) if lost_dir else 0.0
                        lost_xy = (tracker.xy[0] + (lost_dir[0] * ahead if lost_dir else 0.0),
                                   tracker.xy[1] + (lost_dir[1] * ahead if lost_dir else 0.0))
                        log.info("follow %d: lost, going to where they were last seen%s", seq,
                                 f" and {ahead:.1f} m on, the way they walked" if lost_dir else "")
                        status.detail("going where last seen")
                        if head_still_while_driving:
                            phase, phase_why = "go", "lost-ahead (head held)"
                            nav.hold_head(0.0, nav_tilt)
                        else:
                            nav.release_head()
                            nav.set_focus(lost_xy, 0.1)
                    if lost_face is None and (nav.arrived() or now - lost_go > LOST_GO_S):
                        lost_face = now
                        nav.set_focus(None)
                    if lost_face is None:
                        nav.set_goal(lost_xy, keep_m=LOST_KEEP_M, face=False)
                    elif here and now - lost_face < LOST_FACE_S:
                        # Face where they were actually last seen (tracker.xy) unless we already reached it,
                        # in which case face the direction they walked. Prevents a noisy radial range drop
                        # from spinning the robot away from tracker.xy into a wall.
                        face_ang = (math.atan2(lost_dir[1], lost_dir[0])
                                    if (lost_dir and math.dist(here[:2], tracker.xy) < 0.5)
                                    else math.atan2(tracker.xy[1] - here[1], tracker.xy[0] - here[0]))
                        if abs(_wrap(face_ang - here[2])) > math.radians(15):
                            nav.set_goal((here[0] + math.cos(face_ang), here[1] + math.sin(face_ang)), keep_m=1.0)
                        else:
                            lost_done = True
                            nav.set_goal(None)
                    else:
                        lost_done = True
                        nav.set_goal(None)
            elif state == "searching":
                now_s = time.monotonic()
                look = res.get("head") or [0, 0]          # the Mac's search sweep: the robot stands, the head looks
                # 1. If we just entered Search with a known last-seen spot > 60 deg off-centre, turn to face
                #    that proximity first before sweeping.
                if search_turn_target is None and not search_checked_last and here and tracker.xy is not None:
                    search_checked_last = True
                    last_ang = math.atan2(tracker.xy[1] - here[1], tracker.xy[0] - here[0])
                    if abs(_wrap(last_ang - here[2])) > math.radians(60.0):
                        search_turn_target, search_turn_since = last_ang, now_s
                # 2. If the Mac asked for a body turn (e.g. 180 deg after a full gimbal sweep missed), latch it
                #    so the robot finishes the turn instead of cancelling on the next frame.
                if search_turn_target is None and res.get("turn_deg") and here:
                    search_turn_target = _wrap(here[2] - math.radians(res["turn_deg"]))
                    search_turn_since = now_s
                if search_turn_target is not None and here:
                    if abs(_wrap(search_turn_target - here[2])) <= math.radians(15.0) or now_s - search_turn_since > 5.0:
                        search_turn_target = None
                turning = bool(search_turn_target is not None and here)
                phase = "search"
                phase_why = "turn to last seen / 180" if turning else "head sweep"
                if head_still_while_driving and turning:  # the body turns with the head held ahead (it may not move
                    nav.hold_head(0.0, settings.FOLLOW_TILT_DEG)   # while the body does); stopping and looking each
                else:                                     # step had cut the turn into pieces
                    nav.release_head()
                    nav.stop_here()
                if turning:
                    nav.set_goal((here[0] + math.cos(search_turn_target), here[1] + math.sin(search_turn_target)),
                                 keep_m=1.0)
                if not (head_still_while_driving and turning):
                    nav.look(look[0], settings.FOLLOW_TILT_DEG + look[1])
            if mode == "degraded" and any(r.startswith("no ToF") for r in reasons):
                nav.set_goal(None)                        # no obstacle sensing: stand rather than drive blind
                if nav.cmd[0] > 0.02:
                    rules.broke("tof_stale", why=reasons[0])
            debug_upload.offer_event(img, {                 # debug mode: this frame, the Mac's answer, the decision
                "task": "follow", "seq": seq, "state": state, "phase": phase, "phase_why": phase_why,
                "target": res.get("target"),
                "bearing_deg": res.get("bearing_deg"), "distance_m": res.get("distance_m"),
                "distance_from": res.get("distance_from"), "range_m": rng, "pose": here, "pose_now": nav.pose(),
                "goal": nav.goal, "nav": nav.state, "nav_detail": nav.detail, "cmd": nav.cmd,
                "waypoints": nav.waypoints[:6], "reflex_trips": nav.reflex.trips, "head": res.get("head"),
                "range_note": res.get("range_note"), "gated": gated, "near_m_used": nav.near_m,
                "tracker": None if tracker.xy is None else {"xy": [round(v, 3) for v in tracker.xy],
                                                            "sigma": round(math.sqrt(tracker.var), 3),
                                                            "walking": tracker.walking()},
                "nav_source": nav.source, "marked": nav.marked[-3:],
                "rules": rules.snapshot(), "health": [mode, reasons], "grid_pair_s": pair_s, "mac_s": round(mac_s, 3)},
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
