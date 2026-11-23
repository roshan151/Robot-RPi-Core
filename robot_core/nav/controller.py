"""LocalNav: the Pi's local layer. Drives toward a goal at 20 Hz, on its own.

    reflex (50 Hz)        ToF grids in, emergency brake            nav/reflex.py
    costmap               rolling 4 x 4 m memory of what was seen   nav/costmap.py
    DWA (20 Hz)           (v, w) toward the current target          nav/dwa.py
    gimbal aim (20 Hz)    look-ahead + sweep, or the person         nav/aim.py
    global plan (~1 Hz)   waypoints from the Mac (Theta*)           `planner`, in a worker thread

The caller sets a goal (`set_goal`: a point and how far short of it to stop) and, while
following someone, a focus for the head (`set_focus`). The controller does the rest:
every tick it folds the new ToF grids into the costmap, picks a target (the furthest
waypoint of the Mac's path it can see in a straight line, else the goal itself), runs DWA,
and streams the result as /cmd_vel (the drivetrain's velocity loop holds each track's speed,
the gyro its heading). Close enough: it turns to face the goal and stands.

Network fail-safe: the Mac's plan is asked for in the background and never waited on. If it
does not answer, the last waypoints are kept for GLOBAL_FRESH_S (older ones lie behind it);
then the Pi's own Theta* on the costmap, else DWA straight for the goal. A goal
nobody has renewed for `goal_timeout_s` is dropped and the robot stops, ramped.

Stuck (StuckWatch): the encoders gaining distance while what is straight ahead does not get
nearer is the tracks slipping. It stops, takes the slipped metres out of the pose (`fix`),
marks the obstacle, and backs off. The drivetrain's own checks (stall, gyro-blocked, climb)
come in through the reflex as a trip.
"""

from __future__ import annotations

import logging
import math
import statistics
import threading
import time
from collections import deque
from typing import Callable, List, Optional, Tuple

import numpy as np

from robot_core import status
from robot_core.nav import dwa
from robot_core.rules import rules
from robot_core.nav.aim import GimbalAim
from robot_core.nav.costmap import RollingCostmap
from robot_core.nav.plan import local_path
from robot_core.nav.reflex import Reflex

log = logging.getLogger(__name__)

Pose = Tuple[float, float, float]

GLOBAL_FRESH_S = 3.0            # the Mac's waypoints are used this long after they came
LOCAL_EVERY_S = 0.5             # the Pi's own Theta* on the costmap, when they are not
STUCK_WINDOW_S = 0.8            # over this long the encoders went STUCK_MIN_ODO_M or more...
STUCK_MIN_ODO_M = 0.06
STUCK_NEAR_M = 0.40             # ...while something this near straight ahead...
STUCK_STILL_M = 0.025           # ...came no nearer than this: slipping against it
CLIMB_DEG = 6.0                 # nose up this far since it started moving, for CLIMB_HOLD_S: riding up on something
CLIMB_HOLD_S = 0.5              # (12 deg at once missed a ledge it high-centred on at 8: test_scenarios ledge)
STUCK_FAR_M, STUCK_FAR_WINDOW_S, STUCK_FAR_ODO_M = 2.5, 1.5, 0.12   # ...or anything up to 2.5 m ahead that came no
#                                 nearer while the encoders went 12 cm in 1.5 s: tracks spinning on the floor (a cable)
TURN_BLOCKED_S = 0.7            # turning on the spot with the gyro under 0.1 rad/s this long: a corner is held
SAME_PLACE_M, SAME_HEADING_DEG = 0.3, 30.0   # a failure this near an earlier one, this near its heading: same approach
JOIN_M = 0.6                    # two bumps / climbs this near: one thing, wider than either showed (glass, a ledge)
BACK_OFF_M = 0.10
TURN_BACK_M = 0.05              # backing off to make room to turn on the spot
AHEAD_STALE_S = 1.2             # driving with the way ahead unseen this long: glance at it, slow meanwhile
AHEAD_CONE_DEG = 28.0           # a grid taken with the head this near the direction of travel saw the way (ToF: +-30)
ARRIVE_M, RESUME_M = 0.08, 0.25 # arrived within keep + ARRIVE_M; set off again only past keep + RESUME_M, so a
                                # person standing still (their range wobbles) is not chased back and forth
BLIND_MPS = 0.12                # top speed while the way ahead has not been seen...
SEEN_MPS = 0.25                 # ...or this, where the map remembers it as free (the head is on the person)
PIVOT_FOCUS_W = 0.55            # turn rate pivoting while the camera is on someone: 0.8 rad/s blurred the frames
PIVOT_FREE_W = 1.0              # ...and with nobody to keep in frame (0.8 took 3.5 s to turn round; 1.2 broke trap and sudden)
                                # and the person was missed 9 frames in 16 at a doorway (Nov 19)
NO_PROGRESS_S, NO_PROGRESS_M = 4.0, 0.05   # "driving" this long without moving this far: wedged
PLAN_EXTRA_M = 0.025            # extra room asked beyond DWA radius + margin (the planner adds its own 0.05, else 0.03)
SILL_RISE_DEG = 2.5             # a drivetrain trip with the nose this far above the floor's level in the last 1 s,
SILL_SLOW_MPS, SILL_SLOW_S = 0.10, 3.0   # and nothing seen just ahead: a door sill. Go again this slowly, this long


def _length(here, path, goal) -> float:
    """How far it is from here along `path` (its first point is where it was planned from) to `goal`."""
    pts = [tuple(here[:2])] + [tuple(p[:2]) for p in path[1:]]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) + math.dist(pts[-1], goal)


class PoseFix:
    """The odometry pose with the metres the tracks slipped taken back out. The heading is the
    gyro's and is kept. Everything the nav stack keeps is in this frame."""

    def __init__(self, raw: Callable[[], Optional[Pose]]) -> None:
        self.raw, self.dx, self.dy = raw, 0.0, 0.0

    def __call__(self) -> Optional[Pose]:
        p = self.raw()
        return None if p is None else (p[0] - self.dx, p[1] - self.dy, p[2])

    def undo(self, metres: float, heading: float) -> None:
        self.dx += metres * math.cos(heading)
        self.dy += metres * math.sin(heading)


class StuckWatch:
    """Fed with the encoder distance, the ToF's straight-ahead range (None when the head is not
    looking ahead) and how far the nose has risen. Stuck: the encoders went STUCK_MIN_ODO_M in
    STUCK_WINDOW_S while something within STUCK_NEAR_M ahead stayed put, or the nose rose
    CLIMB_DEG. `phantom_m`: the encoder metres the robot did not go. replay_debug.py runs it
    over recorded sessions."""

    def __init__(self) -> None:
        self.hist: deque = deque()
        self.phantom_m = 0.0
        self.kind = ""                # "push" (against something near), "floor" (spinning on the floor), "climb"
        self._climb: Optional[Tuple[float, Optional[float]]] = None   # (since when, odo then) the nose is up

    def reset(self) -> None:
        self.hist.clear()
        self._climb = None

    def update(self, t: float, odo_m: Optional[float], centre_m: Optional[float],
               rise_deg: Optional[float], pan: float = 0.0, facing: bool = True,
               heading: Optional[float] = None) -> str:
        """centre_m: how far ahead (robot x) what the ToF sees at head pan `pan` (deg) is; facing: those points spread
        across the way, not along it (a wall alongside reads the same as the robot drives past it)."""
        if rise_deg is not None and rise_deg > CLIMB_DEG:
            self._climb = self._climb or (t, odo_m)
            if t - self._climb[0] >= CLIMB_HOLD_S:            # riding up: the encoder metres since are not real
                self.phantom_m = max(0.0, (odo_m or 0.0) - (self._climb[1] or odo_m or 0.0))
                self.kind = "climb"
                return f"nose up {rise_deg:.0f} deg for {CLIMB_HOLD_S:.1f} s: riding up on something"
        else:
            self._climb = None
        # Looks at the same head angle compared over time: the head sweeps while driving, so a straight-ahead range
        # comes in bursts 2 s apart and two of them never fell one window apart (test_scenarios cable).
        if odo_m is None or centre_m is None or abs(pan) > 45.0 or not facing:
            return ""
        look = (t, odo_m, pan, centre_m, heading)
        self.hist.append(look)
        while self.hist and t - self.hist[0][0] > 5.0:
            self.hist.popleft()
        same = [h for h in self.hist if abs(h[2] - pan) <= 5.0 and
                (heading is None or h[4] is None or abs(dwa.wrap(heading - h[4])) < math.radians(3.0))]
        for window, min_odo, near in ((STUCK_WINDOW_S, STUCK_MIN_ODO_M, STUCK_NEAR_M),
                                      (STUCK_FAR_WINDOW_S, STUCK_FAR_ODO_M, STUCK_FAR_M)):
            old = [h for h in same if t - h[0] >= window]
            if not old:
                continue
            _, o0, _, c0, _ = old[-1]                         # the newest look old enough decides
            went = odo_m - o0
            if went >= min_odo and max(c0, centre_m) < near and abs(centre_m - c0) < STUCK_STILL_M:
                _, o1, _, c1, _ = same[0]                     # the oldest: all the slip since then
                self.phantom_m = max(0.0, (odo_m - o1) - max(0.0, c1 - centre_m))
                self.kind = "push" if max(c0, centre_m) < STUCK_NEAR_M else "floor"
                return f"wheels went {went:.2f} m but {centre_m:.2f} m ahead stayed put: tracks slipping"
        return ""


def centre_range(rows) -> Optional[float]:
    """Median of the ToF's middle zones (rows 2-5, columns 3-4) as the sensor reads them."""
    vals = [rows[r][c] for r in range(2, 6) for c in range(3, 5) if rows[r][c] is not None]
    return statistics.median(vals) if vals else None


class LocalNav:
    def __init__(self, head, motion, pose: Callable[[], Optional[Pose]],
                 twist: Optional[Callable[[], Optional[Tuple[float, float]]]] = None,
                 bumped: Optional[Callable[[], float]] = None,
                 planner: Optional[Callable[[Pose, Tuple[float, float], dict], Optional[list]]] = None,
                 tilt_deg: float = -21.0, hz: float = 20.0, goal_timeout_s: float = 6.0,
                 plan_every_s: float = 1.0, params: Optional[dwa.DwaParams] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        p = head.terrain
        self.head, self.motion, self.clock = head, motion, clock
        self.fix = PoseFix(pose)
        self.twist = twist
        self.params = params or dwa.DwaParams(radius_m=math.hypot(p.width_m / 2.0, p.width_m / 2.0))   # square
        # What the global planner is asked to keep clear: DWA's radius + margin + PLAN_EXTRA_M. Planned with DWA's
        # bare radius, the Mac's path hugged a wall 0.22 m off past a door frame, DWA (which also wants room at the
        # nose) could not follow it, and the robot stood turning in the doorway (test_scenarios doorway).
        self.plan_radius_m = self.params.radius_m + self.params.margin_m + PLAN_EXTRA_M
        self.costmap = RollingCostmap(flat_tol_m=max(p.h_max_m, p.flat_tol_m), body_m=p.body_m)
        self.cmd = (0.0, 0.0)
        self.reflex = Reflex(head, motion, lambda: self.cmd, self.fix, bumped, front_m=p.width_m / 2.0,
                             back_m=p.width_m / 2.0, half_width_m=p.width_m / 2.0, clock=clock)
        self.aim = GimbalAim(head.gimbal, tilt_deg)
        self.planner, self.plan_every = planner, plan_every_s
        self.period, self.goal_timeout = 1.0 / hz, goal_timeout_s
        self.goal: Optional[Tuple[float, float]] = None
        self.keep_m, self.goal_t, self.face_goal, self.near_m = 0.0, 0.0, True, 0.0
        self.focus: Optional[Tuple[float, float]] = None
        self.ignore: Optional[Tuple[Tuple[float, float], float]] = None   # (xy, radius): not an obstacle
        self.waypoints: List[Tuple[float, float]] = []     # the Mac's (global)
        self.wp_goal: Optional[Tuple[float, float]] = None
        self._wp_t = 0.0
        self.local_wps: List[Tuple[float, float]] = []     # the Pi's own (local Theta*)
        self._local_t = -1e9
        self._local_busy = False
        self.state, self.detail, self.source = "idle", "", "none"
        self._plan_t, self._planning = 0.0, False
        self._stuck = StuckWatch()
        self._moving_since: Optional[Tuple[float, float, Optional[float]]] = None   # (t, odo, pitch)
        self._odo, self._last_pose = 0.0, None
        self._recover: Optional[str] = None
        self._trips_seen = 0
        self._ahead_t = -1e9                          # when a grid last looked along the way it is going
        self.marked: List[Tuple[float, float]] = []    # odom xy of what the reflex braked for (debug record)
        self._pivot_sign, self._flips = 0, deque()    # rule pivot_flip: turn-on-the-spot direction changes
        self._blocked_since: Optional[float] = None   # rule blocked_long
        self._progress: Optional[Tuple[float, Tuple[float, float]]] = None   # (since when, where) it last moved
        self.failures: List[Tuple[float, float, float, str]] = []   # (x, y, heading, kind) of ways that did not work
        self._pitches: deque = deque()                # (t, IMU pitch) over 10 s: the floor's level
        self._via: Optional[Tuple[Tuple[float, float], float]] = None   # a point to go through first, and until when
        self._turn_held: Optional[float] = None       # since when a turn on the spot has not turned (gyro)
        self._sill_at: Optional[Tuple[float, float]] = None   # where it last went again over a sill
        self._slow_until = 0.0                        # until when it drives at SILL_SLOW_MPS at most
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # -------------------------------------------------------------- the API

    def start(self) -> "LocalNav":
        self.reflex.start()
        self._thread = threading.Thread(target=self._run, name="nav-control", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.reflex.stop()
        self._send(0.0, 0.0)

    def set_goal(self, xy: Optional[Tuple[float, float]], keep_m: float = 0.0, face: bool = True,
                 near_m: float = 0.0) -> None:
        """Go to `xy` (odometry frame, as `pose`), stopping `keep_m` short; None: stop here.
        near_m: within keep_m + near_m, if the way there goes the long way round something, stand
        and face it instead (a person seen beside a suitcase). Call again to renew it: a goal not
        renewed for goal_timeout_s is dropped."""
        with self._lock:
            moved = xy is None or self.goal is None or math.dist(xy, self.goal) > 0.3
            self.goal, self.keep_m, self.face_goal, self.goal_t = xy, keep_m, face, self.clock()
            self.near_m = near_m
            if moved:
                self._plan_t = 0.0                       # a new goal: ask for a plan now

    def set_focus(self, xy: Optional[Tuple[float, float]], ignore_radius_m: float = 0.35) -> None:
        """Keep the head (camera) on `xy`; what stands within ignore_radius_m of it (the person's
        legs) is not an obstacle."""
        with self._lock:
            self.focus = xy
            self.ignore = (xy, ignore_radius_m) if xy is not None else None

    def look(self, pan: float, tilt: float) -> None:
        """Take the head for a search look; `release_head()` gives it back to the nav aim."""
        self.aim.look(pan, tilt)

    def release_head(self) -> None:
        self.aim.release()

    def pose(self) -> Optional[Pose]:
        return self.fix()

    def arrived(self) -> bool:
        return self.state == "arrived"

    # ------------------------------------------------------------- the loop

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = self.clock()
            try:
                self.tick()
            except Exception as exc:                     # noqa: BLE001
                log.warning("nav: %s: %s", type(exc).__name__, exc)
                self._send(0.0, 0.0)
            self._stop.wait(max(0.0, self.period - (self.clock() - t0)))

    def _send(self, v: float, w: float) -> None:
        self.cmd = (v, w)
        self.motion.velocity(v, w)

    def _ingest(self, now: float) -> Optional[tuple]:
        """Fold the new grids into the costmap; the latest (middle range, pan, facing) for StuckWatch."""
        ahead = None
        for t, pose, zones, rows, pan in self.reflex.take():
            with self._lock:
                ign = self.ignore
            if ign is not None and pose is not None:      # leave the person out
                zs = []
                c, s = math.cos(pose[2]), math.sin(pose[2])
                for z in zones:
                    wx, wy = pose[0] + c * z[2] - s * z[3], pose[1] + s * z[2] + c * z[3]
                    if math.hypot(wx - ign[0][0], wy - ign[0][1]) > ign[1]:
                        zs.append(z)
                zones = zs
            self.costmap.integrate(pose, zones, self.head.sensor_xy, t=t)
            if ign is not None:                           # and what was marked there before: the tracked spot
                self.costmap.clear_near(*ign)             # lags a walking person, so their back edge was marked
            travel = max(-45.0, min(45.0, math.degrees(self.aim.k_w * self.cmd[1])))
            if abs(pan - travel) <= AHEAD_CONE_DEG:
                self._ahead_t = now
            # What it sees where the head looks (6 deg either side), things standing on the floor: how far ahead
            # (robot x) they are. A thing that stays put comes nearer by what the robot drives, whatever the pan.
            # Facing: hits on both sides of the look, as far ahead. Walls alongside (a corridor) read the same from
            # a moving robot and looked like slip (test_scenarios corridor); they are hit on one side, or unevenly.
            sides: dict = {1: [], -1: []}
            for z in zones:
                off = math.degrees(dwa.wrap(math.atan2(z[3], z[2]) - math.radians(pan)))
                if z[4] is not None and 0.04 < z[4] < 0.6 and 0.5 < abs(off) < 6.0:
                    sides[1 if off > 0 else -1].append(z[2])
            if sides[1] and sides[-1] and abs(pan) <= 45.0:
                l, r = float(np.median(sides[1])), float(np.median(sides[-1]))
                ahead = ((l + r) / 2.0, pan, abs(l - r) <= 0.08 + 0.05 * (l + r) / 2.0)
        return ahead

    def _measured(self) -> Tuple[float, float]:
        """The (v, w) the dynamic window is centred on: the last command. The drivetrain's velocity
        loop ramps to each command at the accelerations DWA assumes, so the command is where the
        robot is headed. The measured twist is not used for it: /odom's gyro turn rate came out up
        to 2x the real one (Nov 19), the window then held only right turns, and the robot turned
        on past the person it was following until it lost them."""
        return self.cmd

    def tick(self) -> None:
        now = self.clock()
        ahead = self._ingest(now)
        here = self.fix()
        if here is None:
            self._send(0.0, 0.0)
            if self.state != "no pose" and self.goal is not None:
                rules.broke("pose_stale", state=self.state)
            self.state, self.detail = "no pose", "waiting for odometry"
            return
        if self._last_pose is not None:
            self._odo += math.hypot(here[0] - self._last_pose[0], here[1] - self._last_pose[1])
        self._last_pose = here
        if self._trips_seen != self.reflex.trips:         # the brake holds /cmd_vel off until a zero command: send
            hits, self.reflex.hits = self.reflex.hits, []
            if hits and self.reflex.why.startswith("the drivetrain") and self._sill(here):
                # A door sill jolted it (Nov 20: a 5 deg rise at a threshold marked a 0.5 m wall across the
                # doorway, sealed it, and it turned away from the person). Mark nothing; go again slowly, once.
                self._sill_at, self._slow_until = tuple(here[:2]), now + SILL_SLOW_S
                log.info("nav: drivetrain trip on a sill at %s: going again slowly", [round(v, 2) for v in here[:2]])
            elif hits:                                    # remember what it braked for and plan round it: it drove
                self.costmap.add_obstacle(here, hits)     # back into the same thing 4 times in 8 s (Nov 19), as
                self._plan_t, self._local_t = 0.0, -1e9   # the head was on the person and the map never had it
                c, s = math.cos(here[2]), math.sin(here[2])
                at = [(round(here[0] + c * x - s * y, 2), round(here[1] + s * x + c * y, 2)) for x, y in hits]
                if any(math.dist(m, at[len(at) // 2]) < 0.3 for m in self.marked):
                    rules.broke("trip_repeat", at=at[len(at) // 2], why=self.reflex.why)
                if self.reflex.why.startswith("the drivetrain"):   # felt, not seen: a way that did not work
                    self._failed(here, "bump", hits[len(hits) // 2], join=True)
                    if self._room_behind(here, 0.15):     # off it before turning: turning against it rubbed it
                        self._recover, self._recover_from, self._back_m = "back", (here, self.clock()), 0.15
                self.marked = (self.marked + [at[len(at) // 2]])[-20:]
                log.info("nav: marked what it braked for (%s) at %s, replanning", self.reflex.why, at[len(at) // 2])
            self._trips_seen = self.reflex.trips          # it first, whatever the state (Nov 19: tripped while
            self._send(0.0, 0.0)                          # "arrived", it never sent one, the drivetrain stayed held,
            return                                        # and the body stood while the head followed them away)
        with self._lock:
            goal, keep, face_goal, goal_t, focus = self.goal, self.keep_m, self.face_goal, self.goal_t, self.focus
            near_m = self.near_m
        if self._recover:
            self._do_recover(here)
            return
        if goal is not None and now - goal_t > self.goal_timeout:
            log.info("nav: goal not renewed for %.0f s: stopping", self.goal_timeout)
            with self._lock:
                self.goal = goal = None
        v_meas, w_meas = self._measured()
        focus_pan = None
        if focus is not None:
            focus_pan = math.degrees(dwa.wrap(math.atan2(focus[1] - here[1], focus[0] - here[0]) - here[2]))
        if goal is None:
            self._send(0.0, 0.0)
            self.state = "idle"
            self.aim.update(now, 0.0, 0.0, focus_pan)
            return
        dist = math.hypot(goal[0] - here[0], goal[1] - here[1])
        bearing = dwa.wrap(math.atan2(goal[1] - here[1], goal[0] - here[0]) - here[2])
        if dist <= keep + (RESUME_M if self.state == "arrived" else ARRIVE_M):   # there: face it and stand
            w = dwa.face(bearing) if face_goal else 0.0
            self._send(0.0, w)
            self.state, self.detail = "arrived", f"{dist:.2f} m from the goal"
            self._moving_since = None
            self.aim.update(now, 0.0, w, focus_pan)
            return
        self._maybe_plan(now, here, goal, keep)
        radius = self.params.radius_m + self.params.margin_m
        if now - self._local_t >= LOCAL_EVERY_S and not self._local_busy:   # always: the Mac's way is checked
            self._local_t, self._local_busy = now, True                    # against it
            threading.Thread(target=self._plan_local, args=(here[:2], goal), name="nav-local",
                             daemon=True).start()            # off the control thread: slow on a Pi
        fresh = self.waypoints and self.wp_goal is not None and math.dist(self.wp_goal, goal) < 0.6 \
            and now - self._wp_t < GLOBAL_FRESH_S
        if fresh and self._detour(here, goal, self.waypoints, radius):
            fresh = False                                 # the Mac goes the long way round something our
            self.detail = "Mac path is a detour: going direct"   # costmap does not have (Nov 19: a phantom)
            if now - getattr(self, "_detour_log_t", -1e9) > 2.0:   # say so in logs.json, not every tick
                self._detour_log_t = now
                log.info("nav: Mac path %.2f m vs %.2f m direct (seen free): going direct",
                         _length(here, self.waypoints, goal), math.dist(here[:2], goal))
        if fresh:
            path, self.source = self.waypoints, "global"
        else:                                             # no word from the Mac: plan on the costmap
            path, self.source = self.local_wps, "local"
        if path and near_m and dist <= keep + near_m and _length(here, path, goal) > dist + 0.5:
            # Near them, but the way to them goes round something (a suitcase beside them, a door frame): face
            # them and stand. Going round turned it away from them and spun it on the spot (Nov 19).
            w = dwa.face(bearing) if face_goal else 0.0
            self._send(0.0, w)
            self.state, self.detail = "arrived", f"{dist:.2f} m, the way round is long: facing it"
            self._moving_since = None
            self.aim.update(now, 0.0, w, focus_pan)
            return
        # Skipping ahead to a later waypoint needs PLAN_EXTRA_M + 0.025 more room than DWA's margin: with the bare margin
        # the straight line to it cut a door-frame corner and the robot wedged there (test_scenarios doorway).
        target = dwa.target_on_path(here, path, self.costmap.clearance, radius + PLAN_EXTRA_M + 0.025,
                                    seen_fn=self._seen_line) if path else None
        if target is None or math.dist(target, goal) < 0.1:   # the goal itself: stop keep_m short of it
            target, stop_at = goal, keep
        else:
            stop_at = 0.0
        if self._via is not None:                         # another approach: through this point first
            if now > self._via[1] or math.dist(here[:2], self._via[0]) < 0.15:
                self._via = None
            else:
                target, stop_at = self._via[0], 0.0
        res = dwa.plan((here[0], here[1], here[2], v_meas, w_meas), target, self.costmap.clearance,
                       self.params, stop_at_m=stop_at)
        v, w = res.v, res.w
        tb = dwa.wrap(math.atan2(target[1] - here[1], target[0] - here[0]) - here[2])   # the way, from here
        if abs(tb) > math.radians(28.0) and not res.blocked:
            # Target is significantly off-heading: pivot in place to face it first,
            # rather than driving wide forward arcs that saturate the gimbal and chase its tail.
            v = 0.0
            max_w = PIVOT_FOCUS_W if focus is not None else PIVOT_FREE_W
            w = dwa.face(tb, max_w=max_w) or (math.copysign(min(0.7, max_w), tb) if abs(w_meas) < 0.3 else w_meas)
        elif not res.blocked and v < 0.02:                # only turning on the spot is safe: turn properly, toward the
            # way, not a 0.03 rad/s creep the tracks cannot do. Toward the target's side: a fixed +0.6 (always left)
            # with the target nearly ahead turned it away, then back, 64 times in a doorway (test_scenarios doorway).
            w = dwa.face(tb) or (math.copysign(0.6, tb) if abs(w) < 0.3 else w)
        if res.blocked:
            # Toward the same point as above, never a blind turn to the left: the two disagreeing spun it on the
            # spot in a doorway until it lost the person (Nov 19). Blocked with the way straight ahead: back off.
            self._plan_t, self._local_t = 0.0, -1e9       # plan a way round now (the Mac's and our own)
            w = dwa.face(tb) if abs(tb) > math.radians(20) else 0.0
            if not w and self._recover is None and self._room_behind(here, 0.1):
                self._recover, self._recover_from, self._back_m = "back", (here, now), TURN_BACK_M
            self.state, self.detail = "blocked", "no safe arc: turning toward the way" if w else "no safe arc: backing off"
        else:
            self.state, self.detail = "driving", f"{dist:.2f} m to go"
            self._watch_progress(now, here, dist)
        if v > 0.02 and now - self._ahead_t > AHEAD_STALE_S:     # not looked where it is going lately
            v = min(v, SEEN_MPS if self.costmap.seen_free(here, 0.6) else BLIND_MPS)
            if now >= self.aim.glance_until:
                self.aim.glance(now, math.degrees(self.aim.k_w * w))
        if now < self._slow_until:                       # going again over a sill
            v = min(v, SILL_SLOW_MPS)
        if self.reflex.tripped():
            v = 0.0
            self.state, self.detail = "tripped", self.reflex.why
        if v < 0.02 and w != 0.0 and self._recover is None and self._turn_hits(here, w):
            # In a doorway or pass, do not trigger repeated back-off oscillations if heading towards the path
            if abs(tb) > math.radians(45.0) and self._room_behind(here, 0.1):
                self._recover, self._recover_from, self._back_m = "back", (here, now), TURN_BACK_M
            v = w = 0.0
        self._watch_rules(now, here, v, w)
        if self._watch_turn(now, here, v, w):
            v = w = 0.0
        self._send(v, w)
        status.detail(f"nav {self.state}")
        self._watch_stuck(now, here, v, ahead)
        near = None
        cl = float(self.costmap.clearance([here[0] + 0.5 * math.cos(here[2])], [here[1] + 0.5 * math.sin(here[2])])[0])
        if cl < 1.5:
            near = 0.5 + cl
        self.aim.update(now, v, w, focus_pan, near)

    # ------------------------------------------------------------- planning

    def _seen_line(self, a, b) -> bool:
        """Every cell on the straight line a -> b was seen free."""
        return self.costmap.known_free_line(a, b)

    def _watch_progress(self, now: float, here: Pose, dist: float) -> None:
        """In "driving" but not NO_PROGRESS_M from where it was NO_PROGRESS_S ago: wedged (Nov 19 sim: against a
        door-frame corner no arc met the margin and it turned on the spot for 40 s). Back off and plan again.
        Measured by where it is, not by the distance to the goal: the way round can first lead away."""
        if self._progress is None or math.dist(here[:2], self._progress[1]) > NO_PROGRESS_M:
            self._progress = (now, tuple(here[:2]))
            return
        if now - self._progress[0] > NO_PROGRESS_S and self._recover is None:
            rules.broke("no_progress", at=[round(here[0], 2), round(here[1], 2)], to_go=round(dist, 2))
            if self._room_behind(here, 0.12):
                self._recover, self._recover_from, self._back_m = "back", (here, now), BACK_OFF_M
            self._plan_t, self._local_t = 0.0, -1e9
            self._progress = (now, tuple(here[:2]))

    def _watch_rules(self, now: float, here: Pose, v: float, w: float) -> None:
        """pivot_flip: turning on the spot changed direction twice in 2 s; blocked_long: no safe arc for 3 s."""
        if v < 0.02 and abs(w) > 0.25:
            s = 1 if w > 0 else -1
            if self._pivot_sign and s != self._pivot_sign:
                self._flips.append(now)
                while self._flips and now - self._flips[0] > 2.0:
                    self._flips.popleft()
                if len(self._flips) >= 2:
                    rules.broke("pivot_flip", at=[round(here[0], 2), round(here[1], 2)], detail=self.detail)
                    self._flips.clear()
            self._pivot_sign = s
        if self.state == "blocked":
            self._blocked_since = self._blocked_since or now
            if now - self._blocked_since > 3.0:
                rules.broke("blocked_long", at=[round(here[0], 2), round(here[1], 2)])
                self._blocked_since = now
        else:
            self._blocked_since = None

    def _sill(self, here) -> bool:
        """The drivetrain tripped with the nose up SILL_RISE_DEG over the floor's level in the last second and
        nothing in the costmap just ahead: a door sill or rug edge, not a wall. Once per place: tripping again
        within 0.3 m of the last try is something real."""
        if not self._pitches or (self._sill_at is not None and math.dist(self._sill_at, here[:2]) < 0.3):
            return False
        last = self._pitches[-1][0]
        floor = statistics.median(p for _, p in self._pitches)
        peak = max(p for t, p in self._pitches if last - t <= 1.0)
        if peak - floor < SILL_RISE_DEG:
            return False
        c, s = math.cos(here[2]), math.sin(here[2])
        return float(self.costmap.clearance([here[0] + 0.25 * c], [here[1] + 0.25 * s])[0]) > 0.2

    def _room_behind(self, here, m: float) -> bool:
        """Room to back up m: the point m behind is clear by the robot's radius (unknown counts as clear, as before),
        or it is narrower but seen free all the way and wide enough for the body: a recess it drove into, too tight
        to turn in, has no room by the first test and it spun into the walls (test_scenarios nook). No rear sensor:
        seen free is all it knows there."""
        c, s = math.cos(here[2]), math.sin(here[2])
        f = np.linspace(0.0, m, 4)
        cl = self.costmap.clearance(here[0] - f * c, here[1] - f * s)
        if float(cl[-1]) > self.params.radius_m:
            return True
        return float(cl.min()) >= self.params.radius_m / math.sqrt(2.0) + 0.02 and \
            self.costmap.seen_free((here[0], here[1], here[2] + math.pi), 0.2 + m)

    def _turn_hits(self, here, w: float, dt: float = 0.4) -> bool:
        """Would turning on the spot at w for dt sweep the square footprint into an occupied cell?"""
        half = self.params.radius_m / math.sqrt(2.0)                    # half the square's side
        pad = self.costmap.res * 0.4 + 0.015                            # realistic corner sweep margin (~3.5 cm)
        near = [p for p in self.costmap.points() if math.dist(p, here[:2]) < self.params.radius_m + pad]
        for th in np.linspace(here[2], here[2] + w * dt, 6):
            c, s = math.cos(th), math.sin(th)
            for x, y in near:
                dx, dy = x - here[0], y - here[1]
                if max(abs(dx * c + dy * s), abs(-dx * s + dy * c)) < half + pad:
                    return True
        return False

    def _detour(self, here, goal, path, radius: float) -> bool:
        """The Mac's path is over 1.5 times as long as our own (plus 0.3 m): its map closed a doorway our
        fresh costmap has open, and it went round through the unseen inside of a wall (Nov 19). Or it is
        over twice as long as the straight line (plus 0.5 m), and the straight line is clear on our own
        costmap: ToF sees nothing there, so the detour is for something only the Mac believes in."""
        length = _length(here, path, goal)
        loc = self.local_wps
        if loc and loc is not path and math.dist(loc[-1], goal) < 0.6 and length > 1.5 * _length(here, loc, goal) + 0.3:
            # Our own Theta* plans through unknown cells as if free: its shorter way only counts if we SAW it free.
            pts = [tuple(here[:2])] + [tuple(p[:2]) for p in loc[1:]]
            if all(self.costmap.known_free_line(a, b) for a, b in zip(pts, pts[1:])):
                return True
        direct = math.dist(here[:2], goal)
        if length <= 2.0 * direct + 0.5 or direct < 0.7:
            return False
        k = (direct - 0.45) / direct                      # up to 0.45 m short: the person's own legs are there
        end = (here[0] + k * (goal[0] - here[0]), here[1] + k * (goal[1] - here[1]))
        # Clear must mean SEEN clear: with the head on the person the way is mostly unknown, which line_clear
        # takes as free, so the Mac's way round (through the passage) was thrown away for a blind straight line.
        ang = math.atan2(goal[1] - here[1], goal[0] - here[0])
        return self.costmap.line_clear(tuple(here[:2]), end, radius) and \
            self.costmap.seen_free((here[0], here[1], ang), math.dist(here[:2], end))

    def _maybe_plan(self, now: float, here: Pose, goal: Tuple[float, float], keep: float = 0.0) -> None:
        if self.planner is None or self._planning or now - self._plan_t < self.plan_every:
            return
        self._plan_t, self._planning = now, True
        local = self.costmap.snapshot()
        # Ask for a way to keep_m short of the goal, as it will stop there: a person in a doorway stands
        # within a robot's width of the frame, and the way to their very spot went round through the next
        # room, so it turned away from them (Nov 19).
        d = math.dist(here[:2], goal)
        k = max(0.0, d - keep) / d if d > 1e-6 else 1.0
        aim = (here[0] + k * (goal[0] - here[0]), here[1] + k * (goal[1] - here[1]))

        def work() -> None:
            try:
                path = self.planner(here, aim, local, goal)   # goal: the person, so the aim stays on their side
                if path:
                    self.waypoints, self.wp_goal, self._wp_t = [tuple(p) for p in path], goal, self.clock()
            except Exception as exc:                     # noqa: BLE001 — the Mac is optional: carry on locally
                log.info("nav: no global plan (%s): keeping %d waypoints", exc, len(self.waypoints))
            finally:
                self._planning = False

        threading.Thread(target=work, name="nav-plan", daemon=True).start()

    def _plan_local(self, start, goal) -> None:
        try:
            # The Mac's waypoints only while fresh: stale ones lie behind it (Nov 20: with the doorway sealed it
            # turned round toward the Mac's path from 10 s before, away from the person straight ahead).
            fresh = self.waypoints and self.clock() - self._wp_t < GLOBAL_FRESH_S
            self.local_wps = local_path(self.costmap, start, goal, self.params.radius_m,
                                        self.params.margin_m) or (self.waypoints if fresh else [])
        except Exception as exc:                         # noqa: BLE001
            log.info("nav: local plan failed: %s", exc)
        finally:
            self._local_busy = False

    # ---------------------------------------------------------------- stuck

    def _watch_stuck(self, now: float, here: Pose, v: float, ahead: Optional[tuple]) -> None:
        pitch = self.head.pitch() if getattr(self.head, "pitch", None) else None
        if pitch is not None:                             # the floor's pitch: the median of the last 10 s
            self._pitches.append((now, pitch))
            while now - self._pitches[0][0] > 10.0:
                self._pitches.popleft()
        if v <= 0.02:
            self._moving_since = None
            self._stuck.reset()
            return
        if self._moving_since is None:
            self._moving_since = (now, self._odo, pitch)
        # Up from where it set off, or from the floor's level: it may set off with the nose already up (it turned on
        # the spot with the nose over a ledge, then drove on up it: test_scenarios ledge).
        floor = statistics.median(p for _, p in self._pitches) if self._pitches else None
        rise = None if pitch is None or self._moving_since[2] is None else \
            pitch - min(self._moving_since[2], floor if floor is not None else self._moving_since[2])
        c, pan, facing = ahead if ahead else (None, 0.0, False)
        why = self._stuck.update(now, self._odo, c, rise, pan=pan, facing=facing, heading=here[2])
        if why:
            log.warning("nav: stuck: %s", why)
            self.motion.stop()
            self._send(0.0, 0.0)
            if self._stuck.phantom_m > 0.0:
                self.fix.undo(self._stuck.phantom_m, here[2])
                log.info("nav: %.2f m of track slip taken out of the pose", self._stuck.phantom_m)
            at = self.fix()
            kind = self._stuck.kind
            if kind == "floor" and not self._failed(at, "floor", (0.2, 0.0)):
                self._cross(at)                           # spinning on a cable head-on: cross it at an angle
            else:                                         # against something, riding up, or crossing failed too
                self.costmap.add_obstacle(at, [(0.22, y / 20.0) for y in range(-3, 4)])
                if kind != "floor":
                    self._failed(at, kind, (0.22, 0.0), join=True)
            self._stuck.reset()
            self._moving_since = None
            self._recover, self._back_m = "back", BACK_OFF_M
            self._recover_from = (at, self.clock())
            self.state, self.detail = "stuck", why

    def _failed(self, here, kind: str, at_base: Tuple[float, float], join: bool = False) -> list:
        """A way that did not work, at_base (robot frame) from `here`. same_approach: an earlier failure within
        SAME_PLACE_M and SAME_HEADING_DEG (it tried the same thing again). join: one within JOIN_M is the same
        thing, wider than either showed (a glass panel, a ledge): mark the line through both, 0.3 m past each end.
        Returns the earlier failures of this kind within JOIN_M."""
        c, s = math.cos(here[2]), math.sin(here[2])
        pt = (here[0] + c * at_base[0] - s * at_base[1], here[1] + s * at_base[0] + c * at_base[1])
        near = [f for f in self.failures if f[3] == kind and math.dist(f[:2], pt) < JOIN_M]
        if any(math.dist(f[:2], pt) < SAME_PLACE_M and abs(dwa.wrap(f[2] - here[2])) < math.radians(SAME_HEADING_DEG)
               for f in self.failures):
            rules.broke("same_approach", at=[round(pt[0], 2), round(pt[1], 2)], kind=kind)
        self.failures = (self.failures + [(pt[0], pt[1], here[2], kind)])[-30:]
        log.info("nav: %s at %s (%d near)", kind, [round(v, 2) for v in pt], len(near))
        for f in near if join else []:
            d = math.dist(f[:2], pt)
            if d < 0.05:
                continue
            ux, uy = (pt[0] - f[0]) / d, (pt[1] - f[1]) / d
            line = [(f[0] + ux * t - here[0], f[1] + uy * t - here[1]) for t in np.arange(-0.3, d + 0.31, 0.05)]
            self.costmap.add_obstacle(here, [(x * c + y * s, -x * s + y * c) for x, y in line])
            self._plan_t, self._local_t = 0.0, -1e9
        return near

    def _cross(self, here) -> None:
        """The tracks spun on something on the floor head-on (a cable, a rug edge): go again through a point 45 deg
        off the way it was going, on the side with more room. One track at a time climbs it (test_scenarios cable)."""
        best = None
        for side in (1.0, -1.0):
            a = here[2] + side * math.radians(45.0)
            p = (here[0] + 0.8 * math.cos(a), here[1] + 0.8 * math.sin(a))
            cl = float(self.costmap.clearance([p[0]], [p[1]])[0])
            if best is None or cl > best[0] + 0.01:
                best = (cl, p)
        self._via = (best[1], self.clock() + 10.0)
        log.info("nav: tracks spun on the floor: crossing again at 45 deg through %s", [round(v, 2) for v in best[1]])

    def _watch_turn(self, now: float, here: Pose, v: float, w: float) -> bool:
        """Turning on the spot (|w| at least 0.3) while the gyro says under 0.1 rad/s for TURN_BLOCKED_S: a corner is
        held by something it cannot see (a chair leg; the tracks turn on, the encoders count). Mark it at the leading
        corner, brake and back off 10 cm: inside the corners' circle it is in the way of a turn either way, so
        turning the other way hit it with the other corner. True: stop this tick. (test_scenarios wedge: 3.9 s held.)"""
        tw = self.twist() if self.twist else None
        if v > 0.02 or abs(w) < 0.3 or tw is None or abs(tw[1]) >= 0.1 or self._recover:
            self._turn_held = None
            return False
        self._turn_held = self._turn_held or now
        if now - self._turn_held < TURN_BLOCKED_S:
            return False
        self._turn_held = None
        side = 1.0 if w > 0 else -1.0
        corner = [(0.19 * math.cos(math.radians(a)), side * 0.19 * math.sin(math.radians(a))) for a in (40, 47, 54)]
        self.costmap.add_obstacle(here, corner)           # where the leading corner stopped
        self.motion.stop()                                # brake: a ramp down pushed on into it for 0.5 s
        self._failed(here, "turn", corner[1])
        if self._room_behind(here, 0.12):
            self._recover, self._recover_from, self._back_m = "back", (here, now), 0.10
        rules.broke("turn_blocked", at=[round(here[0], 2), round(here[1], 2)], w=round(w, 2))
        return True

    def _do_recover(self, here: Pose) -> None:
        """Back off BACK_OFF_M (slowly, watching the reflex), then hand back to DWA."""
        start, t0 = self._recover_from
        want = getattr(self, "_back_m", BACK_OFF_M)
        gone = math.hypot(here[0] - start[0], here[1] - start[1]) if start else want
        if gone >= want or self.clock() - t0 > 3.0 or not self._room_behind(here, 0.12):   # no room behind
            self._recover = None
            self._send(0.0, 0.0)
            self._plan_t = 0.0
            return
        self._send(-0.08, 0.0)
