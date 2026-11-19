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
does not answer, the last waypoints are kept (stale ones too, until the goal moves away
from them); with none, DWA goes straight for the goal on the local costmap alone. A goal
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

from robot_core import status
from robot_core.nav import dwa
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
CLIMB_DEG = 12.0                # nose up this far since it started moving: climbing something
BACK_OFF_M = 0.10
AHEAD_STALE_S = 1.2             # driving with the way ahead unseen this long: glance at it, slow meanwhile
AHEAD_CONE_DEG = 20.0           # a grid taken with the head this near the direction of travel saw the way
BLIND_MPS = 0.08                # top speed while the way ahead has not been seen


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

    def reset(self) -> None:
        self.hist.clear()

    def update(self, t: float, odo_m: Optional[float], centre_m: Optional[float],
               rise_deg: Optional[float]) -> str:
        if rise_deg is not None and rise_deg > CLIMB_DEG:
            self.phantom_m = 0.0
            return f"nose up {rise_deg:.0f} deg: climbing something"
        if odo_m is None:
            return ""
        self.hist.append((t, odo_m, centre_m))
        while len(self.hist) > 1 and t - self.hist[1][0] >= STUCK_WINDOW_S:
            self.hist.popleft()
        t0, o0, c0 = self.hist[0]
        if t - t0 < STUCK_WINDOW_S or c0 is None or centre_m is None:
            return ""
        went = odo_m - o0
        if went >= STUCK_MIN_ODO_M and max(c0, centre_m) < STUCK_NEAR_M and abs(centre_m - c0) < STUCK_STILL_M:
            self.phantom_m = max(0.0, went - max(0.0, c0 - centre_m))
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
        self.params = params or dwa.DwaParams(radius_m=math.hypot(p.width_m / 2.0, 0.14))
        self.costmap = RollingCostmap(flat_tol_m=max(p.h_max_m, p.flat_tol_m), body_m=p.body_m)
        self.cmd = (0.0, 0.0)
        self.reflex = Reflex(head, motion, lambda: self.cmd, self.fix, bumped,
                             half_width_m=p.width_m / 2.0, clock=clock)
        self.aim = GimbalAim(head.gimbal, tilt_deg)
        self.planner, self.plan_every = planner, plan_every_s
        self.period, self.goal_timeout = 1.0 / hz, goal_timeout_s
        self.goal: Optional[Tuple[float, float]] = None
        self.keep_m, self.goal_t, self.face_goal = 0.0, 0.0, True
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

    def set_goal(self, xy: Optional[Tuple[float, float]], keep_m: float = 0.0, face: bool = True) -> None:
        """Go to `xy` (odometry frame, as `pose`), stopping `keep_m` short; None: stop here.
        Call again to renew it: a goal not renewed for goal_timeout_s is dropped."""
        with self._lock:
            moved = xy is None or self.goal is None or math.dist(xy, self.goal) > 0.3
            self.goal, self.keep_m, self.face_goal, self.goal_t = xy, keep_m, face, self.clock()
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

    def _ingest(self, now: float) -> Optional[float]:
        """Fold the new grids into the costmap; the latest straight-ahead range for StuckWatch."""
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
            travel = max(-45.0, min(45.0, math.degrees(self.aim.k_w * self.cmd[1])))
            if abs(pan - travel) <= AHEAD_CONE_DEG:
                self._ahead_t = now
            if abs(pan) < 10.0:                           # only a look straight ahead says how near the front is
                ahead = centre_range(rows)
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
            self.state, self.detail = "no pose", "waiting for odometry"
            return
        if self._last_pose is not None:
            self._odo += math.hypot(here[0] - self._last_pose[0], here[1] - self._last_pose[1])
        self._last_pose = here
        with self._lock:
            goal, keep, face_goal, goal_t, focus = self.goal, self.keep_m, self.face_goal, self.goal_t, self.focus
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
        if dist <= keep + 0.08:                           # there: face it and stand
            w = dwa.face(bearing) if face_goal else 0.0
            self._send(0.0, w)
            self.state, self.detail = "arrived", f"{dist:.2f} m from the goal"
            self._moving_since = None
            self.aim.update(now, 0.0, w, focus_pan)
            return
        self._maybe_plan(now, here, goal)
        radius = self.params.radius_m + self.params.margin_m
        fresh = self.waypoints and self.wp_goal is not None and math.dist(self.wp_goal, goal) < 0.6 \
            and now - self._wp_t < GLOBAL_FRESH_S
        if fresh:
            path, self.source = self.waypoints, "global"
        else:                                             # no word from the Mac: plan on the costmap
            if now - self._local_t >= LOCAL_EVERY_S and not self._local_busy:
                self._local_t, self._local_busy = now, True
                threading.Thread(target=self._plan_local, args=(here[:2], goal), name="nav-local",
                                 daemon=True).start()        # off the control thread: slow on a Pi
            path, self.source = self.local_wps, "local"
        target = dwa.target_on_path(here, path, self.costmap.clearance, radius) if path else None
        if target is None or math.dist(target, goal) < 0.1:   # the goal itself: stop keep_m short of it
            target, stop_at = goal, keep
        else:
            stop_at = 0.0
        res = dwa.plan((here[0], here[1], here[2], v_meas, w_meas), target, self.costmap.clearance,
                       self.params, stop_at_m=stop_at)
        v, w = res.v, res.w
        if res.blocked:
            self._plan_t, self._local_t = 0.0, -1e9       # plan a way round now (the Mac's and our own)
            w = dwa.face(bearing) if abs(bearing) > math.radians(20) else 0.6   # turn to look for a way
            self.state, self.detail = "blocked", "no safe arc: turning to look"
        else:
            self.state, self.detail = "driving", f"{dist:.2f} m to go"
        if v > 0.02 and now - self._ahead_t > AHEAD_STALE_S:     # not looked where it is going lately
            v = min(v, BLIND_MPS)
            if now >= self.aim.glance_until:
                self.aim.glance(now, math.degrees(self.aim.k_w * w))
        if self.reflex.tripped():
            v = 0.0
            self.state, self.detail = "tripped", self.reflex.why
        if self._trips_seen != self.reflex.trips:         # the brake holds /cmd_vel off until a zero command
            self._trips_seen = self.reflex.trips
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

    def _maybe_plan(self, now: float, here: Pose, goal: Tuple[float, float]) -> None:
        if self.planner is None or self._planning or now - self._plan_t < self.plan_every:
            return
        self._plan_t, self._planning = now, True
        local = self.costmap.snapshot()

        def work() -> None:
            try:
                path = self.planner(here, goal, local)
                if path:
                    self.waypoints, self.wp_goal, self._wp_t = [tuple(p) for p in path], goal, self.clock()
            except Exception as exc:                     # noqa: BLE001 — the Mac is optional: carry on locally
                log.info("nav: no global plan (%s): keeping %d waypoints", exc, len(self.waypoints))
            finally:
                self._planning = False

        threading.Thread(target=work, name="nav-plan", daemon=True).start()

    def _plan_local(self, start, goal) -> None:
        try:
            self.local_wps = local_path(self.costmap, start, goal, self.params.radius_m,
                                        self.params.margin_m) or self.waypoints or []
        except Exception as exc:                         # noqa: BLE001
            log.info("nav: local plan failed: %s", exc)
        finally:
            self._local_busy = False

    # ---------------------------------------------------------------- stuck

    def _watch_stuck(self, now: float, here: Pose, v: float, ahead: Optional[float]) -> None:
        pitch = self.head.pitch() if getattr(self.head, "pitch", None) else None
        if v <= 0.02:
            self._moving_since = None
            self._stuck.reset()
            return
        if self._moving_since is None:
            self._moving_since = (now, self._odo, pitch)
        rise = None if pitch is None or self._moving_since[2] is None else pitch - self._moving_since[2]
        why = self._stuck.update(now, self._odo, ahead, rise)
        if why:
            log.warning("nav: stuck: %s", why)
            self.motion.stop()
            self._send(0.0, 0.0)
            if self._stuck.phantom_m > 0.0:
                self.fix.undo(self._stuck.phantom_m, here[2])
                log.info("nav: %.2f m of track slip taken out of the pose", self._stuck.phantom_m)
            at = self.fix()
            self.costmap.add_obstacle(at, [(0.22, y / 20.0) for y in range(-3, 4)])
            self._stuck.reset()
            self._moving_since = None
            self._recover = "back"
            self._recover_from = (at, self.clock())
            self.state, self.detail = "stuck", why

    def _do_recover(self, here: Pose) -> None:
        """Back off BACK_OFF_M (slowly, watching the reflex), then hand back to DWA."""
        start, t0 = self._recover_from
        gone = math.hypot(here[0] - start[0], here[1] - start[1]) if start else BACK_OFF_M
        if gone >= BACK_OFF_M or self.clock() - t0 > 3.0:
            self._recover = None
            self._send(0.0, 0.0)
            self._plan_t = 0.0
            return
        self._send(-0.08, 0.0)
