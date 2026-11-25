"""The emergency reflex, and the one place ToF grids come in.

A thread at 50 Hz. On every tick it records the gimbal's angles and the robot's pose (so a
grid can later be placed where the head really pointed when it was measured: the head keeps
moving). When the ToF has a new grid (about 5 a second on I2C), it projects it once and:

  * brakes at once if any zone reads nearer than `stop_m` (25 cm) and lies where the robot
    is about to go: ahead of it while driving forward, anywhere round it while turning on
    the spot. A reading inside the robot's own footprint is the robot (its deck at the
    bottom of the view) and is ignored. A wall 20 cm to the side of a robot driving
    straight past it does not brake it;
  * hands the projected grid to the controller (`take()`) for the costmap.

It also brakes when the drivetrain reports a bump, stall, slip or climb (`bumped()`), within
a tick. A trip holds forward motion off for `hold_s`; turning on the spot stays allowed, so
the controller can turn away. The reflex brakes with motion.stop() (out of band, in the
drivetrain) and does not wait for the control loop.

The sensor's own rate is the limit: at 0.22 m/s the robot moves 4.4 cm between grids.
"""

from __future__ import annotations

import bisect
import logging
import math
import threading
import time
from collections import deque
from typing import Callable, List, Optional, Tuple

from robot_core import debug_upload

log = logging.getLogger(__name__)

SENSOR_LAG_S = 0.1          # a grid describes the scene about this long before it arrives


class Reflex:
    def __init__(self, head, motion, command: Callable[[], Tuple[float, float]],
                 pose: Callable[[], Optional[tuple]], bumped: Optional[Callable[[], float]] = None,
                 stop_m: float = 0.25, hold_s: float = 0.6, hz: float = 50.0,
                 front_m: float = 0.14, back_m: float = 0.14, half_width_m: float = 0.13,
                 path_margin_m: float = 0.03, clock: Callable[[], float] = time.monotonic) -> None:
        self.head, self.motion, self.command, self.pose, self.bumped = head, motion, command, pose, bumped
        self.stop_m, self.hold_s, self.period = stop_m, hold_s, 1.0 / hz
        self.front, self.back, self.half, self.margin = front_m, back_m, half_width_m, path_margin_m
        self.clock = clock
        self._hist: deque = deque(maxlen=int(hz * 2))          # (t, pan, tilt, pose)
        self._grids: deque = deque(maxlen=8)                   # (t, pose, zones, rows, pan) for the costmap
        self._last_grid_t = 0.0
        self._t0 = clock()
        self.tripped_until = 0.0
        self.why = ""
        self.trips = 0
        self.hits: List[Tuple[float, float]] = []        # where (robot frame) the last trip found something
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------- lifecycle

    def start(self) -> "Reflex":
        self._t0 = self.clock()
        self._thread = threading.Thread(target=self._run, name="nav-reflex", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = self.clock()
            try:
                self.tick()
            except Exception as exc:                     # noqa: BLE001 — never let the reflex die quietly
                log.warning("reflex: %s: %s", type(exc).__name__, exc)
            self._stop.wait(max(0.0, self.period - (self.clock() - t0)))

    # ------------------------------------------------------------------ work

    def tripped(self) -> bool:
        return self.clock() < self.tripped_until

    def take(self) -> List[tuple]:
        """The grids that came in since the last call: (t, pose, zones, rows, pan) each."""
        with self._lock:
            out = list(self._grids)
            self._grids.clear()
        return out

    def _at(self, t: float):
        """(pan, tilt, pose) recorded nearest before time t."""
        with self._lock:
            hist = list(self._hist)
        if not hist:
            return None
        i = bisect.bisect_right([h[0] for h in hist], t) - 1
        return hist[max(0, i)][1:]

    def trip(self, why: str, count: bool = False) -> None:
        """count: a trip even while one holds (the drivetrain braked on its own: the controller must send
        the zero command that lets it drive again)."""
        now = self.clock()
        fresh = now >= self.tripped_until
        self.tripped_until = now + self.hold_s
        self.why = why
        if fresh or count:
            self.trips += 1
        if fresh:
            self.motion.stop()
            log.warning("reflex: %s", why)

    def tick(self) -> None:
        now = self.clock()
        pan, tilt = self.head.gimbal.angles()
        pose = self.pose()
        still = self.head.gimbal.settled(0.15) if self.head.gimbal is not None else True
        debug_upload.note_gimbal(pan, tilt, now, pose, head_still=still)    # debug mode: each upload's ToF grids carry the head + pose + settled state
        with self._lock:
            self._hist.append((now, pan, tilt, pose))
        if self.bumped is not None and (self.bumped() or 0.0) > max(self._t0, self.tripped_until - self.hold_s):
            if self.command()[0] > 0.02:                  # driving forward into it: it is just ahead of the nose
                self.hits = [(self.front + 0.06, y / 20.0) for y in range(-5, 6)]   # felt, not seen: as wide as
                # the body and more, or it bumped the same unseen thing again beside the mark (test_scenarios glass)
            self.trip("the drivetrain felt something (bump, stall, slip or climb)", count=True)
        g = self.head.lidar.grid() if hasattr(self.head.lidar, "grid") else None
        if g is None or g.t <= self._last_grid_t:
            return
        self._last_grid_t = g.t
        at = self._at(g.t - SENSOR_LAG_S) or (pan, tilt, self.pose())
        # Nearer than self_range_m is the robot itself: the bottom-left zones read its own body at 7-9 cm
        # in a fifth of all grids (Nov 19), and braking for them stopped it every second or two.
        zones = self.head.project(g.rows, at[0], at[1])
        why = self.danger(zones)
        if why:
            self.trip(why)
        with self._lock:                                  # the costmap does not want the robot's own body
            self._grids.append((g.t, at[2], [z for z in zones if not self._own(z[2], z[3])], g.rows, at[0]))

    def _own(self, x: float, y: float) -> bool:
        return -self.back <= x <= self.front and abs(y) <= self.half

    def danger(self, zones) -> str:
        """Why to brake now, or "": a zone nearer than stop_m where the robot is going.
        Floor and near-ground returns under the nose/tracks must never trip an emergency stop."""
        v, w = self.command()
        for r, c, x, y, h, d in zones:
            if d >= self.stop_m or self._own(x, y):
                continue
            # Row 7 looks directly down at the floor immediately ahead/under the nose; never brake on it
            if r >= 7:
                continue
            # Floor, ground undulations, and climbable sills (<= 6 cm) are not collision obstacles
            if h is not None and h <= 0.06:
                continue
            # A reading with no verified height (glancing/unfitted ground return) is not an emergency hazard
            if h is None:
                continue
            # For row 6 at a steep downward glance, require solid height above ground (> 8 cm)
            if r == 6 and h <= 0.08:
                continue
            if v > 0.02 and x > 0.0 and abs(y) <= self.half + self.margin:
                self.hits = [(x, y)]
                return f"something {d:.2f} m from the sensor, ahead"
            if abs(w) > 0.1 and math.hypot(x, y) <= math.hypot(self.front, self.half) + self.margin:
                self.hits = [(x, y)]
                return f"something {d:.2f} m from the sensor, in the way of the turn"
        return ""
