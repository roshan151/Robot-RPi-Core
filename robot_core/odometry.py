"""Wheel odometry, and the motion-health signals built on top of it.

Pure Python, no ROS (Layer A). `drivetrain_node` feeds it the firmware's
never-reset encoder totals (v5 `E` frames) and publishes the results as
`/odom`, the `odom -> base_link` transform and `/motion_health`.

Conventions (REP-103): x forward, y left, theta counter-clockwise, metres and
radians.

Why the sides are swapped by default
------------------------------------
The harness wires the firmware's LEFT channel to the physical RIGHT wheel (see
the note above `wheelSigns()` in the firmware). Motor and encoder stay paired,
so the firmware's "left" count is the physical right wheel. `swap_sides=True`
undoes that here; `MotionHealth` flags `wrong_direction` if a commanded turn
ever measures the opposite way, which is what a wrong setting looks like.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional


def wrap(a: float) -> float:
    """Angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def compose(self, d: "Pose2D") -> "Pose2D":
        """self ⊕ d: apply `d`, expressed in self's frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(self.x + c * d.x - s * d.y, self.y + s * d.x + c * d.y,
                      wrap(self.theta + d.theta))

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, -self.theta)

    def between(self, other: "Pose2D") -> "Pose2D":
        """The motion from self to other, in self's frame (self⁻¹ ⊕ other)."""
        return self.inverse().compose(other)

    def as_tuple(self) -> tuple:
        return (self.x, self.y, self.theta)


def effective_wheelbase_m(ticks_per_cm: float, ticks_per_degree: float) -> float:
    """Track width implied by the two calibrations you already did.

    A tank turn of A degrees drives each wheel A * ticks_per_degree ticks, and
    each wheel travels an arc of (wheelbase / 2) * A (in radians). Solving gives
    the *effective* wheelbase — which already includes wheel scrub, so it is a
    better number for odometry than a tape measure between the tyres.
    """
    return 2.0 * ticks_per_degree * 180.0 / (math.pi * ticks_per_cm) / 100.0


@dataclass(frozen=True)
class OdomState:
    pose: Pose2D
    v: float                 # m/s, forward
    w: float                 # rad/s, CCW
    distance_m: float        # total path length since start
    sigma_xy: float          # 1-sigma position uncertainty, metres
    sigma_theta: float       # 1-sigma heading uncertainty, radians
    t: float                 # host monotonic time of the last sample
    samples: int


class DiffDriveOdometry:
    """Integrates never-reset encoder totals into a pose, with a growing error
    estimate that SLAM shrinks again (`correct()`).

    The uncertainty model is deliberately simple — it only has to say *when*
    the pose is no longer worth trusting, so the explorer knows to stop and
    re-localise: distance error grows with distance, heading error grows with
    both turning and driving, and heading error times distance becomes
    position error.
    """

    def __init__(
        self,
        ticks_per_m: float,
        wheelbase_m: float,
        swap_sides: bool = True,
        k_distance: float = 0.03,          # 3 % of distance, 1-sigma
        k_turn: float = 0.04,              # 4 % of every turn
        k_heading_per_m: float = math.radians(1.5),
    ) -> None:
        self.ticks_per_m = ticks_per_m
        self.wheelbase_m = wheelbase_m
        self.swap_sides = swap_sides
        self._kd, self._kt, self._kh = k_distance, k_turn, k_heading_per_m
        self._lock = threading.Lock()
        self._last: Optional[tuple] = None          # (left, right, fw_ms)
        self._pose = Pose2D()
        self._v = self._w = 0.0
        self._dist = 0.0
        self._var_xy = 0.0
        self._var_th = 0.0
        self._t = 0.0
        self._n = 0

    def update(self, total_left: int, total_right: int, fw_ms: int, t_host: float) -> OdomState:
        """Feed one telemetry sample (firmware channel order)."""
        if self.swap_sides:
            total_left, total_right = total_right, total_left
        with self._lock:
            self._n += 1
            self._t = t_host
            if self._last is None or fw_ms < self._last[2]:
                # First sample, or the Arduino rebooted (millis went back):
                # re-baseline rather than integrate a jump from/to zero.
                self._last = (total_left, total_right, fw_ms)
                self._v = self._w = 0.0
                return self._state()
            dl = (total_left - self._last[0]) / self.ticks_per_m
            dr = (total_right - self._last[1]) / self.ticks_per_m
            dt = max(1e-3, (fw_ms - self._last[2]) / 1000.0)
            self._last = (total_left, total_right, fw_ms)

            d = (dl + dr) / 2.0
            dth = (dr - dl) / self.wheelbase_m
            mid = self._pose.theta + dth / 2.0
            self._pose = Pose2D(self._pose.x + d * math.cos(mid),
                                self._pose.y + d * math.sin(mid),
                                wrap(self._pose.theta + dth))
            self._v, self._w = d / dt, dth / dt
            self._dist += abs(d)
            self._var_th += (self._kt * dth) ** 2 + (self._kh * abs(d)) ** 2
            self._var_xy += (self._kd * d) ** 2 + (abs(d) ** 2) * self._var_th
            return self._state()

    def state(self) -> OdomState:
        with self._lock:
            return self._state()

    def _state(self) -> OdomState:
        return OdomState(self._pose, self._v, self._w, self._dist,
                         math.sqrt(self._var_xy), math.sqrt(self._var_th), self._t, self._n)

    def correct(self, sigma_xy: float = 0.05, sigma_theta: float = math.radians(2)) -> None:
        """SLAM has re-localised us: the odom pose itself is left alone (map→odom
        absorbs the correction, as in REP-105), but the drift budget restarts
        from SLAM's own uncertainty."""
        with self._lock:
            self._var_xy = sigma_xy ** 2
            self._var_th = sigma_theta ** 2


# --------------------------------------------------------------------------
# Motion health
# --------------------------------------------------------------------------

IDLE, MOVING, STALLED, SLIPPING, PUSHED, WRONG_DIRECTION = (
    "idle", "moving", "stalled", "slipping", "pushed", "wrong_direction")

# What a commanded op should do: (forward sign, turn sign). Turn sign is CCW+,
# so a commanded RIGHT turn must measure w < 0.
_EXPECT = {"straight": (1, 0), "reverse": (-1, 0), "left": (0, 1), "right": (0, -1)}


@dataclass(frozen=True)
class HealthReport:
    state: str
    reason: str
    v: float
    w: float
    sigma_xy: float
    sigma_theta: float
    drift_deg_per_m: float      # heading change per metre on straight moves (EWMA)
    visual_moving: Optional[bool]
    since_s: float              # how long the current state has held

    @property
    def stuck(self) -> bool:
        return self.state in (STALLED, SLIPPING)


class MotionHealth:
    """Cross-checks three independent opinions of "are we moving":

    1. the command    — the executor is running a move (and which kind)
    2. the wheels     — encoder speed from the odometry
    3. the eyes       — `VisualMotion` from the camera, when available

    Command + no wheel speed      → STALLED   (motor blocked, battery sag)
    Command + wheels + no image  → SLIPPING  (wheels spinning, robot pinned)
    No command + wheels          → PUSHED    (picked up, bumped, rolling)
    Turn measured the wrong way  → WRONG_DIRECTION (swap_sides is wrong)

    Every threshold needs to hold for a while before it fires — a move spends
    its first ~0.5 s in brake dead-time and ramp, which must not read as a stall.
    """

    def __init__(
        self,
        start_grace_s: float = 0.8,
        stall_s: float = 1.0,
        slip_s: float = 1.5,
        pushed_s: float = 0.5,
        min_speed: float = 0.015,          # m/s
        min_turn: float = math.radians(5),  # rad/s
        visual_max_age_s: float = 1.0,
    ) -> None:
        self.start_grace_s, self.stall_s, self.slip_s, self.pushed_s = (
            start_grace_s, stall_s, slip_s, pushed_s)
        self.min_speed, self.min_turn = min_speed, min_turn
        self.visual_max_age_s = visual_max_age_s
        self._op: Optional[str] = None
        self._op_since = 0.0
        self._cond_since: dict = {}
        self._state = IDLE
        self._state_since = 0.0
        self._visual: Optional[tuple] = None      # (moving, t)
        self._drift = 0.0
        self._straight_start: Optional[OdomState] = None

    def on_visual(self, moving: bool, t: float) -> None:
        self._visual = (bool(moving), t)

    def _held(self, name: str, cond: bool, now: float, need_s: float) -> bool:
        if not cond:
            self._cond_since.pop(name, None)
            return False
        start = self._cond_since.setdefault(name, now)
        return now - start >= need_s

    def update(self, odom: OdomState, op: Optional[str], now: float) -> HealthReport:
        # Track the command, and drift over each straight move as it ends.
        if op != self._op:
            if self._op in ("straight", "reverse") and self._straight_start is not None:
                self._fold_drift(self._straight_start, odom)
            self._op, self._op_since = op, now
            self._cond_since.clear()
            self._straight_start = odom if op in ("straight", "reverse") else None

        visual = None
        if self._visual and now - self._visual[1] <= self.visual_max_age_s:
            visual = self._visual[0]
        wheels = abs(odom.v) >= self.min_speed or abs(odom.w) >= self.min_turn
        in_grace = op is not None and now - self._op_since < self.start_grace_s

        state, reason = (MOVING, "") if op else (IDLE, "")
        if op and not in_grace:
            fwd, turn = _EXPECT.get(op, (0, 0))
            if self._held("stall", not wheels, now, self.stall_s):
                state, reason = STALLED, f"{op} commanded but the wheels are not turning"
            elif self._held("slip", wheels and visual is False, now, self.slip_s):
                state, reason = SLIPPING, f"wheels turning during {op} but the camera sees no motion"
            elif self._held("wrong", turn != 0 and odom.w * turn < -self.min_turn, now, 0.5):
                state, reason = WRONG_DIRECTION, (
                    f"commanded {op} but measured the opposite rotation — "
                    "check odometry swap_sides")
        elif op is None:
            if self._held("pushed", wheels, now, self.pushed_s):
                state, reason = PUSHED, "wheels turning with no command (bumped or lifted?)"

        if state != self._state:
            self._state, self._state_since = state, now
        return HealthReport(state, reason, odom.v, odom.w, odom.sigma_xy, odom.sigma_theta,
                            self._drift, visual, now - self._state_since)

    def _fold_drift(self, start: OdomState, end: OdomState) -> None:
        dist = end.distance_m - start.distance_m
        if dist < 0.2:
            return                       # too short to say anything
        dth = math.degrees(wrap(end.pose.theta - start.pose.theta))
        self._drift = 0.7 * self._drift + 0.3 * (dth / dist)
