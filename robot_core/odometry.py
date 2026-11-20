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
    w: float                 # rad/s, CCW (from the gyro when one is fitted)
    distance_m: float        # total path length since start
    sigma_xy: float          # 1-sigma position uncertainty, metres
    sigma_theta: float       # 1-sigma heading uncertainty, radians
    t: float                 # host monotonic time of the last sample
    samples: int
    w_wheels: float = 0.0    # rad/s the encoders alone say; differs from w when the tracks slip


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
        k_gyro: float = 0.01,              # with a gyro: 1 % of every turn
    ) -> None:
        self.ticks_per_m = ticks_per_m
        self.wheelbase_m = wheelbase_m
        self.swap_sides = swap_sides
        self._kd, self._kt, self._kh, self._kg = k_distance, k_turn, k_heading_per_m, k_gyro
        self._last_yaw: Optional[float] = None
        self._last_t: Optional[float] = None        # host time of the last sample (the gyro's clock)
        self._w_wheels = 0.0
        self._lock = threading.Lock()
        self._last: Optional[tuple] = None          # (left, right, fw_ms)
        self._pose = Pose2D()
        self._v = self._w = 0.0
        self._dist = 0.0
        self._var_xy = 0.0
        self._var_th = 0.0
        self._t = 0.0
        self._n = 0

    def update(self, total_left: int, total_right: int, fw_ms: int, t_host: float,
               yaw: Optional[float] = None) -> OdomState:
        """Feed one telemetry sample (firmware channel order). `yaw` is the gyro's
        heading in radians (unwrapped, CCW +) when an IMU is fitted: the heading
        then follows the gyro, which a slipping track cannot fool, and the
        encoders only say how far."""
        if self.swap_sides:
            total_left, total_right = total_right, total_left
        with self._lock:
            self._n += 1
            self._t = t_host
            if self._last is None or fw_ms < self._last[2]:
                # First sample, or the Arduino rebooted (millis went back):
                # re-baseline rather than integrate a jump from/to zero.
                self._last = (total_left, total_right, fw_ms)
                self._last_yaw, self._last_t = yaw, t_host
                self._v = self._w = self._w_wheels = 0.0
                return self._state()
            dl = (total_left - self._last[0]) / self.ticks_per_m
            dr = (total_right - self._last[1]) / self.ticks_per_m
            dt = max(1e-3, (fw_ms - self._last[2]) / 1000.0)
            self._last = (total_left, total_right, fw_ms)

            d = (dl + dr) / 2.0
            dth = (dr - dl) / self.wheelbase_m
            self._w_wheels = dth / dt
            k_turn = self._kt
            w_now = dth / dt
            if yaw is not None and self._last_yaw is not None:
                dth, k_turn = yaw - self._last_yaw, self._kg
                # The gyro's heading is read on the Pi's clock as each sample arrives, so its rate is
                # over host time, not the firmware's dt: frames that arrive bunched made it spike to
                # ~2x the real turn rate (Nov 19).
                w_now = dth / max(1e-3, t_host - (self._last_t if self._last_t is not None else t_host - dt))
            self._last_yaw, self._last_t = yaw, t_host
            mid = self._pose.theta + dth / 2.0
            self._pose = Pose2D(self._pose.x + d * math.cos(mid),
                                self._pose.y + d * math.sin(mid),
                                wrap(self._pose.theta + dth))
            self._v = d / dt
            self._w = 0.7 * self._w + 0.3 * w_now            # light smoothing: one late frame is not a spin
            self._dist += abs(d)
            self._var_th += (k_turn * dth) ** 2 + (self._kh * abs(d)) ** 2
            self._var_xy += (self._kd * d) ** 2 + (abs(d) ** 2) * self._var_th
            return self._state()

    def state(self) -> OdomState:
        with self._lock:
            return self._state()

    def _state(self) -> OdomState:
        return OdomState(self._pose, self._v, self._w, self._dist,
                         math.sqrt(self._var_xy), math.sqrt(self._var_th), self._t, self._n,
                         self._w_wheels)

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

IDLE, MOVING, STALLED, SLIPPING, PUSHED, WRONG_DIRECTION, TILTED, BUMPED = (
    "idle", "moving", "stalled", "slipping", "pushed", "wrong_direction", "tilted", "bumped")

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
    pitch_deg: float = 0.0      # from the IMU; 0 without one
    roll_deg: float = 0.0

    @property
    def stuck(self) -> bool:
        """The move should stop: the robot is not going anywhere, or is tipping."""
        return self.state in (STALLED, SLIPPING, TILTED)


class MotionHealth:
    """Cross-checks three independent opinions of "are we moving":

    1. the command    — the executor is running a move (and which kind)
    2. the wheels     — encoder speed from the odometry
    3. the eyes       — `VisualMotion` from the camera, when available

    Command + no wheel speed      → STALLED   (motor blocked, battery sag)
    Command + wheels + no image  → SLIPPING  (wheels spinning, robot pinned)
    No command + wheels          → PUSHED    (picked up, bumped, rolling)
    Turn measured the wrong way  → WRONG_DIRECTION (swap_sides is wrong)

    With an IMU (`update(..., imu=state)`), two more, and a better slip check:

    Pitch or roll past the limit → TILTED    (climbing something too steep, tipping)
    Nose dropping on a drive     → TILTED    (going over an edge: the front falls more than
                                              max_dip_deg below where the move started, and
                                              fast; driving backwards, the tail. Not held:
                                              it stops the move at once)
    Nose rising on a drive       → TILTED    (climbing something: a wall, a chair base. The front
                                              rises more than max_climb_deg above where the
                                              move started; Nov 19 the tracks climbed a wall
                                              to 34 deg with the encoders still counting)
    A jolt while moving          → BUMPED    (hit something; reported, not stopped on)
    Tracks turning, gyro still   → SLIPPING  (on a turn the gyro sees what the camera may not)

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
        max_pitch_deg: float = 25.0,
        max_roll_deg: float = 20.0,
        bump_g: float = 0.8,
        max_dip_deg: float = 8.0,
        dip_window_s: float = 0.5,
        max_climb_deg: float = 12.0,
    ) -> None:
        self.start_grace_s, self.stall_s, self.slip_s, self.pushed_s = (
            start_grace_s, stall_s, slip_s, pushed_s)
        self.min_speed, self.min_turn = min_speed, min_turn
        self.visual_max_age_s = visual_max_age_s
        self.max_pitch_deg, self.max_roll_deg, self.bump_g = max_pitch_deg, max_roll_deg, bump_g
        self.max_dip_deg, self.dip_window_s = max_dip_deg, dip_window_s
        self.max_climb_deg = max_climb_deg
        self._dip_ref: Optional[float] = None     # pitch when the drive started
        self._pitches: list = []                  # (t, pitch) over the last dip_window_s
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

    def update(self, odom: OdomState, op: Optional[str], now: float, imu=None) -> HealthReport:
        """`imu` is an ImuState (robot_core/sensors/imu.py) or None."""
        # Track the command, and drift over each straight move as it ends.
        if op != self._op:
            if self._op in ("straight", "reverse") and self._straight_start is not None:
                self._fold_drift(self._straight_start, odom)
            self._op, self._op_since = op, now
            self._cond_since.clear()
            self._dip_ref, self._pitches = (imu.pitch_deg if imu is not None else None), []
            self._straight_start = odom if op in ("straight", "reverse") else None

        visual = None
        if self._visual and now - self._visual[1] <= self.visual_max_age_s:
            visual = self._visual[0]
        w_wheels = odom.w_wheels if imu is not None else odom.w
        wheels = abs(odom.v) >= self.min_speed or abs(w_wheels) >= self.min_turn
        pitch = imu.pitch_deg if imu is not None else 0.0
        roll = imu.roll_deg if imu is not None else 0.0
        in_grace = op is not None and now - self._op_since < self.start_grace_s

        state, reason = (MOVING, "") if op else (IDLE, "")
        if op and not in_grace:
            fwd, turn = _EXPECT.get(op, (0, 0))
            if self._held("stall", not wheels, now, self.stall_s):
                state, reason = STALLED, f"{op} commanded but the wheels are not turning"
            elif self._held("slip", wheels and visual is False, now, self.slip_s):
                state, reason = SLIPPING, f"wheels turning during {op} but the camera sees no motion"
            elif self._held("gyro-slip", imu is not None and turn != 0
                            and abs(w_wheels) >= self.min_turn
                            and abs(math.radians(imu.yaw_rate_dps)) < 0.3 * abs(w_wheels),
                            now, self.slip_s):
                state, reason = SLIPPING, f"tracks turning during {op} but the gyro says the robot is not"
            elif self._held("wrong", turn != 0 and odom.w * turn < -self.min_turn, now, 0.5):
                state, reason = WRONG_DIRECTION, (
                    f"commanded {op} but measured the opposite rotation — "
                    "check odometry swap_sides")
        elif op is None:
            if self._held("pushed", wheels, now, self.pushed_s):
                state, reason = PUSHED, "wheels turning with no command (bumped or lifted?)"

        if imu is not None:
            dip = self._dip(op, pitch, now)
            if self._held("tilt", abs(pitch) > self.max_pitch_deg or abs(roll) > self.max_roll_deg, now, 0.3):
                state, reason = TILTED, f"tilted: pitch {pitch:.0f} deg, roll {roll:.0f} deg"
            elif dip:
                state, reason = TILTED, dip
            elif op and not in_grace and state == MOVING and imu.jolt_g > self.bump_g:
                state, reason = BUMPED, f"jolt of {imu.jolt_g:.1f} g during {op}"

        if state != self._state:
            self._state, self._state_since = state, now
        return HealthReport(state, reason, odom.v, odom.w, odom.sigma_xy, odom.sigma_theta,
                            self._drift, visual, now - self._state_since, pitch, roll)

    def _dip(self, op: Optional[str], pitch: float, now: float) -> str:
        """Going over an edge on a drive: why, or "". The front (the back, reversing) has
        fallen more than max_dip_deg below the pitch the move started at, and at least half
        of that within the last dip_window_s. A sill crossed and left behind comes back to
        the start pitch, and a slow ramp is not fast: neither stops the move."""
        if op not in ("straight", "reverse") or self._dip_ref is None:
            return ""
        self._pitches = [(t, p) for t, p in self._pitches if now - t <= self.dip_window_s] + [(now, pitch)]
        sign = 1.0 if op == "straight" else -1.0
        fall = sign * (self._dip_ref - pitch)
        recent = max(sign * (p - pitch) for _, p in self._pitches)
        if fall > self.max_dip_deg and recent > self.max_dip_deg / 2.0:
            return (f"{'nose' if sign > 0 else 'tail'} dropped {fall:.0f} deg on a drive: over an edge?")
        if -fall > self.max_climb_deg:
            return (f"{'nose' if sign > 0 else 'tail'} rose {-fall:.0f} deg on a drive: climbing something?")
        return ""

    def _fold_drift(self, start: OdomState, end: OdomState) -> None:
        dist = end.distance_m - start.distance_m
        if dist < 0.2:
            return                       # too short to say anything
        dth = math.degrees(wrap(end.pose.theta - start.pose.theta))
        self._drift = 0.7 * self._drift + 0.3 * (dth / dist)


# --------------------------------------------------------------------------
# Turn correction (needs the gyro)
