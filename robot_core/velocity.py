"""Velocity driving: (forward m/s, turn rad/s) -> signed PWM per physical wheel.

Used by drivetrain_node for /cmd_vel with firmware v6 (`T` frames, see
firmware/drivetrain/drivetrain.ino). Each wheel gets a feed-forward PWM (the dead
band to get moving, plus PWM per m/s) and a PI correction from its measured speed
(the encoder totals, 10 Hz), so a track that runs slow forward or fast in reverse is
held to the same speed as the other. Commands are rate-limited so starts, stops and
turns are smooth. No ROS, no hardware: tests drive it with numbers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class VelocityParams:
    pwm_min: float = 60.0          # PWM that just gets a track moving (the motors' dead band)
    pwm_per_mps: float = 750.0     # extra PWM per m/s of track speed (feed-forward)
    kp: float = 300.0              # PWM per m/s of speed error
    ki: float = 600.0              # PWM per m of accumulated speed error
    i_max: float = 60.0            # most PWM the integral may add or take away
    max_mps: float = 0.25          # forward speed limit
    max_radps: float = 1.0         # turn rate limit
    accel_mps2: float = 0.4        # how fast the forward speed may change
    accel_radps2: float = 2.0      # how fast the turn rate may change
    yaw_ki: float = 1.5            # rad/s of extra turn per rad the gyro fell behind the asked turn rate
    yaw_i_max: float = 0.5         # most turn rate it may add or take away (rad/s)
    pwm_max: float = 255.0


def params_from(p: dict) -> VelocityParams:
    """From robot.yaml's /drivetrain section (velocity_*); missing keys keep the defaults."""
    d = VelocityParams()
    return VelocityParams(**{k: float(p.get(f"velocity_{k}", getattr(d, k))) for k in vars(d)})


class WheelSpeedController:
    def __init__(self, wheelbase_m: float, params: VelocityParams = VelocityParams()) -> None:
        self.wheelbase_m, self.p = wheelbase_m, params
        self.reset()

    def reset(self) -> None:
        self.v = self.w = 0.0                         # the rate-limited command actually driven
        self._i = [0.0, 0.0]
        self._iw = 0.0                                # accumulated turn-rate error (rad), from the gyro

    def targets(self, v: float, w: float) -> Tuple[float, float]:
        """Track speeds (left, right) in m/s for forward speed v and turn rate w (CCW +)."""
        half = self.wheelbase_m / 2.0
        left, right = v - w * half, v + w * half
        # The outer track cannot go faster than max_mps: keep the turn, give up forward speed. At full
        # speed the outer track saturated and the robot turned at a third of what was asked (Nov 19).
        over = max(abs(left), abs(right)) - self.p.max_mps
        if over > 0.0 and v != 0.0:
            s = math.copysign(min(over, abs(v)), v)
            left, right = left - s, right - s
        return left, right

    def step(self, v_cmd: float, w_cmd: float, meas: Optional[Tuple[float, float]],
             dt: float, yaw_rate: Optional[float] = None) -> Tuple[int, int]:
        """One control tick: the PWM (left, right) to send. `meas`: the tracks' measured
        speeds (m/s, forward +), or None when there is no fresh telemetry (then
        feed-forward only, and the integral is held). `yaw_rate`: the gyro's turn rate (rad/s,
        CCW +), or None: the tracks skid, so equal and opposite track speeds turn the body only
        about half as fast as asked; the gyro trims the turn rate up to what was asked."""
        p = self.p
        v_cmd = max(-p.max_mps, min(p.max_mps, v_cmd))
        w_cmd = max(-p.max_radps, min(p.max_radps, w_cmd))
        self.v += max(-p.accel_mps2 * dt, min(p.accel_mps2 * dt, v_cmd - self.v))
        self.w += max(-p.accel_radps2 * dt, min(p.accel_radps2 * dt, w_cmd - self.w))
        if v_cmd == 0.0 and w_cmd == 0.0 and abs(self.v) < 0.01 and abs(self.w) < 0.05:
            self.reset()
            return 0, 0
        w = self.w
        if yaw_rate is not None and p.yaw_ki > 0.0:
            lim = p.yaw_i_max / p.yaw_ki
            self._iw = max(-lim, min(lim, self._iw + (self.w - yaw_rate) * dt))
            w = self.w + p.yaw_ki * self._iw
        out = []
        for k, target in enumerate(self.targets(self.v, w)):
            if abs(target) < 0.005:
                self._i[k] = 0.0
                out.append(0)
                continue
            ff = math.copysign(p.pwm_min + abs(target) * p.pwm_per_mps, target)
            fb = 0.0
            if meas is not None:
                err = target - meas[k]
                lim = p.i_max / p.ki if p.ki else 0.0
                self._i[k] = max(-lim, min(lim, self._i[k] + err * dt))
                fb = p.kp * err + p.ki * self._i[k]
            pwm = ff + fb
            if pwm * target < 0:                      # feedback never drives a track backwards
                pwm = 0.0
            out.append(int(round(max(-p.pwm_max, min(p.pwm_max, pwm)))))
        return out[0], out[1]
