"""
Intent-level drivetrain API on the Raspberry Pi — v3.
Key changes vs v2:
  - Public straight/reverse API now takes METERS, not duration seconds.
    Internally converted to encoder ticks via settings.TICKS_PER_CM so the
    Arduino auto-stops when the target distance is reached — no time.sleep().
  - Duration-based straight()/reverse() removed. All straight moves are
    encoder-controlled end-to-end.
  - _verify_encoder_delta() runs after every encoder-counted move and logs
    warnings for stalled wheels or excessive sync error.
  - Calibration constants (TICKS_PER_CM, TICKS_PER_DEGREE) live in settings.py
    so they can be tuned via environment variables without touching code.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .arduino_bridge import ArduinoBridge
from robot_core import settings

logger = logging.getLogger(__name__)

# Moves whose target is an ANGLE rather than a distance. Used so the
# diagnostics name the calibration constant that actually governs the move —
# telling an operator to tune TICKS_PER_CM after a bad turn sends them to
# recalibrate the one constant that had no part in it.
_TURN_DIRS = frozenset({"L", "R"})

def duty_from_percent(speed_percent: float) -> int:
    """Map 0–100 % speed to 0–255 PWM duty cycle."""
    return max(0, min(255, int(round(speed_percent / 100.0 * 255.0))))


def _speed(speed: Optional[float]) -> float:
    """Resolve a speed argument against the configured default.

    Ramping is handled by the firmware (RAMP_STEP/RAMP_MS in drivetrain.ino,
    ~0.5 PWM per ms up, 2x that on softStop), which is what keeps the inrush
    and back-EMF off the supply rail.  Nothing here should ramp — a second
    ramp on the host would only fight it.
    """
    return settings.DEFAULT_SPEED_PERCENT if speed is None else float(speed)


def _ticks_per_metre() -> float:
    """Encoder ticks per metre, derived from settings.TICKS_PER_CM."""
    return settings.TICKS_PER_CM * 100.0


def _turn_ticks(angle: float) -> int:
    """Tick target for a turn, less the coast the robot adds while braking.

    The firmware stops counting when the target is reached, but the robot is
    still moving at that instant and carries on for settings.TURN_COAST_TICKS.
    Aiming short by exactly that much is what makes the FINAL heading correct
    rather than the heading at the moment the brake engages.
    """
    raw   = angle * settings.TICKS_PER_DEGREE
    coast = settings.TURN_COAST_TICKS
    ticks = int(round(raw - coast))

    if ticks < 1:
        # The turn is smaller than the distance the robot coasts, so it is not
        # achievable at this speed by braking alone -- whatever target we set,
        # momentum overshoots it. Say so plainly instead of silently sending a
        # 1-tick move and letting the caller believe a 2 deg turn happened.
        min_deg = coast / settings.TICKS_PER_DEGREE if settings.TICKS_PER_DEGREE else 0
        logger.warning(
            "turn of %.1f deg is below the coast floor (~%.1f deg at the "
            "current speed): the robot overshoots it regardless of target. "
            "Clamping to 1 tick. Lower the turn speed to reduce coast, or "
            "recalibrate ROBOT_TURN_COAST_TICKS if this looks wrong.",
            angle, min_deg,
        )
        return 1
    return ticks


class SerialDrivetrain:
    def __init__(
        self,
        port: str = settings.SERIAL_PORT,
        baud: int = settings.BAUD_RATE,
    ) -> None:
        self._enc_lock     = threading.Lock()
        self._motor1_count = 0
        self._motor2_count = 0

        # Counts latched off the last move's D frame, or None if that move
        # fell back to the telemetry path.  Kept apart from the live counters
        # above, which the reader thread keeps overwriting from E frames.
        self._last_move_counts: Optional[tuple] = None

        # Default base speed used by intent (open-loop) commands.
        self._base_speed_pct: float = 60.0

        def on_enc(left: int, right: int) -> None:
            with self._enc_lock:
                self._motor1_count = left
                self._motor2_count = right

        def on_err() -> None:
            logger.error("Arduino reported ERR (unsolicited)")

        # Odometry subscribers get the never-reset totals from v5 firmware.
        # Registered before the bridge opens so the first frame is not lost.
        self._odo_listeners: List[Callable[[int, int, int], None]] = []

        def on_odo(total_left: int, total_right: int, fw_ms: int) -> None:
            for cb in list(self._odo_listeners):
                try:
                    cb(total_left, total_right, fw_ms)
                except Exception:                           # noqa: BLE001
                    logger.exception("odometry listener failed")

        self._bridge = ArduinoBridge(
            port=port,
            baud=baud,
            on_encoder=on_enc,
            on_error=on_err,
            on_odometry=on_odo,
        )

    def add_odometry_listener(self, cb: Callable[[int, int, int], None]) -> None:
        """Call `cb(total_left, total_right, firmware_ms)` on every telemetry
        frame (10 Hz). Needs firmware v5 (`drv8871-v5-odo`); older firmware
        never calls it, and robot_core.odometry reports that as stale."""
        self._odo_listeners.append(cb)

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        self._bridge.close()

    @property
    def is_connected(self) -> bool:
        return self._bridge.is_connected

    # ------------------------------------------------------------------ #
    # High-level movement API  (encoder-counted, blocking, in metres)
    # ------------------------------------------------------------------ #

    def straight_m(self, meters: float, speed: Optional[float] = None) -> None:
        """
        Drive forward exactly `meters` metres.

        Uses encoder ticks — the Arduino stops when the tick target is
        reached regardless of battery voltage or terrain friction.
        Logs a warning if either wheel stalls. (Left/right skew is no longer judged
        here: driven straight, the gyro holds the heading; see drivetrain_node._held.)

        Args:
            meters: Distance in metres (must be > 0).
            speed:  Speed as 0–100 % of full PWM.
                    None -> settings.DEFAULT_SPEED_PERCENT.
        """
        if meters <= 0:
            raise ValueError(f"straight_m: meters must be positive, got {meters}")
        ticks = max(1, int(meters * _ticks_per_metre()))
        self._move_and_verify(
            "F", duty_from_percent(_speed(speed)), ticks,
            label=f"straight_m({meters:.3f}m)",
        )

    def reverse_m(self, meters: float, speed: Optional[float] = None) -> None:
        """
        Drive backward exactly `meters` metres (encoder-counted).

        Args:
            meters: Distance in metres (must be > 0).
            speed:  Speed as 0–100 %. None -> settings.DEFAULT_SPEED_PERCENT.
        """
        if meters <= 0:
            raise ValueError(f"reverse_m: meters must be positive, got {meters}")
        ticks = max(1, int(meters * _ticks_per_metre()))
        self._move_and_verify(
            "B", duty_from_percent(_speed(speed)), ticks,
            label=f"reverse_m({meters:.3f}m)",
        )

    def right(self, angle: Optional[float] = None, speed: Optional[float] = None) -> None:
        """Tank-turn right. None -> settings.DEFAULT_TURN_DEGREES."""
        deg = settings.DEFAULT_TURN_DEGREES if angle is None else abs(float(angle))
        if deg <= 0:
            raise ValueError(f"right: angle must be positive, got {angle}")
        ticks = _turn_ticks(deg)
        self._move_and_verify(
            "R", duty_from_percent(_speed(speed)), ticks,
            label=f"right({deg:.1f}°)",
        )

    def left(self, angle: Optional[float] = None, speed: Optional[float] = None) -> None:
        """Tank-turn left. None -> settings.DEFAULT_TURN_DEGREES."""
        deg = settings.DEFAULT_TURN_DEGREES if angle is None else abs(float(angle))
        if deg <= 0:
            raise ValueError(f"left: angle must be positive, got {angle}")
        ticks = _turn_ticks(deg)
        self._move_and_verify(
            "L", duty_from_percent(_speed(speed)), ticks,
            label=f"left({deg:.1f}°)",
        )

    # ------------------------------------------------------------------ #
    # Names the motion layer calls, with the configured defaults applied.
    #
    # These four absorb the whole of the old movement_adapter.py: a default
    # when the caller passes None, and an abs() so a negative never reaches
    # the "must be positive" guard above. `straight` / `reverse` / `left` /
    # `right` is also exactly the vocabulary MotionExecutor.OPS maps onto, so
    # the executor can hold a SerialDrivetrain directly.
    # ------------------------------------------------------------------ #

    def straight(self, meters: Optional[float] = None) -> None:
        """Drive forward. None -> settings.DEFAULT_MOVE_METERS."""
        m = settings.DEFAULT_MOVE_METERS if meters is None else abs(float(meters))
        self.straight_m(m)

    def reverse(self, meters: Optional[float] = None) -> None:
        """Drive backward. None -> settings.DEFAULT_MOVE_METERS."""
        m = settings.DEFAULT_MOVE_METERS if meters is None else abs(float(meters))
        self.reverse_m(m)

    def stop(self) -> None:
        """Graceful stop. BLOCKS behind any in-flight move() — see
        emergency_stop() for the path that can interrupt one."""
        self._bridge.stop()

    def emergency_stop(self) -> None:
        """Halt a move already in progress. Safe from any thread."""
        self._bridge.emergency_stop()

    # ------------------------------------------------------------------ #
    # Open-loop intent methods (fire-and-forget / brain_loop streaming use)
    # ------------------------------------------------------------------ #

    def set_wheels(self, left_pwm: int, right_pwm: int) -> None:
        """Stream signed PWM per physical wheel (firmware v6 `-vel`; see ArduinoBridge.wheels)."""
        self._bridge.wheels(left_pwm, right_pwm)

    @property
    def supports_velocity(self) -> bool:
        return self._bridge.supports_velocity

    def intent_forward(self)  -> None: self._bridge.forward()
    def intent_backward(self) -> None: self._bridge.backward()
    def intent_left(self)     -> None: self._bridge.left()
    def intent_right(self)    -> None: self._bridge.right()

    def set_base_speed_percent(self, pct: float) -> None:
        """Set the V: speed register on the Arduino (for open-loop intent commands)."""
        self._base_speed_pct = max(0.0, min(100.0, pct))
        self._bridge.set_speed_pwm(duty_from_percent(self._base_speed_pct))

    # ------------------------------------------------------------------ #
    # Encoder status
    # ------------------------------------------------------------------ #

    def tfluna(self):
        """(range cm or None, time.monotonic() it arrived; 0.0 = never) from firmware v7+."""
        return self._bridge.tfluna_cm, self._bridge.tfluna_t

    def get_encoder_status(self) -> Dict[str, Any]:
        """Return the latest encoder counts and left-right sync error."""
        with self._enc_lock:
            c1 = self._motor1_count
            c2 = self._motor2_count
        return {
            "motor1_count": c1,
            "motor2_count": c2,
            "sync_error":   c1 - c2,
        }

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _move_and_verify(
        self,
        direction: str,
        speed_pwm: int,
        ticks: int,
        label: str,
    ) -> None:
        """
        Execute an encoder-counted move and verify the result.

        Logs warnings (never raises) for:
          - A wheel that didn't move at all (wiring / encoder fault)
          - Actual travel significantly below the tick target (stall/slip)
        """
        logger.debug("%s → M:%s,%d,%d", label, direction, speed_pwm, ticks)

        with self._enc_lock:
            self._last_move_counts = None    # never report the PREVIOUS move's
        counts = self._bridge.move(direction, speed_pwm, ticks)

        # The firmware zeroes BOTH counters in beginDrive() at the start of
        # every encoder-counted move, so the counts below already are this
        # move's travel.  Subtracting a "before" snapshot would straddle that
        # reset and produce nonsense (which is why the old reverse rows
        # reported drift figures that had nothing to do with the robot).
        #
        # The D frame carries those counts itself, so nothing has to be waited
        # for.  This used to sleep ENCODER_SETTLE_S (one ENC: telemetry
        # interval) to get a post-stop sample instead — 150 ms that sat between
        # this move's D and the next move's M, on every step of every gesture,
        # to recover numbers the D frame had already delivered.
        #
        # Getting the host off that path is what lets the next M reach
        # armDrive() while the wheels are still ramping down, so the firmware's
        # brake dead-time starts counting immediately at brake engage rather
        # than after us.  The dead-time and the ramp are unchanged: this
        # removes host latency, it does not remove any protection.
        if counts is None:
            # Firmware too old to put counts on the D frame.  Fall back to the
            # telemetry path, sleep and all.
            time.sleep(settings.ENCODER_SETTLE_S)
            status = self.get_encoder_status()
        else:
            left, right = counts
            with self._enc_lock:
                self._motor1_count = left
                self._motor2_count = right
                self._last_move_counts = (left, right)
            status = {
                "motor1_count": left,
                "motor2_count": right,
                "sync_error":   left - right,
            }

        self._verify_encoder_delta(status, ticks, label, direction)

    def move_counts(self) -> Dict[str, int]:
        """Signed encoder counts for the most recent encoder-counted move.

        Valid straight after a straight_m/reverse_m/left/right call, because
        the firmware resets both counters when a move begins.

        Prefers the counts latched off that move's D frame over the live
        telemetry counters.  Since the settle sleep was removed, the live
        counters are whatever the last E frame happened to carry — which,
        called promptly, is a mid-ramp-down sample of the same move and
        drifts upward for ~200 ms after the move returns.  The D snapshot is
        one fixed number for one move, which is what a caller asking for "the
        counts for that move" means.
        """
        with self._enc_lock:
            latched = self._last_move_counts
        if latched is not None:
            return {"left": latched[0], "right": latched[1]}
        status = self.get_encoder_status()
        return {"left": status["motor1_count"], "right": status["motor2_count"]}

    def _verify_encoder_delta(
        self,
        counts: Dict[str, Any],
        expected_ticks: int,
        label: str,
        direction: str = "",
    ) -> None:
        """
        Inspect how much each encoder moved and warn about anomalies.

        This is a diagnostic/safety layer — it does not retry or compensate.
        It surfaces problems early so the operator can tune calibration
        constants or inspect hardware before a mission.

        Every warning here names the specific next action. A diagnostic that
        says only "something is off" costs the operator the same debugging
        session it was written to save.
        """
        dL = abs(counts["motor1_count"])
        dR = abs(counts["motor2_count"])
        is_turn = direction.upper() in _TURN_DIRS

        # ── Stall detection ──────────────────────────────────────────────
        if dL == 0 and dR == 0:
            logger.error(
                "%s: NEITHER encoder moved — "
                "check motor power, encoder wiring, and serial connection",
                label,
            )
            return

        if dL == 0:
            logger.warning(
                "%s: LEFT encoder did not move (ΔL=0, ΔR=%d) — "
                "check left encoder wiring or motor driver channel",
                label, dR,
            )
        if dR == 0:
            logger.warning(
                "%s: RIGHT encoder did not move (ΔL=%d, ΔR=0) — "
                "check right encoder wiring or motor driver channel",
                label, dL,
            )

        # ── Target coverage ──────────────────────────────────────────────
        avg_travel = (dL + dR) / 2.0
        if expected_ticks > 0:
            coverage = avg_travel / expected_ticks
            if coverage < 0.80:
                # Name the constant that governs THIS move. The firmware ends a
                # move when BOTH wheels reach the target, so short coverage is
                # a stall or a lost channel — never an over-counting encoder,
                # which cannot end a move early.
                const = ("TICKS_PER_DEGREE (ROBOT_TICKS_PER_DEGREE)" if is_turn
                         else "TICKS_PER_CM (ROBOT_TICKS_PER_CM)")
                logger.warning(
                    "%s: only %.0f%% of target ticks reached "
                    "(target=%d ticks, actual avg=%.0f) — "
                    "wheel stalled, slipped, or lost encoder edges. "
                    "Note %s scales the TARGET, so it cannot cause short "
                    "coverage; check traction and encoder wiring first, and "
                    "recalibrate it only against MEASURED travel.",
                    label, coverage * 100, expected_ticks, avg_travel, const,
                )
            else:
                logger.debug(
                    "%s: coverage %.0f%% (target=%d ticks, actual avg=%.0f)",
                    label, coverage * 100, expected_ticks, avg_travel,
                )
