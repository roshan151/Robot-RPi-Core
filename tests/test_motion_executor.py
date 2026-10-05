"""
MotionExecutor / Gesturer semantics — no hardware required.

Run:  python -m pytest tests/test_motion_executor.py -v
      python tests/test_motion_executor.py          (plain runner, no pytest)

FakeMovement emulates the one property that makes the real bridge awkward: a
move is a blocking call that only ends early if someone brakes it out of band.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_core import settings
from robot_core.gestures import (
    DRIVE,
    GESTURE_NO_DEGREES,
    GESTURE_YES_METERS,
    NO,
    TURN,
    YES,
    Gesturer,
)
from robot_core.motion import LocalMotion
from robot_core.motion_executor import CANCELLED, DONE, FAILED, MotionExecutor

MOVE_SECONDS = 0.40


class FakeMovement:
    """Stand-in for SerialDrivetrain with realistic blocking + interrupt."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, float]] = []
        self.estops = 0
        self._brake = threading.Event()
        self._lock = threading.Lock()

    def _blocking_move(self, name: str, value: float) -> None:
        with self._lock:
            self.calls.append((name, value))
        self._brake.clear()
        # Interruptible sleep: the real move() returns early (by raising) when
        # the firmware reports a STOP-terminated move.
        if self._brake.wait(timeout=MOVE_SECONDS):
            raise RuntimeError("Arduino returned ERR — move stopped [STOP]")

    def straight(self, meters=None): self._blocking_move("straight", meters)
    def reverse(self, meters=None):  self._blocking_move("reverse", meters)
    def left(self, angle=None):      self._blocking_move("left", angle)
    def right(self, angle=None):     self._blocking_move("right", angle)
    def stop(self, _unused=None):    pass

    def emergency_stop(self) -> None:
        self.estops += 1
        self._brake.set()



def _drain_events(ex: MotionExecutor) -> list:
    out = []
    while not ex.events.empty():
        out.append(ex.events.get_nowait())
    return out


# --------------------------------------------------------------------------- #

def test_submit_is_non_blocking() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        t0 = time.monotonic()
        ex.submit("straight", 3.0)
        assert time.monotonic() - t0 < 0.05, "submit() blocked"
        ex.cancel_all("teardown")


def test_cancel_interrupts_move_in_flight() -> None:
    """The whole point: a move already running must be stoppable."""
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        ex.submit("straight", 3.0)
        time.sleep(0.10)                       # let the worker start it
        t0 = time.monotonic()
        ex.cancel_all("stop command")
        assert ex.drain(timeout=2.0)
        halt = time.monotonic() - t0

        assert move.estops >= 1, "emergency_stop was never called"
        assert halt < 0.30, f"halt took {halt:.3f}s"
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [CANCELLED], ev


def test_cancel_drops_queued_jobs() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        for _ in range(4):
            ex.submit("straight", 1.0)
        time.sleep(0.05)
        ex.cancel_all("stop")
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert len(ev) == 4
        assert all(e.status == CANCELLED for e in ev), ev


def test_normal_move_completes() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        ex.submit("left", 30)
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [DONE], ev
        assert move.calls == [("left", 30)]


def test_gesture_reaches_the_drivetrain() -> None:
    move = FakeMovement()
    motion = LocalMotion(move)
    try:
        Gesturer(motion).play("yes")
        assert motion.executor.drain(timeout=3.0)
        assert [c[0] for c in move.calls] == ["straight", "reverse"], move.calls
    finally:
        motion.close()


def test_gesture_shapes() -> None:
    # These constants live in gestures.py, not settings.py — gestures.py is the
    # documented single source of truth for the vocabulary, and robot_tools
    # builds its function declaration from it.  This asserted against
    # settings.GESTURE_YES_METERS, which has never existed, so the test raised
    # AttributeError before reaching a single one of its real assertions.
    assert YES == [(DRIVE, +GESTURE_YES_METERS),
                   (DRIVE, -GESTURE_YES_METERS)]
    assert NO == [(TURN, -GESTURE_NO_DEGREES),
                  (TURN, +GESTURE_NO_DEGREES * 2),
                  (TURN, -GESTURE_NO_DEGREES)]
    # Steps are signed now, so "net zero" is literally the sum.
    assert sum(v for _op, v in NO) == 0, "NO gesture is not net-zero"


def test_gesture_dropped_not_queued() -> None:
    move = FakeMovement()
    motion = LocalMotion(move)
    try:
        g = Gesturer(motion)
        accepted = [g.play("yes") for _ in range(5)]
        assert sum(a is not None for a in accepted) == 1, \
            "rapid gestures were queued instead of dropped"
        assert motion.executor.drain(timeout=3.0)
    finally:
        motion.close()


def test_gesture_yields_to_real_motion() -> None:
    move = FakeMovement()
    motion = LocalMotion(move)
    try:
        motion.drive(3.0)
        time.sleep(0.10)
        assert Gesturer(motion).play("no") is None, \
            "gesture ran while the robot was driving"
        motion.stop()
        assert motion.executor.drain(timeout=2.0)
    finally:
        motion.close()


def test_failure_is_not_reported_as_cancelled() -> None:
    move = FakeMovement()

    def boom(_v=None):
        raise RuntimeError("serial exploded")

    move.right = boom                                    # type: ignore[assignment]
    with MotionExecutor(move) as ex:
        ex.submit("right", 90)
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [FAILED], ev
        assert "serial exploded" in ev[0].detail


def test_status_reports_motion() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        assert ex.status()["moving"] is False
        ex.submit("straight", 3.0)
        time.sleep(0.10)
        st = ex.status()
        assert st["moving"] is True
        assert st["gesturing"] is False
        assert st["current"]["op"] == "straight"
        ex.cancel_all("teardown")


def test_duty_from_percent_maps_and_clamps() -> None:
    from robot_core.drivetrain.client import duty_from_percent

    assert duty_from_percent(0) == 0
    assert duty_from_percent(100) == 255
    # 70 % of 255 = 178.5; Python's round() breaks the tie to even -> 178.
    assert duty_from_percent(70.0) == 178
    assert duty_from_percent(80.0) == 204
    # Out-of-range percentages clamp instead of producing an invalid duty.
    assert duty_from_percent(-10) == 0
    assert duty_from_percent(150) == 255


def test_default_speed_is_in_the_characterised_band() -> None:
    """The default speed is a calibration input, not a free parameter.

    TICKS_PER_DEGREE and TURN_COAST_TICKS are both measured at whatever this
    is set to — slip and braking momentum move with it — so changing it
    invalidates them.  See the note beside TURN_COAST_TICKS in settings.py.

    This deliberately does NOT pin an exact value.  The old version asserted
    == 70.0 and duly failed the moment the default became 80, which says
    nothing about correctness; the band is what actually matters, because
    outside it the calibration constants no longer describe the robot.
    """
    from robot_core.drivetrain.client import duty_from_percent

    assert 50.0 <= settings.DEFAULT_SPEED_PERCENT <= 90.0, (
        f"DEFAULT_SPEED_PERCENT={settings.DEFAULT_SPEED_PERCENT} is outside the "
        "range the drivetrain was calibrated over — recalibrate "
        "TICKS_PER_DEGREE and TURN_COAST_TICKS before widening this."
    )
    assert 0 < duty_from_percent(settings.DEFAULT_SPEED_PERCENT) <= 255


def test_estop_seq_band_is_disjoint() -> None:
    """Firmware dedupes on the previous seq, so the e-stop must never reuse
    the in-flight move's sequence number."""
    assert settings.NORMAL_SEQ_MAX < settings.ESTOP_SEQ_MIN <= settings.ESTOP_SEQ_MAX <= 255


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
