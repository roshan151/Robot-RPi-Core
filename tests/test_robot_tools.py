"""
Tool dispatch must never block the event loop.

This is the regression guard for the failure that killed the Live session after
one command: tool calls are dispatched from inside
`async for response in session.receive()`, so awaiting a 1.4 s move there stops
the websocket being read, back-pressure stalls the microphone uplink, audio is
dropped (`audio.error stage=mic-queue`) and the server closes the connection.

Run:  python tests/test_robot_tools.py
"""

from __future__ import annotations

import asyncio
import sys
import time
import types
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))


def _stub_genai() -> None:
    """robot_tools imports google.genai only inside declarations()."""
    if "google.genai" in sys.modules:
        return

    class _S:
        def __init__(self, **kw): self.__dict__.update(kw)

    class FunctionDeclaration:
        def __init__(self, name, description, parameters=None):
            self.name, self.description, self.parameters = name, description, parameters

    t = types.ModuleType("google.genai.types")
    t.Schema, t.FunctionDeclaration = _S, FunctionDeclaration
    t.FunctionResponse = t.Blob = _S
    g = types.ModuleType("google.genai")
    g.types, g.Client = t, lambda **kw: types.SimpleNamespace()
    sys.modules.setdefault("google", types.ModuleType("google")).genai = g
    sys.modules["google.genai"], sys.modules["google.genai.types"] = g, t


_stub_genai()

from robot_core import settings  # noqa: E402
from robot_core.live.tools import RobotTools, declarations  # noqa: E402
from robot_core.motion import LocalMotion  # noqa: E402


class FakeMove:
    """SerialDrivetrain stand-in: blocking moves, released only by the brake."""

    def __init__(self, seconds: float = 0.2) -> None:
        self.calls: list = []
        self.estops = 0
        self._secs = seconds
        self._until = 0.0

    def _blocking(self, name, value):
        self.calls.append((name, float(value)))
        self._until = time.monotonic() + self._secs
        while time.monotonic() < self._until:
            time.sleep(0.005)

    def straight(self, m=None): self._blocking("straight", m)
    def reverse(self, m=None):  self._blocking("reverse", m)
    def left(self, a=None):     self._blocking("left", a)
    def right(self, a=None):    self._blocking("right", a)
    def stop(self, _=None):     pass

    def emergency_stop(self):
        self.estops += 1
        self._until = 0.0                 # releases the in-flight move


def run(coro):
    return asyncio.run(coro)


def tools(seconds=0.2):
    """(drivetrain, backend, tools) — the backend owns the executor and is what
    the caller closes, mirroring how the ROS voice node holds its clients."""
    move = FakeMove(seconds)
    motion = LocalMotion(move)
    return move, motion, RobotTools(motion)


# --------------------------------------------------------------------------- #
# The property that matters
# --------------------------------------------------------------------------- #

def test_every_tool_returns_immediately() -> None:
    move, motion, t = tools(seconds=3.0)
    try:
        for name, args in [("drive", {"meters": 2.0}),
                           ("turn", {"degrees": 90}),
                           ("answer", {"value": "yes"}),
                           ("stop", {})]:
            t0 = time.monotonic()
            run(t.dispatch(name, args))
            took = time.monotonic() - t0
            assert took < 0.10, f"{name} blocked the loop for {took:.2f}s"
    finally:
        motion.executor.cancel_all("teardown")
        motion.close()


def test_a_running_move_does_not_stall_the_loop() -> None:
    move, motion, t = tools(seconds=0.6)

    async def scenario():
        await t.dispatch("drive", {"meters": 1.0})
        ticks = 0
        while motion.executor.status()["moving"] or motion.executor.status()["queue_depth"]:
            await asyncio.sleep(0.02)
            ticks += 1
            if ticks > 200:
                break
        return ticks

    try:
        assert run(scenario()) > 10, "loop barely ran during a 0.6 s move"
    finally:
        motion.close()


# --------------------------------------------------------------------------- #
# FIFO
# --------------------------------------------------------------------------- #

def test_commands_execute_in_order() -> None:
    move, motion, t = tools(seconds=0.15)
    try:
        run(t.dispatch("drive", {"meters": 1.0}))
        run(t.dispatch("turn", {"degrees": 90}))
        run(t.dispatch("drive", {"meters": -1.0}))
        assert motion.executor.drain(timeout=5.0)
        assert move.calls == [("straight", 1.0), ("right", 90.0), ("reverse", 1.0)]
    finally:
        motion.close()


def test_queue_depth_is_reported_back_to_the_model() -> None:
    move, motion, t = tools(seconds=1.0)
    try:
        run(t.dispatch("drive", {"meters": 1.0}))
        r = run(t.dispatch("turn", {"degrees": 90}))
        assert r["ok"] and "queued" in r and r["queue_depth"] >= 0
    finally:
        motion.executor.cancel_all("teardown")
        motion.close()


# --------------------------------------------------------------------------- #
# Stop
# --------------------------------------------------------------------------- #

def test_stop_empties_the_queue_and_interrupts() -> None:
    move, motion, t = tools(seconds=5.0)
    try:
        run(t.dispatch("drive", {"meters": 3.0}))     # starts
        run(t.dispatch("turn", {"degrees": 90}))      # queued
        run(t.dispatch("drive", {"meters": 1.0}))     # queued
        time.sleep(0.15)

        t0 = time.monotonic()
        r = run(t.dispatch("stop", {}))
        halt = time.monotonic() - t0

        assert r["stopped"] is True
        assert halt < 0.10, f"stop queued behind the move: {halt:.2f}s"
        assert move.estops >= 1, "emergency_stop never fired"
        assert len(r["cancelled"]) >= 2, f"queue not emptied: {r}"

        assert motion.executor.drain(timeout=3.0)
        assert move.calls == [("straight", 3.0)], \
            f"a cancelled job still reached the drivetrain: {move.calls}"
    finally:
        motion.close()


# --------------------------------------------------------------------------- #
# Argument handling
# --------------------------------------------------------------------------- #

def test_signs_map_to_directions() -> None:
    move, motion, t = tools(seconds=0.05)
    try:
        run(t.dispatch("drive", {"meters": -0.5}))
        run(t.dispatch("turn", {"degrees": -45}))
        assert motion.executor.drain(timeout=3.0)
        assert move.calls == [("reverse", 0.5), ("left", 45.0)]
    finally:
        motion.close()


def test_drive_limit_is_enforced_in_code() -> None:
    move, motion, t = tools(seconds=0.05)
    try:
        r = run(t.dispatch("drive", {"meters": settings.MAX_DRIVE_METERS + 1}))
        assert r["ok"] is False and "limit" in r["error"]
        motion.executor.drain(timeout=0.5)
        assert move.calls == [], "an over-limit drive was still queued"
    finally:
        motion.close()


def test_bad_arguments_become_results_not_crashes() -> None:
    move, motion, t = tools(seconds=0.05)
    try:
        for name, args in [("drive", {}), ("turn", {"degrees": "sideways"}),
                           ("nonsense", {}), ("answer", {"value": "maybe"})]:
            assert run(t.dispatch(name, args))["ok"] is False, (name, args)
    finally:
        motion.close()


def test_gesture_yields_to_real_motion() -> None:
    move, motion, t = tools(seconds=0.5)
    try:
        run(t.dispatch("drive", {"meters": 1.0}))
        time.sleep(0.1)
        r = run(t.dispatch("answer", {"value": "yes"}))
        assert r["gestured"] is False, "nodded while driving"
    finally:
        motion.executor.cancel_all("teardown")
        motion.close()


# --------------------------------------------------------------------------- #
# Wiring
# --------------------------------------------------------------------------- #

def test_run_task_hands_the_job_to_the_agent_and_answers_by_gesture() -> None:
    """run_task only queues; the agent runs it with the session closed, then nods
    (yes) or shakes (no) according to whether the job worked."""
    from robot_core.live import agent as agent_mod
    for ok, expected in ((True, [("straight", 0.1), ("reverse", 0.1)]),
                         (False, [("left", 30.0), ("right", 60.0), ("left", 30.0)])):
        move = FakeMove(0.01)
        motion = LocalMotion(move)
        ran = []
        t = RobotTools(motion, lambda task, name: ran.append((task, name)) or ok)
        try:
            assert run(t.dispatch("run_task", {"task": "enroll_face", "name": "Sam Smith!"}))["ok"]
            assert t.pending == ("enroll_face", "Sam"), "name must be one clean word"
            assert run(t.dispatch("run_task", {"task": "explore"}))["ok"] is False   # one at a time
            run(agent_mod._run_task(t))
            assert ran == [("enroll_face", "Sam")] and t.pending is None
            assert move.calls == expected, move.calls
        finally:
            motion.close()


def test_run_task_refuses_what_it_cannot_do() -> None:
    move, motion, t = tools()
    try:
        assert run(t.dispatch("run_task", {"task": "explore"}))["ok"] is False   # no runner wired
        t.task_runner = lambda task, name: True
        r = run(t.dispatch("run_task", {"task": "enroll_face"}))                     # no name said
        assert r["ok"] is False and r["gestured"] is True and t.pending is None
        time.sleep(0.5)
        assert [c[0] for c in move.calls] == ["left", "right", "left"], "should have shaken its head"
        assert run(t.dispatch("run_task", {"task": "fly"}))["ok"] is False
        assert t.pending is None
    finally:
        motion.close()


def test_head_look_tilts_from_where_it_is_and_clamps() -> None:
    from robot_core.head_look import HeadLook
    from robot_core.sensors.gimbal import Gimbal
    g = Gimbal(backend="fake")
    head = HeadLook(lambda: g)
    try:
        t = RobotTools(LocalMotion(FakeMove()), head=head)
        r = lambda n, **a: run(t.dispatch(n, a))
        assert r("look_up")["tilt"] == 30.0                      # default 30
        res = r("look_up", degrees=30)
        assert res["tilt"] == 45.0 and res["clamped"]            # +45 limit
        assert r("look_down", degrees=40)["tilt"] == 5.0         # relative to where it is
        assert r("look_down", degrees=100)["tilt"] == -45.0      # -45 limit
        head.release()
        assert r("look_up", degrees=10)["tilt"] == 10.0          # parked: starts from level
        assert r("look_left")["pan"] == 30.0                     # + pan is left, default 30
        res = r("look_left", degrees=100)                        # one call is capped at 90, then the limit
        assert res["pan"] == 81.0 and res["clamped"]             # the default 1500 us centre reaches +81 (500-2400 us window)
        assert r("look_right", degrees=81)["pan"] == 0.0         # relative to where it is
        assert r("look_right", degrees=90)["pan"] == -90.0
        assert r("look_up", degrees=10)["tilt"] == 20.0          # tilt is independent of pan
        assert not RobotTools(LocalMotion(FakeMove()))._look(5)["ok"]
    finally:
        head.release()


def test_ten_tools_and_stop_takes_no_arguments() -> None:
    decls = {d.name: d for d in declarations()}
    assert set(decls) == {"drive", "turn", "look_up", "look_down", "look_left", "look_right", "shutdown", "stop", "answer", "run_task"}
    assert not getattr(decls["stop"].parameters, "required", []), \
        "a stop that can be malformed is a stop that can fail"


def test_no_tool_awaits_the_drivetrain() -> None:
    """Guards the fix at the source level: nothing may re-introduce a blocking
    await into the dispatch path."""
    src = (REPO / "robot_core" / "live" / "tools.py").read_text()
    assert "to_thread" not in src, "a tool is blocking on the drivetrain again"
    assert "MotionBackend" in src, "tools bypassed the non-blocking motion backend"


def test_receive_loop_stall_is_measured() -> None:
    src = (REPO / "robot_core" / "live" / "agent.py").read_text()
    assert "recv-stall" in src, "no telemetry for a stalled receive loop"
    assert "send-slow" in src, "no telemetry for a slow uplink send"


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


def test_shutdown_powers_off_after_a_delay_and_stop_cancels_it(monkeypatch) -> None:
    from robot_core.live import tools as tools_mod
    monkeypatch.setattr(tools_mod, "SHUTDOWN_DELAY_S", 0.2)
    calls = []
    t = RobotTools(LocalMotion(FakeMove()), power_off=lambda: calls.append(1))
    res = run(t.dispatch("shutdown", {}))
    assert res["ok"] and not calls                       # not yet: there is time to say stop
    time.sleep(0.5)
    assert calls == [1]
    t2 = RobotTools(LocalMotion(FakeMove()), power_off=lambda: calls.append(2))
    run(t2.dispatch("shutdown", {}))
    run(t2.dispatch("stop", {}))
    time.sleep(0.5)
    assert calls == [1]                                  # cancelled


def test_shutdown_parks_the_head_and_stops_its_servo_pulses(monkeypatch) -> None:
    """Incident 2: the Pi must not halt while still driving a live pulse into a powered servo."""
    from robot_core.head_look import HeadLook
    from robot_core.live import tools as tools_mod
    from robot_core.sensors.gimbal import Gimbal
    monkeypatch.setattr(tools_mod, "SHUTDOWN_DELAY_S", 0.2)
    g = Gimbal(backend="fake")
    t = RobotTools(LocalMotion(FakeMove()), head=HeadLook(lambda: g), power_off=lambda: None)
    run(t.dispatch("look_up", {"degrees": 20}))
    assert g.tilt.out.enabled
    run(t.dispatch("shutdown", {}))
    time.sleep(0.4)
    assert g.angles() == (0.0, 0.0)                                  # parked first ...
    assert not g.pan.out.enabled and not g.tilt.out.enabled          # ... then the pulses stopped


def test_shutdown_also_stops_pwm_an_earlier_task_left_holding(monkeypatch) -> None:
    from robot_core.head_look import HeadLook
    from robot_core.live import tools as tools_mod
    from robot_core.sensors.gimbal import Gimbal
    monkeypatch.setattr(tools_mod, "SHUTDOWN_DELAY_S", 0.2)
    left_holding = Gimbal(backend="fake")                           # e.g. a face task closed it with hold_on_close
    assert left_holding.pan.out.enabled
    t = RobotTools(LocalMotion(FakeMove()), head=HeadLook(lambda: left_holding), power_off=lambda: None)
    run(t.dispatch("shutdown", {}))                                 # the head was never opened by the voice session
    time.sleep(0.4)
    assert not left_holding.pan.out.enabled and not left_holding.tilt.out.enabled


def test_head_look_refuses_while_servo_supply_is_off() -> None:
    from robot_core.head_look import HeadLook
    from robot_core.sensors.gimbal import Gimbal
    rail = {"on": False}
    g = Gimbal(backend="fake", power_sense=lambda: rail["on"])
    head = HeadLook(lambda: g)
    try:
        r = head.look(20)
        assert not r["ok"] and "power" in r["error"]
        assert g.angles() == (0.0, 0.0)
        rail["on"] = True
        assert head.look(20)["ok"]                                  # same call works once the switch is on
    finally:
        head.release()
