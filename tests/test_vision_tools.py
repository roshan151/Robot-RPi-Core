"""The camera tools: offered only with a VisionBackend, never blocking, and
`stop` also ends following."""

from __future__ import annotations

import asyncio
import time

import test_robot_tools as base          # noqa: F401  (stubs google.genai)
from robot_core.live.tools import RobotTools


class FakeMotion:
    def __init__(self) -> None:
        self.stops = 0

    def drive(self, meters, *, gesture=False):
        return {"ok": True}

    def turn(self, degrees, *, gesture=False):
        return {"ok": True}

    def stop(self):
        self.stops += 1
        return {"ok": True, "stopped": True}

    def busy(self):
        return False

    def close(self):
        pass


class FakeVision:
    def __init__(self) -> None:
        self.calls = []

    def _rec(self, *a):
        self.calls.append(a)
        time.sleep(0)            # a real backend returns at once too
        return {"ok": True, "pending": True}

    def look(self): return self._rec("look")
    def who(self): return self._rec("who")
    def look_at(self, what): return self._rec("look_at", what)
    def follow(self, name=None): return self._rec("follow", name)
    def stop_following(self): return self._rec("stop_following")
    def approach(self, what): return self._rec("approach", what)
    def clear_path(self): return self._rec("clear_path")
    def remember_face(self, name): return self._rec("remember", name)


def run(tools, tool, **args):
    return asyncio.run(tools.dispatch(tool, args))


def test_camera_tools_only_offered_with_a_vision_backend():
    names = lambda t: {d.name for d in t.declarations()}          # noqa: E731
    assert "follow_person" not in names(RobotTools(FakeMotion()))
    assert {"look", "who_is_there", "follow_person", "stop_following", "approach",
            "find_clear_path", "remember_face"} <= names(RobotTools(FakeMotion(), FakeVision()))


def test_without_vision_camera_tools_are_unknown():
    r = run(RobotTools(FakeMotion()), "look")
    assert r["ok"] is False and "unknown tool" in r["error"]


def test_dispatch_reaches_the_backend():
    v = FakeVision()
    t = RobotTools(FakeMotion(), v)
    run(t, "follow_person", name="Roshan")
    run(t, "follow_person", name="me")
    run(t, "approach", object=" Chair ")
    run(t, "remember_face", name="Sam")
    assert ("follow", "roshan") in v.calls
    assert ("follow", None) in v.calls           # "me" means whoever is in front
    assert ("approach", "chair") in v.calls
    assert ("remember", "sam") in v.calls


def test_stop_also_stops_following():
    m, v = FakeMotion(), FakeVision()
    run(RobotTools(m, v), "stop")
    assert m.stops == 1 and ("stop_following",) in v.calls


def test_bad_arguments_are_results_not_exceptions():
    t = RobotTools(FakeMotion(), FakeVision())
    assert run(t, "approach", object="")["ok"] is False
    assert run(t, "remember_face", name="")["ok"] is False
