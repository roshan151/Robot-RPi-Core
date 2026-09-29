"""The Part 4.3 backpressure table, one row per test."""

from __future__ import annotations

import threading

import pytest

from robot_core.perception.client import Busy, DetectResult, ServiceError, Unavailable
from robot_core.perception.gateway import DetectGateway


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def result(seq: int) -> DetectResult:
    return DetectResult(seq, [], (640, 360), "yolov8n-mps", 18.0)


def make(fn, clock=None, **kw):
    applied = []
    gw = DetectGateway(fn, lambda r, t: applied.append(r.seq), clock=clock or Clock(), **kw)
    return gw, applied


def test_ok_result_is_applied():
    gw, applied = make(lambda jpeg, seq: result(seq))
    assert gw.run_once(b"x", 1, 100.0) is not None
    assert applied == [1]
    assert gw.counters.ok == 1 and gw.model_version == "yolov8n-mps"


def test_frame_arriving_while_in_flight_is_dropped_not_queued():
    release = threading.Event()
    entered = threading.Event()

    def slow(jpeg, seq):
        entered.set()
        release.wait(2)
        return result(seq)

    gw, applied = make(slow)
    assert gw.submit(b"x", 1, 100.0)
    entered.wait(2)
    assert not gw.submit(b"x", 2, 100.0)          # dropped
    assert gw.counters.dropped_busy == 1
    release.set()
    for _ in range(100):
        if applied:
            break
        threading.Event().wait(0.01)
    assert applied == [1]


def test_server_503_is_a_dropped_frame_not_a_failure():
    def busy(jpeg, seq):
        raise Busy("busy", 503)

    gw, _ = make(busy)
    for i in range(10):
        gw.run_once(b"x", i, 100.0)
    assert gw.counters.server_busy == 10
    assert not gw.degraded


def test_consecutive_failures_degrade_and_one_success_recovers():
    fail = {"on": True}

    def flaky(jpeg, seq):
        if fail["on"]:
            raise Unavailable("down")
        return result(seq)

    gw, _ = make(flaky, degraded_after=5)
    for i in range(4):
        gw.run_once(b"x", i, 100.0)
    assert not gw.degraded
    gw.run_once(b"x", 4, 100.0)
    assert gw.degraded
    fail["on"] = False
    gw.run_once(b"x", 5, 100.0)
    assert not gw.degraded


def test_stale_result_is_discarded():
    clock = Clock()

    def late(jpeg, seq):
        clock.t += 1.5
        return result(seq)

    gw, applied = make(late, clock=clock, max_result_age_s=1.0)
    gw.run_once(b"x", 1, clock.t)
    assert applied == [] and gw.counters.stale == 1


def test_out_of_order_result_is_discarded():
    gw, applied = make(lambda jpeg, seq: result(seq))
    gw.run_once(b"x", 5, 100.0)
    gw.run_once(b"x", 3, 100.0)
    assert applied == [5] and gw.counters.out_of_order == 1


def test_wrong_seq_echo_is_discarded():
    gw, applied = make(lambda jpeg, seq: result(seq + 1))
    gw.run_once(b"x", 1, 100.0)
    assert applied == [] and gw.counters.errors == 1


@pytest.mark.parametrize("exc", [ServiceError("bad", 422), RuntimeError("bug")])
def test_errors_never_escape(exc):
    def boom(jpeg, seq):
        raise exc

    gw, _ = make(boom)
    assert gw.run_once(b"x", 1, 100.0) is None
    assert gw.counters.errors == 1 and not gw.busy
