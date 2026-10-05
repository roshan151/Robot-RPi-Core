"""
Log integrity — the robot has no other way to tell you what went wrong.

Run:  python tests/test_robot_log.py

The crash cases run in subprocesses, because the whole point is what survives
the death of the interpreter. A test that only exercised the happy path would
miss every failure mode this module exists for.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from robot_core import robot_log  # noqa: E402


def _fresh(tmp: Path) -> Path:
    """Reset module state so setup() reconfigures onto a new file."""
    robot_log._state.update({"handler": None, "path": None, "on_fatal": None})
    robot_log._dedupe.clear()
    logging.getLogger().handlers.clear()
    return robot_log.setup(str(tmp), console_level=logging.CRITICAL)


def _lines(p: Path) -> list:
    if not p.exists():
        return []
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


# --------------------------------------------------------------------------- #

def test_each_line_is_standalone_json() -> None:
    """JSON Lines, not a JSON array: a truncated tail must cost one record."""
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        for i in range(5):
            robot_log.event("move.done", job=i, op="left", val=30, dur=0.4)
        raw = p.read_text().splitlines()
        assert len(raw) == 5
        for line in raw:
            json.loads(line)                      # each parses alone

        truncated = "\n".join(raw[:-1] + [raw[-1][: len(raw[-1]) // 2]])
        p.write_text(truncated)
        assert len(robot_log.read_events(str(p))) == 4, \
            "a torn final line should cost exactly one record"


def test_schema_has_ts_lvl_evt() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        robot_log.event("estop", logging.WARNING, reason="stop command")
        rec = _lines(p)[0]
        assert set(rec) >= {"ts", "lvl", "evt"}
        assert rec["evt"] == "estop"
        assert rec["lvl"] == "warn"
        assert rec["reason"] == "stop command"
        assert rec["ts"].endswith("Z") and "T" in rec["ts"]


def test_routine_info_stays_off_disk() -> None:
    """'Clean rather than verbose': library INFO chatter is console-only."""
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        noisy = logging.getLogger("some.library")
        for _ in range(50):
            noisy.info("chatter nobody needs")
        noisy.debug("more chatter")
        assert _lines(p) == [], "INFO chatter reached the log file"

        noisy.warning("this actually matters")
        recs = _lines(p)
        assert len(recs) == 1 and recs[0]["src"] == "some.library"


def test_unregistered_event_is_kept_but_flagged() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        robot_log.event("totally.made.up", x=1)
        rec = _lines(p)[0]
        assert rec["evt"] == "totally.made.up"
        assert rec["_unregistered"] is True, "typo'd events must not vanish"


def test_throttle_collapses_repeats_and_counts_them() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        for _ in range(20):
            robot_log.event_throttled("estop", key="guardian", window_s=0.25,
                                      reason="obstacle")
        assert len(_lines(p)) == 1
        time.sleep(0.3)
        robot_log.event_throttled("estop", key="guardian", window_s=0.25,
                                  reason="obstacle")
        recs = _lines(p)
        assert len(recs) == 2
        assert recs[1]["repeats"] == 19, "suppressed events were lost silently"


def test_exception_records_compact_traceback() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = _fresh(Path(d) / "logs.json")
        try:
            raise RuntimeError("serial exploded")
        except RuntimeError:
            logging.getLogger("drivetrain").error("move failed", exc_info=True)
        rec = _lines(p)[0]
        assert rec["err"] == "RuntimeError: serial exploded"
        assert isinstance(rec["tb"], list) and len(rec["tb"]) <= 6
        assert " in " in rec["tb"][0]


# --------------------------------------------------------------------------- #
# Death — run as subprocesses
# --------------------------------------------------------------------------- #

_HARNESS = """
import sys, time, threading, logging
sys.path.insert(0, {repo!r})
from robot_core import robot_log
robot_log.setup({log!r}, console_level=logging.CRITICAL)
braked = []
robot_log.install_crash_handlers(on_fatal=lambda cause: robot_log.event(
    "estop", logging.CRITICAL, reason="brake: " + cause))
robot_log.event("session.start", mode="test")
{body}
"""


def _run(tmpdir: str, body: str, sig: int | None = None) -> list:
    log = str(Path(tmpdir) / "logs.json")
    script = Path(tmpdir) / "harness.py"
    script.write_text(_HARNESS.format(repo=str(REPO), log=log, body=body))
    proc = subprocess.Popen([sys.executable, str(script)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if sig is not None:
        time.sleep(0.6)
        os.kill(proc.pid, sig)
    proc.wait(timeout=15)
    return _lines(Path(log))


def test_uncaught_exception_logs_cause_and_brakes() -> None:
    with tempfile.TemporaryDirectory() as d:
        recs = _run(d, "raise ValueError('drivetrain vanished')")
        evts = [r["evt"] for r in recs]
        assert "fatal" in evts, f"process died without logging why: {evts}"
        fatal = next(r for r in recs if r["evt"] == "fatal")
        assert fatal["err"] == "ValueError: drivetrain vanished"
        assert fatal["thread"] == "main"
        assert evts.index("estop") < evts.index("fatal"), \
            "motors must be braked before the cause is written"


def test_thread_exception_is_not_silent() -> None:
    """Worker threads bypass sys.excepthook entirely. Without
    threading.excepthook the motion executor could die in total silence."""
    with tempfile.TemporaryDirectory() as d:
        recs = _run(d, """
def boom():
    raise RuntimeError("worker died")
t = threading.Thread(target=boom, name="motion-executor")
t.start(); t.join()
time.sleep(0.3)
""")
        fatal = [r for r in recs if r["evt"] == "fatal"]
        assert fatal, "a dead worker thread left no trace"
        assert fatal[0]["thread"] == "motion-executor"
        assert fatal[0]["err"] == "RuntimeError: worker died"


def test_sigterm_logs_before_exit() -> None:
    """systemd stops the service with SIGTERM."""
    with tempfile.TemporaryDirectory() as d:
        recs = _run(d, "time.sleep(30)", sig=signal.SIGTERM)
        evts = [r["evt"] for r in recs]
        assert "signal" in evts, f"SIGTERM left no record: {evts}"
        sigrec = next(r for r in recs if r["evt"] == "signal")
        assert sigrec["sig"] == "SIGTERM"
        assert "estop" in evts, "motors were not braked on SIGTERM"


def test_sigkill_keeps_everything_already_written() -> None:
    """SIGKILL cannot be handled — but nothing logged before it may be lost,
    which is why every record is flushed as it is written."""
    with tempfile.TemporaryDirectory() as d:
        recs = _run(d, """
for i in range(10):
    robot_log.event("move.done", job=i, op="left", val=30, dur=0.4)
time.sleep(30)
""", sig=signal.SIGKILL)
        moves = [r for r in recs if r["evt"] == "move.done"]
        assert len(moves) == 10, \
            f"buffered records lost on SIGKILL: {len(moves)}/10"


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
