"""
Async, cancellable motion execution.

Why this exists
---------------
`ArduinoBridge.move()` is blocking: it holds the bridge's `_cmd_lock` and waits
for the firmware's D frame, up to `settings.MOVE_TIMEOUT_S`.  Calling it directly
from a voice session means the robot is deaf for the whole move and cannot be
told to stop — the one command that always has to work.

MotionExecutor moves that blocking call onto a single worker thread:

  * `submit()` returns a job id immediately and never blocks.
  * `cancel_all()` clears the queue and fires an out-of-band emergency stop
    that interrupts the move already in flight.
  * Completion events are published on a queue so the caller (and, later, the
    language model) learns what actually happened.

It is also the only thread that issues encoder-counted moves, which removes a
latent serial race: previously the vision guardian could call `stop()` from its
own thread while the main thread sat inside `move()`.

Cancellation semantics
----------------------
`emergency_stop()` makes the firmware call `finishMove("STOP")`, which emits the
D frame the blocked `move()` is waiting on.  `move()` then raises RuntimeError.
That exception *is* the cancellation — the worker checks the cancel flag before
deciding whether an exception means "cancelled" or "failed".
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from robot_core import robot_log

if TYPE_CHECKING:                       # import only for the type hint, so this
    from robot_core.drivetrain import SerialDrivetrain   # module stays hardware-free
                                                         # and the tests can pass a fake.

logger = logging.getLogger(__name__)

# The ONLY op -> method map in the tree. This used to exist three times over
# (here, in gestures, and in a glossary module) and they drifted.
OPS = {
    "straight": "straight",
    "forward": "straight",
    "reverse": "reverse",
    "left": "left",
    "right": "right",
}

DONE = "done"
CANCELLED = "cancelled"
FAILED = "failed"


@dataclass
class MotionJob:
    job_id: int
    op: str
    value: float
    gesture: bool = False


@dataclass
class MotionEvent:
    job_id: int
    op: str
    value: float
    status: str                 # done | cancelled | failed
    detail: str = ""
    gesture: bool = False
    duration_s: float = 0.0

    def as_dict(self) -> Dict[str, Any]:
        return {
            "job_id": self.job_id,
            "op": self.op,
            "value": self.value,
            "status": self.status,
            "detail": self.detail,
            "gesture": self.gesture,
            "duration_s": round(self.duration_s, 3),
        }


class MotionExecutor:
    """Single-threaded owner of the drivetrain, with pre-emptive cancellation."""

    def __init__(self, move: "SerialDrivetrain") -> None:
        self._move = move

        self._q: "queue.Queue[Optional[MotionJob]]" = queue.Queue()
        self._events: "queue.Queue[MotionEvent]" = queue.Queue()

        # Per-job completion slots, alongside the broadcast queue above.
        # The queue is a stream every listener shares; these let ONE caller
        # block on the outcome of ONE job, which is what a ROS action server
        # needs when it maps a goal onto a submit().
        self._waiters: Dict[int, threading.Event] = {}
        self._results: Dict[int, MotionEvent] = {}

        self._lock = threading.Lock()
        self._next_id = 0
        self._current: Optional[MotionJob] = None
        self._cancel = threading.Event()
        self._idle = threading.Event()
        self._idle.set()

        self._stop_flag = threading.Event()
        self._worker = threading.Thread(
            target=self._run, name="motion-executor", daemon=True
        )
        self._last_event: Optional[MotionEvent] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> "MotionExecutor":
        if not self._worker.is_alive():
            self._worker.start()
        return self

    def close(self, timeout: float = 3.0) -> None:
        self.cancel_all("shutdown")
        self._stop_flag.set()
        self._q.put(None)
        if self._worker.is_alive():
            self._worker.join(timeout=timeout)

    def __enter__(self) -> "MotionExecutor":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # ------------------------------------------------------------------ #
    # Submission
    # ------------------------------------------------------------------ #

    def submit(self, op: str, value: float, *, gesture: bool = False) -> int:
        """Enqueue a move. Returns immediately with a job id."""
        op_l = str(op).lower()
        if op_l not in OPS:
            raise ValueError(f"unknown motion op: {op!r}")
        with self._lock:
            self._next_id += 1
            job = MotionJob(self._next_id, op_l, float(value), gesture)
        self._idle.clear()
        self._q.put(job)
        logger.debug("submit #%d %s %s%s", job.job_id, op_l, value,
                     " (gesture)" if gesture else "")
        return job.job_id

    def submit_sequence(
        self, steps: List[tuple], *, gesture: bool = False
    ) -> List[int]:
        """Enqueue several moves atomically relative to each other."""
        return [self.submit(op, val, gesture=gesture) for op, val in steps]

    # ------------------------------------------------------------------ #
    # Cancellation
    # ------------------------------------------------------------------ #

    def cancel_all(self, reason: str = "") -> List[int]:
        """Drop everything queued and interrupt the move in flight.

        Safe from any thread. This is the ONLY correct way to halt a move that
        has already started — `ArduinoMovement.stop()` would block behind the
        bridge's command lock until the move finished on its own.
        """
        dropped: List[int] = []
        while True:
            try:
                job = self._q.get_nowait()
            except queue.Empty:
                break
            if job is None:
                self._q.put(None)       # preserve the shutdown sentinel
                break
            dropped.append(job.job_id)
            self._publish(MotionEvent(
                job.job_id, job.op, job.value, CANCELLED,
                detail=reason or "cancelled before start", gesture=job.gesture,
            ))

        with self._lock:
            current = self._current

        if current is not None:
            self._cancel.set()
            dropped.append(current.job_id)

        # Fire the brake even with nothing queued — cheap, and it is the
        # correct response to "stop" regardless of what we think is running.
        brake_ok = True
        try:
            self._move.emergency_stop()
        except Exception as e:
            brake_ok = False
            robot_log.event("estop", logging.ERROR, reason=reason or "stop",
                            ok=False, err=f"{type(e).__name__}: {e}")

        if brake_ok:
            robot_log.event(
                "estop", logging.WARNING,
                reason=reason or "stop",
                dropped=dropped,
                interrupted=None if current is None else current.job_id,
            )
        return dropped

    # ------------------------------------------------------------------ #
    # Introspection
    # ------------------------------------------------------------------ #

    @property
    def events(self) -> "queue.Queue[MotionEvent]":
        return self._events

    def status(self) -> Dict[str, Any]:
        with self._lock:
            current = self._current
            last = self._last_event
        return {
            "moving": current is not None,
            "current": None if current is None else {
                "job_id": current.job_id,
                "op": current.op,
                "value": current.value,
                "gesture": current.gesture,
            },
            "gesturing": bool(current is not None and current.gesture),
            "queue_depth": self._q.qsize(),
            "last_event": None if last is None else last.as_dict(),
        }

    def drain(self, timeout: Optional[float] = None) -> bool:
        """Block until the queue empties and nothing is executing."""
        return self._idle.wait(timeout=timeout)

    def wait_for(self, job_id: int, timeout: float) -> Optional[MotionEvent]:
        """Block until `job_id` finishes. Returns its event, or None on timeout.

        The caller consumes the result, so each job can only be waited on once.
        Safe to call before the job has started — the slot is created on demand
        by whichever of wait_for/_publish gets there first.
        """
        with self._lock:
            ev = self._waiters.setdefault(job_id, threading.Event())
        if not ev.wait(timeout):
            return None
        with self._lock:
            self._waiters.pop(job_id, None)
            return self._results.pop(job_id, None)

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #

    def _run(self) -> None:
        logger.info("motion executor started")
        while not self._stop_flag.is_set():
            job = self._q.get()
            if job is None:
                break
            self._execute(job)
            if self._q.empty():
                with self._lock:
                    if self._current is None:
                        self._idle.set()
        logger.info("motion executor stopped")

    def _execute(self, job: MotionJob) -> None:
        with self._lock:
            self._current = job
        self._cancel.clear()
        started = time.monotonic()
        # Only real moves announce their start. A `move.start` with no matching
        # `move.done` is how you spot a hang, which is worth one line — but
        # gesture steps are already enumerated by the `gesture` event's job
        # list, so repeating them here would double the log for no new fact.
        if not job.gesture:
            robot_log.event("move.start", logging.DEBUG, job=job.job_id,
                            op=job.op, val=job.value)

        try:
            self._dispatch(job)
            status, detail = DONE, ""
        except Exception as e:
            # emergency_stop() makes the firmware end the move with a non-OK
            # status, which surfaces here as an exception. If we asked for that,
            # it is a cancellation, not a fault.
            if self._cancel.is_set():
                status, detail = CANCELLED, "interrupted by stop"
            else:
                status, detail = FAILED, f"{type(e).__name__}: {e}"
                logger.exception("job #%d %s failed", job.job_id, job.op)
        finally:
            with self._lock:
                self._current = None

        self._publish(MotionEvent(
            job.job_id, job.op, job.value, status, detail,
            gesture=job.gesture, duration_s=time.monotonic() - started,
        ))

    def _dispatch(self, job: MotionJob) -> None:
        getattr(self._move, OPS[job.op])(job.value)

    _EVT = {DONE: "move.done", CANCELLED: "move.cancelled", FAILED: "move.failed"}
    _LVL = {DONE: logging.INFO, CANCELLED: logging.WARNING, FAILED: logging.ERROR}

    def _publish(self, event: MotionEvent) -> None:
        with self._lock:
            self._last_event = event
            self._results[event.job_id] = event
            self._waiters.setdefault(event.job_id, threading.Event()).set()
        self._events.put(event)

        fields: Dict[str, Any] = {
            "job": event.job_id,
            "op": event.op,
            "val": event.value,
            "dur": round(event.duration_s, 3),
        }
        if event.gesture:
            fields["gesture"] = True
        if event.detail:
            fields["detail"] = event.detail
        robot_log.event(
            self._EVT.get(event.status, "move.failed"),
            self._LVL.get(event.status, logging.ERROR),
            **fields,
        )
