"""The /v1/detect backpressure policy. VISION-DESIGN.md Part 4.3, exactly.

    in flight               exactly one, never more
    frame while in flight   dropped, never queued          -> dropped_busy
    hard timeout            DETECT_TIMEOUT_S, no retry      -> unavailable
    503 from the server     a dropped frame, not an error   -> server_busy
    seq older than applied  discarded                       -> out_of_order
    older than MAX_AGE      discarded                       -> stale
    N failures in a row     degraded = True                 -> perception DEGRADED

The request runs on its own thread so the capture/tracker loop never waits on
the network. Every counter is exposed: silent discarding reads as "it's
working" when it isn't (PLAN standing rule #6).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import asdict, dataclass
from typing import Callable, Optional

from .client import Busy, DetectResult, ServiceError, Unavailable

log = logging.getLogger(__name__)

DetectFn = Callable[[bytes, int], DetectResult]           # (jpeg, seq) -> result
ResultFn = Callable[[DetectResult, float], None]          # (result, t_capture)


@dataclass
class GatewayCounters:
    sent: int = 0
    ok: int = 0
    dropped_busy: int = 0
    server_busy: int = 0
    unavailable: int = 0
    errors: int = 0
    out_of_order: int = 0
    stale: int = 0
    consecutive_failures: int = 0
    last_latency_ms: float = 0.0
    last_inference_ms: float = 0.0

    def as_dict(self) -> dict:
        return asdict(self)


class DetectGateway:
    def __init__(
        self,
        detect: DetectFn,
        on_result: ResultFn,
        *,
        max_result_age_s: float = 1.0,
        degraded_after: int = 5,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._detect = detect
        self._on_result = on_result
        self._max_age = max_result_age_s
        self._degraded_after = degraded_after
        self._clock = clock
        self._lock = threading.Lock()
        self._in_flight = False
        self._last_applied_seq = -1
        self.counters = GatewayCounters()
        self.model_version = ""

    # ------------------------------------------------------------------ #

    @property
    def degraded(self) -> bool:
        return self.counters.consecutive_failures >= self._degraded_after

    @property
    def busy(self) -> bool:
        return self._in_flight

    def submit(self, jpeg: bytes, seq: int, t_capture: float) -> bool:
        """Start a request on a worker thread, or drop the frame. Never blocks."""
        if not self._claim():
            return False
        threading.Thread(target=self._run_claimed, args=(jpeg, seq, t_capture),
                         name=f"detect-{seq}", daemon=True).start()
        return True

    def run_once(self, jpeg: bytes, seq: int, t_capture: float) -> Optional[DetectResult]:
        """Synchronous version of submit(), same policy. Returns the applied
        result or None. Used by tests and by one-shot callers."""
        if not self._claim():
            return None
        return self._run_claimed(jpeg, seq, t_capture)

    # ------------------------------------------------------------------ #

    def _claim(self) -> bool:
        with self._lock:
            if self._in_flight:
                self.counters.dropped_busy += 1
                return False
            self._in_flight = True
            self.counters.sent += 1
            return True

    def _run_claimed(self, jpeg: bytes, seq: int, t_capture: float) -> Optional[DetectResult]:
        t0 = self._clock()
        try:
            result = self._detect(jpeg, seq)
        except Busy:
            with self._lock:
                self.counters.server_busy += 1   # not a failure: the server is honest
            return None
        except Unavailable as exc:
            self._fail("unavailable", exc)
            return None
        except ServiceError as exc:
            self._fail("errors", exc)
            return None
        except Exception as exc:                          # noqa: BLE001 — a bug must not kill the loop
            self._fail("errors", exc)
            return None
        finally:
            with self._lock:
                self._in_flight = False

        now = self._clock()
        with self._lock:
            c = self.counters
            c.last_latency_ms = round((now - t0) * 1000.0, 1)
            c.last_inference_ms = result.inference_ms
            if result.seq != seq:
                c.errors += 1
                log.warning("detect: sent seq %d, got seq %d back — discarded", seq, result.seq)
                return None
            if seq <= self._last_applied_seq:
                c.out_of_order += 1
                return None
            if now - t_capture > self._max_age:
                c.stale += 1
                c.consecutive_failures += 1      # a stale answer is as useless as none
                return None
            c.ok += 1
            c.consecutive_failures = 0
            self._last_applied_seq = seq
            if result.model_version and result.model_version != self.model_version:
                log.info("detect model is now %s (was %s)", result.model_version,
                         self.model_version or "unknown")
                self.model_version = result.model_version
        self._on_result(result, t_capture)
        return result

    def _fail(self, counter: str, exc: BaseException) -> None:
        with self._lock:
            setattr(self.counters, counter, getattr(self.counters, counter) + 1)
            self.counters.consecutive_failures += 1
            n = self.counters.consecutive_failures
        if n == self._degraded_after:
            log.warning("detect DEGRADED after %d failures in a row; last: %s", n, exc)
