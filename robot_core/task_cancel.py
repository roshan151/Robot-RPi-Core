"""Cancel the running task (enroll, match, follow, explore) from outside it.

One flag for the process. The agent resets it as a task starts; the cancel-only
voice session (`live/cancel_watch.py`), the `task/stop` service or a button sets
it; the task loops check it between steps and end. Whoever sets it also brakes
(`motion.stop()`), so nothing waits for the loop to notice.
"""

from __future__ import annotations

import threading

_flag = threading.Event()
_by = ""


def reset() -> None:
    global _by
    _by = ""
    _flag.clear()


def request(by: str) -> None:
    """Ask the running task to stop. `by` says who asked (voice, service, ...)."""
    global _by
    if not _flag.is_set():
        _by = by
        _flag.set()


def requested() -> bool:
    return _flag.is_set()


def by() -> str:
    return _by
