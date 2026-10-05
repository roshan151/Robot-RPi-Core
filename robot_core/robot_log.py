"""
Structured event log — the robot's only diagnostic channel.

The robot does not speak.  When something goes wrong there is no verbal
feedback, no screen, and often nobody watching the terminal.  This file is the
entire story of what happened, so it has two hard requirements:

  1. It must be readable, not a wall of text.  One line per event, fixed field
     names, a closed vocabulary of event names.  `grep '"evt":"estop"'` has to
     work.
  2. It must record the cause BEFORE the process dies.  Uncaught exceptions,
     exceptions on worker threads, SIGTERM from systemd, and interpreter-level
     crashes are all covered.  Records are flushed on every write, so a SIGKILL
     loses nothing already logged.

Format
------
JSON Lines: one complete JSON object per line, in a file named `logs.json`.
Deliberately NOT a single JSON array — an array cannot be safely appended to,
and a process killed mid-write would leave the whole file unparseable.  Each
line stands alone, so a truncated final line costs you one record instead of
the entire history.

    {"ts":"2026-08-11T09:48:03.412Z","lvl":"warn","evt":"estop","reason":"stop command","dropped":[4]}

Read it back with:
    python -c "import json,sys;[print(json.loads(l)) for l in open('logs.json')]"

What lands in the file
----------------------
  * every structured event() call, at any level
  * every WARNING or worse from any logger in the process

Routine INFO chatter from library loggers stays on the console only.  That is
the difference between a log you read and a log you scroll past.
"""

from __future__ import annotations

import atexit
import faulthandler
import json
import logging
import logging.handlers
import os
import signal
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from robot_core import settings

# --------------------------------------------------------------------------- #
# Event vocabulary — keep this closed.  A new event name is a deliberate act.
# --------------------------------------------------------------------------- #

EVENTS = {
    # lifecycle
    "session.start",      # process came up
    "session.stop",       # clean shutdown
    "fatal",              # process is dying, and why
    "signal",             # SIGTERM / SIGINT received
    # motion
    "move.start",
    "move.done",
    "move.cancelled",
    "move.failed",
    "estop",              # out-of-band brake fired
    # conversation
    "gesture",            # YES / NO / UNCLEAR played
    "gesture.skip",       # suppressed, with the reason
    # power
    "battery",           # percent / volts at startup
    "battery.say",       # spoken battery report
    # voice
    "voice.connect",     # live session established
    "voice.connected",   # spoken "session up" announcement
    "voice.error",       # spoken failure announcement, with the cause
    "voice.drop",        # session died, with the cause
    "voice.heard",       # transcript of what the operator said
    "voice.say",         # what the model said (never played aloud)
    "voice.tool",        # a tool call and its result
    "audio.error",       # microphone, uplink or playback trouble
}

_LEVEL_NAME = {
    logging.DEBUG: "debug",
    logging.INFO: "info",
    logging.WARNING: "warn",
    logging.ERROR: "error",
    logging.CRITICAL: "fatal",
}

_log = logging.getLogger("robot")
_state: Dict[str, Any] = {"handler": None, "path": None, "on_fatal": None}
_dedupe_lock = threading.Lock()
_dedupe: Dict[str, list] = {}


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.") + \
        f"{datetime.now(timezone.utc).microsecond // 1000:03d}Z"


# --------------------------------------------------------------------------- #
# Redaction
# --------------------------------------------------------------------------- #
# This file is the diagnostic channel, which means it gets copied off the Pi,
# pasted into issues, and sent to whoever is helping. A key that reaches it is
# a key that has leaked. Two independent guards, because either alone fails:
#
#   * exact-value scrubbing catches a credential arriving through any field or
#     any traceback, including ones nobody thought to name
#   * key-name matching catches values that are secret but not in config, e.g.
#     a token parsed out of a response at runtime
#
# Redaction happens in the formatter, so there is no way to log around it.

_SECRET_KEY_HINTS = ("key", "token", "secret", "password", "passwd", "authorization")
_REDACTED = "<redacted>"


def _scrub(value: Any, secrets: tuple) -> Any:
    if isinstance(value, str):
        for s in secrets:
            if s and s in value:
                value = value.replace(s, _REDACTED)
        return value
    if isinstance(value, dict):
        return {k: _scrub_field(k, v, secrets) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_scrub(v, secrets) for v in value]
    return value


def _scrub_field(key: str, value: Any, secrets: tuple) -> Any:
    if any(h in key.lower() for h in _SECRET_KEY_HINTS):
        return _REDACTED if value else value
    return _scrub(value, secrets)


class JsonlFormatter(logging.Formatter):
    """One self-contained JSON object per line."""

    def format(self, record: logging.LogRecord) -> str:
        payload: Dict[str, Any] = {
            "ts": _now(),
            "lvl": _LEVEL_NAME.get(record.levelno, record.levelname.lower()),
            "evt": getattr(record, "evt", "log"),
        }
        fields = getattr(record, "fields", None)
        if fields:
            payload.update(fields)

        # Plain logger.warning()/error() calls carry no structured fields, so
        # keep their text and say where it came from.
        if not hasattr(record, "evt"):
            payload["msg"] = record.getMessage()
            payload["src"] = record.name

        if record.exc_info:
            exc_type, exc_val, tb = record.exc_info
            payload["err"] = f"{exc_type.__name__}: {exc_val}"
            payload["tb"] = _short_tb(tb)

        # Last step before the line is written — nothing can bypass it.
        # An exception message is a common leak path: SDKs happily put the
        # key in the text of an auth error.
        try:
            secrets = settings.secret_values()
        except Exception:
            secrets = ()
        payload = {k: _scrub_field(k, v, secrets) for k, v in payload.items()}

        return json.dumps(payload, default=str, separators=(",", ":"))


def _short_tb(tb: Any, limit: int = 6) -> list:
    """Compact traceback: 'file:line in func' frames, innermost last."""
    return [
        f"{Path(f.filename).name}:{f.lineno} in {f.name}"
        for f in traceback.extract_tb(tb)[-limit:]
    ]


class _FileFilter(logging.Filter):
    """Structured events always; unstructured records only if WARNING+.

    This is the whole 'clean not verbose' policy in four lines.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return hasattr(record, "evt") or record.levelno >= logging.WARNING


# --------------------------------------------------------------------------- #
# Setup
# --------------------------------------------------------------------------- #

def setup(
    path: Optional[str] = None,
    console_level: int = logging.INFO,
    max_bytes: Optional[int] = None,
    backups: Optional[int] = None,
) -> Path:
    """Configure logging. Safe to call twice; the second call is a no-op."""
    if _state["handler"] is not None:
        return _state["path"]

    log_path = Path(path or settings.LOG_PATH).expanduser()
    max_bytes = settings.LOG_MAX_BYTES if max_bytes is None else max_bytes
    backups = settings.LOG_BACKUPS if backups is None else backups
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # StreamHandler.emit() flushes after every record, so anything already
    # logged survives a SIGKILL. Only power loss can lose it.
    handler = logging.handlers.RotatingFileHandler(
        log_path, maxBytes=max_bytes, backupCount=backups, encoding="utf-8"
    )
    handler.setFormatter(JsonlFormatter())
    handler.addFilter(_FileFilter())
    handler.setLevel(logging.DEBUG)

    console = logging.StreamHandler(sys.stderr)
    console.setFormatter(logging.Formatter("%(levelname)-5s %(name)s: %(message)s"))
    console.setLevel(console_level)

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    for h in list(root.handlers):
        root.removeHandler(h)
    root.addHandler(handler)
    root.addHandler(console)

    _state["handler"] = handler
    _state["path"] = log_path

    # Interpreter-level crashes (segfault, C-extension blowup) never reach
    # Python's exception machinery — faulthandler is the only way to see them.
    try:
        crash_file = open(log_path.with_suffix(".crash"), "a", buffering=1)
        faulthandler.enable(file=crash_file)
    except Exception:
        pass

    return log_path


def event(evt: str, level: int = logging.INFO, **fields: Any) -> None:
    """Record one structured event.

    Unknown event names are recorded rather than dropped — losing a diagnostic
    because of a typo would defeat the point — but they are flagged so the
    vocabulary stays honest.
    """
    if evt not in EVENTS:
        fields["_unregistered"] = True
    _log.log(level, evt, extra={"evt": evt, "fields": fields})


def event_throttled(
    evt: str, key: str, window_s: float = 30.0, level: int = logging.INFO, **fields: Any
) -> None:
    """Like event(), but collapses repeats of the same key inside a window.

    For things that fire per control-loop tick — encoder warnings, guardian
    detections. The suppressed count rides along on the next emitted record so
    nothing is silently lost.
    """
    now = time.monotonic()
    ident = f"{evt}:{key}"
    with _dedupe_lock:
        last, count = _dedupe.get(ident, (0.0, 0))
        if now - last < window_s:
            _dedupe[ident] = (last, count + 1)
            return
        _dedupe[ident] = (now, 0)
    if count:
        fields["repeats"] = count
    event(evt, level=level, **fields)


# --------------------------------------------------------------------------- #
# Death
# --------------------------------------------------------------------------- #

def _flush() -> None:
    h = _state["handler"]
    if h is not None:
        try:
            h.flush()
            os.fsync(h.stream.fileno())      # only on the way out; too slow per record
        except Exception:
            pass


def _run_on_fatal(cause: str) -> None:
    cb = _state.get("on_fatal")
    if cb is None:
        return
    try:
        cb(cause)
    except Exception as e:
        event("fatal", logging.CRITICAL, cause="on_fatal handler failed",
              err=f"{type(e).__name__}: {e}")


def install_crash_handlers(on_fatal: Optional[Callable[[str], None]] = None) -> None:
    """Guarantee the cause of death is on disk before the process goes.

    Covers four distinct ways this process can die, all of which would
    otherwise leave an empty log and a robot stopped in the middle of a room:

      * uncaught exception on the main thread
      * uncaught exception on a worker thread (the motion executor, the serial
        reader, the heartbeat) — these do NOT reach sys.excepthook, and without
        threading.excepthook they die in silence
      * SIGTERM, which is how systemd stops the service
      * interpreter-level crash, via faulthandler in setup()

    `on_fatal` runs first and should stop the motors. SIGKILL cannot be caught
    by anything; the firmware's link watchdog is what covers that case, braking
    when the heartbeat stops.
    """
    _state["on_fatal"] = on_fatal

    def _excepthook(exc_type, exc_val, tb):
        if issubclass(exc_type, KeyboardInterrupt):
            event("session.stop", reason="keyboard interrupt")
            _run_on_fatal("keyboard interrupt")
            _flush()
            return
        _run_on_fatal("uncaught exception")
        event("fatal", logging.CRITICAL,
              cause="uncaught exception",
              err=f"{exc_type.__name__}: {exc_val}",
              tb=_short_tb(tb),
              thread="main")
        _flush()

    def _thread_excepthook(args):
        if issubclass(args.exc_type, SystemExit):
            return
        _run_on_fatal(f"thread {args.thread.name} died")
        event("fatal", logging.CRITICAL,
              cause="uncaught exception in thread",
              err=f"{args.exc_type.__name__}: {args.exc_value}",
              tb=_short_tb(args.exc_traceback),
              thread=args.thread.name if args.thread else "?")
        _flush()

    def _on_signal(signum, _frame):
        name = signal.Signals(signum).name
        event("signal", logging.WARNING, sig=name, action="stopping")
        _run_on_fatal(f"signal {name}")
        _flush()
        # Restore default and re-raise so the exit status is honest about
        # having been signalled.
        signal.signal(signum, signal.SIG_DFL)
        os.kill(os.getpid(), signum)

    sys.excepthook = _excepthook
    threading.excepthook = _thread_excepthook
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):
            pass                    # not the main thread, or unsupported

    atexit.register(_flush)


def read_events(path: Optional[str] = None, limit: int = 50) -> list:
    """Load the last `limit` records. Tolerates a truncated final line."""
    p = Path(path or _state["path"] or getattr(settings, "LOG_PATH", "logs.json"))
    if not p.exists():
        return []
    out = []
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines()[-limit:]:
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out
