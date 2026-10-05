"""
Timed (open-loop) movement test — encoder-independent diagnostic.

Unlike test_bot_movements.py, this script does NOT use encoder-counted
moves.  It sends the Arduino a plain direction command (F/B/L/R), waits a
fixed number of seconds, then sends the next one.  The Arduino ACKs a
direction command immediately and keeps driving until told otherwise.

Why this exists:
  Encoder-counted moves abort the whole sequence the moment the Arduino
  reports ERR — including an ERR caused by electrical noise rather than a
  real fault.  This script keeps driving regardless and simply COUNTS the
  ERR/BOOT events, so you can tell the difference between:

    "the drivetrain is broken"          → robot doesn't move correctly
    "the serial link is picking up noise" → robot drives fine, but the
                                            summary shows stray ERRs

Encoder counts are still printed when available, purely as information —
nothing depends on them.

  python test_bot_timed.py
  python test_bot_timed.py --duration 15 --speed 40
  python test_bot_timed.py --sequence forward-only --duration 5
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import List, Tuple

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from robot_core import settings

# (label, drivetrain method name)
Step = Tuple[str, str]

SEQUENCES = {
    "basic": [
        ("forward", "intent_forward"),
        ("reverse", "intent_backward"),
        ("left",    "intent_left"),
        ("right",   "intent_right"),
    ],
    "forward-only":  [("forward", "intent_forward")],
    "fwd-rev":       [("forward", "intent_forward"),
                      ("reverse", "intent_backward")],
    "spin":          [("left",  "intent_left"),
                      ("right", "intent_right")],
}


class FaultCounter(logging.Handler):
    """Counts ERR / reset messages logged by the serial layer."""

    def __init__(self) -> None:
        super().__init__(level=logging.WARNING)
        self.err_count = 0
        self.reset_count = 0
        self.messages: List[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        text = record.getMessage()
        if "reset" in text.lower():
            self.reset_count += 1
        elif "ERR" in text:
            self.err_count += 1
        else:
            return
        self.messages.append(text)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Timed open-loop movement test (no encoder dependency).",
    )
    p.add_argument("--port", default=settings.SERIAL_PORT,
                   help="Serial port.")
    p.add_argument("--baud", type=int, default=settings.BAUD_RATE, help="Baud rate.")
    p.add_argument("--speed", type=float, default=settings.DEFAULT_SPEED_PERCENT,
                   help="Speed percent 0-100. Default: settings.DEFAULT_SPEED_PERCENT.")
    p.add_argument("--duration", type=float, default=5.0,
                   help="Seconds to drive per step. Default: 5. "
                        "At 50%% speed the robot covers roughly 0.2 m per second "
                        "— make sure it has room.")
    p.add_argument("--pause", type=float, default=2.0,
                   help="Seconds stopped between steps. Default: 2.")
    p.add_argument("--repeat", type=int, default=1, help="Cycles. Default: 1.")
    p.add_argument("--sequence", choices=sorted(SEQUENCES), default="basic",
                   help="Which sequence to run. Default: basic.")
    return p.parse_args()


def run_step(drivetrain, label: str, method: str,
             speed: float, duration: float) -> None:
    """Drive in one direction for `duration` seconds, then stop."""
    before = drivetrain.get_encoder_status()

    getattr(drivetrain, method)()
    # The V: speed override only applies to a motor that is already
    # commanded to move, so it must be sent AFTER the direction command.
    drivetrain.set_base_speed_percent(speed)

    t0 = time.monotonic()
    try:
        time.sleep(duration)
    finally:
        drivetrain.stop()

    elapsed = time.monotonic() - t0
    after = drivetrain.get_encoder_status()
    dL = after["motor1_count"] - before["motor1_count"]
    dR = after["motor2_count"] - before["motor2_count"]

    if dL == 0 and dR == 0:
        enc = "(no encoder data)"
    else:
        enc = f"ΔL={dL:+d} ΔR={dR:+d}"
    print(f"     drove {elapsed:.2f}s  {enc}")


def main() -> None:
    args = parse_args()

    logging.basicConfig(level=logging.WARNING, format="       ! %(message)s")
    faults = FaultCounter()
    logging.getLogger().addHandler(faults)

    from drivetrain import SerialDrivetrain

    print(f"Connecting on {args.port} @ {args.baud} baud …")
    try:
        drivetrain = SerialDrivetrain(port=args.port, baud=args.baud)
    except Exception as exc:
        print(f"\nERROR: Could not connect to drivetrain: {exc}")
        raise SystemExit(2)

    steps = SEQUENCES[args.sequence]
    repeat = max(1, args.repeat)
    reach = args.speed / 100.0 * 0.4 * args.duration  # crude distance estimate

    print("Connected. TIMED open-loop mode — encoders are NOT used to stop moves.")
    print(
        f"Sequence : {args.sequence}  |  steps={len(steps)}  |  repeat={repeat}  |  "
        f"speed={args.speed:.1f}%  |  {args.duration:.1f}s per step "
        f"(~{reach:.1f} m of travel each)"
    )

    try:
        for cycle in range(1, repeat + 1):
            print(f"\n=== Cycle {cycle}/{repeat} ===")
            for label, method in steps:
                print(f"  → {label:8s}  {args.duration:.1f}s  speed={args.speed:.1f}%")
                run_step(drivetrain, label, method, args.speed, args.duration)
                if args.pause > 0:
                    time.sleep(args.pause)
        print("\nSequence complete — the robot finished every step.")
    except KeyboardInterrupt:
        print("\n^C — stopping robot...")
    finally:
        import signal as _signal

        def _no_op(sig, frame): pass

        old = _signal.getsignal(_signal.SIGINT)
        _signal.signal(_signal.SIGINT, _no_op)
        try:
            drivetrain.close()
        finally:
            _signal.signal(_signal.SIGINT, old)
        print("Stopped and closed drivetrain connection.")

    # ---- The actual diagnostic ----
    print("\n─── Serial health ───")
    if faults.err_count == 0 and faults.reset_count == 0:
        print("Clean: no stray ERR messages, no Arduino resets.")
    else:
        print(f"Stray ERR messages : {faults.err_count}")
        print(f"Arduino resets     : {faults.reset_count}")
        print(
            "\nThe robot drove anyway, so these are the serial link picking up\n"
            "electrical noise — not a drivetrain fault. This is exactly what\n"
            "aborts the encoder-counted script mid-sequence."
        )


if __name__ == "__main__":
    main()
