"""
Manual movement test script for the robot drivetrain.

All straight moves are specified in METRES and use encoder-counted movement —
the robot drives until the encoder target is reached, not until a timer fires.

Examples:
  python test_bot_movements.py
  python test_bot_movements.py --sequence square --repeat 2 --speed 55
  python test_bot_movements.py --move-distance 0.5 --sequence basic
  python test_bot_movements.py --port /dev/ttyACM0 --baud 115200

Encoder diagnostics are printed after every step so you can immediately spot
a stalled wheel, sync drift, or a TICKS_PER_CM calibration that needs tuning.
"""

from __future__ import annotations

import argparse
import time
from typing import TYPE_CHECKING, Callable, Dict, List, Tuple

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from robot_core import settings

if TYPE_CHECKING:
    from drivetrain import SerialDrivetrain

# Each step is (name, value) where value is metres for straight/reverse
# and degrees for left/right (0 = use --turn-angle default).
Step = Tuple[str, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Test bot movement commands over serial. "
            "Straight moves are encoder-counted — no timer-based guessing."
        ),
    )
    parser.add_argument(
        "--port",
        default=settings.SERIAL_PORT,
        help="Serial port override (default from ROBOT_SERIAL_PORT/config).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=settings.BAUD_RATE,
        help="Baud rate override (default from ROBOT_SERIAL_BAUD/config).",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=settings.DEFAULT_SPEED_PERCENT,
        help="Movement speed percent (0-100). Default: settings.DEFAULT_SPEED_PERCENT (firmware ramps PWM, so inrush is handled there).",
    )
    parser.add_argument(
        "--move-distance",
        type=float,
        default=1.0,
        dest="move_distance",
        help="Distance for forward/reverse steps in METRES. Default: 2.0 m.",
    )
    parser.add_argument(
        "--turn-angle",
        type=float,
        default=90.0,
        help="Degrees for left/right steps. Default: 90.",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=2.0,
        help="Pause in seconds between steps. Default: 5.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="How many times to repeat selected sequence. Default: 1.",
    )
    parser.add_argument(
        "--sequence",
        choices=["basic", "square", "spin"],
        default="basic",
        help="Predefined movement sequence. Default: basic.",
    )
    return parser.parse_args()


def build_sequence(name: str, move_distance_m: float, turn_angle_deg: float) -> List[Step]:
    """
    Build a named movement sequence.

    Each step carries the value it actually uses: metres for
    forward/reverse, degrees for left/right, unused for stop.
    """
    if name == "square":
        return [
            ("forward", move_distance_m),
            ("right",   turn_angle_deg),
        ] * 4
    if name == "spin":
        return [
            ("left",  turn_angle_deg),
            ("right", turn_angle_deg),
            ("left",  turn_angle_deg),
            ("right", turn_angle_deg),
            ("stop",  0.0),
        ]
    # basic
    return [
        ("forward", move_distance_m),
        ("reverse", move_distance_m),
        ("left",    turn_angle_deg),
        ("right",   turn_angle_deg),
        ("stop",    0.0),
    ]


def _encoder_summary(dL: int, dR: int) -> str:
    """Format this move's encoder counts with a drift warning.

    dL and dR are the firmware's per-move counters, which it zeroes when the
    move starts — so they are the travel for THIS step, not a running total.
    They are signed the way each wheel physically turned (negative in reverse,
    opposite signs during a tank turn), so the drift ratio uses magnitudes.
    """
    abs_L = abs(dL)
    abs_R = abs(dR)
    # Compare magnitudes: during a tank turn the wheels counter-rotate, so a
    # raw dL - dR would report ~2x travel as "drift".
    sync = abs_L - abs_R
    denom = max(abs_L, abs_R, 1)
    ratio = abs(abs_L - abs_R) / denom * 100
    flag  = "  ⚠ HIGH DRIFT" if ratio > 10 else ""
    return f"ΔL={dL:+d} ΔR={dR:+d} drift={sync:+d} ({ratio:.1f}%){flag}"


def execute_sequence(
    drivetrain: "SerialDrivetrain",
    sequence: List[Step],
    speed: float,
    pause_s: float,
    repeat: int,
) -> None:
    actions: Dict[str, Callable] = {
        "forward": lambda dist,  spd: drivetrain.straight_m(meters=dist, speed=spd),
        "reverse": lambda dist,  spd: drivetrain.reverse_m(meters=dist, speed=spd),
        "left":    lambda angle, spd: drivetrain.left(angle=angle, speed=spd),
        "right":   lambda angle, spd: drivetrain.right(angle=angle, speed=spd),
        "stop":    lambda _unused, _spd: drivetrain.stop(),
    }

    def label(step_name: str, value: float) -> str:
        if step_name in ("forward", "reverse"):
            return f"{value:.3f} m"
        if step_name in ("left", "right"):
            return f"{value:.1f}°"
        return "—"

    for cycle in range(1, repeat + 1):
        print(f"\n=== Cycle {cycle}/{repeat} ===")
        for step_name, value in sequence:
            print(f"  → {step_name:8s}  {label(step_name, value):<12s}  speed={speed:.1f}%")

            t0 = time.monotonic()
            try:
                actions[step_name](value, speed)
                status = "ok"
            except Exception as exc:
                status = f"ERROR: {exc}"
            elapsed = time.monotonic() - t0

            # The firmware zeroes both counters when a move begins, so these
            # ARE this step's counts — no before/after subtraction, which
            # would straddle that reset and report fictional travel.
            time.sleep(settings.ENCODER_SETTLE_S)
            counts = drivetrain.get_encoder_status()
            dL = counts["motor1_count"]
            dR = counts["motor2_count"]

            if step_name not in ("stop",):
                enc_line = _encoder_summary(dL, dR)
            else:
                enc_line = "(stop — no encoder check)"

            print(f"     {status:<6s}  elapsed={elapsed:.2f}s  {enc_line}")

            if status != "ok":
                print("\nAborting — step failed.")
                raise RuntimeError(f"step '{step_name}' failed: {status}")

            if pause_s > 0:
                time.sleep(pause_s)


def main() -> None:
    args = parse_args()
    repeat = max(1, args.repeat)

    from drivetrain import SerialDrivetrain

    print(f"Connecting on {args.port} @ {args.baud} baud …")
    try:
        drivetrain = SerialDrivetrain(port=args.port, baud=args.baud)
    except TimeoutError:
        print(
            "\nERROR: Timed out waiting for ACK from Arduino.\n"
            "Fixes to try:\n"
            "  1) Confirm the port name (macOS: /dev/cu.usb*, Linux: /dev/ttyUSB0).\n"
            "  2) Close Arduino IDE Serial Monitor or any other app holding the port.\n"
            "  3) Press the Arduino reset button, then rerun this script.\n"
            "  4) Confirm firmware baud is 115200 and the sketch is uploaded."
        )
        raise SystemExit(2)
    except Exception as exc:
        print(f"\nERROR: Could not connect to drivetrain: {exc}")
        raise SystemExit(2)

    sequence = build_sequence(args.sequence, args.move_distance, args.turn_angle)

    print("Connected. Encoder-counted moves active (no timer fallback).")
    print(
        f"Sequence : {args.sequence}  |  "
        f"steps={len(sequence)}  |  "
        f"repeat={repeat}  |  "
        f"speed={args.speed:.1f}%  |  "
        f"distance={args.move_distance:.3f} m  |  "
        f"turn={args.turn_angle:.1f}°"
    )

    try:
        execute_sequence(
            drivetrain=drivetrain,
            sequence=sequence,
            speed=args.speed,
            pause_s=max(0.0, args.pause),
            repeat=repeat,
        )
        print("\nSequence complete.")
    except KeyboardInterrupt:
        print("\n^C — stopping robot...")
    finally:
        # close() sends two S commands and waits 150 ms for the firmware to
        # zero PWM outputs before closing the port.  We suppress further
        # KeyboardInterrupt signals here so a second ^C cannot interrupt the
        # stop sequence and leave the motors running.
        import signal as _signal

        def _no_op(sig, frame): pass

        old = _signal.getsignal(_signal.SIGINT)
        _signal.signal(_signal.SIGINT, _no_op)
        try:
            drivetrain.close()
        finally:
            _signal.signal(_signal.SIGINT, old)
        print("Stopped and closed drivetrain connection.")


if __name__ == "__main__":
    main()
