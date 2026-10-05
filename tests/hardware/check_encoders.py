"""
Encoder hand-spin check — no motors, no PID, just raw counts.

Connect, then spin each wheel BY HAND and watch the live counts:

  1. Spin the LEFT wheel slowly  → only the L column should climb.
  2. Spin the RIGHT wheel slowly → only the R column should climb.
  3. Stop touching everything    → both rates should read 0/s (a count
     that climbs on its own = electrical noise, not a wheel).
  4. Wiggle each encoder cable near its connector while watching —
     a burst of counts = intermittent contact / noise pickup.

Notes:
  * Quadrature (fw v4+): counts are hardware-signed.  Rolling a wheel
    in the robot's FORWARD direction must count UP; if a side counts
    down, flip its ENC_x_INVERT in drivetrain.ino and re-flash.
  * A slow steady hand-spin should produce a smooth, proportional count
    (~2 counts per degree-ish depending on your CPR).  Jumps of hundreds
    of ticks from one 100 ms report to the next are noise.

  python test_encoders.py
  python test_encoders.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import sys
import time
import threading

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from robot_core import settings

from robot_core.drivetrain import ArduinoBridge

# Flag any jump bigger than this between two 100 ms telemetry reports.
# A wheel hand-spun fast is well under this; noise bursts are way over.
BURST_THRESHOLD = 200


def main() -> None:
    p = argparse.ArgumentParser(description="Live encoder counts for hand-spin checks.")
    p.add_argument("--port", default=settings.SERIAL_PORT, help="Serial port.")
    p.add_argument("--baud", type=int, default=settings.BAUD_RATE, help="Baud rate.")
    args = p.parse_args()

    state = {"l": 0, "r": 0, "pl": None, "pr": None, "t": time.monotonic()}
    lock = threading.Lock()
    events: list[str] = []

    def on_encoder(l: int, r: int) -> None:
        now = time.monotonic()
        with lock:
            if state["pl"] is not None:
                dl = abs(l - state["pl"])
                dr = abs(r - state["pr"])
                if dl > BURST_THRESHOLD or dr > BURST_THRESHOLD:
                    events.append(
                        f"[{time.strftime('%H:%M:%S')}] BURST  ΔL={l - state['pl']:+d}"
                        f"  ΔR={r - state['pr']:+d}  in one report — noise or loose wire"
                    )
            state["pl"], state["pr"] = l, r
            state["l"], state["r"] = l, r
            state["t"] = now

    print(f"Connecting on {args.port} @ {args.baud} baud …")
    bridge = ArduinoBridge(port=args.port, baud=args.baud, on_encoder=on_encoder)
    print("Connected. Motors will NOT be driven.")
    print("Spin each wheel by hand. Ctrl-C to finish.\n")
    print(f"{'L count':>12}  {'R count':>12}  {'L rate/s':>10}  {'R rate/s':>10}")

    last_l, last_r = 0, 0
    last_t = time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            with lock:
                l, r = state["l"], state["r"]
            now = time.monotonic()
            dt = now - last_t
            rate_l = (l - last_l) / dt
            rate_r = (r - last_r) / dt
            last_l, last_r, last_t = l, r, now
            sys.stdout.write(
                f"\r{l:>12d}  {r:>12d}  {rate_l:>10.0f}  {rate_r:>10.0f}   "
            )
            sys.stdout.flush()
            with lock:
                while events:
                    print("\n" + events.pop(0))
    except KeyboardInterrupt:
        print("\n\n─── Summary ───")
        print(f"Final counts: L={last_l}  R={last_r}")
        print("Checks: correct wheel → correct column?  rates 0 when idle?")
        print("        any BURST lines above = noise on that channel's wiring.")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
