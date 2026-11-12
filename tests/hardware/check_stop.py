"""Does the Arduino stop when told to, and is it still alive afterwards?

Talks to the drivetrain firmware directly (no robot stack). LIFT THE ROBOT
so the wheels spin freely, stop the service first, then:

    sudo systemctl stop robot-voice
    python tests/hardware/check_stop.py            # or: PORT=/dev/ttyUSB0 python ...

Sequence: boot -> turn R at low speed for 1 s -> S (stop) -> keep sending
heartbeats 2 s -> go silent 3 s (firmware link watchdog should brake).
Every frame is printed with a timestamp. E telemetry should arrive every
100 ms the whole time; a gap means the ATmega has HUNG.
"""
from __future__ import annotations
import glob, os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import serial
from robot_core.drivetrain.serial_protocol import FrameParser, encode_frame, describe_reset

PORT = os.environ.get("PORT") or (glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                                  + glob.glob("/dev/cu.usbserial*") or [""])[0]
SPEED = int(os.environ.get("SPEED", "100"))

ser = serial.Serial(PORT, 115200, timeout=0.02)   # opening resets the Arduino (DTR)
parser = FrameParser()
t0 = time.monotonic()
last_e = None
max_gap = 0.0
boots = []

def now(): return time.monotonic() - t0

def pump(seconds, heartbeat=False):
    global last_e, max_gap
    end = time.monotonic() + seconds
    next_hb = 0.0
    while time.monotonic() < end:
        if heartbeat and time.monotonic() >= next_hb:
            ser.write(encode_frame("P")); next_hb = time.monotonic() + 0.2
        for body in parser.feed(ser.read(256)):
            t = now()
            if body.startswith("E,"):
                if last_e is not None:
                    gap = t - last_e
                    max_gap = max(max_gap, gap)
                    if gap > 0.5:
                        print(f"{t:7.2f}  !!! telemetry gap {gap:.2f}s (MCU was not running)")
                last_e = t
                if int(t * 10) % 5 == 0:            # print ~2/s to keep it readable
                    print(f"{t:7.2f}  {body}")
                continue
            if body.startswith("B,"):
                cause = body.split(",")[1]
                boots.append((t, cause))
                body += f"   <- reset: {describe_reset(cause)}"
            print(f"{t:7.2f}  {body}")
        if last_e is not None and now() - last_e > 0.5 and int(now() * 4) % 4 == 0:
            print(f"{now():7.2f}  !!! no telemetry for {now() - last_e:.1f}s — MCU HUNG?")
            time.sleep(0.25)

def send(body, label):
    print(f"{now():7.2f}  >>> {body}   ({label})")
    ser.write(encode_frame(body))

print(f"port={PORT} speed={SPEED}\n--- waiting for boot")
pump(3.0)
send(f"V,1,{SPEED}", "set speed")
send("I,2,R", "turn right")
pump(1.0, heartbeat=True)
send("S,3", "STOP — wheels should ramp down and brake now")
pump(2.0, heartbeat=True)
print(f"{now():7.2f}  --- going silent; nothing should move")
pump(3.0)

print("\n=== summary")
print(f"max telemetry gap: {max_gap:.2f}s  ({'OK' if max_gap < 0.3 else 'MCU STALLED'})")
print(f"boots seen: {[(round(t,2), c) for t, c in boots]}  (expect exactly one, at start)")
print("Now: did BOTH wheels stop after the S line? Did any wheel start on its own?")
