"""Find out what the head servos really are, moving them only a hair.

  sudo systemctl stop robot-voice            # it holds the PWM channels
  python tests/hardware/check_servo_pulse.py probe tilt
  python tests/hardware/check_servo_pulse.py probe pan
  python tests/hardware/check_servo_pulse.py trim pan      # find the centre / stop pulse
  python tests/hardware/check_servo_pulse.py probe pan --fake    # no hardware, just the prompts

This bypasses Gimbal on purpose: no angles, no invert, no speed profile. What
you see is exactly the pulse printed on screen, and a pulse can never leave
1500 +-150 us (about +-14 degrees on a positional SG90).

  POSITIONAL servo (SG90, MG90S): the pulse picks an ANGLE. It turns there,
  holds, and comes back to the same spot when the pulse comes back.
  CONTINUOUS-ROTATION servo ("SG90 360"): the pulse picks a SPEED and
  direction, ~1500 us = stop. A small offset makes it spin for as long as you
  hold it, and it does not come back.

Everything in robot_core/sensors/gimbal.py assumes the first kind.

Take the horn OFF (or leave the head free to turn and the cable slack), and
stick a bit of tape on the horn so you can see where it started.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
from robot_core.sensors.gimbal import FakePwm, HardwarePwm  # noqa: E402

YAML = ROOT / "ros2_ws/src/robot_bringup/config/robot.yaml"
NEUTRAL_US = 1500.0
MAX_OFFSET_US = 150.0                 # hard cap either side of neutral
HOLD_S = 1.0                          # how long each probe pulse is held
OFFSETS = (10, 20, 30, 40, 60, 80, 100)


class Limited:
    """A PWM output that cannot be asked for a pulse outside NEUTRAL +- MAX_OFFSET."""

    def __init__(self, out) -> None:
        self.out = out

    def set(self, us: float) -> float:
        us = max(NEUTRAL_US - MAX_OFFSET_US, min(NEUTRAL_US + MAX_OFFSET_US, us))
        self.out.set_pulse_us(us)
        print(f"      -> {us:.0f} us")
        return us

    def close(self) -> None:
        try:
            self.out.set_pulse_us(NEUTRAL_US)
            time.sleep(0.4)
        finally:
            self.out.disable()


def ask(prompt: str, keys: str) -> str:
    while True:
        a = input(prompt).strip().lower()
        if a and a[0] in keys:
            return a[0]


def probe(pwm: Limited) -> str:
    print("\nStep 0: holding 1500 us for 3 s. A servo at rest is still and quiet.")
    pwm.set(NEUTRAL_US)
    time.sleep(3.0)
    a = ask("  horn: still (s) / creeping or turning (c) / buzzing or twitching (b)? ", "scb")
    if a == "c":
        return ("\nVERDICT: it will not hold still at 1500 us. Most likely a continuous-rotation servo whose\n"
                "stop point is off 1500 (a positional one just sits there). Run `trim` to find the pulse where\n"
                "it stops, then `probe` again: if bumps then make it keep turning, it is continuous-rotation,\n"
                "and a servo whose pulse sets SPEED cannot be a gimbal axis.")
    if a == "b":
        return ("\nVERDICT: it twitches at a steady pulse. Suspect power or signal, not the code: servos on\n"
                "their own 5 V rail (not the Pi's), ground shared with the Pi, short signal wire.")

    print(f"\nBumps: each pulse is held {HOLD_S:.0f} s, then it returns to 1500 us. For each one say what the horn did:")
    print("  n = nothing moved")
    print("  p = moved to a spot, HELD it, and came BACK to the tape mark when the pulse returned")
    print("  c = kept turning the whole time and did NOT come back to the mark")
    print("  q = quit")
    results: list = []
    for off in OFFSETS:
        for sign in (+1, -1):
            us = NEUTRAL_US + sign * off
            input(f"\n  Enter to send {us:.0f} us ({sign * off:+d})... ")
            pwm.set(us)
            time.sleep(HOLD_S)
            pwm.set(NEUTRAL_US)
            time.sleep(1.0)
            a = ask(f"  {us:.0f} us: nothing (n) / held + came back (p) / kept turning (c) / quit (q)? ", "npcq")
            if a == "q":
                return summary(results)
            results.append((sign * off, a))
            if a == "c":
                return summary(results)
    return summary(results)


def summary(results: list) -> str:
    names = {"n": "nothing", "p": "moved, held, came back (positional)", "c": "kept turning (continuous)"}
    lines = ["", "offset (us)   what it did"]
    lines += [f"  {off:+5d}       {names[a]}" for off, a in results]
    kinds = {a for _, a in results}
    if "c" in kinds:
        verdict = ("CONTINUOUS ROTATION. The pulse sets speed, there is no angle to hold, so the gimbal can\n"
                   "never be made to work in software. Swap in true 180-degree SG90 / MG90S servos (the\n"
                   "pan/tilt code, limits and robot.yaml need no change). Do not try to dead-reckon with\n"
                   "timed spins: tilt would sag under load and the pan lidar sweep needs a real angle.")
    elif "p" in kinds:
        up = min((o for o, a in results if a == "p" and o > 0), default=None)
        dn = max((o for o, a in results if a == "p" and o < 0), default=None)
        verdict = (f"POSITIONAL. First offset that moved it: {up:+d} us on the + side, {dn:+d} us on the - side"
                   if up and dn else "POSITIONAL, but it only responded on one side - check the other direction.")
        verdict += ("\nNext: at +100 and -100 us measure how many degrees the horn really turned (protractor/phone).\n"
                    "  us_per_deg = 100 / degrees  -> put it in robot.yaml as pan_us_per_deg / tilt_us_per_deg\n"
                    "  (the code assumes 11.11). Then run `trim` and write the result to *_center_us.")
    else:
        verdict = ("NOTHING MOVED up to +-100 us. Not a code problem if `cat /sys/class/pwm/pwmchip0/pwm*/duty_cycle`\n"
                   "shows ~1500000 while this runs: check servo power (own 5 V rail, shared ground), the signal\n"
                   "wire on the right GPIO, and `pinctrl get 12,13` (both should say a0).")
    return "\n".join(lines) + "\n\nVERDICT: " + verdict


def trim(pwm: Limited) -> float:
    us = NEUTRAL_US
    print("\nHolding the pulse. Type a / d for -5 / +5 us, A / D for -1 / +1 us (repeat the letter for more,")
    print("'ddd' = +15). Enter when done.")
    print("Positional servo: stop when the horn sits at the middle of its travel.")
    print("Continuous servo: note the pulse where creeping starts going up, and where it starts going down;")
    print("the middle of that quiet zone is its stop pulse.")
    pwm.set(us)
    while True:
        cmd = input(f"  {us:.0f} us > ").strip()
        if not cmd:
            return us
        step = {"a": -5, "d": 5, "A": -1, "D": 1}.get(cmd[0])
        if step is None:
            print("  use a d A D, or Enter")
            continue
        us = pwm.set(us + step * len(cmd))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("probe", "trim"))
    ap.add_argument("axis", choices=("pan", "tilt"))
    ap.add_argument("--fake", action="store_true", help="no hardware")
    a = ap.parse_args()

    p = yaml.safe_load(YAML.read_text())["/explorer"]["ros__parameters"]
    channel = int(p.get(f"{a.axis}_channel", 1 if a.axis == "pan" else 0))
    chip = int(p.get("pwm_chip", 0))
    pwm = Limited(FakePwm() if a.fake else HardwarePwm(channel, chip))
    print(f"{a.axis}: PWM chip {chip} channel {channel}. Pulses are held within "
          f"{NEUTRAL_US - MAX_OFFSET_US:.0f}..{NEUTRAL_US + MAX_OFFSET_US:.0f} us.")
    try:
        if a.mode == "probe":
            print(probe(pwm))
        else:
            us = trim(pwm)
            print(f"\n{a.axis}_center_us: {us:.0f}   <- put this in robot.yaml")
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
    finally:
        pwm.close()                   # back to 1500 us, then release the PWM
    return 0


if __name__ == "__main__":
    sys.exit(main())
