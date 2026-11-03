#!/usr/bin/env python3
"""Read-only diagnostics for the pan/tilt head. Run it ON THE PI, as the user the robot runs as (no sudo):

    python3 tests/hardware/check_head_diag.py              # everything, once (about 5 s)
    python3 tests/hardware/check_head_diag.py --watch 20   # also watch the servo-supply sense pin for 20 s:
                                                           # flip the servo switch OFF then ON while it runs

It only READS. It never touches the PWM channels, the servo pins, the robot, or any file in the repo, so it is
safe to run with the servos powered and the robot running. Every check prints OK / WARN / FAIL / INFO plus a
dim "expect:" line saying what a healthy robot shows. A plain-text copy is saved to ~/robot-diag-<time>.txt:
paste that (or the terminal output) back when asking for help.

Stdlib only; nothing to install.
"""

from __future__ import annotations

import argparse
import datetime as dt
import getpass
import glob
import grp
import json
import os
import platform
import re
import subprocess
import sys
import textwrap
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
YAML_SRC = ROOT / "ros2_ws/src/robot_bringup/config/robot.yaml"
YAML_INST = ROOT / "ros2_ws/install/robot_bringup/share/robot_bringup/config/robot.yaml"   # what the service actually reads
CONFIG_TXT = [Path("/boot/firmware/config.txt"), Path("/boot/config.txt")]
SYS_PWM = Path("/sys/class/pwm")
LOCK = "/dev/shm/robot-gimbal.lock"
SERVICE = "robot-voice"
HOME = Path(os.path.expanduser("~" + os.environ["SUDO_USER"])) if os.environ.get("SUDO_USER") else Path.home()
PERIOD_NS = 20_000_000
WIDTH = 100

# ------------------------------------------------------------------ output

COLORS = {"OK": "32", "WARN": "33", "FAIL": "31", "INFO": "36"}
USE_COLOR = sys.stdout.isatty()
PLAIN: list = []                    # the same text without colour, for the saved copy
COUNT = {k: 0 for k in COLORS}
PROBLEMS: list = []


def emit(text: str = "", color: str = "") -> None:
    PLAIN.append(text)
    print(f"\033[{color}m{text}\033[0m" if USE_COLOR and color else text)


def section(title: str) -> None:
    emit()
    emit("=" * WIDTH, "1")
    emit(f" {title}", "1")
    emit("=" * WIDTH, "1")


def expect(text: str) -> None:
    for i, line in enumerate(textwrap.wrap(text, WIDTH - 14)):
        emit(("    expect: " if i == 0 else "            ") + line, "2")


def check(level: str, msg: str, why: str = "") -> None:
    COUNT[level] += 1
    if level in ("WARN", "FAIL"):
        PROBLEMS.append((level, msg))
    emit(f"[{level:^4}] {msg}", COLORS[level])
    if why:
        expect(why)


# ------------------------------------------------------------------ helpers

def sh(*cmd: str, timeout: float = 4.0):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except (OSError, subprocess.SubprocessError) as e:
        return 127, str(e)


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def to_int(text, default=0) -> int:
    try:
        return int(str(text).strip(), 0)
    except (TypeError, ValueError):
        return default


def dur(sec: float) -> str:
    sec = int(sec)
    return f"{sec // 3600}h{sec % 3600 // 60:02d}m" if sec >= 3600 else f"{sec // 60}m{sec % 60:02d}s"


PIN_RE = re.compile(r"^\s*(\d+):\s+(\S+)\s+(?:d[hl]\s+)?(\S+)\s*\|\s*(hi|lo)\s*//\s*(.*)$")


def pin(n: int) -> dict:
    """One GPIO's function, pull and level, read with `pinctrl get` (read-only)."""
    rc, out = sh("pinctrl", "get", str(n))
    m = PIN_RE.match(out.splitlines()[0]) if rc == 0 and out else None
    if not m:
        return {"error": True, "raw": out or "pinctrl not found"}
    return {"func": m.group(2), "pull": m.group(3), "level": m.group(4), "note": m.group(5), "raw": out.splitlines()[0]}


KEYS = ("pwm_chip pan_channel tilt_channel pan_center_us tilt_center_us pan_us_per_deg tilt_us_per_deg pan_invert "
        "tilt_invert servo_speed_dps servo_min_us servo_max_us hold_on_close servo_power_gpio").split()
DEFAULTS = {"pwm_chip": "0", "pan_channel": "1", "tilt_channel": "0", "pan_center_us": "1500", "tilt_center_us": "1500",
            "pan_us_per_deg": str(2000 / 180), "tilt_us_per_deg": str(2000 / 180), "pan_invert": "false",
            "tilt_invert": "false", "servo_speed_dps": "30", "servo_min_us": "500", "servo_max_us": "2400",
            "hold_on_close": "true", "servo_power_gpio": "0"}


def load_params(path: Path):
    text = read(path)
    if text is None:
        return None
    out = {}
    for k in KEYS:
        m = re.search(rf"^\s*{k}:\s*([^#\s]+)", text, re.M)
        if m:
            out[k] = m.group(1).strip("\"'")
    return out


def num(p: dict, key: str) -> float:
    v = str(p.get(key, DEFAULTS[key])).lower()
    return 1.0 if v == "true" else 0.0 if v == "false" else float(v)


def reach(center: float, upd: float, invert: bool, lo_deg: float, hi_deg: float, min_us: float, max_us: float):
    """The angles the pulse window allows - the same arithmetic as the driver's _Axis."""
    s = -1.0 if invert else 1.0
    a, b = (min_us - center) / (s * upd), (max_us - center) / (s * upd)
    return min(0.0, max(lo_deg, min(a, b))), max(0.0, min(hi_deg, max(a, b)))


# ------------------------------------------------------------------ 1. system

def sec_system() -> None:
    section("1. SYSTEM AND CODE VERSION")
    model = (read("/proc/device-tree/model") or "").strip("\x00 \n")
    if "Raspberry Pi 4" in model:
        check("OK", f"model: {model}", "Raspberry Pi 4 Model B: PWM chip 0 (pwm_chip: 0 in robot.yaml).")
    elif "Raspberry Pi 5" in model:
        check("INFO", f"model: {model}", "A Pi 5 uses PWM chip 2: robot.yaml needs pwm_chip: 2 and these checks look at pwmchip2.")
    else:
        check("WARN", f"model: {model or 'unknown'} - this does not look like the robot's Pi",
              "Run this on the robot's Pi, over ssh.")
    osr = dict(l.split("=", 1) for l in (read("/etc/os-release") or "").splitlines() if "=" in l)
    check("INFO", f"os: {osr.get('PRETTY_NAME', '?').strip(chr(34))}   kernel: {platform.release()}   python: {platform.python_version()}")
    up = to_int((read("/proc/uptime") or "0").split()[0].split(".")[0])
    check("INFO", f"uptime: {dur(up)}", "A short uptime means the Pi rebooted recently (a power dip, a crash or a manual reboot).")
    user = getpass.getuser()
    if os.geteuid() == 0:
        check("WARN", f"running as root; using {HOME} as the robot user's home",
              "Run WITHOUT sudo, as the user the robot service runs as, so file permissions are tested the way the robot sees them.")
    else:
        groups = sorted(grp.getgrgid(g).gr_name for g in os.getgroups())
        check("INFO", f"user: {user}   groups: {' '.join(groups)}",
              "Pin and PWM access normally needs the 'gpio' group (the default for the first user).")
    rc, branch = sh("git", "-C", str(ROOT), "rev-parse", "--abbrev-ref", "HEAD")
    rc2, last = sh("git", "-C", str(ROOT), "log", "-1", "--format=%h %cd %s", "--date=short")
    rc3, dirty = sh("git", "-C", str(ROOT), "status", "--short", "--untracked-files=no")
    if rc == 0 and rc2 == 0:
        check("INFO", f"repo: {ROOT}  branch {branch}  commit {last[:80]}  uncommitted files: {len(dirty.splitlines())}",
              "The branch and commit you think the robot is running (git pull + restart robot-voice after changes).")
    gim = read(ROOT / "robot_core/sensors/gimbal.py") or ""
    if "Fail-safe" in gim:
        check("OK", "gimbal.py contains the fail-safe (PWM readback, fault latch, OLED codes G03-G08)")
    else:
        check("INFO", "gimbal.py is the original driver (no fail-safe yet)", "Pull the fail-safe change if you want it on the robot.")


# ------------------------------------------------------------------ 2. pi power

FLAGS = {0: ("FAIL", "UNDER-VOLTAGE right now"), 1: ("FAIL", "ARM frequency capped right now"),
         2: ("FAIL", "throttled right now"), 3: ("WARN", "soft temperature limit right now"),
         16: ("WARN", "under-voltage has happened since boot"), 17: ("WARN", "frequency capping has happened since boot"),
         18: ("WARN", "throttling has happened since boot"), 19: ("WARN", "soft temperature limit has happened since boot")}


def sec_power() -> None:
    section("2. THE PI'S OWN SUPPLY (servo current spikes can sag it)")
    raw = read("/sys/devices/platform/soc/soc:firmware/get_throttled")
    if raw is None:
        rc, out = sh("vcgencmd", "get_throttled")
        raw = out.split("=")[-1] if rc == 0 else None
    if raw is None:
        check("WARN", "cannot read the Pi's throttle flags (no sysfs node, no vcgencmd)")
    else:
        val = to_int(raw, -1)
        bits = [b for b in FLAGS if val >= 0 and val & (1 << b)]
        if not bits:
            check("OK", f"get_throttled = {raw}: no under-voltage, no throttling")
        for b in bits:
            check(FLAGS[b][0], f"get_throttled = {raw}: {FLAGS[b][1]}")
        expect("0x0. Under-voltage that appears when the head starts moving means the servos' current or ground return is "
               "disturbing the Pi - the 'power.dip' lines in section 9 show when. Bits 16-19 stay set until the next reboot, "
               "so reboot first if you want a clean reading.")
    t = read("/sys/class/thermal/thermal_zone0/temp")
    if t:
        c = to_int(t) / 1000
        check("OK" if c < 75 else "WARN", f"cpu temperature: {c:.1f} C", "Under about 75 C. Hotter can throttle the Pi.")


# ------------------------------------------------------------------ 3. boot config

def sec_boot() -> None:
    section("3. BOOT CONFIG (/boot/firmware/config.txt)")
    path = next((p for p in CONFIG_TXT if p.exists()), None)
    text = read(path) if path else None
    if text is None:
        check("WARN", "cannot read config.txt", "It lives at /boot/firmware/config.txt on current Pi OS.")
        return
    sect, active = "all", []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = re.match(r"\[(.+)\]$", line)
        if m:
            sect = m.group(1)
            continue
        active.append((n, sect, line))
    emit(f"  active lines of {path} (line number, [section]):")
    for n, s, l in active:
        emit(f"    {n:>3}  [{s}] {l}")
    pwm = [a for a in active if "pwm" in a[2].lower()]
    good = [a for a in pwm if re.match(r"dtoverlay=pwm-2chan\b", a[2]) and all(x in a[2] for x in ("pin=12", "func=4", "pin2=13", "func2=4"))]
    if not good:
        check("FAIL", "no 'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' line",
              "Exactly that line, not commented out, in the [all] or [pi4] section. install_pi.sh adds it; reboot after editing.")
    elif good[0][1] not in ("all", "pi4"):
        check("WARN", f"the PWM overlay sits in section [{good[0][1]}], which does not apply to a Pi 4B", "Move it under [all] or [pi4].")
    elif len(pwm) > 1:
        check("WARN", f"PWM overlay present, but {len(pwm)} lines mention pwm: {[a[2] for a in pwm]}",
              "Exactly one pwm line. A second PWM overlay can take the same pins.")
    else:
        check("OK", "PWM overlay present exactly once", "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4")
    if any("w1-gpio" in a[2] for a in active):
        check("FAIL", "the 1-Wire overlay (w1-gpio) is enabled and claims GPIO4",
              "Not present. It owns GPIO4, the servo-supply sense pin, and can hold it high whatever the servo switch does.")
    other = [a[2] for a in active if a not in good and re.search(r"\b(gpio_?pin|gpiopin|pin2?|gpio)\s*=\s*(4|12|13)\b", a[2])]
    if other:
        check("WARN", f"other lines mention GPIO 4, 12 or 13: {other}", "Nothing else should configure the servo or sense pins.")
    aud = [a[2] for a in active if a[2].startswith("dtparam=audio")]
    mods = read("/proc/modules") or ""
    check("INFO", f"onboard audio: {aud or 'not set'}   snd_bcm2835 loaded: {'snd_bcm2835' in mods}",
          "Low-confidence suspect: on a Pi 4 the analogue audio output uses PWM hardware/clock. The robot's audio is Bluetooth, so "
          "dtparam=audio=off is a cheap way to rule it out.")
    try:
        boot = time.time() - to_int((read("/proc/uptime") or "0").split()[0].split(".")[0])
        if path.stat().st_mtime > boot + 5:
            check("WARN", "config.txt was edited AFTER the last boot", "A reboot is needed for overlay changes to apply.")
    except OSError:
        pass


# ------------------------------------------------------------------ 4. kernel pwm

def sec_pwm(eff: dict) -> None:
    section("4. KERNEL PWM CHANNELS (/sys/class/pwm)")
    chips = sorted(SYS_PWM.glob("pwmchip*")) if SYS_PWM.exists() else []
    if not chips:
        check("FAIL", "no PWM controller in /sys/class/pwm", "pwmchip0 on a Pi 4. Missing means the overlay is not applied (reboot?).")
        return
    want = f"pwmchip{int(num(eff, 'pwm_chip'))}"
    if want not in [c.name for c in chips]:
        check("FAIL", f"robot.yaml says {want} but the controllers are {[c.name for c in chips]}", "pwm_chip: 0 on a Pi 4.")
    lo_us, hi_us = num(eff, "servo_min_us"), num(eff, "servo_max_us")
    for chip in chips:
        chans = sorted(chip.glob("pwm[0-9]*"))
        check("INFO", f"{chip.name}: npwm={read(chip / 'npwm')}  exported channels: {[c.name for c in chans] or 'none'}")
        for ch in chans:
            idx = ch.name[3:]
            who = ""
            if chip.name == want:
                who = ("tilt GPIO12" if idx == str(int(num(eff, "tilt_channel"))) else
                       "pan GPIO13" if idx == str(int(num(eff, "pan_channel"))) else "unused")
            en, per, duty = read(ch / "enable"), to_int(read(ch / "period")), to_int(read(ch / "duty_cycle"))
            pol = read(ch / "polarity")
            desc = (f"{chip.name}/{ch.name} ({who}): enable={en} period={per} ns ({per / 1e6:.1f} ms) "
                    f"duty_cycle={duty} ns ({duty / 1e3:.0f} us, {100 * duty / per if per else 0:.1f}%) polarity={pol}")
            if en != "1":
                check("INFO", desc + "  [exported, not running]")
                continue
            level = "OK"
            if per != PERIOD_NS:
                level = "WARN"
            if not lo_us <= duty / 1e3 <= hi_us:
                level = "FAIL"
            check(level, desc + "  [RUNNING]")
            if who.startswith(("tilt", "pan")):
                axis = who.split()[0]
                s = -1.0 if num(eff, f"{axis}_invert") else 1.0
                ang = (duty / 1e3 - num(eff, f"{axis}_center_us")) / (s * num(eff, f"{axis}_us_per_deg"))
                emit(f"          -> about {ang:+.0f} deg from straight ahead ({axis}: + is {'up' if axis == 'tilt' else 'left'}), "
                     f"if the head is following the pulse")
            if not os.access(ch / "duty_cycle", os.W_OK):
                check("WARN", f"{ch.name}: this user cannot write duty_cycle", "The user running the robot must be able to write it (udev rule / gpio group).")
    expect("While the robot runs the head: both channels exported, enable=1, period=20000000 (20 ms), duty near your centres "
           "(tilt 1,400,000, pan 1,600,000 at rest), always inside 500,000-2,400,000. With the service stopped, a channel "
           "still 'RUNNING' is the held-centre pulse left by the last run (hold_on_close): normal, and it still drives a "
           "powered servo. 'exported, not running' or no channels at all is also fine.")


# ------------------------------------------------------------------ 5. pins

def sec_pins(eff: dict, watch: int) -> None:
    section("5. PIN FUNCTIONS AND THE SERVO-SUPPLY SENSE PIN")
    for n, name in ((12, "tilt"), (13, "pan")):
        p = pin(n)
        if p.get("error"):
            check("WARN", f"cannot read GPIO{n} ({p['raw']})", "Needs the 'pinctrl' tool (sudo apt install raspi-utils) and access to /dev/gpiomem (gpio group).")
            break
        pwm = "PWM" in p["note"].upper()
        check("OK" if pwm else "FAIL", f"GPIO{n} ({name}): {p['raw']}")
    else:
        expect("function a0 with 'GPIOnn = PWM0_CHANn'. ip/op/no instead means the pin is NOT set to PWM (overlay missing or "
               "something else claimed it). The level reads lo with no pulses running (with pulses it flickers); a steady hi "
               "with the head idle means something is driving or shorting the line high.")
    gpio = int(num(eff, "servo_power_gpio"))
    if gpio == 0:
        check("INFO", "servo_power_gpio is 0: no supply sensing configured")
        return
    p = pin(gpio)
    if p.get("error"):
        check("WARN", f"cannot read GPIO{gpio} ({p['raw']})")
        return
    check("INFO", f"sense pin GPIO{gpio}: {p['raw']}")
    if p["pull"] == "pu":
        check("INFO", f"GPIO{gpio} has its internal pull-UP active",
              "The Pi boots with GPIO0-8 pulled up; the robot sets pull-down when it opens the head. With the servo switch OFF and a "
              "pull-up on, the pin can sit near 1 V and read unreliably. Trust 'hi' as ON; judge 'lo' with the robot running.")
    hi = 0
    for _ in range(20):
        hi += pin(gpio).get("level") == "hi"
        time.sleep(0.05)
    if hi == 20:
        check("INFO", f"sense reads HIGH in 20/20 samples -> servo supply sensed ON (or the pin is stuck high)")
    elif hi == 0:
        check("INFO", f"sense reads LOW in 20/20 samples -> servo supply sensed OFF (or the wire/divider is broken)")
    else:
        check("FAIL", f"sense FLICKERS: high {hi}/20 samples", "A steady value. Flicker = loose wire, bad divider or a servo supply collapsing.")
    expect("Run this once with the servo switch OFF (expect lo) and once ON (expect hi). If it ignores the switch, the divider "
           "(10 kOhm from servo +5 V to the pin, 20 kOhm from the pin to GND, shared ground) is wrong or the pin is "
           "physically not pin 7.")
    if watch:
        emit(f"  watching GPIO{gpio} for {watch} s - flip the servo switch OFF, then ON, now ...")
        last, t0 = None, time.time()
        while time.time() - t0 < watch:
            lv = pin(gpio).get("level")
            if lv != last:
                emit(f"    t+{time.time() - t0:5.1f}s  GPIO{gpio} = {lv}")
                last = lv
            time.sleep(0.05)
        expect("One clean change per flip of the switch, nothing in between. Extra changes (bounce) mean a loose connection.")


# ------------------------------------------------------------------ 6. who owns the head

def sec_owners(eff: dict) -> None:
    section("6. WHO OWNS THE HEAD, AND ITS SAVED STATE")
    rc, active = sh("systemctl", "is-active", SERVICE)
    check("INFO", f"service {SERVICE}: {active}")
    rc, show = sh("systemctl", "show", SERVICE, "-p", "NRestarts", "-p", "ActiveEnterTimestamp", "-p", "Result")
    kv = dict(l.split("=", 1) for l in show.splitlines() if "=" in l)
    if kv:
        n = to_int(kv.get("NRestarts"))
        check("WARN" if n else "OK", f"restarts since the service started: {n}   last start: {kv.get('ActiveEnterTimestamp')}   result: {kv.get('Result')}",
              "0. Each automatic restart re-opens the head, so a crash loop repeats the first-pulse moment over and over.")
    mine = {os.getpid(), os.getppid()}
    pat = re.compile(r"voice_node|explorer_node|check_head\.py|check_servo_pulse\.py|robot_core\.sensors\.gimbal|start_robot")
    lines = []
    for d in glob.glob("/proc/[0-9]*"):
        pid = int(d.rsplit("/", 1)[1])
        cmd = (read(d + "/cmdline") or "").replace("\x00", " ").strip()
        if pid not in mine and cmd and pat.search(cmd) and "check_head_diag" not in cmd:
            lines.append(f"{pid} {cmd}")
    for l in lines:
        emit(f"    process: {l[:WIDTH - 15]}")
    voice = [l for l in lines if "voice_node" in l]
    manual = [l for l in lines if re.search(r"check_head\.py|check_servo_pulse\.py|sensors\.gimbal", l)]
    if len(voice) > 1:
        check("WARN", f"{len(voice)} voice_node processes", "At most one. Two robots running = two writers on the head.")
    elif manual:
        check("WARN", "a manual head script is running", "None while the robot service runs: it would fight the service for the PWM.")
    else:
        check("OK" if lines else "INFO", f"{len(lines)} robot process(es) that could drive the head",
              "At most one owner (the service, or one script - never both).")
    holders = []
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.readlink(fd) == LOCK:
                pid = fd.split("/")[2]
                holders.append((pid, (read(f"/proc/{pid}/cmdline") or "").replace("\x00", " ")[:70]))
        except OSError:
            pass
    check("INFO", f"head lock {LOCK}: held by {holders or 'nobody'}",
          "Nobody until the first look command opens the head, then exactly one process. Only this user's processes are visible.")
    state = HOME / ".cache" / "robot-head.json"
    latch = state.with_suffix(".fault")
    if latch.exists():
        check("FAIL", f"fault latched: {read(latch)}", f"No latch file. Fix the cause, then: python -m robot_core.sensors.gimbal reset  ({latch})")
    else:
        check("OK", "no fault latched")
    txt = read(state)
    if txt is None:
        check("INFO", f"no saved head position ({state})", "Normal on a first run. Without it the first pulse is the centre pulse.")
    else:
        try:
            d = json.loads(txt)
            for axis in ("pan", "tilt"):
                us = float(d[f"{axis}_us"])
                c = num(eff, f"{axis}_center_us")
                bad = us < 650 or us > 2300
                check("WARN" if bad else "INFO", f"saved {axis} pulse: {us:.0f} us (centre {c:.0f}, {us - c:+.0f} us)",
                      "Close to your centre (within roughly +-300 us). The next start begins from this pulse; near 500 or 2400 "
                      "means it starts at an end stop." if axis == "pan" else "")
            emit(f"    state file {state} modified {dur(time.time() - state.stat().st_mtime)} ago")
        except (ValueError, KeyError, OSError):
            check("WARN", f"saved head position unreadable: {txt[:60]!r}")


# ------------------------------------------------------------------ 7. config

def sec_config(src, inst, eff: dict) -> None:
    section("7. ROBOT.YAML - SOURCE COPY vs THE COPY THE SERVICE READS")
    if src is None:
        check("WARN", f"cannot read {YAML_SRC}")
        return
    if inst is None:
        check("INFO", "no installed copy found (ros2_ws/install not built here); the service will read whatever it was built with",
              "The voice service reads the INSTALLED copy, not the repo file, so edits need a rebuild (or colcon --symlink-install).")
    emit(f"  {'key':<18}{'repo copy':<14}{'installed copy':<16}")
    diff = []
    for k in KEYS:
        a, b = src.get(k, "-"), (inst or {}).get(k, "-") if inst is not None else "n/a"
        mark = "" if inst is None or a == b else "   <-- DIFFERENT"
        if mark:
            diff.append(k)
        emit(f"  {k:<18}{a:<14}{b:<16}{mark}")
    if diff:
        check("WARN", f"installed robot.yaml differs from the repo copy in: {', '.join(diff)}",
              "Identical. The service uses the installed one: rebuild (colcon build) or use --symlink-install, then restart robot-voice.")
    elif inst is not None:
        check("OK", "installed robot.yaml matches the repo copy for every head key")
    for axis, lo_deg, hi_deg in (("pan", -90.0, 90.0), ("tilt", -45.0, 45.0)):
        lo, hi = reach(num(eff, f"{axis}_center_us"), num(eff, f"{axis}_us_per_deg"), bool(num(eff, f"{axis}_invert")),
                       lo_deg, hi_deg, num(eff, "servo_min_us"), num(eff, "servo_max_us"))
        check("INFO", f"{axis}: reachable {lo:+.0f}..{hi:+.0f} deg (asked {lo_deg:+.0f}..{hi_deg:+.0f}); centre {num(eff, axis + '_center_us'):.0f} us; "
                      f"{num(eff, axis + '_us_per_deg'):.2f} us/deg; invert {bool(num(eff, axis + '_invert'))}")
    expect("Centres roughly 1300-1700 us, about 11.1 us per degree, speed at or below 60 deg/s. A reachable range narrower than "
           "asked (e.g. pan +72 instead of +90) just means the horn is off-centre in its travel.")
    if num(eff, "servo_speed_dps") > 60:
        check("WARN", f"servo_speed_dps is {num(eff, 'servo_speed_dps'):.0f}", "60 or less; faster is a snap.")


# ------------------------------------------------------------------ 8. packages

def sec_packages() -> None:
    section("8. PYTHON PACKAGES THE HEAD NEEDS (in the robot's own environment)")
    py = ROOT / ".venv/bin/python"
    py = str(py) if py.exists() else sys.executable
    code = ("import importlib.metadata as m,sys\nfor n in sys.argv[1:]:\n    try: print(n, m.version(n))\n"
            "    except Exception: print(n, 'MISSING')")
    rc, out = sh(py, "-c", code, "rpi-hardware-pwm", "rpi-lgpio", "RPi.GPIO", "lgpio", timeout=15)
    ver = dict(l.rsplit(" ", 1) for l in out.splitlines() if " " in l)
    emit(f"  interpreter: {py}")
    for k, v in ver.items():
        emit(f"    {k:<18}{v}")
    if ver.get("rpi-hardware-pwm", "MISSING") == "MISSING":
        check("FAIL", "rpi-hardware-pwm is not installed in that environment", "Installed. Without it the head cannot start at all.")
    gpio_ok = ver.get("rpi-lgpio", "MISSING") != "MISSING" or ver.get("RPi.GPIO", "MISSING") != "MISSING"
    if not gpio_ok:
        check("FAIL", "neither rpi-lgpio nor RPi.GPIO is installed", "rpi-lgpio (it provides 'import RPi.GPIO'). Without it the servo-supply sense cannot start.")
    elif ver.get("rpi-lgpio", "MISSING") != "MISSING" and ver.get("RPi.GPIO", "MISSING") != "MISSING":
        check("WARN", "both rpi-lgpio and RPi.GPIO are installed", "Only one; they replace each other's 'RPi' module.")
    else:
        check("OK", "PWM library and GPIO library present")


# ------------------------------------------------------------------ 9. logs

def sec_logs() -> None:
    section("9. RECENT HEAD AND POWER EVENTS FROM THE ROBOT'S LOG (times are UTC)")
    cands = [p for p in (ROOT / "logs.json", HOME / "Robot-Computer-Vision/logs.json", HOME / "Robot-RPi-Core/logs.json") if p.exists()]
    if not cands:
        check("INFO", "no logs.json found")
        return
    path = max(cands, key=lambda p: p.stat().st_mtime)
    with open(path, "rb") as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell() - 1_500_000))
        rows = []
        for line in f.read().decode(errors="replace").splitlines():
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    emit(f"  {path}  (last {len(rows)} readable lines, modified {dur(time.time() - path.stat().st_mtime)} ago)")

    def hit(r):
        s = f"{r.get('evt', '')} {r.get('msg', '')} {r.get('err', '')}".lower()
        return any(k in s for k in ("gimbal", "servo", "head", "power.", "pwm", "gesture.skip")) and "power.shutdown" not in s
    picked = [r for r in rows if hit(r)][-30:]
    for r in picked:
        text = " ".join(str(r.get("msg") or r.get("err") or r.get("cause") or r.get("state") or "").split())
        emit(f"    {str(r.get('ts', ''))[5:19]}  {r.get('lvl', ''):<5} {r.get('evt', ''):<14} {text[:WIDTH - 46]}")
    if not picked:
        emit("    (nothing about the head or power in this window)")
    dips = {}
    for r in rows:
        if r.get("evt") == "power.dip":
            dips[r.get("cause", "?")] = dips.get(r.get("cause", "?"), 0) + 1
    faults = [r for r in rows if "FAULT" in str(r.get("msg", ""))]
    check("WARN" if faults else "OK", f"gimbal FAULT lines in this window: {len(faults)}", "0. Each names what tripped the fail-safe.")
    check("WARN" if dips else "OK", f"power dips in this window: {dips or 0}",
          "0. A cause of 'head servo moving' repeating at the moment of a snap points at servo current disturbing the Pi.")


# ------------------------------------------------------------------ 10. by hand

def sec_manual() -> None:
    section("10. WHAT ONLY A MULTIMETER OR SCOPE CAN TELL YOU (write the numbers down)")
    for l in (
        "Servo supply, across +5 V and GND AT THE SERVO CONNECTOR:",
        "    idle ~5.0 V.  While a servo starts moving it must stay above 4.5 V (use the meter's min-hold).  Below ~4.3 V the",
        "    servo electronics can reset -> snaps and shaking.  Fix: 2 A supply, 470-1000 uF capacitor at the connector, short thick wires.",
        "Ground return, millivolts between the servo's brown wire (at the servo) and a Pi GND pin, WHILE it moves:",
        "    about 0 mV (under ~50 mV).  Hundreds of mV = the servo's current is returning through the Pi's ground wire.",
        "Pull-down resistors, power off, ohms from each signal wire to GND:  about 10 kOhm.  Open = not connected; a few hundred",
        "    ohms = wrong resistor or a short.  README says to ground them at the servo end, not back at the Pi.",
        "Sense divider midpoint (the wire to the sense GPIO), volts to GND:  servo supply ON about 3.0-3.3 V; OFF about 0 V.",
        "    Above 3.4 V = divider wrong (risks the pin).  Compare with section 5.",
        "Signal at the servo, on a scope or logic analyser: a 50 Hz train, 3.3 V high, pulse 0.5-2.4 ms, clean edges.",
        "    A missing, narrow or noisy pulse at the moment of the snap is the answer.",
        "Nothing from the servo +5 V should be connected to the Pi's 5 V pins: two supplies tied together fight.",
    ):
        emit("  " + l)


# ------------------------------------------------------------------ main

def main() -> int:
    ap = argparse.ArgumentParser(description="Read-only diagnostics for the pan/tilt head.")
    ap.add_argument("--watch", type=int, default=0, metavar="SECONDS",
                    help="also watch the servo-supply sense pin for this long while you flip the servo switch")
    ap.add_argument("--no-save", action="store_true", help="do not write ~/robot-diag-<time>.txt")
    args = ap.parse_args()
    emit(f"HEAD DIAGNOSTICS  {dt.datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %Z')}   (read-only: nothing is moved or written)", "1")
    src, inst = load_params(YAML_SRC), load_params(YAML_INST)
    eff = {**DEFAULTS, **(inst if inst is not None else src or {})}        # what the service will use
    sec_system()
    sec_power()
    sec_boot()
    sec_pwm(eff)
    sec_pins(eff, args.watch)
    sec_owners(eff)
    sec_config(src, inst, eff)
    sec_packages()
    sec_logs()
    sec_manual()
    section("SUMMARY")
    emit(f"  OK {COUNT['OK']}   INFO {COUNT['INFO']}   WARN {COUNT['WARN']}   FAIL {COUNT['FAIL']}")
    for level, msg in PROBLEMS:
        emit(f"  [{level}] {msg[:WIDTH - 10]}", COLORS[level])
    if not PROBLEMS:
        emit("  no WARN or FAIL - the problem is then in what only a meter can see (section 10) or in a moment this snapshot missed.")
    if not args.no_save:
        out = HOME / f"robot-diag-{time.strftime('%Y%m%d-%H%M%S')}.txt"
        try:
            out.write_text("\n".join(PLAIN) + "\n")
            emit(f"\nSaved a plain-text copy: {out}   <- paste this back", "1")
        except OSError as e:
            emit(f"\n(could not save a copy: {e})")
    return 1 if COUNT["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
