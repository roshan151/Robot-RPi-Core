"""The Pi side of Monitoring-Service/calibrate.py. Not run by hand: the Mac script
starts it over SSH (robot-voice stopped first: it owns the serial port, head and camera)
and sends it one command per line on stdin; each answer is one JSON line on stdout:

    {"cmd": "pan_sweep", "args": {}}   ->   {"progress": "pan +12"} ...  {"ok": true, "result": {...}}

Hardware is opened when first needed and kept open until "quit" (or the SSH session
ends): the head is parked and the motors stopped on the way out. Readings and fits
only; the Mac script does the talking to the person. Angles are degrees, ranges metres.
"""

from __future__ import annotations

import base64
import json
import re
import socket
import statistics
import subprocess
import sys
import time
from pathlib import Path

import yaml

from robot_core import calibrate, settings

_OUT = sys.stdout
sys.stdout = sys.stderr                  # anything else that prints must not break the JSON lines
REPO = Path(__file__).resolve().parent.parent
FIRMWARE = REPO / "firmware/drivetrain/drivetrain.ino"


def say(**msg) -> None:
    _OUT.write(json.dumps(msg, default=lambda o: round(o, 4) if isinstance(o, float) else str(o)) + "\n")
    _OUT.flush()


def progress(text: str) -> None:
    say(progress=text)


class Rig:
    def __init__(self) -> None:
        y = yaml.safe_load(Path(settings.ROBOT_YAML).read_text())
        self.ex = y["/explorer"]["ros__parameters"]
        self.dt = y["/drivetrain"]["ros__parameters"]
        self.disp = y.get("/display", {}).get("ros__parameters", {})
        # Moves must use the robot's own calibration (the drivetrain node takes it from robot.yaml).
        settings.TICKS_PER_CM = float(self.dt.get("ticks_per_cm", settings.TICKS_PER_CM))
        settings.TICKS_PER_DEGREE = float(self.dt.get("ticks_per_degree", settings.TICKS_PER_DEGREE))
        self.offset = float(self.ex.get("head_tilt_offset_deg", 0.0))
        self._drive = self._imu = self._tof = self._gimbal = None
        self.totals = None

    # ---------------------------------------------------------------- hardware, opened on demand

    def drive(self):
        if self._drive is None:
            from robot_core.drivetrain.client import SerialDrivetrain
            progress("opening the Arduino (it resets: ~3 s)")
            self._drive = SerialDrivetrain(port=self.dt.get("serial_port", settings.SERIAL_PORT),
                                           baud=int(self.dt.get("baud", settings.BAUD_RATE)))
            self._drive.add_odometry_listener(lambda l, r, ms: setattr(self, "totals", (l, r)))
            t0 = time.monotonic()
            while self.totals is None and time.monotonic() - t0 < 6.0:
                time.sleep(0.1)
        return self._drive

    def imu(self):
        if self._imu is None and self.dt.get("imu_fitted", False):
            from robot_core.sensors.imu import Imu
            self._imu = Imu(bus=int(self.dt.get("imu_bus", 1)), axes=list(self.dt.get("imu_axes", ["x", "y", "z"])),
                            gyro_addr=int(self.dt.get("imu_gyro_addr", 0x69)))
            progress("zeroing the gyro: keep the robot still")
            self._imu.wait_calibrated(6.0)
        return self._imu

    def tof(self):
        if self._tof is None:
            from robot_core.sensors import tof8x8
            raw = dict(self.ex, tof_range_scale=1.0, tof_range_offset_m=0.0)   # calibrate on raw readings
            self._tof = tof8x8.from_params(raw)
        return self._tof

    def gimbal(self):
        if self._gimbal is None:
            from robot_core.sensors.gimbal import from_params
            self._gimbal = from_params(self.ex, tilt_min=-45.0)
        return self._gimbal

    def close(self) -> None:
        for name, fn in (("drive", lambda d: d.stop()), ("drive", lambda d: d.close()),
                         ("gimbal", lambda g: g.close()), ("tof", lambda t: t.close()), ("imu", lambda i: i.close())):
            obj = getattr(self, "_" + name)
            if obj is not None:
                try:
                    fn(obj)
                except Exception:                    # noqa: BLE001
                    pass

    # ---------------------------------------------------------------- helpers

    def look(self, pan: float, tilt_true: float) -> None:
        g = self.gimbal()
        g.move_to(pan, tilt_true - self.offset, wait=True)
        g.wait_settled(0.4)

    def grid(self, n: int = 5):
        """Per-zone median of the next n grids (None where under half of them had a reading)."""
        tof, t, grids = self.tof(), time.monotonic(), []
        while len(grids) < n and time.monotonic() - t < 3.0 * n:
            g = tof.grid(newer_than=t, timeout=2.0)
            if g is None:
                break
            t = g.t
            grids.append(g.rows)
        if not grids:
            raise RuntimeError("no ToF grid: wiring, I2C address, tof_fitted (see README 'IMU and ToF sensors')")
        out = []
        for r in range(len(grids[0])):
            row = []
            for c in range(len(grids[0][0])):
                v = [g[r][c] for g in grids if g[r][c] is not None]
                row.append(round(statistics.median(v), 4) if len(v) * 2 >= len(grids) else None)
            out.append(row)
        return out

    def centre(self, n: int = 5):
        return calibrate.centre_zone(self.grid(n))

    def yaw(self):
        imu = self.imu()
        s = imu.state() if imu else None
        return None if s is None else s.yaw_deg

    def physical(self, dl: int, dr: int) -> dict:
        """Raw channel deltas -> physical left / right (robot.yaml odom_swap_sides)."""
        swap = bool(self.dt.get("odom_swap_sides", True))
        return {"left": dr if swap else dl, "right": dl if swap else dr}

    # ---------------------------------------------------------------- commands

    def cmd_hello(self) -> dict:
        m = re.search(r"#define\s+REVERSE_TRIM_PCT\s+(-?\d+)", FIRMWARE.read_text()) if FIRMWARE.exists() else None
        keys_ex = ("pan_center_us", "tilt_center_us", "pan_us_per_deg", "tilt_us_per_deg", "pan_invert", "tilt_invert",
                   "head_height_m", "head_tilt_offset_deg", "hfov_deg", "tof_range_scale", "tof_range_offset_m",
                   "tof_tilt", "tof_flip_rows", "tof_flip_cols", "tof_perpendicular", "tof_fitted", "head_backlash_deg")
        keys_dt = ("ticks_per_cm", "ticks_per_degree", "odom_swap_sides", "imu_fitted", "imu_axes",
                   "tfluna_floor_cm", "drop_guard")
        return {"host": socket.gethostname(), "repo": str(REPO),
                "explorer": {k: self.ex.get(k) for k in keys_ex},
                "drivetrain": {k: self.dt.get(k) for k in keys_dt},
                "firmware": {"REVERSE_TRIM_PCT": int(m.group(1)) if m else None}}

    def cmd_diag(self) -> dict:
        out = {}

        def check(name, fn):
            try:
                ok, detail = fn()
            except Exception as e:                   # noqa: BLE001
                ok, detail = False, f"{type(e).__name__}: {e}"
            out[name] = {"ok": bool(ok), "detail": str(detail)}

        def throttled():
            v = int(subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True,
                                   timeout=5).stdout.split("=")[-1], 16)
            now, past = bool(v & 0x1), bool(v & 0x10000)
            return not now, f"0x{v:x}" + (" UNDER-VOLTAGE NOW" if now else "") + (" (under-voltage since boot)" if past else "")
        check("power", throttled)
        check("robot-voice stopped", lambda: (subprocess.run(["systemctl", "is-active", "robot-voice"],
                                                              capture_output=True, text=True).stdout.strip() != "active",
                                               "it must be stopped while calibrating"))
        port = self.dt.get("serial_port", settings.SERIAL_PORT)
        check("serial port", lambda: (Path(port).exists(), port))

        def i2c():
            txt = subprocess.run(["i2cdetect", "-y", str(int(self.ex.get("tof_bus", 1)))], capture_output=True,
                                 text=True, timeout=10).stdout
            want = {"ToF": f"{int(self.ex.get('tof_addr', 0x33)):02x}", "IMU": f"{int(self.dt.get('imu_gyro_addr', 0x69)):02x}"}
            found = {k: v in txt.split() for k, v in want.items()}
            return all(found.values()), ", ".join(f"{k} 0x{want[k]} {'found' if f else 'MISSING'}" for k, f in found.items())
        check("I2C devices", i2c)

        def firmware():
            d = self.drive()
            b = d._bridge.firmware_build or "no boot line"
            return ("-vel" in b), b
        check("Arduino firmware", firmware)

        def tfluna():
            d = self.drive()
            time.sleep(0.5)
            cm, t = d.tfluna()
            return (t > 0 and cm is not None), (f"{cm} cm" if t > 0 else "no reading (firmware v7+ and wiring)")
        check("TF-Luna", tfluna)

        def servos():
            g = self.gimbal()
            return g.powered() and not g.fault, (g.fault or ("servo supply on" if g.powered() else "servo supply OFF"))
        check("head servos", servos)

        def tof():
            c = self.centre(3)
            return c is not None, f"centre {c:.2f} m" if c else "grid but no centre reading"
        check("ToF", tof)

        def imu():
            i = self.imu()
            s = i.state() if i else None
            return s is not None, (f"pitch {s.pitch_deg:+.1f} roll {s.roll_deg:+.1f}" if s else "imu_fitted false or no data")
        check("IMU", imu)

        def camera():
            from robot_core.sensors.camera import Camera
            cam = Camera(main_size=tuple(self.ex.get("face_camera_size", (640, 480))),
                         inverted=bool(self.ex.get("camera_inverted", True)),
                         backend=self.ex.get("camera_backend", "auto"), index=int(self.ex.get("camera_index", 0)))
            try:
                f = cam.capture()
                return True, f"{f.main.shape[1]}x{f.main.shape[0]}"
            finally:
                cam.close()
        check("camera", camera)
        return out

    def cmd_snapshot(self) -> dict:
        """A camera frame (JPEG, base64) and the ToF grid with the head level: for the overlay."""
        from robot_core.sensors.camera import Camera, jpeg
        self.look(0.0, 0.0)
        cam = Camera(main_size=tuple(self.ex.get("face_camera_size", (640, 480))),
                     inverted=bool(self.ex.get("camera_inverted", True)),
                     backend=self.ex.get("camera_backend", "auto"), index=int(self.ex.get("camera_index", 0)))
        try:
            f = cam.capture()
            img = jpeg(f.main, 640)
        finally:
            cam.close()
        return {"jpeg": base64.b64encode(img).decode(), "rows": self.grid(3),
                "geometry": {"camera_hfov_deg": float(self.ex.get("hfov_deg", 66.0)), "tof_fov_deg": 60.0}}

    def cmd_level_grid(self) -> dict:
        self.look(0.0, 0.0)
        return {"rows": self.grid(5)}

    def cmd_watch_grid(self, seconds: float = 4.0) -> dict:
        """The nearest reading each zone saw over `seconds` (a hand passing through)."""
        tof, t0, t, near = self.tof(), time.monotonic(), time.monotonic(), None
        while time.monotonic() - t0 < seconds:
            g = tof.grid(newer_than=t, timeout=1.0)
            if g is None:
                continue
            t = g.t
            near = [[v for v in row] for row in g.rows] if near is None else \
                [[min(x for x in (a, b) if x is not None) if (a is not None or b is not None) else None
                  for a, b in zip(ra, rb)] for ra, rb in zip(near, g.rows)]
        return {"rows": near}

    def cmd_wall(self) -> dict:
        """Head level and straight: centre range to a wall, and corners/centre (tof_perpendicular)."""
        self.look(0.0, 0.0)
        rows = self.grid(8)
        c = calibrate.centre_zone(rows)
        corners = [rows[r][k] for r in (0, len(rows) - 1) for k in (0, len(rows) - 1)]
        ratio = statistics.mean(corners) / c if c and None not in corners else None
        return {"centre_m": c, "corner_ratio": None if ratio is None else round(ratio, 3)}

    def cmd_pan_sweep(self) -> dict:
        samples = []
        for a in range(-30, 31, 3):
            self.look(float(a), 0.0)
            samples.append((a, self.centre(4)))
            progress(f"pan {a:+d}: {samples[-1][1]}")
        p0, d = calibrate.pan_zero([(a, r) for a, r in samples if r])
        cur = float(self.ex.get("pan_center_us", 1500.0))
        us = calibrate.centre_change(p0, float(self.ex.get("pan_us_per_deg", 2000 / 180)), bool(self.ex.get("pan_invert")))
        self.look(0.0, 0.0)
        return {"pan_zero_deg": round(p0, 2), "wall_m": round(d, 3), "samples": samples,
                "pan_center_us": round(cur + us)}

    def cmd_tilt_sweep(self) -> dict:
        samples = []
        for cmd in range(-42, -13, 3):                      # commanded; true = commanded + head_tilt_offset_deg
            true = cmd + self.offset
            self.look(0.0, true)
            samples.append((true, self.centre(4)))
            progress(f"tilt {true:+.0f}: {samples[-1][1]}")
        e, h, spread = calibrate.tilt_and_height([(t, r) for t, r in samples if r])
        cur = float(self.ex.get("tilt_center_us", 1500.0))
        us = calibrate.centre_change(-e, float(self.ex.get("tilt_us_per_deg", 2000 / 180)), bool(self.ex.get("tilt_invert")))
        self.look(0.0, 0.0)
        return {"tilt_error_deg": round(e, 2), "head_height_m": round(h, 3), "height_spread_m": round(spread, 3),
                "samples": samples, "tilt_center_us": round(cur + us)}

    def cmd_encoders_watch(self, seconds: float = 5.0) -> dict:
        self.drive()
        if self.totals is None:
            raise RuntimeError("no encoder telemetry (firmware v5+?)")
        a = self.totals
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            time.sleep(1.0)
            progress(f"{max(0, round(seconds - (time.monotonic() - t0)))} s")
        b = self.totals
        return self.physical(b[0] - a[0], b[1] - a[1])

    def cmd_imu_read(self, seconds: float = 2.0) -> dict:
        imu = self.imu()
        if imu is None:
            raise RuntimeError("imu_fitted is false in robot.yaml")
        ps, rs, ys, t0 = [], [], [], time.monotonic()
        while time.monotonic() - t0 < seconds:
            s = imu.state()
            if s:
                ps.append(s.pitch_deg)
                rs.append(s.roll_deg)
                ys.append(s.yaw_deg)
            time.sleep(0.05)
        return {"pitch_deg": round(statistics.mean(ps), 2), "roll_deg": round(statistics.mean(rs), 2),
                "yaw_start_deg": round(ys[0], 2), "yaw_end_deg": round(ys[-1], 2)}   # yaw: + is left (CCW)

    def cmd_imu_drift(self, seconds: float = 60.0) -> dict:
        y0, t0 = self.yaw(), time.monotonic()
        if y0 is None:
            raise RuntimeError("imu_fitted is false in robot.yaml")
        while time.monotonic() - t0 < seconds:
            time.sleep(5.0)
            progress(f"yaw moved {self.yaw() - y0:+.2f} deg, {max(0, round(seconds - (time.monotonic() - t0)))} s left")
        return {"yaw_drift_deg": round(self.yaw() - y0, 2), "seconds": seconds}

    def cmd_move(self, op: str, value: float) -> dict:
        d = self.drive()
        fn = {"straight": d.straight_m, "reverse": d.reverse_m, "left": d.left, "right": d.right}[op]
        y0, a, t0 = self.yaw(), self.totals, time.monotonic()
        try:
            fn(float(value))
        finally:
            d.stop()
        time.sleep(0.6)                                    # telemetry and the gyro settle
        b, y1 = self.totals, self.yaw()
        out = {"op": op, "value": value, "seconds": round(time.monotonic() - t0, 2),
               "wheels": self.physical(b[0] - a[0], b[1] - a[1]) if a and b else None,
               "yaw_deg": None if y0 is None or y1 is None else round(y1 - y0, 2)}
        if op in ("left", "right") and out["wheels"]:
            w = out["wheels"]
            fwd, bwd = (w["right"], w["left"]) if op == "left" else (w["left"], w["right"])
            out["fwd_bwd_ratio"] = round(abs(fwd) / abs(bwd), 4) if bwd else None
        return out

    def cmd_tfluna(self, seconds: float = 3.0) -> dict:
        d = self.drive()
        vals, none, t0, last = [], 0, time.monotonic(), 0.0
        while time.monotonic() - t0 < seconds:
            cm, t = d.tfluna()
            if t > last:
                last = t
                if cm is None:
                    none += 1
                else:
                    vals.append(cm)
            time.sleep(0.05)
        if not vals:
            raise RuntimeError("no TF-Luna readings: firmware v7+ flashed? wiring (README 'TF-Luna on the Arduino')?")
        return {"median_cm": statistics.median(vals), "spread_cm": max(vals) - min(vals), "readings": len(vals),
                "missed": none}

    def cmd_buzzer(self) -> dict:
        from robot_core import buzzer
        pin = int(self.disp.get("buzzer_pin", 26))
        if not pin:
            return {"played": False, "detail": "buzzer_pin is 0"}
        tone = buzzer.gpio_tone(pin)
        for _, tune in list(buzzer.TUNES.items())[:2]:
            for hz, s in tune:
                tone(hz)
                time.sleep(s)
            tone(0)
            time.sleep(0.5)
        return {"played": True, "pin": pin}

    def cmd_oled(self) -> dict:
        from robot_core import status
        from robot_core.oled import Oled
        board = status.Board()
        oled = Oled(board)
        if not oled.start():
            return {"shown": False, "detail": "OLED not found (wiring, SPI on, pip install '.[oled]')"}
        board.apply({"k": "task", "v": "calibrate"})
        board.apply({"k": "detail", "v": "OLED check: can you read this?"})
        time.sleep(6)
        oled.stop()
        return {"shown": True}

    def cmd_head_moves(self) -> dict:
        """Left, right, up, down and back, slowly: the person watches the directions."""
        for name, pan, tilt in (("left", 45, 0), ("right", -45, 0), ("up", 0, 25), ("down", 0, -25), ("centre", 0, 0)):
            progress(name)
            self.look(float(pan), float(tilt))
            time.sleep(0.8)
        return {"done": True}

    def cmd_fit(self, kind: str, **data) -> dict:
        if kind == "drive":
            return calibrate.drive_fit(data["d0_m"], data["d_end_m"], data["readings"], data["moved_cmd_m"],
                                       float(self.dt.get("ticks_per_cm", settings.TICKS_PER_CM)))
        if kind == "turn":
            return calibrate.turn_fit(data["commanded"], data["measured"],
                                      float(self.dt.get("ticks_per_degree", settings.TICKS_PER_DEGREE)))
        if kind == "trim":
            return calibrate.reverse_trim(data["ratios"], data["current"])
        raise ValueError(f"unknown fit {kind}")


def main() -> None:
    rig = Rig()
    say(ready=True)
    try:
        for line in sys.stdin:
            if not line.strip():
                continue
            try:
                msg = json.loads(line)
                if msg.get("cmd") == "quit":
                    say(ok=True, result={"bye": True})
                    break
                fn = getattr(rig, "cmd_" + msg["cmd"])
                say(ok=True, result=fn(**(msg.get("args") or {})))
            except Exception as e:                       # noqa: BLE001 — answer, don't die: the Mac decides
                try:
                    if rig._drive is not None:
                        rig._drive.stop()
                except Exception:                        # noqa: BLE001
                    pass
                say(ok=False, error=f"{type(e).__name__}: {e}")
    finally:
        rig.close()


if __name__ == "__main__":
    main()
