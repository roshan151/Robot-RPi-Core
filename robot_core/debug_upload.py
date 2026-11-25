"""Debug mode (robot.yaml /voice debug_mode): send the sensor data a task collects
to the Mac, Vision-And-Planning POST /v1/debug/frame, for Monitoring-Service to show.

Built so it can never slow the robot down:
  * the sensors only hand over a reference to what they already hold (offer_*): no
    copy, no encoding and no I/O on their threads;
  * one background thread sends the NEWEST camera frame with the newest ToF grid, at
    most MAX_HZ times a second. Whatever arrives while an upload is in flight replaces
    what was waiting: dropped, never queued;
  * an upload gives up after TIMEOUT_S; while the Mac is unreachable it waits BACKOFF_S
    between tries, and says so once in logs.json;
  * a frame the task already JPEG-encoded for the Mac (offer_jpeg) is sent as it is.
Off (the default, and in every process that never calls configure) each offer_*
returns at once.

Events (offer_event) are different: a task's own record of one step, e.g. each follow
frame with what the Mac answered and what the Pi decided. They are queued, not dropped
(up to EVENT_QUEUE; only the oldest go if the Mac is unreachable that long), and sent
as fast as the link allows, so a debug session has every one of them.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Callable, Optional

log = logging.getLogger(__name__)

MAX_HZ = 2.0                 # uploads a second, at most
TIMEOUT_S = 1.0              # one upload gives up after this
BACKOFF_S = 5.0              # Mac unreachable: wait this long before the next try
JPEG_LONG_EDGE = 640         # frames the task did not encode itself are sent this size...
JPEG_QUALITY = 80            # ...at this quality
TOF_ONLY_AFTER_S = 1.0       # no new camera frame for this long: send ToF grids on their own
EVENT_QUEUE = 120            # events waiting to be sent, at most (the oldest go first)
TOF_LAG_S = 0.1              # a grid describes the scene this long before it arrives (nav/reflex SENSOR_LAG_S)

_on = False
_lock = threading.Lock()
_new = threading.Event()
_stop = threading.Event()
_frame = None                # camera Frame (sensors/camera.py)
_jpeg = None                 # (frame seq, JPEG bytes) the task encoded for the Mac
_tof = None                  # (time.monotonic(), rows in metres / None)
_gimbal = None               # (pan, tilt) the head pointed at most recently (note_gimbal)
_tofs: deque = deque(maxlen=64)      # every ToF grid since the last upload: (time.monotonic(), rows)
_hist: deque = deque(maxlen=150)     # 3 s of (time.monotonic(), pan, tilt, nav pose) from the reflex's 50 Hz tick
_events: deque = deque(maxlen=EVENT_QUEUE)   # (jpeg or None, note dict, rows or None, frame seq or None)
_cfg: dict = {}


def configure(enabled: bool, base_url: str, geometry: Optional[dict] = None,
              extras: Optional[Callable[[], dict]] = None) -> None:
    """Turn debug uploads on (once per process). `geometry`: what the viewer needs to
    lay the ToF grid over the image (camera_hfov_deg, tof_fov_deg). `extras()`: small
    values sent with each upload (pose, pitch), called on the upload thread."""
    global _on
    if not enabled or _on:
        return
    _cfg.update(url=base_url.rstrip("/") + "/v1/debug/frame", geometry=geometry or {},
                extras=extras, session=datetime.now().strftime("%Y%m%d-%H%M%S"))
    _on = True
    threading.Thread(target=_loop, name="debug-upload", daemon=True).start()
    log.info("debug mode: uploading sensor data to %s, session %s", _cfg["url"], _cfg["session"])


def stop() -> None:
    _stop.set()
    _new.set()


def offer_frame(frame) -> None:
    global _frame
    if _on:
        with _lock:
            _frame = frame
        _new.set()


def offer_jpeg(seq: int, data: bytes) -> None:
    global _jpeg
    if _on:
        with _lock:
            _jpeg = (seq, data)


def offer_tof(t: float, rows) -> None:
    global _tof
    if _on:
        with _lock:
            _tof = (t, rows)
            _tofs.append((t, rows))
        _new.set()


def note_gimbal(pan: float, tilt: float, t: Optional[float] = None, pose=None,
                head_still: Optional[bool] = None) -> None:
    """Where the head points now (the nav reflex calls it 50 times a second): sent with every
    upload, so a ToF grid without a camera frame can still be placed (Monitoring-Service --nav).
    With `t` (time.monotonic()) and the nav `pose`, it is also kept for a few seconds, so each ToF
    grid and camera frame goes out with the head angles, pose, and settled state of its moment (_at)."""
    global _gimbal
    if _on:
        _gimbal = (pan, tilt)
        if t is not None and pose is not None:
            _hist.append((t, pan, tilt, tuple(pose), head_still))


def _at(t: float):
    """(pan, tilt, pose, head_still) at time.monotonic() t, between the two reflex ticks around it;
    None when t is outside the history (the reflex not running, or the upload fell behind by seconds)."""
    with _lock:
        h = list(_hist)
    if not h or t < h[0][0] or t > h[-1][0] + 0.05:
        return None
    for (t0, p0, k0, q0, s0), (t1, p1, k1, q1, s1) in zip(h, h[1:] + [h[-1]]):
        if t0 <= t <= t1 or t1 == t0:
            w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            dth = math.atan2(math.sin(q1[2] - q0[2]), math.cos(q1[2] - q0[2]))
            pose = (q0[0] + w * (q1[0] - q0[0]), q0[1] + w * (q1[1] - q0[1]),
                    math.atan2(math.sin(q0[2] + w * dth), math.cos(q0[2] + w * dth)))
            still = s0 if s1 is None else (s1 if s0 is None else bool(s0 and s1))
            return p0 + w * (p1 - p0), k0 + w * (k1 - k0), pose, still
    return None


def offer_event(jpeg: Optional[bytes], note: dict, rows=None, seq: Optional[int] = None) -> None:
    """Queue one step's record (never dropped while there is room): its JPEG, a small dict
    of what happened (`note`), the ToF grid it used, and its camera frame seq."""
    if _on:
        with _lock:
            _events.append((jpeg, note, rows, seq))
        _new.set()


def _real(pan_tilt):
    """[pan, REAL tilt] rounded: the tilt as degrees above level (the head looks
    geometry head_tilt_offset_deg up at gimbal tilt 0), as follow and the nav stream send it."""
    if not pan_tilt:
        return None
    off = float(_cfg.get("geometry", {}).get("head_tilt_offset_deg", 0.0))
    return [round(pan_tilt[0], 1), round(pan_tilt[1] + off, 1)]


def _encode(img) -> bytes:
    import cv2
    h, w = img.shape[:2]
    k = JPEG_LONG_EDGE / max(h, w)
    if k < 1.0:
        img = cv2.resize(img, (round(w * k), round(h * k)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    return buf.tobytes()


def _rows(rows):
    return [[None if v is None else round(float(v), 3) for v in row] for row in rows]


def _loop() -> None:
    import requests
    s = requests.Session()
    n, sent_seq, sent_tof_t, frame_t, failing = 0, None, None, 0.0, False
    while not _stop.is_set():
        _new.wait(0.5)
        _new.clear()
        while _events and not _stop.is_set():          # every event, in order, before the sampled frames
            with _lock:
                jpeg_e, note, rows, seq = _events[0]
            n += 1
            meta = {"session": _cfg["session"], "n": n, "kind": "event",
                    "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    "geometry": _cfg["geometry"], "note": note, "frame": {"seq": seq} if seq else None,
                    "gimbal": _real(_gimbal),
                    "tof": {"age_ms": 0, "rows": _rows(rows)} if rows else None}
            try:
                if _cfg["extras"]:
                    meta["extras"] = _cfg["extras"]()
                files = {"image": ("frame.jpg", jpeg_e, "image/jpeg")} if jpeg_e else None
                r = s.post(_cfg["url"], data={"meta": json.dumps(meta, default=str)}, files=files,
                           timeout=TIMEOUT_S * 2)
                r.raise_for_status()
                with _lock:
                    _events.popleft()
                if seq:
                    sent_seq = seq                         # the sampled path need not send this frame again
                failing = False
            except Exception as e:                         # noqa: BLE001
                if not failing:
                    log.warning("debug mode: event upload failed (%s); keeping up to %d", type(e).__name__, EVENT_QUEUE)
                failing = True
                _stop.wait(BACKOFF_S)
                break
        t0 = time.monotonic()
        with _lock:
            frame, jpeg, tof = _frame, _jpeg, _tof
            grids = [g for g in _tofs if sent_tof_t is None or g[0] > sent_tof_t]
        new_frame = frame is not None and frame.seq != sent_seq
        new_tof = tof is not None and tof[0] != sent_tof_t
        if new_frame:
            frame_t = t0
        elif not new_tof or t0 - frame_t < TOF_ONLY_AFTER_S:
            continue                                    # nothing new, or wait to pair it with an image
        n += 1
        meta = {"session": _cfg["session"], "n": n,
                "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                "geometry": _cfg["geometry"], "frame": None, "tof": None,
                "gimbal": _real(_gimbal)}
        files = {}
        try:
            if new_frame:
                data = jpeg[1] if jpeg and jpeg[0] == frame.seq else _encode(frame.main)
                h, w = frame.main.shape[:2]
                meta["frame"] = {"seq": frame.seq, "pan_deg": round(frame.pan_deg, 1),
                                 "tilt_deg": _real((frame.pan_deg, frame.tilt_deg))[1], "settled": frame.settled,
                                 "w": w, "h": h}
                files["image"] = ("frame.jpg", data, "image/jpeg")
            if tof is not None:
                ref = frame.t_capture if new_frame else t0
                meta["tof"] = {"age_ms": round((ref - tof[0]) * 1000), "rows": _rows(tof[1])}
            # Every grid since the last upload, each with its own time (epoch s) and the head angles and
            # nav pose of the moment it describes: what maps are built from. "tof" above (newest only,
            # with the commanded head angle in "gimbal") stays for the viewer.
            wall = time.time() - time.monotonic()
            seq = []
            for tg, rows in grids:
                at = _at(tg - TOF_LAG_S)
                seq.append({"t": round(wall + tg, 3), "rows": _rows(rows),
                            "pan_deg": round(at[0], 2) if at else None,
                            "tilt_deg": _real((0.0, at[1]))[1] if at else None,
                            "pose": [round(v, 4) for v in at[2]] if at else None,
                            "head_still": at[3] if at else None})
            if seq:
                meta["tof_seq"] = seq
            if new_frame:
                at = _at(frame.t_capture)
                meta["frame"].update(t=round(wall + frame.t_capture, 3),
                                     pose=[round(v, 4) for v in at[2]] if at else None)
            if _cfg["extras"]:
                meta["extras"] = _cfg["extras"]()
            r = s.post(_cfg["url"], data={"meta": json.dumps(meta, default=str)}, files=files or None,
                       timeout=TIMEOUT_S)
            r.raise_for_status()
            if failing:
                log.info("debug mode: uploads to the Mac work again")
            failing = False
        except Exception as e:                          # noqa: BLE001 — never let debug take the robot down
            if not failing:
                log.warning("debug mode: upload failed (%s: %s); retrying every %.0f s",
                            type(e).__name__, str(e)[:120], BACKOFF_S)
            failing = True
            _stop.wait(BACKOFF_S)
            continue
        finally:
            if new_frame:
                sent_seq = frame.seq
            if tof is not None:
                sent_tof_t = max([tof[0]] + [g[0] for g in grids])
        _stop.wait(max(0.0, 1.0 / MAX_HZ - (time.monotonic() - t0)))
