"""Debug mode (robot.yaml /voice debug_mode): send the sensor data a task collects
to the Mac, Vision-Microservice POST /v1/debug/frame, for Monitoring-Service to show.

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
"""

from __future__ import annotations

import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Optional

log = logging.getLogger(__name__)

MAX_HZ = 2.0                 # uploads a second, at most
TIMEOUT_S = 1.0              # one upload gives up after this
BACKOFF_S = 5.0              # Mac unreachable: wait this long before the next try
JPEG_LONG_EDGE = 640         # frames the task did not encode itself are sent this size...
JPEG_QUALITY = 80            # ...at this quality
TOF_ONLY_AFTER_S = 1.0       # no new camera frame for this long: send ToF grids on their own

_on = False
_lock = threading.Lock()
_new = threading.Event()
_stop = threading.Event()
_frame = None                # camera Frame (sensors/camera.py)
_jpeg = None                 # (frame seq, JPEG bytes) the task encoded for the Mac
_tof = None                  # (time.monotonic(), rows in metres / None)
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
        _new.set()


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
        t0 = time.monotonic()
        with _lock:
            frame, jpeg, tof = _frame, _jpeg, _tof
        new_frame = frame is not None and frame.seq != sent_seq
        new_tof = tof is not None and tof[0] != sent_tof_t
        if new_frame:
            frame_t = t0
        elif not new_tof or t0 - frame_t < TOF_ONLY_AFTER_S:
            continue                                    # nothing new, or wait to pair it with an image
        n += 1
        meta = {"session": _cfg["session"], "n": n,
                "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                "geometry": _cfg["geometry"], "frame": None, "tof": None}
        files = {}
        try:
            if new_frame:
                data = jpeg[1] if jpeg and jpeg[0] == frame.seq else _encode(frame.main)
                h, w = frame.main.shape[:2]
                meta["frame"] = {"seq": frame.seq, "pan_deg": round(frame.pan_deg, 1),
                                 "tilt_deg": round(frame.tilt_deg, 1), "settled": frame.settled,
                                 "w": w, "h": h}
                files["image"] = ("frame.jpg", data, "image/jpeg")
            if tof is not None:
                ref = frame.t_capture if new_frame else t0
                meta["tof"] = {"age_ms": round((ref - tof[0]) * 1000), "rows": _rows(tof[1])}
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
                sent_tof_t = tof[0]
        _stop.wait(max(0.0, 1.0 / MAX_HZ - (time.monotonic() - t0)))
