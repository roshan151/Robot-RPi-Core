"""Debug uploads: off means nothing happens; on, the newest frame goes with the newest
ToF grid, at most MAX_HZ a second, and a burst is dropped rather than queued."""

import importlib
import json
import time
import types

import numpy as np
import requests


class _Session:
    posts: list = []

    def post(self, url, data=None, files=None, timeout=None):
        _Session.posts.append((url, json.loads(data["meta"]), files, timeout))
        return types.SimpleNamespace(raise_for_status=lambda: None)


def _fresh(monkeypatch):
    from robot_core import debug_upload
    debug_upload = importlib.reload(debug_upload)
    _Session.posts = []
    monkeypatch.setattr(requests, "Session", _Session)
    return debug_upload


def _frame(seq):
    return types.SimpleNamespace(seq=seq, t_capture=time.monotonic(), pan_deg=0.0, tilt_deg=10.0,
                                 settled=True, main=np.zeros((48, 64, 3), np.uint8))


def test_off_does_nothing(monkeypatch):
    du = _fresh(monkeypatch)
    du.offer_frame(_frame(1))
    du.offer_tof(time.monotonic(), [[1.0] * 8] * 8)
    time.sleep(0.3)
    assert _Session.posts == [] and du._frame is None


def test_newest_only_at_most_max_hz_and_paired_with_the_tof(monkeypatch):
    du = _fresh(monkeypatch)
    du.configure(True, "http://mac:8080/", geometry={"camera_hfov_deg": 66.0})
    try:
        for seq in range(1, 41):                       # a 20 Hz burst for 2 s
            du.offer_tof(time.monotonic(), [[seq / 10] * 8] * 8)
            du.offer_frame(_frame(seq))
            if seq == 40:
                du.offer_jpeg(40, b"already-encoded")
            time.sleep(0.05)
        time.sleep(0.6)
    finally:
        du.stop()
    posts = _Session.posts
    assert 3 <= len(posts) <= 6, len(posts)            # ~2 a second, never 40
    url, meta, files, timeout = posts[-1]
    assert url == "http://mac:8080/v1/debug/frame" and timeout == du.TIMEOUT_S
    assert meta["frame"]["seq"] == 40 and files["image"][1] == b"already-encoded"
    assert meta["tof"]["rows"][0][0] == 4.0 and meta["geometry"]["camera_hfov_deg"] == 66.0
    assert [m["n"] for _, m, _, _ in posts] == list(range(1, len(posts) + 1))
