"""The LK tracker against synthetic frames: a textured patch that moves."""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robot_core.perception.tracker import TrackManager, TrackerConfig  # noqa: E402

W, H = 320, 180


def scene(dx: int, dy: int = 0, *, patch=True) -> np.ndarray:
    rng = np.random.default_rng(0)
    img = np.full((H, W), 90, np.uint8)
    if patch:
        tex = rng.integers(0, 255, (60, 40), dtype=np.uint8)
        tex = cv2.GaussianBlur(tex, (3, 3), 0)
        x, y = 100 + dx, 60 + dy
        img[y:y + 60, x:x + 40] = tex
    return img


def test_track_follows_a_moving_patch():
    tm = TrackManager(W, H)
    tm.step(scene(0), 1, 0.0)
    (tr,) = tm.anchor([("person", 0.9, (100, 60, 140, 120))], 1, 0.0)
    for i in range(1, 11):
        tm.step(scene(3 * i, i), 1 + i, 0.07 * i)
    assert not tr.lost
    cx, cy = tr.center
    assert cx == pytest.approx(120 + 30, abs=3)
    assert cy == pytest.approx(90 + 10, abs=3)


def test_track_is_lost_honestly_when_the_texture_vanishes():
    tm = TrackManager(W, H)
    tm.step(scene(0), 1, 0.0)
    (tr,) = tm.anchor([("person", 0.9, (100, 60, 140, 120))], 1, 0.0)
    tm.step(scene(0, patch=False), 2, 0.07)
    assert tr.lost


def test_reanchor_applies_the_difference_not_the_stale_box():
    """A detection for an old seq corrects drift without teleporting."""
    tm = TrackManager(W, H)
    tm.step(scene(0), 1, 0.0)
    (tr,) = tm.anchor([("person", 0.9, (100, 60, 140, 120))], 1, 0.0)
    for i in range(1, 6):
        tm.step(scene(4 * i), 1 + i, 0.07 * i)       # now at x ~ 120
    before = tr.center[0]
    # Detection for seq 3 (patch was at +8) that says the box was 2 px further right.
    tm.anchor([("person", 0.95, (110, 60, 150, 120))], 3, 1.0)
    assert tr.center[0] == pytest.approx(before + 2, abs=1.0)
    assert tr.last_anchor_seq == 3


def test_unmatched_tracks_go_lost_then_die():
    cfg = TrackerConfig(max_misses=2, reacquire_s=1.0)
    tm = TrackManager(W, H, cfg)
    tm.step(scene(0), 1, 0.0)
    (tr,) = tm.anchor([("chair", 0.8, (100, 60, 140, 120))], 1, 0.0)
    for k in range(3):
        tm.anchor([], 2 + k, 0.5 * k)
    assert tr.lost
    tm.step(scene(0), 9, 5.0)
    assert tr.id not in tm.tracks


def test_different_class_opens_a_new_track():
    tm = TrackManager(W, H)
    tm.step(scene(0), 1, 0.0)
    tm.anchor([("person", 0.9, (100, 60, 140, 120))], 1, 0.0)
    new = tm.anchor([("dog", 0.9, (100, 60, 140, 120))], 2, 0.1)
    assert len(new) == 1 and len(tm.tracks) == 2
