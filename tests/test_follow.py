"""Voice-triggered follow mode, against a fake camera, vision service and drivetrain."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_core import follow  # noqa: E402


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self):
        self.t += 1.0
        return self.t


class Camera:
    def capture(self, gimbal=None):
        return SimpleNamespace(main=np.zeros((48, 64, 3), np.uint8), pan_deg=0.0, tilt_deg=20.0)


class Motion:
    def __init__(self): self.moves, self.stopped = [], 0
    def turn(self, d): self.moves.append(("turn", d)); return {"ok": True}
    def drive(self, m): self.moves.append(("drive", m)); return {"ok": True}
    def busy(self): return False
    def stop(self): self.stopped += 1; return {"ok": True}


def step(state, turn=0.0, drive=0.0, followed=True):
    return {"state": state, "turn_deg": turn, "drive_m": drive, "followed": followed,
            "target": "Sam", "distance_m": None}


class Vision:
    def __init__(self, steps, enrolled=True):
        self.steps, self.enrolled = list(steps), enrolled
        self.calls, self.stopped = [], False

    def follow_start(self, label=""):
        return {"label": label} if self.enrolled else None

    def follow_step(self, jpeg, seq, **geo):
        self.calls.append((seq, geo))
        return self.steps.pop(0) if self.steps else step("holding")

    def follow_stop(self):
        self.stopped = True
        return {}


def test_does_what_the_mac_says_until_gone():
    v, m = Vision([step("searching", followed=False), step("tracking", turn=-20.0, drive=0.0),
                   step("tracking", drive=0.5), step("gone")]), Motion()
    assert follow.follow(Camera(), v, m, "Sam", seconds=60, clock=Clock())
    assert m.moves == [("turn", -20.0), ("drive", 0.5)]
    assert v.stopped and len(v.calls) == 4
    assert v.calls[0][1]["tilt_deg"] == 20.0 and v.calls[0][1]["range_m"] is None


def test_not_enrolled_fails_without_moving():
    v, m = Vision([], enrolled=False), Motion()
    assert not follow.follow(Camera(), v, m, "Nobody", seconds=60, clock=Clock())
    assert m.moves == [] and v.calls == []


def test_never_found_answers_no_and_stops_at_the_time_limit():
    v, m = Vision([step("searching", turn=-30.0, followed=False)] * 100), Motion()
    assert not follow.follow(Camera(), v, m, "", seconds=5, clock=Clock())
    assert v.stopped and 0 < len(v.calls) < 6


def test_a_refused_move_ends_the_task_cleanly():
    class Refuses(Motion):
        def drive(self, m): return {"ok": False, "error": "drivetrain is not running"}
    v, m = Vision([step("tracking", drive=0.5)]), Refuses()
    assert follow.follow(Camera(), v, m, "Sam", seconds=60, clock=Clock())   # it had locked on
    assert v.stopped


class Head:
    """Records where the head was sent; carries a lidar and geometry like sensors.head.Head."""
    def __init__(self):
        self.gimbal, self.moves = self, []
        self.lidar = SimpleNamespace(read=lambda n, timeout: 1.8)
        self.geom = SimpleNamespace(hfov_deg=60.0, height_m=0.3)

    def move_to(self, pan=None, tilt=None, wait=False): self.moves.append((pan, tilt))
    def wait_settled(self): return True


def test_head_is_held_ahead_and_its_range_and_geometry_are_sent():
    h, v = Head(), Vision([step("gone")])
    follow.follow(Camera(), v, Motion(), "Sam", head=h, seconds=60, clock=Clock())
    assert h.moves == [(0.0, follow.settings.FOLLOW_TILT_DEG)]
    geo = v.calls[0][1]
    assert (geo["range_m"], geo["hfov_deg"], geo["camera_height_m"]) == (1.8, 60.0, 0.3)
