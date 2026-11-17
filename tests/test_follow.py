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


def test_finds_them_looking_up_then_tracks_looking_lower_and_searches_looking_up_again():
    S = follow.settings
    h = Head()
    v = Vision([dict(step("searching", followed=False), head=[0, 0], locked=False),
                dict(step("tracking"), head=[0, 0], locked=True),
                dict(step("holding"), head=[0, 0], locked=True),
                dict(step("searching"), head=[30, 0], locked=True),
                step("gone")])
    follow.follow(Camera(), v, Motion(), "Sam", head=h, seconds=60, clock=Clock())
    assert h.moves == [(0.0, S.FOLLOW_TILT_DEG),               # start: the face
                       (0, S.FOLLOW_TILT_DEG),                 # still confirming who it is
                       (0, S.FOLLOW_TRACK_TILT_DEG),           # locked on: body and feet
                       (0, S.FOLLOW_TRACK_TILT_DEG),
                       (30, S.FOLLOW_TILT_DEG)]                # lost: search looks up again


def test_cancel_during_a_turn_brakes_drives_no_further_and_ends():
    from robot_core import task_cancel
    task_cancel.reset()

    class StopsMidTurn(Motion):
        def turn(self, d):
            task_cancel.request("voice")             # "stop" heard while turning
            return super().turn(d)

    v, m = Vision([step("tracking", turn=-20.0, drive=0.5)] + [step("tracking", drive=0.5)] * 5), StopsMidTurn()
    try:
        assert follow.follow(Camera(), v, m, "Sam", seconds=60, clock=Clock())   # had locked on
        assert m.moves == [("turn", -20.0)], "no drive after the cancel"
        assert v.stopped and len(v.calls) == 1
    finally:
        task_cancel.reset()


# --------------------------------------------------------------------------- #
# With the 8x8 ToF on the head
# --------------------------------------------------------------------------- #

class TofHead(Head):
    """A head whose range sensor has a grid, and whose floor check says `kinds` in turn."""
    def __init__(self, kinds=("flat",), grid_range=1.6):
        super().__init__()
        from robot_core import terrain
        from robot_core.sensors import tof8x8
        g = tof8x8.Grid(0.0, tuple(tuple([grid_range] * 8) for _ in range(8)))
        self.lidar = SimpleNamespace(grid=lambda *a, **k: g)
        self.has_grid, self.reaches = True, []
        self._kinds, self._terrain = list(kinds), terrain
        self.terrain = terrain.Params()

    def angles(self): return (0.0, 10.0)

    def terrain_ahead(self, reach_m):
        self.reaches.append(reach_m)
        self.moves.append("down")
        kind = self._kinds.pop(0) if len(self._kinds) > 1 else self._kinds[0]
        return self._terrain.Terrain(kind, 0.35, 0.1)


def test_tof_range_on_the_person_is_sent_once_their_box_is_known():
    boxed = dict(step("tracking"), bbox=[20, 10, 44, 48], head=[0, 0], locked=True)
    h, v = TofHead(), Vision([boxed, boxed, step("gone")])
    follow.follow(Camera(), v, Motion(), "Sam", head=h, seconds=60, clock=Clock())
    assert v.calls[0][1]["range_m"] is None                  # no box yet
    assert v.calls[1][1]["range_m"] == 1.6


def test_the_floor_is_checked_before_every_drive_and_a_blocked_path_is_not_driven():
    drive = dict(step("tracking", turn=-10.0, drive=0.5), head=[0, 0], locked=True)
    h, v, m = TofHead(kinds=("blocked", "flat")), Vision([drive, drive, step("gone")]), Motion()
    follow.follow(Camera(), v, m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("turn", -10.0), ("turn", -10.0), ("drive", 0.5)], "it still turns to face them"
    assert h.reaches == [0.7, 0.7]
    i = h.moves.index("down")
    assert h.moves[i + 1] == (0.0, 10.0), "the head goes back to where it was for the next frame"


def test_only_flat_floor_is_driven_a_drop_a_step_or_an_unreadable_floor_is_not():
    drive = dict(step("tracking", drive=0.3), head=[0, 0], locked=True)
    for kind, drives in (("drop", 0), ("climbable", 0), ("blocked", 0), ("unknown", 0), ("flat", 1)):
        h, m = TofHead(kinds=(kind,)), Motion()
        follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
        assert len(m.moves) == drives, kind
    h, m = TofHead(kinds=("unknown",)), Motion()
    h.terrain.drive_unknown = True                               # robot.yaml drive_on_unknown_floor: true
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert len(m.moves) == 1
