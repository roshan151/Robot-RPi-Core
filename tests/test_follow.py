"""Voice-triggered follow mode, against a fake camera, vision service and drivetrain."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest  # noqa: E402

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

    fast_dps = 120.0
    speeds: list = []

    def move_to(self, pan=None, tilt=None, speed_dps=None, wait=False):
        self.moves.append((pan, tilt))
        self.speeds.append(speed_dps)
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
    h.speeds = []
    follow.follow(Camera(), v, Motion(), "Sam", head=h, seconds=60, clock=Clock())
    assert h.speeds[1:] == [120.0, None, None, 120.0], "searching looks move fast; tracking at the usual speed"
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

def floor(start=0.2, until=1.5, box=None, drop=None):
    """Floor-scan points (x, y, h) every 5 cm across the path: flat, with a 10 cm box from
    `box` (x, height) for 0.3 m, and no floor at all from `drop`."""
    pts, x = [], start
    while x <= until + 1e-9:
        h = 0.0
        if box and box[0] <= x <= box[0] + 0.3:
            h = box[1]
        if drop is not None and x >= drop:
            h = None
        pts += [(round(x, 3), y, h) for y in (-0.1, 0.0, 0.1)]
        x += 0.05
    return pts


def zones(pts, d=None):
    """Tracking-view zones (row, col, x, y, h, distance) from points; distance = x unless given."""
    return [(0, 0, x, y, h, d if d is not None else x) for x, y, h in pts]


class TofHead(Head):
    """A head whose range sensor has a grid. `scans`: what each glance down sees, in turn;
    `view`: the zones of the tracking view."""
    def __init__(self, scans=(floor(),), view=(), grid_range=1.6):
        super().__init__()
        from robot_core import terrain
        from robot_core.sensors import tof8x8
        g = tof8x8.Grid(0.0, tuple(tuple([grid_range] * 8) for _ in range(8)))
        self.lidar = SimpleNamespace(grid=lambda *a, **k: g)
        self.has_grid, self.glances = True, 0
        self._scans, self.view = list(scans), list(view)
        self.terrain = terrain.Params()

    def angles(self): return (0.0, 10.0)
    def zones(self): return list(self.view)

    def floor_scan(self):
        self.glances += 1
        self.moves.append("down")
        return self._scans.pop(0) if len(self._scans) > 1 else self._scans[0]


class Moving(Motion):
    """Motion that moves a pose: x, y, heading (turns + right, as the tools)."""
    def __init__(self, bump_on=None):
        super().__init__()
        self.pose, self.bump_on, self.bump_t = [0.0, 0.0, 0.0], bump_on, 0.0

    def turn(self, d):
        self.pose[2] -= np.radians(d)
        return super().turn(d)

    def drive(self, m):
        import time
        self.pose[0] += m * np.cos(self.pose[2])
        self.pose[1] += m * np.sin(self.pose[2])
        if self.bump_on is not None and len([1 for k, _ in self.moves if k == "drive"]) == self.bump_on - 1:
            self.bump_t = time.monotonic() + 0.001
        return super().drive(m)


def test_tof_range_on_the_person_is_sent_once_their_box_is_known_and_the_grid_every_frame():
    boxed = dict(step("tracking"), bbox=[20, 10, 44, 48], head=[0, 0], locked=True)
    h, v = TofHead(), Vision([boxed, boxed, step("gone")])
    follow.follow(Camera(), v, Motion(), "Sam", head=h, seconds=60, clock=Clock())
    assert v.calls[0][1]["range_m"] is None                  # no box yet
    assert v.calls[1][1]["range_m"] == 1.6
    assert len(v.calls[0][1]["tof_grid"]) == 8 and v.calls[0][1]["tof_grid"][0][0] == 1.6


def test_the_floor_is_checked_before_a_drive_and_the_robot_drives_up_to_what_is_in_the_way():
    drive = dict(step("tracking", turn=-10.0, drive=0.5), head=[0, 0], locked=True)
    h, v, m = TofHead(scans=(floor(box=(0.6, 0.1)), floor())), Vision([drive, drive, step("gone")]), Motion()
    follow.follow(Camera(), v, m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("turn", -10.0), ("drive", 0.35), ("turn", -10.0), ("drive", 0.5)], \
        "a box at 0.6 m: stop 0.25 m short of it, not skip the drive"
    i = h.moves.index("down")
    assert h.moves[i + 1] == (0.0, 10.0), "the head goes back to where it was for the next frame"


def test_too_close_to_a_drop_a_step_or_a_box_or_an_unreadable_floor_is_not_driven():
    drive = dict(step("tracking", drive=0.3), head=[0, 0], locked=True)
    for scan, drives in ((floor(drop=0.3), 0), (floor(box=(0.3, 0.02)), 0), (floor(box=(0.3, 0.1)), 0),
                         ([], 0), (floor(), 1)):
        h, m = TofHead(scans=(scan,)), Motion()
        follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
        assert len(m.moves) == drives, scan[:3]
    h, m = TofHead(scans=([],)), Motion()
    h.terrain.drive_unknown = True                               # robot.yaml drive_on_unknown_floor: true
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert len(m.moves) == 1


def test_stairs_down_ahead_stop_the_drive_short_of_the_edge():
    drive = dict(step("tracking", drive=0.5), head=[0, 0], locked=True)
    h, m = TofHead(scans=(floor(drop=0.7),)), Motion()
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("drive", 0.45)]


def test_floor_seen_clear_is_not_looked_at_again_while_it_is_still_ahead():
    drive = dict(step("tracking", drive=0.3), head=[0, 0], locked=True)
    h, m = TofHead(), Moving()
    follow.follow(Camera(), Vision([drive, drive, drive, step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert [k for k, _ in m.moves] == ["drive"] * 3
    assert h.glances == 1, "floor seen clear to 1.5 m: three 0.3 m drives need one look"


def test_the_tracking_view_is_used_when_it_sees_the_floor_right_in_front():
    drive = dict(step("tracking", drive=0.4), head=[0, 0], locked=True)
    h, m = TofHead(view=zones(floor(start=0.2))), Motion()
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("drive", 0.4)] and h.glances == 0
    h, m = TofHead(view=zones(floor(start=0.6))), Motion()          # the near floor not in view: look down
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("drive", 0.4)] and h.glances == 1


def test_something_in_the_way_at_any_height_shortens_the_drive_but_the_person_does_not():
    drive = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=1.6)
    chair = zones([(0.65, 0.05, 0.25)]) + zones(floor())             # a chair seat edge at 25 cm, 0.65 m ahead
    h, m = TofHead(view=chair), Motion()
    follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
    assert m.moves == [("drive", 0.4)]
    legs = zones([(1.5, 0.0, 0.25)]) + zones(floor())                # the person themselves
    table = zones([(0.5, 0.0, 0.7)]) + zones(floor())                # above the robot: it passes under
    for view in (legs, table):
        h, m = TofHead(view=view), Motion()
        follow.follow(Camera(), Vision([drive, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock())
        assert m.moves == [("drive", 1.1)], "1.6 m away: up to 0.5 m from them"


def test_a_bump_stops_the_drive_and_no_drive_is_tried_until_the_robot_has_turned_away():
    drive = dict(step("tracking", drive=0.5), head=[0, 0], locked=True)
    turn_and_drive = dict(step("tracking", turn=25.0, drive=0.5), head=[0, 0], locked=True)
    h, m = TofHead(), Moving(bump_on=1)
    follow.follow(Camera(), Vision([drive, drive, turn_and_drive, step("gone")]), m, "Sam", head=h,
                  seconds=60, clock=Clock(), bumped=lambda: m.bump_t)
    assert m.moves == [("drive", 0.5), ("turn", 25.0), ("drive", 0.5)]
    assert m.stopped >= 1


def test_once_lost_the_mac_is_told_where_they_were_last_seen():
    seen = dict(step("tracking", turn=10.0), head=[0, 0], locked=True, bearing_deg=-40.0)
    lost = dict(step("searching"), head=[0, 0], locked=True)
    m, v = Moving(), Vision([seen, lost, step("gone")])
    follow.follow(Camera(), v, m, "Sam", head=Head(), seconds=60, clock=Clock(), pose=lambda: tuple(m.pose))
    assert v.calls[0][1]["hint_deg"] is None
    # The Pi's tracker turns to face the tracked person (40 right), not the Mac's partial 10:
    assert m.moves[0] == ("turn", 40.0)
    assert abs(v.calls[1][1]["hint_deg"]) < 0.2, "40 right, then the robot turned 40 right to face them"


# --------------------------------------------------------------------------- #
# Longer steps, and planning round what blocked a drive (robot_core/local_plan.py)
# --------------------------------------------------------------------------- #

def test_a_far_person_gets_a_long_step_but_never_past_the_floor_seen(monkeypatch):
    monkeypatch.setattr(follow.settings, "FOLLOW_MAX_DRIVE_M", 3.0)   # legs of up to 3 m
    far = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=3.0, bearing_deg=0.0)
    for until, drive in ((3.0, 2.5), (1.5, 1.3)):        # 3.0 - 0.5 m; no live watch: floor seen - 0.2
        h, m = TofHead(scans=(floor(until=until),)), Moving()
        follow.follow(Camera(), Vision([far, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock(),
                      pose=lambda: tuple(m.pose))
        assert m.moves == [("drive", drive)], (until, m.moves)
    near = dict(far, distance_m=0.9)                     # 0.5 m from the person is close enough
    h, m = TofHead(scans=(floor(until=2.0),)), Moving()
    follow.follow(Camera(), Vision([near, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock(),
                  pose=lambda: tuple(m.pose))
    assert m.moves == [("drive", 0.4)], "the Pi decides: up to 0.5 m from them, whatever the Mac's step"


def wall(x=0.4, half=0.1):
    """A floor scan with a 10 cm high box at x for 0.3 m, `half` m to each side of the path."""
    pts = [p for p in floor(until=2.0) if not (x <= p[0] <= x + 0.3)]
    y = -half
    while y <= half + 1e-9:
        pts += [(round(x + dx, 3), round(y, 3), 0.1) for dx in (0.0, 0.1, 0.2, 0.3)]
        y += 0.05
    return pts


def test_a_box_in_the_way_is_driven_round_and_the_robot_faces_the_person_again():
    drive = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=1.7, bearing_deg=0.0)
    h, m = TofHead(scans=(wall(), floor(until=2.0))), Moving()
    follow.follow(Camera(), Vision([drive, drive, step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert m.moves[0] == ("drive", 0.15), "first up to the box, stopping short of it"
    rest = m.moves[1:]
    assert rest[0][0] == "turn" and abs(rest[0][1]) >= 20, "out to the side"
    assert sum(v for k, v in rest if k == "drive") > 0.3, "round the box, leg by leg"
    assert rest[-1][0] == "turn", "and turned back towards them"
    assert abs(m.pose[1]) > 0.25, "the path went beside the box"
    assert h.glances >= 2 and h.moves.count("down") == h.glances


def test_with_no_way_round_it_stops_and_does_not_keep_looking():
    drive = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=1.7, bearing_deg=0.0)
    h, m = TofHead(scans=(wall(half=2.0),)), Moving()
    follow.follow(Camera(), Vision([drive, drive, drive, step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert m.moves == [("drive", 0.15)] and h.glances == 1


class WatchingHead(TofHead):
    """A TofHead that can watch the floor while driving: `profiles` are what each grid
    during the drive shows (floor_profile), in turn."""
    def __init__(self, profiles, **kw):
        super().__init__(**kw)
        self._profiles, self.geom = list(profiles), SimpleNamespace(
            hfov_deg=66.0, height_m=0.25, pan_axis_x_m=0.05, drive_tilt_deg=-30.0)
        self.watched = 0

    def drive_pose(self): self.moves.append("drive pose")
    def floor_profile(self):
        self.watched += 1
        return self._profiles.pop(0) if len(self._profiles) > 1 else self._profiles[0]


class SlowDrive(Moving):
    """A drive that stays busy for `polls` checks of busy(), unless stopped first."""
    def __init__(self, polls=4):
        super().__init__()
        self.polls, self._left = polls, 0
    def drive(self, m):
        self._left = self.polls
        return super().drive(m)
    def busy(self):
        self._left -= 1
        return self._left >= 0
    def stop(self):
        self._left = 0
        return super().stop()


def test_a_long_drive_is_watched_as_it_goes_and_brakes_when_something_comes_near(monkeypatch):
    monkeypatch.setattr(follow.settings, "FOLLOW_MAX_DRIVE_M", 3.0)   # legs of up to 3 m
    far = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=4.2, bearing_deg=0.0)
    clear, box_far, box_near = floor(until=0.9), floor(until=0.9, box=(0.7, 0.1)), floor(until=0.9, box=(0.4, 0.1))
    h, m = WatchingHead([clear, box_far, box_near], scans=(floor(until=1.4),)), SlowDrive(polls=6)
    follow.follow(Camera(), Vision([far, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock(),
                  pose=lambda: tuple(m.pose))
    assert m.moves[0] == ("drive", 3.0), "a 3 m drive although the glance saw floor only to 1.4 m"
    assert m.stopped == 1 and h.watched == 3, "clear, a box still far, then the box near: brake"
    assert "drive pose" in h.moves and h.moves[-1] == (0.0, 10.0), "head down for the drive, back up after"


def test_one_thin_grid_does_not_brake_but_several_in_a_row_do(monkeypatch):
    monkeypatch.setattr(follow.settings, "FOLLOW_MAX_DRIVE_M", 3.0)   # legs of up to 3 m
    far = dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=3.2, bearing_deg=0.0)
    h, m = WatchingHead([[], floor(until=0.9), [], [], []], scans=(floor(until=2.5),)), SlowDrive(polls=8)
    follow.follow(Camera(), Vision([far, step("gone")]), m, "Sam", head=h, seconds=60, clock=Clock(),
                  pose=lambda: tuple(m.pose))
    assert m.stopped == 1 and h.watched == 5


# --------------------------------------------------------------------------- #
# Gliding: following by steering (motion.velocity, firmware v6)
# --------------------------------------------------------------------------- #

@pytest.fixture()
def glide(monkeypatch):
    """Gliding is off by default (the ToF is on the gimbal): these tests switch it on."""
    monkeypatch.setattr(follow.settings, "FOLLOW_GLIDE", True)


class Glider(Moving):
    """Motion with the velocity mode: records every (v, w)."""
    def __init__(self):
        super().__init__()
        self.vel = []
    def can_glide(self): return True
    def velocity(self, v, w): self.vel.append((round(v, 3), round(w, 3)))


def glide_head(view=None, scans=None):
    h = WatchingHead([floor(until=0.9)], scans=scans or (floor(until=1.5),),
                     view=zones(floor(start=0.45, until=1.5)) if view is None else view)
    h.geom.tilt_offset_deg = 21.0
    return h


def seen_at(bearing, distance=3.0):
    return dict(step("tracking", drive=0.5), head=[0, 0], locked=True, distance_m=distance, bearing_deg=bearing)


def test_gliding_steers_toward_the_person_and_drives_without_stopping(glide):
    h, m = glide_head(), Glider()
    follow.follow(Camera(), Vision([seen_at(0.0), seen_at(0.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert m.vel[:2] == [(0.2, 0.0), (0.2, 0.0)] and m.vel[-1] == (0.0, 0.0), m.vel
    assert m.moves == [], "no stop-and-go moves at all"
    assert h.glances == 1, "the floor right in front is looked at once, when it sets off"
    assert (0.0, -21.0) in h.moves, "head held level (21 deg below its straight position)"


def test_gliding_turns_in_proportion_and_turns_in_place_when_far_to_the_side(glide):
    h, m = glide_head(), Glider()
    follow.follow(Camera(), Vision([seen_at(20.0), seen_at(-45.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    (v1, w1), (v2, w2) = m.vel[:2]
    assert v1 > 0 and w1 > 0.25, "20 deg left: on the move, curving left"
    assert (v2, w2) == (0.0, 0.0) and ("turn", 27.0) in m.moves, "45 deg right: a counted turn on the spot"


def test_close_enough_it_only_turns_to_keep_facing_them(glide):
    h, m = glide_head(), Glider()
    follow.follow(Camera(), Vision([seen_at(10.0, distance=0.55), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert m.vel[0] == (0.0, 0.0) and m.moves == [("turn", -6.0)] and h.glances == 0


def test_a_box_ahead_slows_the_glide_and_close_up_it_stops_and_falls_back_to_a_step(glide):
    h, m = glide_head(view=zones(floor(start=0.45, until=1.5, box=(0.6, 0.1)))), Glider()
    follow.follow(Camera(), Vision([seen_at(0.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert 0.1 < m.vel[0][0] < 0.2, "0.35 m clear: slower than the top speed"
    boxed = floor(start=0.2, until=1.5, box=(0.3, 0.1))
    h, m = glide_head(view=zones(boxed), scans=(boxed,)), Glider()
    follow.follow(Camera(), Vision([seen_at(0.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert all(v == 0.0 for v, _ in m.vel), "too close: never driven on"
    assert sum(d for k, d in m.moves if k == "drive") < 0.1


def test_without_the_velocity_mode_it_follows_in_steps_as_before():
    h, m = glide_head(), Moving()                      # no can_glide: firmware v5
    follow.follow(Camera(), Vision([seen_at(0.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert [k for k, _ in m.moves] == ["drive"]


def test_floor_noise_near_the_robot_is_not_something_in_the_way(glide):
    import random
    rnd = random.Random(2)
    noisy = [(x, y, round(0.03 + 0.012 * rnd.random(), 3) if x < 0.5 else h)    # near rows read 3-4 cm high
             for x, y, h in floor(start=0.45, until=1.5)]
    h, m = glide_head(view=zones(noisy)), Glider()
    h.terrain.flat_tol_m = 0.045
    follow.follow(Camera(), Vision([seen_at(0.0), step("gone")]), m, "Sam", head=h, seconds=60,
                  clock=Clock(), pose=lambda: tuple(m.pose))
    assert m.vel[0][0] > 0.1, m.vel


# --------------------------------------------------------------------------- #
# The Pi's person tracker (follow.PersonTracker)
# --------------------------------------------------------------------------- #

def test_tracker_places_the_person_in_the_odom_frame_and_weighs_readings_by_how_sure_they_are():
    t = follow.PersonTracker(keep_m=0.5, max_leg_m=1.5)
    t.update((1.0, 0.0, math.pi / 2), 0.0, 2.0, 0.05, t=0.0)          # facing +y: they are at (1, 2)
    assert t.xy == pytest.approx((1.0, 2.0))
    t.update((1.0, 0.0, math.pi / 2), 0.0, 2.6, 0.78, t=0.1)           # a box-width guess: barely moves it
    assert t.xy[1] < 2.05
    turn, drive, dist = t.move((1.0, 0.0, math.pi / 2))
    assert turn == 0.0 and drive == pytest.approx(min(dist - 0.5, 1.5)) and dist < 2.05


def test_tracker_turns_past_the_deadband_and_ignores_one_wild_reading():
    t = follow.PersonTracker(keep_m=0.5, max_leg_m=1.5)
    t.update((0.0, 0.0, 0.0), 30.0, 2.0, 0.05, t=0.0)                  # 30 deg left
    turn, drive, _ = t.move((0.0, 0.0, 0.0))
    assert turn == pytest.approx(-30.0) and drive > 0                   # turn left to face them, then the leg
    assert t.move((0.0, 0.0, math.radians(25.0)))[0] == 0.0             # within 8 deg: no turn
    before = t.xy
    t.update((0.0, 0.0, 0.0), -60.0, 4.0, 0.05, t=0.2)                  # a lookalike far off: ignored once
    assert t.xy == before
    t.update((0.0, 0.0, 0.0), -60.0, 4.0, 0.05, t=0.4)                  # twice: believed
    assert t.xy == pytest.approx((2.0, -3.464), abs=1e-3)
