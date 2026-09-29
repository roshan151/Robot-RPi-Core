"""Identity, free space, follow, approach, gimbal and the behavior state machine.
All pure — no camera, no network, no ROS."""

from __future__ import annotations

import pytest

from robot_core.perception import approach, follow
from robot_core.perception.behavior import BehaviorConfig, Mode, TargetView, VisionBehavior
from robot_core.perception.freespace import analyze, corridor, grid
from robot_core.perception.geometry import CameraModel
from robot_core.perception.gimbal import GimbalConfig, GimbalController
from robot_core.perception.identity import UNKNOWN, IdentityConfig, IdentityResolver
from robot_core.perception.motion_types import Move
from robot_core.perception.scene import describe

CAM = CameraModel(640, 360, 66.0)

# ------------------------------------------------------------------ identity


def test_identity_needs_two_agreeing_matches_then_sticks():
    r = IdentityResolver()
    assert r.wants_embed(1, 100, 0.0)
    r.started(1, 0.0)
    assert r.result(1, 0.9, "roshan", 0.85) is None
    r.started(1, 1.0)
    assert r.result(1, 0.9, "roshan", 0.88) == "roshan"
    assert r.identity(1) == "roshan"
    assert not r.wants_embed(1, 100, 10.0)          # never re-query


def test_disagreement_resets_the_streak():
    r = IdentityResolver()
    r.result(1, 0.9, "roshan", 0.8)
    r.result(1, 0.9, "sam", 0.8)
    assert r.identity(1) is None
    assert r.result(1, 0.9, "sam", 0.8) == "sam"


def test_low_quality_and_below_threshold_decide_nothing():
    r = IdentityResolver()
    r.result(1, 0.3, "roshan", 0.9)          # blurry crop ignored
    r.result(1, 0.9, None, 0.5)              # below threshold
    r.result(1, 0.9, "roshan", 0.8)
    assert r.identity(1) is None


def test_gives_up_as_unknown_after_budget():
    r = IdentityResolver(IdentityConfig(max_attempts=3))
    for i in range(3):
        r.started(1, float(i))
        r.no_face(1)
    assert r.identity(1) == UNKNOWN


def test_network_failures_do_not_use_up_attempts():
    r = IdentityResolver(IdentityConfig(max_attempts=2))
    for i in range(5):
        r.started(1, float(i))
        r.failed(1)
    assert r.identity(1) is None and r.wants_embed(1, 100, 10.0)

# ----------------------------------------------------------------- freespace


def _depths(layout, far_by_col, near=10.0):
    n = len(layout.columns)
    mid = [(f + near) / 2 for f in far_by_col]
    return list(far_by_col) + mid + [near] * n


def test_open_floor_is_clear_and_a_wall_is_not():
    lay = grid(640, 360)
    open_floor = analyze(_depths(lay, [4.0] * 7), False, lay, CAM)
    assert open_floor.ahead_clear is True
    wall = analyze(_depths(lay, [10.0] * 7), False, lay, CAM)
    assert wall.ahead_clear is False and wall.best_bearing_deg is None


def test_best_direction_is_the_open_side():
    lay = grid(640, 360)
    # Blocked on the left and centre, open on the right.
    fs = analyze(_depths(lay, [10, 10, 10, 10, 9.5, 3.0, 3.5]), False, lay, CAM)
    assert fs.ahead_clear is False
    assert fs.best_bearing_deg > 15


def test_metric_depth_uses_inverse():
    lay = grid(640, 360)
    far_m, near_m = 3.0, 0.5            # metres: far row genuinely farther -> clear
    depths = [far_m] * 7 + [1.0] * 7 + [near_m] * 7
    assert analyze(depths, True, lay, CAM).ahead_clear is True


def test_refuses_when_tilted_up():
    lay = grid(640, 360)
    fs = analyze(_depths(lay, [4.0] * 7), False, lay, CAM, tilt_deg=20)
    assert fs.ahead_clear is None


def test_corridor_needs_visible_floor():
    assert corridor(640, 360, top_v=0.95 * 360) is None
    lay = corridor(640, 360, top_v=0.5 * 360)
    assert lay is not None and len(lay.columns) == 3

# -------------------------------------------------------------------- follow


FC = follow.FollowConfig()


def fin(b=0.0, d=3.0, clear=True, age=0.1):
    return follow.FollowInput(b, d, clear, age)


def test_follow_turns_to_face_first():
    m = follow.decide(fin(b=30.0), FC)
    assert m.kind == "turn" and 0 < m.value <= FC.max_turn_deg


def test_follow_steps_forward_only_with_fresh_clearance():
    assert follow.decide(fin(d=3.0), FC).kind == "drive"
    assert follow.decide(fin(d=3.0, clear=None), FC).reason == "need clearance check"
    assert follow.decide(fin(d=3.0, age=5.0), FC).reason == "need clearance check"
    assert follow.decide(fin(d=3.0, clear=False), FC).kind == "hold"


def test_follow_never_steps_bigger_than_max_or_past_standoff():
    m = follow.decide(fin(d=1.7), FC)
    assert m.kind == "drive" and m.value <= FC.max_step_m
    assert m.value <= 1.7 - FC.standoff_m + 1e-9


def test_follow_holds_too_close_and_never_reverses():
    m = follow.decide(fin(d=0.5), FC)
    assert m.kind == "hold" and m.reason == "too close"


def test_follow_holds_without_distance():
    assert follow.decide(fin(d=None), FC).kind == "hold"


def test_stop_distance_is_inside_standoff():
    assert FC.stop_m < FC.standoff_m

# ------------------------------------------------------------------ approach


AC = approach.ApproachConfig()


def ain(b=0.0, d=2.0, travelled=0.0, detours=0, clear=True, age=0.1, open_b=None):
    return approach.ApproachInput(b, d, travelled, detours, clear, age, open_b)


def test_approach_aligns_then_drives_then_arrives():
    assert approach.step(ain(b=20), AC).move.kind == "turn"
    s = approach.step(ain(d=2.0), AC)
    assert s.move.kind == "drive" and s.move.value == AC.max_step_m
    assert approach.step(ain(d=0.65), AC).done


def test_approach_is_bounded():
    s = approach.step(ain(travelled=3.0), AC)
    assert s.failed


def test_approach_detours_then_gives_up():
    s = approach.step(ain(clear=False, open_b=25.0), AC)
    assert s.detour and s.move.kind == "turn" and s.move.value == 25.0
    assert approach.step(ain(clear=False, open_b=None), AC).failed
    assert approach.step(ain(clear=False, open_b=25.0, detours=2), AC).failed


def test_approach_creeps_without_distance():
    s = approach.step(ain(d=None), AC)
    assert s.move.kind == "drive" and s.move.value == AC.creep_step_m

# -------------------------------------------------------------------- gimbal


def test_gimbal_is_slew_limited_and_clamped():
    g = GimbalController(GimbalConfig(max_rate_dps=100, gain=1.0))
    g.track(200.0, -90.0)
    pan, tilt = g.tick(0.05, 0.05)
    assert pan == pytest.approx(5.0)           # 100 deg/s * 50 ms
    for i in range(200):
        pan, tilt = g.tick(0.1 + i * 0.05, 0.05)
    assert pan == 90.0 and tilt == -30.0        # clamped to limits


def test_gimbal_settles_after_it_stops_moving():
    g = GimbalController(GimbalConfig(settle_s=0.12))
    g.track(10.0, 0.0)
    t = 0.0
    for _ in range(100):
        t += 0.05
        g.tick(t, 0.05)
    assert g.settled(t + 0.2)
    g.track(40.0, 0.0)
    g.tick(t + 0.25, 0.05)
    assert not g.settled(t + 0.3)


def test_sweep_covers_both_sides():
    g = GimbalController(GimbalConfig(max_rate_dps=1000))
    g.start_sweep(0.0, 0.0)
    pans = []
    for i in range(200):
        pans.append(g.tick(i * 0.05, 0.05)[0])
    assert max(pans) >= 55 and min(pans) <= -55

# ------------------------------------------------------------------ behavior


def person(tid=1, bearing=0.0, dist=3.0, identity=None, lost=False, anchor=1.0, area=0.05):
    return TargetView(tid, "person", identity, bearing, 5.0, dist, lost, anchor, 0.7, area)


def test_follow_happy_path_look_then_step():
    b = VisionBehavior()
    b.follow(None, 0.0)
    d = b.tick([person()], motion_busy=False, degraded=False, now=1.0)
    assert d.gimbal[0] == "track"
    assert d.want_clearance == "corridor" and d.move is None
    assert any("locked on" in e for e in d.events)
    from robot_core.perception.freespace import FreeSpace
    b.set_clearance(FreeSpace([], 0.0, True), 1.1)
    d = b.tick([person()], motion_busy=False, degraded=False, now=1.2)
    assert d.move is not None and d.move.kind == "drive"
    # After the move ends, no new move until perception re-anchors the target.
    b.move_finished(d.move, "done", 2.0)
    d = b.tick([person(anchor=1.5)], motion_busy=False, degraded=False, now=2.1)
    assert d.move is None and d.want_clearance is None
    assert b.status["travelled_m"] == pytest.approx(0.3)


def test_follow_by_name_ignores_strangers_and_binds_to_the_right_person():
    b = VisionBehavior()
    b.follow("roshan", 0.0)
    d = b.tick([person(1, identity="sam")], motion_busy=False, degraded=False, now=0.5)
    assert b._goal.track_id is None and d.gimbal[0] == "sweep"
    d = b.tick([person(1, identity="sam"), person(2, bearing=20, identity="roshan")],
               motion_busy=False, degraded=False, now=1.0)
    assert b._goal.track_id == 2
    assert d.gimbal == ("track", 20, 3.0)       # identity known: tilt capped to keep floor in view


def test_follow_keeps_the_floor_in_view():
    b = VisionBehavior()
    b.follow(None, 0.0)
    d = b.tick([person(bearing=0.0)], motion_busy=False, degraded=False, now=1.0)
    assert d.gimbal[2] <= b.cfg.moving_max_tilt_deg     # aim_elevation 5.0 capped


def test_named_follow_keeps_the_face_in_view_until_identified():
    b = VisionBehavior()
    b.follow("roshan", 0.0)
    d = b.tick([person(identity=None)], motion_busy=False, degraded=False, now=1.0)
    assert d.gimbal[2] == 5.0


def test_follow_by_name_gives_up_if_never_seen():
    b = VisionBehavior(BehaviorConfig(find_timeout_s=2.0))
    b.follow("roshan", 0.0)
    b.tick([], motion_busy=False, degraded=False, now=0.1)
    d = b.tick([], motion_busy=False, degraded=False, now=3.0)
    assert b.mode == Mode.IDLE
    assert any("can't see roshan" in e for e in d.events)


def test_lost_target_sweeps_then_reacquires():
    b = VisionBehavior()
    b.follow(None, 0.0)
    b.tick([person(1, bearing=30.0)], motion_busy=False, degraded=False, now=1.0)
    d = b.tick([], motion_busy=False, degraded=False, now=2.0)
    assert b.mode == Mode.REACQUIRE and d.gimbal == ("sweep", 30.0)
    d = b.tick([person(7, bearing=40.0, anchor=3.0)], motion_busy=False, degraded=False, now=3.0)
    assert b.mode == Mode.FOLLOW and b._goal.track_id == 7
    assert any("reacquired" in e for e in d.events)


def test_degraded_perception_stops_the_behavior():
    b = VisionBehavior()
    b.follow(None, 0.0)
    b.tick([person()], motion_busy=False, degraded=False, now=1.0)
    d = b.tick([person()], motion_busy=False, degraded=True, now=2.0)
    assert b.mode == Mode.IDLE and d.move is None
    assert any("degraded" in e for e in d.events)


def test_drivetrain_rejection_stops_the_behavior():
    b = VisionBehavior()
    b.follow(None, 0.0)
    b.move_finished(Move("drive", 0.3), "rejected", 1.0)
    assert b.mode == Mode.IDLE


def test_look_tracks_with_the_head_only():
    b = VisionBehavior()
    b.look("person", 0.0)
    d = b.tick([person(bearing=25.0)], motion_busy=False, degraded=False, now=1.0)
    assert d.gimbal[0] == "track" and d.move is None


def test_approach_arrives_and_goes_idle():
    b = VisionBehavior()
    b.approach("chair", 0.0)
    chair = TargetView(3, "chair", None, 0.0, 0.0, 0.7, False, 1.0, 0.8, 0.1)
    d = b.tick([chair], motion_busy=False, degraded=False, now=1.0)
    assert b.mode == Mode.IDLE and any("arrived" in e for e in d.events)


def test_describe():
    s = describe([person(identity="roshan", bearing=-20.0, dist=2.0),
                  TargetView(3, "chair", None, 40.0, 0.0, 1.5, False, 1.0, 0.8, 0.01)])
    assert "roshan about 2.0 m away, slightly left" in s
    assert "a chair about 1.5 m away, 40 degrees right" in s
