"""Follow an enrolled person, as a job the voice agent runs with its session closed.

A stop-and-go loop with all the thinking on the Mac (Vision-Microservice
`follow.py`, see docs/FOLLOW-DESIGN.md):

    capture -> POST /v1/follow/step -> aim head -> turn turn_deg -> drive drive_m -> repeat

While searching the robot stays put and only the head sweeps (`head` in the
answer: pan, tilt above FOLLOW_TILT_DEG).

With the 8x8 ToF on the head (docs/TERRAIN-DESIGN.md, docs/FOLLOW-DESIGN.md):

  * the whole grid goes to the Mac with each frame, which ranges every person in
    view by the zones on them (distance, and a check on who is who);
  * before a drive, the path: something nearer than the person, at any height up
    to the robot's own, in the tracking view (#4), then the floor (#1, #3): only
    flat floor is driven. A box, a step, a drop (stairs down, a table edge) or a
    floor the sensor cannot read (unless drive_on_unknown_floor) ends the drive
    `stop_short_m` before it: the robot drives part of the way instead of not at all;
  * the floor is looked at as little as possible: floor seen clear in the last few
    seconds, still ahead on the same line, is not looked at again, and the tracking
    view's own lower rows are tried before the head glances down.

With the robot's pose (odometry, gyro heading when the IMU is fitted), `pose`:

  * once lost, the Mac is told where the person was last seen, in the robot's
    present frame, and searches there first (#5).

With `bumped` (the drivetrain's motion health): a jolt during a drive is something
the ToF did not see (glass, a thin chair leg). The drive is stopped, and no drive is
tried again until the robot has turned away by FOLLOW_BUMP_TURN_DEG (#7).

The robot stands still while the Mac decides; the person is expected to walk
slowly. The head looks straight ahead at FOLLOW_TILT_DEG while it finds them,
high enough to see a standing person's face from ~1.3 m (the face is what
locks on). Once locked on it drops to FOLLOW_TRACK_TILT_DEG: from then on they
are tracked by their clothes, and a lower view keeps their feet on the floor in
frame for the distance. A search (they were lost) looks from FOLLOW_TILT_DEG
again. Plain functions over injected parts — no ROS, no audio.

Ends when the Mac says the person is gone (lost, then a full head sweep),
after `seconds`, or when the task is cancelled (`task_cancel`: "stop" to the
cancel-only voice session, or the task/stop service). Returns True if it locked
onto the person at all.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from robot_core import debug_upload, local_plan, settings, status, task_cancel, terrain
from robot_core.sensors.camera import jpeg
from robot_core.sensors.tof8x8 import range_in_box
from robot_core.vision_client import VisionClient, VisionError

log = logging.getLogger(__name__)

MOVE_TIMEOUT_S = 30.0
MIN_DRIVE_M = 0.08              # a drive cut shorter than this is not worth making
MARGIN_M = 0.2                  # the floor must be clear this far past the end of a drive
FLOOR_MEMORY_S = 10.0           # floor seen clear is trusted this long: people and pets move
LAST_SEEN_S = 30.0              # where they were last seen is a search hint this long
FOLLOW_BUMP_TURN_DEG = 20.0     # after a bump, no drive until the robot has turned this far
FAR_LOOK_DEG = 12.0             # a long drive: one more floor look this much higher than the drive pose
PERSON_CLEAR_M = 0.35           # the person's own legs are not a hazard to remember
DETOUR_LEGS = 3                 # legs of a way round driven per step before looking for the person
DETOUR_MAX_TURN_DEG = 100.0     # a way round that starts by turning further (behind it, unseen) is not taken
WATCH_REACH_M = 0.8             # driving: the floor judged on every new grid, this far ahead
WATCH_UNSEEN_GRIDS = 3          # ...and this many grids in a row too thin to judge also brake
GLIDE_TURN_GAIN = 0.8           # gliding: turn rate (rad/s) per radian the person is off to the side
GLIDE_MAX_TURN = 0.8            # rad/s
GLIDE_DEADBAND_DEG = 3.0        # nearer ahead than this: no turning
GLIDE_MAX_BEARING_DEG = 30.0    # further to the side: turn in place first
GLIDE_SPEED_GAIN = 0.4          # forward speed (m/s) per metre of clear path: slows as it closes in
GLIDE_SPIN_DEG = 8.0            # not driving and further off than this: a counted turn (tracks scrub
                                # turning on the spot; a slow velocity spin barely moves)

TURN_DEADBAND_DEG = 8.0         # tracked person nearer ahead than this: no turn
DRIVE_MAX_BEARING_DEG = 25.0    # further to the side: turn first, drive next frame
WALK_MPS = 0.6                  # how fast a tracked person may have moved, for the tracker's doubt

Pose = Tuple[float, float, float]   # x, y (m), heading (rad, CCW) in the odometry frame


class PersonTracker:
    """Where the person stands, in the odometry frame, from the Mac's measurements.

    Each tracking answer places them from the robot's pose when the frame was taken
    (the robot stands still while the Mac answers, so that is the pose read just after
    the capture): `distance_m` along `bearing_deg`. Readings are combined by how sure
    each is (distance_sigma_m: a ToF range counts for far more than a box-size guess),
    and the estimate's doubt grows by WALK_MPS for every second it is not seen, so a
    person who walked on is taken at their new place. A reading far outside the doubt
    (a lookalike, a bad range) is ignored once; twice running, it is believed.
    The robot then turns to face the estimate and drives a leg toward it."""

    def __init__(self, keep_m: float, max_leg_m: float) -> None:
        self.keep_m, self.max_leg_m = keep_m, max_leg_m
        self.xy: Optional[Tuple[float, float]] = None
        self.var = 0.0
        self.t = 0.0
        self.rejected = 0

    def update(self, here: Pose, bearing_deg: float, dist_m: float, sigma_m: Optional[float],
               t: float) -> None:
        a = here[2] + math.radians(bearing_deg)
        z = (here[0] + dist_m * math.cos(a), here[1] + dist_m * math.sin(a))
        r = max(0.03, sigma_m if sigma_m else 0.3 * dist_m) ** 2
        if self.xy is None:
            self.xy, self.var, self.t = z, r, t
            return
        var = self.var + (WALK_MPS * max(0.0, t - self.t)) ** 2
        gap = math.hypot(z[0] - self.xy[0], z[1] - self.xy[1])
        if gap > 3.0 * math.sqrt(var + r) + 0.3 and self.rejected < 1:
            self.rejected += 1
            return
        if gap > 3.0 * math.sqrt(var + r) + 0.3:
            self.xy, self.var, self.t, self.rejected = z, r, t, 0     # twice: it is real, start again there
            return
        k = var / (var + r)
        self.xy = (self.xy[0] + k * (z[0] - self.xy[0]), self.xy[1] + k * (z[1] - self.xy[1]))
        self.var, self.t, self.rejected = (1.0 - k) * var, t, 0

    def move(self, now: Pose) -> Optional[Tuple[float, float, float]]:
        """(turn_deg + right, drive_m, distance_m) toward the estimate from `now`, or None."""
        if self.xy is None:
            return None
        dx, dy = self.xy[0] - now[0], self.xy[1] - now[1]
        dist = math.hypot(dx, dy)
        bearing = math.degrees(_wrap(math.atan2(dy, dx) - now[2]))       # + left
        turn = -bearing if abs(bearing) > TURN_DEADBAND_DEG else 0.0
        left = bearing + turn                                             # after the turn
        drive = min(dist - self.keep_m, self.max_leg_m) \
            if dist - self.keep_m >= MIN_DRIVE_M and abs(left) <= DRIVE_MAX_BEARING_DEG else 0.0
        return round(turn, 1), round(max(0.0, drive), 2), round(dist, 2)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _move(motion, fn, value: float, bumped: Optional[Callable[[], float]] = None) -> bool:
    """Send one turn/drive and wait for it to finish: the next frame must be
    taken standing still. True if the robot was bumped during it (then stopped)."""
    if task_cancel.requested():
        return False
    t0 = time.monotonic()
    res = fn(value)
    if not res.get("ok"):
        raise RuntimeError(f"move refused: {res.get('error')}")
    deadline = t0 + MOVE_TIMEOUT_S
    hit = lambda: bool(bumped) and (bumped() or 0.0) >= t0       # noqa: E731
    while motion.busy():
        if task_cancel.requested():
            return False                             # the canceller braked; follow() ends
        if hit():
            motion.stop()
            return True
        if time.monotonic() > deadline:
            raise RuntimeError("move timed out")
        time.sleep(0.1)
    if hit():                                        # bumped as it ended: brake all the same
        motion.stop()
        return True
    return False


@dataclass
class _Seen:
    """Floor seen clear: from `start` (a Pose) for `clear_m` straight ahead, at time `t`."""
    start: Pose
    clear_m: float
    t: float

    def left(self, pose: Optional[Pose], now: float) -> float:
        """How much of it is still ahead of the robot; 0 once it no longer applies
        (too old, turned off its line, or moved aside)."""
        if pose is None or now - self.t > FLOOR_MEMORY_S:
            return 0.0
        x0, y0, th0 = self.start
        dx, dy = pose[0] - x0, pose[1] - y0
        along = dx * math.cos(th0) + dy * math.sin(th0)
        side = -dx * math.sin(th0) + dy * math.cos(th0)
        if abs(_wrap(pose[2] - th0)) > math.radians(8.0) or abs(side) > 0.08 or along < -0.05:
            return 0.0
        return max(0.0, self.clear_m - along)


def _shorten(pts, p: terrain.Params, want_m: float) -> Tuple[float, Optional[str]]:
    """How much of `want_m` may be driven over the floor `pts` (with MARGIN_M clear
    after it): all of it, part of the way (up to `stop_short_m` before the first thing
    in the way), or nothing. And why it is short, if it is."""
    reach, why = want_m, None
    for _ in range(4):
        if reach < MIN_DRIVE_M:
            return 0.0, why
        t = terrain.classify(pts, p, reach + MARGIN_M)
        if terrain.may_drive(t, p):
            return reach, why
        why = terrain.why_not(t)
        if t.distance_m is None:                     # unknown floor: no distance to stop short of
            return 0.0, why
        reach = round(min(reach - p.cell_m, t.distance_m - p.stop_short_m), 2)
    return 0.0, why


def _turned(zones, turn_deg: float):
    """Zones seen before a turn of `turn_deg` (+ right), as they lie after it."""
    a = math.radians(turn_deg)                       # a right turn moves everything left (CCW)
    c, s = math.cos(a), math.sin(a)
    return [(r, col, x * c - y * s, x * s + y * c, h, d) for r, col, x, y, h, d in zones]


def _in_the_way(zones, p: terrain.Params, person_m: Optional[float]) -> Optional[float]:
    """The nearest thing in the robot's path in the tracking view: ahead of it, within
    the path's width, higher than a step it could climb and lower than the robot
    (a table top it passes under does not count), and nearer than the person by 0.3 m
    (so not the person). None without the person's range: their legs would count."""
    xs = [x for x, _ in _way_points(zones, p, person_m)]
    return min(xs) if xs else None


def _way_points(zones, p: terrain.Params, person_m: Optional[float]):
    """(x, y) of the tracking-view zones that `_in_the_way` counts."""
    if not zones or person_m is None:
        return []
    return [(x, y) for _, _, x, y, h, d in zones
            if h is not None and x > p.near_m and abs(y) <= p.corridor_m / 2.0
            and max(p.h_max_m, p.flat_tol_m) < h < p.body_m + 0.05 and d < person_m - 0.3]


def _levelled(zones, p: terrain.Params):
    """The tracking view's zones with heights measured from the floor in front (as the
    floor scan's): floor noise on the near rows must not read as something in the way."""
    if not zones or not p.auto_level:
        return zones
    lv, fit = terrain.level([(x, y, h) for _, _, x, y, h, _ in zones])
    if fit is None:
        return zones
    return [(r, c, x, y, h2, d) for (r, c, x, y, _, d), (_, _, h2) in zip(zones, lv)]


def _seen_to(pts, p: terrain.Params) -> float:
    """How far ahead, in the robot's path, the floor was actually measured (0: not at all)."""
    xs = [x for x, y, h in pts if h is not None and abs(y) <= p.corridor_m / 2.0 and x >= p.near_m]
    return max(xs) if xs else 0.0


def _radius(p: terrain.Params) -> float:
    """How far the robot's middle keeps from a hazard when planning: half the path, plus a gap
    (the ToF sees only an obstacle's near side, so its far side needs room too)."""
    return local_plan.radius(p)


def _remember(hazards, pose: Optional[Pose], pts_base, person_xy) -> None:
    if hazards is None or pose is None or not pts_base:
        return
    hazards.add(pose, pts_base)
    if person_xy is not None:
        hazards.forget_near(person_xy, PERSON_CLEAR_M)


def _plan_drive(head, want_m: float, zones, person_m: Optional[float], seen: Optional[_Seen],
                pose: Optional[Pose], now: float, hazards=None, person_xy=None,
                check_known: bool = True, watched: bool = False) -> Tuple[float, Optional[str], Optional[_Seen]]:
    """How far to drive of `want_m`: (metres, why short or None, floor now known clear).
    `zones` is the tracking view, already turned to the heading the drive starts from.
    `hazards` (local_plan.Hazards): what made earlier drives unsafe; a drive into one is
    refused without looking again (unless `check_known` is False: a leg of a way round
    that was planned round them), and what this check finds is added to it.
    `watched`: the drive will be watched as it goes (_drive): it may run past the floor
    seen now, and the far look is not needed.
    Without the 8x8 ToF: all of it, unchecked, as before."""
    if not head or not getattr(head, "has_grid", False) or task_cancel.requested():
        return want_m, None, seen
    p = head.terrain
    limit, why = want_m, None
    if check_known and hazards is not None and pose is not None and not local_plan.line_clear(
            pose[:2], (pose[0] + limit * math.cos(pose[2]), pose[1] + limit * math.sin(pose[2])),
            hazards.points(), _radius(p)):
        return 0.0, "known hazard", seen             # found before: no need to look at it again
    near = _in_the_way(zones, p, person_m)           # anything up to the robot's height (#4)
    if near is not None:
        _remember(hazards, pose, _way_points(zones, p, person_m), person_xy)
    if near is not None and near - p.stop_short_m < limit:
        limit, why = max(0.0, round(near - p.stop_short_m, 2)), f"in the way at {near:.2f} m"
    if limit < MIN_DRIVE_M:
        return 0.0, why, seen

    # The floor (#3: look as little as possible).
    have = seen.left(pose, now) if seen else 0.0
    if have >= limit + MARGIN_M - 1e-9:
        return limit, why, seen                      # seen clear a moment ago, still ahead
    if zones:                                        # the tracking view's lower rows
        pts = [(x, y, h) for _, _, x, y, h, _ in zones]
        floor = [x for x, y, h in pts if h is not None and abs(y) <= p.corridor_m / 2.0 and x >= p.near_m]
        # Only if it saw the floor right in front, or memory covers the stretch it did not.
        if floor and min(floor) <= max(p.near_m + 2 * p.cell_m, have + 0.05):
            got, _ = _shorten(pts, p, limit)
            if got >= limit - 1e-9 and _seen_to(pts, p) >= limit + MARGIN_M:
                return limit, why, (_Seen(pose, limit + MARGIN_M, now) if pose else None)
    # Glance down (two looks), then put the head back for the next frame.
    back = head.gimbal.angles()
    pts = head.floor_scan()
    geom = getattr(head, "geom", None)
    if not watched and _seen_to(pts, p) < limit + MARGIN_M and hasattr(head, "floor_look"):
        pts = pts + head.floor_look(head.geom.drive_tilt_deg + FAR_LOOK_DEG)    # a long drive: look further out
    head.gimbal.move_to(back[0], back[1], wait=True)
    head.gimbal.wait_settled()
    _remember(hazards, pose, local_plan.hazards_from_floor(
        pts, p, (geom.pan_axis_x_m, geom.height_m) if hasattr(geom, "pan_axis_x_m") else None), person_xy)
    got, short = _shorten(pts, p, limit)
    reach = round(_seen_to(pts, p) - MARGIN_M, 2)    # never drive onto floor it did not measure
    if got > reach and not p.drive_unknown and not watched:
        got, short = (reach if reach >= MIN_DRIVE_M else 0.0), short or f"floor seen only to {reach + MARGIN_M:.2f} m"
    if short:                                        # what it judged by (debug: the raw grid goes to the Mac)
        lv = getattr(head, "last_level", None)
        log.debug("floor seen at tilt %s, pitch %s, levelled by %s (x m, y m, height m): %s",
                 getattr(geom, "drive_tilt_deg", None), head.pitch() if getattr(head, "pitch", None) else None,
                 None if lv is None else f"{lv[0]:+.3f} m, pitch {lv[1]:+.1f}, roll {lv[2]:+.1f} deg",
                 sorted(((round(x, 2), round(y, 2), None if h is None else round(h, 3))
                         for x, y, h in pts if abs(y) <= p.corridor_m / 2.0 and x >= p.near_m),
                        key=lambda t: t[0])[:20])
    if got <= 0.0:
        return 0.0, short or why, None
    clear = got + MARGIN_M                           # and how far it is clear beyond that, for next time
    if short is None:
        clear = next((far for far in (1.7, 1.5, 1.2, 1.0, 0.8, 0.6) if clear < far <= _seen_to(pts, p)
                      and terrain.may_drive(terrain.classify(pts, p, far), p)), clear)
    return got, short or why, (_Seen(pose, clear, now) if pose else None)


def _watching(head) -> bool:
    """Can the floor be watched while driving (the 8x8 ToF on a real Head)?"""
    return bool(head) and getattr(head, "has_grid", False) and hasattr(head, "floor_profile") \
        and hasattr(head, "drive_pose")


def _drive(head, motion, go_m: float, bumped, hazards, pose, person_xy, seq: int) -> bool:
    """Drive `go_m`; True if bumped (as _move). With the 8x8 ToF the head looks down at
    the floor for the whole drive and every new grid (5 a second) is judged out to
    WATCH_REACH_M: the robot brakes once something is within stop_short_m + MARGIN_M,
    so a drive can run on past the floor that was seen before it started."""
    if not _watching(head):
        return _move(motion, motion.drive, go_m, bumped)
    if task_cancel.requested():
        return False
    p, geom = head.terrain, head.geom
    back = head.gimbal.angles()
    head.drive_pose()
    t0 = time.monotonic()
    hit = lambda: bool(bumped) and (bumped() or 0.0) >= t0       # noqa: E731
    try:
        res = motion.drive(go_m)
        if not res.get("ok"):
            raise RuntimeError(f"move refused: {res.get('error')}")
        unseen = 0
        while motion.busy():
            if task_cancel.requested():
                return False
            if hit():
                motion.stop()
                return True
            if time.monotonic() > t0 + MOVE_TIMEOUT_S:
                raise RuntimeError("move timed out")
            pts = head.floor_profile()               # waits for the next grid
            if p.auto_level:
                pts, _ = terrain.level(pts)
            t = terrain.classify(pts, p, WATCH_REACH_M)
            if terrain.may_drive(t, p):
                unseen = 0
                continue
            thin = t.kind == terrain.UNKNOWN and t.distance_m is None
            unseen = unseen + 1 if thin else 0
            near = t.distance_m is not None and t.distance_m <= p.stop_short_m + MARGIN_M
            if (near or unseen >= WATCH_UNSEEN_GRIDS) and motion.busy():
                motion.stop()
                log.info("follow %d: braked mid-drive, %s", seq, terrain.why_not(t))
                status.detail(f"braked: {terrain.why_not(t)}")
                _remember(hazards, pose(), local_plan.hazards_from_floor(
                    pts, p, (geom.pan_axis_x_m, geom.height_m)), person_xy)
                break
        if hit():
            motion.stop()
            return True
        return False
    finally:
        head.gimbal.move_to(back[0], back[1], wait=True)


def _glide_tilt(head) -> float:
    """The head angle while gliding: level (plus FOLLOW_GLIDE_TILT_DEG) in reality, i.e.
    after the head's own upward tilt at its straight position."""
    return settings.FOLLOW_GLIDE_TILT_DEG - getattr(head.geom, "tilt_offset_deg", 0.0)


def _glide_step(head, motion, res: dict, rng, here, pose, hazards, seen, gliding: bool, seq: int):
    """B: one frame of following by steering (motion.velocity). Turn toward the person
    in proportion to their bearing; drive at a speed set by the clear path ahead, judged
    on this frame's grid (the head held level sees the floor 0.5-1.3 m ahead and the
    person both); nothing to drive and only a turn left: turn in place. Returns (gliding,
    seen), or None when the way ahead is blocked: the caller stops and falls back to a
    step (and a way round) this frame."""
    p = head.terrain
    person = res.get("distance_m") or rng
    bearing = res["bearing_deg"]                      # + left, as the head
    w = 0.0 if abs(bearing) < GLIDE_DEADBAND_DEG else \
        max(-GLIDE_MAX_TURN, min(GLIDE_MAX_TURN, GLIDE_TURN_GAIN * math.radians(bearing)))
    v, clear = 0.0, 0.0
    want = min(person - settings.FOLLOW_KEEP_M, settings.FOLLOW_MAX_DRIVE_M) if person else 0.0
    person_xy = None
    if here and person:
        a = here[2] + math.radians(bearing)
        person_xy = (here[0] + person * math.cos(a), here[1] + person * math.sin(a))
    if want >= MIN_DRIVE_M and abs(bearing) <= GLIDE_MAX_BEARING_DEG:
        if not gliding:                               # from standing: the floor right in front first
            go, _, seen = _plan_drive(head, want, [], person, seen, pose(), time.monotonic(), hazards,
                                      person_xy, watched=True)
            if go < MIN_DRIVE_M:
                motion.velocity(0.0, 0.0)
                return None
        zones = _levelled(head.zones(), p)
        pts = [(x, y, h) for _, _, x, y, h, _ in zones]
        _remember(hazards, pose(), local_plan.hazards_from_floor(
            pts, p, (head.geom.pan_axis_x_m, head.geom.height_m)), person_xy)
        clear, why = _shorten(pts, p, want)
        near = _in_the_way(zones, p, person)
        if near is not None:
            clear, why = min(clear, near - p.stop_short_m), f"in the way at {near:.2f} m"
        at = pose()
        if at is not None and hazards is not None and not local_plan.line_clear(
                at[:2], (at[0] + clear * math.cos(at[2]), at[1] + clear * math.sin(at[2])),
                hazards.points(), _radius(p)):
            clear, why = 0.0, "known hazard"
        if clear < MIN_DRIVE_M:
            motion.velocity(0.0, 0.0)
            log.info("follow %d: glide stopped, path %s", seq, why)
            status.detail(f"path {why}")
            return None
        v = min(settings.FOLLOW_GLIDE_MAX_MPS, GLIDE_SPEED_GAIN * clear)
    if v == 0.0:                                      # turning only: on the spot, as a step does
        motion.velocity(0.0, 0.0)
        if abs(bearing) >= GLIDE_SPIN_DEG:
            turn = res.get("turn_deg") or round(-0.6 * bearing, 1)
            log.info("follow %d: glide turn %+.0f deg on the spot", seq, turn)
            _move(motion, motion.turn, turn)
        return False, seen
    motion.velocity(v, w)
    log.info("follow %d: glide %.2f m/s, turn %+.0f deg/s (clear %.2f m)", seq, v, math.degrees(w), clear)
    status.detail(f"following {person or 0:.1f} m")
    return (v > 0.0 or w != 0.0), seen


def _detour(head, motion, hazards, want_m: float, person_xy, pose, bumped, seq: int) -> Optional[_Seen]:
    """The straight drive is blocked: plan a way round what is known (local_plan) and
    drive its first leg, checked like any drive; then face the person again so the next
    frame sees them. Returns the floor now known clear (None)."""
    here = pose()
    if here is None:
        return None
    p = head.terrain
    goal = (here[0] + want_m * math.cos(here[2]), here[1] + want_m * math.sin(here[2]))
    path = local_plan.plan(here[:2], goal, hazards.points(), _radius(p))
    if not path or len(path) < 2:
        log.info("follow %d: no way round (%d hazard points known)", seq, len(hazards.points()))
        status.detail("no way round")
        return None
    if abs(local_plan.leg(here, path[1])[0]) > DETOUR_MAX_TURN_DEG:
        log.info("follow %d: way round starts behind the robot: not taken", seq)
        status.detail("no way round ahead")
        return None
    log.info("follow %d: detour, %d legs via %s", seq, len(path) - 1,
             [(round(x, 2), round(y, 2)) for x, y in path[1:]])
    status.detail("going round")
    for wp in path[1:DETOUR_LEGS + 1]:                # each leg: turn, check the floor, drive
        if task_cancel.requested():
            return None
        at = pose() or here
        turn, dist = local_plan.leg(at, wp)
        if abs(turn) > 3.0:
            _move(motion, motion.turn, round(turn, 1))
        want = round(min(dist, settings.FOLLOW_MAX_DRIVE_M), 2)
        go, short, _ = _plan_drive(head, want, [], None, None, pose(), time.monotonic(), hazards, person_xy,
                                   check_known=False, watched=_watching(head))
        if short:
            log.info("follow %d: detour leg %.2f of %.2f m, path %s", seq, go, want, short)
        if go > 0.0 and _drive(head, motion, go, bumped, hazards, pose, person_xy, seq):
            _remember(hazards, pose(), [(0.3, 0.0)], None)
            break
        if go < want - 0.02:                          # something new on the way round: re-plan next step
            break
    if person_xy is not None:                         # look at them again for the next frame
        back, _ = local_plan.leg(pose() or here, person_xy)
        if abs(back) > 8.0:
            _move(motion, motion.turn, round(back, 1))
    return None


def follow(camera, vision: VisionClient, motion, name: str = "", head=None,
           seconds: float = settings.FOLLOW_SECONDS,
           clock: Callable[[], float] = time.monotonic,
           pose: Optional[Callable[[], Optional[Pose]]] = None,
           bumped: Optional[Callable[[], float]] = None) -> bool:
    """Follow `name` (empty: whichever enrolled person is seen first).

    `pose()`: the robot's (x, y, heading) in the odometry frame, or None (unknown).
    `bumped()`: time.monotonic() of the last bump the drivetrain reported (0: none)."""
    pose = pose or (lambda: None)
    if vision.follow_start(name) is None:
        why = f"{name or 'nobody'} not enrolled"
        log.warning("follow failed: %s", why)
        status.error("F01", why)
        status.detail(why)
        status.result("follow failed")
        return False
    lidar = getattr(head, "lidar", None) if head else None
    if head:
        head.gimbal.move_to(0.0, settings.FOLLOW_TILT_DEG, wait=True)
        head.gimbal.wait_settled()
    status.timer(f"follow {name}".strip(), seconds)
    res: dict = {}
    why = ""                                          # why the follow failed, if it did
    seen: Optional[_Seen] = None                      # floor known clear (#3)
    last_seen: Optional[Tuple[float, float]] = None   # (heading of the person in the odom frame, when) (#5)
    owed_turn = 0.0                                   # degrees still to turn away after a bump (#7)
    hazards = local_plan.Hazards() if head and getattr(head, "has_grid", False) else None   # what blocked drives
    glide = settings.FOLLOW_GLIDE and _watching(head) and getattr(motion, "can_glide", lambda: False)()
    gliding, glide_t = False, 0.0                     # moving under motion.velocity right now, since
    tracker = PersonTracker(settings.FOLLOW_KEEP_M, settings.FOLLOW_MAX_DRIVE_M)   # the Pi decides the moves
    if glide:
        log.info("follow: gliding (velocity mode)")
    t0, seq = clock(), 0
    try:
        while clock() - t0 < seconds and not task_cancel.requested():
            frame = camera.capture(head.gimbal) if head else camera.capture()
            here = pose()                             # where the robot is for this frame
            seq += 1
            grid = None
            if lidar is not None and hasattr(lidar, "grid"):     # 8x8 ToF: the zones on the person
                h, w = frame.main.shape[:2]
                grid = lidar.grid()
                rng: Optional[float] = range_in_box(grid, res.get("bbox"), w, h, head.geom.hfov_deg)
            else:
                rng = lidar.read(n=5, timeout=0.5) if lidar else None
            hint = None                               # where they were last seen, from here (+ left)
            if last_seen and here and time.monotonic() - last_seen[1] < LAST_SEEN_S:
                hint = round(math.degrees(_wrap(last_seen[0] - here[2])), 1)
            img = jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE)
            debug_upload.offer_jpeg(getattr(frame, "seq", 0), img)   # debug mode: sent as is, no second encode
            res = vision.follow_step(
                img, seq,
                pan_deg=frame.pan_deg, tilt_deg=frame.tilt_deg,
                hfov_deg=head.geom.hfov_deg if head else 66.0,
                camera_height_m=head.geom.height_m if head else 0.25, range_m=rng,
                tof_grid=grid.rows if grid is not None else None, hint_deg=hint)
            log.info("follow %d: %s %s, bearing %s, %s m (%s), turn %s, drive %s", seq, res["state"],
                     res.get("target") or "-", res.get("bearing_deg"), res.get("distance_m"),
                     res.get("distance_from"), res.get("turn_deg"), res.get("drive_m"))
            status.detail(f"{res['state']} {res.get('distance_m') or ''}")
            if res["state"] == "gone":
                break
            if res["state"] == "tracking" and here and res.get("bearing_deg") is not None:
                last_seen = (here[2] + math.radians(res["bearing_deg"]), time.monotonic())
                d = res.get("distance_m") or rng
                if d and res.get("distance_from") != "near":
                    tracker.update(here, res["bearing_deg"], d, res.get("distance_sigma_m"), time.monotonic())
                    mv = None if glide else tracker.move(pose() or here)   # gliding steers by itself
                    if mv:                            # the Pi's turn and leg, from the tracked position
                        res = {**res, "turn_deg": mv[0], "drive_m": mv[1], "distance_m": mv[2]}
                        log.info("follow %d: tracked at %.2f m: turn %+.0f, drive %.2f", seq, mv[2], mv[0], mv[1])
            # The Mac drives only to 1.2 m from them; the Pi goes on to FOLLOW_KEEP_M (same 25 deg rule).
            near_m, drive_m = res.get("distance_m") or rng, res["drive_m"]
            if res["state"] == "tracking" and near_m and abs(res.get("bearing_deg") or 0.0) <= 25.0 \
                    and near_m - settings.FOLLOW_KEEP_M >= MIN_DRIVE_M:
                drive_m = max(drive_m, round(min(near_m - settings.FOLLOW_KEEP_M, settings.FOLLOW_MAX_DRIVE_M), 2))
            zones = []                                # the tracking view, for the path check (#3, #4)
            if drive_m and head and getattr(head, "has_grid", False):
                zones = _levelled(head.zones(), head.terrain)
            look = res.get("head")
            if head and look:
                tracking = res.get("locked") and res["state"] in ("tracking", "holding")
                base = settings.FOLLOW_TILT_DEG if not tracking else settings.FOLLOW_TRACK_TILT_DEG
                if glide and tracking:
                    base = _glide_tilt(head)
                fast = getattr(head.gimbal, "fast_dps", None) if res["state"] == "searching" else None
                head.gimbal.move_to(look[0], base + look[1], speed_dps=fast, wait=True)   # search: quicker, still ramped
                head.gimbal.wait_settled()
            if glide and gliding and bumped and (bumped() or 0.0) >= glide_t:
                motion.velocity(0.0, 0.0)             # hit something the ToF did not see (#7)
                log.warning("follow %d: bumped while gliding; stopped", seq)
                status.detail("bumped: something unseen")
                _remember(hazards, pose(), [(0.3, 0.0)], None)
                gliding, seen, owed_turn = False, None, FOLLOW_BUMP_TURN_DEG
            if glide and res["state"] == "tracking" and res.get("locked") and res.get("bearing_deg") is not None \
                    and owed_turn <= 0.0:
                done = _glide_step(head, motion, res, rng, here, pose, hazards, seen, gliding, seq)
                if done is not None:
                    if done[0] and not gliding:
                        glide_t = time.monotonic()
                    gliding, seen = done
                    continue
                gliding = False                       # blocked: a step (and a way round) this frame
            elif glide and gliding:
                motion.velocity(0.0, 0.0)             # lost them, or turning away after a bump: stop
                gliding = False
            if res["turn_deg"]:
                _move(motion, motion.turn, res["turn_deg"])
                owed_turn = max(0.0, owed_turn - abs(res["turn_deg"]))
            if drive_m and owed_turn > 0.0:
                log.info("follow %d: drive skipped, bumped here: turn %.0f deg more first", seq, owed_turn)
                status.detail("bumped: turn away first")
            elif drive_m:
                person = res.get("distance_m") or rng
                want = drive_m                        # the Mac caps a step at 0.5 m: go up to keep distance
                if person:
                    want = round(max(want, min(person - settings.FOLLOW_KEEP_M, settings.FOLLOW_MAX_DRIVE_M)), 2)
                person_xy = None                      # where they stand, in the odometry frame
                if here and person and res.get("bearing_deg") is not None:
                    a = here[2] + math.radians(res["bearing_deg"])
                    person_xy = (here[0] + person * math.cos(a), here[1] + person * math.sin(a))
                go, short, seen = _plan_drive(head, want, _turned(zones, res["turn_deg"]),
                                              person, seen, pose(), time.monotonic(), hazards, person_xy,
                                              watched=_watching(head))
                if short:
                    log.info("follow %d: drive %.2f of %.2f m, path %s", seq, go, want, short)
                    status.detail(f"path {short}")
                if go > 0.0 and _drive(head, motion, go, bumped, hazards, pose, person_xy, seq):
                    log.warning("follow %d: bumped into something the ToF did not see; stopped", seq)
                    status.detail("bumped: something unseen")
                    seen, owed_turn = None, FOLLOW_BUMP_TURN_DEG
                    _remember(hazards, pose(), [(0.3, 0.0)], None)
                elif go <= 0.0 and short and hazards is not None:
                    seen = _detour(head, motion, hazards, want, person_xy, pose, bumped, seq)
    except (VisionError, RuntimeError) as exc:       # RuntimeError: camera or drivetrain gave up
        log.warning("follow ended: %s", exc)
        why = f"vision error: {exc}" if isinstance(exc, VisionError) else str(exc)
    finally:
        if glide:
            motion.velocity(0.0, 0.0)
        if motion.busy():
            motion.stop()
        try:
            vision.follow_stop()
        except VisionError:
            pass
    followed = bool(res.get("followed"))
    if task_cancel.requested():
        log.info("follow cancelled (%s)", task_cancel.by())
        status.result("follow cancelled")
        return followed
    if not followed:
        if not why:
            who = name or "anyone"
            why = (f"searched, {who} not found" if res.get("state") == "gone"
                   else f"time up, {who} not found" if res else "no answer from vision")
        log.warning("follow failed: %s", why)
        status.error("F01", why)
        status.detail(why)
    status.result(f"followed {res.get('target')}" if followed else "follow failed")
    return followed
