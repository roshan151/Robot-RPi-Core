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

from robot_core import settings, status, task_cancel, terrain
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

Pose = Tuple[float, float, float]   # x, y (m), heading (rad, CCW) in the odometry frame


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
    if not zones or person_m is None:
        return None
    xs = [x for _, _, x, y, h, d in zones
          if h is not None and x > p.near_m and abs(y) <= p.corridor_m / 2.0
          and p.h_max_m < h < p.body_m + 0.05 and d < person_m - 0.3]
    return min(xs) if xs else None


def _plan_drive(head, want_m: float, zones, person_m: Optional[float], seen: Optional[_Seen],
                pose: Optional[Pose], now: float) -> Tuple[float, Optional[str], Optional[_Seen]]:
    """How far to drive of the Mac's `want_m`: (metres, why short or None, floor now
    known clear). `zones` is the tracking view, already turned to the heading the drive
    starts from. Without the 8x8 ToF: all of it, unchecked, as before."""
    if not head or not getattr(head, "has_grid", False) or task_cancel.requested():
        return want_m, None, seen
    p = head.terrain
    limit, why = want_m, None
    near = _in_the_way(zones, p, person_m)           # anything up to the robot's height (#4)
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
            if got >= limit - 1e-9:
                return limit, why, (_Seen(pose, limit + MARGIN_M, now) if pose else None)
    # Glance down (two looks), then put the head back for the next frame.
    back = head.gimbal.angles()
    pts = head.floor_scan()
    head.gimbal.move_to(back[0], back[1], wait=True)
    head.gimbal.wait_settled()
    got, short = _shorten(pts, p, limit)
    if got <= 0.0:
        return 0.0, short or why, None
    clear = got + MARGIN_M                           # and how far it is clear beyond that, for next time
    if short is None:
        clear = next((far for far in (1.0, 0.8, 0.6) if far > clear
                      and terrain.may_drive(terrain.classify(pts, p, far), p)), clear)
    return got, short or why, (_Seen(pose, clear, now) if pose else None)


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
            res = vision.follow_step(
                jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE), seq,
                pan_deg=frame.pan_deg, tilt_deg=frame.tilt_deg,
                hfov_deg=head.geom.hfov_deg if head else 66.0,
                camera_height_m=head.geom.height_m if head else 0.25, range_m=rng,
                tof_grid=grid.rows if grid is not None else None, hint_deg=hint)
            log.info("follow %d: %s", seq, res)
            status.detail(f"{res['state']} {res.get('distance_m') or ''}")
            if res["state"] == "gone":
                break
            if res["state"] == "tracking" and here and res.get("bearing_deg") is not None:
                last_seen = (here[2] + math.radians(res["bearing_deg"]), time.monotonic())
            zones = []                                # the tracking view, for the path check (#3, #4)
            if res["drive_m"] and head and getattr(head, "has_grid", False):
                zones = head.zones()
            look = res.get("head")
            if head and look:
                tracking = res.get("locked") and res["state"] in ("tracking", "holding")
                base = settings.FOLLOW_TILT_DEG if not tracking else settings.FOLLOW_TRACK_TILT_DEG
                fast = getattr(head.gimbal, "fast_dps", None) if res["state"] == "searching" else None
                head.gimbal.move_to(look[0], base + look[1], speed_dps=fast, wait=True)   # search: quicker, still ramped
                head.gimbal.wait_settled()
            if res["turn_deg"]:
                _move(motion, motion.turn, res["turn_deg"])
                owed_turn = max(0.0, owed_turn - abs(res["turn_deg"]))
            if res["drive_m"] and owed_turn > 0.0:
                log.info("follow %d: drive skipped, bumped here: turn %.0f deg more first", seq, owed_turn)
                status.detail("bumped: turn away first")
            elif res["drive_m"]:
                person = res.get("distance_m") or rng
                go, short, seen = _plan_drive(head, res["drive_m"], _turned(zones, res["turn_deg"]),
                                              person, seen, pose(), time.monotonic())
                if short:
                    log.info("follow %d: drive %.2f of %.2f m, path %s", seq, go, res["drive_m"], short)
                    status.detail(f"path {short}")
                if go > 0.0 and _move(motion, motion.drive, go, bumped):
                    log.warning("follow %d: bumped into something the ToF did not see; stopped", seq)
                    status.detail("bumped: something unseen")
                    seen, owed_turn = None, FOLLOW_BUMP_TURN_DEG
    except (VisionError, RuntimeError) as exc:       # RuntimeError: camera or drivetrain gave up
        log.warning("follow ended: %s", exc)
        why = f"vision error: {exc}" if isinstance(exc, VisionError) else str(exc)
    finally:
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
