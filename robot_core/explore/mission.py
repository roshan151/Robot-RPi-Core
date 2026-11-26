"""The explore mission, as one readable sequential loop.

    ┌─► SCAN      stop; 4 × (sweep TF-Luna ±45°, 2 camera looks, turn 90°)
    │     │       → one 360° keyframe + any plants the detector saw
    │   LOCALISE  POST keyframe → Mac matches it into the map, returns the
    │     │       corrected pose and a path to the nearest unexplored edge
    │   PLANTS    for each plant sighting: where is it on the map? seen before?
    │     │         new/known → ORBIT: 3-4 viewpoints around it, a photo from
    │     │         each → enroll (new plant) or add a visit (known plant)
    │   DRIVE     follow the path ≤ max_leg_m, probing the floor before every
    └─────┘       step; stop early on an obstacle, drop, or stuck wheels
    done when the Mac says there is nothing reachable left to explore.

With the 8x8 ToF on the head (docs/TERRAIN-DESIGN.md) the scan is different and
gathers more: the robot turns once (180°), and at each of nine head positions per
half the grid gives eight bearings at once: the nearest thing in the robot's way,
low obstacles a level beam would pass over, places where the floor falls away,
and how far the floor was seen with nothing on it (free space). All of that, and
every single reading as a 3-D point, goes to the Mac with the keyframe and onto
the map. Plants are looked for twice per camera position, level and looking up,
so a plant stand's upper shelves are seen. Each is ranged by the grid zones on
its pot and placed in three dimensions: where on the map, and how high its pot
stands above the floor. Two plants at the same spot on different shelves are two
plants. Each is measured (width and height) and named by the image+text model
once photographed.

Before EVERY drive, in either scan mode, the floor ahead is checked
(`Head.floor_probe`); the robot only moves onto floor it has seen to be flat.

Everything hardware- or network-shaped is injected (`Base`, `Head`,
`VisionClient`), so this whole file runs in the tests against fakes. It is
deliberately blocking and single-threaded: the robot is slow by design, and a
mission you can read top to bottom is one you can debug from a log.

Frames: odom is the wheels' drifting frame; map is SLAM's. The Mac returns
map_T_odom with every keyframe, and every map position here is
map_T_odom ⊕ odom_T_base ⊕ point.
"""

from __future__ import annotations

import concurrent.futures
import logging
import math
import threading
import time
from dataclasses import dataclass, field, replace
from types import SimpleNamespace
from typing import Callable, List, Optional, Protocol, Sequence, Tuple

from robot_core import debug_upload, local_plan, status as screen, terrain
from robot_core.odometry import Pose2D, wrap
from robot_core.sensors.camera import jpeg
from robot_core.sensors.tof8x8 import range_in_box
from robot_core.vision_client import VisionError

VISION_RETRIES, VISION_RETRY_S = 6, 10.0    # ~1 min of a silent Mac before explore stops (Nov 23: one 10 s timeout did)

log = logging.getLogger(__name__)

DETOUR_MARGIN_M = 0.35   # going round: floor must be seen clear this far past each leg
WATCH_REACH_M = 0.8      # a watched leg: the floor judged this far ahead on every grid
WATCH_MARGIN_M = 0.2     # ...and the robot brakes once a hazard is within stop_short_m + this
WATCH_UNSEEN = 3         # ...or this many grids in a row are too thin to judge
ROOMY_M = 0.3            # a leg passing at least this far from anything mapped is driven whole
PLANT_MAX_RANGE_M = 8.0  # a plant the ToF gave no range on is somewhere on its bearing, up to this far
APPROACH_STEP_M = 2.5    # ...and is approached along that bearing this far between looks
MIN_CROSS_DEG = 10.0     # two bearings fix a plant where they cross, if they cross this steeply


class Base(Protocol):
    def odom(self) -> Pose2D: ...
    def drive(self, meters: float, watch=None) -> str: ...   # done|stuck|drop|blocked|cancelled|failed|timeout
    def turn_ccw(self, radians: float) -> str: ...
    def set_map_to_odom(self, pose: Pose2D) -> None: ...
    def relocalized(self) -> None: ...


@dataclass
class ExploreConfig:
    # auto: join whatever named or pending map this run overlaps (on the Mac), else become a new
    # pending map; continue: must join a named map within 3 scans, else stop.
    mode: str = "auto"
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # where we start on this run's working map
    max_minutes: float = 60.0             # per session: a run that hits it carries on next time
    explore_radius_m: float = 0.0         # only explore within this radius of the start (<= 0: no limit)
    no_behind: bool = False               # only explore ahead of the start heading
    sector_pans: Tuple[float, ...] = (-30.0, 30.0)    # camera looks per 90° sector
    # ToF anchor scan: 5 overlapping 60°-wide looks per half (35° step = 25° overlap, full 200° swath).
    grid_pans: Tuple[float, ...] = (-70.0, -35.0, 0.0, 35.0, 70.0)
    scan_tilt_deg: float = -10.0                      # real tilt during ToF anchor scans: steeper floor angle on glossy wood
    camera_pans: Tuple[float, ...] = (-70.0, 0.0, 70.0)   # 80° HFOV: 3 looks cover 220° per half
    camera_tilts: Tuple[float, ...] = (0.0, 30.0)     # ...level, then looking up (when scan_up_during_map=True)
    scan_up_during_map: bool = False                  # skip upward shelf tilt during Phase A fast mapping
    orbit_after_map: bool = True                      # Phase A: map whole space cleanly; Phase B: visit/orbit plants
    anchor_spacing_m: float = 2.5                     # glide this far with LocalNav + rolling frontiers between 360° anchors
    scan_stream_step_m: float = 0.35                  # fold continuous ToF scan into Mac map every 0.35 m driven
    scan_stream_turn_deg: float = 25.0                # ...or every 25° turned
    same_level_m: float = 0.15            # pots whose bases differ by more than this are on different shelves
    plant_classes: Tuple[str, ...] = (
        "potted plant", "plant", "shrub", "bush", "flower",
        "garden bed", "houseplant", "fern", "succulent", "tree",
    )
    plant_min_conf: float = 0.45
    orbit_radius_m: float = 0.7
    orbit_views: int = 8
    min_views: int = 3
    same_place_m: float = 0.6
    pot_radius_m: float = 0.08            # the beam hits the pot's near side; centre is further
    ambiguous_margin: float = 0.12        # below threshold by less than this → ask the VLM
    revisit_known: bool = True            # photograph known plants again (growth tracking)
    max_leg_m: float = 1.5                # discrete-fallback re-scan distance (LocalNav uses anchor_spacing_m)
    probe_step_m: float = 0.6             # floor-probe look-ahead per discrete drive step
    stop_short_m: float = 0.15            # gap to leave in front of an obstacle
    front_m: float = 0.12                 # MEASURE: front bumper ahead of the wheel axle
    ask_before_drive: bool = False        # VLM "is the way clear?" before each step (slow)
    photo_quality: int = 92


@dataclass
class Sighting:
    sector_odom: Pose2D
    local_xy: Tuple[float, float]          # base_link at sector_odom
    crop: bytes
    confidence: float
    map_xy: Optional[Tuple[float, float]] = None
    size_m: Optional[Tuple[float, float]] = None   # width, height as seen (ToF range x angular size)
    z_m: Optional[float] = None            # the pot's base above the floor (0 = standing on the floor); None = not measured
    top_m: Optional[float] = None          # the top of the plant above the floor
    level: int = 0                         # which shelf, counted up from the lowest plant at this spot
    on_stand: bool = False                 # another plant shares this spot at another height
    ray_from: Optional[Tuple[float, float]] = None   # bearing only (no ToF range): the ray starts here
                                           # (base_link at sector_odom); local_xy is its far end


@dataclass
class Stats:
    keyframes: int = 0
    plants_new: List[str] = field(default_factory=list)
    plants_revisited: List[str] = field(default_factory=list)
    obstacles: int = 0
    stuck: int = 0
    low_obstacles: int = 0
    drops: int = 0


class Explorer:
    def __init__(self, base: Base, head, vision, cfg: ExploreConfig = ExploreConfig(),
                 on_status: Callable[[str], None] = lambda s: None) -> None:
        self.base, self.head, self.vision, self.cfg = base, head, vision, cfg
        self.status = on_status
        self.m2o = Pose2D(*cfg.start_pose)
        self.stats = Stats()
        self._handled: List[Tuple[Tuple[float, float], Optional[float]]] = []   # (map xy, pot height)
        self._pending_plants: List[Sighting] = []             # bearing-only sightings, placed in Phase B
        self._held: List[Sighting] = []                       # seen before the Mac knows which map this is
        self._scope = Pose2D(*cfg.start_pose)                 # start_pose on the current map (moves on a merge)
        self._kf_clear: list = []                     # this keyframe's free-space rays and 3-D points (ToF scan)
        self._kf_cloud: list = []
        self._plant = ""                              # plant_NN on the OLED
        self._obstacles_odom: List[Tuple[float, float]] = []
        self._low_odom: List[Tuple[float, float]] = []       # low obstacles seen by the ToF
        self._drops_odom: List[Tuple[float, float]] = []     # places where the floor falls away
        self._hazards = local_plan.Hazards()                 # what blocked steps, for going round (local_plan)
        self._last_online_odom: Optional[Pose2D] = None      # last pose where vision server answered (for return-to-signal)
        self._recon_pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="explore-recon")
        self._stop = threading.Event()

    # ================================================================ loop

    def stop(self) -> None:
        self._stop.set()

    def _wait_odom_fresh(self, max_wait_s: float = 8.0) -> bool:
        """Pause cleanly if /odom went stale (e.g. Arduino DTR reset takes ~2-4 s)
        instead of driving blind or aborting the mission."""
        age_fn = getattr(self.base, "odom_age", None)
        if age_fn is None or age_fn() <= 1.5:
            return True
        self._say("odometry paused — waiting for drivetrain telemetry to resume")
        deadline = time.monotonic() + max_wait_s
        while not self._stop.is_set() and time.monotonic() < deadline:
            if age_fn() <= 1.0:
                self._say("odometry resumed")
                return True
            self._stop.wait(0.25)
        return age_fn() <= 1.5

    def _return_to_signal(self, max_step_m: float = 0.8) -> None:
        """Backtrack toward the last pose where the Mac answered so an outdoor Wi-Fi dead zone
        does not strand the robot."""
        if self._last_online_odom is None or self._stop.is_set():
            return
        cur = self.base.odom()
        dx, dy = self._last_online_odom.x - cur.x, self._last_online_odom.y - cur.y
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            return
        step = min(dist, max_step_m)
        target_th = math.atan2(dy, dx)
        turn_rad = wrap(target_th - cur.theta)
        self._say(f"Wi-Fi lost: backtracking {step:.2f} m toward last online pose")
        if abs(turn_rad) > math.radians(10.0):
            self.base.turn_ccw(turn_rad)
        if not self._stop.is_set():
            self.base.drive(step)

    def _say(self, msg: str) -> None:
        log.info("explore: %s", msg)
        o = self.base.odom()
        debug_upload.offer_event(None, {"task": "explore", "msg": msg, "pose_now": [o.x, o.y, o.theta]})
        screen.detail(msg)
        self.status(msg)

    def _can_continuous(self) -> bool:
        """True when 8x8 ToF + velocity firmware (-vel) are available to drive with LocalNav."""
        return bool(getattr(self.head, "has_grid", False)) and bool(
            getattr(self.base, "can_glide", lambda: False)())

    def run(self) -> Stats:
        cfg = self.cfg
        deadline = time.monotonic() + cfg.max_minutes * 60
        scope = dict(explore_radius_m=cfg.explore_radius_m, no_behind=cfg.no_behind)
        try:
            self.vision.slam_reset(cfg.start_pose, mode=cfg.mode, **scope)
        except VisionError as exc:
            # HTTP 409: too many pending maps (delete some in the web app first), or the Mac is down.
            self._say(f"cannot start explore: {exc}")
            screen.detail("delete pending maps" if "409" in str(exc) else "vision service down")
            self.head.home()
            return self.stats
        fails = 0
        try:
            # --- Phase A: Map the entire reachable space cleanly ---
            while not self._stop.is_set() and time.monotonic() < deadline:
                if not self._wait_odom_fresh():
                    self._say("odometry did not recover — stopping explore safely")
                    break
                try:
                    self._say("scanning")
                    kf_odom, rays, sightings = self.scan_keyframe()
                    extra = {k: self._in_frame(kf_odom, pts) for k, pts in
                             (("low", self._low_odom), ("drops", self._drops_odom)) if pts}
                    if self._kf_clear:
                        extra["clear"] = self._kf_clear
                    if self._kf_cloud:
                        extra["cloud"] = self._kf_cloud
                    res = self.vision.slam_keyframe(
                        kf_odom.as_tuple(), rays, self.head.sensor_xy, self._obstacles_in(kf_odom), **extra)
                    for pts in (self._obstacles_odom, self._low_odom, self._drops_odom):
                        pts.clear()
                    self._last_online_odom = kf_odom
                    if hasattr(self.vision, "flush_spool"):
                        self.vision.flush_spool()
                    self.stats.keyframes += 1
                    self.m2o = Pose2D(*res["map_to_odom"])
                    self.base.set_map_to_odom(self.m2o)
                    if res.get("merged"):
                        mg = res["merged"]
                        self._scope = Pose2D(*mg["transform"]).compose(self._scope)
                        self._say(f"on map '{mg['survivor']}' now ({mg['folded']} joined it, score {mg['score']:.2f})")
                    if res.get("relocalize_failed"):
                        self._say("I can't find where I am on the map: stopping (continue mode)")
                        screen.detail("not on a known map")
                        self._stop.set()
                        break
                    if res["match"]["accepted"]:
                        self.base.relocalized()
                    loop_note = f", LOOP closed (kf {res['loop']['with_kf']})" if res.get("loop", {}).get("closed") else ""
                    self._say(f"keyframe {res['kf_id']}: match {res['match']['score']:.2f} "
                              f"({'ok' if res['match']['accepted'] else 'odometry only'}), "
                              f"{res.get('explored_m2', 0):.1f} m² explored{loop_note}")

                    if sightings:
                        if cfg.orbit_after_map:
                            self._held.extend(sightings)
                            if res.get("settled", True):
                                self._register_held()
                        else:
                            o0 = self.base.odom()
                            for s in self._dedupe(sightings):
                                if self._stop.is_set():
                                    break
                                self.handle_plant(s)
                            o1 = self.base.odom()
                            if math.hypot(o1.x - o0.x, o1.y - o0.y) > 0.15:
                                continue

                    if res.get("done"):
                        self._say("mapping complete — nothing reachable left unexplored")
                        break

                    goal = res["goal"]
                    leg_lim = cfg.anchor_spacing_m if self._can_continuous() else cfg.max_leg_m
                    outcome = self.follow(goal["path"], leg_lim, goal.get("clear_m"), rolling_frontier=True)
                    self._say(f"drive: {outcome}")

                    # If we couldn't reach this frontier, blacklist it temporarily so the planner
                    # picks the next-best frontier instead of retrying the same blocked alcove.
                    if (outcome.startswith("blocked") or outcome in ("stuck", "drop", "timeout")) \
                            and hasattr(self.vision, "slam_unreachable"):
                        try:
                            p_now = self.map_pose()
                            self.vision.slam_unreachable(goal["xy"], pose=p_now.as_tuple())
                            self._say(f"marked frontier ({goal['xy'][0]:.2f}, {goal['xy'][1]:.2f}) temporarily unreachable")
                        except VisionError:
                            pass
                    fails = 0
                except VisionError as exc:          # the Mac or the Wi-Fi slow for a while: wait, don't give up
                    fails += 1
                    if fails > VISION_RETRIES:
                        raise
                    self._say(f"vision service not answering ({exc}); retry {fails}/{VISION_RETRIES}")
                    if fails >= 2 and self._last_online_odom is not None:
                        self._return_to_signal()
                    self._stop.wait(VISION_RETRY_S)

            # --- Phase B: Visit & orbit plants on the finished, loop-closed map ---
            if cfg.orbit_after_map and not self._stop.is_set() and time.monotonic() < deadline:
                self._register_held()                 # a short run that never settled: register them now
                self._phase_b(deadline)
        except VisionError as exc:
            self._say(f"vision service unavailable, stopping: {exc}")
        finally:
            # Stop quickly: the Mac already saved the map after every scan; this marks the run ended.
            self._recon_pool.shutdown(wait=False)
            self.head.home()
            try:
                self.vision.slam_save(timeout=3.0)
            except VisionError:
                pass
        self._say(f"finished: {self.stats.keyframes} keyframes, new plants {self.stats.plants_new}, "
                  f"revisited {self.stats.plants_revisited}, {self.stats.low_obstacles} low obstacle and "
                  f"{self.stats.drops} drop points mapped")
        return self.stats

    def _register_held(self) -> None:
        """Register the plants seen so far on the Mac at once (preliminary, anchored to the scan
        they were seen from, so their place follows the map), so a stopped run loses none. Phase B
        photographs them from the Mac's list. Plants seen only as a bearing wait for Phase B."""
        held, self._held = self._held, []
        for s in self._dedupe(held):
            if self._stop.is_set():
                self._held.append(s)
                continue
            if s.ray_from is not None:
                self._pending_plants.append(s)
                continue
            if not self._in_scope(s.map_xy) or any(self._same(s.map_xy, s.z_m, xy, z) for xy, z in self._handled):
                continue
            try:
                where = list(s.map_xy) + ([s.z_m] if s.z_m is not None else [])
                label = self.identify(s, self.vision.plant_match(self.vision.plant_embed(s.crop), where))
                if label is None:
                    continue
                meta = {"position": [round(v, 3) for v in where], "preliminary": True,
                        "anchor": {"odom": [round(v, 4) for v in s.sector_odom.as_tuple()],
                                   "local": [round(v, 4) for v in s.local_xy]},
                        "detector_confidence": round(float(s.confidence), 3)}
                if s.z_m is not None:
                    meta.update(height_m=s.z_m, top_m=s.top_m, on_stand=s.on_stand, level=s.level)
                if s.size_m:
                    meta["size_m"] = list(s.size_m)
                res = self.vision.plant_enroll([s.crop], label, meta)
                self._handled.append((s.map_xy, s.z_m))
                self._say(f"plant at ({s.map_xy[0]:.2f}, {s.map_xy[1]:.2f}) registered as {res['label']}"
                          + (f"; {', '.join(res['stale'])} now stale" if res.get("stale") else ""))
            except VisionError as exc:
                self._say(f"plant registration skipped ({exc})")

    def _phase_b(self, deadline: float) -> None:
        """Photograph from all sides every plant on this map seen since its last all-sides visit.
        The list comes from the Mac, so a run stopped half-way through picks up the rest next time."""
        for s in self._dedupe(self._pending_plants):     # bearing-only sightings: placed by approaching
            if self._stop.is_set() or time.monotonic() >= deadline:
                return
            try:
                self.handle_plant(s)
            except VisionError as exc:
                self._say(f"plant visit skipped ({exc})")
        try:
            waiting = self.vision.plants_waiting()
        except VisionError as exc:
            self._say(f"no plant list from the Mac ({exc})")
            return
        if waiting:
            self._say(f"Phase B: photographing {len(waiting)} plant(s) from all sides")
        for w in waiting:
            if self._stop.is_set() or time.monotonic() >= deadline:
                return
            pos, label = w["position"], w["label"]
            self._plant = label
            z = float(pos[2]) if len(pos) > 2 and pos[2] is not None else None
            try:
                screen.detail(f"{label} starting photos")
                views = self.orbit((float(pos[0]), float(pos[1])),
                                   SimpleNamespace(z_m=z, top_m=None) if z is not None else None)
                if not views:
                    self._say(f"could not reach any viewpoint around {label} — skipping")
                    continue
                meta = {"position": [round(v, 3) for v in pos], "views": [v[1] for v in views]}
                describe = getattr(self.vision, "describe_plant", None)
                about = describe(views[0][0]) if describe else None
                if about:
                    meta["about"] = about
                if not w.get("new"):
                    compare = getattr(self.vision, "compare_plant_health", None)
                    prev_photo = getattr(self.vision, "plant_photo", None)
                    prev_jpg = prev_photo(label) if (compare and prev_photo) else None
                    diff = compare(views[0][0], prev_jpg) if prev_jpg else None
                    if diff:
                        meta["health_diff"] = diff
                res = self.vision.plant_enroll([v[0] for v in views], label, meta)
                (self.stats.plants_new if w.get("new") else self.stats.plants_revisited).append(res["label"])
                screen.detail(f"{res['label']} saved {res['photos_saved']} photos")
                self._say(f"saved {res['photos_saved']} photos as {res['label']} ({res['folder']})")
            except VisionError as exc:
                self._say(f"plant visit skipped ({exc})")

    # ================================================================ scan

    def scan_keyframe(self) -> Tuple[Pose2D, list, List[Sighting]]:
        if getattr(self.head, "has_grid", False):
            return self._scan_grid()
        kf_odom = self.base.odom()
        sx, sy = self.head.sensor_xy
        rays, sightings = [], []
        for k in range(4):
            if self._stop.is_set():
                break
            if k:
                self.base.turn_ccw(math.pi / 2)
            sec = self.base.odom()
            rel = kf_odom.between(sec)            # this sector's pose in the keyframe
            # Alternate sweep direction so the head never swings back empty.
            a, b_ = (-45.0, 45.0) if k % 2 == 0 else (45.0, -45.0)
            for b, r in self.head.sweep(a, b_, self.head.cmd_tilt(0.0)):
                if r is None:
                    continue
                # Re-express the hit from the keyframe's sensor position: turning
                # in place swings the head, so each sector's origin differs a little.
                hit = rel.compose(Pose2D(sx + r * math.cos(b), sy + r * math.sin(b), 0.0))
                rays.append((math.atan2(hit.y - sy, hit.x - sx), math.hypot(hit.x - sx, hit.y - sy)))
            sightings += self.look_for_plants(sec)
        self.head.home()
        return kf_odom, rays, sightings

    def _scan_grid(self) -> Tuple[Pose2D, list, List[Sighting]]:
        """The keyframe with the 8x8 ToF: two halves, one 180° turn between them."""
        cfg, head = self.cfg, self.head
        kf_odom = self.base.odom()
        sx, sy = head.sensor_xy
        rays, clear, cloud, sightings = [], [], [], []
        for k in range(2):
            if self._stop.is_set():
                break
            if k:
                self.base.turn_ccw(math.pi)
            sec = self.base.odom()
            rel = kf_odom.between(sec)            # this half's pose in the keyframe

            def ray(x, y):                        # a point seen from this half, as (bearing, range) in the keyframe
                hit = rel.compose(Pose2D(x, y, 0.0))
                return (math.atan2(hit.y - sy, hit.x - sx), math.hypot(hit.x - sx, hit.y - sy))

            # Alternate direction so the head never swings back empty.
            scan_tilt = head.cmd_tilt(cfg.scan_tilt_deg)
            tilts = cfg.camera_tilts if cfg.scan_up_during_map else (cfg.scan_tilt_deg,)
            for pan in (cfg.grid_pans if k % 2 == 0 else cfg.grid_pans[::-1]):
                if self._stop.is_set():
                    break
                head.gimbal.move_to(pan, scan_tilt, wait=True)    # slightly down: steeper floor incidence on glossy wood
                sl, pts = head.scan_look()
                rays += [ray(x, y) for x, y in sl.rays]
                clear += [ray(x, y) for x, y in sl.clear]
                self._remember_obstacles(sl.low, "low")
                self._remember_obstacles(sl.drops, "drop")
                if pan in cfg.camera_pans and head.camera is not None:
                    for tilt in tilts:
                        if abs(tilt - cfg.scan_tilt_deg) > 1.0:   # extra look up for upper plant shelves when enabled
                            head.gimbal.move_to(pan, head.cmd_tilt(tilt), wait=True)
                            pts += head.cloud()
                        frame = head.camera.capture(head.gimbal)
                        self._recon(frame, sec)       # async upload for the Mac's 3-D mesh (vision_service/recon.py)
                        for d in self._plants_in(frame):
                            s = self._localise_grid(frame, d, sec)
                            if s is not None:
                                sightings.append(s)   # seen more than once (looks overlap): _dedupe sorts that out
                for x, y, z in pts:
                    q = rel.compose(Pose2D(x, y, 0.0))
                    cloud.append((round(q.x, 2), round(q.y, 2), round(z, 2)))
        head.home()
        self._kf_clear, self._kf_cloud = clear, cloud
        return kf_odom, rays, sightings

    def _recon(self, frame, sec_odom: Pose2D) -> None:
        """Send one scan frame asynchronously to the Mac's 3-D reconstruction with everything
        that places it. Non-blocking so HTTP latency never stalls the anchor scan."""
        send = getattr(self.vision, "recon_frame", None)
        if send is None:
            return
        try:
            head, g = self.head, self.head.geom
            grid = head.lidar.grid(newer_than=frame.t_capture, timeout=0.25)
            blob = jpeg(frame.main)
            meta = {
                "odom": list(sec_odom.as_tuple()), "pan_deg": frame.pan_deg,
                "tilt_deg": head.real_tilt(frame.tilt_deg),             # real: degrees above level
                "settled": frame.settled, "pitch_deg": head.pitch() if head.pitch else None,
                "tof_rows": grid.rows if grid is not None else None,
                "geom": {"height_m": g.height_m, "pan_axis_x_m": g.pan_axis_x_m, "pan_axis_y_m": g.pan_axis_y_m,
                         "tilt_offset_deg": g.tilt_offset_deg, "hfov_deg": g.hfov_deg,
                         "tof_pan_offset_deg": g.tof_pan_offset_deg, "tof_tilt_offset_deg": g.tof_tilt_offset_deg,
                         "tof_perpendicular": g.tof_perpendicular},
            }
            self._recon_pool.submit(self._send_recon, send, blob, meta)
        except Exception as exc:                                  # noqa: BLE001 — the mesh is optional
            log.info("recon frame skipped: %s", exc)

    @staticmethod
    def _send_recon(send, blob: bytes, meta: dict) -> None:
        try:
            send(blob, meta)
        except Exception as exc:                                  # noqa: BLE001
            log.info("recon frame upload skipped: %s", exc)

    def _localise_grid(self, frame, det, sec_odom: Pose2D) -> Optional[Sighting]:
        """Where a detected plant is, from the ToF zones on its pot (the lower part
        of the box: leaves let the light through to the wall behind). No re-aiming
        unless the pot is outside the ToF's view, which is narrower than the camera's."""
        head = self.head
        h, w = frame.main.shape[:2]

        def pot(d):
            x1, y1, x2, y2 = d["bbox"]
            return (x1, y1 + 0.5 * (y2 - y1), x2, y2)

        g = head.geom
        rng = range_in_box(head.lidar.grid(newer_than=time.monotonic()), pot(det), w, h, g.hfov_deg,
                           g.tof_pan_offset_deg, g.tof_tilt_offset_deg)
        if rng is None:
            x1, y1, x2, y2 = det["bbox"]
            px, py = (x1 + x2) / 2.0, y1 + 0.7 * (y2 - y1)
            f_px = head.fx(w)
            ang_left = math.degrees(math.atan2(w / 2.0 - px, f_px)) - g.tof_pan_offset_deg
            ang_up = math.degrees(math.atan2(h / 2.0 - py, f_px)) - g.tof_tilt_offset_deg
            # Only slew & re-detect if the pot lay outside the 60 deg ToF FOV (wide 80 deg camera edge).
            # If it was already inside the ToF FOV and returned None, it is simply out of ToF range:
            # record its bearing immediately in 0 ms instead of burning ~2 s re-aiming.
            if abs(ang_left) <= 27.0 and abs(ang_up) <= 27.0:
                return self._bearing_only(frame, det, sec_odom)
            head.aim_at_pixel(frame, px, py)
            head.gimbal.wait_settled()
            f2 = head.camera.capture(head.gimbal)
            dets = self._plants_in(f2)
            if not dets:
                return self._bearing_only(frame, det, sec_odom)
            det = min(dets, key=lambda d: abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2))
            frame = f2
            rng = range_in_box(head.lidar.grid(newer_than=time.monotonic()), pot(det), w, h, g.hfov_deg,
                               g.tof_pan_offset_deg, g.tof_tilt_offset_deg)
            if rng is None:
                return self._bearing_only(frame, det, sec_odom)
        x1, y1, x2, y2 = det["bbox"]
        f = head.fx(w)
        left = math.atan2(w / 2.0 - (x1 + x2) / 2.0, f)           # the pot's bearing in the view, + left
        # A perpendicular range is depth along the view axis; a radial one is along the ray.
        depth = rng if head.geom.tof_perpendicular else rng * math.cos(left)
        # Real elevation of the view: the head's tilt (it looks tilt_offset_deg up at 0) plus the robot's
        # nose-up pitch, which tilts the view by pitch x cos(pan) (as Head.project, small angles).
        pitch = (head.pitch() if head.pitch else None) or 0.0
        tilt = math.radians(head.real_tilt(frame.tilt_deg) + pitch * math.cos(math.radians(frame.pan_deg)))

        def at(v):
            """Image row v at the pot's depth -> (metres ahead along the floor, height above the floor)."""
            up = depth * (h / 2.0 - v) / f
            return (depth * math.cos(tilt) - up * math.sin(tilt),
                    head.geom.height_m + depth * math.sin(tilt) + up * math.cos(tilt))

        ahead, _ = at(y1 + 0.75 * (y2 - y1))                      # the pot, in the lower part of the box
        side = depth * math.tan(left)
        reach = math.hypot(ahead, side) + self.cfg.pot_radius_m   # to the pot's centre, along its bearing
        b = math.radians(frame.pan_deg) + math.atan2(side, max(ahead, 1e-6))
        centre = (head.geom.pan_axis_x_m + reach * math.cos(b), reach * math.sin(b))
        # Heights, from the box's lower and upper edges. An edge on the picture's border is
        # the picture's edge, not the plant's: then that height is not known from this look
        # (unless the picture's lower border is itself at the floor: then the pot is on it).
        base = at(y2)[1]
        if y2 >= h - 2:
            base = 0.0 if base < 0.10 else None
        else:
            base = 0.0 if base < 0.05 else round(base, 2)         # under 5 cm: standing on the floor
        top = None if y1 <= 1 else round(at(y1)[1], 2)
        size = (round(depth * (x2 - x1) / f, 2), round(depth * (y2 - y1) / f, 2))
        return Sighting(sec_odom, centre, _crop(frame.main, det["bbox"]), det["confidence"], size_m=size,
                        z_m=base, top_m=top)

    def _bearing_only(self, frame, det, sec_odom: Pose2D) -> Sighting:
        """A plant the ToF gave no range on (too far, a dark pot): keep its bearing instead of
        dropping it (Nov 25: every plant of a run was dropped). It is on that ray, up to
        PLANT_MAX_RANGE_M; a second bearing from elsewhere (_fuse_bearings) or a ToF range from
        closer (_approach) puts it on the map."""
        h, w = frame.main.shape[:2]
        x1, _, x2, _ = det["bbox"]
        b = math.radians(frame.pan_deg) + math.atan2(w / 2.0 - (x1 + x2) / 2.0, self.head.fx(w))
        ax = self.head.geom.pan_axis_x_m
        log.info("plant seen but no ToF range on its pot — keeping its bearing (%.0f deg), up to %.0f m",
                 math.degrees(b), PLANT_MAX_RANGE_M)
        return Sighting(sec_odom, (ax + PLANT_MAX_RANGE_M * math.cos(b), PLANT_MAX_RANGE_M * math.sin(b)),
                        _crop(frame.main, det["bbox"]), det["confidence"], ray_from=(ax, 0.0))

    def _map_xy(self, s: Sighting) -> Tuple[float, float]:
        p = self.m2o.compose(s.sector_odom).compose(Pose2D(*s.local_xy, 0.0))
        return (p.x, p.y)

    def _ray(self, s: Sighting):
        """A bearing-only sighting as (origin, heading) on the map."""
        base = self.m2o.compose(s.sector_odom)
        a, b = base.compose(Pose2D(*s.ray_from, 0.0)), base.compose(Pose2D(*s.local_xy, 0.0))
        return (a.x, a.y), math.atan2(b.y - a.y, b.x - a.x)

    def _near_ray(self, ray, xy) -> bool:
        (ox, oy), th = ray
        dx, dy = xy[0] - ox, xy[1] - oy
        along = dx * math.cos(th) + dy * math.sin(th)
        return -0.3 <= along <= PLANT_MAX_RANGE_M + self.cfg.same_place_m \
            and abs(dy * math.cos(th) - dx * math.sin(th)) <= self.cfg.same_place_m

    def _placed(self, s: Sighting, xy) -> Sighting:
        """s, now known to stand at map `xy`."""
        loc = self.m2o.compose(s.sector_odom).inverse().compose(Pose2D(xy[0], xy[1], 0.0))
        return replace(s, local_xy=(loc.x, loc.y), ray_from=None)

    def _fuse_bearings(self, sightings: List[Sighting]) -> List[Sighting]:
        """Bearing-only sightings: one whose ray passes a placed plant is that plant (dropped);
        two that cross from different spots fix the plant there (odometry + bearings); the rest
        stay bearing-only, for handle_plant to approach."""
        out = [s for s in sightings if s.ray_from is None]
        if len(out) == len(sightings):
            return sightings

        def explained(r: Sighting) -> bool:
            return any(o.ray_from is None and self._near_ray(self._ray(r), self._map_xy(o)) for o in out)

        def same_bearing(ra, rb) -> bool:
            (ax, ay), ta = ra
            (bx, by), tb = rb
            if abs(wrap(ta - tb)) >= math.radians(10.0):
                return False
            dx, dy = bx - ax, by - ay
            perp = abs(dy * math.cos(ta) - dx * math.sin(ta))
            return perp <= self.cfg.same_place_m

        pending = [s for s in sightings if s.ray_from is not None and not explained(s)]
        while pending:
            a = pending.pop(0)
            ra = self._ray(a)
            hit = next(((b, xy) for b in pending for xy in [_cross(ra, self._ray(b))] if xy), None)
            if hit is None:
                # Merge near-parallel duplicate bearings of the same distant plant (keep highest confidence)
                dups = [b for b in pending if same_bearing(ra, self._ray(b))]
                best = max([a] + dups, key=lambda o: o.confidence)
                out.append(best)
                if dups:
                    dup_ids = {id(b) for b in dups}
                    pending = [r for r in pending if id(r) not in dup_ids]
                continue
            b, xy = hit
            out.append(self._placed(max(a, b, key=lambda o: o.confidence), xy))
            self._say(f"plant placed at ({xy[0]:.2f}, {xy[1]:.2f}) from two bearings")
            pending = [r for r in pending if r is not b and not explained(r)]
        return out

    def _look_along(self, s: Sighting, far) -> Optional[Sighting]:
        """Face the bearing's far end and look: a plant on the ray with a ToF range, or a
        second bearing crossing it, is where s is. None if neither."""
        self.face(far)
        frame = self.head.look(0.0, self.head.cmd_tilt(0.0))
        ray = self._ray(s)
        for det in sorted(self._plants_in(frame), key=lambda d: -d["confidence"]):
            s2 = self._localise_grid(frame, det, self.base.odom())
            if s2 is None:
                continue
            if s2.ray_from is None:
                if self._near_ray(ray, self._map_xy(s2)):
                    return s2
            else:
                xy = _cross(ray, self._ray(s2))
                if xy is not None:
                    return self._placed(max(s, s2, key=lambda o: o.confidence), xy)
        return None

    def _approach(self, s: Sighting) -> bool:
        """A plant known only by its bearing: look from here (a new bearing may fix it), then
        drive along the bearing APPROACH_STEP_M at a time, looking again, until the ToF ranges it.
        Every step shortens how far it can still be (odometry); past the far end there was no
        plant there (or no way to it). True if the robot moved."""
        (ox, oy), th = self._ray(s)
        far = (ox + PLANT_MAX_RANGE_M * math.cos(th), oy + PLANT_MAX_RANGE_M * math.sin(th))
        moved = False
        for _ in range(int(PLANT_MAX_RANGE_M / APPROACH_STEP_M) + 2):
            if self._stop.is_set():
                return moved
            fixed = self._look_along(s, far)
            if fixed is not None:
                return self.handle_plant(self._dedupe([fixed])[0]) or moved
            here = self.map_pose()
            left = _dist((here.x, here.y), far)
            step = min(APPROACH_STEP_M, left - 1.0)
            if step < 0.3:
                break
            # Probe shorter fractions along the ray if the full step lands inside a wall or beyond the mapped room.
            path = None
            chosen = step
            for frac in (1.0, 0.65, 0.35):
                cand = step * frac
                if cand < 0.3:
                    continue
                k = cand / left
                p_try = self.vision.slam_plan((here.x, here.y), (here.x + k * (far[0] - here.x),
                                                                 here.y + k * (far[1] - here.y)))
                if p_try:
                    path, chosen = p_try, cand
                    break
            if not path:
                break
            moved = True
            if self.follow(path, chosen) not in ("arrived", "leg_limit"):
                break
        self._say(f"plant seen at bearing {math.degrees(th):.0f}° never got a range — skipping")
        return moved

    def look_for_plants(self, sec_odom: Pose2D) -> List[Sighting]:
        cfg, head = self.cfg, self.head
        if getattr(head, "camera", None) is None:
            return []
        found: List[Sighting] = []
        pans_done: List[float] = []
        for pan in cfg.sector_pans:
            frame = head.look(pan, head.cmd_tilt(0.0))
            for d in self._plants_in(frame):
                x1, y1, x2, y2 = d["bbox"]
                bpan, _ = head.pixel_angles(frame, (x1 + x2) / 2, (y1 + y2) / 2)
                # Outside ±45° belongs to the neighbouring sector, which will
                # see it closer to centre (and the pan servo can't reach it).
                if abs(bpan) > 45.0 or any(abs(bpan - p) < 8.0 for p in pans_done):
                    continue
                pans_done.append(bpan)
                s = self._localise(frame, d, sec_odom)
                if s is not None:
                    found.append(s)
        return found

    def _in_scope(self, xy) -> bool:
        """Same rule the Mac applies to frontiers (slam.in_scope), for plants."""
        cfg = self.cfg
        x, y, th = self._scope.as_tuple()
        dx, dy = xy[0] - x, xy[1] - y
        if cfg.explore_radius_m > 0 and math.hypot(dx, dy) > cfg.explore_radius_m:
            return False
        return not (cfg.no_behind and dx * math.cos(th) + dy * math.sin(th) < 0.0)

    def _plants_in(self, frame) -> list:
        res = self.vision.detect(jpeg(frame.main), frame.seq, self.cfg.plant_min_conf)
        h, w = frame.main.shape[:2]
        sx = w / res["image_size"][0]
        out = []
        for d in res["detections"]:
            if d["class"] in self.cfg.plant_classes and d["confidence"] >= self.cfg.plant_min_conf:
                d = dict(d, bbox=[v * sx for v in d["bbox"]])      # back to full-res pixels
                out.append(d)
        return out

    def _localise(self, frame, det, sec_odom: Pose2D) -> Optional[Sighting]:
        """Aim at the pot (lower part of the box — leaves let the beam through
        to the wall behind), refine once with a fresh detection, then range it."""
        head = self.head
        x1, y1, x2, y2 = det["bbox"]
        head.aim_at_pixel(frame, (x1 + x2) / 2, y1 + 0.7 * (y2 - y1))
        f2 = head.camera.capture(head.gimbal)
        dets = self._plants_in(f2)
        if dets:
            h, w = f2.main.shape[:2]
            best = min(dets, key=lambda d: abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2))
            x1, y1, x2, y2 = best["bbox"]
            head.aim_at_pixel(f2, (x1 + x2) / 2, y1 + 0.7 * (y2 - y1))
            frame = f2
            det = best
        rng = head.range_at()
        if rng is None:
            log.info("plant seen but the rangefinder got no return — skipping")
            return None
        pan, tilt = head.gimbal.angles()
        crop = _crop(frame.main, det["bbox"])
        centre = head.point_at(pan, tilt, rng + self.cfg.pot_radius_m)
        return Sighting(sec_odom, centre, crop, det["confidence"])

    def _same(self, a_xy, a_z, b_xy, b_z) -> bool:
        """The same plant: the same spot on the map and, where both heights were
        measured, the same shelf. A plant stand is one spot and several plants."""
        if _dist(a_xy, b_xy) > self.cfg.same_place_m:
            return False
        return a_z is None or b_z is None or abs(a_z - b_z) <= self.cfg.same_level_m

    def _dedupe(self, sightings: List[Sighting]) -> List[Sighting]:
        sightings = self._fuse_bearings(sightings)
        out: List[Sighting] = []
        # Sightings with a measured height first: one without (its pot was out of the
        # picture) then joins the plant it belongs to, instead of hiding a shelf.
        for s in sorted(sightings, key=lambda s: (s.z_m is None, -s.confidence)):
            p = self.m2o.compose(s.sector_odom).compose(Pose2D(*s.local_xy, 0.0))
            s.map_xy = (p.x, p.y)
            same = next((o for o in out if self._same(s.map_xy, s.z_m, o.map_xy, o.z_m)), None)
            if same is None:
                out.append(s)
            elif same.top_m is None and s.top_m is not None:
                same.top_m = s.top_m                 # the look upward saw the top the level look cut off
        for s in out:                                # shelves: the plants sharing a spot, lowest first
            stack = sorted((o for o in out if _dist(o.map_xy, s.map_xy) <= self.cfg.same_place_m),
                           key=lambda o: o.z_m or 0.0)
            s.on_stand, s.level = len(stack) > 1, stack.index(s)
        return out

    # ============================================================== plants

    def handle_plant(self, s: Sighting) -> bool:
        """Identify, orbit and save one plant. True if the robot moved."""
        if s.ray_from is not None:
            return self._approach(s)                 # not placed yet: only its bearing is known
        if any(self._same(s.map_xy, s.z_m, xy, z) for xy, z in self._handled):
            return False                             # already dealt with this run
        if not self._in_scope(s.map_xy):
            return False                             # outside the radius / behind the start
        self._handled.append((s.map_xy, s.z_m))
        self._plant = f"plant_{len(self._handled):02d}"
        where = list(s.map_xy) + ([s.z_m] if s.z_m is not None else [])
        match = self.vision.plant_match(self.vision.plant_embed(s.crop), where)
        label = self.identify(s, match)
        if label is None:
            return False
        up = f", {s.z_m:.2f} m up" if s.z_m else ""
        self._say(f"plant at ({s.map_xy[0]:.2f}, {s.map_xy[1]:.2f}){up}: "
                  f"{'new' if not label else label} — photographing it from all sides")
        screen.detail(f"{self._plant} {'new' if not label else label}, starting photos")
        views = self.orbit(s.map_xy, s)
        if not views:
            self._say("could not reach any viewpoint around the plant — skipping")
            return True
        # position: x, y on the map and, when measured, the height of the pot's base above the floor.
        meta = {"position": [round(v, 3) for v in where],
                "views": [v[1] for v in views],
                "detector_confidence": s.confidence}
        if s.z_m is not None:
            meta.update(height_m=s.z_m, top_m=s.top_m, on_stand=s.on_stand, level=s.level)
        if s.size_m:
            meta["size_m"] = list(s.size_m)             # width, height as first seen
        describe = getattr(self.vision, "describe_plant", None)
        about = describe(views[0][0]) if describe else None     # what plant it is, from the image+text model
        if about:
            meta["about"] = about
            screen.detail(f"{self._plant} {about.get('name', '')}")
        if label:
            compare = getattr(self.vision, "compare_plant_health", None)
            prev_photo = getattr(self.vision, "plant_photo", None)
            prev_jpg = prev_photo(label) if (compare and prev_photo) else None
            diff = compare(views[0][0], prev_jpg) if prev_jpg else None
            if diff:
                meta["health_diff"] = diff
        res = self.vision.plant_enroll([v[0] for v in views], label, meta)
        (self.stats.plants_revisited if label else self.stats.plants_new).append(res["label"])
        screen.detail(f"{res['label']} saved {res['photos_saved']} photos")
        self._say(f"saved {res['photos_saved']} photos as {res['label']} ({res['folder']})")
        return True

    def identify(self, s: Sighting, match: dict) -> Optional[str]:
        """'' = new plant, a label = known plant, None = known and nothing to do."""
        cfg = self.cfg
        if not match["matches"]:
            return ""

        def here(m) -> bool:
            """Known to stand where this one does: same spot and, where measured, same shelf."""
            pos = m.get("position")
            if pos:
                return self._same(s.map_xy, s.z_m, pos[:2], pos[2] if len(pos) > 2 else None)
            return m.get("distance_m") is not None and m["distance_m"] <= cfg.same_place_m

        # Among look-alikes (the same kind of plant on each shelf of a stand), the one that stands here.
        best = next((m for m in match["matches"] if here(m)), match["matches"][0])
        thr = match["threshold"]
        near = here(best)
        label: Optional[str] = ""
        if best["similarity"] >= thr and near:
            label = best["label"]
        elif best["similarity"] >= thr - cfg.ambiguous_margin:
            # Looks similar but somewhere else (moved? a sibling?), or in the
            # right place but not quite the same look (rotated? grown?): the
            # embedding can't settle it, so ask the image+text model.
            before = self.vision.plant_photo(best["label"])
            verdict = self.vision.same_plant(s.crop, [before]) if before else None
            if verdict is None:
                verdict = near                        # no VLM: trust the map
            label = best["label"] if verdict else ""
        if label and not cfg.revisit_known:
            return None
        return label

    def orbit(self, target: Tuple[float, float], s: Optional[Sighting] = None) -> List[Tuple[bytes, dict]]:
        cfg = self.cfg
        here = self.map_pose()
        vps = self.vision.slam_viewpoints((here.x, here.y), target, cfg.orbit_radius_m, cfg.orbit_views)
        if len(vps) < cfg.min_views:
            wider = self.vision.slam_viewpoints((here.x, here.y), target,
                                                cfg.orbit_radius_m * 1.4, cfg.orbit_views)
            vps = wider if len(wider) > len(vps) else vps
        views = []
        for i, vp in enumerate(vps, 1):
            if self._stop.is_set():
                break
            screen.detail(f"{self._plant} moving to view {i}/{len(vps)}")
            for attempt in range(2):
                now = self.map_pose()
                path = self.vision.slam_plan((now.x, now.y), vp["pose"][:2])
                st = "no path" if path is None else self.follow(path, max_dist=float("inf"))
                if st == "arrived" or path is None or st.startswith("blocked") or self._stop.is_set():
                    break
                if attempt == 0:                      # stuck on a turn or a leg: back off, re-plan once
                    self._say(f"viewpoint {i}/{len(vps)}: {st}, backing off to try again")
                    self.base.drive(-0.15)
            if st != "arrived":
                self._say(f"viewpoint {i}/{len(vps)} unreachable ({st}) — skipping")
                continue
            self.face(target)
            screen.detail(f"{self._plant} capturing {i}/{len(vps)}")
            views.append(self.photograph(target, vp.get("angle_deg"), s))
        return views

    def photograph(self, target, angle_deg, s: Optional[Sighting] = None) -> Tuple[bytes, dict]:
        """One photo of the plant from here. With its height known (`s`), the head
        looks up to that shelf and, of the plants in view, takes the one on it."""
        head = self.head
        z = s.z_m if s is not None else None
        tilt0, want = -5.0, None
        if z is not None:
            p = self.map_pose()
            dist = max(0.2, math.hypot(target[0] - p.x, target[1] - p.y) - head.geom.pan_axis_x_m)
            mid = (z + (s.top_m if s.top_m is not None else z + 0.3)) / 2.0
            tilt0 = math.degrees(math.atan2(mid - head.geom.height_m, dist))
            want = math.degrees(math.atan2(z - head.geom.height_m, dist))      # where its pot's base should appear
        frame = head.look(0.0, head.cmd_tilt(tilt0))
        dets = self._plants_in(frame)
        if dets:
            h, w = frame.main.shape[:2]
            f = head.fx(w)

            def off(d) -> float:
                """How far this box is from the plant we came for: sideways, and (if known) in height."""
                side = abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2) / w
                if want is None:
                    return side
                base = head.real_tilt(frame.tilt_deg) + math.degrees(math.atan2(h / 2.0 - d["bbox"][3], f))
                return side + abs(base - want) / 30.0

            best = min(dets, key=off)
            x1, y1, x2, y2 = best["bbox"]
            head.aim_at_pixel(frame, (x1 + x2) / 2, (y1 + y2) / 2)
            head.gimbal.wait_settled()
            frame = head.camera.capture(head.gimbal)
        pose = self.map_pose()
        meta = {"pose": [round(pose.x, 3), round(pose.y, 3), round(pose.theta, 4)],
                "angle_deg": angle_deg, "pan_deg": round(frame.pan_deg, 1),
                "tilt_deg": round(head.real_tilt(frame.tilt_deg), 1), "range_m": head.range_at(),   # real: degrees above level
                "detected": bool(dets)}
        return jpeg(frame.main, long_edge=None, quality=self.cfg.photo_quality), meta

    # ================================================================ motion

    def map_pose(self) -> Pose2D:
        return self.m2o.compose(self.base.odom())

    def face(self, xy) -> str:
        p = self.map_pose()
        dth = wrap(math.atan2(xy[1] - p.y, xy[0] - p.x) - p.theta)
        return self.base.turn_ccw(dth) if abs(dth) > math.radians(3) else "done"

    def _watching(self) -> bool:
        return bool(getattr(self.head, "has_grid", False)) and hasattr(self.head, "floor_profile") \
            and hasattr(self.head, "drive_pose")

    def _watch(self):
        """A watch for base.drive: one ToF grid per call with the head in its drive pose;
        True (brake) once a hazard is within stop_short_m + WATCH_MARGIN_M, or the floor
        cannot be judged for WATCH_UNSEEN grids running. What it saw is kept for planning."""
        p, unseen = self.head.terrain, [0]

        def watch() -> bool:
            pts = self.head.floor_profile()
            if getattr(p, "auto_level", False):
                pts, _ = terrain.level(pts)
            t = terrain.classify(pts, p, WATCH_REACH_M)
            if terrain.may_drive(t, p):
                unseen[0] = 0
                return False
            unseen[0] = unseen[0] + 1 if t.kind == terrain.UNKNOWN and t.distance_m is None else 0
            near = t.distance_m is not None and t.distance_m <= self.cfg.stop_short_m + WATCH_MARGIN_M
            if near or unseen[0] >= WATCH_UNSEEN:
                geom = self.head.geom
                hz = local_plan.hazards_from_floor(pts, p, (geom.pan_axis_x_m, geom.height_m))
                if hz:
                    self._hazards.add(self.base.odom().as_tuple(), hz)
                    self._remember_obstacles(hz, "drop" if t.kind == terrain.DROP else "obstacle")
                self._say(f"braked mid-leg: {terrain.why_not(t)}")
                return True
            return False
        return watch

    def _stream_scan(self, odom_pose: Pose2D, plan: bool = False) -> Optional[dict]:
        """Fold the newest in-transit ToF grid into the Mac's SLAM map without stopping.
        When `plan=True`, also returns the Mac's updated frontier goal so LocalNav can
        keep gliding continuously instead of stopping after short open-floor steps."""
        send = getattr(self.vision, "slam_scan", None)
        if send is None or not getattr(self.head, "has_grid", False):
            return None
        try:
            grid = self.head.lidar.latest()
            if grid is None or not getattr(grid, "rows", None):
                return None
            angle_at = getattr(self.head.gimbal, "angle_at", None)
            pan, tilt = angle_at(grid.t) if angle_at else self.head.gimbal.angles()
            zones = self.head.project(grid.rows, pan, tilt)
            if not zones:
                return None
            sl = terrain.scan_slice(zones, self.head.terrain)
            sx, sy = self.head.sensor_xy
            rays = [(math.atan2(y - sy, x - sx), math.hypot(x - sx, y - sy)) for x, y in sl.rays]
            clear = [(math.atan2(y - sy, x - sx), math.hypot(x - sx, y - sy)) for x, y in sl.clear]
            if not rays and not clear:
                return None
            return send(odom_pose.as_tuple(), rays, (sx, sy), plan=plan,
                        low=sl.low, drops=sl.drops, clear=clear)
        except Exception as exc:                                  # noqa: BLE001 — best effort in transit
            log.debug("in-transit slam_scan skipped: %s", exc)
            return None

    def _follow_continuous(self, path: Sequence[Sequence[float]], max_dist: float,
                           rolling_frontier: bool = False) -> str:
        """Drive smoothly toward the goal using LocalNav (DWA + RollingCostmap + 50 Hz
        reflex + smart-arc escape), streaming continuous ToF scans to the Mac.
        When `rolling_frontier=True` (Phase A mapping), advances the frontier goal on the
        fly via `slam_scan(..., plan=True)` so the robot glides up to `max_dist` (~2.5 m)
        continuously instead of stopping after a 0.25-0.50 m hardwood step."""
        from robot_core.nav.controller import LocalNav

        cfg = self.cfg
        pts = list(path)
        if len(pts) < 2:
            return "arrived"
        goal_map = (float(pts[-1][0]), float(pts[-1][1]))
        inv_m2o = self.m2o.inverse()
        goal_odom_p = inv_m2o.compose(Pose2D(goal_map[0], goal_map[1], 0.0))
        goal_odom = (goal_odom_p.x, goal_odom_p.y)

        age_fn = getattr(self.base, "odom_age", None)

        def pose_odom():
            if age_fn is not None and age_fn() > 1.5:
                return None
            return self.base.odom().as_tuple()

        def mac_planner(p_odom, g_odom, _local, target=None):
            pm = self.m2o.compose(Pose2D(*p_odom))
            gm = self.m2o.compose(Pose2D(g_odom[0], g_odom[1], 0.0))
            m_path = self.vision.slam_plan((pm.x, pm.y), (gm.x, gm.y))
            if not m_path:
                return None
            inv = self.m2o.inverse()
            return [(inv.compose(Pose2D(x, y, 0.0)).x, inv.compose(Pose2D(x, y, 0.0)).y) for x, y in m_path]

        nav_tilt = self.head.cmd_tilt(-8.0)
        nav = LocalNav(
            self.head, self.base, pose_odom,
            twist=getattr(self.base, "twist", None),
            bumped=getattr(self.base, "bumped", None),
            planner=mac_planner,
            tilt_deg=nav_tilt,
        )
        from robot_core import settings as robot_settings
        if robot_settings.NAV_GAP_HINT:
            nav.gap_check = getattr(self.vision, "nav_gap", None)

        def _seed_map_path(m_pts) -> Tuple[float, float]:
            inv = self.m2o.inverse()
            wps = [(inv.compose(Pose2D(x, y, 0.0)).x, inv.compose(Pose2D(x, y, 0.0)).y) for x, y in m_pts]
            g_o = wps[-1]
            nav.waypoints = wps
            nav.wp_goal = g_o
            nav._wp_t = nav.clock()
            return g_o

        goal_odom = _seed_map_path(pts)

        nav.start()
        try:
            start_o = self.base.odom()
            prev_o = start_o
            last_scan_o = start_o
            last_scan_t = time.monotonic()
            prog_o = start_o
            prog_t = time.monotonic()
            travelled = 0.0
            max_leg_s = max(25.0, max_dist / 0.08 + 15.0)
            leg_deadline = time.monotonic() + max_leg_s
            outcome = "arrived"

            def _try_advance_frontier(cur_o: Pose2D) -> Tuple[bool, bool]:
                """Stream current ToF scan with plan=True and advance goal_odom if a further
                frontier exists. Returns (advanced, map_done)."""
                nonlocal goal_odom, last_scan_o, last_scan_t
                res = self._stream_scan(cur_o, plan=rolling_frontier)
                last_scan_o, last_scan_t = cur_o, time.monotonic()
                if not res or not rolling_frontier:
                    return False, False
                if res.get("done"):
                    return False, True
                new_goal = res.get("goal")
                if not new_goal or not new_goal.get("path"):
                    return False, False
                cand_o = self.m2o.inverse().compose(Pose2D(new_goal["xy"][0], new_goal["xy"][1], 0.0))
                if math.hypot(cand_o.x - cur_o.x, cand_o.y - cur_o.y) > 0.35:
                    goal_odom = _seed_map_path(new_goal["path"])
                    return True, False
                return False, False

            while not self._stop.is_set():
                now = time.monotonic()
                if now >= leg_deadline:
                    outcome = "timeout"
                    break
                if age_fn is not None and age_fn() > 1.5:
                    nav.set_goal(None)
                    if not self._wait_odom_fresh(max_wait_s=8.0):
                        outcome = "timeout"
                        break
                    prog_t = time.monotonic()
                    continue

                cur_o = self.base.odom()
                step_d = math.hypot(cur_o.x - prev_o.x, cur_o.y - prev_o.y)
                travelled += step_d
                prev_o = cur_o

                # Keep renewing goal in odom frame
                nav.set_goal(goal_odom, keep_m=0.18, face=False)

                if travelled >= max_dist:
                    outcome = "leg_limit"
                    break

                # Continuous ToF scan fold into the Mac map while moving (and roll frontier forward)
                rem = math.hypot(goal_odom[0] - cur_o.x, goal_odom[1] - cur_o.y)
                moved_since_scan = math.hypot(cur_o.x - last_scan_o.x, cur_o.y - last_scan_o.y)
                turned_since_scan = abs(math.degrees(wrap(cur_o.theta - last_scan_o.theta)))
                near_goal = nav.arrived() or rem <= 0.35
                if (near_goal
                        or moved_since_scan >= cfg.scan_stream_step_m
                        or turned_since_scan >= cfg.scan_stream_turn_deg
                        or now - last_scan_t >= 2.0):
                    advanced, map_done = _try_advance_frontier(cur_o)
                    if map_done:
                        outcome = "arrived"
                        break
                    if advanced:
                        nav.set_goal(goal_odom, keep_m=0.18, face=False)
                        rem = math.hypot(goal_odom[0] - cur_o.x, goal_odom[1] - cur_o.y)

                # Stop only if we reached the goal AND rolling pursuit had no further frontier
                if (nav.arrived() or rem <= 0.25) and not (rolling_frontier and rem > 0.35):
                    outcome = "arrived"
                    break

                if math.hypot(cur_o.x - prog_o.x, cur_o.y - prog_o.y) >= 0.12:
                    prog_o, prog_t = cur_o, now
                elif now - prog_t >= 10.0:
                    outcome = f"blocked ({nav.state})"
                    break

                self._stop.wait(0.15)

            if self._stop.is_set():
                outcome = "stopped"
            # Final in-transit scan before stopping for anchor scan
            if outcome in ("arrived", "leg_limit"):
                self._stream_scan(self.base.odom(), plan=False)
            return outcome
        finally:
            nav.stop()
            try:
                self.base.velocity(0.0, 0.0)
            except Exception:
                pass

    def follow(self, path: Sequence[Sequence[float]], max_dist: float,
               clear: Optional[Sequence[float]] = None, rolling_frontier: bool = False) -> str:
        """Drive a map path toward a waypoint. Uses continuous LocalNav (DWA + reflex +
        live ToF map streaming) when 8x8 ToF + velocity firmware are available, falling
        back to discrete turn-probe-drive steps otherwise."""
        if self._can_continuous() and not self.cfg.ask_before_drive:
            return self._follow_continuous(path, max_dist, rolling_frontier=rolling_frontier)
        cfg = self.cfg
        travelled = 0.0
        watched = self._watching()
        for i, wp in enumerate(list(path)[1:], start=1):
            roomy = watched and clear is not None and i < len(clear) and clear[i] >= ROOMY_M
            for _ in range(8):                        # bounded re-aims per waypoint
                if self._stop.is_set():
                    return "stopped"
                p = self.map_pose()
                dist = math.hypot(wp[0] - p.x, wp[1] - p.y)
                if dist < 0.08:
                    break
                st = self.face(wp)
                if st != "done":
                    return st
                step = min(dist, max_dist - travelled) if roomy else min(dist, cfg.probe_step_m, max_dist - travelled)
                if step <= 0.05:
                    return "leg_limit"
                probe = self.head.floor_probe(min(step, cfg.probe_step_m) + cfg.stop_short_m)
                if probe.status in ("obstacle", "drop", "unseen"):
                    # unseen: a floor the ToF cannot read. Marked like an obstacle so the next
                    # path goes another way; never driven (robot.yaml drive_on_unknown_floor).
                    self._remember_obstacles(probe.points, "drop" if probe.status == "drop" else "obstacle")
                    self._add_hazards(probe)
                    if self._go_round(wp):                # a way round, without waiting for the next scan
                        continue
                    room = (probe.distance_m or 0.0) - cfg.front_m - cfg.stop_short_m
                    if probe.status == "obstacle" and room > 0.1:     # close up to an obstacle; never toward a drop
                        self.base.drive(min(room, step))
                    return f"blocked ({probe.status})"
                if cfg.ask_before_drive:
                    view = self.head.look(0.0, self.head.cmd_tilt(-10.0))
                    if self.vision.path_clear(jpeg(view.main), step) is False:
                        self._remember_obstacles([(self.head.sensor_xy[0] + step, 0.0)])
                        return "blocked (vlm)"
                if roomy and step > cfg.probe_step_m:
                    self.head.drive_pose()
                    p0 = self.base.odom()
                    st = self.base.drive(step, watch=self._watch())
                    p1 = self.base.odom()
                    travelled += math.hypot(p1.x - p0.x, p1.y - p0.y)
                    if st == "blocked":
                        roomy = False                 # short probed steps from here: it is seen and gone round
                        continue
                else:
                    st = self.base.drive(step)
                    travelled += step
                if st == "drop":                      # the TF-Luna guard braked: a drop just ahead
                    self.stats.drops += 1
                    self._remember_obstacles([(0.35, 0.0)], "drop")
                    self.base.drive(-0.15)
                    return "drop"
                if st == "stuck":
                    self.stats.stuck += 1
                    self._remember_obstacles([(0.25, 0.0)])
                    self.base.drive(-0.15)
                    return "stuck"
                if st != "done":
                    return st
                if travelled >= max_dist - 0.05:
                    return "leg_limit"
        return "arrived"

    def _add_hazards(self, probe) -> None:
        """What a blocked probe found, kept for planning round it: the whole floor scan's
        hazards when the head has the 8x8 ToF, else the probe's own points."""
        pts, p = getattr(self.head, "last_floor", None), getattr(self.head, "terrain", None)
        geom = getattr(self.head, "geom", None)
        hz = local_plan.hazards_from_floor(pts, p, (geom.pan_axis_x_m, geom.height_m)) if pts and p else []
        if not hz:                                    # a single-beam probe: its points, and behind an obstacle
            hz = list(probe.points) + (local_plan.fill_behind(probe.points) if probe.status == "obstacle" else [])
        self._hazards.add(self.base.odom().as_tuple(), hz)

    def _go_round(self, wp) -> bool:
        """A step toward map waypoint `wp` is blocked: plan round what is known
        (local_plan, in the odometry frame) and drive up to three legs of the way round,
        each probed first. True if it moved; the caller then re-aims at `wp`."""
        o = self.base.odom()
        goal = self.m2o.inverse().compose(Pose2D(wp[0], wp[1], 0.0))
        p = getattr(self.head, "terrain", None)
        path = local_plan.plan((o.x, o.y), (goal.x, goal.y), self._hazards.points(),
                               local_plan.radius(p) if p else 0.25)
        if not path or len(path) < 2:
            self._say("no way round here")
            return False
        self._say(f"going round: {len(path) - 1} legs")
        moved = False
        for to in path[1:4]:
            if self._stop.is_set():
                break
            turn, dist = local_plan.leg(self.base.odom().as_tuple(), to)
            if abs(turn) > 3.0 and self.base.turn_ccw(math.radians(-turn)) != "done":
                break
            step = min(dist, self.cfg.probe_step_m)
            # Clear floor well past the leg's end: short legs must not creep up to an edge.
            probe = self.head.floor_probe(step + max(self.cfg.stop_short_m, DETOUR_MARGIN_M))
            if probe.status in ("obstacle", "drop", "unseen"):
                self._remember_obstacles(probe.points, "drop" if probe.status == "drop" else "obstacle")
                self._add_hazards(probe)
                break
            if self.base.drive(step) != "done":
                break
            moved = True
            if step < dist - 0.02:                    # a long leg: the waypoint loop carries on from here
                break
        return moved

    def _remember_obstacles(self, pts_base, kind: str = "obstacle") -> None:
        """Points in base_link, seen from where the robot stands now, kept for the
        next keyframe. kind: obstacle (floor probe) | low (ToF: below knee height) | drop."""
        o = self.base.odom()
        store = {"low": self._low_odom, "drop": self._drops_odom}.get(kind, self._obstacles_odom)
        for x, y in pts_base:
            q = o.compose(Pose2D(x, y, 0.0))
            store.append((q.x, q.y))
        if kind == "low":
            self.stats.low_obstacles += len(pts_base)
        elif kind == "drop":
            self.stats.drops += len(pts_base)
        else:
            self.stats.obstacles += len(pts_base)

    @staticmethod
    def _in_frame(kf_odom: Pose2D, pts_odom) -> list:
        inv = kf_odom.inverse()
        return [[round(v, 3) for v in inv.compose(Pose2D(x, y, 0.0)).as_tuple()[:2]] for x, y in pts_odom]

    def _obstacles_in(self, kf_odom: Pose2D) -> list:
        inv = kf_odom.inverse()
        return [inv.compose(Pose2D(x, y, 0.0)).as_tuple()[:2] for x, y in self._obstacles_odom]


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _cross(r1, r2) -> Optional[Tuple[float, float]]:
    """Where two bearings ((x, y), heading) meet: in front of both, within PLANT_MAX_RANGE_M,
    and crossing at least MIN_CROSS_DEG (nearly parallel bearings fix nothing). Else None."""
    (o1, t1), (o2, t2) = r1, r2
    d1, d2 = (math.cos(t1), math.sin(t1)), (math.cos(t2), math.sin(t2))
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < math.sin(math.radians(MIN_CROSS_DEG)):
        return None
    wx, wy = o2[0] - o1[0], o2[1] - o1[1]
    a = (wx * d2[1] - wy * d2[0]) / den
    b = (wx * d1[1] - wy * d1[0]) / den
    if not (0.3 <= a <= PLANT_MAX_RANGE_M and 0.3 <= b <= PLANT_MAX_RANGE_M):
        return None
    return (o1[0] + a * d1[0], o1[1] + a * d1[1])


def _crop(img, bbox, pad: float = 0.1) -> bytes:
    h, w = img.shape[:2]
    x1, y1, x2, y2 = bbox
    px, py = (x2 - x1) * pad, (y2 - y1) * pad
    x1, y1 = int(max(0, x1 - px)), int(max(0, y1 - py))
    x2, y2 = int(min(w, x2 + px)), int(min(h, y2 + py))
    return jpeg(img[y1:y2, x1:x2], long_edge=448)
