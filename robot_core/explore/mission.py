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

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Protocol, Sequence, Tuple

from robot_core import local_plan, status as screen
from robot_core.odometry import Pose2D, wrap
from robot_core.sensors.camera import jpeg
from robot_core.sensors.tof8x8 import range_in_box
from robot_core.vision_client import VisionError

log = logging.getLogger(__name__)

DETOUR_MARGIN_M = 0.35   # going round: floor must be seen clear this far past each leg


class Base(Protocol):
    def odom(self) -> Pose2D: ...
    def drive(self, meters: float) -> str: ...          # done|stuck|cancelled|failed|timeout
    def turn_ccw(self, radians: float) -> str: ...
    def set_map_to_odom(self, pose: Pose2D) -> None: ...
    def relocalized(self) -> None: ...


@dataclass
class ExploreConfig:
    map_name: str = "home"
    continue_map: bool = False            # keep building the saved map `map_name`
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # where we start ON that map
    max_minutes: float = 60.0
    explore_radius_m: float = 10.0        # only explore within this radius of the start (<= 0: no limit)
    no_behind: bool = False               # only explore ahead of the start heading
    sector_pans: Tuple[float, ...] = (-30.0, 30.0)    # camera looks per 90° sector
    # ToF scan: head positions per half turn. 18.75° apart is two and a half zones, so
    # neighbouring looks interleave: a bearing every 3.75° across about 200°.
    grid_pans: Tuple[float, ...] = (-75.0, -56.25, -37.5, -18.75, 0.0, 18.75, 37.5, 56.25, 75.0)
    camera_pans: Tuple[float, ...] = (-75.0, -37.5, 0.0, 37.5, 75.0)   # which of them also look for plants
    camera_tilts: Tuple[float, ...] = (0.0, 30.0)     # ...level, then looking up (shelves, plant stands)
    same_level_m: float = 0.15            # pots whose bases differ by more than this are on different shelves
    plant_classes: Tuple[str, ...] = ("potted plant",)
    plant_min_conf: float = 0.45
    orbit_radius_m: float = 0.7
    orbit_views: int = 4
    min_views: int = 3
    same_place_m: float = 0.6
    pot_radius_m: float = 0.08            # the beam hits the pot's near side; centre is further
    ambiguous_margin: float = 0.12        # below threshold by less than this → ask the VLM
    revisit_known: bool = True            # photograph known plants again (growth tracking)
    max_leg_m: float = 1.5                # re-scan after this much driving
    probe_step_m: float = 0.6             # floor-probe look-ahead per drive step
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
        self._kf_clear: list = []                     # this keyframe's free-space rays and 3-D points (ToF scan)
        self._kf_cloud: list = []
        self._plant = ""                              # plant_NN on the OLED
        self._obstacles_odom: List[Tuple[float, float]] = []
        self._low_odom: List[Tuple[float, float]] = []       # low obstacles seen by the ToF
        self._drops_odom: List[Tuple[float, float]] = []     # places where the floor falls away
        self._hazards = local_plan.Hazards()                 # what blocked steps, for going round (local_plan)
        self._stop = threading.Event()

    # ================================================================ loop

    def stop(self) -> None:
        self._stop.set()

    def _say(self, msg: str) -> None:
        log.info("explore: %s", msg)
        screen.detail(msg)
        self.status(msg)

    def run(self) -> Stats:
        cfg = self.cfg
        deadline = time.monotonic() + cfg.max_minutes * 60
        scope = dict(explore_radius_m=cfg.explore_radius_m, no_behind=cfg.no_behind)
        if self.vision.slam_reset(cfg.map_name, cfg.start_pose, load=cfg.continue_map, **scope) is None:
            self._say(f"no saved map '{cfg.map_name}' — starting a new one")
            self.vision.slam_reset(cfg.map_name, cfg.start_pose, load=False, **scope)
        try:
            while not self._stop.is_set() and time.monotonic() < deadline:
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
                self.stats.keyframes += 1
                self.m2o = Pose2D(*res["map_to_odom"])
                self.base.set_map_to_odom(self.m2o)
                if res["match"]["accepted"]:
                    self.base.relocalized()
                self._say(f"keyframe {res['kf_id']}: match {res['match']['score']:.2f} "
                          f"({'ok' if res['match']['accepted'] else 'odometry only'}), "
                          f"{res.get('explored_m2', 0):.1f} m² explored")

                moved = False
                for s in self._dedupe(sightings):
                    if self._stop.is_set():
                        break
                    moved |= self.handle_plant(s)
                if moved:
                    continue      # the orbit moved us: re-scan rather than follow a stale path

                if res.get("done"):
                    self._say("exploration complete — nothing reachable left unexplored")
                    break
                outcome = self.follow(res["goal"]["path"], self.cfg.max_leg_m)
                self._say(f"drive: {outcome}")
        except VisionError as exc:
            self._say(f"vision service unavailable, stopping: {exc}")
        finally:
            try:
                self.vision.slam_save()
            except VisionError:
                pass
            self.head.home()
        self._say(f"finished: {self.stats.keyframes} keyframes, new plants {self.stats.plants_new}, "
                  f"revisited {self.stats.plants_revisited}, {self.stats.low_obstacles} low obstacle and "
                  f"{self.stats.drops} drop points mapped")
        return self.stats

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
            for b, r in self.head.sweep(a, b_, 0.0):
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
            for pan in (cfg.grid_pans if k % 2 == 0 else cfg.grid_pans[::-1]):
                if self._stop.is_set():
                    break
                head.gimbal.move_to(pan, 0.0, wait=True)
                sl, pts = head.scan_look()
                rays += [ray(x, y) for x, y in sl.rays]
                clear += [ray(x, y) for x, y in sl.clear]
                self._remember_obstacles(sl.low, "low")
                self._remember_obstacles(sl.drops, "drop")
                if pan in cfg.camera_pans:
                    for tilt in cfg.camera_tilts:
                        if tilt:                      # look up: shelves, the upper pots of a plant stand
                            head.gimbal.move_to(pan, tilt, wait=True)
                            pts += head.cloud()
                        frame = head.camera.capture(head.gimbal)
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

    def _localise_grid(self, frame, det, sec_odom: Pose2D) -> Optional[Sighting]:
        """Where a detected plant is, from the ToF zones on its pot (the lower part
        of the box: leaves let the light through to the wall behind). No re-aiming
        unless the pot is outside the ToF's view, which is narrower than the camera's."""
        head = self.head
        h, w = frame.main.shape[:2]

        def pot(d):
            x1, y1, x2, y2 = d["bbox"]
            return (x1, y1 + 0.5 * (y2 - y1), x2, y2)

        rng = range_in_box(head.lidar.grid(newer_than=time.monotonic()), pot(det), w, h, head.geom.hfov_deg)
        if rng is None:
            x1, y1, x2, y2 = det["bbox"]
            head.aim_at_pixel(frame, (x1 + x2) / 2, y1 + 0.7 * (y2 - y1))
            head.gimbal.wait_settled()
            f2 = head.camera.capture(head.gimbal)
            dets = self._plants_in(f2)
            if not dets:
                return None
            det = min(dets, key=lambda d: abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2))
            frame = f2
            rng = range_in_box(head.lidar.grid(newer_than=time.monotonic()), pot(det), w, h, head.geom.hfov_deg)
            if rng is None:
                log.info("plant seen but no ToF zone on its pot gave a range — skipping")
                return None
        x1, y1, x2, y2 = det["bbox"]
        f = head.fx(w)
        left = math.atan2(w / 2.0 - (x1 + x2) / 2.0, f)           # the pot's bearing in the view, + left
        # A perpendicular range is depth along the view axis; a radial one is along the ray.
        depth = rng if head.geom.tof_perpendicular else rng * math.cos(left)
        tilt = math.radians(frame.tilt_deg)

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

    def look_for_plants(self, sec_odom: Pose2D) -> List[Sighting]:
        cfg, head = self.cfg, self.head
        found: List[Sighting] = []
        pans_done: List[float] = []
        for pan in cfg.sector_pans:
            frame = head.look(pan, 0.0)
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
        x, y, th = cfg.start_pose
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
                "map": self.cfg.map_name, "detector_confidence": s.confidence}
        if s.z_m is not None:
            meta.update(height_m=s.z_m, top_m=s.top_m, on_stand=s.on_stand, level=s.level)
        if s.size_m:
            meta["size_m"] = list(s.size_m)             # width, height as first seen
        describe = getattr(self.vision, "describe_plant", None)
        about = describe(views[0][0]) if describe else None     # what plant it is, from the image+text model
        if about:
            meta["about"] = about
            screen.detail(f"{self._plant} {about.get('name', '')}")
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
            now = self.map_pose()
            path = self.vision.slam_plan((now.x, now.y), vp["pose"][:2])
            if path is None or self.follow(path, max_dist=float("inf")) != "arrived":
                self._say(f"viewpoint {i}/{len(vps)} unreachable — skipping")
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
        frame = head.look(0.0, tilt0)
        dets = self._plants_in(frame)
        if dets:
            h, w = frame.main.shape[:2]
            f = head.fx(w)

            def off(d) -> float:
                """How far this box is from the plant we came for: sideways, and (if known) in height."""
                side = abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2) / w
                if want is None:
                    return side
                base = frame.tilt_deg + math.degrees(math.atan2(h / 2.0 - d["bbox"][3], f))
                return side + abs(base - want) / 30.0

            best = min(dets, key=off)
            x1, y1, x2, y2 = best["bbox"]
            head.aim_at_pixel(frame, (x1 + x2) / 2, (y1 + y2) / 2)
            head.gimbal.wait_settled()
            frame = head.camera.capture(head.gimbal)
        pose = self.map_pose()
        meta = {"pose": [round(pose.x, 3), round(pose.y, 3), round(pose.theta, 4)],
                "angle_deg": angle_deg, "pan_deg": round(frame.pan_deg, 1),
                "tilt_deg": round(frame.tilt_deg, 1), "range_m": head.range_at(),
                "detected": bool(dets)}
        return jpeg(frame.main, long_edge=None, quality=self.cfg.photo_quality), meta

    # ================================================================ motion

    def map_pose(self) -> Pose2D:
        return self.m2o.compose(self.base.odom())

    def face(self, xy) -> str:
        p = self.map_pose()
        dth = wrap(math.atan2(xy[1] - p.y, xy[0] - p.x) - p.theta)
        return self.base.turn_ccw(dth) if abs(dth) > math.radians(3) else "done"

    def follow(self, path: Sequence[Sequence[float]], max_dist: float) -> str:
        """Drive a map path waypoint by waypoint: turn, probe the floor, step."""
        cfg = self.cfg
        travelled = 0.0
        for wp in list(path)[1:]:
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
                step = min(dist, cfg.probe_step_m, max_dist - travelled)
                if step <= 0.05:
                    return "leg_limit"
                probe = self.head.floor_probe(step + cfg.stop_short_m)
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
                    view = self.head.look(0.0, -10.0)
                    if self.vision.path_clear(jpeg(view.main), step) is False:
                        self._remember_obstacles([(self.head.sensor_xy[0] + step, 0.0)])
                        return "blocked (vlm)"
                st = self.base.drive(step)
                travelled += step
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
        self._hazards.add(self.base.odom().as_tuple(), hz or probe.points)

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


def _crop(img, bbox, pad: float = 0.1) -> bytes:
    h, w = img.shape[:2]
    x1, y1, x2, y2 = bbox
    px, py = (x2 - x1) * pad, (y2 - y1) * pad
    x1, y1 = int(max(0, x1 - px)), int(max(0, y1 - py))
    x2, y2 = int(min(w, x2 + px)), int(min(h, y2 + py))
    return jpeg(img[y1:y2, x1:x2], long_edge=448)
