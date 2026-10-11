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

from robot_core import status as screen
from robot_core.odometry import Pose2D, wrap
from robot_core.sensors.camera import jpeg
from robot_core.vision_client import VisionError

log = logging.getLogger(__name__)


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


@dataclass
class Stats:
    keyframes: int = 0
    plants_new: List[str] = field(default_factory=list)
    plants_revisited: List[str] = field(default_factory=list)
    obstacles: int = 0
    stuck: int = 0


class Explorer:
    def __init__(self, base: Base, head, vision, cfg: ExploreConfig = ExploreConfig(),
                 on_status: Callable[[str], None] = lambda s: None) -> None:
        self.base, self.head, self.vision, self.cfg = base, head, vision, cfg
        self.status = on_status
        self.m2o = Pose2D(*cfg.start_pose)
        self.stats = Stats()
        self._handled: List[Tuple[float, float]] = []
        self._plant = ""                              # plant_NN on the OLED
        self._obstacles_odom: List[Tuple[float, float]] = []
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
                res = self.vision.slam_keyframe(
                    kf_odom.as_tuple(), rays, self.head.sensor_xy, self._obstacles_in(kf_odom))
                self._obstacles_odom.clear()
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
                  f"revisited {self.stats.plants_revisited}")
        return self.stats

    # ================================================================ scan

    def scan_keyframe(self) -> Tuple[Pose2D, list, List[Sighting]]:
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

    def _dedupe(self, sightings: List[Sighting]) -> List[Sighting]:
        out: List[Sighting] = []
        for s in sorted(sightings, key=lambda s: -s.confidence):
            p = self.m2o.compose(s.sector_odom).compose(Pose2D(*s.local_xy, 0.0))
            s.map_xy = (p.x, p.y)
            if all(_dist(s.map_xy, o.map_xy) > self.cfg.same_place_m for o in out):
                out.append(s)
        return out

    # ============================================================== plants

    def handle_plant(self, s: Sighting) -> bool:
        """Identify, orbit and save one plant. True if the robot moved."""
        if any(_dist(s.map_xy, h) <= self.cfg.same_place_m for h in self._handled):
            return False                             # already dealt with this run
        if not self._in_scope(s.map_xy):
            return False                             # outside the radius / behind the start
        self._handled.append(s.map_xy)
        self._plant = f"plant_{len(self._handled):02d}"
        match = self.vision.plant_match(self.vision.plant_embed(s.crop), s.map_xy)
        label = self.identify(s, match)
        if label is None:
            return False
        self._say(f"plant at ({s.map_xy[0]:.2f}, {s.map_xy[1]:.2f}): "
                  f"{'new' if not label else label} — photographing it from all sides")
        screen.detail(f"{self._plant} {'new' if not label else label}, starting photos")
        views = self.orbit(s.map_xy)
        if not views:
            self._say("could not reach any viewpoint around the plant — skipping")
            return True
        meta = {"position": [round(v, 3) for v in s.map_xy],
                "views": [v[1] for v in views],
                "map": self.cfg.map_name, "detector_confidence": s.confidence}
        res = self.vision.plant_enroll([v[0] for v in views], label, meta)
        (self.stats.plants_revisited if label else self.stats.plants_new).append(res["label"])
        screen.detail(f"{res['label']} saved {res['photos_saved']} photos")
        self._say(f"saved {res['photos_saved']} photos as {res['label']} ({res['folder']})")
        return True

    def identify(self, s: Sighting, match: dict) -> Optional[str]:
        """'' = new plant, a label = known plant, None = known and nothing to do."""
        cfg = self.cfg
        best = match["matches"][0] if match["matches"] else None
        if best is None:
            return ""
        thr = match["threshold"]
        near = best["distance_m"] is not None and best["distance_m"] <= cfg.same_place_m
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

    def orbit(self, target: Tuple[float, float]) -> List[Tuple[bytes, dict]]:
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
            views.append(self.photograph(target, vp.get("angle_deg")))
        return views

    def photograph(self, target, angle_deg) -> Tuple[bytes, dict]:
        head = self.head
        frame = head.look(0.0, -5.0)
        dets = self._plants_in(frame)
        if dets:
            h, w = frame.main.shape[:2]
            best = min(dets, key=lambda d: abs((d["bbox"][0] + d["bbox"][2]) / 2 - w / 2))
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
                if probe.status in ("obstacle", "drop"):
                    self._remember_obstacles(probe.points)
                    room = (probe.distance_m or 0.0) - cfg.front_m - cfg.stop_short_m
                    if room > 0.1:
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

    def _remember_obstacles(self, pts_base) -> None:
        o = self.base.odom()
        for x, y in pts_base:
            q = o.compose(Pose2D(x, y, 0.0))
            self._obstacles_odom.append((q.x, q.y))
        self.stats.obstacles += len(pts_base)

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
