"""Client for the vision service on the Mac (Vision-And-Planning).

The one place on the Pi that knows the endpoint paths. Explore mode is a
sequential mission — it waits for each answer before acting — so this is a
plain blocking client: one request at a time by construction, hard timeouts,
no retries (PLAN standing rule #8). A failure raises `VisionError`; the mission
decides whether to pause, skip or stop.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Optional, Sequence

import requests

from robot_core import settings

log = logging.getLogger(__name__)


class VisionError(RuntimeError):
    pass


SAME_PLANT_Q = (
    "These photos were taken by a small home robot. The first is a plant it is "
    "looking at now; the others are photos of a plant it saw before, possibly "
    "from other sides or on another day. Is it the SAME individual plant (it may "
    "be rotated, have grown, or been moved), or a different plant? Answer JSON: "
    '{"same": true|false, "confidence": 0..1, "reason": "<short>"}')

PATH_CLEAR_Q = (
    "This is the forward view of a small wheeled robot (about 30 cm wide, "
    "wheels ~3 cm tall). Can it safely drive straight ahead about {d:.1f} m? "
    "Look for low obstacles, cables, rugs edges it could snag on, steps down, "
    "pet bowls, and anything fragile. Answer JSON: "
    '{{"clear": true|false, "reason": "<short>"}}')


DESCRIBE_PLANT_Q = (
    "This photo was taken by a small home robot of a plant it found. What plant "
    "is it? Answer JSON: "
    '{"name": "<common name, or unknown>", "species": "<botanical name, or unknown>", '
    '"condition": "<healthy | dry | wilting | yellowing | other, in a few words>", '
    '"pot": "<the pot or stand in a few words>", "confidence": 0..1}')


class VisionClient:
    def __init__(self, base_url: str = settings.VISION_SERVICE_BASE_URL,
                 timeout_s: float = 10.0, vlm_timeout_s: float = 60.0) -> None:
        self.base = base_url.rstrip("/")
        self.timeout = timeout_s
        self.vlm_timeout = vlm_timeout_s
        self._s = requests.Session()

    # ------------------------------------------------------------ plumbing

    def _call(self, method: str, path: str, ok_404: bool = False, ok_422: bool = False,
              timeout: Optional[float] = None, **kw) -> Any:
        try:
            r = self._s.request(method, self.base + path, timeout=timeout or self.timeout, **kw)
        except requests.RequestException as exc:
            raise VisionError(f"{method} {path}: {exc}") from exc
        if (ok_404 and r.status_code == 404) or (ok_422 and r.status_code == 422):
            return None
        if not r.ok:
            raise VisionError(f"{method} {path}: HTTP {r.status_code} {r.text[:200]}")
        return r.json()

    @staticmethod
    def _files(jpegs: Sequence[bytes], field: str = "images") -> list:
        return [(field, (f"{i}.jpg", b, "image/jpeg")) for i, b in enumerate(jpegs)]

    def health(self) -> dict:
        return self._call("GET", "/healthz")

    # ----------------------------------------------------------- detection

    def detect(self, jpeg: bytes, seq: int, min_confidence: float = 0.4) -> dict:
        return self._call("POST", "/v1/detect", data={"seq": seq, "min_confidence": min_confidence},
                          files={"image": ("f.jpg", jpeg, "image/jpeg")})

    # --------------------------------------------------------------- faces

    def face_embed(self, jpeg: bytes, seq: int = 0) -> Optional[dict]:
        """None when there is no face in the frame (the service answers 404)."""
        return self._call("POST", "/v1/faces/embed", ok_404=True, data={"seq": seq},
                          files={"image": ("f.jpg", jpeg, "image/jpeg")})

    def face_match(self, embedding: list) -> dict:
        return self._call("POST", "/v1/faces/match", json={"embedding": embedding})

    def face_enroll(self, jpegs: Sequence[bytes], label: str) -> dict:
        return self._call("POST", "/v1/faces/enroll", timeout=60.0, data={"label": label},
                          files=self._files(jpegs))

    # -------------------------------------------------------------- follow

    def follow_start(self, label: str = "") -> Optional[dict]:
        """None when `label` is not enrolled (empty label: nobody is)."""
        return self._call("POST", "/v1/follow/start", ok_404=True, json={"label": label})

    def follow_step(self, jpeg: bytes, seq: int, pan_deg: float = 0.0, tilt_deg: float = 0.0,
                    hfov_deg: float = 66.0, camera_height_m: float = 0.25,
                    range_m: Optional[float] = None, tof_grid=None, tof_fov_deg: float = 60.0,
                    hint_deg: Optional[float] = None, pose=None,
                    tof_pan_offset_deg: Optional[float] = None,
                    tof_tilt_offset_deg: Optional[float] = None) -> dict:
        """Where the person is and how to move: turn `turn_deg` (+ right), then drive `drive_m`.
        tof_grid: the 8x8 ToF's rows (metres, None = no range; row 0 the top, column 0 the
        left, as the camera sees), for the Mac to range every person in view. hint_deg:
        where the person was last seen, from the robot's present heading (+ left)."""
        data = {"seq": seq, "pan_deg": pan_deg, "tilt_deg": tilt_deg, "hfov_deg": hfov_deg,
                "camera_height_m": camera_height_m}
        if range_m is not None:
            data["range_m"] = range_m
        if tof_grid is not None:
            data["tof_grid"] = json.dumps([[None if v is None else round(v, 3) for v in row] for row in tof_grid])
            data["tof_fov_deg"] = tof_fov_deg
        if tof_pan_offset_deg is not None:
            data["tof_pan_offset_deg"] = float(tof_pan_offset_deg)
        if tof_tilt_offset_deg is not None:
            data["tof_tilt_offset_deg"] = float(tof_tilt_offset_deg)
        if hint_deg is not None:
            data["hint_deg"] = hint_deg
        if pose is not None:                              # lets the Mac put what the camera sees on its map
            data["pose"] = json.dumps([round(float(v), 4) for v in pose])
        return self._call("POST", "/v1/follow/step", timeout=20.0, data=data,
                          files={"image": ("f.jpg", jpeg, "image/jpeg")})

    def follow_stop(self) -> dict:
        return self._call("POST", "/v1/follow/stop")

    # ---------------------------------------------------------- navigation

    def nav_frame(self, jpeg: bytes, t: float, pose, pan_deg: float, tilt_deg: float, tof=None) -> dict:
        """One frame of the camera stream, for the Mac's visual SLAM: its capture time (the Pi's
        monotonic clock), the odometry pose then, where the head pointed, and the ToF grid nearest
        it ({"rows", "age_s"}: grid time minus capture time), whose ranges give SLAM its scale."""
        data = {"t": round(t, 4), "pan_deg": round(pan_deg, 2), "tilt_deg": round(tilt_deg, 2)}
        if tof is not None:
            data["tof"] = json.dumps(tof, separators=(",", ":"))
        if pose is not None:
            data["pose"] = json.dumps([round(float(v), 4) for v in pose])
        return self._call("POST", "/v1/nav/frame", timeout=2.0, data=data,
                          files={"image": ("f.jpg", jpeg, "image/jpeg")})

    def nav_plan(self, pose, goal, radius_m: float, local: Optional[dict] = None) -> Optional[list]:
        """Waypoints [(x, y), ...] from `pose` to `goal` in the odometry frame (the Mac plans with
        Theta* on its global map, corrected by visual SLAM), or None when it finds no way. `local`:
        the Pi's rolling costmap (RollingCostmap.snapshot), folded into the global map first."""
        body = {"pose": [float(v) for v in pose], "goal": [float(v) for v in goal],
                "radius_m": radius_m, "local": local}
        out = self._call("POST", "/v1/nav/plan", timeout=3.0, ok_404=True, json=body)
        return None if not out or not out.get("path") else [tuple(p) for p in out["path"]]

    # -------------------------------------------------------------- plants

    def plant_embed(self, jpeg: bytes, seq: int = 0) -> list:
        return self._call("POST", "/v1/plants/embed", data={"seq": seq},
                          files={"image": ("p.jpg", jpeg, "image/jpeg")})["embedding"]

    def plant_match(self, embedding: list, position: Optional[Sequence[float]] = None,
                    top_k: int = 3) -> dict:
        body = {"embedding": embedding, "top_k": top_k}
        if position is not None:
            body["position"] = list(position[:3])       # x, y on the map and, when known, height above the floor
        return self._call("POST", "/v1/plants/match", json=body)

    def plant_enroll(self, jpegs: Sequence[bytes], label: str = "", meta: Optional[dict] = None) -> dict:
        return self._call("POST", "/v1/plants/enroll", timeout=60.0,
                          data={"label": label, "meta": json.dumps(meta or {})},
                          files=self._files(jpegs))

    def plants(self) -> list:
        return self._call("GET", "/v1/plants")["plants"]

    def plant_photo(self, label: str) -> Optional[bytes]:
        """The first stored photo of a plant, for side-by-side questions."""
        rec = self._call("GET", f"/v1/plants/{label}", ok_404=True)
        try:
            name = rec["visits"][0]["views"][0]["file"]
        except (TypeError, KeyError, IndexError):
            return None
        try:
            r = self._s.get(f"{self.base}/v1/plants/{label}/{name}", timeout=self.timeout)
        except requests.RequestException:
            return None
        return r.content if r.ok else None

    # ------------------------------------------------ image + text reasoning

    def ask(self, question: str, jpegs: Sequence[bytes] = (), as_json: bool = False) -> Optional[dict]:
        """Free-form visual question. None when no model is reachable — the
        caller must always have a non-VLM fallback."""
        try:
            return self._call("POST", "/v1/ask", timeout=self.vlm_timeout,
                              data={"question": question, "json": str(as_json).lower()},
                              files=self._files(jpegs))
        except VisionError as exc:
            log.warning("VLM unavailable: %s", exc)
            return None

    def same_plant(self, now_jpeg: bytes, before_jpegs: Sequence[bytes]) -> Optional[bool]:
        res = self.ask(SAME_PLANT_Q, [now_jpeg, *before_jpegs], as_json=True)
        parsed = (res or {}).get("parsed") or {}
        if "same" not in parsed:
            return None
        log.info("VLM same-plant: %s (%s)", parsed.get("same"), parsed.get("reason"))
        return bool(parsed["same"]) if float(parsed.get("confidence", 1.0)) >= 0.6 else None

    def describe_plant(self, jpeg: bytes) -> Optional[dict]:
        """What the image+text model says the plant is: {name, species, condition,
        pot, confidence}. None when no model answered; the plant is saved without it."""
        res = self.ask(DESCRIBE_PLANT_Q, [jpeg], as_json=True)
        parsed = (res or {}).get("parsed")
        return parsed if isinstance(parsed, dict) and parsed.get("name") else None

    def path_clear(self, forward_jpeg: bytes, distance_m: float) -> Optional[bool]:
        res = self.ask(PATH_CLEAR_Q.format(d=distance_m), [forward_jpeg], as_json=True)
        parsed = (res or {}).get("parsed") or {}
        if "clear" not in parsed:
            return None
        if not parsed["clear"]:
            log.info("VLM says path blocked: %s", parsed.get("reason"))
        return bool(parsed["clear"])

    # ---------------------------------------------------------------- SLAM

    def slam_reset(self, name: str = "default", initial_pose=(0.0, 0.0, 0.0), load: bool = False,
                   explore_radius_m: Optional[float] = None, no_behind: bool = False) -> dict:
        return self._call("POST", "/v1/slam/reset", ok_404=load,
                          json={"name": name, "initial_pose": list(initial_pose), "load": load,
                                "explore_radius_m": explore_radius_m, "no_behind": no_behind})

    def slam_keyframe(self, odom, rays, sensor_xy=(0.0, 0.0), obstacles=(), plan: bool = True,
                      low=(), drops=(), clear=(), cloud=()) -> dict:
        """Everything in base_link at `odom`. obstacles, low, drops: [[x, y]], floor-probe
        obstacles, low obstacles the ToF saw, places where the floor falls away. clear:
        [[bearing, range]] rays along which the floor was seen that far and nothing was in
        the way (free space, no wall at the end). cloud: [[x, y, height]], every ToF reading."""
        return self._call("POST", "/v1/slam/keyframe", timeout=30.0, json={
            "odom": list(odom), "rays": [[b, r] for b, r in rays],
            "sensor_xy": list(sensor_xy), "obstacles": [list(p) for p in obstacles],
            "low": [list(p) for p in low], "drops": [list(p) for p in drops],
            "clear": [[b, r] for b, r in clear], "cloud": [list(p) for p in cloud], "plan": plan})

    def slam_plan(self, start, goal) -> Optional[list]:
        res = self._call("POST", "/v1/slam/plan", ok_422=True, timeout=30.0,
                         json={"start": list(start[:2]), "goal": list(goal[:2])})
        return None if res is None else res["path"]

    def slam_viewpoints(self, start, target, radius: float = 0.7, n: int = 4) -> list:
        return self._call("POST", "/v1/slam/viewpoints", timeout=30.0, json={
            "start": list(start[:2]), "target": list(target[:2]), "radius": radius, "n": n})["viewpoints"]

    def slam_save(self) -> dict:
        return self._call("POST", "/v1/slam/save")
