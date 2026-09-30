"""Client for the vision service on the Mac (Vision-Microservice).

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

    # -------------------------------------------------------------- plants

    def plant_embed(self, jpeg: bytes, seq: int = 0) -> list:
        return self._call("POST", "/v1/plants/embed", data={"seq": seq},
                          files={"image": ("p.jpg", jpeg, "image/jpeg")})["embedding"]

    def plant_match(self, embedding: list, position: Optional[Sequence[float]] = None,
                    top_k: int = 3) -> dict:
        body = {"embedding": embedding, "top_k": top_k}
        if position is not None:
            body["position"] = list(position[:2])
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

    def slam_keyframe(self, odom, rays, sensor_xy=(0.0, 0.0), obstacles=(), plan: bool = True) -> dict:
        return self._call("POST", "/v1/slam/keyframe", timeout=30.0, json={
            "odom": list(odom), "rays": [[b, r] for b, r in rays],
            "sensor_xy": list(sensor_xy), "obstacles": [list(p) for p in obstacles], "plan": plan})

    def slam_plan(self, start, goal) -> Optional[list]:
        res = self._call("POST", "/v1/slam/plan", ok_422=True, timeout=30.0,
                         json={"start": list(start[:2]), "goal": list(goal[:2])})
        return None if res is None else res["path"]

    def slam_viewpoints(self, start, target, radius: float = 0.7, n: int = 4) -> list:
        return self._call("POST", "/v1/slam/viewpoints", timeout=30.0, json={
            "start": list(start[:2]), "target": list(target[:2]), "radius": radius, "n": n})["viewpoints"]

    def slam_save(self) -> dict:
        return self._call("POST", "/v1/slam/save")
