"""HTTP client for the vision service (the Mac mini). The only code that knows
the /v1 paths.

Transport rules from VISION-DESIGN.md Part 3: binary JPEG over multipart, never
base64; one keep-alive Session for the life of the process; every inference
request carries `seq` and the caller checks the echo.

Errors are typed so callers can apply policy without parsing strings:
  Busy         503 — the server refused rather than queue. Drop the frame.
  NoFace       404 on embed — normal for someone seen from behind.
  Unavailable  connection refused / DNS / timeout — the mini is asleep or off.
  ServiceError anything else, with the server's {"error","detail"} if it sent one.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import requests

log = logging.getLogger(__name__)


class ServiceError(RuntimeError):
    def __init__(self, message: str, status: Optional[int] = None) -> None:
        super().__init__(message)
        self.status = status


class Busy(ServiceError):
    pass


class NoFace(ServiceError):
    pass


class Unavailable(ServiceError):
    pass


@dataclass(frozen=True)
class Detection:
    cls: str
    confidence: float
    bbox: tuple[float, float, float, float]   # x1,y1,x2,y2 in the pixels of the image as sent


@dataclass(frozen=True)
class DetectResult:
    seq: int
    detections: list[Detection]
    image_size: tuple[int, int]               # (w, h) as the server received it
    model_version: str
    inference_ms: float


@dataclass(frozen=True)
class EmbedResult:
    seq: int
    embedding: list[float]
    face_bbox: tuple[float, float, float, float]
    quality: float
    model_version: str


@dataclass(frozen=True)
class Match:
    label: str
    similarity: float


@dataclass(frozen=True)
class MatchResult:
    matches: list[Match]
    threshold: float
    gallery_size: int

    @property
    def best(self) -> Optional[Match]:
        """Top match if it clears the server's threshold, else None ("unknown")."""
        if self.matches and self.matches[0].similarity >= self.threshold:
            return self.matches[0]
        return None


@dataclass(frozen=True)
class DepthResult:
    seq: int
    depths: list[float]
    confidence: list[float]
    metric: bool          # False: values ORDER correctly but are not metres
    model_version: str


class VisionClient:
    def __init__(self, base_url: str, session: Optional[requests.Session] = None) -> None:
        self.base_url = base_url.rstrip("/")
        self._s = session or requests.Session()

    def close(self) -> None:
        self._s.close()

    # ------------------------------------------------------------------ ops

    def health(self, timeout: float = 2.0) -> dict:
        return self._call("GET", "/healthz", timeout=timeout)

    # ------------------------------------------------------------ inference

    def detect(self, jpeg: bytes, seq: int, *, min_confidence: float = 0.4,
               timeout: float = 0.6) -> DetectResult:
        body = self._call(
            "POST", "/v1/detect", timeout=timeout,
            files={"image": ("frame.jpg", jpeg, "image/jpeg")},
            data={"seq": str(seq), "min_confidence": str(min_confidence)},
        )
        return DetectResult(
            seq=int(body["seq"]),
            detections=[
                Detection(d["class"], float(d["confidence"]),
                          tuple(float(v) for v in d["bbox"]))  # type: ignore[arg-type]
                for d in body["detections"]
            ],
            image_size=(int(body["image_size"][0]), int(body["image_size"][1])),
            model_version=body.get("model_version", ""),
            inference_ms=float(body.get("inference_ms", 0.0)),
        )

    def embed(self, jpeg: bytes, seq: int, timeout: float = 2.0) -> EmbedResult:
        body = self._call(
            "POST", "/v1/faces/embed", timeout=timeout,
            files={"image": ("crop.jpg", jpeg, "image/jpeg")},
            data={"seq": str(seq)},
            not_found=NoFace,
        )
        return EmbedResult(
            seq=int(body["seq"]), embedding=list(body["embedding"]),
            face_bbox=tuple(body["face_bbox"]),  # type: ignore[arg-type]
            quality=float(body["quality"]), model_version=body.get("model_version", ""),
        )

    def match(self, embedding: Sequence[float], top_k: int = 1, timeout: float = 2.0) -> MatchResult:
        body = self._call("POST", "/v1/faces/match", timeout=timeout,
                          json={"embedding": list(embedding), "top_k": top_k})
        return MatchResult(
            matches=[Match(m["label"], float(m["similarity"])) for m in body["matches"]],
            threshold=float(body["threshold"]), gallery_size=int(body["gallery_size"]),
        )

    def enroll(self, label: str, jpegs: Sequence[bytes], timeout: float = 20.0) -> dict:
        files = [("images", (f"{i}.jpg", j, "image/jpeg")) for i, j in enumerate(jpegs)]
        return self._call("POST", "/v1/faces/enroll", timeout=timeout,
                          files=files, data={"label": label})

    def forget(self, label: str, timeout: float = 5.0) -> dict:
        return self._call("DELETE", f"/v1/faces/{label}", timeout=timeout)

    def faces(self, timeout: float = 5.0) -> dict:
        return self._call("GET", "/v1/faces", timeout=timeout)

    def depth(self, jpeg: bytes, seq: int, points: Sequence[tuple[float, float]],
              timeout: float = 2.0) -> DepthResult:
        body = self._call(
            "POST", "/v1/depth", timeout=timeout,
            files={"image": ("frame.jpg", jpeg, "image/jpeg")},
            data={"seq": str(seq), "points": json.dumps([[float(x), float(y)] for x, y in points])},
        )
        return DepthResult(
            seq=int(body["seq"]), depths=[float(v) for v in body["depths_m"]],
            confidence=[float(v) for v in body["confidence"]], metric=bool(body["metric"]),
            model_version=body.get("model_version", ""),
        )

    # ------------------------------------------------------------ internals

    def _call(self, method: str, path: str, *, timeout: float,
              not_found: type[ServiceError] = ServiceError, **kw: Any) -> Any:
        try:
            r = self._s.request(method, self.base_url + path, timeout=timeout, **kw)
        except (requests.ConnectionError, requests.Timeout) as exc:
            raise Unavailable(f"{method} {path}: {type(exc).__name__}: {exc}") from exc
        if r.status_code == 200:
            return r.json()
        detail = _detail(r)
        if r.status_code == 503:
            raise Busy(f"{path}: {detail}", 503)
        if r.status_code == 404:
            raise not_found(f"{path}: {detail}", 404)
        if r.status_code == 504:
            raise Unavailable(f"{path}: {detail}", 504)
        raise ServiceError(f"{path}: HTTP {r.status_code}: {detail}", r.status_code)


def _detail(r: requests.Response) -> str:
    try:
        body = r.json()
        return str(body.get("detail") or body.get("error") or body)
    except ValueError:
        return r.text[:200]
