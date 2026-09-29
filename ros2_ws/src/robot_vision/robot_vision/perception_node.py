"""perception_node: camera -> local tracker -> REST -> small messages.

VISION-DESIGN.md Part 4.2. Runs in its OWN process (`perception`): it makes
network calls, and network calls hang for seconds. Nothing latency-critical may
share a process with it.

Threads inside this process — keep them straight:

    capture thread   picamera2 at TRACK_HZ: stamp, LK tracker step, publish /tracks,
                     and every 1/DETECT_HZ hand one JPEG to the gateway
    detect thread    one at a time, owned by DetectGateway (never more than one in flight)
    face thread      one embed+match at a time, for person tracks without an identity
    rclpy executor   services (/vision/look, /vision/clear_path, /vision/enroll),
                     the gimbal-state subscription and the status timer

Publishes (no pixels, ever):
    /tracks               robot_interfaces/Tracks            TRACK_HZ
    /detections           robot_interfaces/Detections        per applied detect result
    /perception_status    robot_interfaces/PerceptionStatus  1 Hz
    /vision/events        std_msgs/String                    identity decisions
"""

from __future__ import annotations

import json
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_core import camera as camera_mod
from robot_core import settings
from robot_core.perception import freespace
from robot_core.perception.client import NoFace, ServiceError, VisionClient
from robot_core.perception.gateway import DetectGateway
from robot_core.perception.geometry import CameraModel, head_region, scale_bbox, to_base_bearing
from robot_core.perception.identity import UNKNOWN, IdentityConfig, IdentityResolver
from robot_core.perception.scene import describe, target_views
from robot_core.perception.stamps import FrameStamp, StampRing
from robot_core.perception.tracker import TrackerConfig, TrackManager
from robot_interfaces.msg import (Detection, Detections, GimbalState, PerceptionStatus,
                                  TrackedObject, Tracks)
from robot_interfaces.srv import ClearPath, Enroll, Look

LORES = camera_mod.LORES_SIZE
MAIN = camera_mod.MAIN_SIZE
NAN = float("nan")


class PerceptionNode(Node):
    def __init__(self, camera=None, client: Optional[VisionClient] = None) -> None:
        super().__init__("perception")
        p = self._load_params()
        self.p = p

        self._client = client or VisionClient(p["base_url"])
        self._camera = camera
        self._detect_long_edge = p["detect_long_edge_px"]
        self._cam_lores = CameraModel(LORES[0], LORES[1], p["hfov_deg"])
        self._cam_main = self._cam_lores.scaled(*MAIN)

        self._tm = TrackManager(*LORES, TrackerConfig(min_inliers=p["min_inliers"],
                                                      reacquire_s=p["reacquire_s"]))
        self._tm_lock = threading.Lock()
        self._ident = IdentityResolver(IdentityConfig(min_quality=p["face_min_quality"]))
        self._stamps = StampRing(64)
        self._gateway = DetectGateway(
            lambda jpeg, seq: self._client.detect(jpeg, seq, min_confidence=p["min_confidence"],
                                                  timeout=p["detect_timeout_s"]),
            self._on_detect,
            max_result_age_s=p["max_result_age_s"],
            degraded_after=p["degraded_after"],
        )
        self._faces = ThreadPoolExecutor(max_workers=1, thread_name_prefix="faces")
        self._face_busy = False

        self._gimbal = (0.0, 0.0, True)          # pan, tilt, settled — commanded
        self._latest = None                      # (Frame, FrameStamp)
        self._latest_lock = threading.Lock()
        self._camera_error = ""
        self._last_applied_t = 0.0
        self._hz = 0.0
        self._running = False

        self._pub_tracks = self.create_publisher(Tracks, "tracks", 5)
        self._pub_dets = self.create_publisher(Detections, "detections", 5)
        self._pub_status = self.create_publisher(PerceptionStatus, "perception_status", 5)
        self._pub_events = self.create_publisher(String, "vision/events", 10)

        self.create_subscription(GimbalState, "gimbal/state", self._on_gimbal, 10)

        slow = ReentrantCallbackGroup()          # service calls may take seconds
        self.create_service(Look, "vision/look", self._srv_look, callback_group=slow)
        self.create_service(ClearPath, "vision/clear_path", self._srv_clear, callback_group=slow)
        self.create_service(Enroll, "vision/enroll", self._srv_enroll, callback_group=slow)
        self.create_timer(1.0, self._publish_status, callback_group=MutuallyExclusiveCallbackGroup())

    # ---------------------------------------------------------------- setup

    def _load_params(self) -> dict:
        s = settings
        defaults = {
            "base_url": s.VISION_SERVICE_BASE_URL,
            "detect_hz": s.VISION_DETECT_HZ,
            "track_hz": s.VISION_TRACK_HZ,
            "detect_timeout_s": s.VISION_DETECT_TIMEOUT_S,
            "max_result_age_s": s.VISION_MAX_RESULT_AGE_S,
            "min_confidence": s.VISION_MIN_CONFIDENCE,
            "detect_long_edge_px": s.VISION_DETECT_LONG_EDGE_PX,
            "degraded_after": s.VISION_DEGRADED_AFTER,
            "min_inliers": s.VISION_MIN_INLIERS,
            "reacquire_s": s.VISION_REACQUIRE_S,
            "hfov_deg": s.CAMERA_HFOV_DEG,
            "camera_kind": s.CAMERA_KIND,
            "hflip": s.CAMERA_HFLIP,
            "vflip": s.CAMERA_VFLIP,
            "face_min_quality": s.FACE_MIN_QUALITY,
            "enroll_images": s.FACE_ENROLL_IMAGES,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        return {k: self.get_parameter(k).value for k in defaults}

    def start(self) -> None:
        if self._camera is None:
            kind = self.p["camera_kind"]
            try:
                if kind == "opencv":
                    self._camera = camera_mod.OpenCVCamera().start()
                else:
                    self._camera = camera_mod.PiCamera(fps=self.p["track_hz"], hflip=self.p["hflip"],
                                                       vflip=self.p["vflip"]).start()
            except Exception as exc:                      # noqa: BLE001
                if kind == "auto":
                    self.get_logger().warning(f"picamera2 unavailable ({exc}); trying OpenCV")
                    self._camera = camera_mod.OpenCVCamera().start()
                else:
                    raise
        try:
            health = self._client.health(timeout=3.0)
            self.get_logger().info(f"vision service at {self._client.base_url}: {health.get('versions')}")
        except ServiceError as exc:
            # Not fatal: the mini may wake up later. The gateway counts failures
            # and reports DEGRADED, which is the honest state until it does.
            self.get_logger().warning(f"vision service not reachable yet: {exc}")
        self._running = True
        threading.Thread(target=self._capture_loop, name="capture", daemon=True).start()

    def shutdown(self) -> None:
        self._running = False
        time.sleep(0.2)
        self._faces.shutdown(wait=False, cancel_futures=True)
        if self._camera is not None:
            try:
                self._camera.close()
            except Exception:                             # noqa: BLE001
                pass
        self._client.close()

    # ------------------------------------------------------- capture thread

    def _capture_loop(self) -> None:
        period = 1.0 / max(1.0, self.p["track_hz"])
        detect_every = 1.0 / max(0.1, self.p["detect_hz"])
        next_detect = 0.0
        ema = None
        last = time.monotonic()
        while self._running and rclpy.ok():
            t0 = time.monotonic()
            try:
                frame = self._camera.capture()
                self._camera_error = ""
            except Exception as exc:                      # noqa: BLE001
                self._camera_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(f"camera capture failed: {self._camera_error}",
                                        throttle_duration_sec=5.0)
                time.sleep(1.0)
                continue

            pan, tilt, settled = self._gimbal
            stamp = FrameStamp(frame.seq, frame.t_capture, pan, tilt, settled)
            self._stamps.add(stamp)
            with self._tm_lock:
                self._tm.step(frame.gray, frame.seq, frame.t_capture)
            with self._latest_lock:
                self._latest = (frame, stamp)

            now = time.monotonic()
            if now >= next_detect:
                jpeg = camera_mod.jpeg(frame.main_bgr, long_edge=self._detect_long_edge)
                if self._gateway.submit(jpeg, frame.seq, frame.t_capture):
                    next_detect = now + detect_every
                else:
                    next_detect = now + period          # in flight: try the next frame

            self._schedule_faces(frame, now)
            self._publish_tracks(stamp, now)

            dt = now - last
            last = now
            if dt > 0:
                ema = (1.0 / dt) if ema is None else 0.9 * ema + 0.1 / dt
                self._hz = ema
            spare = period - (time.monotonic() - t0)
            if spare > 0:
                time.sleep(spare)

    # ------------------------------------------------------- detect thread

    def _on_detect(self, result, t_capture: float) -> None:
        stamp = self._stamps.get(result.seq)
        pan = stamp.pan_deg if stamp else self._gimbal[0]
        w, h = result.image_size
        cam = self._cam_lores.scaled(w, h)
        dets = [(d.cls, d.confidence, scale_bbox(d.bbox, (w, h), LORES)) for d in result.detections]
        with self._tm_lock:
            self._tm.anchor(dets, result.seq, t_capture)
        self._last_applied_t = time.monotonic()

        msg = Detections()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.seq = result.seq
        msg.model_version = result.model_version
        msg.inference_ms = float(result.inference_ms)
        msg.latency_ms = float(self._gateway.counters.last_latency_ms)
        for d in result.detections:
            m = Detection()
            m.cls = d.cls
            m.confidence = float(d.confidence)
            m.bbox = [float(d.bbox[0] / w), float(d.bbox[1] / h), float(d.bbox[2] / w), float(d.bbox[3] / h)]
            m.bearing_deg = float(to_base_bearing(cam.bbox_bearing_deg(d.bbox), pan))
            dist = cam.distance_m(d.bbox, d.cls)
            m.distance_m = float(dist) if dist is not None else NAN
            msg.detections.append(m)
        self._pub_dets.publish(msg)

    # --------------------------------------------------------- face thread

    def _schedule_faces(self, frame, now: float) -> None:
        if self._face_busy or self._gateway.degraded:
            return
        sx = MAIN[0] / LORES[0]
        with self._tm_lock:
            people = self._tm.live("person")
            self._ident.prune(set(self._tm.tracks))
            for tr in sorted(people, key=lambda t: -(t.bbox[3] - t.bbox[1])):
                box_main = scale_bbox(tr.bbox, LORES, MAIN)
                if self._ident.wants_embed(tr.id, (tr.bbox[3] - tr.bbox[1]) * sx, now):
                    self._ident.started(tr.id, now)
                    region = head_region(box_main, MAIN)
                    jpeg = camera_mod.jpeg(camera_mod.crop(frame.main_bgr, region), quality=85)
                    self._face_busy = True
                    self._faces.submit(self._identify, tr.id, jpeg, frame.seq)
                    return

    def _identify(self, track_id: int, jpeg: bytes, seq: int) -> None:
        try:
            emb = self._client.embed(jpeg, seq)
            match = self._client.match(emb.embedding) if emb.quality >= self.p["face_min_quality"] else None
            best = match.best if match else None
            with self._tm_lock:
                decided = self._ident.result(track_id, emb.quality, best.label if best else None,
                                             best.similarity if best else 0.0)
            if decided and decided != UNKNOWN:
                self.get_logger().info(f"track {track_id} is {decided}")
                self._event(f"recognised {decided}")
        except NoFace:
            with self._tm_lock:
                self._ident.no_face(track_id)
        except ServiceError as exc:
            with self._tm_lock:
                self._ident.failed(track_id)
            self.get_logger().warning(f"face identify failed: {exc}", throttle_duration_sec=10.0)
        except Exception as exc:                          # noqa: BLE001
            with self._tm_lock:
                self._ident.failed(track_id)
            self.get_logger().error(f"face identify crashed: {type(exc).__name__}: {exc}")
        finally:
            self._face_busy = False

    # ------------------------------------------------------------- publish

    def _views(self, pan: float, tilt: float):
        with self._tm_lock:
            tracks = list(self._tm.tracks.values())
            return tracks, target_views(tracks, self._cam_lores, pan, tilt, self._ident.identity)

    def _publish_tracks(self, stamp: FrameStamp, now: float) -> None:
        tracks, views = self._views(stamp.pan_deg, stamp.tilt_deg)
        msg = Tracks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.seq = stamp.seq
        msg.pan_deg, msg.tilt_deg = float(stamp.pan_deg), float(stamp.tilt_deg)
        for tr, v in zip(tracks, views):
            o = TrackedObject()
            o.id, o.cls = tr.id, tr.cls
            o.identity = v.identity or ""
            o.identity_score = float(self._ident.score(tr.id))
            o.confidence = float(tr.confidence)
            o.bbox = [float(tr.bbox[0] / LORES[0]), float(tr.bbox[1] / LORES[1]),
                      float(tr.bbox[2] / LORES[0]), float(tr.bbox[3] / LORES[1])]
            o.bearing_deg = float(v.bearing_deg)
            o.aim_elevation_deg = float(v.aim_elevation_deg)
            o.distance_m = float(v.distance_m) if v.distance_m is not None else NAN
            o.lost = bool(v.lost)
            o.anchor_age_s = float(max(0.0, now - tr.last_anchor_t))
            o.bottom_v = float(v.bottom_v_frac)
            o.area_frac = float(v.area_frac)
            msg.tracks.append(o)
        self._pub_tracks.publish(msg)

    def _publish_status(self) -> None:
        c = self._gateway.counters
        m = PerceptionStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        if self._camera_error:
            m.state, m.detail = "NO_CAMERA", self._camera_error
        elif self._gateway.degraded:
            m.state = "DEGRADED"
            m.detail = f"{c.consecutive_failures} detect failures in a row ({self._client.base_url})"
        else:
            m.state = "OK"
        m.model_version = self._gateway.model_version
        m.track_hz = float(self._hz)
        m.detect_sent, m.detect_ok = c.sent, c.ok
        m.detect_dropped_busy, m.detect_server_busy = c.dropped_busy, c.server_busy
        m.detect_unavailable, m.detect_errors = c.unavailable, c.errors
        m.detect_out_of_order, m.detect_stale = c.out_of_order, c.stale
        m.last_latency_ms, m.last_inference_ms = float(c.last_latency_ms), float(c.last_inference_ms)
        self._pub_status.publish(m)

    def _event(self, text: str) -> None:
        self._pub_events.publish(String(data=text))

    def _on_gimbal(self, msg: GimbalState) -> None:
        self._gimbal = (float(msg.pan_deg), float(msg.tilt_deg), bool(msg.settled))

    # ------------------------------------------------------------ services

    def _srv_look(self, req, res):
        if self._camera_error:
            res.ok, res.summary = False, "My camera isn't working."
            return res
        if self._gateway.degraded:
            res.ok, res.summary = False, "My vision service isn't answering, so I can't see right now."
            return res
        pan, tilt, _ = self._gimbal
        _, views = self._views(pan, tilt)
        views = [v for v in views if not v.lost and (not req.filter or v.cls == req.filter)]
        res.ok = True
        res.summary = describe(views)
        if req.filter == "person" and not views:
            res.summary = "I don't see anyone."
        if time.monotonic() - self._last_applied_t > 3.0:
            res.summary += " (My view may be out of date.)"
        res.json = json.dumps([{
            "id": v.track_id, "class": v.cls, "identity": v.identity,
            "bearing_deg": v.bearing_deg, "distance_m": v.distance_m,
        } for v in views])
        return res

    def _srv_clear(self, req, res):
        with self._latest_lock:
            latest = self._latest
        if latest is None:
            res.ok, res.known, res.summary = False, False, "no camera frame yet"
            return res
        frame, stamp = latest
        jpeg = camera_mod.jpeg(frame.main_bgr, long_edge=self._detect_long_edge)
        h, w = frame.main_bgr.shape[:2]
        s = self._detect_long_edge / float(max(w, h))
        dw, dh = int(round(w * s)), int(round(h * s))
        cam = self._cam_lores.scaled(dw, dh)
        if req.mode == "corridor":
            layout = freespace.corridor(dw, dh, top_v=float(req.corridor_top_v) * dh)
            if layout is None:
                res.ok, res.known, res.summary = True, False, "not enough floor visible to judge"
                return res
        else:
            layout = freespace.grid(dw, dh)
        try:
            d = self._client.depth(jpeg, frame.seq, layout.points, timeout=2.5)
        except ServiceError as exc:
            res.ok, res.known, res.summary = False, False, f"depth unavailable: {exc}"
            return res
        fs = freespace.analyze(d.depths, d.metric, layout, cam, tilt_deg=stamp.tilt_deg)
        res.ok = True
        res.known = fs.ahead_clear is not None
        res.ahead_clear = bool(fs.ahead_clear)
        res.has_best = fs.best_bearing_deg is not None
        res.best_bearing_deg = float(to_base_bearing(fs.best_bearing_deg, stamp.pan_deg)) if res.has_best else 0.0
        res.summary = fs.summary()
        return res

    def _srv_enroll(self, req, res):
        label = req.label.strip().lower()
        n = req.images if req.images > 0 else int(self.p["enroll_images"])
        if not label:
            res.ok, res.message = False, "a name is required"
            return res
        crops, last_seq = [], -1
        deadline = time.monotonic() + 4.0 + 0.6 * n
        while len(crops) < n and time.monotonic() < deadline:
            with self._latest_lock:
                latest = self._latest
            if latest is not None and latest[0].seq != last_seq:
                frame, _ = latest
                last_seq = frame.seq
                with self._tm_lock:
                    people = self._tm.live("person")
                if people:
                    tr = max(people, key=lambda t: (t.bbox[2] - t.bbox[0]) * (t.bbox[3] - t.bbox[1]))
                    region = head_region(scale_bbox(tr.bbox, LORES, MAIN), MAIN)
                    crops.append(camera_mod.jpeg(camera_mod.crop(frame.main_bgr, region), quality=90))
            time.sleep(0.4)       # spacing: different expressions/angles beat 6 identical frames
        if not crops:
            res.ok, res.message = False, "I don't see anyone to remember."
            return res
        try:
            out = self._client.enroll(label, crops)
        except ServiceError as exc:
            res.ok, res.message = False, f"enroll failed: {exc}"
            return res
        # Tracks that were UNKNOWN may be this person now; let them re-query.
        with self._tm_lock:
            for tid in list(self._tm.tracks):
                if self._ident.identity(tid) == UNKNOWN:
                    self._ident.forget(tid)
        res.ok = True
        res.message = (f"remembered {label}: {out.get('embeddings_added', 0)} face images "
                       f"({out.get('images_rejected', 0)} rejected)")
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        node.start()
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
