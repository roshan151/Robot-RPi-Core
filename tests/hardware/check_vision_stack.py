"""Camera + vision service + tracker on the Pi, no ROS. Phase V0/V1 bench check.

    python tests/hardware/check_vision_stack.py                 # 20 s detect + track run
    python tests/hardware/check_vision_stack.py --seconds 60    # longer; run once with voice live
    python tests/hardware/check_vision_stack.py --servos        # sweep the pan/tilt head
    python tests/hardware/check_vision_stack.py --save out.jpg  # also save one annotated frame

What it prints, and what to look for:
  * capture rate       should be close to VISION_TRACK_HZ (15). Much lower: CPU-bound
  * detect latency     p50 / p95 / p99 of the Pi's wall-clock round trip, next to the
                       server's own inference time. The difference is your wifi.
                       p99 above ~500 ms and every follow/approach constant needs
                       revisiting (VISION-DESIGN Part 9)
  * gateway counters   dropped_busy is normal (frames arriving while one is in flight);
                       unavailable/stale/errors are not
  * tracks             what the local tracker holds at the end, with bearings
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_core import camera as cam_mod                                  # noqa: E402
from robot_core import settings                                           # noqa: E402
from robot_core.perception.client import VisionClient                     # noqa: E402
from robot_core.perception.gateway import DetectGateway                   # noqa: E402
from robot_core.perception.geometry import CameraModel, scale_bbox        # noqa: E402
from robot_core.perception.scene import describe, target_views           # noqa: E402
from robot_core.perception.tracker import TrackManager                    # noqa: E402


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * p))] if xs else float("nan")


def run_detect(seconds: float, save: str | None) -> int:
    client = VisionClient(settings.VISION_SERVICE_BASE_URL)
    try:
        h = client.health(timeout=3.0)
    except Exception as exc:                                   # noqa: BLE001
        print(f"FAIL  vision service at {client.base_url}: {exc}")
        print("      Is the mini awake and the service running (make status on the mini)?")
        return 1
    print(f"ok    service {client.base_url}  versions={h.get('versions')}")

    camera = cam_mod.open_camera(settings.CAMERA_KIND)
    print(f"ok    camera {type(camera).__name__}")
    lores = CameraModel(*cam_mod.LORES_SIZE, settings.CAMERA_HFOV_DEG)
    tm = TrackManager(*cam_mod.LORES_SIZE)
    latencies, inference, last = [], [], {}

    def on_result(res, t_capture):
        w, h = res.image_size
        tm.anchor([(d.cls, d.confidence, scale_bbox(d.bbox, (w, h), cam_mod.LORES_SIZE))
                   for d in res.detections], res.seq, t_capture)
        latencies.append(gw.counters.last_latency_ms)
        inference.append(res.inference_ms)
        last["res"] = res

    gw = DetectGateway(
        lambda jpeg, seq: client.detect(jpeg, seq, min_confidence=settings.VISION_MIN_CONFIDENCE,
                                        timeout=settings.VISION_DETECT_TIMEOUT_S),
        on_result, max_result_age_s=settings.VISION_MAX_RESULT_AGE_S)

    t_end = time.monotonic() + seconds
    next_detect, frames, frame = 0.0, 0, None
    t0 = time.monotonic()
    try:
        while time.monotonic() < t_end:
            frame = camera.capture()
            frames += 1
            tm.step(frame.gray, frame.seq, frame.t_capture)
            now = time.monotonic()
            if now >= next_detect:
                if gw.submit(cam_mod.jpeg(frame.main_bgr, long_edge=settings.VISION_DETECT_LONG_EDGE_PX),
                             frame.seq, frame.t_capture):
                    next_detect = now + 1.0 / settings.VISION_DETECT_HZ
    finally:
        camera.close()
    elapsed = time.monotonic() - t0

    print(f"\ncapture   {frames / elapsed:.1f} fps (target {settings.VISION_TRACK_HZ:g})")
    if latencies:
        print(f"detect    round trip p50 {pct(latencies, .5):.0f} ms  p95 {pct(latencies, .95):.0f} ms  "
              f"p99 {pct(latencies, .99):.0f} ms   (server inference p50 {statistics.median(inference):.0f} ms)")
    print(f"gateway   {gw.counters.as_dict()}")
    views = target_views(tm.tracks.values(), lores, 0.0, 0.0, lambda _id: None)
    print(f"tracks    {describe(views)}")

    if save and frame is not None and "res" in last:
        import cv2
        img = cv2.resize(frame.main_bgr, (640, 360))
        for d in last["res"].detections:
            x1, y1, x2, y2 = (int(v) for v in scale_bbox(d.bbox, last["res"].image_size, (640, 360)))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img, f"{d.cls} {d.confidence:.2f}", (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imwrite(save, img)
        print(f"saved     {save}")
    bad = gw.counters.unavailable + gw.counters.errors
    return 1 if bad or not latencies else 0


def run_servos() -> int:
    from robot_core.servo import PanTilt, ServoConfig
    s = settings
    head = PanTilt(
        pan=ServoConfig(1, s.GIMBAL_PAN_MIN, s.GIMBAL_PAN_MAX, s.GIMBAL_PAN_TRIM, s.GIMBAL_PAN_INVERT),
        tilt=ServoConfig(0, s.GIMBAL_TILT_MIN, s.GIMBAL_TILT_MAX, s.GIMBAL_TILT_TRIM, s.GIMBAL_TILT_INVERT),
        chip=s.GIMBAL_PWM_CHIP)
    steps = [("centre", 0, 0), ("pan right 45", 45, 0), ("pan left 45", -45, 0), ("centre", 0, 0),
             ("tilt up 20", 0, 20), ("tilt down 20", 0, -20), ("centre", 0, 0)]
    print("Watch the head. Pan RIGHT must look to the robot's right; tilt UP must look up.")
    print("If one is backwards, set GIMBAL_PAN_INVERT=1 / GIMBAL_TILT_INVERT=1.")
    print("If 'centre' isn't straight ahead / level, adjust GIMBAL_PAN_TRIM / GIMBAL_TILT_TRIM (degrees).\n")
    try:
        for name, p, t in steps:
            print(f"  {name:14s} pan={p:+4d} tilt={t:+4d}")
            head.set(p, t)
            time.sleep(1.2)
    finally:
        head.relax()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--servos", action="store_true")
    ap.add_argument("--save", default=None)
    a = ap.parse_args()
    return run_servos() if a.servos else run_detect(a.seconds, a.save)


if __name__ == "__main__":
    sys.exit(main())
