"""Follow an enrolled person, as a job the voice agent runs with its session closed.

A stop-and-go loop with all the thinking on the Mac (Vision-Microservice
`follow.py`, see docs/FOLLOW-DESIGN.md):

    capture -> POST /v1/follow/step -> aim head -> turn turn_deg -> drive drive_m -> repeat

While searching the robot stays put and only the head sweeps (`head` in the
answer: pan, tilt above FOLLOW_TILT_DEG).

The robot stands still while the Mac decides; the person is expected to walk
slowly. The head is held straight ahead at FOLLOW_TILT_DEG, high enough to see
a standing person's face from ~1.3 m and low enough for the TF-Luna to hit
their body. Plain functions over injected parts — no ROS, no audio.

Ends when the Mac says the person is gone (lost, then a full head sweep),
after `seconds`, or when the task is cancelled (`task_cancel`: "stop" to the
cancel-only voice session, or the task/stop service). Returns True if it locked
onto the person at all.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from robot_core import settings, status, task_cancel
from robot_core.sensors.camera import jpeg
from robot_core.vision_client import VisionClient, VisionError

log = logging.getLogger(__name__)

MOVE_TIMEOUT_S = 30.0


def _move(motion, fn, value: float) -> None:
    """Send one turn/drive and wait for it to finish: the next frame must be
    taken standing still."""
    if task_cancel.requested():
        return
    res = fn(value)
    if not res.get("ok"):
        raise RuntimeError(f"move refused: {res.get('error')}")
    deadline = time.monotonic() + MOVE_TIMEOUT_S
    while motion.busy():
        if task_cancel.requested():
            return                                   # the canceller braked; follow() ends
        if time.monotonic() > deadline:
            raise RuntimeError("move timed out")
        time.sleep(0.1)


def follow(camera, vision: VisionClient, motion, name: str = "", head=None,
           seconds: float = settings.FOLLOW_SECONDS,
           clock: Callable[[], float] = time.monotonic) -> bool:
    """Follow `name` (empty: whichever enrolled person is seen first)."""
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
    t0, seq = clock(), 0
    try:
        while clock() - t0 < seconds and not task_cancel.requested():
            frame = camera.capture(head.gimbal) if head else camera.capture()
            seq += 1
            rng: Optional[float] = lidar.read(n=5, timeout=0.5) if lidar else None
            res = vision.follow_step(
                jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE), seq,
                pan_deg=frame.pan_deg, tilt_deg=frame.tilt_deg,
                hfov_deg=head.geom.hfov_deg if head else 66.0,
                camera_height_m=head.geom.height_m if head else 0.25, range_m=rng)
            log.info("follow %d: %s", seq, res)
            status.detail(f"{res['state']} {res.get('distance_m') or ''}")
            if res["state"] == "gone":
                break
            look = res.get("head")
            if head and look:
                head.gimbal.move_to(look[0], settings.FOLLOW_TILT_DEG + look[1], wait=True)
                head.gimbal.wait_settled()
            if res["turn_deg"]:
                _move(motion, motion.turn, res["turn_deg"])
            if res["drive_m"]:
                _move(motion, motion.drive, res["drive_m"])
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
