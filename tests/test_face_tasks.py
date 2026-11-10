"""Voice-triggered face enroll / match, against a fake camera and vision service."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_core import face_tasks, settings  # noqa: E402


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self):
        self.t += 0.5            # every look at the clock is half a second
        return self.t


class Camera:
    def capture(self, gimbal=None):
        return SimpleNamespace(main=np.zeros((48, 64, 3), np.uint8))


class Vision:
    def __init__(self, faces=1, quality=0.9, sim=0.9):
        self.faces, self.quality, self.sim = faces, quality, sim
        self.enrolled = None

    def face_embed(self, jpeg, seq=0):
        if not self.faces:
            return None
        return {"embedding": [1.0], "face_count": self.faces, "quality": self.quality,
                "face_bbox": [10, 10, 30, 30]}

    def face_match(self, emb):
        return {"matches": [{"label": "sam", "similarity": self.sim}], "threshold": 0.75}

    def face_enroll(self, jpegs, label):
        self.enrolled = (len(jpegs), label)
        return {}


def test_enroll_saves_good_frames_under_the_name():
    v = Vision()
    assert face_tasks.enroll(Camera(), v, "sam", seconds=30, clock=Clock())
    assert v.enrolled[1] == "sam" and v.enrolled[0] >= 5


def test_enroll_fails_with_a_second_face_poor_quality_or_nobody():
    for v in (Vision(faces=2), Vision(quality=0.3), Vision(faces=0)):
        assert not face_tasks.enroll(Camera(), v, "sam", seconds=30, clock=Clock())
        assert v.enrolled is None


def test_match_says_who_it_is_or_nobody():
    assert face_tasks.match(Camera(), Vision(sim=0.9), seconds=5, clock=Clock()) == "sam"
    assert face_tasks.match(Camera(), Vision(sim=0.2), seconds=5, clock=Clock()) is None
    assert face_tasks.match(Camera(), Vision(faces=0), seconds=5, clock=Clock()) is None


class Head:
    """Records where the head was sent; a face 'appears' only when it looks at pan >= 15, tilt >= 20."""
    def __init__(self):
        self.moves, self.aimed = [], 0
        self.gimbal = self
        self.pose = (0.0, 0.0)

    def move_to(self, pan=None, tilt=None, wait=False):
        self.pose = (pan, tilt)
        self.moves.append(self.pose)

    def wait_settled(self): return True
    def aim_at_pixel(self, frame, u, v): self.aimed += 1
    def pixel_angles(self, frame, u, v):                # the face is a little left of / above centre
        return (self.pose[0] + 2.0, self.pose[1] + 1.0)


class HeadCamera(Camera):
    def __init__(self, head): self.head = head
    def capture(self, gimbal=None):
        return SimpleNamespace(main=np.zeros((48, 64, 3), np.uint8), pan_deg=0.0, tilt_deg=0.0)


class SeesFaceAt(Vision):
    def __init__(self, head, **kw):
        super().__init__(**kw)
        self.head = head

    def face_embed(self, jpeg, seq=0):
        pan, tilt = self.head.pose
        return super().face_embed(jpeg, seq) if pan >= 15 and tilt >= 20 else None

    def face_match(self, emb):
        self.checks.append(self.head.pose)
        return super().face_match(emb)

    checks: list


def sees(head, **kw):
    v = SeesFaceAt(head, **kw)
    v.checks = []
    return v


def test_enroll_frames_the_face_but_never_sweeps():
    head = Head()
    assert face_tasks.enroll(HeadCamera(head), Vision(), "sam", seconds=30, clock=Clock(), head=head)
    assert head.moves == [(0.0, settings.FACE_ENROLL_TILT_DEG)], "enroll must not sweep"
    assert head.aimed > 0, "an off-centre face should be re-aimed at"


def test_enroll_holds_still_on_a_centred_face():
    head = Head()
    v = Vision()
    v.face_embed = lambda jpeg, seq=0: {"embedding": [1.0], "face_count": 1, "quality": 0.9,
                                        "face_bbox": [24, 14, 40, 34]}       # centre of the 64x48 frame
    assert face_tasks.enroll(HeadCamera(head), v, "sam", seconds=30, clock=Clock(), head=head)
    assert head.aimed == 0


class CutFace(Vision):
    def face_embed(self, jpeg, seq=0):
        out = super().face_embed(jpeg, seq)
        out["face_bbox"] = [10, -12, 30, 20]         # top of the face above the frame
        return out


def test_enroll_keeps_no_shot_with_the_face_cut_off():
    v = CutFace()
    assert not face_tasks.enroll(Camera(), v, "sam", seconds=30, clock=Clock())
    assert v.enrolled is None


def test_full_face_scales_wire_boxes_back_to_the_frame():
    big = SimpleNamespace(main=np.zeros((1080, 1920, 3), np.uint8))
    assert face_tasks.full_face(big, [800, 300, 1000, 560])
    assert not face_tasks.full_face(big, [800, -40, 1000, 560])


def test_match_scans_right_to_left_upward_only_centring_and_checking_each_face():
    head = Head()
    v = sees(head, sim=0.0)                                      # never matches: the scan runs to the end
    assert face_tasks.match(HeadCamera(head), v, seconds=1000, clock=Clock(), head=head) is None
    assert all(tilt >= 0 for _, tilt in head.moves), "looked below the base position"
    assert head.moves[:7] == [(p, 0.0) for p in (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)]
    scan = [m for m in head.moves if m[0] in settings.FACE_SEARCH_PANS and m[1] in settings.FACE_SEARCH_TILTS]
    assert len(scan) == 21, "one full scan, no repeats"
    seen_at = [m for m in scan if m[0] >= 15 and m[1] >= 20]
    assert len(v.checks) == len(seen_at), "each face matched once"
    assert all(c[0] % 30 == 2 and c[1] % 10 == 1 for c in v.checks), "matched only after centring on it"


def test_the_same_face_seen_from_two_positions_is_matched_once():
    head = Head()
    head.pixel_angles = lambda frame, u, v: (70.0, 35.0)        # one person, wherever the head looks
    v = sees(head, sim=0.0)
    face_tasks.match(HeadCamera(head), v, seconds=1000, clock=Clock(), head=head)
    assert len(v.checks) == 1


def test_an_unreachable_mac_says_so_instead_of_no_face(monkeypatch):
    from robot_core import status
    from robot_core.vision_client import VisionError
    shown = []
    status.set_sink(shown.append)
    v = Vision()
    v.face_embed = lambda jpeg, seq=0: (_ for _ in ()).throw(VisionError("POST /v1/faces/embed: No route to host"))
    try:
        assert face_tasks.match(Camera(), v, seconds=5, clock=Clock()) is None
    finally:
        status.set_sink(None)
    codes = [m["v"] for m in shown if m["k"] == "error"]
    assert codes == ["V01"]


def test_head_match_reports_unknown_when_nobody_is_found():
    head = Head()
    v = sees(head, sim=0.9)
    head.pose = (0.0, 0.0)
    assert face_tasks.match(HeadCamera(head), v, seconds=30, clock=Clock(), head=head) == "sam"
    nobody = sees(Head(), faces=0)
    h2 = nobody.head
    assert face_tasks.match(HeadCamera(h2), nobody, seconds=30, clock=Clock(), head=h2) is None
    assert all(t >= 0 for _, t in h2.moves) and len(h2.moves) > 4      # it did sweep


def test_a_cancelled_enroll_saves_nothing_and_a_cancelled_match_finds_nobody():
    from robot_core import task_cancel
    task_cancel.request("service")
    try:
        v = Vision()
        assert not face_tasks.enroll(Camera(), v, "sam", seconds=30, clock=Clock())
        assert v.enrolled is None
        assert face_tasks.match(Camera(), Vision(sim=0.9), seconds=5, clock=Clock()) is None
    finally:
        task_cancel.reset()
