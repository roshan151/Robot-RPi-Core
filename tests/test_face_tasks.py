"""Voice-triggered face enroll / match, against a fake camera and vision service."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_core import face_tasks  # noqa: E402


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


def test_head_searches_upward_only_then_locks_on_the_face_and_holds():
    head = Head()
    v = SeesFaceAt(head)
    assert face_tasks.enroll(HeadCamera(head), v, "sam", seconds=30, clock=Clock(), head=head)
    assert all(tilt >= 0 for _, tilt in head.moves), "looked below the base position"
    assert head.aimed == 1, "should centre on the face once, then hold"
    last_search = head.moves[-1]
    assert last_search[0] >= 15 and last_search[1] >= 20       # stopped searching where it found it


def test_head_match_reports_unknown_when_nobody_is_found():
    head = Head()
    v = SeesFaceAt(head, sim=0.9)
    head.pose = (0.0, 0.0)
    assert face_tasks.match(HeadCamera(head), v, seconds=30, clock=Clock(), head=head) == "sam"
    nobody = SeesFaceAt(Head(), faces=0)
    h2 = nobody.head
    assert face_tasks.match(HeadCamera(h2), nobody, seconds=30, clock=Clock(), head=h2) is None
    assert all(t >= 0 for _, t in h2.moves) and len(h2.moves) > 4      # it did sweep
