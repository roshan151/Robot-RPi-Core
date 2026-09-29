from __future__ import annotations

import pytest

from robot_core.perception.geometry import CameraModel, iou, to_base_bearing, wrap_deg

CAM = CameraModel(640, 360, 66.0)


def test_center_is_zero_and_right_is_positive():
    assert CAM.bearing_deg(320) == pytest.approx(0.0)
    assert CAM.bearing_deg(640) == pytest.approx(33.0, abs=0.01)
    assert CAM.bearing_deg(0) == pytest.approx(-33.0, abs=0.01)
    assert CAM.elevation_deg(0) > 0          # top of image is up


def test_scaled_model_gives_same_angles():
    small = CAM.scaled(320, 180)
    assert small.bearing_deg(240) == pytest.approx(CAM.bearing_deg(480))


def test_person_distance_from_height():
    # A 1.7 m person 4 m away spans fy * 1.7 / 4 pixels.
    h = CAM.fy * 1.7 / 4.0
    d = CAM.distance_m((300, 50, 340, 50 + h), "person")
    assert d == pytest.approx(4.0, abs=0.02)


def test_truncated_person_falls_back_to_shoulders_never_too_far():
    # Cut by the bottom edge: height would say "far"; shoulder width must win.
    w = CAM.fx * 0.45 / 1.0
    d = CAM.distance_m((320 - w / 2, 50, 320 + w / 2, 360), "person")
    assert d == pytest.approx(1.0, abs=0.02)


def test_unknown_class_has_no_distance():
    assert CAM.distance_m((10, 10, 50, 50), "frisbee-shaped-thing") is None


def test_base_bearing_adds_pan_and_wraps():
    assert to_base_bearing(10.0, 30.0) == 40.0
    assert wrap_deg(190.0) == -170.0


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(1 / 3)
