"""The TF-Luna drop guard: learns the floor standing still, brakes forward drives on a
drop, a low obstacle or lost floor, never on one noisy reading, never when not fitted."""

from robot_core.drop_guard import DropGuard


def test_learns_the_floor_then_brakes_on_a_drop_but_not_on_one_noisy_reading():
    g = DropGuard(learn_n=5)
    for cm in (33, 34, 33, 32, 33):
        assert g.update(cm, forward=False) is None
    assert g.armed and g.floor_cm == 33
    assert g.update(34, True) is None
    assert g.update(60, True) is None                 # one long reading: noise
    assert g.update(33, True) is None
    assert g.update(60, True) is None
    assert "drop ahead" in g.update(61, True)


def test_low_obstacle_and_lost_floor_brake_reverse_and_turns_do_not():
    g = DropGuard(floor_cm=33, misses=3)
    g.update(33, False)                               # one valid reading: the sensor is there
    assert g.update(20, True) is None and "low" in g.update(21, True)
    assert g.update(None, True) is None and g.update(None, True) is None
    assert "not seen" in g.update(None, True)
    for _ in range(5):
        assert g.update(80, forward=False) is None    # reversing or turning: never


def test_not_fitted_or_old_firmware_never_brakes():
    g = DropGuard(floor_cm=33)
    for _ in range(10):
        assert g.update(None, True) is None           # every reading -1: no sensor
    g2 = DropGuard()
    for _ in range(10):
        assert g2.update(None, False) is None
    assert not g2.armed
