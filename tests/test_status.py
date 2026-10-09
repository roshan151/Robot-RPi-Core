import json
import logging

import pytest

from robot_core import status
from robot_core.power import PowerMonitor, classify


class Clock:
    t = 0.0
    def __call__(self): return self.t


def board():
    c = Clock()
    return status.Board(c), c


def test_every_row_fits_the_screen_and_messages_are_ten_words():
    b, _ = board()
    for k, v in (("task", "explore the whole house now please"), ("tool", "x " * 40),
                 ("detail", " ".join(["word"] * 40)), ("result", "a " * 30)):
        b.apply({"k": k, "v": v})
    b.apply({"k": "error", "v": "P01", "t": "q " * 30})
    rows = b.lines()
    assert len(rows) == status.ROWS and all(len(r) <= status.WIDTH for r in rows[1:]) and len(rows[0]) == 26
    assert len(b.detail.split()) <= status.MAX_WORDS and len(b.tool.split()) <= status.MAX_WORDS


def test_timer_counts_down_and_result_replaces_it():
    b, c = board()
    b.apply({"k": "timer", "v": "enroll Sam", "n": 30})
    c.t = 13
    assert b.lines()[4] == "enroll Sam 0:17"
    b.apply({"k": "result", "v": "Sam enrolled", "n": 6})
    assert b.lines()[4] == "Sam enrolled"
    c.t = 20
    assert b.lines()[4] == ""


def test_error_box_and_new_task_clears_tool_but_keeps_result():
    b, _ = board()
    b.apply({"k": "tool", "v": "drive 1"})
    b.apply({"k": "error", "v": "F02"})
    b.apply({"k": "result", "v": "Roshan matched", "n": 6})
    b.apply({"k": "task", "v": "voice"})
    rows = b.lines()
    assert b.frame().error == ["E-F02 several faces"] and rows[1] == "" and rows[4] == "Roshan matched"


def test_error_box_shows_the_whole_message_then_goes_away():
    b, c = board()
    assert b.frame().error == [] and not any("E-" in r for r in b.lines())
    b.apply({"k": "error", "v": "G02", "t": "pan limited to -50..+9"})
    assert b.frame().error == ["E-G02 servo range limited", "pan limited to -50..+9"]
    c.t = status.ERROR_HOLD_S + 1
    assert b.frame().error == [] and b.code == "G02"
    for code, text in status.CODES.items():
        assert len(f"E-{code} {text}") <= status.SMALL_WIDTH


def test_animation_is_eyes_until_asked_and_times_out():
    b, c = board()
    assert b.frame().anim == ""
    b.apply({"k": "anim", "v": "tank", "n": 10})
    assert b.frame().anim == "tank"
    c.t = 11
    assert b.frame().anim == ""


def test_sink_forwards_json_and_never_raises():
    got = []
    status.set_sink(lambda m: got.append(json.dumps(m)))
    try:
        status.detail("plant_03 capturing 4/8")
        assert json.loads(got[0]) == {"k": "detail", "v": "plant_03 capturing 4/8"}
        status.set_sink(lambda m: 1 / 0)
        status.detail("still fine")
    finally:
        status.set_sink(None)


@pytest.mark.parametrize("name,msg,code", [
    ("robot_core.face_tasks", "enroll rejected: 2 faces in view", "F02"),
    ("robot_core.face_tasks", "enroll: only 3 usable frames, nothing saved", "F03"),
    ("robot_core.sensors.gimbal", "gimbal: pan limited to -50..+9", "G02"),
    ("estop", "", "S03"), ("voice.drop", "closed", "L01"), ("voice.tool", "unknown tool", "L03"),
    ("robot_core.vision_client", "request timed out", "V02"),
])
def test_log_records_map_to_codes(name, msg, code):
    assert status.code_for(name, msg) == code and code in status.CODES


def test_log_handler_posts_codes_only_for_warnings():
    b = status.Board()
    status.set_sink(b.apply)
    h = status.LogHandler()
    try:
        log = logging.getLogger("robot_core.face_tasks")
        log.addHandler(h)
        log.warning("enroll rejected: %d faces in view", 2)
        assert b.code == "F02"
        log.propagate = False
        b.code = ""
        log.info("enroll rejected: 2 faces in view")
        assert b.code == ""
    finally:
        log.removeHandler(h)
        log.propagate = True
        status.set_sink(None)


def test_power_dip_counted_once_with_cause_and_note_survives(tmp_path):
    b, c = board()
    status.set_sink(b.apply)
    try:
        flags = [0]
        f = tmp_path / "dip.json"
        m = PowerMonitor(b, read=lambda: flags[0], last_file=f, clock=c)
        c.t = 300
        b.apply({"k": "tool", "v": "drive 0.5m"})
        m.poll_once()
        flags[0] = 0x10001                       # under-voltage now + since boot
        m.poll_once(); m.poll_once()
        assert m.dips == 1 and b.code == "P01" and b.cause == "wheels drawing current"
        assert b.power == "DIP" and json.loads(f.read_text())["cause"] == "wheels drawing current"
        flags[0] = 0x10000
        m.poll_once()
        assert b.power == "OK"
        flags[0] = 0x10001
        m.poll_once()
        assert m.dips == 2
        m2 = PowerMonitor(status.Board(c), read=lambda: 0, last_file=f, clock=c)
        m2.show_last_boot()
        assert "wheels" in b.note and not f.exists()
    finally:
        status.set_sink(None)


def test_classify_rules():
    assert classify("explore", "", "", 10) == "right after boot"
    assert classify("voice", "look_up 30", "", 500) == "head servo moving"
    assert classify("enroll_face", "", "", 500) == "camera and vision load"
    assert classify("", "", "", 500, battery=10) == "low battery"


def test_oled_renders_rows_to_a_dummy_device():
    pytest.importorskip("luma.core")
    from luma.core.device import dummy
    from robot_core.oled import ANIMS, draw, pose
    dev = dummy(width=128, height=64)
    seen = set()
    for anim in list(ANIMS) + ["no such animation"]:
        for p in set(ANIMS.get(anim, ANIMS[""])[0]):
            draw(dev, status.Frame(["PWR OK", "TASK explore"] + [""] * 6, 82.0, ["E-P01 UNDERVOLT NOW"], anim), p)
            assert dev.image.getbbox() is not None and dev.image.size == (128, 64)
            seen.add(dev.image.tobytes())
    assert len(seen) > 4 and pose("tank", 0.0) != pose("tank", 0.13)


def test_power_events_land_in_logs_json_without_any_screen(tmp_path):
    import json as j
    from robot_core import robot_log
    robot_log._state.update(handler=None, path=None)
    path = robot_log.setup(str(tmp_path / "logs.json"))
    try:
        b, c = board()
        flags = [0]
        m = PowerMonitor(b, read=lambda: flags[0], last_file=tmp_path / "d.json", clock=c)   # no Oled anywhere
        c.t = 300
        m.poll_once()
        flags[0] = 0x10001
        m.poll_once()
        c.t = 302
        flags[0] = 0x10000
        m.poll_once()
        rows = [j.loads(line) for line in open(path)]
        dip = next(r for r in rows if r["evt"] == "power.dip")
        clear = next(r for r in rows if r["evt"] == "power.clear")
        assert dip["lvl"] == "warn" and dip["n"] == 1 and "cause" in dip and clear["secs"] == 2.0
    finally:
        robot_log._state.update(handler=None, path=None)
        for h in list(__import__("logging").getLogger().handlers):
            __import__("logging").getLogger().removeHandler(h)


def test_header_is_task_power_uptime_and_the_battery_is_only_an_icon():
    b, _ = board()
    assert b.lines()[0].split() == ["idle", "OK", "0m"] and b.frame().battery is None
    b.apply({"k": "battery", "n": 82.0, "volts": 4.12, "amps": -0.85})
    b.apply({"k": "power", "v": "DIP", "n": 3})
    assert b.lines()[0].split() == ["idle", "DIP", "x3", "0m"] and b.frame().battery == 82.0
    assert not any(c in "".join(b.lines()) for c in "%VA") and b.volts == 4.12
