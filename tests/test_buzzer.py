"""The buzzer's three sounds and when each is played (robot_core/buzzer.py)."""

from robot_core import buzzer, status


class Rig:
    def __init__(self):
        self.t, self.notes = 0.0, []
        self.b = buzzer.Buzzer(self.notes.append, clock=lambda: self.t, sleep=lambda s: None)

    def heard(self):
        """Play whatever is due now; which tune it was (by its first note), or None for silence."""
        self.notes.clear()
        self.b.step(timeout=0.01)
        first = next((n for n in self.notes if n), None)
        return next((name for name, tune in buzzer.TUNES.items() if tune[0][0] == first), None)


def test_boot_blips_until_the_voice_session_is_up_then_plays_online_once():
    r = Rig()
    assert r.heard() == "connecting" and r.heard() == "connecting"      # booting
    r.b.on_message({"k": "sound", "v": "online"})
    assert r.heard() == "online"
    assert r.notes[-1] == 0, "always ends silent"
    assert r.heard() is None, "then quiet: waiting for a command"
    r.b.on_message({"k": "sound", "v": "online"})                        # session-length reconnect
    assert r.heard() is None


def test_a_lost_session_blips_until_it_is_back_and_gives_up_after_a_while():
    r = Rig()
    r.b.on_message({"k": "sound", "v": "online"}); r.heard()
    r.b.on_message({"k": "error", "v": "L01"})
    assert r.heard() == "connecting" and r.heard() == "connecting"
    r.b.on_message({"k": "sound", "v": "online"})
    assert r.heard() == "online" and r.heard() is None
    r.b.on_message({"k": "error", "v": "L01"})
    r.t += buzzer.CONNECT_FOR_S + 1
    assert r.heard() is None, "a lost 'online' cannot leave it blipping for ever"


def test_failures_sound_once_however_often_they_are_reported_and_end_the_blips():
    r = Rig()
    for _ in range(3):
        r.b.on_message({"k": "error", "v": "X02"})
    assert r.heard() == "failure"
    assert r.heard() is None, "no connecting blips after a failure"
    r.t += buzzer.REPEAT_GAP_S + 1
    r.b.on_message({"k": "error", "v": "S01"})
    assert r.heard() == "failure"


def test_a_passing_error_is_one_pair_of_blips_and_other_messages_are_silent():
    r = Rig()
    r.b.on_message({"k": "sound", "v": "online"}); r.heard()
    r.b.on_message({"k": "error", "v": "V02"})
    assert r.heard() == "connecting" and r.heard() is None
    for msg in ({"k": "error", "v": "F01"}, {"k": "detail", "v": "x"}, {"k": "task", "v": "voice"}, {}):
        r.b.on_message(msg)
    assert r.heard() is None


def test_explore_without_a_voice_session_stops_the_boot_blips():
    r = Rig()
    r.b.on_message({"k": "task", "v": "voice"})
    assert r.heard() == "connecting"
    r.b.on_message({"k": "task", "v": "explore"})
    assert r.heard() is None


def test_every_code_the_buzzer_knows_exists_and_status_sound_reaches_it():
    assert (buzzer.FAILURES | buzzer.RECONNECTING | buzzer.TRANSIENT) <= set(status.CODES)
    got = []
    status.set_sink(got.append)
    try:
        status.sound("online")
    finally:
        status.set_sink(None)
    assert got == [{"k": "sound", "v": "online"}]
