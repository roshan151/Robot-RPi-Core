"""Mic choice: buds when their mic is up, else the camera's (docs/USB-CAMERA-MIC.md)."""

from __future__ import annotations

import asyncio
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_core import audio_route, settings  # noqa: E402

CAM = "alsa_input.usb-innomaker_USB_Camera-02.mono-fallback"
BUDS_IN, BUDS_OUT = "bluez_input.B4_84_D5_B9_14_23.0", "bluez_output.B4_84_D5_B9_14_23.1"


def fake_pactl(monkeypatch, sources=(), sinks=(), missing=False):
    calls = []

    def run(cmd, **kw):
        calls.append(cmd[1:])
        if missing:
            raise FileNotFoundError("pactl")
        rows = {"sources": sources, "sinks": sinks}.get(cmd[-1], ()) if cmd[1:3] == ["list", "short"] else ()
        out = "".join(f"{i}\t{n}\tPipeWire\ts16le 1ch 16000Hz\tSUSPENDED\n" for i, n in enumerate(rows))
        return types.SimpleNamespace(stdout=out)

    monkeypatch.setattr(audio_route.subprocess, "run", run)
    return calls


def test_buds_mic_up_means_buds_for_everything(monkeypatch):
    fake_pactl(monkeypatch, sources=[CAM, BUDS_IN, BUDS_IN + ".monitor"], sinks=["alsa_output.hdmi", BUDS_OUT])
    assert audio_route.current() == audio_route.Route("buds", BUDS_IN, BUDS_OUT)


def test_no_buds_mic_means_the_camera_mic(monkeypatch):
    fake_pactl(monkeypatch, sources=[CAM, CAM + ".monitor"], sinks=[BUDS_OUT])   # buds on music only
    assert audio_route.current() == audio_route.Route("room", CAM)


def test_bt_mac_picks_those_buds(monkeypatch):
    other = "bluez_input.AA_BB_CC_DD_EE_FF.0"
    fake_pactl(monkeypatch, sources=[other, BUDS_IN])
    monkeypatch.setattr(settings, "BT_MAC", "b4:84:d5:b9:14:23")
    assert audio_route.current().source == BUDS_IN


def test_no_pactl_is_room_mode_and_changes_nothing(monkeypatch):
    fake_pactl(monkeypatch, missing=True)
    assert audio_route.use() == audio_route.Route("room")


def test_mic_mode_can_be_forced(monkeypatch):
    fake_pactl(monkeypatch, sources=[CAM, BUDS_IN])
    monkeypatch.setattr(settings, "MIC_MODE", "room")
    assert audio_route.current() == audio_route.Route("room", CAM)


def test_use_makes_the_choice_the_default(monkeypatch):
    calls = fake_pactl(monkeypatch, sources=[BUDS_IN], sinks=[BUDS_OUT])
    audio_route.use()
    assert ["set-default-source", BUDS_IN] in calls and ["set-default-sink", BUDS_OUT] in calls
    calls = fake_pactl(monkeypatch, sources=[CAM])
    audio_route.use()
    assert ["set-default-source", CAM] in calls and not any(c[0] == "set-default-sink" for c in calls)


def test_checks_every_30_s_for_5_minutes_then_every_2(monkeypatch):
    now = audio_route.time.monotonic()
    monkeypatch.setattr(audio_route, "_T0", now - 60)
    assert audio_route.poll_interval() == 30
    monkeypatch.setattr(audio_route, "_T0", now - 301)
    assert audio_route.poll_interval() == 120


def test_only_the_room_mic_needs_the_name():
    assert settings.live_prompt("buds") == settings.LIVE_SYSTEM_PROMPT
    room = settings.live_prompt("room")
    assert room.startswith(settings.LIVE_SYSTEM_PROMPT) and settings.ROOM_MIC_RULE in room


def test_buds_coming_or_going_ends_the_session_and_it_reconnects_with_the_other_prompt(monkeypatch):
    """The session's route check ends it; the supervisor brakes, then reconnects
    with the new mic's prompt."""
    from robot_core.live import agent as agent_mod

    routes = iter([audio_route.Route("room", CAM), audio_route.Route("buds", BUDS_IN)])
    monkeypatch.setattr(audio_route, "use", lambda: next(routes))
    prompts, braked = [], []

    class Fake:
        n = 0

        def __init__(self, tools, system_prompt=None, route=None, **kw):
            prompts.append(system_prompt)
            self.rerouted = False

        async def run(self):
            Fake.n += 1
            if Fake.n == 1:
                self.rerouted = True               # buds came on
                return
            raise asyncio.CancelledError          # end the test

    monkeypatch.setattr(agent_mod, "LiveAgent", Fake)
    tools = types.SimpleNamespace(pending=None, motion=types.SimpleNamespace(stop=lambda: braked.append(1)))
    try:
        asyncio.run(agent_mod._run_supervised(tools))
    except asyncio.CancelledError:
        pass
    assert prompts == [settings.live_prompt("room"), settings.live_prompt("buds")]
    assert braked == [1]


def test_the_session_route_check_notices_the_change(monkeypatch):
    from robot_core.live.agent import LiveAgent
    monkeypatch.setattr(audio_route, "poll_interval", lambda: 0.01)
    monkeypatch.setattr(audio_route, "current", lambda: audio_route.Route("buds", BUDS_IN))

    async def go():
        a = LiveAgent(types.SimpleNamespace(motion=None, pending=None), route=audio_route.Route("room", CAM))
        await asyncio.wait_for(a._pump_route(), 2)
        return a.rerouted

    assert asyncio.run(go())
