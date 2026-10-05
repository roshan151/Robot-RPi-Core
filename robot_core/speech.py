"""
Spoken output — Gemini text-to-speech, cached to disk.

This module replaces the two things it was split across before: `audio_cues.py`
(rendered sine tones) and `speech.py` (espeak-ng / Nix TTS). Both are gone. The
robot now says what it means in one voice, and the ordering rules that made the
tones safe are unchanged and still the whole trick:

    * an utterance plays BEFORE the microphone stream is opened, never while
      it is live. If the stream is already open the audio lands in the capture
      buffer and is streamed to the model as if the operator had said it.
    * on the failure path it plays AFTER the session is torn down, for the same
      reason in reverse: by then nothing is listening.

`guard_s` covers the room's reverb tail after playback, before capture begins.

One key, one host
-----------------
Speech goes to generativelanguage.googleapis.com with the same GEMINI_API_KEY
the Live session uses. Google Cloud Text-to-Speech (WaveNet) would have been
cheaper per utterance, but it is a different API on a different host: AI Studio
keys are restricted to the Generative Language API, so it would have meant a
second credential on a billing-enabled Cloud project. At this volume — five
cached phrases and one battery line a boot — that trade is not worth a key.

The bill that does apply is per project, not per key, and the Live session is
on the same project. Which is one more reason the cache matters.

Why the cache is not optional
-----------------------------
Synthesis is a network call. The two moments the robot speaks are the two
moments the network is least trustworthy: boot (Wi-Fi may not be associated
yet) and failure (the session just died, possibly because the link did).
Anything that must be sayable when the network is down has to already be on
disk.

So: every synthesis is written to `TTS_CACHE_DIR` keyed by a hash of the text
and the voice parameters, and a cache hit never touches the network. Static
phrases are primed once — `prime()` at first successful boot is enough to make
the robot permanently able to announce its own failure offline.

Dynamic text (battery readings, error descriptions) misses the cache the first
time it is said and is synthesized live. If that call fails, `say()` falls back
to `fallback_text` when the caller supplied one — which is how the error path
still speaks something useful with no network at all.

The cache does a second job here that it would not have done for WaveNet. Gemini
TTS is a language model, so the same input renders differently on each call —
different pacing, different emphasis. Caching freezes one take, and the robot
says "Robot online" the same way every boot instead of reinterpreting it.

Playback backends, in order:
    1. sounddevice + numpy      (in-process, exact timing)
    2. aplay / paplay           (subprocess, no numpy needed)
    3. silence                  (logged, structured, and carry on)

Nothing here raises. Speech failing must never take the robot down, and on the
failure path this module runs when other things are already broken.
"""

from __future__ import annotations

import base64
import hashlib
import io
import json
import logging
import shutil
import subprocess
import threading
import time
import wave
from pathlib import Path
from typing import Dict, Optional

from robot_core import settings
from robot_core import robot_log

logger = logging.getLogger(__name__)

GENERATE_URL = (
    "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
)

# Statuses worth trying again. 500 is the documented random failure mode of the
# TTS models (they occasionally emit text tokens instead of audio and the server
# rejects its own response); 429/503 are load. A 400 or 403 is a configuration
# mistake and retrying it just makes the same mistake twice.
RETRY_STATUSES = frozenset({429, 500, 503})

# Phrases the robot must be able to say with no network. `prime()` renders
# these; after one successful boot they live in the cache forever.
#
# EXACTLY TWO EVENTS, for the same reason there were exactly two tones. The
# Live session holds the microphone open continuously, so every sound the robot
# makes is heard by the model as if you had said it. A per-turn announcement is
# not a convenience, it is interference — and there is no listening state to
# announce any more, because the robot is always listening.
#
#   connected  the session is up. Plays BEFORE the stream opens.
#   error      the session has died. Plays AFTER it is torn down.
#
# Do not add a third without a reason that beats "it goes into the microphone".
STATIC_PHRASES: Dict[str, str] = {
    "connected": "Robot online. Voice session connected.",
    # Said alone when there is no description, and used as the offline fallback
    # for the described form below.
    "error": "Voice session failed. Check the log.",
    "battery_unknown": "Battery level unknown.",
}

_lock = threading.Lock()
_sd = None
_np = None
_player: Optional[str] = None


# --------------------------------------------------------------------------- #
# Playback
# --------------------------------------------------------------------------- #

def _pick_backend() -> str:
    """Choose a playback backend once. Never raises."""
    global _sd, _np, _player
    if _player is not None:
        return _player

    try:
        import numpy as np  # type: ignore
        import sounddevice as sd  # type: ignore

        _sd, _np = sd, np
        _player = "sounddevice"
        return _player
    except Exception as e:
        logger.debug("sounddevice/numpy unavailable for playback: %s", e)

    for exe in ("paplay", "aplay"):
        if shutil.which(exe):
            _player = exe
            return _player

    # Structured, not just a log line: with no screen and no voice, "the robot
    # is mute" has to be greppable in logs.json rather than buried in console
    # output nobody is watching.
    robot_log.event(
        "audio.error", logging.WARNING, stage="playback-backend",
        err="no playback backend (no sounddevice/numpy, no aplay, no paplay)",
        fix="pip install sounddevice numpy, or apt install alsa-utils",
    )
    _player = "none"
    return _player


def backend() -> str:
    """Which playback backend is in use: sounddevice / aplay / paplay / none."""
    return _pick_backend()


def available() -> bool:
    """True if the robot can make sound at all — cache or network aside."""
    return settings.TTS_ENABLED and _pick_backend() != "none"


def _wav_bytes(pcm_or_wav: bytes) -> bytes:
    """Gemini returns headerless 24 kHz 16-bit mono PCM, so we add the RIFF
    header ourselves. The check is for the day an endpoint returns a real WAV —
    wrapping one twice produces a file that plays four bytes of noise."""
    if pcm_or_wav[:4] == b"RIFF":
        return pcm_or_wav
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(settings.TTS_SAMPLE_RATE)
        w.writeframes(pcm_or_wav)
    return buf.getvalue()


def _play_wav(data: bytes) -> bool:
    """Play a WAV and block until it has finished.

    Blocking is deliberate. Callers speak immediately before opening the
    microphone or immediately before exiting, and returning early would put the
    speech into the capture buffer or let process exit cut it off.
    """
    p = _pick_backend()
    if p == "none":
        return False

    try:
        if p == "sounddevice":
            with wave.open(io.BytesIO(data), "rb") as w:
                frames = w.readframes(w.getnframes())
                rate = w.getframerate()
                channels = w.getnchannels()
            arr = _np.frombuffer(frames, dtype="<i2")
            if channels > 1:
                arr = arr.reshape(-1, channels)
            kw = {"device": settings.TTS_DEVICE} if settings.TTS_DEVICE else {}
            _sd.play(arr, rate, blocking=True, **kw)
            return True

        cmd = [p]
        if p == "aplay":
            cmd += ["-q"]
            if settings.TTS_DEVICE:
                cmd += ["-D", settings.TTS_DEVICE]
        elif settings.TTS_DEVICE:
            cmd += ["--device", settings.TTS_DEVICE]
        cmd += ["-"]
        subprocess.run(cmd, input=data, timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        # Never propagate. A missing speaker must not stop the robot.
        logger.warning("playback failed (%s): %s", p, e)
        return False


# --------------------------------------------------------------------------- #
# Cache + synthesis
# --------------------------------------------------------------------------- #

def _cache_dir() -> Path:
    d = Path(settings.TTS_CACHE_DIR).expanduser()
    d.mkdir(parents=True, exist_ok=True)
    return d


def _cache_path(text: str) -> Path:
    """Key on everything that changes the audio, not just the words.

    Model, voice and style direction all alter the rendering, so a change to
    any of them has to miss the cache. Otherwise switching voices leaves the
    robot speaking in the old one until someone thinks to delete the directory.
    """
    key = "|".join((
        text,
        settings.TTS_MODEL,
        settings.TTS_VOICE,
        settings.TTS_STYLE,
        str(settings.TTS_SAMPLE_RATE),
    ))
    return _cache_dir() / f"{hashlib.sha1(key.encode()).hexdigest()}.wav"


def _normalize(text: str) -> str:
    """Terminal punctuation, always.

    Not cosmetic. A prompt ending mid-phrase — "Battery 55 percent, 3.6 volts"
    — reads to the speech classifier like an instruction that was cut off, and
    the observed result is `finishReason: OTHER` with zero output tokens: a
    200 response containing no audio at all. The static phrases all ended in a
    period and synthesized fine; the battery line did not and never once
    succeeded. One character.
    """
    t = " ".join(text.split())
    return t if not t or t[-1] in ".!?" else t + "."


def _prompt(text: str) -> str:
    """`Say clearly and calmly: Robot online.`

    The directive prefix is the shape Google's single-speaker example uses, and
    it matters for more than tone: a bare transcript can fail to trigger the
    speech classifier, which either rejects the request as PROHIBITED_CONTENT
    or — worse, because it is silent about it — makes the model read the text
    as if it were an instruction. Keep the style short for the same reason.
    """
    style = settings.TTS_STYLE.strip().rstrip(":")
    body = _normalize(text)
    return f"{style}: {body}" if style else body


def _why_no_audio(payload: dict) -> dict:
    """Name the reason a 200 came back mute.

    `finishReason: OTHER` with promptTokenCount == totalTokenCount means the
    model generated nothing at all — the speech classifier declined the prompt
    rather than the request being malformed. That is a very different fix from
    a blocked-content stop, and without pulling these fields out both look
    identical in the log: "200 with no audio".
    """
    out: dict = {}
    cands = payload.get("candidates") or []
    if cands and isinstance(cands[0], dict):
        out["finish"] = cands[0].get("finishReason")
    fb = payload.get("promptFeedback")
    if isinstance(fb, dict):
        out["blocked"] = fb.get("blockReason")
    usage = payload.get("usageMetadata")
    if isinstance(usage, dict):
        prompt_t = usage.get("promptTokenCount")
        total_t = usage.get("totalTokenCount")
        out["tokens"] = f"{prompt_t}/{total_t}"
        if prompt_t is not None and prompt_t == total_t:
            out["note"] = "model produced no output tokens"
    return out


def _extract_audio(payload: dict) -> Optional[bytes]:
    """Pull the first audio blob out of a generateContent response.

    Written as a walk rather than a fixed path because the audio has sat at
    two different depths across API revisions (`inlineData` under a part, and
    `output_audio` on the newer Interactions shape). A missing key here would
    present as a mute robot with a 200 in the log, which is the least
    debuggable failure this module has.
    """
    stack = [payload]
    while stack:
        node = stack.pop()
        if isinstance(node, dict):
            for k in ("inlineData", "inline_data", "output_audio", "outputAudio"):
                blob = node.get(k)
                if isinstance(blob, dict) and blob.get("data"):
                    return base64.b64decode(blob["data"])
            stack.extend(node.values())
        elif isinstance(node, list):
            stack.extend(node)
    return None


def _request(model: str, prompt: str, api_key: str, requests_mod) -> tuple:
    """One call. Returns (audio_or_None, retryable, detail_for_the_log)."""
    try:
        resp = requests_mod.post(
            GENERATE_URL.format(model=model),
            headers={"x-goog-api-key": api_key, "Content-Type": "application/json"},
            json={
                "contents": [{"parts": [{"text": prompt}]}],
                "generationConfig": {
                    "responseModalities": ["AUDIO"],
                    "speechConfig": {
                        "voiceConfig": {
                            "prebuiltVoiceConfig": {"voiceName": settings.TTS_VOICE},
                        },
                    },
                },
            },
            timeout=settings.TTS_TIMEOUT_S,
        )
    except Exception as e:
        return None, True, {"err": f"{type(e).__name__}: {e}"}

    if resp.status_code == 200:
        try:
            payload = resp.json()
        except Exception as e:
            return None, True, {"err": f"unparseable 200: {type(e).__name__}: {e}"}
        audio = _extract_audio(payload)
        if audio:
            return audio, False, {}
        return None, True, {"err": "200 with no audio", **_why_no_audio(payload)}

    # Body quoted, not parsed: Google's message names the fix more often than
    # the status does ("model not found" for a non-TTS model id, "API key not
    # valid", quota exceeded, and so on).
    return (None, resp.status_code in RETRY_STATUSES,
            {"status": resp.status_code, "err": resp.text[:300]})


def _synthesize(text: str) -> Optional[bytes]:
    """Gemini TTS with retries, then a fallback model. WAV bytes or None.

    The fallback exists because the preview TTS models fail in ways that are
    specific to the model rather than to the request: a bad minute on
    2.5-flash-preview-tts produces `finishReason: OTHER` or a 500 on every
    attempt, and no amount of retrying the same endpoint fixes it. Trying a
    different model is the only retry that changes anything.
    """
    try:
        api_key = settings.require("GEMINI_API_KEY")
    except Exception as e:
        robot_log.event("audio.error", logging.WARNING, stage="tts-auth",
                        err=f"{type(e).__name__}: {e}",
                        fix="set GEMINI_API_KEY in /etc/robot.env")
        return None

    try:
        import requests  # imported here so config-only tools need no network stack
    except ImportError as e:
        robot_log.event("audio.error", logging.WARNING, stage="tts-import",
                        err=str(e), fix="pip install requests")
        return None

    prompt = _prompt(text)
    models = [settings.TTS_MODEL]
    if settings.TTS_FALLBACK_MODEL and settings.TTS_FALLBACK_MODEL != settings.TTS_MODEL:
        models.append(settings.TTS_FALLBACK_MODEL)

    for model in models:
        for attempt in range(1, settings.TTS_RETRIES + 2):
            audio, retryable, detail = _request(model, prompt, api_key, requests)
            if audio:
                if model != settings.TTS_MODEL:
                    robot_log.event("audio.error", logging.WARNING,
                                    stage="tts-fallback-model", model=model,
                                    err=f"{settings.TTS_MODEL} produced no audio",
                                    fix=f"set ROBOT_TTS_MODEL={model} to skip the wait")
                return _wav_bytes(audio)

            robot_log.event("audio.error", logging.WARNING, stage="tts-response",
                            model=model, attempt=attempt, **detail)
            if not retryable:
                break                     # config error; the next model may differ
            if attempt <= settings.TTS_RETRIES:
                time.sleep(min(2 ** (attempt - 1) * 0.5, 4.0))

    return None


def render(text: str, *, allow_network: bool = True) -> Optional[bytes]:
    """WAV bytes for `text` — from cache, else synthesized and cached.

    `allow_network=False` answers "can this be said offline right now?" without
    a request, which is what the failure path uses to pick its fallback.
    """
    path = _cache_path(text)
    try:
        if path.is_file() and path.stat().st_size > 0:
            return path.read_bytes()
    except OSError as e:
        logger.debug("tts cache read failed for %s: %s", path.name, e)

    if not allow_network:
        return None

    data = _synthesize(text)
    if data is None:
        return None

    try:
        # Write-then-rename: a truncated file from a power cut mid-write would
        # be served forever afterwards, and this robot loses power for a living.
        tmp = path.with_suffix(".part")
        tmp.write_bytes(data)
        tmp.replace(path)
    except OSError as e:
        logger.debug("tts cache write failed for %s: %s", path.name, e)
    return data


# --------------------------------------------------------------------------- #
# Speaking
# --------------------------------------------------------------------------- #

def say(
    text: str,
    *,
    event: str = "voice.say",
    guard: bool = True,
    fallback_text: Optional[str] = None,
    **fields,
) -> bool:
    """Speak `text` and block until finished. Always logged; never raises.

    `fallback_text` should be a phrase known to be in the cache. If `text`
    cannot be produced — no network, no key, API down — the fallback is spoken
    instead, so a novel sentence never degrades into silence at the exact
    moment the operator needs to hear something.

    `guard` adds a short settle before returning, for the room's reverb tail.
    Skip it only when nothing is about to listen.
    """
    robot_log.event(event, text=text, **fields)

    if not settings.TTS_ENABLED or _pick_backend() == "none":
        return False

    with _lock:                       # two overlapping utterances help nobody
        data = render(text)
        spoken = text
        if data is None and fallback_text and fallback_text != text:
            data = render(fallback_text, allow_network=False) or render(fallback_text)
            spoken = fallback_text
            if data is not None:
                robot_log.event("audio.error", logging.WARNING, stage="tts-fallback",
                                err="synthesis unavailable", spoke=fallback_text)
        if data is None:
            return False

        ok = _play_wav(data)

    if ok and guard and settings.TTS_GUARD_S > 0:
        time.sleep(settings.TTS_GUARD_S)
    return ok and spoken is not None


def connected() -> bool:
    """Session is up. Say this BEFORE the microphone stream opens, so it cannot
    be streamed to the model as if the operator had spoken."""
    return say(STATIC_PHRASES["connected"], event="voice.connected")


def error(description: str = "") -> bool:
    """Session has died. Say this AFTER it is torn down, for the same reason in
    reverse: by then nothing is listening.

    `description` is the point of the whole rewrite — a tone could only say
    "something broke", where this says which thing.
    """
    generic = STATIC_PHRASES["error"]
    if not description:
        return say(generic, event="voice.error", guard=False)
    text = f"Voice session failed. {description.rstrip('.')}. Check the log."
    return say(text, event="voice.error", guard=False,
               fallback_text=generic, detail=description)


def prime(names: Optional[list] = None) -> dict:
    """Render the static phrases into the cache. Safe to call at every boot —
    a hit costs a stat() — and it is what makes the offline failure path work.

    Returns {name: bool} so a caller can log which ones are now safe.
    """
    out = {}
    for name in (names or list(STATIC_PHRASES)):
        phrase = STATIC_PHRASES.get(name)
        out[name] = phrase is not None and render(phrase) is not None
    return out


def diagnose(text: str = "Battery 55 percent, 3.6 volts") -> list:
    """Try every combination of model and prompt shape, report what returns
    audio. Bypasses the cache entirely.

    Exists because the failure this is aimed at — a 200 with no audio — cannot
    be reasoned about from the outside. Whether it is the model, the missing
    terminal punctuation or the style prefix is an empirical question, and the
    answer differs by key and by day. Run it on the robot and read the table.
    """
    try:
        api_key = settings.require("GEMINI_API_KEY")
        import requests
    except Exception as e:
        return [{"ok": False, "err": f"{type(e).__name__}: {e}"}]

    style = settings.TTS_STYLE.strip().rstrip(":")
    bare = " ".join(text.split()).rstrip(".")
    shapes = {
        "bare, no period": bare,
        "bare + period": bare + ".",
        "styled, no period": f"{style}: {bare}" if style else None,
        "styled + period": f"{style}: {bare}." if style else None,
        "labelled transcript": (
            "Read the following aloud, clearly and calmly.\n"
            f"TRANSCRIPT: {bare}."),
    }
    models = [m for m in (settings.TTS_MODEL, settings.TTS_FALLBACK_MODEL) if m]

    results = []
    for model in models:
        for name, prompt in shapes.items():
            if prompt is None:
                continue
            audio, _retryable, detail = _request(model, prompt, api_key, requests)
            results.append({
                "model": model, "shape": name, "ok": bool(audio),
                "bytes": len(audio) if audio else 0, **detail,
            })
    return results


def cache_info() -> dict:
    d = _cache_dir()
    files = list(d.glob("*.wav"))
    return {
        "dir": str(d),
        "files": len(files),
        "bytes": sum(f.stat().st_size for f in files),
        "primed": {n: _cache_path(p).is_file() for n, p in STATIC_PHRASES.items()},
    }


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Audition / prime the robot's voice.")
    ap.add_argument("text", nargs="*", help="text to speak (default: the static phrases)")
    ap.add_argument("--prime", action="store_true", help="cache the static phrases, do not play")
    ap.add_argument("--info", action="store_true", help="show cache state and exit")
    ap.add_argument("--diagnose", action="store_true",
                    help="try every model/prompt shape and report what returns audio")
    args = ap.parse_args()

    robot_log.setup()

    if args.info:
        print(json.dumps(cache_info(), indent=2))
        raise SystemExit(0)

    if args.diagnose:
        rows = diagnose(" ".join(args.text) or "Battery 55 percent, 3.6 volts")
        for r in rows:
            mark = "ok  " if r.get("ok") else "FAIL"
            extra = " ".join(f"{k}={v}" for k, v in r.items()
                             if k not in ("ok", "model", "shape", "bytes"))
            print(f"  {mark} {r.get('model','?'):32} {r.get('shape','?'):22} "
                  f"{r.get('bytes',0):>7}B  {extra}")
        raise SystemExit(0 if any(r.get("ok") for r in rows) else 1)

    print(f"model   : {settings.TTS_MODEL}")
    print(f"voice   : {settings.TTS_VOICE}")
    print(f"style   : {settings.TTS_STYLE or '(none)'}")
    print(f"playback: {backend()}")

    if args.prime:
        for name, ok in prime().items():
            print(f"  {'ok  ' if ok else 'FAIL'} {name}: {STATIC_PHRASES[name]!r}")
        raise SystemExit(0)

    if args.text:
        print("spoke:", say(" ".join(args.text), guard=False))
    else:
        for name, phrase in STATIC_PHRASES.items():
            print(f"  {name}: {phrase!r}")
            say(phrase, guard=False)
            time.sleep(0.4)
