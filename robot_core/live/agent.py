"""
Gemini Live agent: continuous audio in, tool calls out.

This replaces the turn-based loop entirely. There is no push-to-listen, no
per-utterance upload, no ambient calibration, no local speech gate and no
request budget — all of that existed to decide *when* to spend a request, and
a streaming session has no discrete requests to spend.

What the model gets is a live microphone and the robot's actual functions. It
decides when the operator has finished speaking (server-side VAD) and calls
`drive`, `turn`, `stop` or `answer` directly.

Two tasks run concurrently:

    _pump_audio   microphone -> session, forever
    _pump_events  session -> tool dispatch, forever

They are cancelled together. If either dies the session is torn down, the
failure is spoken aloud (naming the cause), and the supervisor reconnects — in
that order, because the announcement must not play into a live microphone.

Audio format
------------
16 kHz signed 16-bit mono PCM upstream, which is what the Live API expects and
what the Pi's capture path already produces. Gemini's server-side VAD determines
speech activity; this client continuously streams the PCM and does not implement
its own speech gate.
"""

from __future__ import annotations

import asyncio
import logging
import re
import time
from typing import Any, Dict, Optional

from robot_core import settings
from robot_core import robot_log
from robot_core import speech
from robot_core.live.tools import RobotTools, declarations

logger = logging.getLogger(__name__)

SEND_SAMPLE_RATE = 16000     # uplink: what the Live API expects
RECV_SAMPLE_RATE = 24000     # downlink: what it returns (only used if played)
CHUNK_FRAMES = 640           # 40 ms at 16 kHz


class LiveAgentError(RuntimeError):
    """The Live session could not be established or has failed."""


class LiveConfigError(LiveAgentError):
    """The session was rejected for how it was configured.

    Kept separate from ordinary failures because retrying is pointless and
    actively harmful: the same config will be refused every time, and each
    attempt brakes the motors and writes another identical error. One bad
    setting should produce one clear line, not a log full of them.
    """


def _is_config_rejection(exc: BaseException) -> bool:
    text = f"{type(exc).__name__} {exc}".lower()
    return (
        "1007" in text                       # websocket policy violation
        or "not supported by the model" in text
        or "invalid_argument" in text
        or "response modalities" in text
    )


class LiveAgent:
    """One connected session. Construct, `run()`, and it returns when done."""

    def __init__(self, tools: RobotTools, model: Optional[str] = None) -> None:
        self.model = model or settings.GEMINI_LIVE_MODEL
        self._tools = tools
        self._audio_q: "asyncio.Queue[bytes]" = asyncio.Queue(maxsize=64)
        self._stream = None
        self._dropped = 0
        self._audio_out_bytes = 0
        self._out = None

    # ------------------------------------------------------------------ #

    def _config(self) -> Dict[str, Any]:
        from google.genai import types  # type: ignore

        # AUDIO, because the native-audio Live models are speech-to-speech and
        # reject TEXT outright ("1007 ... response modalities (TEXT) is not
        # supported by the model").
        #
        # The robot is still silent. response_modalities controls what the
        # model GENERATES, not what we render — and nothing here plays the
        # returned PCM. It is read off the socket and dropped, so no speaker
        # ever emits it and the microphone never hears it.
        #
        # The transcriptions are what make that free: rather than losing the
        # model's words, we get them as text for logs.json, plus a transcript
        # of what it heard the operator say. On a robot with no screen, that
        # pair is the difference between "it ignored me" and "it misheard me".
        cfg: Dict[str, Any] = {
            "response_modalities": [settings.LIVE_RESPONSE_MODALITY],
            "system_instruction": settings.LIVE_SYSTEM_PROMPT,
            "tools": [{"function_declarations": declarations()}],

            # Let Gemini perform server-side voice activity detection (VAD).
            #
            # We continuously stream microphone PCM with send_realtime_input().
            # Gemini decides when speech starts/ends; we do NOT send
            # activity_start/activity_end ourselves.
            #
            # TURN_INCLUDES_ONLY_ACTIVITY means the user's turn contains
            # detected activity rather than accumulating all the silence between
            # turns. This is the desired behavior for a continuously-open mic.
            "realtime_input_config": types.RealtimeInputConfig(
                automatic_activity_detection=types.AutomaticActivityDetection(
                    disabled=False,
                ),
                turn_coverage="TURN_INCLUDES_ONLY_ACTIVITY",
            ),
        }
        try:
            cfg["input_audio_transcription"] = types.AudioTranscriptionConfig()
            cfg["output_audio_transcription"] = types.AudioTranscriptionConfig()
        except AttributeError:
            # Older SDK without the config type — the agent still works, the
            # log is just quieter about what was said.
            logger.debug("SDK has no AudioTranscriptionConfig; skipping")
        return cfg

    # ------------------------------------------------------------------ #

    def _open_microphone(self) -> None:
        import sounddevice as sd  # type: ignore

        loop = asyncio.get_running_loop()

        def callback(indata, frames, time_info, status):  # runs on PortAudio's thread
            if status:
                logger.debug("audio input status: %s", status)
            # call_soon_threadsafe because this fires on PortAudio's own
            # thread, not the event loop.
            try:
                loop.call_soon_threadsafe(self._offer, bytes(indata))
            except RuntimeError:
                pass                     # loop closing

        self._stream = sd.RawInputStream(
            samplerate=SEND_SAMPLE_RATE,
            blocksize=CHUNK_FRAMES,
            device=settings.AUDIO_INPUT_DEVICE or None,
            dtype="int16",
            channels=1,
            callback=callback,
        )
        self._stream.start()

    def _offer(self, chunk: bytes) -> None:
        """Enqueue a chunk, dropping the oldest if we have fallen behind.

        Dropping beats blocking: if the uplink stalls, the newest audio is what
        matters and an unbounded queue would just grow until the robot was
        acting on minute-old speech.
        """
        try:
            self._audio_q.put_nowait(chunk)
        except asyncio.QueueFull:
            self._dropped += 1
            try:
                self._audio_q.get_nowait()
                self._audio_q.put_nowait(chunk)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass
            robot_log.event_throttled(
                "audio.error", key="uplink-lag", window_s=30.0,
                level=logging.WARNING, stage="mic-queue",
                err="uplink behind, dropping audio", dropped=self._dropped,
            )

    def _close_microphone(self) -> None:
        if self._out is not None:
            try:
                self._out.stop()
                self._out.close()
            except Exception:
                pass
            self._out = None
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    # ------------------------------------------------------------------ #

    async def _pump_audio(self, session) -> None:
        from google.genai import types  # type: ignore

        while True:
            chunk = await self._audio_q.get()
            blob = types.Blob(data=chunk,
                              mime_type=f"audio/pcm;rate={SEND_SAMPLE_RATE}")
            t0 = time.monotonic()
            # send_realtime_input is the current name; older SDKs used
            # send(input=..., end_of_turn=False). Try the modern one first.
            if hasattr(session, "send_realtime_input"):
                await session.send_realtime_input(audio=blob)
            else:
                await session.send(input=blob)

            # A single chunk is ~64 ms of audio. If sending one takes longer
            # than that we are falling behind in real time, and the mic queue
            # is about to start dropping. Says WHY the uplink stalled, rather
            # than only that it did.
            sent_s = time.monotonic() - t0
            if sent_s > settings.LIVE_SEND_WARN_S:
                robot_log.event_throttled(
                    "audio.error", key="send-slow", window_s=30.0,
                    level=logging.WARNING, stage="uplink-send",
                    err="websocket send is slower than real time",
                    send_s=round(sent_s, 2), qdepth=self._audio_q.qsize(),
                )

    async def _pump_events(self, session) -> None:
        """Read the socket forever, across an unbounded number of turns.

        `session.receive()` is a ONE-TURN generator: google-genai breaks out of
        it the moment a message arrives with `server_content.turn_complete`.
        A bare `async for response in session.receive()` therefore does not mean
        "read until the session ends", it means "read until the model finishes
        answering once" — and then it returns, normally, with no exception and
        nothing in the log to say so.

        Nobody is reading the socket after that. The websocket's inbound queue
        fills, the library's reader task stops draining the TCP buffer, and the
        pongs that answer the server's keepalive pings are never processed. One
        ping_interval plus one ping_timeout later (20 s + 20 s in `websockets`,
        which is why the gap was always ~20-40 s and never anything else) the
        server gives up:

            1011 (internal error) keepalive ping timeout

        So: restart the generator each turn. The outer loop is the session; the
        inner loop is one turn.
        """
        last = time.monotonic()
        while True:
            turn_msgs = 0
            async for response in session.receive():
                turn_msgs += 1
                # How long this loop went without reading the socket. If it
                # grows, the receive side has stopped draining and back-pressure
                # will stall the microphone uplink next, so measure it directly
                # rather than inferring it from the wreckage.
                gap = time.monotonic() - last
                if gap > settings.LIVE_STALL_WARN_S:
                    robot_log.event_throttled(
                        "audio.error", key="recv-stall", window_s=30.0,
                        level=logging.WARNING, stage="receive-loop",
                        err="event loop did not read the socket",
                        stalled_s=round(gap, 2),
                    )
                last = time.monotonic()

                calls = self._extract_tool_calls(response)
                if calls:
                    await self._handle_tool_calls(session, calls)
                    last = time.monotonic()
                    continue

                # Audio comes back because the model is speech-to-speech, but
                # the robot is silent by design: read it off the socket and drop
                # it. Nothing plays it, so nothing re-enters the microphone.
                self._drain_audio(response)
                self._log_transcripts(response)

            # A turn that yielded nothing means the socket is gone rather than
            # that the turn was short — `_receive()` returned falsy instead of
            # raising. Without this the outer loop becomes a hot spin that
            # starves the mic uplink and never reconnects.
            if turn_msgs == 0:
                raise LiveAgentError(
                    "live session closed by the server (receive stream ended)")
            last = time.monotonic()

    async def _pump_task(self) -> None:
        """Returns the moment run_task queues a job, which ends the session cleanly
        (microphone closed, socket closed) so `_run_supervised` can run it."""
        while self._tools.pending is None:
            await asyncio.sleep(0.1)

    async def _pump_results(self, session, results) -> None:
        """Tell the model what actually happened to the moves it queued.

        Every tool answers "queued" and returns — that is what keeps the
        microphone alive. The cost is that the model has no idea how a move
        ended: it asks for 3 m, the guardian halts it at 0.4 m, and as far as
        the model knows the robot is still driving.

        Polling does not fix this. A `get_status` tool only helps if the model
        remembers to call it, and it will not. So push instead: each finished
        move becomes one short line of context, injected without ending the
        turn, and the model simply knows.

        This is the difference between an open-loop and a closed-loop agent,
        and it is the whole reason the backend carries a results queue.
        """
        while True:
            event = await results.get()
            text = (f"[robot] {event['op']} {event['value']:g}: "
                    f"{event['status']}")
            robot_log.event("voice.feedback", text=text)
            await session.send_client_content(
                turns={"role": "user", "parts": [{"text": text}]},
                turn_complete=False,
            )

    def _drain_audio(self, response) -> None:
        """Discard generated speech. Counted, so 'silent' stays a deliberate
        choice rather than something we stopped noticing."""
        data = getattr(response, "data", None)
        if not data:
            return
        self._audio_out_bytes += len(data)

        if settings.LIVE_PLAY_AUDIO:
            # Off by default, and it should stay off: the microphone is open
            # for the whole session, so anything played here is streamed
            # straight back to the model as if the operator had said it.
            self._play(data)
            return

        robot_log.event_throttled(
            "voice.say", key="discarded", window_s=300.0,
            text="(model speech discarded — robot answers by moving)",
            bytes_dropped=self._audio_out_bytes,
        )

    def _play(self, pcm: bytes) -> None:
        try:
            import numpy as np  # type: ignore
            import sounddevice as sd  # type: ignore

            if self._out is None:
                self._out = sd.OutputStream(
                    samplerate=RECV_SAMPLE_RATE, channels=1, dtype="int16")
                self._out.start()
            self._out.write(np.frombuffer(pcm, dtype="<i2"))
        except Exception as e:
            robot_log.event_throttled(
                "audio.error", key="playback", window_s=60.0,
                level=logging.WARNING, stage="live-playback",
                err=f"{type(e).__name__}: {e}")

    def _log_transcripts(self, response) -> None:
        """What the model heard, and what it would have said.

        The only window into the conversation on a robot with no screen.
        """
        server = getattr(response, "server_content", None)
        if server is None:
            return
        heard = getattr(server, "input_transcription", None)
        said = getattr(server, "output_transcription", None)

        text = getattr(heard, "text", None)
        if text and text.strip():
            robot_log.event("voice.heard", text=text.strip()[:400])

        text = getattr(said, "text", None)
        if text and text.strip():
            robot_log.event("voice.say", text=text.strip()[:400], spoken=False)

    @staticmethod
    def _extract_tool_calls(response) -> list:
        """Tool calls have lived in two places across SDK versions.

        Checking both costs nothing and avoids a silent no-op where the robot
        hears commands, decides what to do, and never does it.
        """
        tc = getattr(response, "tool_call", None)
        if tc is None:
            server = getattr(response, "server_content", None)
            tc = getattr(server, "tool_call", None) if server else None
        return list(getattr(tc, "function_calls", []) or []) if tc else []

    async def _handle_tool_calls(self, session, calls) -> None:
        from google.genai import types  # type: ignore

        responses = []
        for call in calls:
            name = getattr(call, "name", "")
            args = dict(getattr(call, "args", {}) or {})
            t0 = time.monotonic()
            result = await self._tools.dispatch(name, args)
            logger.info("tool %s(%s) -> %s in %.2fs",
                        name, args, result, time.monotonic() - t0)
            responses.append(types.FunctionResponse(
                id=getattr(call, "id", None), name=name,
                response={"output": result},
            ))

        if hasattr(session, "send_tool_response"):
            await session.send_tool_response(function_responses=responses)
        else:
            await session.send(input={"tool_response": {
                "function_responses": [
                    {"id": r.id, "name": r.name, "response": r.response}
                    for r in responses
                ]
            }})

    # ------------------------------------------------------------------ #

    async def run(self) -> None:
        """Connect and serve until the session ends or a task fails."""
        try:
            from google import genai  # type: ignore
        except ImportError as e:
            raise LiveAgentError(
                "google-genai is not installed. pip install -U google-genai"
            ) from e

        client = genai.Client(api_key=settings.require("GEMINI_API_KEY"))

        try:
            connection = client.aio.live.connect(
                model=self.model, config=self._config())
        except Exception as e:
            if _is_config_rejection(e):
                raise LiveConfigError(str(e)) from e
            raise

        async with connection as session:
            robot_log.event("voice.connect", backend="gemini-live",
                            model=self.model,
                            modality=settings.LIVE_RESPONSE_MODALITY)

            # Spoken BEFORE the microphone opens, so it cannot be streamed to
            # the model as if the operator had said it. Blocking, and the guard
            # after it covers the room's reverb tail.
            speech.connected()

            self._open_microphone()
            try:
                # Deliberately not asyncio.TaskGroup: the Pi runs Python 3.10
                # and TaskGroup / except* are 3.11+. asyncio.wait gives the
                # same shape — whichever pump finishes first takes the session
                # down, and the other is cancelled.
                #
                # FIRST_COMPLETED, not FIRST_EXCEPTION. Both pumps are infinite
                # by contract, so either one *returning* is as much a failure as
                # either one raising. FIRST_EXCEPTION degrades to ALL_COMPLETED
                # when nothing raises, which meant a receive loop that quietly
                # ran out of turns left the audio pump shouting into a socket
                # no one was reading, with no error anywhere until the server's
                # keepalive timer killed it 20-40 s later.
                tasks = [
                    asyncio.create_task(self._pump_audio(session),
                                        name="mic-uplink"),
                    asyncio.create_task(self._pump_events(session),
                                        name="events"),
                ]
                # Only when the backend has a feedback channel. LocalMotion
                # does not yet; RosMotion does, and gets the model told what
                # became of every move it asked for.
                tasks.append(asyncio.create_task(self._pump_task(), name="task"))
                results = getattr(self._tools.motion, "results", None)
                if results is not None:
                    tasks.append(asyncio.create_task(
                        self._pump_results(session, results), name="results"))
                done, pending = await asyncio.wait(
                    tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in pending:
                    task.cancel()
                if pending:
                    await asyncio.gather(*pending, return_exceptions=True)
                for task in done:
                    task.result()          # re-raise the real failure
            finally:
                self._close_microphone()


async def _run_supervised(tools: RobotTools) -> None:
    """Reconnect on failure, with backoff. Motors stop first, every time."""
    backoff = settings.LIVE_RECONNECT_BACKOFF_S
    while True:
        agent = LiveAgent(tools)
        try:
            await agent.run()
            if tools.pending is not None:
                await _run_task(tools)       # the session is closed; reconnect after it
                backoff = settings.LIVE_RECONNECT_BACKOFF_S
                continue
            robot_log.event("session.stop", reason="live session closed")
            return
        except asyncio.CancelledError:
            raise
        except Exception as e:
            _on_failure(tools, agent, e)
            if isinstance(e, LiveConfigError) or _is_config_rejection(e):
                # Retrying a rejected configuration just produces the same
                # error forever, braking the motors on every attempt. Say what
                # is wrong once, in terms that name the fix, and stop.
                robot_log.event(
                    "fatal", logging.CRITICAL,
                    cause="live session configuration rejected",
                    model=agent.model,
                    modality=settings.LIVE_RESPONSE_MODALITY,
                    err=str(e)[:300],
                    fix=("native-audio Live models are speech-to-speech and "
                         "only accept AUDIO. Set ROBOT_LIVE_MODALITY=AUDIO — "
                         "the robot stays silent because the returned audio is "
                         "discarded, not played."),
                )
                raise

        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, settings.LIVE_RECONNECT_MAX_S)


async def _run_task(tools: RobotTools) -> None:
    """Run the job run_task queued, then answer by gesture: yes / no.

    Called with the Live session already closed. The job blocks (a minute of
    camera frames, or a whole exploration), so it goes to a worker thread.
    """
    task, name = tools.pending
    tools.pending = None
    tools.motion.stop()                      # anything the model had queued yields to the job
    robot_log.event("task.start", task=task, name=name)
    try:
        ok = bool(await asyncio.get_running_loop().run_in_executor(
            None, tools.task_runner, task, name))
    except Exception as e:                   # noqa: BLE001
        ok = False
        robot_log.event("task.error", logging.ERROR, task=task, err=f"{type(e).__name__}: {e}")
    robot_log.event("task.end", task=task, ok=ok)

    await tools.dispatch("answer", {"value": "yes" if ok else "no"})
    # Let the gesture finish before the microphone reopens, and drop its move
    # results so the next session is not told about them.
    for _ in range(100):
        if not tools.motion.busy():
            break
        await asyncio.sleep(0.2)
    results = getattr(tools.motion, "results", None)
    while results is not None and not results.empty():
        results.get_nowait()


def _spoken_cause(exc: BaseException) -> str:
    """A short phrase naming what broke, for the failure announcement.

    Derived from the exception's class name, split at the capitals, so
    `ConnectionClosedError` is read as "connection closed error" rather than
    as one unpronounceable word. A handful of causes the operator can actually
    act on get a sentence written for them instead.
    """
    name = type(exc).__name__
    friendly = {
        "LiveConfigError": "session configuration rejected",
        "MissingSecret": "the API key is not configured",
        "ConnectionRefusedError": "connection refused",
        "TimeoutError": "the session timed out",
        "OSError": "an audio device error",
    }
    if name in friendly:
        return friendly[name]
    words = re.findall(r"[A-Z]+(?![a-z])|[A-Z][a-z]*|[a-z]+", name)
    return " ".join(w.lower() for w in words) if words else "an unknown error"


def _on_failure(tools: RobotTools, agent: LiveAgent, exc: BaseException) -> None:
    """Order matters: brake, tear down, THEN speak."""
    try:
        tools.motion.stop()
    except Exception:
        pass
    robot_log.event("voice.drop", logging.ERROR,
                    err=f"{type(exc).__name__}: {exc}")
    # Safe to speak now: the session is gone, so nothing is listening.
    #
    # The exception type is the description rather than str(exc): the message
    # can be a wrapped multi-line server payload, and a robot reading a JSON
    # blob aloud tells the operator less than "connection closed error" does.
    # The full text is already in logs.json on the line above.
    speech.error(_spoken_cause(exc))


def run_live_agent(motion, task_runner=None) -> None:
    """Blocking entrypoint. `motion` is a MotionBackend (robot_core/motion.py).

    Does not close the backend — whoever built it owns it. In the ROS build
    that is the voice node, which needs its action clients to outlive one
    session so the supervisor can reconnect without rebuilding them.

    `task_runner(task, name) -> bool` runs the jobs `run_task` asks for
    (enroll_face, match_face, explore); without one the tool refuses.
    """
    tools = RobotTools(motion, task_runner)
    try:
        asyncio.run(_run_supervised(tools))
    except KeyboardInterrupt:
        robot_log.event("session.stop", reason="keyboard interrupt")
    finally:
        try:
            motion.stop()
        except Exception:
            pass
