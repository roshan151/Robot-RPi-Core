"""Serial link, motion calibration, secrets, and vision service defaults.

Single source of truth for configuration. Nothing else in the tree should call
os.environ directly — if a setting matters, it gets a name here.

Secrets are READ here, never STORED here. This file is committed; the values
live in .env (gitignored) or /etc/robot.env (systemd). See .env.example.
"""

import os
from pathlib import Path

# Load .env before anything reads a value, so imports in any order behave the
# same. Optional: on the Pi the systemd unit supplies the environment instead
# (EnvironmentFile=/etc/robot.env), and python-dotenv may not be installed at
# all in a minimal deployment.
try:
    from dotenv import load_dotenv

    load_dotenv(Path(__file__).resolve().parent / ".env")
except Exception:  # pragma: no cover - absence is a valid deployment
    pass

# USB serial from Arduino (often /dev/ttyUSB0 or /dev/ttyACM0).
# On macOS this is a /dev/cu.usbserial-* name — set ROBOT_SERIAL_PORT rather
# than editing this, so the same checkout runs on the Pi and on a laptop.
SERIAL_PORT = os.environ.get("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("ROBOT_SERIAL_BAUD", "115200"))

# Heartbeat: must be comfortably faster than the firmware's link watchdog
# (LINK_TIMEOUT_MS = 1000 ms in drivetrain.ino).  At 0.25 s the firmware
# would need to miss four consecutive heartbeats before braking.
PING_INTERVAL_S = float(os.environ.get("ROBOT_PING_INTERVAL_S", "0.25"))

# ---------------------------------------------------------------------------
# Framed-protocol settings (firmware v3)
# ---------------------------------------------------------------------------
# How long to wait for a command's A/N reply before retransmitting it.
ACK_TIMEOUT_S = float(os.environ.get("ROBOT_ACK_TIMEOUT_S", "0.35"))

# Retransmissions per command (same sequence number — the firmware
# deduplicates, so retries never double-execute).
CMD_RETRIES = int(os.environ.get("ROBOT_CMD_RETRIES", "3"))

# Host-side ceiling on one encoder-counted move.  Must exceed the
# firmware's own MOVE_TIMEOUT_MS (15 s) so the firmware's D report,
# not a host timeout, is the normal failure path.
MOVE_TIMEOUT_S = float(os.environ.get("ROBOT_MOVE_TIMEOUT_S", "20.0"))

# Optional read timeout for non-blocking serial reads (seconds)
SERIAL_TIMEOUT = 0.05

# ---------------------------------------------------------------------------
# Encoder calibration — tune these on flat ground before running missions.
# ---------------------------------------------------------------------------
# How to calibrate TICKS_PER_CM:
#   1. Mark a start line on the floor.
#   2. Run: drivetrain.straight_m(1.0)
#   3. Measure actual distance traveled in cm.
#   4. New value = current TICKS_PER_CM * (100 / measured_cm)
#
# 38.5 came from 25.0 * (100 / 65) after a 1.0 m command travelled 65 cm.
# SINGLE TRIAL — the procedure asks for three runs averaged, so treat this as
# provisional until it has been repeated.
TICKS_PER_CM: float = float(os.environ.get("ROBOT_TICKS_PER_CM", "38.5"))

# How to calibrate TICKS_PER_DEGREE:
#   1. Run: drivetrain.right(360)
#   2. Count actual degrees rotated.
#   3. New value = current TICKS_PER_DEGREE * (360 / measured_degrees)
#
# 7.37 = 6.63 * (360 / 324), after a 360 deg command rotated 324 deg and a
# 90 deg command rotated 81 deg.  Both UNDERSHOT by the same ratio (0.90), in
# both directions -- proportional error, so it is scale and belongs here.  Had
# the two shortfalls been equal in DEGREES rather than in ratio it would have
# been coast, and belonged in TURN_COAST_TICKS below instead.
#
# History: 6.63 = 630 ticks / 95 deg, from the earlier OVERSHOOT at the
# previous value of 7.0.  That correction went too far, which is what the
# undershoot above measured; the current value re-derives from it.
#
# MEASURED AT 70 % SPEED.  DEFAULT_SPEED_PERCENT is now 80 -- more momentum at
# the moment of braking means more coast, so re-run right(360) and check
# whether the residue is still pure scale before trusting this number.
TICKS_PER_DEGREE: float = float(os.environ.get("ROBOT_TICKS_PER_DEGREE", "7.37"))

# Ticks of rotation the robot coasts AFTER the firmware hits its tick target
# and starts braking.  Subtracted from every turn target.
#
# This is deliberately SEPARATE from TICKS_PER_DEGREE because the two errors
# have different shapes, and folding them together makes one angle right at the
# cost of every other angle:
#
#   * TICKS_PER_DEGREE is a SCALE error -- overshoot grows with the angle.
#   * Coast is an OFFSET -- roughly the same overshoot at 90 deg and 360 deg,
#     because it is momentum at the moment of braking and the robot arrives at
#     every target with the same speed.
#
# Scaling TICKS_PER_DEGREE to cancel a 5 deg overshoot at 90 deg shrinks every
# target by 5.6%, which then UNDERSHOOTS 360 deg by about 14 deg.
#
# To separate them, measure at both angles with this set to 0:
#     overshoot(90) ~= overshoot(360)  -> pure coast, tune this constant
#     overshoot(360) ~= 4x overshoot(90) -> pure scale, tune TICKS_PER_DEGREE
#     in between -> both; fix scale on the 360 reading first, then coast
#
# 0 because the measured overshoot turned out to be SCALE, not coast: it grew
# with the commanded angle, so it is fully absorbed by TICKS_PER_DEGREE above.
# The mechanism is kept because coast is speed-dependent and real -- if turns
# start overshooting by a CONSTANT amount at some future speed, that residue
# belongs here, not in TICKS_PER_DEGREE.
#
# Speed-dependent: more speed, more momentum, more coast.  Recalibrate if
# DEFAULT_SPEED_PERCENT changes.
TURN_COAST_TICKS: float = float(os.environ.get("ROBOT_TURN_COAST_TICKS", "0"))

# Acceptable sync-error ratio between left and right encoders (0.0–1.0).
# A move producing more skew than this triggers a warning log.
# 0.10 = allow up to 10% difference between wheels.
ENCODER_SYNC_WARN_RATIO: float = float(os.environ.get("ROBOT_SYNC_WARN_RATIO", "0.10"))

# The firmware emits ENC: telemetry every 100 ms and zeroes both counters at
# the start of each encoder-counted move.  Reading the counters right after a
# move therefore returns a mid-move sample unless you first wait one telemetry
# interval — which is what this is.
#
# FALLBACK ONLY as of the D-frame change.  The firmware's own D frame carries
# the move's final counts (D,<seq>,<status>,<el>,<er>), so _move_and_verify()
# takes them from there and sleeps for nothing.  This is used only against
# firmware old enough to send a D with no counts on it.
#
# It was never a safety margin, only a measurement wait, which is why removing
# it from the normal path costs nothing: 150 ms of it sat between every move's
# D and the next move's M, on every step of every gesture, to recover numbers
# the firmware had already sent.
ENCODER_SETTLE_S: float = float(os.environ.get("ROBOT_ENCODER_SETTLE_S", "0.15"))

# ---------------------------------------------------------------------------
# Connection / handshake settings
# ---------------------------------------------------------------------------
# How long to wait after a DTR reset before draining the buffer and retrying
# the handshake. The UNO bootloader takes ~1.5 s; 2.5 s gives real margin.
# Only used in the DTR-reset fallback path — the fast-path ping skips this.
ARDUINO_DRAIN_WAIT_S: float = float(os.environ.get("ROBOT_DRAIN_WAIT_S", "2.5"))


# ---------------------------------------------------------------------------
# Motion defaults
# ---------------------------------------------------------------------------
DEFAULT_TURN_DEGREES = float(os.environ.get("ROBOT_DEFAULT_TURN_DEG", "90"))
DEFAULT_MOVE_METERS = float(os.environ.get("ROBOT_DEFAULT_MOVE_M", "1.0"))

# Default PWM duty for every encoder-counted move, as a percentage of full
# scale.  Measured on hardware: right(30) completes in < 1.2 s at 70 %, which
# is the budget the gesture channel needs (a NO gesture is three turns).
#
# Ramping is NOT done here.  The firmware already slews PWM at RAMP_STEP/RAMP_MS
# = 0.5 PWM per ms (drivetrain.ino), so 70 % (=178 PWM) is reached in ~356 ms
# from rest, and softStop() ramps down at 2x that rate before engaging the
# brake.  That is what protects the supply rail from the inrush/back-EMF dip
# that resets the board.  Adding a second ramp on the host would fight it.
DEFAULT_SPEED_PERCENT = float(os.environ.get("ROBOT_DEFAULT_SPEED_PCT", "80.0"))

# ---------------------------------------------------------------------------
# Speech (Gemini text-to-speech)
# ---------------------------------------------------------------------------
# The robot runs headless, so its voice is the only channel it has. It speaks
# at exactly two moments — session up, session dead — plus the battery report,
# and every one of them happens while no capture stream is open. See speech.py for
# why that ordering (speak first, THEN open the microphone) is the whole trick.
TTS_ENABLED = os.environ.get("ROBOT_TTS", "1") not in ("0", "false", "no")

# Gemini's own TTS, on generativelanguage.googleapis.com — the same host and
# the same GEMINI_API_KEY the Live session already uses. Cloud Text-to-Speech
# (WaveNet) would need a second key on a billing-enabled Cloud project, because
# AI Studio keys are restricted to the Generative Language API.
#
# It must be a TTS model variant. Plain `gemini-2.5-flash` is text-out only and
# rejects responseModalities=["AUDIO"] — the audio suffix is not cosmetic.
TTS_MODEL = os.environ.get("ROBOT_TTS_MODEL", "gemini-2.5-flash-preview-tts")

# Tried when the primary returns no audio. The preview TTS models fail in ways
# that belong to the model rather than the request — a bad minute produces
# finishReason OTHER or a 500 on every attempt — so switching model is the only
# retry that changes anything. Set to "" to disable.
TTS_FALLBACK_MODEL = os.environ.get(
    "ROBOT_TTS_FALLBACK_MODEL", "gemini-3.1-flash-tts-preview")

# One of the 30 prebuilt voices. Kore (firm) and Charon (informative) both read
# terse status lines well; Iapetus (clear) is the pick if the room is noisy.
TTS_VOICE = os.environ.get("ROBOT_TTS_VOICE", "Kore")

# Gemini TTS has no rate or pitch dials — delivery is directed in natural
# language instead. The prefix is applied as "<style>: <text>", which is the
# shape Google's own single-speaker example uses; a bare transcript with no
# directive can trip the speech classifier.
#
# Keep it short and behavioural. Long director's notes are the documented cause
# of the model reading the instructions out loud instead of following them.
TTS_STYLE = os.environ.get("ROBOT_TTS_STYLE", "Say clearly and calmly")

# The model returns raw 24 kHz 16-bit mono PCM, which speech.py wraps in a WAV
# header. Changing this does not resample anything — it only changes the header
# we write, so a wrong value plays back at the wrong pitch.
TTS_SAMPLE_RATE = int(os.environ.get("ROBOT_TTS_SAMPLE_RATE", "24000"))
TTS_DEVICE = os.environ.get("ROBOT_TTS_DEVICE", "")

# Google documents that TTS models occasionally return text tokens instead of
# audio and fail the request with a 500, at random, in a small share of calls,
# and says to retry. Cheap here: the only live calls are the battery line at
# boot and a novel error description.
TTS_RETRIES = int(os.environ.get("ROBOT_TTS_RETRIES", "2"))

# Synthesis is a network call, and the two moments the robot speaks are the two
# moments the network is least trustworthy: boot, and just after the session
# died. Everything synthesized is cached here and a cache hit never touches the
# network, which is what lets the robot announce its own failure offline.
TTS_CACHE_DIR = os.environ.get(
    "ROBOT_TTS_CACHE_DIR", str(Path.home() / ".cache" / "robot-tts"))
# Short on purpose. This runs before the microphone opens; a slow API must
# delay boot by a second or two, not by half a minute.
TTS_TIMEOUT_S = float(os.environ.get("ROBOT_TTS_TIMEOUT_S", "8.0"))
# Settle time after speaking before capture opens, covering the room's reverb
# tail. Raise it if the first syllable of a command goes missing.
TTS_GUARD_S = float(os.environ.get("ROBOT_TTS_GUARD_S", "0.15"))

# ---------------------------------------------------------------------------
# Battery (PiSugar)
# ---------------------------------------------------------------------------
# Spoken once at startup, before the microphone opens. On a headless robot a
# flat battery is otherwise invisible until the Pi browns out mid-drive — which
# on this board also resets the Arduino.
BATTERY_ANNOUNCE = os.environ.get("ROBOT_BATTERY_ANNOUNCE", "1") not in ("0", "false", "no")
BATTERY_LOW_PCT = float(os.environ.get("ROBOT_BATTERY_LOW_PCT", "20"))
PISUGAR_SOCKETS = tuple(
    s.strip() for s in os.environ.get(
        "PISUGAR_SOCKETS", "/tmp/pisugar-server.sock,/tmp/pisugar.sock"
    ).split(",") if s.strip()
)
PISUGAR_TCP = (
    os.environ.get("PISUGAR_HOST", "127.0.0.1"),
    int(os.environ.get("PISUGAR_PORT", "8423")),
)

# How long one listen() call waits for speech to START before looping.
# This is NOT a prompt interval: a timeout re-arms the microphone silently,
# with no announcement and no output. It exists only so a wedged capture device
# can be distinguished from an idle one — an infinite block would hang forever
# with nothing in the log. Raise it to make the robot more patient; it never
# changes what the operator hears.


# ---------------------------------------------------------------------------
# Live agent
# ---------------------------------------------------------------------------
# The session streams audio continuously and the model calls the robot's
# functions directly. There is no push-to-listen, no per-utterance upload and
# no ambient calibration — those existed to decide when to spend a request,
# and a streaming session has no discrete requests to spend.

# AUDIO is the only value the native-audio Live models accept — they are
# speech-to-speech and reject TEXT with "1007 ... response modalities (TEXT)
# is not supported by the model".
#
# The robot is silent anyway. This controls what the model GENERATES, not what
# gets played: live_agent reads the returned PCM off the socket and drops it,
# so no speaker emits it and the open microphone never hears it. The model's
# words still reach logs.json via output_audio_transcription.
GEMINI_LIVE_MODEL = os.environ.get(
    "GEMINI_LIVE_MODEL", "gemini-3.1-flash-live-preview")

LIVE_RESPONSE_MODALITY = os.environ.get("ROBOT_LIVE_MODALITY", "AUDIO").upper()

# Play the model's speech instead of discarding it. Off by design: the session
# holds the microphone open continuously, so anything played is streamed
# straight back to the model as if the operator had said it.
LIVE_PLAY_AUDIO = os.environ.get("ROBOT_LIVE_PLAY_AUDIO", "0") in ("1", "true", "yes")

# Capture device for the uplink; blank means the system default.
AUDIO_INPUT_DEVICE = os.environ.get("ROBOT_AUDIO_INPUT_DEVICE", "")

# Diagnostics for the failure that kills a Live session: the event loop stops
# reading the websocket, back-pressure stalls the microphone uplink, audio is
# dropped, and the server closes the connection. Both thresholds are generous —
# they should never fire in normal operation, and when they do they name the
# cause instead of leaving "audio.error" to be guessed at.
LIVE_STALL_WARN_S = float(os.environ.get("ROBOT_LIVE_STALL_WARN_S", "0.75"))
LIVE_SEND_WARN_S = float(os.environ.get("ROBOT_LIVE_SEND_WARN_S", "0.25"))

# Reconnect backoff after a dropped session, doubling to the cap.
LIVE_RECONNECT_BACKOFF_S = float(os.environ.get("ROBOT_LIVE_BACKOFF_S", "2.0"))
LIVE_RECONNECT_MAX_S = float(os.environ.get("ROBOT_LIVE_BACKOFF_MAX_S", "60.0"))

# Voice-triggered face tasks (robot_core/face_tasks.py).
FACE_ENROLL_SECONDS = float(os.environ.get("ROBOT_FACE_ENROLL_S", "30"))   # capture time once a face is found
FACE_SEARCH_SECONDS = float(os.environ.get("ROBOT_FACE_SEARCH_S", "30"))   # enroll: time allowed to find one
FACE_MATCH_SECONDS = float(os.environ.get("ROBOT_FACE_MATCH_S", "30"))     # search + match
# The head looks for a face by sweeping these (degrees); never below level.
FACE_SEARCH_PANS = (-45.0, -15.0, 15.0, 45.0)
FACE_SEARCH_TILTS = (0.0, 20.0, 40.0)
FACE_MIN_QUALITY = 0.6           # the vision service's own "worth matching" line
FACE_MIN_SHOTS = 5               # fewer good frames than this and the enroll fails

# Hard ceiling on a single drive call, enforced in the tool layer rather than
# the prompt: a limit the model can talk itself out of is not a limit.
MAX_DRIVE_METERS = float(os.environ.get("ROBOT_MAX_DRIVE_M", "5.0"))

LIVE_SYSTEM_PROMPT = os.environ.get("ROBOT_LIVE_PROMPT", """\
You are Robin, a small wheeled robot. You hear the operator continuously and
act by calling your functions. Do not narrate; call the function.

You have no voice and no screen. Your only reply is movement:
  answer("yes")      nods
  answer("no")       shakes
  answer("dance")    dances and gets back to position
  answer("unclear")  the same shake as "no"
                                    
If you want to do a happy movement just do a360 degree spin.

Long jobs — run_task() closes your session, so you cannot hear "stop" until the
job is over. When it ends you answer by gesture: yes = it worked or the face is
known, no = it failed or the face is unknown.
  run_task("enroll_face", name)  "remember my face, I'm Sam"
  run_task("match_face")         "do you know me?" / "who am I?"
  run_task("explore")            "go explore" / "map the house and find the plants"

Rules that matter:
  - Call stop() the instant you hear "stop", and whenever you are unsure
    whether it is safe to keep moving. A needless stop costs nothing.
  - Never guess a movement you are unsure of. The robot drives on a floor with
    obstacles it cannot see. If you did not understand, answer("unclear").
  - Ignore speech that is not addressed to you, and background conversation.
  - Defaults when no number is given: 1 metre, 90 degrees.
  - turn() takes positive degrees for RIGHT, negative for LEFT.
    drive() takes positive metres for FORWARD, negative for BACKWARD.
""")

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
# The robot has no verbal feedback, so this file is the only place a fault is
# ever explained. JSON Lines — one object per line, appendable, and a process
# killed mid-write costs one record instead of the whole file.
LOG_PATH = os.environ.get("ROBOT_LOG_PATH", "logs.json")
LOG_MAX_BYTES = int(os.environ.get("ROBOT_LOG_MAX_BYTES", "2000000"))
LOG_BACKUPS = int(os.environ.get("ROBOT_LOG_BACKUPS", "3"))

# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------
# ArduinoBridge.move() holds _cmd_lock for the whole blocking move, so a normal
# stop() issued from another thread cannot interrupt it — it waits for the lock
# and arrives after the move has already finished.  emergency_stop() bypasses
# _cmd_lock and writes an out-of-band S frame directly.
#
# The firmware deduplicates on the single previous sequence number, so the
# e-stop's seq must differ from the in-flight move's.  Reserve the top of the
# range for e-stops and cap the normal counter below it.
ESTOP_SEQ_MIN = 240
ESTOP_SEQ_MAX = 255
NORMAL_SEQ_MAX = ESTOP_SEQ_MIN - 1     # normal commands use 0..239

# ---------------------------------------------------------------------------
# Vision service
# ---------------------------------------------------------------------------
# The off-Pi perception service, normally the Mac mini on the LAN. Source and
# the full endpoint contract live in vision_service/; the design rationale is in
# docs/VISION-DESIGN.md Part 3.
#
# This is a BASE URL, not a single endpoint. The gateway client appends the
# versioned paths and is the only thing that knows them:
#
#   POST {base}/v1/detect          object detection, ~2 Hz, the hot path
#   POST {base}/v1/faces/embed     once per person track, not per frame
#   POST {base}/v1/faces/match     identity lookup (no GPU, sub-millisecond)
#   POST {base}/v1/faces/enroll    admin/offline
#   POST {base}/v1/depth           FALLBACK ONLY — prefer lidar range at bearing
#   GET  {base}/healthz            liveness, backend kind, queue depth
#
# Start it with: cd vision_service && ./run.sh
VISION_SERVICE_BASE_URL = os.environ.get(
    "VISION_SERVICE_BASE_URL",
    os.environ.get("VISION_SERVICE_URL", "http://127.0.0.1:8080"),
).rstrip("/")

# Send binary multipart, never base64 (33% inflation for nothing), and size the
# payload to the detector's input rather than to the camera's sensor. See
# PLAN.md standing rule #3.
VISION_DETECT_LONG_EDGE_PX = int(os.environ.get("VISION_DETECT_LONG_EDGE_PX", "640"))

# Backpressure, the Pi half of PLAN.md standing rule #8: one request in flight,
# hard timeout, drop frames rather than queue them, discard stale results.
VISION_DETECT_HZ = float(os.environ.get("VISION_DETECT_HZ", "2.0"))
VISION_DETECT_TIMEOUT_S = float(os.environ.get("VISION_DETECT_TIMEOUT_S", "0.6"))
VISION_MAX_RESULT_AGE_S = float(os.environ.get("VISION_MAX_RESULT_AGE_S", "1.0"))

# --- RETIRED ---------------------------------------------------------------
# VISION_HALT_OBJECTS and VISION_GUARD_HZ configured a camera-based obstacle
# guardian that could stop the robot. That is now a contradiction rather than a
# feature: decision #7 gives horizontal obstacles to the lidar, and REST
# inference is advisory by construction — 100-500 ms of round trip, and nothing
# at all when the wifi drops. HARDWARE-BASICS.md Part 7 puts it plainly: the
# lidar stops the robot, the camera tells the agent what is there.
#
# Nothing reads these. They are kept named here only so an old .env does not
# look like it is still doing something.
VISION_HALT_OBJECTS: list[str] = []   # retired — the camera never stops the robot
VISION_GUARD_HZ = 0.0                 # retired — see above

# ---------------------------------------------------------------------------
# Secrets
# ---------------------------------------------------------------------------
# Values come from the environment or .env — never from this file, which is
# committed. Missing secrets resolve to "" rather than raising at import time,
# so tests and offline tools can import config without credentials present.
# Call require() at the point of use to fail with a message that names the
# variable and where to put it.
#
# Source order (first non-empty wins), per secret:
#   1. process environment  (systemd EnvironmentFile=/etc/robot.env)
#   2. .env next to this file
#   3. ""  -> require() raises with instructions

def _first_env(*names: str) -> str:
    """First non-empty value among `names`, or "".

    Accepting several names is what lets GOOGLE_API_KEY stand in for
    GEMINI_API_KEY without a second entry in .env, and it is why the value is
    resolved here rather than at each call site.
    """
    for n in names:
        v = os.environ.get(n)
        if v:
            return v.strip()
    return ""


# Google Gemini — the only voice backend.
#
# Read from the environment, never written here. This was briefly hardcoded to
# "" which meant require() could not succeed no matter what was in /etc/robot.env
# — the robot could not start, and the error named the variable that was in fact
# set correctly.
GEMINI_API_KEY = _first_env("GEMINI_API_KEY", "GOOGLE_API_KEY")
# The robot's voice uses this same key: Gemini TTS lives on
# generativelanguage.googleapis.com, so there is no second credential and no
# Cloud project to enable. Note that rate limits are per PROJECT — the Live
# session and the spoken announcements draw on the same quota.

# Bluetooth headset (see check_bt_audio.sh).
BT_MAC = os.environ.get("BT_MAC", "")

# Every name here is treated as sensitive by the log redactor.
SECRET_NAMES = (
    "GEMINI_API_KEY",
)

# ---------------------------------------------------------------------------
# Voice backend
# ---------------------------------------------------------------------------
# Gemini only, deliberately. It takes the microphone audio and the planning
# prompt in ONE request and returns structured JSON, replacing the old
# two-hop  mic -> Google Web Speech -> text -> OpenAI  pipeline.
#
# The toggle exists so the value is named and validated rather than implied,
# and so a future backend has an obvious place to land. Anything other than
# "gemini" is rejected at import — a silently ignored setting is worse than
# no setting.


class MissingSecret(RuntimeError):
    """A required credential is not configured."""


def require(name: str) -> str:
    """Return a secret, or raise with instructions naming the variable.

    Preferred over reading the constant directly, so a missing key produces
    one clear line in logs.json instead of a 401 from a vendor SDK three
    frames deep.
    """
    value = globals().get(name, "")
    if not value:
        raise MissingSecret(
            f"{name} is not set. Provide it in one of:\n"
            f"  - {Path(__file__).resolve().parent / '.env'}  (development)\n"
            f"  - /etc/robot.env                              (systemd service)\n"
            f"  - the process environment\n"
            f"See .env.example for the full list."
        )
    return str(value)


def secret_values() -> tuple:
    """Non-empty secret values, for redaction. Never log the result."""
    return tuple(v for v in (globals().get(n, "") for n in SECRET_NAMES) if v)
