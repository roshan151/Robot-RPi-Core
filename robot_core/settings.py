"""All configuration. Every value reads an environment variable and falls back to the default here.

Variables come from the environment, else from .env / /etc/robot.env (see _load_env_files).
Secrets live there, never in this file.
"""

import os
from pathlib import Path


def _load_env_files():
    """Fill in variables from env files. Already-set variables always win, then the
    first file listed that sets a name: ./.env, robot_core/.env, /etc/robot.env."""
    here = Path(__file__).resolve().parent
    for path in (os.environ.get("ROBOT_ENV_FILE"), here.parent / ".env", here / ".env", "/etc/robot.env"):
        try:
            lines = Path(path).read_text().splitlines() if path else []
        except OSError:               # missing, or /etc/robot.env not readable by this user
            continue
        for line in lines:
            line = line.strip().removeprefix("export ").strip()
            if line and not line.startswith("#") and "=" in line:
                key, value = line.split("=", 1)
                os.environ.setdefault(key.strip(), value.strip().strip("\"'"))


_load_env_files()


def _env(name, default, cast=str):
    return cast(os.environ.get(name, default))


def _flag(name, default="1"):
    return os.environ.get(name, default) not in ("0", "false", "no")


# --- Arduino serial link ------------------------------------------------------
SERIAL_PORT = _env("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")        # macOS: /dev/cu.usbserial-*
BAUD_RATE = _env("ROBOT_SERIAL_BAUD", "115200", int)
SERIAL_TIMEOUT = 0.05
PING_INTERVAL_S = _env("ROBOT_PING_INTERVAL_S", "0.25", float)  # firmware brakes after 1 s of silence
ACK_TIMEOUT_S = _env("ROBOT_ACK_TIMEOUT_S", "0.35", float)
CMD_RETRIES = _env("ROBOT_CMD_RETRIES", "3", int)
MOVE_TIMEOUT_S = _env("ROBOT_MOVE_TIMEOUT_S", "20.0", float)    # must exceed the firmware's 15 s
ARDUINO_DRAIN_WAIT_S = _env("ROBOT_DRAIN_WAIT_S", "2.5", float)

# Emergency-stop frames use sequence numbers 240-255; normal commands use 0-239.
ESTOP_SEQ_MIN = 240
ESTOP_SEQ_MAX = 255
NORMAL_SEQ_MAX = ESTOP_SEQ_MIN - 1

# --- Motion calibration (measure on flat ground) --------------------------------
# TICKS_PER_CM:     new = old * 100 / measured_cm after a 1 m drive.
# TICKS_PER_DEGREE: new = old * commanded_deg / measured_deg after a 360 deg turn.
#                   Overshooting -> lower it. Undershooting -> raise it.
# TURN_COAST_TICKS: only for overshoot that is the SAME at 90 and 360 deg (momentum).
TICKS_PER_CM = _env("ROBOT_TICKS_PER_CM", "38.5", float)
TICKS_PER_DEGREE = _env("ROBOT_TICKS_PER_DEGREE", "6", float)   # measured at 70 % speed
TURN_COAST_TICKS = _env("ROBOT_TURN_COAST_TICKS", "0", float)
DEFAULT_SPEED_PERCENT = _env("ROBOT_DEFAULT_SPEED_PCT", "80.0", float)
DEFAULT_TURN_DEGREES = _env("ROBOT_DEFAULT_TURN_DEG", "90", float)
DEFAULT_MOVE_METERS = _env("ROBOT_DEFAULT_MOVE_M", "1.0", float)
MAX_DRIVE_METERS = _env("ROBOT_MAX_DRIVE_M", "5.0", float)         # enforced in code, not the prompt
ENCODER_SYNC_WARN_RATIO = _env("ROBOT_SYNC_WARN_RATIO", "0.10", float)   # left/right skew that logs a warning
ENCODER_SETTLE_S = _env("ROBOT_ENCODER_SETTLE_S", "0.15", float)         # only for old firmware

# --- Gemini (one API key for the Live session and the speech) -----------------
SECRET_NAMES = ("GEMINI_API_KEY",)


def _first_env(*names):
    return next((os.environ[n].strip() for n in names if os.environ.get(n)), "")


GEMINI_API_KEY = _first_env("GEMINI_API_KEY", "GOOGLE_API_KEY")
GEMINI_LIVE_MODEL = _env("GEMINI_LIVE_MODEL", "gemini-3.1-flash-live-preview")
LIVE_RESPONSE_MODALITY = _env("ROBOT_LIVE_MODALITY", "AUDIO").upper()   # native-audio models reject TEXT
LIVE_PLAY_AUDIO = _env("ROBOT_LIVE_PLAY_AUDIO", "0") in ("1", "true", "yes")   # keep off: the mic would hear it
AUDIO_INPUT_DEVICE = _env("ROBOT_AUDIO_INPUT_DEVICE", "")              # blank = system default
BT_MAC = _env("BT_MAC", "")
LIVE_STALL_WARN_S = _env("ROBOT_LIVE_STALL_WARN_S", "0.75", float)
LIVE_SEND_WARN_S = _env("ROBOT_LIVE_SEND_WARN_S", "0.25", float)
LIVE_RECONNECT_BACKOFF_S = _env("ROBOT_LIVE_BACKOFF_S", "2.0", float)
LIVE_RECONNECT_MAX_S = _env("ROBOT_LIVE_BACKOFF_MAX_S", "60.0", float)
# While a task runs (main session closed) a cancel-only Live session listens for
# "stop" (robot_core/live/cancel_watch.py). 0 = off; task/stop still works.
CANCEL_LISTEN = _flag("ROBOT_CANCEL_LISTEN")
CANCEL_PROMPT = os.environ.get("ROBOT_CANCEL_PROMPT", """\
You are listening to the operator of a small robot while it carries out a task:
following someone, learning a face, or exploring a room. You cannot move the
robot and you cannot speak. Your only function is cancel_task.

Call cancel_task as soon as the operator wants the robot to stop or give up
what it is doing, for example "stop", "Robin stop", "cancel", "that's enough",
"quit following me", "come back", "wait", "hold on". When in doubt, call it:
stopping by mistake is harmless, not stopping is not.

Do not call it for "don't stop" or "keep going", or for speech that is not
addressed to the robot (a TV, people talking to each other). Otherwise do
nothing and say nothing.""")

# --- Speech (Gemini text-to-speech, cached on disk) ---------------------------
TTS_ENABLED = _flag("ROBOT_TTS")
TTS_MODEL = _env("ROBOT_TTS_MODEL", "gemini-2.5-flash-preview-tts")
TTS_FALLBACK_MODEL = _env("ROBOT_TTS_FALLBACK_MODEL", "gemini-3.1-flash-tts-preview")   # "" = none
TTS_VOICE = _env("ROBOT_TTS_VOICE", "Kore")
TTS_STYLE = _env("ROBOT_TTS_STYLE", "Say clearly and calmly")
TTS_SAMPLE_RATE = _env("ROBOT_TTS_SAMPLE_RATE", "24000", int)
TTS_DEVICE = _env("ROBOT_TTS_DEVICE", "")
TTS_RETRIES = _env("ROBOT_TTS_RETRIES", "2", int)
TTS_TIMEOUT_S = _env("ROBOT_TTS_TIMEOUT_S", "8.0", float)
TTS_GUARD_S = _env("ROBOT_TTS_GUARD_S", "0.15", float)                 # quiet time before the mic opens
TTS_CACHE_DIR = _env("ROBOT_TTS_CACHE_DIR", str(Path.home() / ".cache" / "robot-tts"))

# --- Battery (PiSugar), announced once at startup -----------------------------
BATTERY_ANNOUNCE = _flag("ROBOT_BATTERY_ANNOUNCE")
BATTERY_LOW_PCT = _env("ROBOT_BATTERY_LOW_PCT", "20", float)
PISUGAR_SOCKETS = tuple(s.strip() for s in os.environ.get(
    "PISUGAR_SOCKETS", "/tmp/pisugar-server.sock,/tmp/pisugar.sock").split(",") if s.strip())
PISUGAR_TCP = (_env("PISUGAR_HOST", "127.0.0.1"), _env("PISUGAR_PORT", "8423", int))

# --- Vision service (the Mac) ---------------------------------------------------
VISION_SERVICE_BASE_URL = os.environ.get(
    "VISION_SERVICE_BASE_URL", os.environ.get("VISION_SERVICE_URL", "http://127.0.0.1:8080")).rstrip("/")
VISION_DETECT_LONG_EDGE_PX = _env("VISION_DETECT_LONG_EDGE_PX", "640", int)
VISION_DETECT_HZ = _env("VISION_DETECT_HZ", "2.0", float)
VISION_DETECT_TIMEOUT_S = _env("VISION_DETECT_TIMEOUT_S", "0.6", float)
VISION_MAX_RESULT_AGE_S = _env("VISION_MAX_RESULT_AGE_S", "1.0", float)

# --- Voice face tasks ---------------------------------------------------------
FACE_ENROLL_SECONDS = _env("ROBOT_FACE_ENROLL_S", "30", float)   # capture time once a face is found
FACE_SEARCH_SECONDS = _env("ROBOT_FACE_SEARCH_S", "30", float)   # enroll: time allowed to find one
FACE_MATCH_SECONDS = _env("ROBOT_FACE_MATCH_S", "30", float)     # search + match
FACE_SEARCH_PANS = (-75.0, -15.0, 15.0, 75.0)                    # head sweep, degrees
FACE_SEARCH_TILTS = (0.0, 30.0, 50.0)                            # never below level
FACE_MIN_QUALITY = 0.6
FOLLOW_SECONDS = _env("ROBOT_FOLLOW_S", "300", float)            # follow mode ends after this
FOLLOW_TILT_DEG = _env("ROBOT_FOLLOW_TILT", "20", float)         # head tilt while following
FACE_MIN_SHOTS = 5
# Long edge of frames sent for faces and follow. Faces need the pixels: with the
# wide USB HD lens a 640 px frame leaves a face at 2 m ~18 px wide (useless), the
# full 1920 px ~55 px. A smaller camera frame is sent as it is (never upscaled).
FACE_WIRE_LONG_EDGE = _env("ROBOT_FACE_WIRE_PX", "1920", int)
FACE_ENROLL_TILT_DEG = _env("ROBOT_FACE_ENROLL_TILT", "20", float)  # enroll starts here, then frames the face
FACE_FRAME_DEADBAND = 0.12      # enroll re-aims when the face centre is this far (share of frame) off centre
FACE_EDGE_MARGIN = 0.02         # a shot is kept only with the whole face this far inside the frame

# --- Logging ------------------------------------------------------------------
LOG_PATH = _env("ROBOT_LOG_PATH", "logs.json")
LOG_MAX_BYTES = _env("ROBOT_LOG_MAX_BYTES", "2000000", int)
LOG_BACKUPS = _env("ROBOT_LOG_BACKUPS", "3", int)

# --- What the voice model is told ---------------------------------------------
LIVE_SYSTEM_PROMPT = os.environ.get("ROBOT_LIVE_PROMPT", """\
You are Robin, a small wheeled robot. You hear the operator continuously and
act by calling your functions. Do not narrate; call the function.

You have no voice and no screen. Your only reply is movement:
  answer("yes")      nods
  answer("no")       shakes
  answer("dance")    dances and gets back to position
  answer("unclear")  the same shake as "no"

If you want to do a happy movement just do a 360 degree spin.

Long jobs: run_task() closes your session until the job is over ("stop" during
the job is heard by a separate listener, not you). When it ends you answer by
gesture: yes = it worked, the face is known or it was stopped; no = it failed or
the face is unknown.
  run_task("enroll_face", name)  "remember my face, I'm Sam" / "register Sam". The name
                                 must be in what was said; if it is not, pass none: the
                                 robot shakes its head and does nothing.
  run_task("match_face")         "do you know me?" / "who am I?"
  run_task("explore")            "go explore" / "map the house and find the plants"
  run_task("follow", name)       "follow Sam" -> name "Sam"; "follow me" -> no name
                                 (follows whichever enrolled person it sees)

Rules that matter:
  - Call stop() the instant you hear "stop", and whenever you are unsure
    whether it is safe to keep moving. A needless stop costs nothing.
  - Never guess a movement you are unsure of. The robot drives on a floor with
    obstacles it cannot see. If you did not understand, answer("unclear").
  - Ignore speech that is not addressed to you, and background conversation.
  - look_up() / look_down() / look_left() / look_right() move the head from where
    it is now; the default is 30 degrees and the head stops at its own limit if
    asked for more.
  - shutdown() powers the robot off. Call it only when the operator clearly says
    to shut down, power off or turn off; if unsure, answer("unclear") instead.
  - Defaults when no number is given: 1 metre, 90 degrees.
  - turn() takes positive degrees for RIGHT, negative for LEFT.
    drive() takes positive metres for FORWARD, negative for BACKWARD.
""")


# --- Secrets ------------------------------------------------------------------
class MissingSecret(RuntimeError):
    """A required credential is not configured."""


def require(name):
    """A secret's value, or a clear error naming where to set it."""
    value = globals().get(name, "")
    if not value:
        raise MissingSecret(
            f"{name} is not set. Put it in /etc/robot.env, "
            f"{Path(__file__).resolve().parent.parent / '.env'} or the environment.")
    return str(value)


def secret_values():
    """Non-empty secret values, for log redaction. Never log the result."""
    return tuple(v for v in (globals().get(n, "") for n in SECRET_NAMES) if v)
