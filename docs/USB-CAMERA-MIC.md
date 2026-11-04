# USB camera + microphone fallback — design and setup

The head gets the **innomaker U20CAM-1080P-WDR** (PS5268 sensor, 1080p30,
120 dB HDR, 121° diagonal, built-in MEMS microphone), mounted on the
pan/tilt gimbal. It is the only camera: the ribbon Camera V2 comes off and
there is no mono camera for now (CAMERA-PLAN Phase 2 is on hold).

Audio follows one rule:

| Buds connected (their mic is up) | Buds not connected |
|---|---|
| **Buds mode** — buds for everything: their mic for voice, their speaker for the robot's voice | **Room mode** — the camera's built-in mic for voice |
| Every command is acted on, as today | Only commands that **address the robot as "Robin"** are acted on. "Stop" always works, name or not |

Status: **implemented** (code, config, `check_audio.sh`, tests). The hardware
and Pi setup in Part 3 are still to do.

---

## Part 0 — Decisions

| Question | Decision |
|---|---|
| Camera | USB only, on the gimbal. `camera_backend: "opencv"` |
| Which mic | Buds when their mic is available, else the camera mic |
| When the robot acts | Buds: every command. Room mic: only when addressed as "Robin" (stop exempt) |
| How the mic is chosen in code | PipeWire's default input, set by the robot at every session start. `ROBOT_AUDIO_INPUT_DEVICE` stays blank |
| Mic change mid-session | The main session ends cleanly, the robot brakes, and it reconnects with the right mic and rules |
| Cancel listener during tasks | Uses the same mic choice; its prompt is unchanged ("stop" never needs the name) |
| Speaker without buds | **Open.** The robot's voice goes to the Pi's default output (3.5 mm jack / HDMI). With nothing plugged in it is silent; OLED and gestures still work. See Part 5 |

---

## Part 1 — Code changes (≈130 lines + ≈130 lines of tests)

### 1.1 Camera — config only (`ros2_ws/src/robot_bringup/config/robot.yaml`, `/explorer`)

```yaml
camera_backend: "opencv"         # USB only; don't probe for a ribbon camera
camera_index: 0                  # check with v4l2-ctl --list-devices (Part 3, step 4)
camera_main_size: [1920, 1080]   # unchanged
face_camera_size: [1920, 1080]   # was [640, 480] for the V2
camera_inverted: true            # as mounted: true = upside down (flipped in software on USB)
hfov_deg: 105.0                  # MEASURE (Part 3, step 6); was 66.0
```

No Python change: MJPEG capture, the one-frame buffer, full-resolution face
frames, enrol auto-framing and follow re-identification all landed in
Phase 1 (CAMERA-PLAN).

### 1.2 New: `robot_core/audio_route.py` (≈60 lines)

Decides buds vs room from PipeWire's list of inputs, and points the
default input (and, with buds, the default output) at the right device.

```python
@dataclass(frozen=True)
class Route:
    mode: str             # "buds" | "room"
    source: str | None    # PipeWire input to make the default
    sink: str | None      # with buds: their output; room: leave as is

def current() -> Route         # from `pactl list short sources` / `sinks`
def use() -> Route             # current(), made the default (pactl set-default-source/-sink), logged
def poll_interval() -> float   # 30 s for 5 min after start, then 120 s
```

* **Buds** = an input named `bluez_input.*` (PipeWire) or `bluez_source.*`
  (PulseAudio). With `BT_MAC` set, the one containing that address
  (`B4_84_D5_…`) is preferred.
  Keyed on the buds' **mic**, so buds connected for music only (A2DP) count
  as room mode, and still play the robot's voice.
* **Room** = anything else. The camera mic is the input whose name matches
  `ROBOT_ROOM_MIC` (default `alsa_input.usb-`). If none is found the default
  input is left alone, but the session still runs with the room rule (the
  safe side).
* `pactl` missing (laptop, tests) → room mode, nothing applied.
* `ROBOT_MIC_MODE=auto|buds|room` (default `auto`) forces a mode for testing.

### 1.3 `robot_core/settings.py` (≈20 lines)

* `ROBOT_MIC_MODE`, `ROBOT_ROOM_MIC`, and the buds check schedule:
  `ROBOT_ROUTE_POLL_FAST_S` (30 s), `ROBOT_ROUTE_FAST_FOR_S` (300 s),
  `ROBOT_ROUTE_POLL_S` (120 s).
* `ROOM_MIC_RULE` (`ROBOT_ROOM_MIC_PROMPT` to override), added to the end of
  `LIVE_SYSTEM_PROMPT` in room mode only:

  > You are listening through a microphone on your own body and will hear
  > people who are not talking to you, and the TV. Act only on a request that
  > addresses you by name ("Robin, go forward", "follow me, Robin"); the name
  > can be anywhere in the sentence. Without your name, do nothing at all: no
  > move, no gesture, no task. The one exception is stop: call stop() for any
  > "stop", "wait" or "hold on", with or without your name.

### 1.4 `robot_core/live/agent.py` (≈25 lines)

* `_run_supervised`, before each `LiveAgent(...)`:
  `route = audio_route.use()`, then the main prompt is
  `settings.live_prompt(route.mode)`. All prompt text and the rule for
  combining it live in `settings.py` (`LIVE_SYSTEM_PROMPT`, `ROOM_MIC_RULE`,
  `live_prompt()`, next to `CANCEL_PROMPT`). The `system_prompt=` parameter
  already exists (added for the cancel listener).
* Log `audio.route mode=… source=…` (new name in `robot_log.EVENTS`) and
  show `mic: buds` / `mic: room` on the OLED for a few seconds (`status.note`).
* A route watch in the main session re-reads the route on a schedule:
  **every 30 s for the first 5 minutes after the voice node starts, then
  every 2 minutes**. The clock starts once, when the voice node starts
  (kept in `audio_route`), so reconnects after tasks don't restart the fast
  phase. If the mode changed (buds connected or dropped), the session ends
  cleanly with `agent.rerouted = True`. Every session start also checks the
  route (1.4, first bullet), so the mode is always right right after boot,
  after a task and after a dropped connection.
* `_run_supervised` on `rerouted`: brake (`motion.stop()`), no spoken error,
  reconnect at once with the new route and rules. The brake is deliberate:
  buds dropping mid-command means the operator lost their voice channel.

### 1.5 `robot_core/live/cancel_watch.py` (≈3 lines)

`audio_route.use()` before each connect, so the
listener hears whichever mic is live. Its prompt and behaviour do not change.

### 1.6 Scripts, service and docs (no Python)

| File | Change |
|---|---|
| `check_audio.sh` (new, ≈40 lines; `check_bt_audio.sh` kept for the buds' mSBC) | Lists inputs/outputs, prints the route the robot would pick, records 3 s from the camera mic and plays it back |
| `robot-voice.service` | Unchanged. It keeps waiting for Bluetooth, since buds are still the first choice. `PULSE_SERVER`/`XDG_RUNTIME_DIR` are what make `pactl` work under systemd |
| `install_pi.sh` | Add `v4l-utils` (for `v4l2-ctl`) |
| `README.md`, `README-SETUP.md` | Audio section: buds first, room mic fallback, the "Robin" rule, the new settings |
| `docs/CAMERA-PLAN.md` | Phase 1 done with this camera; Phase 2 on hold |

### 1.7 Tests (≈70 lines)

* `audio_route.current()` against canned `pactl` output: buds present (with
  and without `BT_MAC`), buds in A2DP only, camera only, nothing, no `pactl`.
* The prompt gets the room rule only in room mode.
* A route change ends the session with `rerouted`, braking without speaking,
  and reconnects with the other prompt.
* The cancel listener applies the route before connecting.

---

## Part 2 — What happens when

| Situation | Mic | Robot acts on |
|---|---|---|
| Boot, buds on | Buds | Everything |
| Boot, buds off | Camera | "Robin, …" and any "stop" |
| Buds switched on mid-session | Noticed at the next check: within 30 s in the first 5 minutes, within 2 minutes after that (or at once if a task starts or ends first) → brake, reconnect (~1–2 s) on the buds | Everything |
| Buds die / walk out of range mid-session | Same delay. **Until the next check** PipeWire has already moved the session onto the camera mic, but the buds rules are still on (see hiccup 8) | Until then: everything; after: "Robin, …" and "stop" |
| During a task (follow, enrol, explore) | The cancel listener on whichever mic is live | "Stop" (no name needed) |
| Camera unplugged / USB reset, no buds | No mic: the session drops and retries with backoff; the log shows `audio.route source=null` | Nothing until it is back. Use `task/stop` for tasks |

---

## Part 3 — Setup steps (on the Pi)

1. **Hardware.** Power off. Remove the Camera V2 and its ribbon. Mount the
   U20CAM on the gimbal with a slack loop of thin USB cable (the SG90s are
   weak and the ribbon used to snap here). Plug it into a Pi 4 USB port.
2. **Packages.** `sudo apt install v4l-utils` (the rest — PipeWire,
   `pulseaudio-utils`, `libportaudio2` — is already there from `install_pi.sh`).
3. **Is it seen?** `lsusb` should list the camera. `arecord -l` should show
   a USB audio card.
4. **Video device.** `v4l2-ctl --list-devices` lists the camera's
   `/dev/videoN` (usually `/dev/video0`, with `/dev/video1` as metadata).
   Put N in `camera_index`.
5. **Config.** Edit `robot.yaml` as in 1.1. Rebuild/install the ROS
   workspace so the installed copy picks it up.
6. **Measure the field of view.** Tape a 1 m wide target to a wall, head
   level and straight. Take a frame:
   `v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG --stream-mmap --stream-count=1 --stream-to=/tmp/f.jpg`,
   copy it to the Mac and look. Move the robot until the target exactly fills
   the image width; d = camera-to-wall distance in metres. Then
   `hfov_deg = 2·atan(0.5 / d)` (expect ~105°). Also check the image is upright,
   and flip `camera_inverted` if not.
   `python tests/hardware/check_camera.py head` then confirms frames keep
   coming while the servos move.
7. **Mic name.** `pactl list short sources`: the camera mic is the
   `alsa_input.usb-…` line. If the default `ROBOT_ROOM_MIC=alsa_input.usb-`
   matches more than one input, set a longer piece of the name in
   `/etc/robot.env`.
8. **Mic gain.** `pactl set-source-volume <camera source> 100%` (camera mics
   often ship quiet). Then record and listen:
   `parecord --device=<camera source> /tmp/t.wav` from 2 m away, and
   `paplay /tmp/t.wav`. Raise the gain if quiet, lower it if the motors clip.
   PipeWire remembers it.
9. **Leave `ROBOT_AUDIO_INPUT_DEVICE` blank** in `/etc/robot.env`. The robot
   picks the input through PipeWire, which also converts the camera mic to the
   16 kHz mono Gemini wants. Naming the raw ALSA device instead can fail with
   "invalid sample rate".
10. **Buds unchanged.** Keep them paired. `check_bt_audio.sh` still switches
    them to mSBC (16 kHz).
11. **Restart.** `sudo systemctl restart robot-voice`. Check `logs.json` for
    `audio.route mode=room` (buds off) and `mode=buds` (buds on).
12. **Re-enrol faces** with the new camera (new sensor, new colours). The
    gallery from the V2 still works, but re-enrolling is better.

---

## Part 4 — Test checklist

| # | Test | Pass |
|---|---|---|
| 1 | Buds off. "Go forward" (no name) | Nothing happens |
| 2 | Buds off. "Robin, go forward" | Drives |
| 3 | Buds off, while driving. "Stop" (no name) | Brakes |
| 4 | Buds off. TV on with talk/speech for 2 min | No moves, no gestures |
| 5 | Buds on. "Go forward" (no name) | Drives (`audio.route mode=buds`) |
| 6 | Buds on, then switch them off (within the first 5 minutes) | Within 30 s: brakes, OLED `mic: room`, test 1 behaviour |
| 7 | Buds off, then switch them on (after the first 5 minutes) | Within 2 minutes: brakes, OLED `mic: buds`, test 5 behaviour |
| 8 | Buds off. "Robin, follow me", walk, then "stop" | Follow ends, robot nods |
| 9 | Buds off. Speak at 1, 2 and 3 m, head still vs panning | Note where commands start being missed |
| 10 | Buds off. "Robin, remember my face, I'm Roshan" | Saved as `Roshan` (watch for misheard names) |
| 11 | Explore for 10 min, watch Pi CPU (`htop`) | Note the load (see hiccup 7) |

---

## Part 5 — Hiccups and regressions to expect

1. **No speaker without the buds.** Today the buds are the robot's voice too
   ("Robot online", spoken errors, battery report). In room mode that audio
   goes to the Pi's default output; with nothing on the 3.5 mm jack the robot is
   mute. Fix later with a small powered speaker on the jack (or USB) — no code
   change, it just becomes the default output.
2. **Far-field pickup is worse.** The camera mic is 1–3 m from you and right
   next to the motors and gears. Expect more misses (especially "stop" while
   driving) and misheard names when enrolling. Test 9 measures it.
3. **The mic is on the gimbal.** SG90 whine during head moves and gestures is
   right at the mic, and its direction changes as the head pans. Commands
   given while the head moves are the most likely to be missed.
4. **The "Robin" rule depends on the model following it.** It is a prompt
   rule, not a hard gate. It should hold for everyday TV and conversation,
   but a sentence like "Robin go forward" from the TV would still move it.
   `stop` is exempt by design: a false stop is harmless.
5. **Forgetting the name means silence.** In room mode the robot ignores
   un-named commands without even a gesture. That's deliberate, or it would
   shake its head at the TV. The OLED `mic: room` note is the hint.
6. **One USB link for camera and mic.** A power dip that resets the camera
   takes the mic with it. Without buds the session drops and retries until it
   is back. Tasks can still be cancelled with
   `ros2 service call /task/stop std_srvs/srv/Trigger`.
7. **Explore CPU.** Explore's motion check takes a small frame 5 times a
   second, and on USB each one means decoding a full 1080p MJPEG frame
   (≈15–25% of a core, estimated). If `htop` shows it hurting, open explore's
   camera at 1280×720 (`camera_main_size`) — a config change.
8. **Buds dropping leaves a gap of up to 2 minutes.** When the buds die,
   PipeWire moves the session onto the camera mic straight away, but the
   robot only notices at its next check (30 s early on, 2 minutes later).
   Until then it still acts on every command, from the room mic: the TV or
   other people could move it. Ways to close the gap if it matters in
   practice: say "stop" and reconnect by voice, power-cycle the buds near
   the robot, or later switch the check to PipeWire's change events
   (`pactl subscribe`), which notices at once and costs nothing between
   changes. Each change also costs a brake and ~1–2 s deaf, and ends any
   command in progress. The checks themselves cost nothing noticeable.
9. **Buds connected but not worn.** If they stay connected (out of the case,
   on the desk), the robot listens to them, not the room. The OLED shows
   `mic: buds`.
10. **Video device number.** `camera_index` is a number; a second USB video
    device would shift it. Not an issue with one camera. Re-check step 4
    if you add one.
11. **Echo.** Low risk: the robot only speaks while the mic is closed, and
    Gemini's replies are discarded. If a speaker is added and
    `ROBOT_LIVE_PLAY_AUDIO` is ever turned on, the room mic will hear it.

---

## Part 6 — Settings summary

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_MIC_MODE` | `auto` | `auto` = buds if their mic is up, else room; `buds` / `room` force it |
| `ROBOT_ROOM_MIC` | `alsa_input.usb-` | Piece of the camera mic's PipeWire name |
| `ROBOT_ROUTE_POLL_FAST_S` | `30` | Buds check interval for the first `ROBOT_ROUTE_FAST_FOR_S` after the voice node starts |
| `ROBOT_ROUTE_FAST_FOR_S` | `300` | Length of the fast phase (5 minutes) |
| `ROBOT_ROUTE_POLL_S` | `120` | Buds check interval after that (2 minutes) |
| `ROBOT_ROOM_MIC_PROMPT` | see `settings.py` | The "Robin" rule added in room mode |
| `ROBOT_AUDIO_INPUT_DEVICE` | blank | Leave blank: the mic is chosen through PipeWire |
| `BT_MAC` | blank | Prefer these buds when several are paired |
