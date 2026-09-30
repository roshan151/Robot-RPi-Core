# Robot Computer Vision

Raspberry Pi–based robot with **computer vision**, **voice + LLM planning**, and an **Arduino** handling real-time **drivetrain** control (PID, encoders) over **USB serial**. The Pi sends high-level intent only (`F` / `B` / `L` / `R` / `S`, speed, heartbeat); the Arduino runs the low-level control loop.

---

## Repository layout

Two layers, and the boundary is the point. `robot_core/` is the robot as plain
Python and imports **no ROS**; `ros2_ws/` is thin nodes that call into it. That
keeps the tests runnable on a laptop, stops a node quietly growing hardware
logic, and means deleting `ros2_ws/` still leaves a working robot. There is a
test for it (`tests/test_layering.py`).

| Path | Purpose |
|------|--------|
| `firmware/drivetrain/drivetrain.ino` | Arduino sketch — motors, encoders, PID, serial protocol. Its header comment is the authoritative pinout |
| **`robot_core/`** | **Layer A — no ROS imports, ever** |
| `robot_core/settings.py` | Serial port, baud, calibration, timeouts, environment-variable defaults |
| `robot_core/drivetrain/` | Framed serial protocol, the Arduino bridge, and `SerialDrivetrain` |
| `robot_core/motion_executor.py` | FIFO worker that owns the link; cancellation that interrupts a move in flight |
| `robot_core/motion.py` | `MotionBackend` — the seam the tools call, and `LocalMotion`, its no-ROS implementation |
| `robot_core/gestures.py` | The gesture vocabulary. Single source of truth for what the model may ask for |
| `robot_core/live/agent.py` | The Gemini Live session: mic uplink, event pump, result feedback |
| `robot_core/live/tools.py` | `drive` / `turn` / `stop` / `answer`, as Live function declarations |
| `robot_core/speech.py` | Gemini TTS over REST, cached to disk |
| `robot_core/run.py` | Run everything **without** ROS — the bench path and the fallback |
| **`ros2_ws/src/`** | **Layer B — thin nodes** |
| `robot_interfaces/` | `Drive.action`, `Turn.action`, `Encoders.msg` |
| `robot_drivetrain/` | Action servers, the e-stop service, encoder telemetry |
| `robot_voice/` | The Live session as a node, plus `RosMotion` (MotionBackend over actions) |
| `robot_bringup/` | Launch files, `robot.yaml`, and the one-process node host |
| **`vision_service/`** | **The off-Pi perception service — runs on the Mac mini, not the robot** |
| `tests/` | Unit tests — no hardware, no ROS needed |
| `tests/hardware/` | `check_*.py` diagnostics that need a real robot |
| `docs/PLAN.md` | The migration and expansion plan. Start here |
| `docs/VISION-DESIGN.md` | The vision/perception layer: what runs where, and why |

## Hardware (summary)

- **Raspberry Pi 4B** — vision, planning, serial to Arduino  
- **Camera** — high-resolution module (Picamera2) or USB / OpenCV  
- **Drivetrain** — 2× DC motors, **L298** (or similar), **separate motor battery**; **common ground** with logic  
- **Arduino** — motor PWM/direction, quadrature encoders, watchdog on serial  
- Optional: TF-Luna distance sensor (your older design mentioned obstacle override — wire through planning/firmware as you prefer)

Longer power/wiring notes from your build are still valid; keep motor supply separate from logic where applicable.

---

## Prerequisites

- **Raspberry Pi**: Python 3.10+ recommended  
- **Arduino**: Uno-class or compatible (interrupt-capable encoder pins per sketch)  
- USB cable **Arduino ↔ Pi** (serial; often `/dev/ttyACM0` or `/dev/ttyUSB0`)  
- **API keys**: `OPENAI_API_KEY` or `OPENAI_API_KEY_ROBIN` in `.env` for voice mode  

---

## Installation

**Layer A** (works anywhere — laptop included):

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[voice]"
pytest                      # 49 tests, no hardware required
```

**Layer B** (on the Pi) needs ROS 2 Jazzy. See `docs/PLAN.md` Part 3 for the
apt setup, then:

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

Only `robot_interfaces` actually compiles; `--symlink-install` means Python
edits take effect without rebuilding. Rebuild when you change a `.action` or
`.msg`.

## Configuration

### Environment variables

All can be set in a `.env` file, in `/etc/robot.env` for the systemd service, or exported in the shell. Anything in `robot.yaml` overrides the corresponding default when running under ROS.

| Variable | Purpose |
|----------|--------|
| `ROBOT_SERIAL_PORT` | Default `/dev/ttyUSB0` — set to `/dev/ttyACM0` if needed |
| `ROBOT_SERIAL_BAUD` | Must match Arduino `Serial.begin(...)` (default `115200`) |
| `ROBOT_METERS_PER_SECOND` | Scales spoken “meters” into drive duration |
| `VISION_SERVICE_BASE_URL` | Base URL of the vision service, e.g. `http://mini.local:8080` |
| `VISION_DETECT_HZ` | Detect call rate (default 2.0) |
| `VISION_DETECT_TIMEOUT_S` | Hard abandon, no retry (default 0.6) |
| `OPENAI_API_KEY` / `OPENAI_API_KEY_ROBIN` | Voice + planning |

### Vision service

The perception models run **off the Pi** — normally on a Mac mini on the LAN,
because the Pi has no GPU. Source, endpoint contract and deployment live in
[`vision_service/`](vision_service/README.md); the design rationale is in
[`docs/VISION-DESIGN.md`](docs/VISION-DESIGN.md).

```bash
cd vision_service && pip install -e '.[models,dev]' && ./run.sh
```

`VISION_SERVICE_BASE_URL` is a **base URL** (default `http://127.0.0.1:8080`),
not a single endpoint — the gateway client appends the versioned paths
(`/v1/detect`, `/v1/faces/embed`, `/v1/faces/match`, `/v1/faces/enroll`,
`/v1/depth`) and is the only thing that knows them. Point it at the mini if the
service runs elsewhere, and check `GET /healthz` when the robot seems blind.

The camera-based obstacle guardian is **retired**: the lidar stops the robot and
the camera tells the agent what is there. `VISION_HALT_OBJECTS` and
`VISION_GUARD_HZ` no longer do anything.

---

## Arduino firmware

1. Open `firmware/drivetrain/drivetrain.ino` in the Arduino IDE.  
2. Adjust **pin defines** at the top for your motor driver and encoders. The
   header comment of that file is the authoritative pinout — see
   [Motor driver wiring](#motor-driver-wiring-2-drv8871) and
   [Motor ↔ encoder pairing](#motor--encoder-pairing-invariant) for the
   as-built harness and the invariant it has to satisfy.  
3. Select board/port, **Upload**.  
4. Optional: Serial Monitor at the same baud as `ROBOT_SERIAL_BAUD` to see `ACK` / `ENC:` lines.

You can test upload and serial **without motors connected**; encoder lines use internal pull-ups (counts may be noisy when floating).

---

## Running the robot stack

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_bringup command.launch.py
```

Two processes on purpose: the voice node holds an always-open microphone and an
asyncio loop, the robot process holds the serial link and the e-stop. Separate
processes mean separate GILs, so a stall in the conversation cannot delay a stop.

| Want | Command |
|------|---------|
| Drivetrain only, no mic or API key | `ros2 launch robot_bringup command.launch.py voice:=false` |
| Drive it by hand | `ros2 action send_goal /drive robot_interfaces/action/Drive "{meters: 0.5}" --feedback` |
| Stop it | `ros2 service call /estop std_srvs/srv/Trigger` |
| Watch the encoders | `ros2 topic echo /encoders` |
| Calibrate live | `ros2 param set /drivetrain ticks_per_cm 103.4` |
| No ROS at all | `python -m robot_core.run` |

As a service: `start_robot.sh` sources both overlays and launches the graph;
`robot-voice.service` calls it.

### Boot service: stop, run manually, re-enable

The Pi starts the robot at boot via `robot-voice.service`. To work on it:

| Want | Command |
|------|---------|
| Stop it now (starts again next boot) | `sudo systemctl stop robot-voice` |
| Stop it and keep it off across reboots | `sudo systemctl disable --now robot-voice` |
| Run it manually in the foreground (Ctrl+C to stop) | `./start_robot.sh` |
| Start the service manually | `sudo systemctl start robot-voice` |
| Restart after code changes | `sudo systemctl restart robot-voice` |
| Re-enable start at boot (and start now) | `sudo systemctl enable --now robot-voice` |
| Is it running / enabled? | `systemctl status robot-voice` · `systemctl is-enabled robot-voice` |
| Follow its logs | `journalctl -u robot-voice -f` |

Stop the service before running `./start_robot.sh` by hand — two copies will fight over the serial port and microphone. After editing `robot-voice.service` itself, re-copy it to `/etc/systemd/system/` and run `sudo systemctl daemon-reload`. If it crash-looped (5 failures in 2 min), clear it with `sudo systemctl reset-failed robot-voice` before starting again.

### Explore mode (map the house, photograph every plant)

```bash
sudo systemctl stop robot-voice     # explore and command mode can't share the serial port or camera
./start_robot.sh explore            # ends by itself when nothing reachable is left unexplored
ros2 service call /explore/stop std_srvs/srv/Trigger   # stop early; the map is still saved
ros2 topic echo /explore/status     # what it is doing, in plain English
```

The robot stops, sweeps the TF-Luna ±45° on the head in each of four directions (a 360° scan), and sends that keyframe to the Mac. The Mac matches it into the map, corrects the drifting wheel odometry, and returns a path to the nearest unexplored area. Whenever the camera sees a plant, the robot ranges it, checks the plant register on the Mac, then drives to 3–4 viewpoints around it and saves a photo from each. New plants are enrolled as `plant_NNN`; known plants get a new visit, which is what growth tracking reads. Before every drive step the head tilts down to check the floor, which catches low obstacles and drops that the level sweep misses.

- **Map:** `http://<mac>:8080/v1/slam/map.png` (plants marked in red). **Photos:** `~/.robot-vision/plants/<plant>/` on the Mac.
- **Re-scan later on the same map:** set `continue_map: true` and `start_pose` (where the robot stands, read off the map) under `/explorer` in `robot.yaml`.
- **Motion health:** `ros2 topic echo /motion_health` compares the command, the encoders and the camera. It reports `stalled` (wheels blocked), `slipping` (wheels turning, image static), `pushed`, or `wrong_direction`; the drivetrain brakes on stalled or slipping.
- **Park the head by hand** after a crash or power cut: `python -m robot_core.sensors.gimbal home`.

One-time setup:

1. Flash firmware v5 (`./flash.sh`); the boot line must say `drv8871-v5-odo`. `/odom` needs its never-reset encoder totals.
2. In `/boot/firmware/config.txt`, add `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` (tilt on GPIO12, pan on GPIO13). Then run `sudo raspi-config` → Interface → Serial Port: login shell **No**, hardware **Yes** (TF-Luna on `/dev/serial0`). Reboot.
3. `sudo apt install python3-picamera2 python3-opencv` and `pip install rpi-hardware-pwm`.
4. Rebuild the workspace (new messages and the `robot_explore` package): `cd ros2_ws && colcon build --symlink-install`.
5. Measure and set `head_height_m`, `pan_axis_x_m`, `hfov_deg` and the servo `*_center_us` trims under `/explorer` in `robot.yaml`. `+pan` must turn the head left and `+tilt` must look up; flip `pan_invert`/`tilt_invert` if not.
6. On the Mac, restart the vision service (DINOv2 downloads on first start). For `/v1/ask`, set `GEMINI_API_KEY` and/or run `ollama pull qwen3-vl:8b`.

## Robot images

**Top view:**  
![Top](https://github.com/user-attachments/assets/65aa1004-ad71-4d3f-be6d-fdf788f3cd46)

**Side view:**  
![Side](https://github.com/user-attachments/assets/65aa1004-c084-4d42-8a80-22c9ce910a82)

**Front view:**  
![Front](https://github.com/user-attachments/assets/65aa1004-b8d2-4fd7-ae23-dbf517e464cc)

---

## Resources

- Speech recognition overview: [Real-time speech-to-text on Raspberry Pi](https://atsss.medium.com/real-time-speech-to-text-on-raspberry-pi-and-python-4be8c347a8fc)  
- Text-to-speech: [Gemini TTS](https://ai.google.dev/gemini-api/docs/speech-generation) — the robot's voice, called over REST from `robot_core/speech.py` and cached to disk. The old tone cues (`audio_cues.py`) and the espeak-ng / Nix TTS path (`speech.py`) are gone.

### Voice setup

No extra credential: Gemini TTS is on the same host and uses the same
`GEMINI_API_KEY` as the Live session. Cloud Text-to-Speech (WaveNet) is cheaper
per utterance but needs a second key on a billing-enabled Cloud project, since
AI Studio keys are restricted to the Generative Language API.

Two things follow from sharing the key. Rate limits are per *project*, so
announcements and the Live session draw on the same quota — which is the main
reason everything is cached. And the model must be a TTS variant:
`gemini-2.5-flash-preview-tts`, not `gemini-2.5-flash`, which is text-out only
and rejects the audio response modality.

Cache the phrases the robot must be able to say with no network — do this once,
while it does have one:

```
python -m robot_core.speech --prime          # renders the static phrases into the cache
python -m robot_core.speech --info           # cache location, size, and what is primed
python -m robot_core.speech "hello there"    # audition any text
```

The robot speaks at exactly three moments, all of them while no capture stream
is open: the battery report at boot, "voice session connected" before the
microphone opens, and the failure announcement after the session is torn down.
Anything else would be streamed straight back into the model as if you had said
it. Set `ROBOT_TTS=0` to mute it entirely; see the speech section of `robot_core/settings.py`
for model, voice, style, output device, and cache directory.

Delivery is directed in natural language rather than with rate/pitch dials —
`ROBOT_TTS_STYLE` is prepended as `"<style>: <text>"`. Keep it short: long
director's notes are the documented cause of the model reading the instructions
aloud instead of following them.


## Debugging Arduino
Check if anything is occupying the port: lsof /dev/cu.usbserial-A5069RR4

## Electrical Schematics

### Bill of Materials

Motors: Uses 2 DC motors, Specs -  12v, 130 RPM, Geared motors
Battery pack: (Used to power DRV and thr 2 motors) 3S Lipo Battery, 50C, 2200mAh, 11.1V
Motor Controller: **2× DRV8871 breakout** (one per motor, ILIM ≈ 2.1 A via 30k).
Earlier revisions used a single dual-channel DRV8833; the sections below that
still say "DRV8833" have not been re-verified against the current build.
**Note the voltage warning in [Power path](#power-path) is a DRV8833 limit
(10.8 V max) and does not apply to the DRV8871** — but do not raise the buck
on that basis alone; the firmware header specifies 8–9 V, and the motors and
encoders have their own limits. Re-verify before changing the supply.
Microcontroller: Arduino used to control motors (Powered by Raspberry Pi through USB B port).
Raspberry Pi 4B: Used for computer vision and voice controls, Powered by PiSugar battery.

| Component | Qty | Value | Purpose |
|-----------|-----|-------|---------|
| Ceramic Capacitor | 1 | 100nF (0.1µF) | DRV8833 VCC bypass |
| Ceramic Capacitor | 1 | 100nF (0.1µF) | Left motor output filter |
| Ceramic Capacitor | 1 | 100nF (0.1µF) | Right motor output filter |
| Electrolytic | 1 | 1000µF | Buck converter output bulk |

### Power path
```
LiPo 3S (11.1 V nominal, 12.6 V charged)
   ├─ (+) ──→ Buck Converter IN+
   └─ (−) ──→ STAR POINT ──→ Buck Converter IN−

Buck Converter OUT+ (set to 10 V) ──→ DRV8833 VCC   ← powers motors AND chip
Buck Converter OUT− ──→ Star ground
1000 µF electrolytic across Buck OUT+ / OUT−
```

> **Important:** this DRV8833 module has **no separate VM pin** — the pin
> labelled VCC is the single supply for both the motors and the chip itself.
> The DRV8833's recommended maximum is **10.8 V**, so a 10 V buck setting
> leaves almost no headroom; **setting the buck to ~9 V is safer** (the 12 V
> motors just run a little slower). Never connect raw LiPo voltage to VCC.

### STAR Ground Physical Layout
```
                    ┌─── 16 AWG wire ──→ DRV8833 GND pin
                    │
LiPo (−) ──STAR POINT
                    │
                    ├─── 16 AWG wire ──→ Arduino GND pin (Arduino itself is USB-powered)
                    │
                    └─── 16 AWG wire ──→ Buck Converter GND IN
                    (encoder grounds also return here)
```

### Motor driver wiring (2× DRV8871)

> **Superseded.** This section previously described a single dual-channel
> DRV8833 with IN1–IN4 and an EEP pin on **D7**. The build now uses **two
> single-channel DRV8871 breakouts**, one per motor, and **D7 is the right
> encoder's B channel**. Wiring anything to D7 as an enable line will break
> quadrature decoding on the right wheel. The authoritative pinout is the
> header comment of `firmware/drivetrain/drivetrain.ino`.

Each DRV8871 board has its own `IN1` / `IN2` inputs and its own `OUT1` / `OUT2`
motor terminals. Per the driver's truth table (mirrored in `motorWrite()`):

| IN1 | IN2 | Result |
|-----|-----|--------|
| PWM | LOW | forward |
| LOW | PWM | reverse |
| LOW | LOW | coast (auto-sleep) |
| HIGH | HIGH | brake |

**As built:**

```
Arduino          DRV8871 board          Wheel
─────────        ─────────────          ─────
D5  (PWM) ──→    LEFT  board IN1        Motor 2  = LEFT wheel
D6  (PWM) ──→    LEFT  board IN2
D9  (PWM) ──→    RIGHT board IN2  ⚠     Motor 1  = RIGHT wheel
D10 (PWM) ──→    RIGHT board IN1  ⚠
Star GND  ──→    both boards POWER−     (own wire each, no ground ring)
                 both boards POWER+ ←── Buck OUT+ (8–9 V)
```

⚠ **The right board's IN1/IN2 are deliberately reversed** relative to the
firmware's naming (D9 is `IN1_R`, "forward", but lands on the board's IN2).

This is not a mistake, and it must not be "corrected". The two motors are
mirror-mounted, so one side has to be inverted for a `F` command to drive both
wheels the same way down the floor. The firmware has **no motor-invert flag**
— only `ENC_L_INVERT` / `ENC_R_INVERT`, which invert *encoders*, not motors —
so the inversion has to live in the wiring. Swapping a board's IN1/IN2 is
exactly equivalent to swapping its OUT1/OUT2 motor leads, because `motorWrite()`
is symmetric in the two pins; either is fine, but only one may be applied.

If you ever rebuild the harness to match the firmware header literally
(D9→IN1, D10→IN2 on the right board), you must then swap that motor's OUT1/OUT2
leads instead, or the robot will spin in place on every `F`.

### Encoder wiring

2× TSINY-8370 dual-channel Hall encoders. Colours: **yellow = A**, **white = B**,
blue = Vcc, green = GND.

| Encoder wire | Connect to | Notes |
|--------------|-----------|-------|
| blue (Vcc) | Arduino **5V** | not the 9 V rail — the internal 10k pull-up ties Vout to Vcc, so Vcc must equal the logic voltage |
| green (GND) | Star GND | same node as Arduino GND |
| LEFT yellow (A) | **D2** | INT0, rising-edge interrupt |
| LEFT white (B) | **D4** | sampled inside the ISR |
| RIGHT yellow (A) | **D3** | INT1, rising-edge interrupt |
| RIGHT white (B) | **D7** | sampled inside the ISR |

**Both channels are load-bearing.** The firmware is true quadrature: the ISR
fires on A's rising edge and samples B at that instant, so counts are
*hardware-signed* and register real motion — a wheel rolling backwards down a
slope counts down. Direction is **not** inferred from the commanded move.
(Earlier revisions of this README listed right B on D8 and called the B
channels "informational". Both statements are obsolete.)

Consequences of getting A/B backwards on one side: the decoded sign inverts,
signed progress clamps to zero at `drivetrain.ino:673`, the move never reaches
its tick target, and it ends in `TIMEOUT` after 15 s. Swap the two wires rather
than reaching for `ENC_x_INVERT` — a physically correct A/B pairing also fixes
the edge timing, which the invert flag does not.

`ENC_L_INVERT` / `ENC_R_INVERT` are for the *other* problem: correctly paired
A/B, but the motor mounted mirror-image so forward rotation counts down.

**As built: `ENC_R_INVERT 1`.** The right motor is mirror-mounted, so forward
travel spins it the opposite way and its (correctly paired) A/B decodes
negative. Confirmed by hand-spin — rolling both wheels forward gave L=+242,
R=−187 before the flag was set.

> **Both faults negate the sign, so counts alone cannot tell them apart.**
> This bit us once: the harness originally had right yellow→D7 / white→D3
> (A/B swapped) *and* the mirror-mounted motor, and the two errors cancelled.
> `ΔR` read positive, everything looked plausible, and the real inversion only
> appeared once the wire colours were corrected. Distinguish by inspecting the
> **wire colours against the convention** (yellow = A → interrupt pin D2/D3),
> not by the sign of the counts. Colours correct + counts negative → set the
> invert flag. Colours swapped → fix the wires first, then re-test.

### Motor ↔ encoder pairing (invariant)

**Each motor's driver channel and its encoder channel must name the same
wheel.** The left board (D5/D6) must be paired with the left encoder (D2/D4);
the right board (D9/D10) with the right encoder (D3/D7).

This is easy to violate and expensive to diagnose, because straight moves hide
it. `wheelSigns()` maps `F` to (+1, +1) and `B` to (−1, −1) — symmetric, so
crossed channels look fine. `L` is (−1, +1) and `R` is (+1, −1) — antisymmetric,
so crossed channels mirror every turn.

The subtler damage is to the sync trim. The loop at `drivetrain.ino:610`
compares `enc_left` against `enc_right` and applies its correction to channels
L and R. If the pairing is crossed, the correction lands on the wheel it was
not computed for and the loop becomes **positive feedback**: it speeds up the
wheel already ahead until it saturates at `SYNC_AUTHORITY_PCT`. The signature
is a left/right error pinned at a constant magnitude across every move —
consistently one-sided, and never converging.

**Symptom → cause:**

| Symptom | Cause |
|---------|-------|
| `F` drives straight, but `R` turns left | motor channels crossed |
| Sync error stuck at a constant %, always same side | motor↔encoder pairing crossed |
| Sync error alternates sides, varying size | genuine drift — this is what `SYNC_KP` is for |
| Move ends in `TIMEOUT`, one side counts down, other side's count runs far past target | that side's sign is inverted — A/B swapped **or** a mirror-mounted motor without its `ENC_x_INVERT` |
| Neither encoder moves | power, encoder wiring, or serial |

**Verify after any harness change**, before calibrating anything:

1. `tests/hardware/check_encoders.py`; hand-roll each wheel in the robot's forward
   direction. Both must count **up** — fix with `ENC_x_INVERT` and re-flash.
2. In the same test, roll the **left** wheel only. `enc_left` must be the
   counter that moves. If `enc_right` moves instead, the pairing is crossed.
3. Command `F` briefly. Both wheels must drive the robot forward, not spin it.
4. Command `R`. The robot must turn right.

Only once all four pass are the calibration numbers meaningful.

### Calibration

Two constants in `robot_core/settings.py`, both overridable by environment variable so you
can calibrate without editing code:

| Constant | Env var | Governs |
|----------|---------|---------|
| `TICKS_PER_CM` | `ROBOT_TICKS_PER_CM` | straight moves |
| `TICKS_PER_DEGREE` | `ROBOT_TICKS_PER_DEGREE` | tank turns |

Both scale the **target**, not the measurement. The firmware ends a move when
*both* wheels reach the target (`drivetrain.ino:714`), so neither constant can
cause a move to stop early — short travel is always traction or a lost encoder
channel, never calibration.

**Distance.** On flat ground, on the surface you will actually run on, at your
normal mission speed (slip changes with PWM, so calibrating at 100 % and
running at 70 % gives a wrong constant):

1. Mark a start line. Run `drivetrain.straight_m(1.0)`.
2. Measure actual travel in cm.
3. `new = current × (100 / measured_cm)`
4. Repeat three times and average — a single trial is noise.

**Turn.** Use a full rotation, not 90°: a 5° reading error at 90° is inside
your measurement precision, while the same relative error at 360° shows up as
20° and is actually readable.

1. Mark the floor under the robot's centre; tape a pointer to the chassis.
2. Run `drivetrain.right(360)`.
3. `new = current × (360 / measured_degrees)`
4. Verify with 4× `right(90)` — the robot should return to its start heading.

`TICKS_PER_DEGREE` is the least portable constant in the tree: a tank turn
scrubs both wheels sideways, so carpet and hardwood genuinely need different
values. Distance is closed-loop on ticks and therefore battery-independent,
but slip is not — calibrate at mid-charge.

### Capacitors: What, Where, How

| Capacitor | Type | Location | Why |
|-----------|------|----------|-----|
| #1 (100nF) | Ceramic disc | **DRV8833 VCC → GND** | High-frequency bypass for the driver's supply pin |
| #2 (100nF) | Ceramic disc | **DRV8833 OUT1 → OUT2** | Suppress left motor brush noise |
| #3 (100nF) | Ceramic disc | **DRV8833 OUT3 → OUT4** | Suppress right motor brush noise |
| #4 (1000µF) | Electrolytic | **Buck Vout → Buck GND** | Bulk storage on the motor rail (VCC) for current spikes |

### Reading the BOOT code (reset diagnosis)

Every time the Arduino starts it prints `BOOT:<hex>`. The hex digit says
**why** the chip restarted — this is the primary tool for diagnosing
mid-drive resets:

| Code | Cause | What it means here |
|------|-------|--------------------|
| `1` | Power-on | The Arduino's 5 V vanished completely → **USB power was interrupted** (cable snag, loose connector, Mac port overcurrent shutdown) |
| `2` | Reset pin | Normal right after opening the serial port (DTR pulse). Mid-drive: electrical noise reached the RESET pin |
| `4` | Brown-out | The 5 V rail sagged → something wired to the Arduino's 5 V (both encoders' blue leads) is dragging it down, or ground bounce |
| `8` | Watchdog | Not used by this firmware |

Bits can combine (e.g. `3` = power-on + reset pin).

---

Wire the white B wires: left → D4, right → D7 (blue → 5V, green → GND, yellow → D2/D3 as before).
Flash (./flash.sh or IDE) — confirm the drv8871-v4-quad stamp.
Calibrate polarity: run tests/hardware/check_encoders.py, roll each wheel in the robot's forward direction by hand. Both must count up. A side counting down → set its ENC_x_INVERT to 1, re-flash, re-check.
Then tests/hardware/check_movements.py — with working, signed encoders this should be the first honest closed-loop run the bot has ever had.
