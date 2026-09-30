# Robot Computer Vision

A Raspberry Pi robot that listens (Gemini Live voice), looks (camera + TF-Luna on a pan/tilt head), and explores. An **Arduino** runs the real-time drivetrain (PID, encoders) over **USB serial**; a **Mac mini** runs the heavy vision work (detection, faces, plants, SLAM, image+text reasoning). The Pi keeps no data: maps and photos are stored on the Mac.

---

## Repository layout

Two layers, and the boundary is the point. `robot_core/` is the robot as plain Python and imports **no ROS**; `ros2_ws/` is thin nodes that call into it. That keeps the tests runnable on a laptop and means deleting `ros2_ws/` still leaves a working robot. `tests/test_layering.py` enforces it.

| Path | Purpose |
|------|--------|
| `firmware/drivetrain/drivetrain.ino` | Arduino sketch: motors, encoders, PID, serial protocol. Its header comment is the authoritative pinout |
| `robot_core/settings.py` | Every setting, with its environment variable and default |
| `robot_core/drivetrain/` | Framed serial protocol, the Arduino bridge, `SerialDrivetrain` |
| `robot_core/motion_executor.py`, `motion.py` | The move queue that owns the serial link, and the `MotionBackend` seam the tools call |
| `robot_core/odometry.py` | Wheel odometry, drift estimate, and motion health (stalled / slipping / pushed) |
| `robot_core/sensors/` | Pan/tilt gimbal, TF-Luna, camera, visual motion, and the `Head` that combines them |
| `robot_core/explore/` | The explore mission: scan, plan, find plants, photograph them |
| `robot_core/face_tasks.py`, `vision_client.py` | Voice-triggered face enroll/match, and the client for the Mac service |
| `robot_core/live/` | The Gemini Live session (`agent.py`) and its tools (`tools.py`: drive, turn, stop, answer, run_task) |
| `robot_core/gestures.py`, `speech.py` | Yes/no/dance gestures; Gemini text-to-speech cached to disk |
| `robot_core/run.py` | Run everything without ROS (bench use) |
| `ros2_ws/src/` | ROS 2 nodes: `robot_interfaces` (actions, messages), `robot_drivetrain`, `robot_voice`, `robot_explore`, `robot_vision` (Mac-service client node), `robot_bringup` (launch files, `robot.yaml`) |
| `install_pi.sh`, `start_robot.sh`, `robot-voice.service` | Pi setup, the launcher, the boot service |
| `flash.sh`, `check_bt_audio.sh`, `vision_test.py`, `read_log.py` | Flash the Arduino (from the Mac), check the buds' audio, test the vision service, read `logs.json` |
| `tests/` | Unit tests (no hardware or ROS needed); `tests/hardware/` needs the real robot |
| `docs/` | `PLAN.md`, `VISION-DESIGN.md`, `EXPLORE-DESIGN.md`, `HARDWARE-BASICS.md` |

The Mac service lives in its own repo, **Vision-Microservice**.

## Hardware (summary)

- **Raspberry Pi 4B** (64-bit OS): voice, camera, head, serial to the Arduino
- **Arduino** (Uno-class): 2× DRV8871 motor drivers, quadrature encoders, serial watchdog; separate motor battery, common ground
- **Head:** Pi camera (mounted inverted), TF-Luna range sensor (UART) and two hobby servos: pan on GPIO13 (±45°), tilt on GPIO12 (-30°/+45°)
- **Bluetooth buds** for microphone and speaker
- **Mac mini** on the LAN running Vision-Microservice

Wiring, power and encoder notes are in the Electrical Schematics section below.

---

## Installation

### On a laptop (tests only)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

### On the Pi

Needs **64-bit Raspberry Pi OS Trixie** (`dpkg --print-architecture` must say `arm64`; ROS 2 Jazzy has no 32-bit build). Then one script does everything it can: the ROS 2 Jazzy repo and packages, all apt packages (listed at the top of the script), the `.venv` (created with `--system-site-packages` so it sees libcamera/picamera2/OpenCV), the pip packages, and the ROS build.

```bash
cd ~/Robot-Computer-Vision          # the checkout
./install_pi.sh                     # safe to re-run, e.g. after a git pull
```
curl -sS -o /dev/null -w "%{http_code}\n" https://rospian.github.io/rospian-repo/rospian-archive-keyring.asc
curl -sSI https://rospian.github.io/rospian-repo/rospian-archive-keyring.asc | head -5
git clone --depth 1 https://github.com/rospian/rospian-repo /tmp/rospian-repo
ls -la /tmp/rospian-repo /tmp/rospian-repo/dists 2>&1 | head -30
git -C /tmp/rospian-repo branch -a

# use local rospian
mv /tmp/rospian-repo ~/rospian-repo
ls ~/rospian-repo/public ~/rospian-repo/dists/trixie-jazzy
du -sh ~/rospian-repo/pool

Then the parts a script shouldn't do for you:
gpg --dearmor < ~/rospian-repo/public/KEYFILE | sudo tee /usr/share/keyrings/rospian-archive-keyring.gpg >/dev/null
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/rospian-archive-keyring.gpg] file:$HOME/rospian-repo trixie-jazzy main" | sudo tee /etc/apt/sources.list.d/rospian.list
sudo apt update
apt policy ros-jazzy-ros-base

sudo mv ~/rospian-repo /opt/rospian-repo
sudo chmod -R a+rX /opt/rospian-repo
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/rospian-archive-keyring.gpg] file:/opt/rospian-repo trixie-jazzy main" | sudo tee /etc/apt/sources.list.d/rospian.list
sudo apt update
apt policy ros-jazzy-ros-base
grep -c "Package: ros-jazzy-ros-base$" /opt/rospian-repo/dists/trixie-jazzy/main/binary-arm64/Packages

P=/opt/rospian-repo/dists/trixie-jazzy/main/binary-arm64/Packages
grep -c "^Package:" $P
for n in ros-core ros-base rclpy std-msgs std-srvs sensor-msgs geometry-msgs nav-msgs action-msgs rosidl-default-generators rosidl-default-runtime ament-cmake ament-cmake-python launch launch-ros launch-xml ros2cli ros2run ros2launch ros2action ros2param ros2topic ros2service ros2pkg rmw-cyclonedds-cpp tf2-ros; do printf "%-28s" $n; grep -c "^Package: ros-jazzy-$n\$" $P; done
apt-cache policy python3-colcon-common-extensions | head -3

cd ~/Robot-RPi-Core
python3 - <<'E'
import re,pathlib
p=pathlib.Path("install_pi.sh"); s=p.read_text()
s=s.replace("  ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp python3-colcon-common-extensions\n","""  ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
  ros-jazzy-nav-msgs ros-jazzy-action-msgs ros-jazzy-tf2-ros
  ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime
  ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-launch-xml
  ros-jazzy-ros2cli ros-jazzy-ros2run ros-jazzy-ros2launch ros-jazzy-ros2action ros-jazzy-ros2param
  ros-jazzy-ros2topic ros-jazzy-ros2service ros-jazzy-ros2pkg ros-jazzy-rmw-cyclonedds-cpp
""")
s=s.replace('.venv/bin/pip install -e','.venv/bin/pip install colcon-common-extensions\n.venv/bin/pip install -e',1)
p.write_text(s)
E
bash -n install_pi.sh && ./install_pi.sh


1. **Secrets:** create `/etc/robot.env`, see [Configuration](#configuration).
2. **Head and TF-Luna:** add `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` to `/boot/firmware/config.txt`; run `sudo raspi-config` → Interface → Serial Port: login shell **No**, hardware **Yes**; reboot.
3. **Groups:** `sudo usermod -aG dialout,audio,bluetooth roshan151 && sudo loginctl enable-linger roshan151` (log out and in, or reboot).
4. **Buds:** pair them, see [Bluetooth buds](#bluetooth-buds-microphone--speaker).
5. **Boot service:** check `User=` and the paths inside `robot-voice.service`, then

   ```bash
   sudo cp robot-voice.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now robot-voice
   ```

After a `git pull`, rebuild with `./install_pi.sh` (or just `cd ros2_ws && colcon build --symlink-install` in a shell that sourced `/opt/ros/jazzy/setup.bash` and `.venv/bin/activate`), then `sudo systemctl restart robot-voice`. `--symlink-install` means edits to existing Python files need only the restart; new packages, `.msg` and `.action` files need the build.

### The Mac vision service

In the Vision-Microservice folder on the Mac: `make deploy` (creates the venv, installs, runs the tests, starts it), `make status` to check. Point the Pi at it with `VISION_SERVICE_BASE_URL` (default `http://127.0.0.1:8080`, which is wrong on the Pi: set it to e.g. `http://mini.local:8080`). If the robot seems blind, check `GET /healthz`. For `/v1/ask` set `GEMINI_API_KEY` on the Mac and/or `ollama pull qwen3-vl:8b`. Endpoints and storage are in that repo's README.

### Arduino firmware

Flash from the Mac with the Arduino plugged in: `brew install arduino-cli && arduino-cli core install arduino:avr` once, then `./flash.sh` (override with `PORT=` and `FQBN=`). The boot line must say `drv8871-v5-odo`; `/odom` needs its never-reset encoder totals. Or use the Arduino IDE on `firmware/drivetrain/drivetrain.ino`. You can test upload and serial without motors connected.

---

## Configuration

Settings are environment variables (every name, default and meaning is in `robot_core/settings.py`). Under ROS, `robot.yaml` overrides the matching defaults, so calibrate in **both** places or keep the value only in the yaml.

### Secrets file (`/etc/robot.env`)

One file holds the keys for the boot service (systemd reads it as root) and for manual runs (`settings.py` reads it too). Plain `KEY=value` lines:

```bash
sudo nano /etc/robot.env
#   GEMINI_API_KEY=your-key-here
#   VISION_SERVICE_BASE_URL=http://mini.local:8080
#   BT_MAC=AA:BB:CC:DD:EE:FF          # optional: Bluetooth buds
sudo chown root:roshan151 /etc/robot.env && sudo chmod 640 /etc/robot.env
sudo systemctl restart robot-voice    # after any change
```

Lookup order, first hit wins: the real environment, `ROBOT_ENV_FILE` (a path), `./.env`, `robot_core/.env`, `/etc/robot.env`. `chmod 640` with your user's group (rather than 600) lets `./start_robot.sh` run by hand read the same file. Never commit a real key.

### Common variables

| Variable | Purpose |
|----------|--------|
| `GEMINI_API_KEY` (or `GOOGLE_API_KEY`) | Live voice, speech, and the Mac's `/v1/ask`. Either name works |
| `VISION_SERVICE_BASE_URL` | Base URL of the Mac service, e.g. `http://mini.local:8080` |
| `ROBOT_SERIAL_PORT` | Arduino port, default `/dev/ttyUSB0` (often `/dev/ttyACM0`) |
| `ROBOT_SERIAL_BAUD` | Must match the firmware, default `115200` |
| `ROBOT_TICKS_PER_CM`, `ROBOT_TICKS_PER_DEGREE` | Drive calibration, see [Calibration](#calibration) |
| `ROBOT_MAX_DRIVE_M` | Longest single drive, default `5` |
| `ROBOT_TTS` | Robot speech, on by default; `0` mutes it |
| `BT_MAC`, `ROBOT_AUDIO_INPUT_DEVICE`, `ROBOT_TTS_DEVICE` | Bluetooth buds and audio devices |
| `ROBOT_FACE_SEARCH_S`, `ROBOT_FACE_ENROLL_S`, `ROBOT_FACE_MATCH_S` | Face tasks: time to find a face when enrolling, capture time once found, and total time for a match (all default 30 s) |
| `VISION_DETECT_HZ`, `VISION_DETECT_TIMEOUT_S` | Detect rate (2.0) and hard timeout, no retry (0.6 s) |
| `ROBOT_LOG_PATH` | Robot event log, default `logs.json` |

---

## Running the robot stack

```bash
./start_robot.sh                # command mode: voice + drivetrain (what the boot service runs)
./start_robot.sh explore        # explore mode: map, find plants, photograph them
```

`start_robot.sh` sources ROS and the workspace, activates `.venv`, and launches the graph. Command mode is two processes on purpose: the voice node holds an always-open microphone and an asyncio loop, the robot process holds the serial link and the e-stop. Separate processes mean separate GILs, so a stall in the conversation cannot delay a stop.

| Want | Command |
|------|---------|
| Drivetrain only, no mic or API key | `./start_robot.sh command voice:=false` |
| Drive it by hand | `ros2 action send_goal /drive robot_interfaces/action/Drive "{meters: 0.5}" --feedback` |
| Stop it | `ros2 service call /estop std_srvs/srv/Trigger` |
| Watch the encoders | `ros2 topic echo /encoders` |
| Calibrate live | `ros2 param set /drivetrain ticks_per_cm 38.5` |
| No ROS at all | `python -m robot_core.run` |

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

- **Map:** `http://<mac>:8080/v1/slam/map.png` (plants marked in red). **Stored on the Mac only** (the Pi keeps nothing), in the Vision-Microservice folder: photos in `plants/map_{index}/plant_{NNN}/photo_{i}.png` + `bbox_{i}.txt` (a new map gets a new `map_{index}`), maps in `maps/<name>.npz` + `.png`, rewritten after every scan.
- **Guard rails** (`/explorer` in `robot.yaml`, or `--ros-args -p no_behind:=true -p explore_radius_m:=5.0`): `explore_radius_m` (default `10`, `0` = unlimited) only explores within that radius of `start_pose`; `no_behind: true` only explores ahead of the start heading. Both apply to where it drives and which plants it photographs; the 360° scans still map everything around it. When nothing inside the limits is left, the run ends as complete.
- **Re-scan later on the same map:** set `continue_map: true` and `start_pose` (where the robot stands, read off the map) under `/explorer` in `robot.yaml`.
- **Motion health:** `ros2 topic echo /motion_health` compares the command, the encoders and the camera. It reports `stalled` (wheels blocked), `slipping` (wheels turning, image static), `pushed`, or `wrong_direction`; the drivetrain brakes on stalled or slipping.
- **Park the head by hand** after a crash or power cut: `python -m robot_core.sensors.gimbal home`.

One-time setup, besides the Installation steps:

1. Flash firmware v5 (see Arduino firmware above).
2. Measure and set `head_height_m`, `pan_axis_x_m`, `hfov_deg` and the servo `*_center_us` trims under `/explorer` in `robot.yaml`. `+pan` must turn the head left and `+tilt` must look up; flip `pan_invert`/`tilt_invert` if not.
3. Restart the vision service on the Mac after updating it (DINOv2 downloads on first start).

### Voice commands: enroll face, match face, explore

In command mode, say it to the robot: *"remember my face, I'm Sam"* (`enroll_face`, about 30 s of frames — stand in front of the camera alone), *"do you know me?"* (`match_face`), *"go explore"* (`explore`, runs the explorer node against the running drivetrain until it finishes).

For face tasks the head **searches for you**: it sweeps pan ±45° at 0°, 20° and 40° up (never below level), stops on the first face, centres it, and holds while it captures, then parks. Tilt is limited to -30°/+45° everywhere; face tasks narrow it to 0°/+45°. The durations are set in [Common variables](#common-variables).

The voice session **closes for the whole task** (so you can't say "stop" until it ends), then reopens. The robot answers by gesture: **nod = it worked / the face is known, shake = it failed / the face is unknown** (for explore: nod = finished cleanly). Faces are saved on the Mac as `Vision-Microservice/faces/<name>/photo_{i}.png` + `bbox_{i}.txt`.

---

## Audio

### Bluetooth buds (microphone + speaker)

Pair once, by hand, on the Pi:

```bash
bluetoothctl
  power on
  agent on
  default-agent
  scan on                 # buds in pairing mode; note their MAC (AA:BB:CC:DD:EE:FF)
  pair AA:BB:CC:DD:EE:FF
  trust AA:BB:CC:DD:EE:FF   # so they reconnect by themselves at boot
  connect AA:BB:CC:DD:EE:FF
  exit
```

Then:

1. Put the MAC in `/etc/robot.env` as `BT_MAC=AA:BB:CC:DD:EE:FF` (used by `check_bt_audio.sh`).
2. `sudo usermod -aG bluetooth,audio roshan151`, and **`sudo loginctl enable-linger roshan151`** — without lingering, the user's audio server doesn't exist at boot and the service hears nothing. `robot-voice.service` already points at it (`XDG_RUNTIME_DIR`, `PULSE_SERVER`; `id -u` must be 1000, else edit both).
3. Reboot with the buds on and nearby. The service waits 5 s for them to connect.
4. Check: `pactl list short sources | grep bluez` (the mic) and `python -m robot_core.speech "hello there"` (the speaker). If several devices exist, set `ROBOT_AUDIO_INPUT_DEVICE` / `ROBOT_TTS_DEVICE`.
5. Use 16 kHz wideband for the mic: run `./check_bt_audio.sh`. If it says mSBC isn't offered, add `monitor.bluez.properties = { bluez5.enable-msbc = true }` in `~/.config/wireplumber/wireplumber.conf.d/51-bluez.conf`, restart `wireplumber`, reconnect the buds, and run it again.

The buds must be in headset mode (HFP) for their microphone to work, which is why step 5 matters.

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

---

## Robot images

**Top view:**  
![Top](https://github.com/user-attachments/assets/65aa1004-ad71-4d3f-be6d-fdf788f3cd46)

**Side view:**  
![Side](https://github.com/user-attachments/assets/65aa1004-c084-4d42-8a80-22c9ce910a82)

**Front view:**  
![Front](https://github.com/user-attachments/assets/65aa1004-b8d2-4fd7-ae23-dbf517e464cc)

---

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
