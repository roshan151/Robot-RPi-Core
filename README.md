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
| `robot_core/status.py`, `oled.py`, `power.py` | Status OLED: short messages (10 words max), error codes, and the Pi under-voltage watcher |
| `ros2_ws/src/` | ROS 2 nodes: `robot_interfaces` (actions, messages), `robot_drivetrain`, `robot_voice`, `robot_explore`, `robot_vision` (Mac-service client node), `robot_display` (OLED + power watcher), `robot_bringup` (launch files, `robot.yaml`) |
| `install_pi.sh`, `cleanup_pi.sh`, `README-SETUP.md`, `start_robot.sh`, `robot-voice.service` | Pi installer and its guide, the launcher, the boot service |
| `flash.sh`, `check_bt_audio.sh`, `vision_test.py`, `read_log.py` | Flash the Arduino (from the Mac), check the buds' audio, test the vision service, read `logs.json` |
| `tests/` | Unit tests (no hardware or ROS needed); `tests/hardware/` needs the real robot |
| `docs/` | `PLAN.md`, `VISION-DESIGN.md`, `EXPLORE-DESIGN.md`, `HARDWARE-BASICS.md` |

The Mac service lives in its own repo, **Vision-Microservice**.

## Hardware (summary)

- **Raspberry Pi 4B** (64-bit OS): voice, camera, head, serial to the Arduino
- **Arduino** (Uno-class): 2× DRV8871 motor drivers, quadrature encoders, serial watchdog; separate motor battery, common ground
- **Head:** Pi camera (mounted inverted), TF-Luna range sensor (UART) and two hobby servos: pan on GPIO13 (±90°), tilt on GPIO12 (±45°)
- **Bluetooth buds** for microphone and speaker
- **Mac mini** on the LAN running Vision-Microservice

Wiring, power and encoder notes are in the Electrical Schematics section below.

---

## Installation

Setting up a Pi (or the Mac vision service, the Arduino firmware, or a laptop for tests) is in **[README-SETUP.md](README-SETUP.md)**. The short version, on a fresh 64-bit Raspberry Pi OS Trixie:

```bash
git clone <your-repo-url> ~/Robot-RPi-Core && cd ~/Robot-RPi-Core
./install_pi.sh
```

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
- **Park the head by hand** after a crash or power cut: `python -m robot_core.sensors.gimbal home` (goes to the centres in `robot.yaml` and keeps holding them; `... release` lets the servos go limp).

One-time setup, besides [README-SETUP.md](README-SETUP.md):

1. Flash firmware v5 (see [README-SETUP.md](README-SETUP.md#6-the-other-two-machines)).
2. Measure and set `head_height_m`, `pan_axis_x_m`, `hfov_deg` and the servo `*_center_us` trims under `/explorer` in `robot.yaml`. `+pan` must turn the head left and `+tilt` must look up; flip `pan_invert`/`tilt_invert` if not.
3. Restart the vision service on the Mac after updating it (DINOv2 downloads on first start).

### Voice commands: look up/down/left/right, enroll face, match face, explore

*"Look up"* / *"look down"* tilt the head 30° from where it is now (or the number you say), stopping at the limits (±45°). *"Look left"* / *"look right"* pan it the same way (+pan is left, so if your head turns the wrong way flip `pan_invert` in `robot.yaml`); pan stops at its own limit, which is shorter on one side until the horn is re-fitted (see [Head servos](#head-servos-pan--tilt)). The position is remembered for the session; a task parks the head, so the next look starts from level.

In command mode, say it to the robot: *"remember my face, I'm Sam"* (`enroll_face`, about 30 s of frames — stand in front of the camera alone), *"do you know me?"* (`match_face`), *"go explore"* (`explore`, runs the explorer node against the running drivetrain until it finishes).

For face tasks the head **searches for you**: it sweeps pan ±45° at 0°, 20° and 40° up (never below level), stops on the first face, centres it, and holds while it captures, then parks. Tilt is limited to ±45° everywhere; face tasks narrow it to 0°/+45°. The durations are set in [Common variables](#common-variables).

The voice session **closes for the whole task** (so you can't say "stop" until it ends), then reopens. The robot answers by gesture: **nod = it worked / the face is known, shake = it failed / the face is unknown** (for explore: nod = finished cleanly). Faces are saved on the Mac as `Vision-Microservice/faces/<name>/photo_{i}.png` + `bbox_{i}.txt`.

### Shutting the Pi down by voice

Say *"shut down"* / *"power off"* / *"turn off"*. The robot brakes, parks the head, nods, shows `shutting down` on the OLED, and powers the Pi off cleanly after 4 seconds (so the SD card is closed properly). Say *"stop"* within those 4 seconds to cancel. It runs `sudo systemctl poweroff`, so it needs passwordless sudo for that one command; `install_pi.sh` adds it (`/etc/sudoers.d/robot-shutdown`). On an existing Pi, add it once: `echo "$USER ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff" | sudo tee /etc/sudoers.d/robot-shutdown && sudo chmod 440 /etc/sudoers.d/robot-shutdown`. After a shutdown the Pi's own power is still on: wait for the green LED to stop flashing, then cut power. To power back on, use the PiSugar button or reconnect power. A failed shutdown shows `E-X01 shutdown failed` and is logged as `power.shutdown` in `logs.json`.

### Adding a new voice tool

A tool is a function the voice model can call. All of it lives in `robot_core/live/tools.py`; the model never talks to hardware directly. Four edits, plus a test:

1. **Declare it** in `declarations()`: a name, a description that says *when* to use it (the model reads this on every turn, so keep it short and concrete), and its parameters.
   ```python
   types.FunctionDeclaration(
       name="beep",
       description="Beep the buzzer. Default 1 time; returns at once.",
       parameters=schema(times={"type": "NUMBER", "description": "How many beeps, default 1."}),
   ),
   ```
2. **Write the handler** as a method on `RobotTools`. It must be quick and must never block (the voice loop waits on it), and it returns a dict with `"ok"` plus whatever the model should know.
   ```python
   def _beep(self, times: float = 1) -> Dict[str, Any]:
       self._buzzer.beep(int(times))
       return {"ok": True}
   ```
3. **Register it** in the table inside `dispatch()`: `"beep": self._beep,` (use a lambda to pass defaults, as `look_left` does). The tool name shows on the OLED automatically, and exceptions become an `{"ok": False}` result instead of crashing the session.
4. **Tell the model in the prompt**: add a line to the rules in `robot_core/settings.py` so it knows when to use the tool (for example *"beep(): ... "*).
5. **Add a test** in `tests/test_robot_tools.py` (dispatch it, check the result), and add the name to the test that lists every tool (`test_*_tools_and_stop_takes_no_arguments`, rename its count), which will fail until you do.

If the tool needs hardware, create the object in the node and pass it in, the way `head` is: `run_live_agent(motion, run_task, head)` in `voice_node.py` hands it to `RobotTools(...)` through `robot_core/live/agent.py`.

If the job takes a long time (seconds or more, like face enroll), do not make it a tool that blocks. Add its name to `TASKS` in `tools.py` and handle it in `_run_task` in `voice_node.py`: the agent closes the voice session, runs it, and answers with a nod or a shake.

To apply a change on the Pi: `git pull` and `sudo systemctl restart robot-voice`. A rebuild is only needed for a new ROS package or message.

---

## Head servos (pan / tilt)

Two SG90 micro servos move the head. Each has three wires: **yellow = signal, red = +5 V, brown = ground**. The signals come from the Pi's two hardware-PWM pins, and the servos are powered from a separate 5 V supply, never from the Pi's 5 V pin (a moving servo draws far more than the Pi can spare and will brown the Pi out).

| Servo wire | Pan servo | Tilt servo |
|------------|-----------|------------|
| Yellow (signal) | GPIO13 (pin 33) | GPIO12 (pin 32) |
| Red (+5 V) | + of the external 5 V supply | + of the external 5 V supply |
| Brown (ground) | - of the external 5 V supply | - of the external 5 V supply |

```
 External 5 V supply          Pi header
   (+) ──────────────────────  red  of BOTH servos
   (-) ──┬───────────────────  brown of BOTH servos
         └───────────────────  any Pi GND pin (for example pin 34)   <- shared ground, required
                               pin 33 (GPIO13) ── yellow of the PAN servo
                               pin 32 (GPIO12) ── yellow of the TILT servo
```

- **The grounds must be joined.** The supply's - and a Pi GND pin have to meet, or the signal has no reference and the servos jitter or ignore it.
- **Supply size:** an SG90 draws roughly 10 mA at rest, a few hundred mA while moving, and up to about 0.6 A if it stalls against a stop. A 5 V supply of 1 A or more for the pair is sensible (typical figures, not measured on yours). A 470-1000 uF capacitor across the servo supply, close to the servos, smooths the start-up spike.
- **Hardware PWM:** the signal pins must be GPIO12 and GPIO13, which `install_pi.sh` sets up with `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` in `/boot/firmware/config.txt` (reboot after). Other pins will not work. Do not use GPIO18, which is the audio clock.
- **Power-on jerk:** a servo moves as soon as it gets power plus a signal, and the signal pins float while the Pi boots. Switch the servo supply on after the Pi is up if you can.
- **Pull each signal line down (10 kΩ from the yellow wire to GND, at the servo end).** Without it the line floats whenever the Pi is booting or halted, and a powered servo chases that noise to an end stop - this is what drives the head into its stops and tears the camera ribbon. It costs two resistors and makes "Pi not driving the pin" mean "no pulse" instead of "random pulse".
- **Sense the servo supply (recommended).** The servos have their own switch, so the Pi cannot otherwise tell whether they are powered, and it keeps "moving" an unpowered head; the moment the switch goes on the servo dashes to where the software thinks it is. Wire a spare GPIO to the servo +5 V through a divider (10 kΩ from +5 V to the pin, 20 kΩ from the pin to GND; share the ground) and set `servo_power_gpio: <BCM pin>` in `robot.yaml`. While the pin reads low the head is frozen and `look_*` answers "head servos have no power"; switching on changes nothing, and motion resumes from where the head really is. A broken sense wire reads as "off", so it fails safe. `0` (default) = not fitted, supply assumed on.
- **Switching order:** servo supply on after the Pi is up, off before the Pi is powered off. The voice `shutdown` parks the head and then stops the servo pulses itself (the head goes limp), so the Pi never halts while still driving a live pulse; cutting the servo supply afterwards is still the cleanest.
- **Mechanics:** the pan servo can only turn about 170 degrees (pulses 500-2400 us). `pan_center_us` in `robot.yaml` is the pulse where the head faces straight ahead. With the horn fitted so that is near the end of the travel (currently 2300 us), pan only reaches -90 to +9 degrees; refit the horn so straight ahead is near 1500 us to get the full +-90. Tilt is currently centred at 1750 us and inverted, and reaches +-45 degrees.

Tools: `python tests/hardware/check_head.py` (four look commands with degree variables to find a good centre and sweet spot), `python -m robot_core.sensors.gimbal home` (go to the centres and keep holding), `... release` (let the servos go limp), `... move PAN [TILT]`. The head remembers where it last rested in `~/.cache/robot-head.json` (a few bytes, overwritten) so a restart can begin from there.

## Status OLED and power dips

A 2.42" SSD1309 OLED (128x64, SPI) shows what the robot is doing. Every node publishes short messages on `/robot/status`; the `display_node` (started by both launch files) draws them and watches the Pi's supply.

### Wiring the OLED

The board's 7-pin header (labelled GND, VCC, SCK, SDA, RES, DC, CS on the back) goes to the Pi's 40-pin header like this. Power the Pi off before connecting.

| OLED pin | Pi GPIO (BCM) | Pi physical pin | What it does |
|----------|---------------|-----------------|--------------|
| GND | GND | 25 | Ground |
| VCC | 3.3 V | 1 | Power. Use 3.3 V, not 5 V, so it matches the Pi's 3.3 V signals |
| SCK | GPIO11 (SPI0 SCLK) | 23 | SPI clock |
| SDA | GPIO10 (SPI0 MOSI) | 19 | Data in. On this board "SDA" is the SPI data line, not I2C |
| RES | GPIO17 | 11 | Reset |
| DC | GPIO25 | 22 | Data / command select |
| CS | GPIO8 (SPI0 CE0) | 24 | Chip select |

```
 OLED          Pi header
 GND  ───────  pin 25  (GND)
 VCC  ───────  pin 1   (3.3 V)
 SCK  ───────  pin 23  (GPIO11)
 SDA  ───────  pin 19  (GPIO10)
 RES  ───────  pin 11  (GPIO17)
 DC   ───────  pin 22  (GPIO25)
 CS   ───────  pin 24  (GPIO8)
```

None of these pins clash with the servos (GPIO12 and 13), the TF-Luna, or the Arduino link. Keep the wires short (under about 20 cm) and, if you see noise or a blank screen, check them for loose connections first. The 2.42" boards have a mode resistor on the back: R8 fitted means SPI, R9 to R12 fitted means I2C. If the screen stays blank with correct wiring, check that yours is set to SPI.

### Power the OLED uses

Roughly 10 to 30 mA at 3.3 V (about 0.03 to 0.1 W) for a mostly dark screen of small text like this one, and up to about 60 mA if most pixels are lit. That is a rough figure for this kind of module, not measured on yours, and it is small next to the Pi itself (about 600 mA to 1 A with the camera running) and the servos. The Pi's 3.3 V pin supplies it comfortably.

To measure it on your robot, compare the PiSugar amps (`echo "get battery_i" | nc -q 1 127.0.0.1 8423`) with the OLED wired and then unwired (with the robot idle); the difference is the screen's draw.

`install_pi.sh` turns SPI on and installs the libraries (`pip install -e ".[oled]"`). Check the wiring with `python tests/hardware/check_oled.py` (stop `robot-voice` first, only one program can own the screen). After pulling this change, rebuild once for the new package: `cd ros2_ws && colcon build --symlink-install`.

The screen: a header in small print with the task, power state (`OK`, `DIP xN`), uptime and a battery icon (PiSugar, polled every 2 s; no icon if it does not answer); then rows of 21 characters: `TOOL` (the voice agent's current call); two detail lines (for explore, `plant_03 capturing 4/8`); a countdown or result (`enroll Sam 0:17`, then `Sam enrolled` or `Sam matched`). Every message is cut to 10 words.

An error shows only while it is fresh: a box of small print at the bottom with `E-xxx text` and its cause, gone 20 s after it was last reported (`ERROR_HOLD_S`). The space the text leaves free plays an animation: a face with blinking eyes, and a little box robot rolling on its treads while the drivetrain executes a move. To add one, write a painter in `robot_core/oled.py`, add a line to `ANIMS`, and call `status.anim("name", seconds)` from any node.

Under-voltage: the firmware's live flag is polled about 5 times a second. Each dip shows `PWR DIP xN`, `E-P01`, and a guess at the cause (wheels, head servo, camera load, just booted, low battery). One small file (`~/.cache/robot-lastdip.json`, overwritten, at most every 10 s) lets the next boot show `last dip: ...` even if the dip reset the Pi; it is deleted once shown.

| Code | Meaning | Code | Meaning |
|------|---------|------|---------|
| P01 / P02 | undervolt now / earlier | F01 | no face found |
| P03 / P04 / P05 | throttled / cpu freq capped / too hot | F02 / F03 / F04 | several faces / too few frames / task failed |
| S01 / S02 / S03 / S04 | arduino link / wheel stuck / e-stop / cmd timeout | N01 / N02 / N03 | drive stuck / viewpoint unreachable / explore aborted |
| C01 / C02 | camera open / capture | G01 / G02 | head unavailable / servo range limited |
| V01 / V02 | vision unreachable / timeout | L01 / L02 / L03 | voice lost / tool failed / unknown tool |
| A01 / B01 | audio error / battery low | X01 / X02 | unexpected / fatal error |

The screen is optional. With it unplugged or broken the robot runs the same, and the power watcher keeps going: every dip is written to `logs.json` as `power.dip` (cause, task, tool, battery, temp, flags, uptime) and `power.clear` (how long it lasted), plus `power.flag` (throttled, capped, hot), `power.earlier` and `power.lastdip`. Find them with `grep power logs.json`.

The full list is `CODES` in `robot_core/status.py`; `RULES` there maps log warnings to codes.

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


# debugging explore
vcgencmd get_throttled                 # anything other than 0x0 means under-voltage or throttling happened
last -x reboot | head                  # were there real reboots, and when?
journalctl -b -1 -e --no-pager | tail -40   # last lines of the previous boot, including the explorer's own output


## Debug camera

vcgencmd get_throttled
rpicam-hello -t 5000 --nopreview                 # camera alone
python tests/hardware/check_camera.py            # camera alone, through our code
python tests/hardware/check_camera.py head       # opens the head, then moves it

2. Check camera detection
   rpicam-hello --list-cameras
   dmesg | grep -i -E "ov5647|unicam|csi|i2c" | tail -20
   vcgencmd get_throttled

3. Did the rpicam package change
grep -E "Start-Date|Commandline|Upgrade:" /var/log/apt/history.log | tail -30
dpkg -l | grep -E "libcamera|rpicam|picamera2|raspi-firmware|linux-image" 
uname -r

4. 
rpicam-hello --list-cameras
dmesg | grep -i -E "ov5647|unicam|csi" | tail -20

5. source .venv/bin/activate
python -c "import picamera2; print(picamera2.__file__)"

## Debug voltage display

ls -l /tmp/*.sock
systemctl status pisugar-server --no-pager
python3 -c "import socket;s=socket.socket(socket.AF_UNIX);s.connect('/tmp/pisugar-server.sock');s.sendall(b'get battery\n');print(s.recv(100))"

## Refresh build

git checkout oled && git pull
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
pip install -e ".[oled]"
cd ros2_ws && colcon build --symlink-install && source install/setup.bash && cd ..


## Face enroll issue

rpicam-hello --list-cameras          # which sensor is this?

cd ~/Robot-RPi-Core && python3 -c "
from robot_core.sensors.camera import Camera, jpeg
from robot_core.vision_client import VisionClient
import cv2
c = Camera(main_size=(640, 480)); v = VisionClient()
for i in range(15):
    f = c.capture(); r = v.face_embed(jpeg(f.main), i)
    print(i, 'no face' if not r else f\"q={r['quality']:.2f} n={r['face_count']} bbox={[int(x) for x in r['face_bbox']]}\")
cv2.imwrite('/tmp/cam_check.jpg', f.main); c.close()
"