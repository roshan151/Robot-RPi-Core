# Robot-Computer-Vision → ROS 2 — the plan

> **Amended by `EXPLORE-DESIGN.md`** for the final hardware: no LD14P lidar (the TF-Luna on the head is the scanner), TF-Luna on the Pi UART instead of the Arduino, SLAM on the Mac instead of `slam_toolbox` on the Pi, pan limited to ±45°.

The single source of truth for this rebuild. Supersedes `ROS2-MIGRATION.md` and
`COMPUTE-BUDGET.md`, both of which contained decisions that have since changed.

Companion documents:

- **`HARDWARE-BASICS.md`** — from-first-principles explanations of the GIL,
  latency percentiles, I²C/SPI/UART/PWM, servos, and camera streams. A tutorial,
  not a plan; it stays separate.
- **`VISION-DESIGN.md`** — the vision/perception layer. It **amends decisions #5
  and #8 below** and revises standing rule #3; those changes are folded in here,
  with the reasoning kept there next to the design that forced it.

**Contents**

- [Part 0 — Decisions of record](#part-0--decisions-of-record)
- [Part 1 — ROS 2 in terms you already know](#part-1--ros-2-in-terms-you-already-know)
- [Part 2 — What you can skip, and when that changes](#part-2--what-you-can-skip-and-when-that-changes)
- [Part 3 — Environment setup](#part-3--environment-setup)
- [Part 4 — Architecture](#part-4--architecture)
- [Part 5 — Hardware map](#part-5--hardware-map)
- [Part 6 — Compute and memory budget](#part-6--compute-and-memory-budget)
- [Part 7 — The phases](#part-7--the-phases)
- [Part 8 — Standing rules](#part-8--standing-rules)
- [Part 9 — Things that will bite you](#part-9--things-that-will-bite-you)
- [Part 10 — CLI cheat sheet](#part-10--cli-cheat-sheet)

---

## Part 0 — Decisions of record

Every settled decision, with the reasoning compressed. Where a decision reversed
an earlier one, that's noted — those reversals are the reason the old documents
were retired.

| # | Decision | Why |
|---|---|---|
| 1 | **ROS 2 Jazzy** | Tier 1 on arm64, LTS to May 2029, everything is a prebuilt deb. Lyrical Luth is the upgrade target in ~a year. |
| 2 | **Stay on Raspberry Pi OS Trixie**, add the `rospian` apt repo for Jazzy | *Reverses an earlier recommendation to move to Ubuntu 24.04.* Ubuntu idles ~150 MB heavier than Pi OS Lite — more than the entire ROS multi-process overhead this plan spends pages justifying — and picamera2, `dtoverlay`, and the hardware JPEG encoder are first-class only on the Pi Foundation kernel. Your peripheral plan is deeply Pi-specific. Ubuntu's one advantage was Tier-1 ROS packaging; that's worth less than a working camera stack. |
| 3 | `ros-jazzy-ros-base`, never `-desktop` | ~2 GB and the whole GL stack saved. RViz runs on your laptop. |
| 4 | **ROS is transport, not rewrite** — hardware logic stays in plain Python | Keeps tests runnable without ROS; makes the migration reversible. |
| 5 | **Camera, servos, e-ink, lidar → Pi** | *Reverses an earlier recommendation to put the servo on the Arduino.* Keeps the firmware completely unchanged, which removes a whole risk class and lets you iterate in Python instead of re-flashing. *Amended by `VISION-DESIGN.md`: **two** servos, pan (GPIO13) and tilt (GPIO12). Turning the whole base to look around is slow and, mid-voice-session, intrusive; `dtoverlay=pwm-2chan` already provides both channels.* |
| 6 | **TF-Luna → Arduino, tilted forward-down, cliff detection only** | *Reverses an earlier plan to use it for forward obstacle stopping.* A fall is instantaneous and unrecoverable; it deserves a firmware reflex that survives Python stalling, wifi dropping and ROS restarting. |
| 7 | **Horizontal obstacle sensing → LD14P lidar, on the Pi** | One sensor, 360°, and it feeds mapping too. |
| 8 | **Local camera CV: tracking only** | *Resolves the open question from the retired budget doc, then is **partially reversed** by `VISION-DESIGN.md`.* The lidar still handles all obstacle work, and no detector, segmenter or ML model ever runs on the Pi. But REST answers at 1-2 Hz with a 100-500 ms lag, and between answers there is no new visual information at all — so a gimbal or follow behavior driven by detections alone is extrapolating from a measurement that is already 0.5-1.5 s stale. One bounded exception: **sparse optical-flow tracking on the 320×240 `lores` stream, ~2-5 ms/frame at 15 Hz**, inside `perception_node`. See `VISION-DESIGN.md` Part 4.2. |
| 9 | **Face recognition and scene understanding → remote REST** | Keeps the Pi free. Latency makes it advisory-only, never a stop path. |
| 10 | **Two modes: scan and command, never simultaneous** | SLAM's loop-closure bursts are unpredictable and CPU-spiky. Putting them in the mode with no real-time deadline means they never have to coexist with the audio session. |
| 11 | **Command mode runs no SLAM, ever** | Obstacle avoidance needs `/scan`, not a map. Localization is only needed for "go to the kitchen," which isn't on the roadmap yet. *Reinforced by `VISION-DESIGN.md`: approach planning runs in the **odom frame only**, bounded at 3 m — beyond that it drives partway and re-plans. Odometry drift over 3 m is small next to a 1.5 m standoff radius, so the bound is what keeps SLAM out of command mode.* |
| 12 | **Arduino firmware changes only once**, for the cliff reflex | Everything else stays as-is. The framed protocol with checksums is a better interface than micro-ROS would give you, and micro-ROS needs a 32-bit MCU anyway. |

### The principle behind #5 and #6

They look contradictory — *"put things on the Pi"* and then *"put this on the
Arduino."* The line is:

> **A reflex goes in firmware. A policy goes on the Pi.**

A reflex is one number, one threshold, one action, no judgment. Cliff detection
is the purest example: no floor → stop. A policy is which way to go, whether that
shape is worth avoiding, how to route around it — things you want to iterate on
in Python without re-flashing.

---

## Part 1 — ROS 2 in terms you already know

| ROS 2 | What it is | Your analogue |
|---|---|---|
| **Node** | A unit of functionality | A service in your system |
| **Topic** | Named pub/sub stream, fire-and-forget, many-to-many | An event stream |
| **Service** | Request → response RPC. Should return fast | A synchronous function call |
| **Action** | Goal → feedback → result, cancellable. For slow things | **An async tool call** |
| **`.msg`/`.srv`/`.action`** | Typed schema files compiled to Python classes | Your tool JSON schemas, statically checked |
| **Parameter** | Named config on a node, settable at launch or live | Env var, but introspectable |
| **Launch file** | Python script describing which nodes to start | docker-compose |
| **Executor** | The loop dispatching callbacks. Blocking one blocks all | Your asyncio event loop, same hazard |
| **Callback group** | Which callbacks may run concurrently | Lanes within that loop |
| **QoS** | Delivery semantics: reliable vs best-effort, queue depth | at-least-once vs at-most-once |
| **`colcon`** | The build tool over a workspace of packages | Monorepo build system |
| **DDS / RMW** | The pub/sub transport underneath. Auto-discovers peers | The bit that bites you on wifi |
| **`ros2` CLI** | Poke a live system from a shell | Your debugging REPL |

**The model that matters:** an action *is* a tool call. Named typed arguments,
long-running, cancellable, emits progress, terminates with a status. Your `drive`
tool and a ROS action are the same object described twice.

**The hazard that matters:** the executor is an event loop, and everything you
know about not blocking one applies unchanged.

---

## Part 2 — What you can skip, and when that changes

| Skip | Until |
|---|---|
| **Gazebo / simulation** | Never — you have hardware in the room |
| **ros2_control** | Never — your Arduino *is* the controller |
| **rclcpp / C++** | Never — you're a Python shop |
| **URDF / robot_state_publisher** | Optional even at Phase 7; a static transform is enough |
| **tf2 / transforms** | **Phase 6.** SLAM is what makes it mandatory |
| **nav2** | **Phase 8, maybe never.** The agent-driven explorer is lighter and fits better |

For Phases 0–5 you need only: nodes, topics, services, actions, parameters,
launch files, executors, callback groups. That's about two days of reading, and
most of it lands by doing Phase 1.

---

## Part 3 — Environment setup

### Install (Raspberry Pi OS Trixie, arm64)

You stay where you are — no reflash. ROS 2 has no official Debian Trixie
binaries (Trixie is Tier 3), so use the community `rospian` buildfarm, which
ships native Jazzy debs for Raspberry Pi OS Trixie arm64. Add its apt repository
per the instructions at `github.com/rospian/rospian-repo`, then:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base \
                    ros-jazzy-rmw-cyclonedds-cpp \
                    python3-colcon-common-extensions \
                    python3-rosdep
sudo rosdep init && rosdep update
```

**Check coverage before you commit.** The list you actually need is short:
`ros-base`, `rmw-cyclonedds` (now), `tf2` tools (Phase 6), `slam-toolbox` and
`nav2-map-server` (Phase 7), `vision_msgs` (Phase 4). Confirm those exist in the
repo on day one. The LD14P driver is a source build regardless — clone
`ldlidar_ros2` into `ros2_ws/src` and colcon it; a few minutes of C++.

**Mitigate the single-maintainer risk.** Once it works, mirror the debs you
installed (`apt-get download` the set, keep them in the repo or on a USB stick).
If rospian goes stale you're *frozen, not broken* — and you can rebuild from
source whenever you feel like it.

**Fallbacks, in order.** A missing package → build that one from source in your
workspace. The whole repo dying → ROS in Docker on Pi OS is viable *for you
specifically*; the usual Docker-on-Pi complaints are about OpenGL and Gazebo,
neither of which you use. Reflashing to Ubuntu stays available as the last
resort, not the first move.

### Shell environment

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robot-Computer-Vision/ros2_ws/install/setup.bash 2>/dev/null

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # not optional — see Part 9
export ROS_DOMAIN_ID=42
```

### Sanity check

```bash
ros2 run demo_nodes_py talker      # terminal 1
ros2 topic echo /chatter           # terminal 2
```

If terminal 2 prints, you have working ROS 2. Fix this before writing any of
your own code.

---

## Part 4 — Architecture

### 4.1 The two-layer rule

```
robot_core/       Layer A — plain Python. ZERO ROS imports. pip-installable.
ros2_ws/src/      Layer B — thin nodes that import Layer A. No hardware logic.
```

Everything hard already lives in Layer A: the framed serial protocol, the
out-of-band e-stop, the FIFO motion worker, encoder verification, the TTS cache.
None of it should learn what a node is.

Three consequences, and they are the whole argument:

1. `tests/` keeps running on your laptop with no ROS installed.
2. Lightweight is enforced structurally — a node that grew logic would have to
   import Layer A anyway, so DDS stays a pipe carrying ~1 msg/sec.
3. It's reversible. Delete Layer B, you still have a robot.

**CI check:** `grep -r "import rclpy" robot_core/` must return nothing.

### 4.2 Where the agent sits

Your LLM agent is **not** a node in the dataflow graph; it's a *client* of the
graph. It sits where a human with a joystick would — a teleop source that speaks
English. Outside the control stack, free to crash and restart, never in the
safety path.

```
reflex         Arduino firmware      ~1 kHz       PID, watchdog, cliff stop
reactive       lidar → obstacle      6–10 Hz      deterministic, holds the veto
deliberative   LLM agent             0.1–1 Hz     intent, advisory only
```

Each layer may veto the one above, never the reverse. Your `MAX_DRIVE_METERS`
clamp has the right instinct — *"a limit the model can talk itself out of is not
a limit"* — and in ROS it moves from the client into the drivetrain node's
**goal-rejection callback**, because enforcement belongs with the actuator.

### 4.3 The node graph

```
                    ┌─────────────── COMMAND MODE ───────────────┐

voice_node ──action /drive, /turn──►  drivetrain_node ──serial──► Arduino
     │     ──srv    /estop ────────►        │                      │
     │     ──srv    /say ──► speech_node    ├──► /encoders         └─ TF-Luna
     │     ──srv    /tilt ─► servo_node     │                         (cliff reflex)
     │     ──topic  /emotion ► eink_node    │
     │                                      │
obstacle_node ──srv /estop ─────────────────┘
     ▲
     └── /scan ── lidar_node          perception_node ──► /detections
                                      battery_node ────► /battery_state

                    ┌──────────────── SCAN MODE ─────────────────┐

lidar_node ──/scan──► slam_toolbox ──► /map ──► explorer_node ──► drivetrain_node
                          ▲                                            │
                     /odom, tf ─────────────────────────────────────────┘
                                          eink_node shows "MAPPING"
```

Shared between modes: `drivetrain_node`, `lidar_node`, `eink_node`,
`battery_node`. Command-only: `voice_node`, `perception_node`, `speech_node`,
`servo_node`, `obstacle_node`. Scan-only: `slam_toolbox`, `explorer_node`.

**Process layout** — three processes in command mode, and the rule is absolute:
**the audio process and anything slow must never be the same process.**

| Process | Contains | Why separate |
|---|---|---|
| `robot` | drivetrain, lidar relay, obstacle, e-ink, servo, battery, speech, **gimbal, follow, approach planner, vision behavior** | All light. Nothing here can stall the stop |
| `perception` | camera capture, REST calls, **the local tracker** | Network calls hang for seconds |
| `voice` | the Gemini Live session | Needs steady timing; biggest CPU user |

This split is what decides the vision layer's node graph, not the other way
round. Anything that touches pixels must live in `perception` (rule #1 below);
anything that needs a target at camera rate must **not**, because a REST call
there will hang for seconds. `perception_node` therefore owns the tracker and
publishes a tiny `/target_bearing`, and `gimbal_node` consumes it from `robot`.
Inside `perception`, the REST call runs on its own thread so it cannot stall the
tracker loop — the Part 9 #3 hazard, re-entered through a different door.

### 4.4 Target layout

```
Robot-Computer-Vision/
├── firmware/drivetrain/drivetrain.ino   # + cliff reflex (Phase F). Otherwise unchanged
│
├── robot_core/                          # Layer A — pip install -e .
│   ├── pyproject.toml
│   ├── settings.py                      # was config.py, minus tunables
│   ├── log.py                           # was robot_log.py
│   ├── drivetrain/
│   │   ├── serial_protocol.py           # unchanged
│   │   ├── arduino_bridge.py            # unchanged
│   │   └── client.py                    # was drivetrain_client.py
│   ├── motion_executor.py               # one change, Phase 1
│   ├── gestures.py                      # step table only
│   ├── speech.py                        # was tts.py
│   ├── battery.py                       # unchanged
│   ├── camera.py                        # was vision_client.py, BytesIO rewrite
│   └── live/agent.py                    # was live_agent.py, deps injected
│
├── ros2_ws/src/
│   ├── robot_interfaces/
│   ├── robot_drivetrain/
│   ├── robot_voice/
│   ├── robot_safety/                    # obstacle_node
│   ├── robot_sensors/                   # lidar relay, battery, perception
│   ├── robot_hmi/                       # eink_node, servo_node
│   └── robot_bringup/                   # launch/, config/, mode manager
│
├── maps/                                # build artifacts from scan mode
├── tests/                               # tests robot_core only
└── docs/
```

### 4.5 What gets deleted

| File | Fate |
|---|---|
| `movement_context.py` | **Delete.** `MotionExecutor.status()["moving"]` already carries this |
| `movement_adapter.py` | **Delete.** Pass-through; fold defaults into `drivetrain/client.py` |
| `coordinator.py` | Becomes `obstacle_node.py`. `RobotCoordinator` disappears |
| `run_robot.py` | Becomes launch files + a mode manager |

### 4.6 What `config.py` becomes

- **`robot_core/settings.py`** — constants, env fallbacks, `require()`,
  `SECRET_NAMES`, sequence ranges. Layer A still runs standalone.
- **`robot_bringup/config/robot.yaml`** — the tunables: `ticks_per_cm`,
  `ticks_per_degree`, `default_speed_pct`, `move_timeout_s`, `max_drive_m`,
  `min_obstacle_m`, model names, system prompt.

Calibration becomes `ros2 param set`, live, no restart.

---

## Part 5 — Hardware map

### What lives where

| Component | Host | Connection | Job |
|---|---|---|---|
| Motors, encoders | Arduino | existing | PID, closed-loop moves |
| **TF-Luna** | **Arduino** | I²C, pins A4/A5 | **Cliff reflex only.** Tilted forward-down |
| LD14P lidar | Pi | USB-TTL adapter, 230400 baud | 360° horizontal obstacles + mapping |
| Camera | Pi | CSI ribbon | Frames → REST |
| Servo (camera tilt) | Pi | hardware PWM, GPIO12 | Aim camera + TF-Luna |
| Servo (camera pan) | Pi | hardware PWM, GPIO13 | Track a target without turning the base |
| E-ink | Pi | SPI0 | Emotion + mode indication |
| PiSugar | Pi | I²C (GPIO 2/3) | Battery |
| Arduino | Pi | USB | Serial link |

### Pi pin budget

```
Function          Pins
────────────────────────────────────────────────────────
E-ink (SPI0)      GPIO 8 (CE0), 9 (MISO), 10 (MOSI), 11 (SCLK)
                  + GPIO 25 (DC), 17 (RST), 24 (BUSY)
PiSugar (I2C1)    GPIO 2 (SDA), 3 (SCL)
Servo tilt (PWM0) GPIO 12          — needs dtoverlay=pwm-2chan
Servo pan  (PWM1) GPIO 13          — same overlay, second channel
Camera            CSI ribbon
Lidar             USB (via USB-TTL adapter)
Arduino           USB
```

No collisions; ~15 GPIO pins spare. **Avoid GPIO18** — it's the I²S audio clock
and you do audio.

### Hardware rules

**Servo: hardware PWM only.** Add `dtoverlay=pwm-2chan` to
`/boot/firmware/config.txt`, drive GPIO12 via `rpi-hardware-pwm`. Software PWM
twitches visibly while competing with the PortAudio callback. Use `gpiozero` with
the `lgpio` backend for ordinary GPIO; `RPi.GPIO` is not the current path.

**Power servos and lidar from the buck converter, not the Pi.** Two servos under
load and the lidar's 1 A spin-up surge will brown out a PiSugar-powered Pi
mid-write and corrupt the SD card. Separate 5 V rail, ground shared with the Pi —
the same star-ground discipline already in your README.

**Limit pan travel and give the CSI ribbon a service loop.** The camera ribbon
does not tolerate repeated twisting. Clamp pan to ±90° in `servo_node` (rule #4:
in the node, not the caller) and route the ribbon with slack before the head ever
moves under software control.

**udev rules before the second USB serial device.** Arduino and lidar will both
be `/dev/ttyUSB*` and the numbering can swap on reboot. Name them by USB serial
number (`/dev/arduino`, `/dev/lidar`) **before** you plug the lidar in. Otherwise
one morning the robot sends drive commands to the lidar.

**TF-Luna mounting geometry.** Beam must land **10–15 cm ahead of the front
wheels** — straight down means you detect the edge while already on it. Mount at
**≥25 cm height**: TF-Luna won't read below ~20 cm, so a low mount puts the floor
in its blind zone. Trigger on `expected_floor_distance + 5 cm`, or on no return.
One beam is one point — a stair edge met at an angle can be crossed by a wheel
before the beam. Two sensors, one per front corner, is the robust version.

**Lidar height is a genuine trade.** A 2D lidar is one horizontal slice. Mounted
at the top, anything it hits is something the robot's tallest point would hit —
which is the clearance property you want. What you lose is everything below the
plane: shoes, cables, thresholds, low furniture bases. Your map will show clear
floor where a low obstacle sits. For a ~25 cm robot that's tolerable. Also: lidar
cannot see glass or mirrors, ever.

**LD14P facts that shape the design:** 0.1–8 m range (6 m on dark surfaces),
**6 Hz default scan** — a fresh scan only every 167 ms, so it is a *mapping and
avoidance* sensor, not a stopping sensor. 2.2K-hour service life (~92 days
continuous) — build in an off switch, which the mode split gives you for free.

---

## Part 6 — Compute and memory budget

### CPU

| Load | Cost (1 core = 100%) |
|---|---|
| Gemini Live audio | 5–10 % |
| Camera → REST @ 2 Hz | 5–8 % (down from 8–15 %: smaller payloads, binary not base64) |
| Local LK tracker @ 15 Hz on 320×240 | 5–10 % (OpenCV, releases the GIL) |
| Lidar driver + `/scan` | 3–5 % |
| Obstacle policy from `/scan` | < 2 % |
| Drivetrain serial | 2–3 % |
| E-ink, servos, battery | < 1 % |
| Gimbal control loop @ 20 Hz | < 1 % |
| DDS transport at these rates | 1–3 % |
| **slam_toolbox (mapping)** | **30–60 %, spiky** |

Command mode sits around 30–45% of one core out of four, with the vision layer
fully built. Scan mode is heavier but has no real-time deadline — which is
exactly why the modes are split.

The tracker is the only new CPU cost on the roadmap and it is the price of
decision #8's amendment. It is bounded deliberately: 320×240, 15 Hz not 30, and
sparse optical flow rather than a dense method or a correlation tracker. If it
ever shows up in a profile, the knobs in order are rate, then resolution, then
feature count — **not** a rewrite in another language. See the note at the end of
Part 9.

### Memory

RSS estimates, ±30%. Measure with `ps -o rss=,comm= -C python3`.

| Process | Contents | RSS |
|---|---|---|
| `robot` | rclpy + drivetrain + e-ink (PIL) + servo + obstacle + battery | ~80 MB |
| `lidar` | ldlidar driver (C++) | ~35 MB |
| `perception` | rclpy + picamera2 + requests | ~130 MB |
| `voice` | rclpy + genai + sounddevice | ~100 MB |
| `slam_toolbox` | C++, mapping mode, home-scale map | 150–250 MB |

| Mode | Running | Total |
|---|---|---|
| **Command** | robot + lidar + perception + voice | **~345 MB** |
| **Scan** | robot + lidar + slam_toolbox + explorer | **~265–365 MB** |
| *(all at once — avoided by the mode split)* | | *~545 MB* |

| Pi 4B variant | Peak as % of RAM | Verdict |
|---|---|---|
| 8 GB / 4 GB | 4–9 % | irrelevant |
| 2 GB | ~18 % | comfortable |
| 1 GB | ~35 % | works, but collapse to two processes |

Check yours with `free -h`.

**The OS baseline is part of this budget**, and it's why decision #2 reversed.
Raspberry Pi OS Lite idles at ~100–150 MB; Ubuntu Server idles at ~250–300 MB,
because it carries snapd, cloud-init and unattended-upgrades. That ~150 MB gap is
larger than the ~120 MB this plan spends on running ROS as three processes
instead of one. Choosing Ubuntu would have handed back more than the entire
architecture budget, in exchange for nothing the robot can feel.

**What the mode split actually buys.** Not mainly RAM. slam_toolbox's cost is
*spiky* — most of the time it cheaply matches consecutive scans, then recognises
a loop and re-optimises the whole pose graph in a burst that can saturate a core
for hundreds of milliseconds. That burst arriving during a live audio session is
precisely what kills your websocket. The split means the unpredictable workload
lives in the mode with no latency deadline, so the two never have to be tuned
against each other. It also makes the map a **build artifact** — a file written
by one mode and read by the other — rather than a live dependency.

---

## Part 7 — The phases

Each phase ends with a robot that drives.

### Phase F — cliff reflex *(independent; do it whenever the hardware arrives)*

Doesn't block on anything and isn't blocked by anything. It's the only
safety-critical item on the list, and it's the one firmware change in the plan.

- Mount TF-Luna per the geometry rules in Part 5.
- Wire to Arduino I²C (A4/A5 — free in your pinout, and unlike a servo there's
  **no timer conflict**; I²C uses separate hardware from the motor PWM).
- ~15 lines: poll at 20 Hz, and if range exceeds `floor + 5 cm` (or returns
  nothing) while a forward move is running, call the existing `finishMove("STOP")`.
- Report range up the serial link as a new frame type so the Pi can see it.
- Watch SRAM — you have 2 KB and the Wire library takes buffers.

**Done when:** the robot stops at a table edge with the Pi unplugged.

### Phase 0 — repo surgery + camera fix *(an evening, no ROS)*

Does most of the real work. If you stopped here you'd still be ahead.

1. `git mv sketches/drivetrain firmware/drivetrain`, `documentation/` → `docs/`.
2. Move hardware modules into `robot_core/` per §4.4.
3. `live_agent.py` → `robot_core/live/agent.py`; change `run_live_agent(move)` to
   take its motion interface as a parameter. **This is the seam Phase 2 needs.**
4. **Delete** `movement_context.py` and `movement_adapter.py`.
5. Rewrite `vision_client.py` → `robot_core/camera.py`: capture to `io.BytesIO`,
   picamera2's hardware JPEG encoder, two streams (`main` 1280×720 for REST,
   `lores` 320×240 available), **stop writing files to the SD card.**
6. `pyproject.toml`, `pip install -e .`, fix imports, `pytest tests/`.

```toml
[project]
name = "robot-core"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["pyserial>=3.5", "requests>=2.28", "python-dotenv>=1.0"]

[project.optional-dependencies]
voice = ["google-genai>=1.0.0", "sounddevice>=0.4.6"]
camera = ["picamera2"]
```

**Done when:** `pytest` passes, the robot drives via the old entrypoint, and
`grep -r "import rclpy" robot_core/` returns nothing.

### Phase 1 — interfaces + drivetrain node *(a weekend)*

```bash
ros2 pkg create --build-type ament_cmake  robot_interfaces
ros2 pkg create --build-type ament_python robot_drivetrain
```

Interfaces packages **must** be `ament_cmake`. Everything else `ament_python`.

`Drive.action`:

```
float64 meters              # negative reverses
bool    gesture false       # gesture goals are REJECTED when busy, not queued
float64 speed_pct 0.0       # 0 = node default
---
string  status              # done | cancelled | failed
int32   ticks_left
int32   ticks_right
float64 duration_s
---
int32   ticks_left
int32   ticks_right
```

`Turn.action` is identical with `float64 degrees` (positive = right).
Reuse `std_srvs/Trigger` for `/estop`. Add `Say.srv` and `Tilt.srv` in later
phases — define the minimum, reuse the rest.

**One change to `MotionExecutor`.** Its shared event queue can't let an action
goal await its own result. Add per-job slots:

```python
# __init__
self._waiters: Dict[int, threading.Event] = {}
self._results: Dict[int, MotionEvent] = {}

def wait_for(self, job_id: int, timeout: float) -> Optional[MotionEvent]:
    ev = self._waiters.setdefault(job_id, threading.Event())
    if not ev.wait(timeout):
        return None
    return self._results.pop(job_id, None)

# end of _publish(), keeping the existing queue put:
with self._lock:
    self._results[event.job_id] = event
    self._waiters.setdefault(event.job_id, threading.Event()).set()
```

Keep the shared queue — Phase 2 uses it to push results to the model.

**The node.** Two callback lanes, and the separation is load-bearing:

```python
moves  = MutuallyExclusiveCallbackGroup()
safety = ReentrantCallbackGroup()

self._drive_srv = ActionServer(
    self, Drive, "drive",
    goal_callback=self._accept_drive,
    cancel_callback=lambda _gh: CancelResponse.ACCEPT,
    execute_callback=self._run_drive,
    callback_group=moves,
)
self.create_service(Trigger, "estop", self._on_estop, callback_group=safety)

def _accept_drive(self, goal):
    limit = self.get_parameter("max_drive_m").value
    if abs(goal.meters) > limit:
        return GoalResponse.REJECT          # enforcement lives HERE, not in the agent
    if goal.gesture and self._exec.status()["moving"]:
        return GoalResponse.REJECT          # replaces Gesturer's suppression race
    return GoalResponse.ACCEPT
```

`_run_drive` submits to `MotionExecutor`, polls `wait_for()` in 0.1 s slices
checking `goal_handle.is_cancel_requested`, and maps the terminal status onto
`succeed()` / `canceled()` / `abort()`. `_on_estop` calls `cancel_all()`, which
already writes out of band past the command lock.

**Build and verify:**

```bash
colcon build --symlink-install    # only robot_interfaces actually compiles
source install/setup.bash
ros2 action send_goal /drive robot_interfaces/action/Drive "{meters: 0.5}" --feedback
ros2 service call /estop std_srvs/srv/Trigger
```

**Done when:** the robot drives from the CLI, feedback shows ticks, and `/estop`
halts a move **already in progress**. Test that deliberately.

### Phase 2 — voice node *(a weekend; the interesting one)*

`robot_tools.py` survives almost intact — only the injected dependency changes.
`_drive` swaps `self._exec.submit(...)` for `bridge.drive(...)`. The
non-blocking contract its docstring is about is preserved: `send_goal_async()`
returns immediately.

**The bridge.** Two event loops, one process. rclpy in a daemon thread, asyncio
on main, one crossing point:

```python
class RosBridge:
    """Called from asyncio tool handlers. Nothing here ever blocks."""

    def __init__(self, node, loop):
        self._node, self._loop = node, loop
        self._drive = ActionClient(node, Drive, "drive")
        self._estop = node.create_client(Trigger, "estop")
        self.results: asyncio.Queue = asyncio.Queue()

    def drive(self, meters, gesture=False) -> dict:
        fut = self._drive.send_goal_async(Drive.Goal(meters=float(meters),
                                                     gesture=gesture))
        fut.add_done_callback(lambda f: self._on_accepted(f, "drive", meters))
        return {"ok": True, "queued": True, "meters": meters}

    def stop(self) -> dict:
        self._estop.call_async(Trigger.Request())      # fire, don't await
        return {"ok": True, "stopped": True}

    def _push(self, evt: dict):
        """rclpy thread -> asyncio loop. The ONLY crossing point."""
        self._loop.call_soon_threadsafe(self.results.put_nowait, evt)
```

**Never** call `spin_until_future_complete` from inside the asyncio loop — that's
the `audio.error stage=mic-queue` → `voice.drop` cascade reintroduced through a
different door.

**The third pump — closing the loop.** Today you publish `MotionEvent`s and
nothing consumes them: the model queues a 3 m drive, the guardian halts it at
0.4 m, and the model still thinks it's going. Polling won't fix it. Push:

```python
async def _pump_results(self, session, bridge):
    while True:
        evt = await bridge.results.get()
        await session.send_client_content(
            turns={"role": "user",
                   "parts": [{"text": f"[robot] {evt['op']} {evt['value']}: {evt['status']}"}]},
            turn_complete=False,
        )
```

Add it to the `asyncio.wait(..., FIRST_COMPLETED)` set with the other two.
*Verify the content-injection call against your installed `google-genai` — that
API has moved between releases.*

This turns the agent from open-loop into closed-loop, and it's the single biggest
capability gain in the migration.

**Done when:** you say "drive forward half a metre," it does, and the model is
told when it finished.

### Phase 3 — remaining core nodes *(an evening each)* — **migration complete**

| Node | Wraps | Interface |
|---|---|---|
| `battery_node` | `robot_core.battery` | `sensor_msgs/BatteryState` @ 0.2 Hz |
| `speech_node` | `robot_core.speech` | service `/say` |
| `bringup` | — | all light nodes, one process, one `MultiThreadedExecutor` |

**Gestures get no node and no action.** `gestures.py` stays a step table;
`voice_node` expands `"yes"` into two `/drive` goals with `gesture=True`. The
drivetrain's goal rejection replaces `Gesturer`'s check-then-act race.

```python
def main():
    rclpy.init()
    nodes = [DrivetrainNode(), BatteryNode(), SpeechNode()]
    ex = MultiThreadedExecutor(num_threads=6)
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
```

`voice_node` stays a separate process with `respawn=True` — restartable without
taking the drivetrain down, which replaces part of your `_run_supervised` loop.

Retire `run_robot.py`; point `robot-voice.service` at the launch file.

**Done when:** `ros2 launch robot_bringup command.launch.py` gives you everything
`run_robot.py` used to, plus a battery topic.

---

*Everything above is the migration. Everything below is expansion.*

---

### Phase 4 — peripherals *(an evening each)*

| Node | Does | Notes |
|---|---|---|
| `servo_node` | services `/tilt` and `/pan` (degrees) | Hardware PWM GPIO12 (tilt) and GPIO13 (pan). Clamp both ranges in the node, not the caller. Publishes the `base_link → gimbal_pan → gimbal_tilt` transforms at 20 Hz |
| `eink_node` | subscribes `/emotion`, `/mode` | **QoS `KEEP_LAST, depth=1`** — drop stale emotions rather than queueing |
| `perception_node` | camera → REST → `/detections` | Wraps the `robot_core.camera` you already fixed in Phase 0. **This is only the detect-and-publish skeleton** — the backpressure policy, the local tracker and the gimbal loop are `VISION-DESIGN.md` phases V1–V3 |

The e-ink is slow: 2–15 s full refresh, 0.3–1 s partial. Depth-1 QoS is the
declarative way to say "only the newest matters" — a face thirty seconds behind
the conversation is worse than no face. Rate-limit refreshes to avoid ghosting.

**Note this changes your output design.** The robot currently answers by moving
because it has no other channel, which is why "absence of motion is ambiguous
between thinking, no, and dead." A screen resolves that; gestures become flourish
rather than the entire vocabulary.

Add the servo tool to the agent — it can aim its own camera, which is a genuinely
new capability rather than a peripheral.

### Phase 5 — lidar + reactive obstacle avoidance *(a weekend)*

1. udev rules for `/dev/arduino` and `/dev/lidar` **first**.
2. Clone `ldlidar_ros2` into `ros2_ws/src`, colcon build, launch with the port
   and 230400 baud. Verify `ros2 topic hz /scan` — expect ~6 Hz.
3. Look at it in RViz **on your laptop**, not the Pi.
4. `obstacle_node`: subscribe `/scan`, compute nearest return within a forward
   arc, call `/estop` below `min_obstacle_m`. Replaces `coordinator.py` entirely.
5. Add a `get_clearance(angle)` tool so the agent can ask before committing.

**No map, no SLAM, no tf.** Obstacle avoidance needs the raw scan, nothing more.

**Set the speed cap here.** Measure, don't estimate:

```python
samples.sort()
p99 = samples[int(len(samples) * 0.99)]
```

Measure with everything else running — voice live, camera posting. Then
`stopping distance = speed × p99`. The lidar's 167 ms scan period is already
most of your budget, so expect p99 around 200–300 ms; at 0.3 m/s that's 6–9 cm.
If it's worse, **lower `default_speed_pct`** rather than trying to make Python
fast.

**Done when:** the robot refuses to drive into a wall, and the agent is told why.

### Phase 6 — odometry + tf *(a weekend)* — the SLAM prerequisite

This is the phase that un-skips tf2, and it's most of the work in getting a map.

Three transforms are needed:

| Transform | Meaning | Effort |
|---|---|---|
| `base_link` → `laser` | where the lidar sits on the robot | One static transform in a launch file |
| `odom` → `base_link` | where the wheels think you've gone | **The work** |
| `map` → `odom` | SLAM's correction | Free — slam_toolbox publishes it |

The maths is short for a differential drive:

```python
d_left   = ticks_left  / ticks_per_metre
d_right  = ticks_right / ticks_per_metre
d_centre = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / wheelbase

theta += d_theta
x += d_centre * math.cos(theta)
y += d_centre * math.sin(theta)
```

You're most of the way there — `TICKS_PER_CM` is calibrated and your quadrature
is signed, which is the part people get wrong.

**Two things to check first:**
1. Measure the wheelbase (distance between wheel contact points).
2. ~~Does the firmware stream encoder counts continuously?~~ **Resolved —
   it does.** `drivetrain_node` already declares `encoder_publish_hz` (default
   10.0) and publishes `/encoders` on a timer, and the parameter is in
   `robot.yaml`. No second firmware change needed; Phase F stays a single
   change.

Publish both `nav_msgs/Odometry` and the tf transform.

**Done when:** `ros2 run tf2_tools view_frames` shows a connected tree and the
odom pose roughly matches reality after driving a square.

### Phase 7 — SLAM + the two-mode split *(a weekend)*

```bash
sudo apt install ros-jazzy-slam-toolbox
```

**Two launch files, different node sets:**

| | `scan.launch.py` | `command.launch.py` |
|---|---|---|
| Runs | robot core, lidar, slam_toolbox (mapping), explorer | robot core, lidar, obstacle, perception, voice, servo |
| Audio | **none** | live |
| SLAM | mapping mode | **none** |
| Output | `maps/house.pgm` + `.yaml` | commands executed |

Swapping the entire node graph becomes a config change rather than a code change
— another place ROS quietly earns its keep.

Above them, a **mode manager**: a systemd unit or a thirty-line supervisor that
owns which launch file is running. Neither mode should know about the other.

**The design question to settle before building:** scan mode has no voice, so
something has to end it. In rough order of preference — exploration reports
itself complete; a timeout ("map for 20 minutes"); a physical button. "The robot
is mapping and I can't talk to it" is a bad state to be stuck in. This is exactly
what the e-ink is for: show `MAPPING` with progress in scan mode, the face in
command mode.

**The explorer.** Don't reach for nav2. Give the agent — or a small node — two
inputs: unexplored frontiers from `/map`, and clearance from `/scan`. Then drive
toward the nearest frontier with clearance, using the `/drive` and `/turn` actions
that already exist. A couple hundred lines, runs on what you have, and it fits
the architecture you actually built.

**Done when:** the robot maps a room unattended, saves it, and returns to command
mode by itself.

### Phase 8 — optional, later

- **Localization mode** — load the saved map, run slam_toolbox in localization
  mode (~80 MB) so the robot knows where it is. Only needed for destination
  commands like "go to the kitchen."
- **nav2** — the full navigation stack. Several hundred MB and a lot of CPU.
  Only if the agent-driven explorer proves too crude.
- **rosbag2** — great for debugging, bad for SD cards. Record to USB.

---

## Part 8 — Standing rules

These outlive any phase.

**1. Never publish image frames between processes.** `rclpy` has no
intra-process zero-copy. A 1080p frame costs ~6 MB to serialise each way. Camera
capture and the REST POST live in **one node**; publish the JSON result, not the
pixels. Obey this and DDS costs nothing.

**2. Capture to memory, never to disk.** `io.BytesIO` + picamera2's hardware
encoder. The old code wrote a JPEG to the SD card four times a second and never
deleted them.

**3. Match the payload to the model's input size, and send it binary.**
*Revised — the original rule said "send 720p, not 1080p," which was right about
1080p and too generous about 720p.* A YOLO-class detector runs at 640×640
letterboxed, so a 720p payload buys pixels the model discards in its first
operation. Send ~640 px on the long edge (~30 KB) as `multipart/form-data`, never
base64 — base64 costs 33% for nothing. That is ~60 KB/s against ~320 KB/s for the
original design, on a radio that already carries a continuous microphone stream
that must not stutter. Capture high, send low, crop faces from the high one.
"HD camera" describes the sensor, not the payload.

**4. Enforcement belongs with the actuator.** Limits the model can talk itself
out of aren't limits. Clamp in goal-rejection callbacks, not in tool handlers.

**5. Reflexes in firmware, policies on the Pi.** One number and one threshold →
Arduino. Anything needing judgment or iteration → Python.

**6. Log what you drop.** Skipped gestures, dropped emotions, throttled events,
dropped frames, stale and out-of-order REST results — silent discarding reads as
"it's working" when it isn't.

**7. A measurement is worthless without its timestamp and its frame.** Any sensor
reading that crosses a latency boundary carries when it was taken and where the
sensor was pointing. A bbox is not a bearing until you know the capture time, the
gimbal angles and the base pose at that instant. See `VISION-DESIGN.md` Part 2.

**8. One request in flight; drop, don't queue.** Every network call from the
robot to an external service gets exactly one outstanding request, a hard
timeout, and a policy of dropping new work rather than queueing it. A queue in
front of a slow link turns a wifi hiccup into the robot acting on
three-second-old data.

---

## Part 9 — Things that will bite you

**1. Blocking the executor.** The default `SingleThreadedExecutor` runs one
callback at a time — a 4-second move in `execute_callback` means nothing else in
that node runs, including `/estop`. `MultiThreadedExecutor`, estop on a
`ReentrantCallbackGroup`.

**2. The e-stop path — test it explicitly.** Start a 3 m drive, call `/estop` at
t = 1 s, assert the wheels stop in under 200 ms. Run it after every change to the
node's threading. Your `VisionGuardian` comment documents this exact bug from
last time; ROS offers a fresh way to reintroduce it one layer up.

**3. rclpy + asyncio.** One crossing point, `call_soon_threadsafe`, never
`spin_until_future_complete` from the agent's loop.

**4. DDS discovery on wifi.** By default every ROS 2 process multicasts to the
whole subnet. Set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and a non-default
`ROS_DOMAIN_ID`. Switch to `SUBNET` only for laptop debugging sessions.

**5. Two writers on one serial port.** During Phases 1–2 the old entrypoint and
the new node can both open the Arduino. Only one at a time — the firmware's link
watchdog cannot tell them apart. A lockfile in `SerialDrivetrain.__init__` is
cheap insurance during the transition.

**6. USB serial renumbering.** See Part 5. udev rules before the lidar arrives.

**7. Wifi is a shared resource.** The mode split doesn't fix this — command mode
still has audio and camera POSTs on the same radio. Rule 3 is the mitigation.

**8. Every REST detection is stale on arrival, and the head has moved since.**
100–500 ms of round trip plus a 1–2 Hz call rate means a detection describes the
world 0.5–1.5 s ago, in the frame of a camera that has since panned. Acting on it
as if it were current is the single easiest way to build a vision layer that is
smoothly, confidently wrong. Rule 7 is the mitigation; `VISION-DESIGN.md` Part 2
is the implementation.

**9. Re-anchoring a track by overwriting it.** When a detection finally arrives,
the tempting move is to snap the tracked box to it. That box is 0.5–1.5 s old, so
snapping to it reintroduces exactly the REST-rate jumping the tracker was added
to remove. Correct by the *difference* against the track's own historical box at
that `seq`. `VISION-DESIGN.md` Part 4.2.

---

### A note on rewriting this in C++

It will be tempting, somewhere around Phase 5, to decide the Pi code is too slow
and should be C++. It almost certainly shouldn't, and it's worth writing down why
while the reasoning is fresh.

**Everything actually latency-critical here is already C++.** The PID and the
cliff reflex are Arduino firmware. `ldlidar_ros2` is C++. `slam_toolbox` is C++.
OpenCV is C++ with a thin binding that releases the GIL. What's left in Python is
policy at 1–10 Hz, serial framing at ~100 Hz, and state machines — nowhere near
where the language is the bound.

**The p99 problem is latency *variance*, not throughput** — scheduler, GIL
contention, GC pauses, network. C++ removes two of those four and leaves the two
that dominate. The budget is already 167 ms of lidar scan period plus 100–500 ms
of REST round trip; a rewrite buys maybe 10–30 ms off the tail. Part 2 of
`HARDWARE-BASICS.md` names the correct knobs and they are all cheaper: process
isolation (done), `nice`, rate, resolution. Language is last.

**What it would cost:** `colcon` C++ builds on a Pi, the end of laptop-testable
Layer A, the end of "delete Layer B and you still have a robot," and a rewrite of
the most debugged code in the repo — the framed serial protocol, the out-of-band
e-stop, the motion executor.

So: **measure p99 first**, with the voice session live, and only consider
rewriting a specific loop that a profile actually names. And if CPU genuinely
binds, a Pi 5 buys more than a Python→C++ rewrite of anything in this plan, for a
weekend of zero engineering — with the caveats that GPIO and PWM go through the
RP1 southbridge so the overlay config differs, power draw is higher, and PiSugar
compatibility needs checking first.

---

## Part 10 — CLI cheat sheet

```bash
# what's alive
ros2 node list
ros2 node info /drivetrain

# topics
ros2 topic list
ros2 topic echo /battery_state
ros2 topic hz /scan                      # expect ~6 Hz from the LD14P
ros2 topic info /detections --verbose    # QoS mismatches show here

# services
ros2 service call /estop std_srvs/srv/Trigger

# actions — your main tool from Phase 1
ros2 action send_goal /drive robot_interfaces/action/Drive "{meters: 0.5}" --feedback

# parameters — live, no restart. Your new calibration workflow
ros2 param list /drivetrain
ros2 param set /drivetrain ticks_per_cm 103.4

# transforms (Phase 6+)
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser

# maps (Phase 7+)
ros2 run nav2_map_server map_saver_cli -f maps/house

# build
colcon build --symlink-install
colcon build --packages-select robot_interfaces
```

---

## Effort summary

| Phase | What | Effort | Blocks on |
|---|---|---|---|
| **F** | TF-Luna cliff reflex (firmware) | an evening | nothing — do it early |
| **0** | Repo surgery + camera fix | an evening | nothing |
| **1** | Interfaces + drivetrain node | a weekend | 0 |
| **2** | Voice node + result feedback | a weekend | 1 |
| **3** | Battery, speech, bringup | an evening each | 2 |
| **4** | Servo, e-ink, perception | an evening each | 3 |
| **5** | Lidar + obstacle avoidance | a weekend | 3 |
| **6** | Odometry + tf | a weekend | 5 |
| **7** | SLAM + two modes | a weekend | 6 |
| **8** | Localization, nav2 | open-ended | 7 |

Phases F and 0 are independent of everything and of each other. Phase 3 is the
end of the migration — a natural stopping point if enthusiasm runs out, with
nothing left half-done.

**The vision layer interleaves rather than follows.** `VISION-DESIGN.md` Part 9
has its own V0–V6 table; the dependencies onto this plan are: V0 needs Phase 0
item 5 (still undone), V1 needs Phase 3, V3 needs Phase 4, V5 needs Phase 5, V6
needs Phase 6. V1 is the one to reach early — it measures the bandwidth and
latency the rest of that document's constants are guesses about.
