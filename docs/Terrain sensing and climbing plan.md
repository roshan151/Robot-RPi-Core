# Terrain sensing and climbing plan

Nov 11, 2026 · @Roshan Tiwari

## Objective

The robot should sense the floor and obstacles ahead, decide between go around, climb, or stop, and climb what its tracks allow (2–4 cm, to be measured). Follow and explore should use the same sensing.

The plan is in build order. Each step leaves a working robot, and nothing before step 8 touches the firmware.

- **Source:** the Pi repo (Robot-RPi-Core) was read for this plan. Mac-side file names come from the design docs; Vision-Microservice was not opened.
- **TF-Luna:** `robot.yaml` has `tfluna_fitted: false`, so the TF-Luna is off the robot and explore is currently refused. The ToF becomes the range sensor, and step 2 is what re-enables explore.
- **Chassis:** tank tracks 3 in tall and 11 in long, 1.5 in sprockets, 2.8 lb, TS-25GA370 12 V 130 RPM motors.
- **Sensors:** Fermion 10 DOF IMU (ADXL345, ITG3205, VCM5883L, BMP280) and Gravity 8×8 ToF (60° field of view, 3.5 m), with the ToF on the gimbal.

## Step 0: Measure and wire (no code)

**Objective:** get the numbers every later threshold depends on.

**Do:**

- Run the 1–4 cm board test to find `h_max`.
- Run `i2cdetect` to check the gyro's 0x68 address against the PiSugar.
- Measure the ToF's height and its offset from the pan axis.

**Done when:** the numbers are written down.

## Step 1: IMU driver

**Objective:** pitch, roll and yaw rate available on the Pi.

**Changes:**

- New `robot_core/sensors/imu.py`: reads the accelerometer and gyro, calibrates gyro bias at standstill, runs a small fusion filter in a background thread (same shape as `tfluna.py`), and includes a fake for tests.
- `pyproject.toml`: add an I²C library to an optional extra.
- New `tests/hardware/check_imu.py`; cases added to `tests/test_sensors.py`.

**Done when:** tilting the robot by hand shows correct angles, and yaw drifts under about 1° per minute at rest.

## Step 2: ToF driver

**Objective:** an 8×8 range grid that also stands in for the TF-Luna.

**Changes:**

- New `robot_core/sensors/tof8x8.py`: `grid()` returns 64 ranges, and `read()`/`latest()` match the `TFLuna` interface using the centre zones, so `Head` works unchanged.
- `explorer_node.py` and `voice_node.py`: construct it where the TF-Luna is built today.
- `robot.yaml`: add `tof_fitted` and the mount measurements.
- New `tests/hardware/check_tof.py`.

**Done when:** explore starts again and existing tests pass.

## Step 3: Gyro heading and accurate turns

**Objective:** turns that end at the requested angle despite track slip.

**Changes:**

- `odometry.py`: `DiffDriveOdometry.update` takes gyro heading when present. `MotionHealth` judges slipping from the gyro and adds `tilted` and `bumped` states.
- `drivetrain_node.py`: starts the IMU, publishes `/imu`, and after each turn compares the gyro angle to the goal and issues a short correction move if off by more than about 2°.
- `MotionHealth.msg`: append `pitch_deg` and `roll_deg`.
- `robot.yaml` (`/drivetrain`): add `imu_fitted`, `turn_tolerance_deg`, `max_pitch_deg`, `max_roll_deg`. `stop_on_stuck` also trips on tilt.

**Done when:** ten 90° turns on carpet each land within 3°.

## Step 4: Drive pose, floor profile, terrain rules

**Objective:** one function that says flat, climbable, blocked or drop for the path ahead.

**Changes:**

- `head.py`: add `drive_pose()` and `floor_profile()` (ToF grid, tilt and IMU pitch combined into heights ahead). `floor_probe()` uses it when the ToF is fitted and returns the same `Probe`, so `mission.py` needs no edit.
- New `robot_core/terrain.py`: a pure function `classify(profile, params)`, with no hardware or ROS imports.
- New `tests/test_terrain.py` with synthetic sills, stairs and drops.
- `robot.yaml`: add `drive_tilt_deg`, `h_max_m`, `min_landing_m`, `max_drop_m`.
- Voice node: allow tilt below 0° (FOLLOW-DESIGN Part 3 already calls for this).

**Done when:** the robot correctly labels a book, a door sill, a box and a table edge from 30 cm.

## Step 5: Follow mode

**Objective:** follow never drives blind and never loses distance.

**Changes:**

- `follow.py`: before each drive, glance down and classify; if blocked, skip the drive and report why. Send the median ToF range inside the person's box as `range_m`.
- Mac: none, since `/v1/follow/step` already accepts `range_m`.
- `docs/FOLLOW-DESIGN.md`: update the limits table.

**Done when:** the robot stops for a box placed between it and you.

## Step 6: Explore mode

**Objective:** faster scans, low obstacles on the map, and sills treated as crossable.

**Changes:**

- `mission.py`: `scan_keyframe` takes a row of ranges per pan stop, with fewer stops. `_remember_obstacles` records low obstacles with their terrain class.
- Mac `slam.py` and the keyframe API: accept a `kind` per point (wall, low, climbable, drop) and add a cost layer to the path planner.
- `docs/EXPLORE-DESIGN.md`: update Parts 0 and 6.

**Done when:** a keyframe takes under 10 s and a chair leg appears on the map.

## Step 7: Vision veto

**Objective:** never climb something that is a cable, a pet or soft.

**Changes:**

- `vision_client.py`: one method that asks `/v1/ask` "what is the low object ahead" with a JSON answer.
- Call it only when `terrain.py` says climbable.
- `robot.yaml`: add `climb_needs_label`, which decides what happens when no answer comes.

**Done when:** a cable across the path is refused and a door sill is accepted.

## Step 8: Firmware velocity mode

**Objective:** the Pi can set each track's speed continuously.

**Changes:**

- `drivetrain.ino`: one new frame carrying left and right speed in ticks per second, a per-track speed loop, and a 200 ms watchdog in this mode. Existing `M` moves stay untouched.
- `serial_protocol.py`, `arduino_bridge.py`, `client.py`: add `set_track_speeds(left, right)`.
- New `tests/hardware/check_velocity.py`.

**Done when:** both tracks hold a slow, equal speed up a ramp, and unplugging USB stops them within 200 ms.

## Step 9: Climb controller

**Objective:** cross a climbable hurdle under control.

**Changes:**

- New `robot_core/climb.py`: a state machine covering square up, approach, nose up, tip over, rear over and settle, with slip control and aborts on tilt or no progress.
- New `Climb.action` in `robot_interfaces`, served by `drivetrain_node.py`.
- `follow.py` and `mission.py`: call it when the class is climbable.

**Done when:** nine of ten crossings succeed at `h_max` minus 0.5 cm, with clean aborts above `h_max`.

## Docs

Add `docs/TERRAIN-DESIGN.md` with a "what this changes" table like the other design docs. It should record two reversals in PLAN.md:

- A second firmware change (decision #12).
- Velocity mode.
