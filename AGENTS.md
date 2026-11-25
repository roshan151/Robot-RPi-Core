# AGENTS.md — Robot Engineering & Holistic Analysis Guide

Read this before modifying hardware configuration, sensors, odometry, navigation, or follow/explore behavior in `Robot-RPi-Core`.

---

## 1. Core Principle: Analyze the Whole Robot Before Changing Anything

This robot is a tightly coupled closed-loop system spanning **three repositories** and **physical hardware**:

1. **Physical robot & config**: `Robot-RPi-Core/ros2_ws/src/robot_bringup/config/robot.yaml` (sensor mounts, axes, offsets, limits)
2. **Pi runtime (ROS 2 + control)**: `Robot-RPi-Core/robot_core/` (`imu.py`, `odometry.py`, `nav/controller.py`, `follow.py`, `drivetrain_node.py`) and `Robot-RPi-Core/logs.json`
3. **Mac vision, mapping & debug recordings**: `Vision-And-Planning/vision_service/` and `Vision-And-Planning/debug/<session>/`
4. **Calibration & regression suites**: `Monitoring-Service/` (`calibrate_and_apply.py`, `test_software.py`, `test_scenarios.py`, `navigation_debugging.py`)

**Never design a change from a single symptom or a single file.** Before proposing any change, trace every consumer of the signal you are touching — across hardware physics, ROS nodes, nav/reflex/stuck watchers, and Mac planning/replay.

---

## 2. Case Study: Why Mounting the IMU on the Gimbal Failed (Nov 24–25, 2026)

### What happened
In an attempt to measure true head tilt directly and avoid SG90 tilt-servo backlash, the IMU was moved from the rigid chassis onto the pan-tilt gimbal (`imu_on_gimbal: true`). That single assumption broke multiple chassis-level invariants at once:

* **Wrong physical frame**: Odometry heading (`DiffDriveOdometry`), tilt/climb/dip safety (`MotionHealth`), and climb stuck detection (`LocalNav._watch_stuck`) all require **chassis** attitude and yaw — not a panning/tilting head on a 22 cm mast.
* **Compounding workarounds**: Trying to make one gimbal-mounted IMU serve both the head and the chassis required a 16-file patch stack (`head_still_while_driving` interlock, 4-phase Watch/Align/Go stop-and-go follow, `relative_tilt`, `_odo_yaw` gating, Euler-rate coupling). Each workaround introduced new failure modes:
  * **Phantom walls & immediate reversing**: In `20261125-123841` and `20261125-124049`, inverted `imu_axes` plus calling `_watch_stuck` with pre-settle `v` caused the gimbal tilting down (`+41° → 0°`) to look like a `+35°` nose-up climb at `pose = (0, 0, 0)`. The robot planted a phantom wall at `[0.22, 0.0]` and immediately reversed (`cmd = [-0.08, 0.0]`) before moving 1 mm.
  * **Mast whip & backlash false trips**: Servo play and track start/stop vibration on the cantilevered gimbal caused `9–13°` attitude spikes, repeatedly tripping `MotionHealth` (`tilted — attitude shifted 10 deg fast`), planting more phantom bumps, and driving **0.70 m backwards** in `123841`.
  * **Stop-and-go thrashing**: Forcing the head to stop while driving degraded follow from a smooth **33.4 m path** with 0 false climb trips (`20261123-194111`, chassis IMU) down to **0.22 m net travel** (`20261125-124049`, 144 of 204 events stuck in `watch`/`align`).

### Lessons to carry forward
1. **Respect frame ownership**: Chassis sensors (IMU for yaw, pitch, roll, climb/dip/bump) belong on the **rigid chassis**. Head sensors (camera, 8×8 ToF) belong on the gimbal.
2. **Complexity is a warning sign**: If a change forces you to add interlocks, mode-gated sensor routing, or stop-and-go phases across 5+ files just to make an earlier idea viable, **stop and question the root assumption** instead of stacking more patches.
3. **Compare against a known-good baseline**: Always benchmark new runs against the last known-good debug session (e.g., `Vision-And-Planning/debug/20261123-194111`). If a metric regresses sharply (path length, false stuck/trip rate, reversing rate), roll back early.

---

## 3. Architectural Invariants

* **IMU mounting**: Keep the IMU rigidly mounted on the chassis. Use:
  * `imu_auto_tare: true` in `robot.yaml` (`Fusion.auto_tare_level` in `robot_core/sensors/imu.py`) so fixed mounting pitch/roll drift (within `12°`) is zeroed automatically on stationary startup.
  * `Monitoring-Service/calibrate_and_apply.py imu` (fixed-angle ramp self-climb + sideways roll check + self-driven 90° left turn) whenever the IMU is remounted or `imu_axes` changes.
* **No gimbal/drivetrain interlock**: `follow` and `LocalNav` rely on continuous head tracking while driving (`GimbalAim` glances/focus). Do not freeze the head while driving or freeze the tracks while the head settles.
* **Reset stateful watchers on idle/arrived**: Any timer or distance watcher (e.g., `LocalNav._progress`, `_moving_since`, `_stuck`) must reset when entering `idle` or `arrived` so resuming a drive never triggers an instant `no_progress` or `stuck` reverse.
* **Config deployment**: ROS 2 loads `robot.yaml` from `ros2_ws/install/robot_bringup/share/robot_bringup/config/robot.yaml` unless symlink-installed. Always verify whether Pi `install/` is updated after editing `src/.../robot.yaml`.

---

## 4. Mandatory Workflow Before & After Changes

### Before editing
1. **Inspect evidence first**: Check `Vision-And-Planning/debug/latest.json`, the latest debug session JSONs (`pose`, `cmd`, `nav`, `nav_detail`, `gimbal`, `extras.pitch_deg`, `rules`), and `Robot-RPi-Core/logs.json`.
2. **Trace all downstream consumers**: For any parameter or sensor you touch, grep across **both** `Robot-RPi-Core` and `Vision-And-Planning` (and `Monitoring-Service`).
3. **Check physical reality**: Account for servo backlash, track slip, acceleration transients, and mounting geometry before assuming a software filter will solve a mechanical problem.

### After editing
1. Run software unit tests:
   ```bash
   python3 ../Monitoring-Service/test_software.py fixes
   ```
2. Run navigation simulation scenarios:
   ```bash
   python3 ../Monitoring-Service/test_scenarios.py
   ```
3. If `robot.yaml` or sensor calibration changed, explicitly tell the user which Pi calibration/sync step is needed (e.g., `calibrate_and_apply.py imu` and restarting `robot-voice`).
