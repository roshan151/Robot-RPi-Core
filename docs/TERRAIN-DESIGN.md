# Terrain sensing — design

The robot senses its own tilt and the floor ahead, so it can turn accurately, stop
before it tips, and refuse to drive into a box or off an edge. This records what is
built from `Terrain sensing and climbing plan.md` (steps 1 to 5) and what is not yet
(steps 6 to 9). **It amends `PLAN.md`, `EXPLORE-DESIGN.md` and `FOLLOW-DESIGN.md`**:
see Part 0. Wiring is in the README ("IMU and ToF sensors").

## Part 0 — What this changes in the other docs

| Was | Now | Why |
|---|---|---|
| TF-Luna on the head is the range sensor (EXPLORE-DESIGN Part 0) | **8x8 ToF on the head** (`tof8x8.py`). It answers the TF-Luna's `read()` / `latest()` / `since()` with its middle four zones, so `Head`, explore and follow use it unchanged. `tfluna_fitted` stays for a robot that still has one | The TF-Luna is off the robot; one beam cannot see the shape of the floor |
| Floor check: tilt down and compare one beam with where the floor should be, three pans (`Head.floor_probe`) | With the ToF: **two looks from the drive pose**, every zone turned into a height above the floor, judged by `terrain.classify`. `floor_probe` returns the same `Probe`, so `mission.py` is not edited | One look covers the whole path width, and says how high |
| Heading from the encoders (PLAN Part 4, odometry) | **Heading from the gyro** when `imu_fitted`; the encoders say how far. `/imu` is published | Tracks slip in every turn; the gyro does not |
| A turn is N encoder ticks | The turn runs as before, then the gyro says how far the robot really turned and **a short second move makes up the difference** (twice at most, `turn_tolerance_deg`) | Ten 90° turns on carpet should each land within 3° |
| Motion health: stalled, slipping, pushed, wrong direction | Adds **tilted** (stops the move with `stop_on_stuck`), **bumped** (reported only) and a gyro check for slipping in a turn. `MotionHealth.msg` gains `imu_known`, `pitch_deg`, `roll_deg` | The camera cannot see a slipping turn; nothing could see a tilt |
| Follow: no obstacle check, tilt limited to 0..45° in the voice node (FOLLOW-DESIGN Part 0, Part 3) | Follow with the ToF **checks the floor before every drive** and may tilt down to `drive_tilt_deg` to do it | FOLLOW-DESIGN Part 3 asked for exactly this |
| Arduino firmware changes only once (PLAN decision #12); no velocity mode | **Unchanged so far.** Steps 8 and 9 (velocity mode, climb controller) will reverse both; nothing built here touches the firmware | Climbing under control needs per-track speed, which the tick-move protocol cannot give |

Unchanged: `robot_core` imports no ROS; the drivetrain node is the only thing that
talks to the Arduino and the only one that reads the IMU; one request in flight to the Mac.

## Part 1 — Where things run

| Piece | Runs on | Code |
|---|---|---|
| IMU: read, zero the gyro at rest, fuse to pitch / roll / yaw | Pi, drivetrain node | `robot_core/sensors/imu.py` |
| Gyro heading, turn trim, tilt stop, `/imu`, pitch and roll on `/motion_health` | Pi, drivetrain node | `robot_core/odometry.py`, `robot_drivetrain/drivetrain_node.py` |
| ToF: read the grid, stand in for the TF-Luna | Pi, whichever process owns the head (voice task or explorer) | `robot_core/sensors/tof8x8.py` |
| Drive pose, floor profile, floor scan | same | `robot_core/sensors/head.py` |
| Flat / climbable / blocked / drop | same; a pure function | `robot_core/terrain.py` |
| Floor check before a follow drive; range on the person | Pi, voice node task | `robot_core/follow.py` |

The head's owner needs the robot's pitch (a nose lifted by a sill makes flat floor look
like a slope). It takes it from `/motion_health`, so the IMU has one reader.

## Part 2 — The IMU

Accelerometer and gyro only (the board's BMI323). The magnetometer sits beside two motors
and is not read; nor is the barometer.

* **Angles**: pitch > 0 nose up, roll > 0 right side down, yaw > 0 turned left and
  unwrapped. `imu_axes` maps the board's printed axes to the robot's.
* **Fusion**: pitch and roll follow the gyro and are pulled back by the accelerometer
  whenever it reads only gravity (within 0.15 g). Yaw is the gyro alone.
* **Gyro zero**: measured whenever a full second of samples is steady and the drivetrain
  says nothing is moving. A steady mean far from the last zero is taken for a slow turn,
  not drift. This is what holds yaw to about a degree a minute.
* **Tilt**: past `max_pitch_deg` / `max_roll_deg` for 0.3 s is `tilted`, which stops the
  move like a stall does. **Nose dip**: on a drive, the front falling more than
  `max_dip_deg` (8°) below the pitch the drive started at, at least half of it in the last
  0.5 s, is `tilted` too, at once (reversing: the tail). That is going over an edge the ToF
  missed; a sill crossed and left behind comes back to the start pitch, and a gentle ramp
  is not fast, so neither trips it. **Bump**: a jolt over 0.8 g during a move is `bumped`,
  reported on `/motion_health` and in the log. The drivetrain does not stop on it (tracks
  jolt on sills); follow does (FOLLOW-DESIGN Part 3).

## Part 3 — The ToF and the floor

* 8 x 8 zones over 60° x 60°, 7.5° each, about five grids a second over I2C.
* **Drive pose**: pan 0, tilt `drive_tilt_deg` (-30°). With the sensor 25 cm up, the eight
  rows meet the floor from about 17 cm to beyond 3 m ahead.
* **Floor scan**: two looks, the second 3.75° lower, so its rows fall between the first's:
  sixteen strips of floor, about 4 to 9 cm apart within half a metre.
* **Floor profile**: each zone's range becomes (ahead, left, height above the floor) using
  the head's pan and tilt and the robot's pitch. `tof_perpendicular` says whether the
  module reports distance to its face or along the ray; `check_tof.py --wall` measures it.

`terrain.classify(points, params, reach)` judges a strip `path_width_m` wide, for as far
as the robot means to drive:

| Answer | Means | Rule |
|---|---|---|
| `flat` | Drive | Nothing above 1.5 cm (sensor noise), nothing below `max_drop_m` |
| `climbable` | A step the tracks can take | A rise of at most `h_max_m`, with readings beyond it that stay under `h_max_m` for `min_landing_m` |
| `blocked` | Something in the way | A rise above `h_max_m`, or one whose far side cannot be seen (a wall, a box, stairs up) |
| `drop` | The floor falls away | Readings more than `max_drop_m` below the floor. At an edge the rays carry on and land lower and further off, so the drop is placed just after the last strip that was still floor |
| `unknown` | Cannot say | Fewer than two strips of floor seen within reach (dark or shiny floor) |

Nothing climbs yet, so today `climbable` is treated as an obstacle by both follow and
`floor_probe`.

## Part 4 — Follow

Only flat floor is driven: the floor must be flat for the drive plus 0.2 m. Something in
the way (an obstacle, a step, a drop) ends the drive `stop_short_m` (0.25 m, robot.yaml)
before it, measured from the middle of the tracks; a floor that could not be read
(`unknown`) stops it altogether unless `drive_on_unknown_floor: true`. The turn toward the
person always happens. Explore applies the same rule before every step of its path
(EXPLORE-DESIGN Part 4b). The details of what follow looks at and when are in
FOLLOW-DESIGN Part 3; in short:

* the tracking view (the head on the person) is checked for anything in the path nearer
  than the person, up to the robot's height, and its lower rows for the floor;
* the head glances down only when neither the tracking view nor floor seen clear in the
  last 10 s (still ahead, on the same line) covers the drive;
* the whole grid goes to the Mac, which ranges every person in view.

## Part 5 — Known limits

* **Nothing here has run on the robot.** The IMU and ToF drivers are written from the
  chips' registers and DFRobot's library and are tested against fakes. Expect to adjust
  after `check_imu.py` and `check_tof.py`.
* **Low sills are at the edge of what the ToF resolves.** A ray that meets the front of a
  2 cm sill reads a height between 0 and 2 cm, under the noise floor, so a narrow sill can
  read as flat. That is safe (the tracks climb it) but means `climbable` is mostly reported
  for low platforms, seen from above. Thin things (a chair leg, a cable) can fall between
  zones.
* **A drop with no return at all** (a stairwell deeper than the sensor reaches) reads as
  a hole in the floor and is treated as a drop; a floor with too few readings is `unknown`.
  Neither is driven. This is tested against a simulated stairwell only: keep the robot
  away from open stairs until it has been tried with a hand on it.
* A dark or glossy floor may read `unknown` everywhere, and the robot will then refuse to
  move. `drive_on_unknown_floor: true` is the way out, at the cost of the stair check there.
* The floor is assumed level where the robot stands. On a ramp the pitch correction keeps
  the ramp itself flat, but the floor beyond its end reads as a rise or a drop.
* Turn trim relies on the firmware making a turn of a few degrees. If short turns
  overshoot, raise `turn_tolerance_deg`.

## Part 6 — Not built yet (plan steps 7 to 9)

Step 6 (explore with the whole grid: walls, free space, low obstacles, drops and a 3-D
point cloud on the Mac's map; plants placed in 3-D) is built: EXPLORE-DESIGN Part 4b.

| Step | What | Needs |
|---|---|---|
| 7 | Vision veto before a climb ("is that a cable, a pet?") | Step 9, the only caller |
| 8 | Firmware velocity mode | A second firmware change (reverses PLAN #12) |
| 9 | Climb controller | Steps 7 and 8, and a measured `h_max_m` |
