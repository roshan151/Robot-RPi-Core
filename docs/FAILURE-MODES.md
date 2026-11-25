# Failure modes: what can go wrong, how we notice, what the robot does

Written Nov 19 2026, after a week where every follow and obstacle bug was a failure nobody
had written down: a frozen pose steered by, "unknown" taken as "free", a door frame taken as
the person, a planner goal moved to the wrong side of a wall, hardware faults that looked
like software. This file is where those are written down **before** code is written.

## How to use this file

- **A new module or input:** add a table (or rows) before writing the code. For every input:
  what if it is *missing*, *stale*, *plausible but wrong*, *from the wrong object*. For every
  output: who trusts it, and what happens to them if it is wrong. For every piece of hardware:
  *reset*, *power off*, *slow*.
- **An incident:** add a row, then a test that fails without the fix (a saved replay scenario
  or a simulated room, below), then the fix. A dated comment in the code is not a test.
- **Columns:** *Seen* = when it happened for real (or "expected" if not yet). *Detected by* = the
  code that notices. *Response* = what the robot does (see the health modes). *Covered by* = the
  test or rule that keeps it fixed.

Tests and tools (Monitoring-Service, from the Mac, no robot needed except where said):

| Tool | What it does |
|---|---|
| `test_replay.py` | Replays saved debug sessions (`scenarios/replay/`) through today's code: Mac range rules, Pi tracker, planner on the recorded maps, steering. `--save NAME --session S` freezes a new one. |
| `test_scenarios.py` | The real nav stack in simulated rooms (box, door, trap, lookaway, sudden, follow, doorway, doorway2, doorleaf, corridor, nook, cable, wedge, ledge, glass, door20, corner20, behind, chair) with pass rules and pictures. |
| `scorecard.py` | Scores any recorded session against fixed limits (`session_checks.LIMITS`); `--compare` against another. |
| `test_software.py` / `test_hardware.py` | End to end on the Mac and on the robot. |

Rule breaks (`robot_core/rules.py`) go to logs.json as WARNING `"rule"` events and are counted
in follow's debug note; the scorecard reads both.

## Health modes (`robot_core/health.py`)

| Mode | When | The task does |
|---|---|---|
| preflight fail | servo power off, no ToF grid in 1 s, no fresh pose, Mac not answering /healthz in 2 s, no velocity firmware | does not start; says every failure at once, with the fix |
| normal | everything fresh | |
| degraded | no ToF grid for 1 s · odometry 1-3 s old · Mac step over 2 s | stand still (no driving blind) · nav stops itself ("no pose") · keep the last goal |
| stop | odometry over 3 s old · servo supply off | end the task, saying why |

## Sensors and hardware

| Failure | Seen | Detected by | Response | Covered by |
|---|---|---|---|---|
| Odometry stops (Arduino reset / hang / serial drop) | Nov 19 (twice) | `reading.Latest` age in voice_node; drivetrain "odometry telemetry stopped"; `Watchdog` | pose is None after 1 s → nav stops; task ends after 3 s; preflight refuses to start | `health`, rule `pose_stale`, scorecard `no_pose_steps` |
| Arduino brown-out / RESET noise mid-move | Nov 19 | bridge "Arduino reset mid-command" | move fails; odometry gap handled as above | hardware: power wiring (open) |
| Head servo supply off | Nov 19 | `gimbal.powered()` (GPIO4 sense) | preflight fail; `Watchdog` stop | `health` |
| ToF grids stop | expected | grid age in `Watchdog` | degraded: stand still | rule `tof_stale` |
| ToF grid paired with the wrong camera frame | Nov 19 | `grid.t - frame.t_capture` > 0.35 s | grid not used to range that frame's boxes | rule `grid_unpaired` |
| ToF on the gimbal looks at the person, not the path | Nov 19 (root of several) | `AHEAD_STALE_S`, `seen_free` | speed capped where unseen; glance ahead; unknown never counted as free for straight-line decisions | test_scenarios lookaway, doorway; option: a fixed forward ToF |
| Camera motion blur when turning fast | Nov 19 (9 misses in 16 at a doorway) | | pivot capped at 0.55 rad/s while the camera is on someone | `PIVOT_FOCUS_W` |
| Camera fisheye distortion: floor distances near the picture edge jump | Nov 19 (found by the replay) | replay `pi_goal_jumps` | server supports calibrated `VS_CAMERA_DIST` + `VS_CAMERA_FX/FY/CX/CY` (`_undistort_px` in `nav_api.py` and `vslam.settings_yaml`); run chessboard calibration and set env vars | test_software fixes, test_replay doorway-20261119 |
| IMU pitch/roll mounting drift turns flat floor into false ToF obstacles (19% on Nov 23) | Nov 23 (3.3° pitch error) | `--report` floor-as-obstacle check | `imu_auto_tare: true` auto-zeros pitch & roll during the first stationary 1 s window on boot (within ±12° guard) | test_software fixes |
| No drop/cliff detection when `tfluna_fitted: false` | Nov 23 audit | `terrain.params_from` | `terrain.Params.detect_drops` automatically enables ToF visible-drop & stairwell-hole detection when `tfluna_fitted` is `false` | test_software fixes |
| Gimbal pan at its limit (+81°) while the body does not turn | Nov 19 | head pan in debug note | body turns toward the goal (pivot when > 28° off) | scorecard (planned: pan-at-limit time) |
| Bump / stall while driving forward | Nov 19 | drivetrain `motion_health` → reflex trip | brake; the spot is marked in the costmap; replan | rule `trip_repeat`, test_scenarios lookaway |
| Bump while turning on the spot | Nov 19 (door leaf) | reflex trip | brake only: which corner hit is unknown (open) | |

## Perception: the person (Mac `vision_service/follow.py`, Pi `robot_core/follow.py`)

| Failure | Seen | Detected by | Response | Covered by |
|---|---|---|---|---|
| ToF range on the box hits something in front (chair leg) | Nov 19 (39 in one run) | Mac `_refuse`: nearer than the feet by max(0.3 m, 3σ); Pi `refuse_range`: 0.4 m nearer than tracked | range refused; floor/face distance used | test_replay (mac, pi), rule `range_refused` |
| Edge-only ToF zones read the door frame beside a far person | Nov 19 (doorway) | Mac `_refuse(edge_only)` | refused when nearer than the feet by 0.3 m | test_replay, scorecard `edge_ranges_used` |
| ToF zones on the floor in front of the feet | Nov 19 | zone height < 8 cm | zone skipped | Mac range_in_box `floor` count |
| Range under 0.4 m on a box that does not fill the frame | Nov 18 | Mac `_refuse` | refused | test_replay `near_ranges_used` |
| Pi's range_m read on the previous frame's box | Nov 19 | Mac `_refuse` on the Pi range | refused if implausible | test_replay |
| A false box (slippers, a coat) taken as the person | Nov 19 | clothes similarity | lone-person reacquire needs ≥ 0.5 (was 0.35) | (add a replay scenario when seen again) |
| Tracked person jumps further than anyone walks | Nov 19 | `WALK_MAX_MPS · dt + 0.5 m` | logged; tracker gate rejects one wild reading | rule `goal_jump`, test_replay |
| Lost for good too soon ("gone") | Nov 19 | Mac search timing | head sweep dwells 0.8 s per position; gone after 20 s | |

## Navigation (`robot_core/nav/`, Mac `vision_service/nav_map.py`)

| Failure | Seen | Detected by | Response | Covered by |
|---|---|---|---|---|
| "Unknown" cells taken as free for a straight-line decision | Nov 19 (twice) | `costmap.known_free_line` | skipping ahead along a path, dropping the Mac's detour, and preferring our own Theta* all need SEEN free | **no test yet** (undoing it turns nothing red): write a scenario |
| Planner goal moved to the robot's side of a wall the person stands behind; the plan then alternates "through the doorway" / "at the wall" | Nov 19 (root of the doorway loss) | `GlobalMap.plan(target=)`: the goal must see the person | goal re-chosen on the person's side | test_replay (3 flips without it, limit 1) |
| Path ends where DWA cannot reach (the start's relaxed margin reached the goal) | Nov 19 (simulator) | planner: ends only where the full margin fits; relaxes only within 0.4 m of the start | | test_scenarios trap, doorway, doorway2 |
| Global path hugs a wall DWA cannot follow | Nov 19 (simulator) | | robot asks for DWA radius + margin + 2.5 cm (`plan_radius_m`) | test_scenarios (4 rooms red without it) |
| Skipping ahead cuts a door-frame corner and wedges | Nov 19 (simulator) | | skipping ahead needs 5 cm more room than DWA's margin | test_scenarios doorway |
| Wedged: "driving" but not moving | Nov 19 (simulator) | `_watch_progress`, rule `no_progress` | after 4 s under 5 cm: back off 10 cm, plan again | test_scenarios trap |
| Steering target flips sides: left-right-left on the spot | Nov 19 | rule `pivot_flip` | (a 1 s turn commitment and a keep-the-longer-plan rule were tried and removed: no test needed them, and doorway2 and the replay were better without) | test_replay, test_scenarios (every room checks it) |
| Fallback turn always to the left when DWA will not advance | Nov 19 (simulator) | | turns toward the target's side | test_scenarios box |
| Brakes again for the same thing | Nov 19 (4 times in 8 s) | reflex hit marked in the costmap | replan at once | rule `trip_repeat` |
| No safe arc for a long time | expected | `blocked_long` | backs off / turns toward the way | rule `blocked_long` |
| Stands still "the way round is long" while the person walks on | Nov 19 | | only for a person standing still | scorecard `stood_way_round_long` |
| Too tight to turn on the spot, no room behind by the radius test: spins into the walls | Nov 20 (simulator) | `_room_behind`: also room when narrower but seen free and wide enough for the body | backs out along seen-free ground in 5 cm steps, then turns | test_scenarios nook |
| Creeps: 0.8 rad/s turns with nobody in frame, end-of-arc progress slowed it near every target | Nov 20 ("too cautious") | | 1.0 rad/s turns when not tracking (1.2 broke trap and sudden); progress is the arc's closest approach | test_scenarios behind (spin), trap (28 s limit) |
| Tracks spin on a cable head-on; the odometry drives on, the robot does not | Nov 20 (simulator) | `StuckWatch`: something at most 2.5 m ahead, facing it (hit on both sides of the look), came no nearer at the same head angle while the encoders went 12 cm | slip taken out of the pose (measured from the oldest look); crosses again 45° off square (`_cross`); a second slip there: marked, routed round | test_scenarios cable |
| Walls alongside read the same range while driving: false slip | Nov 20 (simulator) | the facing test | not counted | test_scenarios corridor |
| A turn held by a leg it cannot see (gyro still, tracks turning) | Nov 20 (simulator) | `_watch_turn`: commanded at least 0.3 rad/s, gyro below 0.1 for 0.7 s; rule `turn_blocked` | brake; mark the leading corner; back off 10 cm, then turn (turning the other way hits it with another corner) | test_scenarios wedge |
| Nose rides up on a ledge it cannot see and stops; the odometry counts on | Nov 20 (simulator) | nose up 6° above the floor's level (median pitch over 10 s) for 0.5 s; was 12° at once from where it set off | encoder metres since are taken out; marked; backs off; goes round | test_scenarios ledge |
| Bumps something it cannot see (glass) again beside the mark | Nov 20 (simulator) | `_failed`: rule `same_approach` (within 0.3 m and 30° of an earlier failure) | backs off 15 cm before turning; the bump mark is 0.5 m wide; two failures within 0.6 m are joined into one line, 0.3 m past each end | test_scenarios glass |
| A door the person walked through maps too narrow for the Mac (ToF cones draw the frame into it); the robot takes the Mac's long way round and loses them | Nov 20 (18-27 s) | | oriented capsule body clearance in `dwa.rect_clearance` + `nav_map.py` margin relaxation down to 0 | test_scenarios door20 (passing), replay follow-20261120 |
| Swings left-right beside a wall corner 0.2 m off: the Mac's 0.17 m step aside is dropped as reached; the line on is at DWA's limit | Nov 20 (68-82 s) | rule `pivot_flip` | oriented capsule body clearance (`dwa.rect_clearance`: 0.165 m half-width vs 0.177 m circle) and Mac/Pi aligned `plan_radius_m` (`0.230 m` from `DwaParams.body_radius_m`) | test_scenarios corner20 & doorleaf (passing), test_software fixes |
| Gimbal-mounted single IMU: head panning feeds chassis speed loop as turn, or head tilt trips 25° tip stop / ledge check | Single-IMU gimbal mount | `navigation_debugging.py --report` (`interlock` leak, `phase` band, head-still vs moving floor split) | `head_still_while_driving: true` interlock (`Gimbal` refuses moves while tracks move; `LocalNav._send` / `RosBase` wait 0.15 s for head to settle); true-vertical yaw (`gyro · up` in `imu.Fusion`); `MotionHealth(relative_tilt=True)` measures `√(Δpitch²+Δroll²)` from move start; `Head.project` uses IMU pitch only when `gimbal.settled(0.2)`; 4-phase follow (`watch`/`align`/`go`/`search`) | test_software fixes (Part A & B) |
| Person-ignore circle erases a real obstacle | Nov 19 | | person placed by refused/accepted ranges only (see Perception) | test_replay `pi_placed_too_near` |

## Mac: SLAM, planning service, network

| Failure | Seen | Detected by | Response | Covered by |
|---|---|---|---|---|
| ORB-SLAM3 SIGSEGV in `getTrackedMappoints()` on NULL `MapPoint*` | Nov 23 (`python3.12-2026-11-23-150045.ips`) | child exit code -11 | verified regex patch in `install_orbslam3_macos.sh` skips `!Mps[i] \|\| Mps[i]->isBad()`; defensive finite-point filter in `vslam._tof_step()` | `deploy/install_orbslam3_macos.sh` |
| ORB-SLAM3 does not track (white walls, blur, panning camera) | Nov 19 (2 of ~7 frames, one 4-minute run) | nav.jsonl `slam_frame` count vs frames in | odometry alone; plans in the odometry frame | scorecard SLAM line |
| YOLO projects clutter near person as false `"suitcase"` obstacles (55-172/run) | Nov 23 | `nav.jsonl` semantic counts | removed `"suitcase"` from `nav_api.OBSTACLES` | test_software fixes |
| Mac slow or down | expected | follow step time; preflight /healthz | degraded (keep last goal); local Theta*; explore spools plant/recon uploads locally (`UploadSpool`) and backtracks toward last online pose (`_return_to_signal`); `FrameStream` motion-gates & adapts JPEG quality to RTT | rule `mac_slow`, test_software fixes |
| Uneven outdoor ground (grass, mulch, garden slopes) reads as blocked | Phase 3 outdoor | `terrain.classify` | `outdoor_mode: true` fits local ground slope (blocks > `max_slope_deg`), tolerates `roughness_tol_m` (4 cm) while still catching rocks/curbs/drops | test_software fixes |
| Follow step returns 500 | Nov 19 (3 attempts) | VisionError | follow ends, says "vision error" | test_software vision |

## Logging and evidence

| Failure | Seen | Detected by | Response | Covered by |
|---|---|---|---|---|
| logs.json deleted while the robot runs | Nov 19 | `robot_log._ReopeningFileHandler` (inode check per record) | a new logs.json is started | |
| Debug session pruned before it was looked at | Nov 19 | | freeze it: `test_replay.py --save` | |
