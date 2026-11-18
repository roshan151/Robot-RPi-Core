# Follow mode — design

"Follow me" / "follow Roshan": the robot follows an enrolled person. This
records the simple first version. **It amends `VISION-DESIGN.md`** (§4.6
`follow_node`, §7.2) the way `EXPLORE-DESIGN.md` does for explore mode.

## Part 0 — What this changes in PLAN.md / VISION-DESIGN.md

| Was | Now | Why |
|---|---|---|
| `follow_node` on the Pi, driven by a local optical-flow tracker between 1–2 Hz detections (VISION-DESIGN 4.2, 4.6) | **Stop-and-go on the Mac.** The Pi captures one frame, waits for `/v1/follow/step`, turns, drives, then looks again. No tracker on the Pi | The person is expected to walk slowly and the robot can wait, so nothing is extrapolated from a stale detection |
| Distance from the LD14P scan at the person's bearing | TF-Luna on the head, when its beam (the image centre column) lands on the person. Otherwise the floor point under the box's bottom edge, using the camera height | There is no 2-D lidar (EXPLORE-DESIGN Part 0) |
| Gimbal tracks the person | Head held at **pan 0, tilt `FOLLOW_TILT_DEG` (20°)**. The base turns to face the person | Simplest. At 20° a standing face is visible from ~1.3 m and the beam hits the torso at 1–3 m |
| Identity: 2 consecutive face matches, sticky for the track (VISION-DESIGN Part 6) | Unchanged. After that, identity is carried by **continuity** — where they should be next frame, and a bank of **clothes looks** (torso + legs histograms, up to 8, learned while tracking, back included). A face that is someone else's rules a person out. Details: `CAMERA-PLAN.md` Phase 1 | The face is rarely visible from behind |
| Obstacle veto at `MIN_OBSTACLE_M` | **With the 8x8 ToF: a floor check before every drive** (`TERRAIN-DESIGN.md`). Without it, none: it never reverses and drives at most 0.5 m per step | The ToF sees the whole path in one glance; follow may tilt down to the drive pose for it. See Part 3 |

Unchanged: `robot_core` imports no ROS, one request in flight to the Mac, and
follow is a `run_task` job like enroll/match/explore (the Live session is
closed while it runs).

## Part 1 — Where things run

| Piece | Runs on | Code |
|---|---|---|
| Voice trigger: `run_task("follow", name)`. Empty name = "follow me" | Pi, voice node | `robot_core/live/tools.py`, `settings.LIVE_SYSTEM_PROMPT` |
| The loop: capture → step → turn → drive | Pi, voice node, in the task thread | `robot_core/follow.py`, `robot_voice/voice_node.py` |
| Person detection, face match, histogram tracking, the move plan | Mac | `vision_service/follow.py` |

## Part 2 — The Mac's step

`POST /v1/follow/start {label}`: 404 if the label isn't enrolled (case-insensitive). An empty label
locks onto whichever enrolled person is confirmed first.

`POST /v1/follow/step` (JPEG + seq, pan/tilt, hfov, camera height, optional
`range_m`) returns `{state, target, bbox, bearing_deg, distance_m, turn_deg, drive_m, followed}`:

| State | Meaning | Move |
|---|---|---|
| `searching` | Not locked yet (or lost for a while) | The robot stays put and the **head** sweeps (`head` in the answer: pan, tilt above the follow tilt): eight looks: left to 90° in 45° steps, up 30°, across to −90°, down, back to centre, at `servo_fast_dps` (120°/s, still ramped). With `hint_deg` it looks where they were last seen first. The sweep pauses while anyone is in view |
| `tracking` | Found | Turn to face them (deadband 8°). If within 25° of ahead and the distance is known, drive `min(distance − 1.2, 0.5)` m |
| `holding` | Missed for ≤ 3 frames | Nothing |
| `gone` | The whole head sweep with no luck, or 20 s since they were lost (`search_s`), whichever comes first | The Pi ends the task and the voice session reopens |

`turn_deg` uses the drive/turn tools' convention (+ = right); `bearing_deg`
uses the head's (+ = left).

## Part 3 — Known limits / next

* **The path before a drive (needs the 8x8 ToF, `tof_fitted: true`).** In order:
  1. *Anything in the way, at any height up to the robot's own* (`robot_height_m`): the
     zones of the frame just taken (the head on the person, turned by the turn the robot
     makes first) that lie in the robot's path, higher than a climbable step, and nearer
     than the person by 0.3 m. A table top above the robot does not count; the person
     does not either. Needs the person's distance.
  2. *The floor*, looked at as little as possible: floor seen clear in the last 10 s that
     is still ahead on the same line (odometry; turned more than 8° or moved 8 cm aside
     and it no longer counts) is not looked at again; else the same frame's lower rows,
     if they reach the floor right in front of the robot; else the head glances down
     (`Head.floor_scan`, two looks) and back up. A glance records how far the floor is
     clear (up to 1 m), so the next drives usually need no look.

  Only flat floor is driven, for the drive plus 0.2 m. Something in the way, a step or a
  drop ends the drive `stop_short_m` (0.25 m) before it: the robot drives **part of the
  way** instead of not at all, and anything shorter than 8 cm is not driven. A floor the
  sensor cannot read is not driven (`drive_on_unknown_floor: true` allows it). The robot
  keeps turning to face the person either way; the OLED shows `path in the way at 0.62 m`.
  Without the ToF there is no check. See `TERRAIN-DESIGN.md`.
* **Step length** (Pi side, `settings.FOLLOW_MAX_DRIVE_M` / `FOLLOW_KEEP_M`): the Mac's step is
  capped at 0.5 m and keeps 1.2 m back; the Pi drives up to `min(distance − 0.5, 3)` m instead, but never past the
  floor it has actually measured (minus 0.2 m). When the drive pose does not see that far, one
  more floor look is taken 12° higher.
* **Going round** (`robot_core/local_plan.py`, shared with explore): whatever made a drive unsafe
  (box, step, drop, unreadable floor, something in the way) is remembered in the odometry frame
  for 2 min, so a drive into it is refused without looking again (`path known hazard`). Then A* on
  a 5 cm grid round the robot, every hazard grown by half the robot's width + `plan_clearance_m`
  (0.255 / 2 + 0.08 m; the unseen far side of an obstacle is filled in 0.2 m deep), gives a way
  round as straight legs; up to three are driven per step, each floor-checked first, and the robot
  turns back to face the person. No way round: it stays and keeps facing them (`no way round`).
  The person's own legs (within 0.35 m of where they stand) are not remembered.
  Only something 8 cm or more off the floor is remembered, and a floor zone with no return is
  floor not seen, not a hazard (`floor_holes_are_drops: false`).
* **Levelling** (`floor_auto_level`, `terrain.level`): every floor scan is judged against a plane
  fitted to the floor right in front, because the SG90 tilt servo stops a few degrees off and the
  chassis rocks: the same flat floor read 7 deg sloped one way, then the other. Corrections over
  7 cm or 12 deg are not believed (a ramp, a box filling the view) and the scan is left as it was.
* **Watched drives** (A): with the 8x8 ToF the head looks down at the floor for the whole drive
  and every new grid (5 a second) is judged out to 0.8 m; the robot brakes once something is
  within `stop_short_m` + 0.2 m (`follow N: braked mid-drive, ...`), or after three grids in a row
  too thin to judge. So a drive may run past the floor seen before it, up to
  `FOLLOW_MAX_DRIVE_M` (3 m), and the glance needs no extra far look.
* **Gliding** (B, `settings.FOLLOW_GLIDE`, needs firmware `drv8871-v6-vel`): the drivetrain then
  takes `/cmd_vel` (forward m/s, turn rad/s) and holds each track's speed with encoder feedback
  (`robot_core/velocity.py`, robot.yaml `velocity_*`; it stops when `/cmd_vel` goes quiet for
  `velocity_timeout_s`, during a Drive/Turn goal, and after an e-stop or a motion-health stop
  until a zero command). Follow, once locked on, holds the head level (`FOLLOW_GLIDE_TILT_DEG`
  above the real horizon) so the camera sees the person and the ToF the floor 0.5-1.3 m ahead,
  and on every frame: turns at 0.8 rad/s per radian of bearing (none within 3°, in place beyond
  30°), drives at 0.4 m/s per metre of clear path (top `FOLLOW_GLIDE_MAX_MPS`, 0.2), judged on
  that frame's grid against the floor, things in the way and known hazards. From standing it
  glances at the floor right in front once. Blocked (under 8 cm clear): it stops and that frame
  is a step, with a way round. Lost, or a bump: it stops. Without v6 firmware, steps as before.
* **Bumps**: a jolt during a drive (the drivetrain's `/motion_health` says `bumped`) is
  something the ToF did not see: glass, a thin chair leg. The drive is braked, the floor
  memory dropped, and no drive is tried until the robot has turned 20° away
  (`FOLLOW_BUMP_TURN_DEG`). Straight ahead behind it, the robot turns but does not drive.
* **Edges**: with the IMU, the nose falling fast on a drive stops it at once
  (TERRAIN-DESIGN Part 2, `max_dip_deg`): the backstop for a drop the ToF missed.
* **Range and identity**: with the ToF, the whole grid goes to the Mac with every frame
  (`tof_grid`). The Mac ranges each person by the zones on their box: that is their
  distance (`distance_from: tof`), and a check on who is who. Once the target's ToF range
  is known, a person further than 1.2 m (plus 0.5 m per missed frame) from where the
  target should be is not taken by their clothes. A face match always wins. `range_m`
  (the zones on the previous frame's box) is still sent, for a Mac without this.
* **Searching where they went**: the Pi keeps the person's last bearing in the odometry
  frame (gyro heading with the IMU) and sends it, from its present heading, as
  `hint_deg`. When a search starts, the head looks there first; behind the robot, past
  ±90°, the robot turns that way instead. Once per loss.
* **Cancelling**: say "stop" (the cancel-only voice session, needs internet) or call
  `ros2 service call /task/stop std_srvs/srv/Trigger` — see `TASK-CANCEL.md`. Otherwise
  it ends when the person is gone or after `FOLLOW_SECONDS` (300 s).
* Two people in similar colours: the one nearer where the target should be wins, and a
  person whose face is seen to be someone else is never taken. A faceless lookalike who
  steps in where the target was can still be taken until their face is seen.
* Feet out of frame and no TF-Luna hit: the distance is guessed from the face's size,
  else the box's width. A box cut off at both the top and the bottom of the frame means
  they fill the view: the robot turns but doesn't drive.
* While searching, the only person in view is taken back on a weak clothes match (0.35)
  unless their face is someone else's. A stranger in similar clothes, alone in view with
  no face showing, can be followed.
