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
| Obstacle veto at `MIN_OBSTACLE_M` | **None yet.** It never reverses, closes only to `standoff_m` (1.2 m) and drives at most 0.5 m per step | The voice node limits tilt to 0..45°, so `Head.floor_probe` can't look down. See Part 3 |

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
| `searching` | Not locked yet (or lost for a while) | The robot stays put and the **head** sweeps (`head` in the answer: pan, tilt above the follow tilt): left to 90° in 30° steps, up 45°, across to −90°, down, back to centre. The sweep pauses while anyone is in view |
| `tracking` | Found | Turn to face them (deadband 8°). If within 25° of ahead and the distance is known, drive `min(distance − 1.2, 0.5)` m |
| `holding` | Missed for ≤ 3 frames | Nothing |
| `gone` | The whole head sweep with no luck | The Pi ends the task |

`turn_deg` uses the drive/turn tools' convention (+ = right); `bearing_deg`
uses the head's (+ = left).

## Part 3 — Known limits / next

* **No obstacle check before a drive.** Next: allow tilt below 0 during follow
  and run `Head.floor_probe` before each step, as explore does.
* **You can't say "stop" during the task**: the Live session is closed, as in explore.
  It ends when the person is gone or after `FOLLOW_SECONDS` (300 s).
* Two people in similar colours: the one nearer where the target should be wins, and a
  person whose face is seen to be someone else is never taken. A faceless lookalike who
  steps in where the target was can still be taken until their face is seen.
* Feet out of frame and no TF-Luna hit means the distance is unknown, so the robot
  turns but doesn't drive.
