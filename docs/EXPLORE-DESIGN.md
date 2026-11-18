# Explore mode — design

Explore mode maps the house, finds every plant, photographs each one from all
sides and keeps a register of plants that later runs add visits to (growth
tracking). This doc records the decisions behind it. **It amends `PLAN.md` and
`VISION-DESIGN.md`** where the final hardware differs from what they assumed —
see Part 0.

## Part 0 — What this changes in PLAN.md / VISION-DESIGN.md

| Was | Now | Why |
|---|---|---|
| LD14P 2-D lidar on the Pi for mapping and obstacles (PLAN #7, Phase 5) | **No 2-D lidar.** The TF-Luna, mounted on the head at camera level, *is* the scanner: pan sweeps ±45° × four 90° base turns = one 360° keyframe | Final sensor set is camera + TF-Luna + pan/tilt |
| TF-Luna on the Arduino, tilted down, cliff reflex only (PLAN Phase F) | TF-Luna on the **Pi UART** (`/dev/serial0`), on the head. Cliffs and low obstacles are checked by **tilting the head down before every drive step** (`Head.floor_probe`) | One sensor, three jobs; the reflex moves from firmware to a pre-drive check, acceptable because every move is a short, stopped, probed step |
| `slam_toolbox` on the Pi (Phase 7) | **Keyframe SLAM on the Mac** (`Vision-Microservice/vision_service/slam.py`) | A single-beam scan takes ~20 s; that is batch SLAM, not a streaming problem. The Mac has the CPU, keeps the map next to the plant register, and renders it |
| Pan limited to ±90° (VISION-DESIGN 4.5) | **Pan ±90°, tilt ±45°**, clamped in the driver, parked at the start position on every shutdown | Final mechanical limits |
| Frontier explorer on the Pi from `/map` | Frontiers and paths come back from the Mac with every keyframe; the Pi follows them with the existing `/drive` and `/turn` actions | Keeps the Pi thin; paths are computed on the same map the scan was matched into |
| Camera upright | Mounted **inverted**; rotated 180° in the ISP (`Transform(hflip, vflip)`) so every consumer sees an upright image for free | — |

| One level beam; one range per plant; plants placed in 2-D | With the 8x8 ToF fitted (`tof_fitted`): each keyframe carries wall rays, **free-space rays**, **low obstacles**, **drops** and **every reading as a 3-D point**; plants are placed in **3-D** (map x, y and pot height), so a plant stand holds several plants at one spot. See Part 4b | The ToF sees 64 directions at once, floor to above head height |

Unchanged: `robot_core` imports no ROS; the drivetrain is the only thing
talking to the Arduino; enforcement stays with the actuator; one request in
flight to the Mac.

## Part 1 — Where things run

| Piece | Runs on | Code |
|---|---|---|
| Odometry (`/odom`, odom→base_link) + motion health | Pi, `robot` process | `robot_core/odometry.py`, `drivetrain_node.py` |
| Head: gimbal, TF-Luna, camera, visual motion | Pi, `explorer` process | `robot_core/sensors/` |
| Mission loop | Pi, `explorer` process | `robot_core/explore/mission.py`, `robot_explore/explorer_node.py` |
| SLAM: scan matching, occupancy grid, frontiers, paths, viewpoints | Mac | `vision_service/slam.py` |
| Object detection, plant embedding (DINOv2), plant register + photos | Mac | `vision_service/explore_api.py`, `plants.py` |
| Image+text reasoning (`/v1/ask`) | Gemini API, or local `qwen3-vl:8b` via Ollama on the Mac | `vision_service/vlm.py` |

## Part 2 — Odometry and its health signals

* **Firmware v5** adds never-reset totals and `millis()` to the telemetry frame
  (`E,<el>,<er>,<ol>,<or>,<ms>`). Per-move counters still reset as before, so
  nothing else changes. Without the totals, the coast-down ticks between one
  move's `D` frame and the next move's reset were lost.
* **Wheelbase** defaults to the *effective* value implied by the two existing
  calibrations: `2 · ticks_per_degree · 180 / (π · ticks_per_cm)`. That value
  includes wheel scrub, which a tape measure misses.
* **Drift budget.** σ_xy and σ_θ grow with distance and turning and are reset
  when SLAM accepts a scan match (`/odom/relocalized`). They are published so
  the mission can see how much the pose is worth.
* **Motion health** compares three opinions: the command (the executor's
  current op), the wheels (encoder speed) and the eyes (`/visual_motion`):

  | Command | Wheels | Camera | State |
  |---|---|---|---|
  | moving | still ≥ 1 s | — | `stalled` → brake |
  | moving | turning | static ≥ 1.5 s | `slipping` → brake |
  | none | turning ≥ 0.5 s | — | `pushed` |
  | turn R | measured L | — | `wrong_direction` (check `odom_swap_sides`) |

  Straight moves also give **`drift_deg_per_m`**: heading change per metre
  while driving straight, a live read on wheel mismatch.

**"Are we moving?" from the camera, cheaply.** Two measurements on
consecutive 320×240 frames, ~1–2 ms on a Pi 4. Phase correlation gives the
global shift (turning). A grid of changed cells, after removing the global
brightness change, catches forward motion (most cells change). A plant
swaying in a draught changes a few cells and does not count. It only runs
while the head is settled, since head motion is not robot motion.

## Part 3 — SLAM on the Mac

The robot is slow and the room is static, so SLAM becomes a batch problem on
keyframes:

1. **Predict**: last keyframe's map pose ⊕ odometry delta.
2. **Correct**: a correlative scan matcher searches ±0.4 m / ±12° around the
   prediction. It scores ray endpoints against a blurred copy of the walls
   already mapped, with a small penalty for straying from the prediction so
   symmetric rooms don't alias. A match is accepted at a mean likelihood ≥ 0.35;
   otherwise the prediction stands.
3. **Map**: log-odds grid, 5 cm cells. Rays carve free space, endpoints mark
   walls, the robot's footprint is free, and floor-probe hits are marked as
   obstacles.
4. **Plan**: frontiers are free cells touching unknown ones. One BFS over free
   cells, inflated by the robot radius, finds the nearest reachable frontier.
   The path is shortcut to the fewest straight legs, i.e. the fewest turn-then-drive steps.
5. **Viewpoints** for a plant: N angles around it at `orbit_radius_m`, each
   nudged in or out until it is reachable and has line of sight.

Returned with every keyframe: the corrected pose, **map→odom**, match score,
explored m² and the next goal. Maps are saved per name. A later run can
`load` a map and start at a user-given pose on it: the rescan and growth
tracking case.

**Not yet — loop closure.** Matching every keyframe against the whole map
keeps drift bounded in a room. After a long loop through several rooms, a seam
is possible. Keyframe poses are kept, so a pose-graph pass can be added
behind the same API.

**Tested** in simulation (`tests/test_slam.py`): with 8 % distance and 4°
heading error injected per leg, matched poses stay within 8 cm and 2.5°, and a
full room is explored end to end.

## Part 4 — Plants

* **Identity** is a DINOv2 embedding per view. It is trained for instance-level
  similarity (*this* plant, not *a* fern), and the best match is taken over
  all of a plant's views, which is what makes a rotated plant still match.
* **Place** is a separate signal. `/v1/plants/match` returns similarity and
  distance from the plant's last known map position. The robot decides:

  | Look | Place | Decision |
  |---|---|---|
  | ≥ threshold | ≤ 0.6 m | same plant: new visit |
  | ≥ threshold − 0.12 | anywhere | **ask the VLM**: "same individual, possibly rotated / grown / moved?" (no VLM: trust the map) |
  | lower | — | new plant |

* **Localising a sighting**: aim at the *pot* (70 % down the box, since leaves let
  the beam through), refine once with a fresh detection, take the median of 12
  TF-Luna readings, then add the pot radius to reach the centre.
* **Photos**: at each viewpoint the robot faces the plant, re-centres it with
  the detector, and saves a full-resolution frame. Each photo's metadata holds
  the pose, pan/tilt and range.

## Part 5 — Image+text model

`POST /v1/ask` (images + question, optional JSON answer). Providers are tried
in order:

* **Gemini** (`gemini-3.6-flash`) — best judgement; needs the internet, costs
  per call. The robot already has a Gemini key.
* **Local** `qwen3-vl:8b` through Ollama (4-bit), for private or offline use on
  the Mac mini. It fits next to YOLO, InsightFace and DINOv2 on a 16 GB machine.

Used today for: same-plant disambiguation, and optionally (`ask_before_drive`)
"is the way ahead clear?" before each step. Every caller has a non-VLM
fallback, so the robot never depends on a VLM answering.

## Part 4b — With the ToF grid: the whole room, and plants in 3-D

**Safety first, in both follow and explore.** Before every drive the head looks
down and the ToF judges the floor for the length of the drive plus a margin
(`terrain.classify`, see TERRAIN-DESIGN). The robot drives only over floor it has
*seen* and found flat. An obstacle, a step, a drop (stairs down) or a floor that
gives no reading (`unknown`) all stop the drive; `drive_on_unknown_floor: true`
allows the last one only. Explore also writes what it found onto the map so the
planner routes around it from then on.

**Going round without waiting for a scan** (`robot_core/local_plan.py`, shared with follow): a
step that the floor probe refuses still goes onto the map as before, and its hazards (the whole
floor scan's, with the 8x8 ToF) are also kept in the odometry frame. The robot plans a way round
to the same waypoint on a small local grid and drives up to three legs of it, each probed with
floor seen clear 0.35 m past its end (short legs must not creep up to an edge). Only when there
is no way round does it fall back to the next keyframe for a new path from the Mac.

**One keyframe** = two half-turns of the base x nine pan stops. At each stop:

| Sent to the Mac | From | On the map |
|---|---|---|
| `rays` | nearest thing at robot height, per grid column | walls (black); what the pose is matched against |
| `clear` | furthest floor seen with nothing standing on it | free space (white) with no wall at the end; fills the room where walls are beyond the ToF's ~3.5 m |
| `low` | things under the level beam (a step, a box, a pot on the floor) | orange; never planned through, with the robot's radius around them |
| `drops` | floor missing or lower than `max_drop_m` | magenta; never planned near |
| `cloud` | every valid zone of every look, as (x, y, height) | saved in `maps/<name>.npz` (`cloud`, N x 3, map frame, one point per 5 cm cube): the room in 3-D |

`low` and `drops` are sticky on the Mac: later rays passing over a step do not
erase it.

**Plants.** At five of the nine pan stops the camera looks twice: level, then
tilted up 30 deg (`camera_tilts`), so the upper shelves of a plant stand are in
the picture. For each detection the ToF zones on the box give the range, and
the box's lower and upper edges, the head tilt and that range give the
**height of the pot's base** and of the plant's top above the floor. A box edge
on the picture's border is not the plant's edge: that height is then taken from
another look, or left unknown.

* Two sightings are the same plant only if they are within `same_place_m`
  (0.6 m) on the map **and** their pot bases are within `same_level_m` (0.15 m).
  Same spot, different height = two plants on a stand.
* The record on the Mac holds `position: [x, y, z]`, `height_m`, `top_m`,
  `size_m`, `level` (0 = lowest at that spot) and `on_stand`.
* Recognising a known plant uses the same rule: among look-alikes, the one
  known to stand at this spot *and* this height wins.
* For the photos the head tilts to the plant's own shelf and, of the plants in
  view, takes the one whose pot is at the expected height.
* `map.png` writes plants at one spot one under the other, each with its height.

## Part 6 — Known limits

* **Stacked plants: not run on the robot yet.** Heights depend on the camera's
  real pitch. If `tilt_center_us` is off by 5 deg, a pot 1 m away reads ~9 cm too
  high or low: trim the tilt before trusting `height_m`, or raise `same_level_m`.
* Shelves above what a 30 deg upward look shows from where the robot scanned
  (roughly above 1.2 m at 1 m range) are not seen. Add a steeper tilt to
  `camera_tilts` if the stand is taller.
* A plant directly behind another at the same height and spot is one plant.
* The orbit path around a stand is planned once per plant, so a three-shelf
  stand is circled three times.

* One scan height. Anything below the head's level beam is seen only by the
  floor probe, and only straight ahead (±12°) of each step.
* Monocular photos plus one range per plant. No 3-D plant model yet.
* A TF-Luna gives no return from dark, absorbent or glancing surfaces. Those
  rays are dropped, not treated as free space.
* The head cannot be parked after a hard power cut. Run
  `python -m robot_core.sensors.gimbal home` before trusting the angles.
