# Explore that finishes the job: coverage, plant measurement, "try, then try differently"

Plan, Nov 20 2026. Goals:

1. Explore every space the robot can physically get into.
2. Measure each plant's position and size in 3-D, with a stated uncertainty.
3. Attempt a passage it is unsure of instead of skipping it.
4. When stuck (gyro, encoders, ToF, camera disagree with the command), notice fast and try a
   different approach. Never repeat one that failed.

3 and 4 apply to follow as well as explore. Every phase is gated by simulation tests that are
written first and fail on today's code (Part 4), the same way as `test_scenarios.py`.

Amends `EXPLORE-DESIGN.md` (explore's driving moves onto the nav stack follow uses) and adds rows
to `FAILURE-MODES.md` as each phase lands.

**Decided (Nov 20):**
- A light rub against a door frame during a passage try is fine.
- Plant height is measured from the pot base.
- A whole-house explore may take 2 to 3 hours.
- Go under tables and chairs only where the clearance is over 20 cm; never under beds or sofas.

**Progress:**

| Phase | Status |
|---|---|
| A (stuck rooms only) | done: the sim has odometry separate from the truth, a gyro, IMU pitch, and terrain the ToF cannot see (slip lines, thin legs, ledges, glass) |
| B | done in `LocalNav` (follow has it): rooms `cable`, `wedge`, `ledge`, `glass` were red, now green; 17/17 rooms, 4/4 replays; each new behaviour turns its room red when undone |
| C to F | not started |

---

## Part 1: what "done" means (acceptance)

| Goal | Measured as | Target (sim) | Target (real) |
|---|---|---|---|
| Coverage | share of floor the robot could reach (truth map, rectangular body, gaps of at least body + 3 cm, under tables and chairs with over 20 cm clearance; beds and sofas excluded) that is seen free or occupied | at least 98 % | at least 95 % |
| Every room | rooms behind gaps of at least 0.29 m visited | all | all |
| Done is honest | explore ends with `done`; every unexplored area listed with a reason (closed gap, tried twice, drop, out of scope) | always | always |
| Plants found | recall | 100 % | at least 95 % |
| Plant position | pot centre x, y error | at most 3 cm | at most 5 cm |
| Plant heights | pot base z; plant top | at most 2 cm; at most 5 cm | at most 3 cm; at most 7 cm |
| Plant size | widest foliage width; pot diameter | at most 10 %; at most 1.5 cm | at most 15 %; at most 2 cm |
| Uncertainty | the stored σ covers the true error | 95 % of plants | checked with fixtures |
| Stuck noticed | command to "stuck" event | at most 1.5 s (turning: at most 1.0 s) | the same |
| Different approach | the same (place, heading within 30°) approach after a failure | never | never |
| Damage | contact while not in a try | 0 s | reflex brakes only |
| Try contact | contact during a passage try | at most 0.5 s per try (a light rub is fine) | light rub at most |

---

## Part 2: where we are today (gaps, with evidence)

| # | Gap | Evidence | Effect |
|---|---|---|---|
| G1 | Explore drives with its own stop-probe-step loop | `explore/mission.py` `follow()`, `_go_round`, `local_plan.py` | slow; no DWA or reflex; recovery written twice (follow and explore) and diverging |
| G2 | Planners treat the robot as a circle: Mac `robot_radius 0.18 + clearance 0.05`, inflated by whole 5 cm cells (needs a gap of about 0.5 m); Pi DWA `0.177 + 0.04` (about 0.43 m) | `slam.py traversable`, `dwa.DwaParams` | the body is 0.255 m wide. Gaps from 0.29 to 0.5 m are never planned and the frontiers behind them never explored. This is most of "explore every space" |
| G3 | Turning on the spot while wedged is not caught by nav | `controller._watch_stuck` returns when `v <= 0.02` | spins in place against a chair leg until the 4 s `no_progress` |
| G4 | Arcs are not slip-checked by the gyro in /cmd_vel driving | `drivetrain_node`: op = `"straight"` whenever v > 0.02; the gyro slip check runs on turns only | one track held (a cable, a rug edge) drifts unnoticed |
| G5 | Forward slip is only seen when the head looks ahead | `StuckWatch` needs the ToF straight-ahead range | while following, the head is on the person: a slipping drive goes unseen |
| G6 | Recovery is "back 10 cm straight, replan"; no memory of what failed | `_do_recover`, `_watch_progress` | the same approach is retried (`trip_repeat` fires; Nov 19: 4 times in 8 s) |
| G7 | Unreachable frontiers are dropped silently | `slam.py` picks the nearest reachable one; `mission.run` stops on `done` | "done" can mean "gave up on half the house" |
| G8 | 2-D costmap: anything up to the robot's height blocks; overhangs are not modelled | costmap, `robot_height_m` used for point filtering only | under tables and chairs (seats about 0.45 m high) is never explored |
| G9 | Plants: one range plus box edges per sighting; position is the median of visits; no lens undistortion; heights depend on the tilt angle | `_localise_grid`, `plants.combine_positions`, FAILURE-MODES "fisheye undistortion not done", EXPLORE-DESIGN Part 6 | cm-level position and size are not reachable as built; no σ is stored |
| G10 | The tilt servo does not move (the camera stays level when 20° is commanded) | Nov 19 debug: bottom ToF rows read 0.50 / 0.72 m (level) while tilt 20 was commanded | blocks every height measurement and the face search. **Hardware: fix first** |
| G11 | No simulator for explore, plants, slip or wedging | `NavSim` has front contact and stall only | none of the above can be tested before the robot runs it |

---

## Part 3: design

### 3.1 One drive stack for follow and explore (fixes G1)

Explore keeps what is its own: keyframe scans, frontier goals from the Mac, plant work, orbits.
It hands every move to `LocalNav`, the same controller, DWA, reflex, gimbal aim and recovery
that follow uses. `Explorer.follow(path)` becomes "set goal, wait for arrived / failed (with a
reason)". The stepwise driving, `_go_round` and the per-step floor probe are removed (the
costmap and reflex cover them). One stack means everything below is written once and works in
both modes.

### 3.2 Passages: rectangle, not circle, and "unsure means try" (fixes G2)

- **Footprint.** The planners keep the circle for open driving, where turning anywhere must be
  possible. A passage is judged by the **rectangle** (0.255 x 0.28 m) driven straight through.
- **Measured width.** When a planned or frontier route is blocked only by circle inflation, the
  gap is measured from ToF points on both sides: at least two looks, taken roughly square to
  the gap. That gives a width w with an uncertainty σ_w.
- **Classes** (body b = 0.255 m):

  | Class | Rule | Action |
  |---|---|---|
  | clear | w at least 0.46 m | plan normally |
  | tight / unsure | w + 2σ_w at least b + 3 cm, and w below 0.46 m | **try** (below) |
  | closed | w + 2σ_w below b + 3 cm | do not try; listed as "closed gap, w = ..." |

  "Unsure" deliberately leans toward trying: a gap that might fit gets a try.
- **The Mac** (`slam.py`, `nav_map.py`): a second traversability layer at the rectangle's half
  width (no turning room) marks **corridor cells**. A path may cross corridor cells only along
  a straight segment, which the Mac returns as a `passage` leg: entry, exit and measured width.
  Frontiers reachable only through passages are kept and flagged `tight`.
- **The try manoeuvre** (`LocalNav`, new state `passage`):
  1. **Stage.** Drive to a point 0.4 m before the gap on its centre line, where turning fits,
     and face along the line (gyro-held). Re-measure the width; if now closed, mark it and stop.
  2. **Go.** Drive straight at 0.06 m/s with the head locked ahead. Heading is held on the gyro
     to within 3°. The DWA circle check is off; a rectangle check against the costmap stays on,
     with 1 cm per side. The reflex stays on.
  3. **Abort** on any of these: a bump or stall; uncommanded yaw over 8° (a corner caught one
     side); pitch or roll over 6°; a side ToF zone closer than b/2 + 0.5 cm; no progress for 2 s.
  4. **Back out** in reverse along its own breadcrumb trail (poses every 5 cm) to the staging
     point. Reversing only over ground it has just driven is the one safe reverse, since there
     is no rear sensor.
  5. **Remember** the gap as `tried(heading)` and go to the recovery ladder (3.4).

### 3.3 Stuck detection: one detector, all signals (fixes G3, G4, G5)

A `StuckDetector` in `nav/` replaces `StuckWatch` plus the scattered checks. It runs every
tick, whatever the head is doing, on: commanded (v, w), encoder (v, w), gyro yaw rate, IMU
pitch and roll, displacement, ToF straight ahead (when the head is ahead), and the Mac's VSLAM
pose (when tracking).

| Kind | Fires when | After | Today |
|---|---|---|---|
| stalled | command non-zero, encoder speed about 0 | 1.0 s | drivetrain: yes |
| turn blocked (wedged) | commanded w at least 0.3, gyro w below 0.1 (encoders may still count) | 0.6 s | only for Turn actions; not /cmd_vel, not nav: **G3** |
| arc slip / one track held | gyro w differs from commanded w by over 0.3 rad/s (or over 40 %) while v is above 0 | 0.8 s | arcs counted as straight: **G4** |
| forward slip | encoders advance at least 8 cm while **any** of: ToF ahead unchanged; costmap scan not shifting (match against the robot's own last 2 s of points); VSLAM pose static; displacement from scan matching under 2 cm | 1.5 s | ToF ahead only: **G5** |
| climbing / high-centred | pitch or roll past its limit, or pitch changed by over 4° with no progress | 0.3 s | pitch: yes; roll and high-centring: no |
| no progress | under 5 cm displacement in 4 s while driving | 4 s | yes |

The drivetrain keeps braking on stalled, slipping and tilted, as its own safety layer. The
detector is what lets nav know **what kind** of stuck it is, which decides the next approach.
Each detection is logged as a `rule` event with the kind and the evidence.

### 3.4 The recovery ladder: try a different approach (fixes G6)

An **approach** is (goal, entry cell, heading bucket of 30°). Every failure is written to a
per-goal attempt log and to a **learned layer** on the costmap. The layer is sticky and is sent
to the Mac map like `low` and `drops`, so explore, the next keyframe and the next session
remember it.

| After | Mark | Next approach |
|---|---|---|
| bump / stall going forward | obstacle at the nose (as today) | back out on breadcrumbs 0.2 m, replan round it |
| wedged turning | a point obstacle at the swept corner | reverse 0.1 m, turn the other way; or a three-point turn (reverse arc, forward arc) |
| forward slip (cable, rug edge) | a "slip line" across the heading | back 0.2 m, cross again at 45° once (tracks climb cables better at an angle); if that slips too, route round the line |
| climbing / high-centred | `low` obstacle where the nose rose | reverse on breadcrumbs until level, route round |
| passage try aborted | `tried(heading)` on the gap | a second, different try: re-stage 10 cm off centre toward the side that did **not** touch, or come from the other side if a route exists; after 2 tries the gap is **closed (tried)** |
| 3 different approaches to one goal failed | goal unreachable (with reasons) | explore: drop that frontier, come back to it once at the end; follow: Mac's long way round, else wait at the last-seen spot, else search |

Rules: **never the same approach twice** (new rule `same_approach`, a scorecard limit of 0); at
most 3 approaches per goal; every rung is logged with what it tried.

### 3.5 Coverage: explore every space (fixes G7, G8)

- **Height-aware costmap.** Only points between the floor plus 1.5 cm (or a drop) and the
  clearance limit block a cell. Points above mark the cell **covered** (a table top).
  Under-furniture is driveable when the gap under the top is over `under_clear_m` (20 cm,
  robot.yaml) and the furniture is not a bed or a sofa. Beds and sofas are told apart by the
  Mac's detector (`bed`, `couch`), and their footprint is kept off the plan. The ToF's top rows
  and the tilted-up looks of a keyframe supply the overhead points.
- **Frontier order.** Clear frontiers first, by information gain over cost. Tight ones next,
  since a try is slower and riskier. Frontiers given up on are revisited **once** at the end,
  after a fresh keyframe.
- **Camera coverage for plants.** Driving coverage is not plant coverage: a plant on a shelf in
  a corner can sit in a seen-free room. A "camera-seen" layer ray-casts each keyframe frustum
  against ToF depth, at both tilts. Extra viewpoints go to reachable surfaces not yet seen up
  to 1.5 m high.
- **SLAM.**
  - A keyframe is taken right after every passage (rooms are joined through doors).
  - When the drift budget σ goes over 10 cm, a keyframe is taken before the next leg.
  - VSLAM is fused when tracking is good.
  - Loop closure exists; it is now covered by a sim test (C4).
- **Done.** Explore ends when every reachable cell (truth: rectangle flood fill, tight gaps
  included) is seen, or is behind something listed as `closed` / `unreachable` with a reason.
  It reports coverage %, the map and that list (Mac `GET /v1/slam/coverage`, and in the run
  summary).

### 3.6 Plants: measured, not estimated (fixes G9)

**Prerequisites** (a `calibrate_and_apply.py` mode each, written to robot.yaml):

1. Tilt servo working (G10).
2. Camera intrinsics and **fisheye distortion** from a checkerboard (k1 to k4); every bearing
   and edge is undistorted from then on.
3. Head zero and backlash (exists).
4. Camera to ToF alignment (exists).
5. IMU pitch and roll folded into every camera ray.
6. A **fixture check**: a box and a cylinder pot of known size at taped positions (Part 5).

**Pipeline** (Mac `plants.py` / `explore_api.py`; the Pi only drives and looks):

1. Detect, then **SAM 2 masks** for foliage and pot separately (SAM 2 is already the chosen
   segmenter).
2. Per view (undistorted): the pot centroid bearing, the pot's mask bottom edge, and the
   foliage mask extents. ToF zones inside the pot mask give 3-D points on the pot surface.
3. **Orbit:** at least 4 viewpoints, adaptive up to 8, at about 0.7 m, spread at least 180° when
   reachable. Each view carries its SLAM pose and real head angles.
4. **Solve jointly** by least squares: pot centre (x, y) and pot radius r from all bearing rays
   plus a circle fit to the pot-surface ToF points across views. The covariance is kept.
5. **Heights:**
   - Pot base z: the ray through the pot's bottom edge, met with the vertical line at the
     centre; median across views and tilts.
   - Plant top z: the highest foliage top-edge ray from any view, checked against the ToF's
     highest foliage points.
   - Width: twice the widest foliage half-extent across views at the solved range (visual-hull
     envelope).
6. **Gate:** if σ_xy is over 2 cm or σ_z over 3 cm, add a view (closer, or another tilt), within
   the 8-view budget. The record stores every value with its σ and the views used.
7. **Stands:** cluster pot-base z per spot (keeps `same_level_m`); each tier is solved alone.
8. **Growth:** later visits compare height and width with their σ, not raw numbers.

---

## Part 4: simulation tests (written first, red on today's code)

### 4.1 Harness: `Monitoring-Service/explore_sim.py`

Built from the pieces `test_scenarios.py` already uses: the real `robot_core` nav stack, the
real Mac `slam.py` / `nav_map.py` / plant code in-process, `SyncThread` for determinism, and
pictures. New in the world and sensor models:

| Piece | Model |
|---|---|
| World | boxes with a height range (z0, z1), so tables have legs and a top, shelves have tiers, and door leaves exist; walls; drops |
| Terrain patches | slip zones (cable, rug edge): encoders count, the body does not move, unless crossing at 45° or more; ridges: pitch rises over 3 cm, high-centring over 5 cm; thin legs (2 cm) the ToF misses at range |
| Drivetrain | (v, w) tracking with acceleration limits; encoders separate from true motion; a turn whose swept rectangle hits something stops (wedged) while the encoders keep counting |
| IMU | gyro = true w + bias (default 0.3 °/s) + noise; pitch and roll from terrain |
| ToF | today's 8x8 ray cast, extended to cylinders and ellipsoids (pots, foliage) and the box heights |
| Camera | pinhole plus fisheye distortion (k1, k2) to boxes **and masks** of plants and people, with z-buffer occlusion and pixel noise |
| Plants | pot cylinder (r, h, z0) plus foliage ellipsoid (w, h): known truth |
| Odometry error | scale 0 to 8 % and gyro bias, set per scenario |
| Metrics from truth | coverage (rectangle flood fill on the truth map); map error (wall chamfer distance); plant errors against truth; contact time; approaches per goal and `same_approach` count (from the rule log); time |

Run: `python test_explore.py [names]` (like `test_scenarios.py`). It saves the JSON/HTML report
and a picture per room: truth, map, coverage, plants with σ ellipses.

### 4.2 Scenarios and pass rules

**Passages** (run in explore, and in follow with the person walking through):

| Name | Room | Pass |
|---|---|---|
| P1 tight-gap | the only way to a plant room is a 0.32 m gap between a sofa and a wall | goes through on the first try; plant found; no contact |
| P2 closed-gap | a 0.27 m gap whose ToF width reads 0.30 ± 0.02 (noise) | tries at most twice, from different approaches; at most 0.3 s contact per try; marks it `closed (tried)`; reaches the room the long way |
| P3 door-leaf | a half-open door leaving 0.34 m | through on try 1 or 2 |
| P4 really-closed | 0.22 m (clearly below the body) | never tries; listed `closed, w = 0.22` |

**Stuck** (each in explore **and** follow). Built in `test_scenarios.py` as `cable` (S1), `wedge` (S2),
`ledge` (S3) and `glass` (S5), with nav goals. What changed from the design:
- **S2:** "turn the other way" does not work. A leg inside the corners' circle is in the way of a
  big turn either way. It backs off 10 cm, then turns.
- **S4** (one track on a rug): left for later. Steering is closed on the gyro heading, so the drift
  is corrected anyway; there is nothing to fail yet.
- **S1:** slip is detected about 3.6 s after it starts; the target is 1.5 s. The comparison needs
  two looks at the same head angle at least 1.5 s apart.
- **Follow variants:** these need the head on a person 45° or more off the way (the check uses
  looks within 45°): to do.

| Name | Room | Pass |
|---|---|---|
| S1 cable | a slip line across the only route | slip detected within 1.5 s with the head **not** looking ahead (follow); retries at 45° and gets across; no third attempt on the same line |
| S2 wedged-turn | a 2 cm chair leg the ToF misses, inside the turn's swept corner | "turn blocked" within 1.0 s; reverses or turns the other way; no repeat of that turn |
| S3 ridge | a 5 cm threshold strip (high-centres) | detected within 0.5 s; reverses until level; routes round it; marked `low` |
| S4 one-track | a rug edge under one track only | arc slip within 0.8 s |
| S5 same-trap | an obstacle seen only on contact, approached three ways | `same_approach` = 0; 3 different approaches, then goal unreachable with reasons |

**Coverage and SLAM:**

| Name | World | Pass |
|---|---|---|
| C1 house | 3 rooms through doors of 0.75, 0.45 and 0.33 m, furniture | coverage at least 98 %; all rooms; at most 15 min (sim) |
| C2 under-table | a table 0.7 m high with 4 legs; a chair with a seat at 0.45 m | the floor under both counted reachable and explored; no climbing onto legs |
| C3 dead-ends | a corridor with two closets and a dead end too tight to turn in | explores all; reverses out of the dead end (uses `_room_behind`) |
| C4 loop-drift | a 12 m loop through 4 rooms, 5 % odometry scale error, gyro bias 0.5 °/s | map wall error at most 10 cm after loop closure; coverage at least 95 % |
| C5 changes | the door closes mid-run; a person walks through the frontier | no hang; the closed room is listed; finishes |
| C6 honest-done | a room only reachable over a drop and one behind a 0.22 m gap | `done`, with both listed and correct reasons; no false 100 % |

**Plants:**

| Name | Setup | Pass |
|---|---|---|
| M1 one-plant | pot r 9 cm at a known x, y, z | x, y at most 3 cm; base z at most 2 cm; top at most 5 cm; width at most 10 %; σ covers the error |
| M2 stand | 3 tiers at 0.0, 0.45, 0.9 m | 3 plants, levels right, each within M1 limits |
| M3 corner | a plant in a corner (only 90° of orbit reachable) and one partly behind another | within M1 x 1.5; σ larger and honest; recall 100 % |
| M4 distortion | the same as M1 with lens k1 = -0.3 | passes **only** with undistortion on (mutation) |
| M5 bad-tilt | a 5° tilt error injected | the fixture or gravity check flags it; heights refused or corrected, never silently wrong |
| M6 revisit | run 2: the plant moved 0.3 m and turned 90° | the same plant label; new position within M1 |
| M7 shelf-search | a plant on a 1.2 m shelf in a seen-free room | found through the camera-seen layer |

### 4.3 Each new behaviour must turn a test red when undone (mutation checks, as for nav)

| Undo | Red |
|---|---|
| the rectangle passage layer | P1, P3, C1 |
| the try abort / back out | P2 (contact over 0.3 s or stuck) |
| the gyro turn-blocked check | S2 |
| the arc slip check | S4 |
| slip without the ToF ahead (scan / VSLAM evidence) | S1 (follow) |
| the learned layer / approach memory | S5 (`same_approach` over 0), P2 |
| the height-aware costmap | C2 |
| the end-of-run revisit and reasons | C6 |
| undistortion | M4 |
| the joint pot solve (back to a single range) | M1, M3 |
| camera-seen viewpoints | M7 |

### 4.4 Also kept green

All 13 `test_scenarios` rooms and the 4 `test_replay` checks. Explore sessions recorded on the
robot get frozen into `scenarios/replay/` the same way follow sessions are.

---

## Part 5: on the real robot (after sim)

New `test_hardware.py` modes, each run against measured truth:

- **passage**: two boxes making a 0.32 m gap (must pass), then 0.27 m (at most 2 tries, backs
  out).
- **stuck**: a cable taped across the way; a book wedged at a turn.
- **measure**: the fixture, a 20 x 20 x 40 cm box and a cylinder pot of known diameter at taped
  positions. Reports errors against Part 1.
- **explore room**: one room, coverage % and the "unexplored because" list, checked by eye on
  the map.

---

## Part 6: order of work

| Phase | Work | Gate |
|---|---|---|
| 0 | Fix the tilt servo (hardware); camera checkerboard calibration | fixture heights within 3 cm by hand |
| A | `explore_sim.py`: world, terrain, IMU, camera and plant models; every scenario written, baseline recorded red | the harness runs; baselines saved |
| B | `StuckDetector` plus the recovery ladder in `LocalNav` (follow gets it at once) | S1 to S5 in follow |
| C | The passage layer (Mac) plus the try manoeuvre (Pi) | P1 to P4 |
| D | Explore on `LocalNav`; height-aware costmap; frontier order; done-report; camera-seen layer | C1 to C6, S1 to S5 in explore, M7 |
| E | The plant pipeline: undistortion, SAM 2 masks, joint solve, σ gate | M1 to M6 |
| F | Real robot: Part 5 modes; freeze explore replays | Part 1 real targets |

B and C help follow straight away; D is the large one. Each phase is merged only with its
scenarios green, the earlier ones still green, and its mutation checks red when undone.

---

## Part 7: questions (answered Nov 20, see the top)

1. **Try contact.** Is a light rub of the tracks against a door frame acceptable during a try
   (the target allows 0.3 s)? Or must a try abort on the first touch?
2. **Plant size definitions.** Is height measured from the pot base or the soil line to the
   top leaf? Is width the widest foliage, or the average over views?
3. **Time budget.** How long may a full-house explore take? Trying tight gaps and 6 to 8 views
   per plant cost minutes.
4. **Under furniture.** Explore under beds and sofas too, or only tables and chairs? The camera
   sees little there, and it is where it is likeliest to get stuck.
