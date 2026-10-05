# Vision & Perception — Design Doc

> **Amended by `EXPLORE-DESIGN.md`**: pan is ±45° (not ±90°), there is no 2-D lidar — range at a bearing comes from the TF-Luna on the head, aimed with the camera — and SLAM runs on the Mac.

Companion to `PLAN.md` (ROS 2 migration) and `HARDWARE-BASICS.md`. This doc covers
only the vision/perception layer: object detection, object tracking, gimbal aim,
distance estimation, approach planning, person identification, and person
following. Drivetrain execution is out of scope — this doc stops at "here is a
goal pose / bearing," never "here is a wheel command."

---

## Part 0 — What this doc changes in `PLAN.md`

Three of PLAN.md's decisions of record do not survive contact with a working
vision layer. They are amended there; recorded here so the reasoning lives next
to the thing that forced it.

| PLAN decision | Change | Why |
|---|---|---|
| **#8 — no local camera CV** | **Partially reversed.** A short-horizon tracker now runs on the Pi, on the 320×240 `lores` stream. | REST answers at 1-2 Hz with a 100-500 ms lag. Between answers there is no new visual information at all, so a "tracker" fed only by detections is pure dead reckoning on a measurement that is already 0.5-1.5 s old. The gimbal would be smoothly, confidently wrong. Bounded to optical flow on `lores` — still no detector, no segmentation, no ML on the Pi. |
| **#5 — one servo (tilt)** | **Extended to two (pan + tilt).** | Reacquisition by turning the whole base is slow and, during a voice session, intrusive. `dtoverlay=pwm-2chan` already gives GPIO12 and GPIO13. Cost is a new error source, contained in Part 4. |
| **#11 — no localization in command mode** | **Unchanged, and now load-bearing.** Approach planning is scoped to the **odom frame only**, bounded at 3 m. | The obvious reading of "ring of candidate poses around the object" needs a map. It doesn't have to. Odometry alone is accurate over 3 m; beyond that you drive partway and re-plan. This keeps SLAM out of command mode, as decided. |

Standing rule #3 ("send 720p") is also revised — see Part 5. Everything else in
PLAN.md holds, in particular:

- One node captures the frame and makes the REST call. Raw frames never cross a
  ROS topic — only results do.
- REST inference is advisory and latency-tolerant. It never sits in a hard
  safety path (the firmware cliff reflex and the lidar obstacle veto stay
  authoritative).
- Reflexes → firmware. Policies that need judgment or iteration → Pi. Anything
  that needs a GPU or a big model → REST, hosted off-Pi.

---

## Part 1 — Where things run

| Capability | Runs on | Why |
|---|---|---|
| Object detection (inference) | **REST** | Needs a real detector (YOLO-class); Pi has no GPU |
| Face embedding (inference) | **REST** | Embedding models want a GPU |
| Face matching (embedding → identity) | **REST** (gallery server-side) | Keeps the Pi from holding a face database; enrollment needs no Pi deploy |
| Monocular depth | **REST**, *fallback only* | Demoted — see Part 3. The lidar is a better range sensor than a monocular model, and you already own it |
| **Short-horizon object tracking** | **Pi**, on `lores` 320×240 | Amends decision #8. The only way the gimbal and follow behavior get information between REST calls |
| Gimbal aim control | **Pi**, `robot` process | Tight loop, sub-100 ms, cannot share a process with anything that makes network calls |
| Range to a detected object | **Pi**, from `/scan` at the detection's bearing | Local, free, 6 Hz, metric, and accurate. Replaces the depth endpoint on the hot path |
| Approach-viewpoint generation & scoring | **Pi** | Pure geometry over `/scan` + object position. Odom frame, no map |
| Path planning (frontier/waypoint) | **Pi** | The custom explorer from PLAN Phase 7, reused |
| SLAM | **Pi**, `slam_toolbox`, **scan mode only** | Unchanged. Not used by anything in this doc |
| Odometry (dead reckoning) | **Pi** | PLAN Phase 6 |
| Cliff / hard-stop obstacle reflex | **Arduino firmware** | Must survive Python/wifi/ROS failure |
| Forward obstacle veto | **Pi**, `obstacle_node` from `/scan` | Unchanged, and it outranks every behavior in this doc |

Rule of thumb, revised: **if it needs a trained model it's REST; if it needs a
live sensor loop it's Pi; if it needs to happen between REST calls it's Pi and it
is deliberately cheap.**

---

## Part 2 — The time and transform contract

This is the part the first draft of this doc was missing entirely, and it is the
foundation everything else stands on.

A bounding box is not a bearing. It becomes one only if you know **when** the
frame was captured and **where the camera was pointing at that instant**. With a
pan/tilt head that slews while a REST call is in flight, the head has moved by
the time the answer comes back. Without this contract, every bearing derived from
a detection is silently wrong by however far the head turned during the call.

### 2.1 Every frame carries a stamp

```python
@dataclass(frozen=True)
class FrameStamp:
    seq:       int      # monotonic per-capture counter
    t_capture: float    # seconds, CLOCK_MONOTONIC
    pan_deg:   float    # commanded gimbal pan at capture
    tilt_deg:  float    # commanded gimbal tilt at capture
    settled:   bool     # head stationary for >= SETTLE_MS before capture
    odom:      tuple    # (x, y, theta) of base_link in odom at t_capture
```

`t_capture` comes from **picamera2's `SensorTimestamp`** in the request metadata,
not from `time.monotonic()` after the capture returns. The difference is the
encode and copy time, which is exactly the error you are trying to remove.

`perception_node` keeps the last 64 stamps in a ring buffer keyed by `seq`. Every
REST request carries its `seq`; every response echoes it. On arrival you look up
the stamp and interpret the result in the world as it was at capture.

### 2.2 The transform chain

```
odom          → base_link       dynamic, from PLAN Phase 6 odometry
base_link     → gimbal_pan      dynamic, rotation about Z by pan_deg
gimbal_pan    → gimbal_tilt     dynamic, rotation about Y by tilt_deg
gimbal_tilt   → camera_optical  static, REP-103 optical frame (z fwd, x right, y down)
base_link     → laser           static
```

`servo_node` publishes the two dynamic gimbal transforms at 20 Hz from the
commanded angles. This makes the vision layer a real consumer of PLAN Phase 6's
tf work — the second caller that justifies the effort.

### 2.3 Bbox → bearing

```python
# fx from camera intrinsics; for a first pass, derive from the published HFOV:
#   fx = (width / 2) / tan(hfov_rad / 2)
u_c = (x1 + x2) / 2
bearing_cam = math.atan2(u_c - cx, fx)      # radians, positive = right
```

Then transform `bearing_cam` from `camera_optical` into `base_link` using the tf
buffer **at `t_capture`**, not at now. `tf2_ros.Buffer.lookup_transform` takes a
time argument precisely for this.

**Calibrate the camera.** Nobody has listed this as a task and it gates the
accuracy of every bearing in this document. The Camera Module 3 standard and wide
variants have very different fields of view, so the published number matters.
Minimum viable: record the horizontal FOV and derive `fx` as above. Proper:
`ros2 run camera_calibration cameracalibrator` with a checkerboard, once, and
store the result in `robot_bringup/config/camera_intrinsics.yaml`.

### 2.4 What servo inaccuracy does and does not break

Hobby servos have no position feedback. Commanded angle is not actual angle,
especially under load or mid-slew. That error lands in `pan_deg`/`tilt_deg` and
flows straight into any bearing you hand the planner.

The saving grace is that **the gimbal control loop does not care**. It closes on
*pixel* error — "move the box toward the centre of the frame" — which needs no
absolute accuracy at all. Absolute angle only matters when converting a detection
into a world bearing for the approach planner or the follow behavior.

So the mitigations are targeted, not global:

- **Settle before capture.** Do not send a frame to `/v1/detect` within
  `SETTLE_MS` (start at 120 ms) of a pan/tilt command. Set `settled=False` on
  those frames and let the tracker use them while the planner ignores them.
- **Slew-rate limit** the gimbal so commanded tracks actual closely.
- **Budget ±3° of bearing error** in the approach planner's scoring rather than
  pretending it is zero. At a 1.5 m standoff that is ±8 cm — tolerable.
- Prefer to compute approach goals when the head is near centre, where the
  linkage is least loaded.

---

## Part 3 — REST endpoints

**Implemented in [`../vision_service/`](../vision_service/README.md)** — that
README is the authoritative contract (every field, every error code, curl
examples); this part is the reasoning behind it. Where the two disagree, the
service is right and this doc is stale.

Externally hosted, on the Mac mini over the LAN. Cloud hosting is available as a
fallback but costs 100-300 ms of extra latency and puts your camera uplink on
your internet uplink; the LAN is the default.

**Transport rules, all four endpoints:**

- **`multipart/form-data`, binary JPEG. Never base64.** Base64 inflates the
  payload by 33% and costs CPU on both ends to encode a 120 KB string into a
  160 KB one. The first draft of this doc showed base64 in every example; that
  was 33% of your wifi budget given away for nothing.
- **One `requests.Session` for the life of the process**, keep-alive on. At 2 Hz
  a fresh TCP (and TLS) handshake per call is a meaningful fraction of the round
  trip.
- **Versioned paths** (`/v1/...`). The gateway client in `perception_node` is the
  only thing that knows them.
- **Every request carries `seq`; every response echoes it.** See Part 2.1.
  A response without a `seq` you recognise is discarded, not guessed at.

### `POST /v1/detect` — object detection

Called continuously by `perception_node` at `DETECT_HZ` (2 Hz).

Request: `image` (JPEG part), `seq`, `min_confidence`.

```json
{
  "seq": 10432,
  "detections": [
    { "class": "person", "confidence": 0.91, "bbox": [x1,y1,x2,y2] },
    { "class": "potted_plant", "confidence": 0.77, "bbox": [x1,y1,x2,y2] }
  ],
  "model_version": "yolov8n-mps",
  "image_size": [640, 360],
  "inference_ms": 18.4
}
```

`image_size` echoes the resolution the server received, so the Pi never has to
assume what space its bbox is in. `inference_ms` is the server's own view of the
call — log it next to the Pi's wall-clock round trip and the difference is your
network, which is worth more than either number alone.

`bbox` is in the coordinate space of the image **as sent**. Send the scale
factor or agree it once — a bbox silently in the wrong resolution is a bug that
looks like bad calibration.

Log `model_version` on change. Note that branching on `"person"` couples you to
the COCO label set; keep that mapping in one place.

### `POST /v1/faces/embed` — face embedding

Called **once per person track**, not per frame. See Part 6.

Request: `image` (JPEG of the cropped head region, from the `main` stream), `seq`.

```json
{
  "seq": 10433,
  "embedding": [0.0123, -0.045, "... 512 floats, L2-normalised ..."],
  "face_bbox": [x1,y1,x2,y2],
  "quality": 0.86,
  "model_version": "insightface-buffalo_l",
  "inference_ms": 11.2
}
```

`quality` lets the caller discard blurry or steeply-angled crops before wasting a
match call. Require `quality > 0.6` before even sending.

### `POST /v1/faces/match` — identity lookup

```json
{ "matches": [ { "label": "roshan", "similarity": 0.89 } ], "threshold": 0.75, "gallery_size": 12 }
```

Below `threshold` (`MATCH_THRESHOLD`), treat as `unknown` — don't let the model force a guess. And
never act on a single frame's match; see the two-agreeing-matches rule in Part 6.

### `POST /v1/faces/enroll` — add a known person

Admin/offline. Not on the hot path.

```json
{ "label": "roshan", "embeddings_added": 4, "images_rejected": 1, "gallery_size": 12 }
```

Images with no detectable face are counted rather than failing the call — one bad
photo in a batch of ten should not lose the other nine.

### `POST /v1/depth` — monocular depth *(fallback only)*

**Demoted from the hot path.** The original design called this to place objects
in space. Two problems:

1. Relative-depth models (MiDaS-class) do not return metres at all, despite the
   endpoint's `depths_m` field name. Metric models (Metric3D, Depth Pro,
   UniDepth) are heavier and still carry 10-20% error indoors.
2. At 3 m, 20% error is ±0.6 m of object-position error feeding a standoff ring
   of radius 1.5 m. The ring ends up in the wrong place and the approach looks
   broken for a reason that has nothing to do with the planner.

**You already own a better range sensor.** The camera gives an accurate
*bearing* (pixel → angle, pure geometry). The LD14P gives an accurate *range* at
that bearing, locally, metrically, at 6 Hz, for free. Fuse them:

```python
def range_at_bearing(scan, bearing_rad, half_width_rad=0.035):   # ±2°
    idx = [i for i, a in enumerate(scan_angles) if abs(a - bearing_rad) < half_width_rad]
    hits = [scan.ranges[i] for i in idx if math.isfinite(scan.ranges[i])]
    return statistics.median(hits) if len(hits) >= 3 else None
```

Call `/v1/depth` only when that returns `None`, or when the lidar range is
implausibly far compared to a bbox-size heuristic — both of which mean the same
thing: **the object is above or below the lidar's horizontal plane** and the
scan is seeing past it. For a person this basically never happens; the lidar sees
legs, which is the range you want.

Request: `image`, `seq`, `points` (bbox centres only, so the server returns sparse
values rather than a full depth map).

```json
{
  "seq": 7,
  "depths_m": [1.8, 3.2],
  "confidence": [0.7, 0.5],
  "metric": false,
  "model_version": "Depth-Anything-V2-Small-hf",
  "inference_ms": 142.0
}
```

Note the shipped default returns `metric: false` — Depth Anything V2 is a
relative model. Swapping in a metric one (Metric3D, Depth Pro, UniDepth) is a
backend change and flips the flag.

The `metric` flag is not decoration — if the server ever falls back to a relative
model, the Pi must know to treat the numbers as ordering rather than distance.

### ~~`POST /v1/segment`~~ — cut

The original doc already suspected this one. Cutting it: it is the only proposed
endpoint whose job the lidar does not already do better, cheaper and offline.
Revisit only if lidar-only obstacle handling produces a felt gap, which it has
not yet had the chance to.

---

## Part 4 — Pi-side nodes and process layout

### 4.1 Process assignment is not optional

PLAN.md isolates the perception process because **network calls hang for
seconds**. The gimbal needs a target at camera rate. Those two facts put them in
different processes, and that constraint decides the whole node layout.

| Process | Vision nodes | Why |
|---|---|---|
| `perception` | `perception_node` (capture + REST + **local tracker**) | Owns the frames. Rule #1 means whatever touches pixels lives here |
| `robot` | `gimbal_node`, `servo_node`, `follow_node`, `approach_planner_node`, `vision_behavior_node` | All light, all latency-sensitive, none touch pixels |
| `voice` | unchanged | Never shares with anything slow |

Inside `perception`, the REST call runs on its **own thread**. If it ran on the
capture thread it would stall the local tracker for the length of a network
round trip, which is the same hazard PLAN Part 9 #3 flags for `voice_node`,
re-entered through a different door.

`perception_node` publishes only small messages: `/detections`, `/people`,
`/target_bearing`, `/perception_status`. No pixels, ever.

### 4.2 `perception_node` — capture, track, and the REST cascade

Owns the camera, the gateway client, the frame-stamp ring buffer, and the local
tracker.

**Capture loop**, `TRACK_HZ` (15 Hz) on the `lores` 320×240 stream. Not 30 — halving the rate
halves the CPU and still gives the tracker seven updates per REST answer.

**The local tracker** (this is the amendment to decision #8). Sparse
Lucas-Kanade optical flow, not an OpenCV tracker object:

```
1. goodFeaturesToTrack inside the current box
2. calcOpticalFlowPyrLK to the next lores frame
3. median translation + median pairwise-distance ratio for scale
4. inlier count below MIN_INLIERS → the track is lost, say so
```

Roughly 60 lines and 2-5 ms per frame on a Pi 4, with the GIL released inside
each OpenCV call. Chosen over CSRT/KCF because it has no tracker-object lifecycle
to manage, it is deterministic and testable against recorded video (Part 8), and
it fails *honestly* — a collapsing inlier count is a real "I lost it" signal,
where a correlation tracker will happily track a patch of carpet forever.

**Re-anchoring is the subtle part.** When a detection for `seq=S` arrives, do
**not** overwrite the current box with it — that box describes the world 0.5-1.5 s
ago, and slamming it in is exactly what makes the gimbal jump at REST rate,
which is the problem the tracker exists to solve. Instead:

```
1. look up the track's own historical box at seq S (keep ~2 s of history)
2. IOU-match the detection against it
3. on match, apply the *difference* as a correction to the current box
4. on no match, open a new track
```

You are correcting accumulated drift, not teleporting to a stale measurement.

**Track state:** `id`, `bbox`, `velocity_px_s`, `class`, `last_anchor_seq`,
`identity` (sticky, see Part 6), `age`, `consecutive_misses`.

### 4.3 The REST backpressure policy

Without this, a wifi hiccup turns into a queue and the robot acts on scene data
from three seconds ago. State it explicitly and implement it exactly:

| Rule | Value |
|---|---|
| In-flight `/v1/detect` requests | **Exactly one.** Never more |
| Frame arrives while a request is in flight | **Dropped, never queued.** Increment `detect_dropped_busy` |
| Hard timeout | `DETECT_TIMEOUT_S = 0.6`. Abandon; do not retry that frame |
| Out-of-order result (`seq` < last applied) | Discard, `detect_out_of_order++` |
| Stale result (older than `MAX_RESULT_AGE_S`, 1.0 s) | Discard, `detect_stale++` — too old to anchor anything |
| N consecutive failures (N = 5) | Publish `/perception_status: DEGRADED` |

The server implements the matching half: inference is serialized through one
worker and requests arriving while `VS_MAX_QUEUE_DEPTH` are already waiting are
refused with `503 busy` + `Retry-After` rather than queued. Treat a `503` as a
dropped frame, not as an error worth retrying.

On `DEGRADED`, `vision_behavior_node` drops to `IDLE/SCAN`, the gimbal holds
position, and the agent is told. Drivetrain, lidar and obstacle veto are
unaffected — perception going dark must never be able to stop the robot working.

Every counter above is published. PLAN standing rule #6: silent discarding reads
as "it's working" when it isn't.

### 4.4 `gimbal_node`

Two-axis pan/tilt. Lives in the `robot` process, subscribes `/target_bearing`,
drives a P (or PI) controller on **pixel error**, not world angle — see Part 2.4
for why that distinction saves you.

- Runs at 20 Hz, matching the servo's own 50 Hz update rate closely enough.
- Slew-rate limited, so commanded angle stays near actual.
- Publishes commanded `pan_deg`/`tilt_deg` back to `perception_node` so they land
  in the `FrameStamp`, and to `servo_node`'s tf publisher.
- Clamps range in the node, never in the caller (PLAN standing rule #4).
- Signals `settled` after `SETTLE_MS` of no commanded motion.

### 4.5 `servo_node`

Extended from PLAN Phase 4's tilt-only design:

- Hardware PWM, **GPIO12 (tilt) and GPIO13 (pan)**, both from
  `dtoverlay=pwm-2chan`, driven via `rpi-hardware-pwm`.
- Both servos on the buck converter's 5 V rail, **not the Pi's**, ground shared.
  A panning head under load plus a tilting head is well past what the Pi's 5 V
  pin will supply without browning out mid-SD-write.
- Publishes the `base_link → gimbal_pan → gimbal_tilt` transforms at 20 Hz.
- Keeps the `/tilt` service and adds `/pan`; or one `/aim` service taking both.
- **Cable routing matters on a rotating head.** The CSI ribbon does not enjoy
  repeated twisting. Limit pan travel to ±90° and give the ribbon a service loop.

### 4.6 `follow_node`

Consumes `/people` (with sticky identity) and the tracker's prediction, publishes
`/follow_target {bearing, distance_m, identity, tracking}`.

The safety design is in Part 7 and it is the part that matters.

### 4.7 `approach_planner_node`

Odom-frame, bounded. Full design in Part 7.

### 4.8 `vision_behavior_node`

`IDLE/SCAN → TRACK_OBJECT → APPROACH_PLAN → FOLLOW_PERSON → LOST/REACQUIRE`,
sitting inside the existing command-mode graph, not a new top-level mode. Drops
to `IDLE/SCAN` on `/perception_status: DEGRADED`.

---

## Part 5 — The bandwidth budget

PLAN standing rule #3 said "send 720p, not 1080p." That was right about 1080p and
too generous about 720p. **Revised rule: match the payload to the model's input
size.**

A YOLO-class detector runs at 640×640 letterboxed. Sending it 1280×720 means
paying wifi for pixels the model discards in its first operation. Send `DETECT_LONG_EDGE_PX` (640) on
the long edge:

| What | Per frame | At 2 Hz |
|---|---|---|
| 720p JPEG q75, base64 *(original design)* | ~160 KB | **~320 KB/s** |
| 640×360 JPEG q75, binary multipart | ~30 KB | **~60 KB/s** |

That is a 5× reduction for a detector accuracy difference you will struggle to
measure on people and furniture at household distances. It matters because it
shares a radio with a continuous Gemini Live audio uplink (~32 KB/s) that has
already died once from contention — see the `audio.error stage=mic-queue` →
`voice.drop` cascade in the logs.

**Capture high, send low, crop from the high one.** Keep `main` at 1280×720
for face crops (a head region is a small crop even from a big frame) and send the
downscaled `lores`-derived frame to `/v1/detect`. picamera2 gives you both sizes
of the same moment for free, and the hardware JPEG encoder does the compression
at zero CPU cost.

Steady-state totals in command mode:

| Stream | Rate |
|---|---|
| Gemini Live audio uplink | ~32 KB/s continuous |
| `/v1/detect` | ~60 KB/s |
| `/v1/faces/embed` | ~15 KB per person *per appearance*, not per frame |
| `/v1/depth` | rare — fallback only |
| **Total** | **~95 KB/s**, down from ~330 KB/s |

---

## Part 6 — Identity is a property of a track, not a frame

The original cascade gated `/v1/faces/embed` on "detect reports a person." But
person-following means a person is in frame *continuously*, so the gate is open
in exactly the case it was meant to protect against. Bind identity to the track
instead:

```
on track creation with class == person:
    wait for a frame where quality > 0.6
    POST /v1/faces/embed  -> embedding
    POST /v1/faces/match  -> candidate label

require 2 consecutive matches above threshold, same label
    -> assign identity to the track, STICKY for the track's life
    -> never re-query

on track loss > REACQUIRE_S:
    the track dies; a new track re-queries from scratch
```

This does three things at once: it bounds embed calls to roughly one per person
per appearance, it removes the single-frame match as a failure mode, and it makes
the similarity threshold far less delicate — two agreeing frames is a much
stronger signal than one, at no extra design cost.

`/people` publishes the bbox immediately and the identity a beat later. Position
lag is unacceptable; identity lag is fine.

---

## Part 7 — Behaviors

### 7.1 Approach planning, in the odom frame

The constraint that makes this work without SLAM is the 3 m bound.

```
Inputs
  bearing θ     from the tracker, de-rotated to base_link at t_capture (Part 2)
  range r       from /scan at θ  (Part 3), or /v1/depth if the scan sees past it
  /scan         current, for clearance
  odom pose     current

Frame:  odom. No map, no localization, no SLAM.
Bound:  MAX_APPROACH_M = 3.0. Beyond that, drive partway, re-detect, re-plan.

Candidates
  N = 12 points on a ring of radius APPROACH_STANDOFF_M around the estimated object position

Score each on
  (a) path clearance from current pose to candidate, in /scan, inflated by robot radius
  (b) line-of-sight clearance from candidate back to the object
  (c) angular spread from viewpoints already visited — spreads coverage rather
      than clustering three near-identical shots
  (d) turn cost from the current heading
  ... with ±3° of bearing uncertainty budgeted in, per Part 2.4

Output: /approach_candidates — ranked {x, y, theta, score}
Re-plan: on arrival, or when the object's range/bearing moves past a threshold
```

Odometry drift over 3 m is small compared to the standoff radius, which is why
the bound is what buys you the freedom from SLAM. This is where the doc's scope
ends; a downstream motion node consumes the top candidate.

### 7.2 Person following, and the obstacle veto

Following someone means deliberately approaching an obstacle, and `obstacle_node`
holds an e-stop veto on obstacles. That conflict needs a designed answer, not a
discovered one.

**The invariant:**

```
MIN_OBSTACLE_M   = 0.45    # obstacle_node's e-stop threshold. AUTHORITATIVE.
FOLLOW_STOP_M    = 0.80    # follow_node stops requesting forward motion
FOLLOW_STANDOFF_M = 1.20   # follow_node's target distance

FOLLOW_STOP_M > MIN_OBSTACLE_M, with margin for closing speed.
```

The margin is not arbitrary. The LD14P scans at 6 Hz — 167 ms between fresh
scans — and a person walking toward the robot at 1.4 m/s covers 23 cm in that
time. `MIN_OBSTACLE_M` must exceed `stopping_distance + 0.25 m`, and
`FOLLOW_STOP_M` must clear `MIN_OBSTACLE_M` by more than one scan period of
closing motion.

**`follow_node` never calls `/estop` and never overrides one.** It simply stops
requesting forward motion below `FOLLOW_STOP_M`. If a person walks into the robot
and `obstacle_node` fires, that is correct behavior, not a bug to work around.

**Recovery from a too-close stop: hold, don't reverse.** Reversing blind is worse
than standing still — the lidar is forward-facing policy and there is no rear
sensor. The robot holds, keeps the gimbal on the person, and reports "too close"
to the agent, which has a voice and can use it.

**On track loss:** `gimbal_node` sweeps the last known bearing ±30°, then ±60°,
for `REACQUIRE_S`. If the person does not reappear, the track dies, identity is
released, and the behavior returns to `IDLE/SCAN`. It does not drive around
looking — searching by driving needs a map.

---

## Part 8 — Testing this without the robot

Almost none of this needs hardware to develop, and building it hardware-first is
the slowest possible path.

**Layer A gets a `robot_core/perception/` package** — no ROS imports, per PLAN
§4.1 — holding the pure functions: the tracker, the cascade and backpressure
policy, bbox→bearing math, the range-at-bearing fusion, and the viewpoint scorer.
All of them are functions over data.

**Record once, replay forever.** A recording session writes a directory of JPEGs
plus a `frames.jsonl` index of `FrameStamp` records and the `/scan` message at
each capture. At 2 Hz that is small enough to keep in the repo for a short clip;
longer ones go to USB, never the SD card.

**A mock REST server replays recorded detections**, keyed by `seq`, with
configurable injected latency and failures. That makes the whole cascade —
including every branch of the backpressure table in Part 4.3 — deterministic and
`pytest`-able on a laptop with no robot, no camera, and no Mac mini.

This is how you tune `APPROACH_STANDOFF_M`, the match threshold, `MIN_INLIERS` and the
tracker parameters without spending the robot's time, and it is the single
highest-leverage item in this document for development speed.

---

## Part 9 — Phases, and what they depend on

These interleave with PLAN.md's phases rather than following them.

| Phase | What | Effort | Blocks on |
|---|---|---|---|
| **V0** | `robot_core/camera.py` — BytesIO, dual stream, hardware encoder, `FrameStamp` with `SensorTimestamp`. Camera intrinsics. | an evening | PLAN Phase 0 item 5, **which is still undone** |
| **V1** | `perception_node`: detect only, backpressure policy, `/detections`. No tracker, no gimbal. | a weekend | V0, PLAN Phase 3 |
| **V2** | Local LK tracker + re-anchoring + `/target_bearing` | a weekend | V1 |
| **V3** | Pan servo hardware, `servo_node` two-axis, tf chain, `gimbal_node` | a weekend | V2, PLAN Phase 4 |
| **V4** | Identity: embed/match, sticky per track, `/people` | an evening | V2 |
| **V5** | `follow_node` + the obstacle-veto invariant | a weekend | V3, V4, PLAN Phase 5 |
| **V6** | `approach_planner_node`, odom-frame | a weekend | V5, PLAN Phase 6 |

**V1 is the phase that validates the risky assumptions** — bandwidth, latency
distribution, and the backpressure policy — before any of the behaviors are built
on top of them. Measure p99 detect latency there, with the voice session live,
the same way PLAN Part 2 measures stopping latency. If it is much worse than
300 ms, every downstream design constant in this doc needs revisiting, and it is
much cheaper to learn that at V1 than at V5.

---

## Part 10 — Summary

**Outsourced to REST:** object detection, face embedding, face identity matching.
Monocular depth as a fallback only.

**Off-the-shelf, run locally:** `slam_toolbox` — and nothing in this doc uses it.

**Custom code on the Pi, all of it:** the REST gateway with its backpressure
policy, the LK optical-flow tracker and re-anchoring, the gimbal P/PI controller,
the frame-stamp/tf plumbing, the range-at-bearing fusion, the approach-viewpoint
scorer, the follow/reacquire state machine, and the vision behavior state
machine. One of those touches pixels; the rest is geometry, control and
bookkeeping.

**Firmware:** unchanged from PLAN.md. The cliff reflex, and nothing else.

---

## Part 11 — Open decisions

- **Where the REST models run.** LAN/Mac mini is the default and the design
  assumes it. Two operational details that are not optional if you rely on it:
  keep the mini awake (it sleeps, and a sleeping perception server looks exactly
  like bad wifi), and keep models resident rather than loading per request.
- **Pan travel limit and ribbon routing.** ±90° is the starting assumption in
  Part 4.5; the real number is whatever the CSI ribbon tolerates in practice.
- **`REACQUIRE_S` and the sweep pattern** in Part 7.2 — tune against recordings
  (Part 8) before tuning against the robot.
- **Enrollment capture:** manual upload, or the robot capturing live on request.
  The endpoint supports either; the interaction design is undecided.

---

## Appendix — Constants introduced here

| Constant | Start at | Set in | Notes |
|---|---|---|---|
| `DETECT_HZ` | 2.0 | `robot.yaml` | REST detect rate |
| `TRACK_HZ` | 15.0 | `robot.yaml` | Local tracker rate on `lores` |
| `DETECT_TIMEOUT_S` | 0.6 | `robot.yaml` | Hard abandon, no retry |
| `MAX_RESULT_AGE_S` | 1.0 | `robot.yaml` | Discard results older than this |
| `DETECT_LONG_EDGE_PX` | 640 | `robot.yaml` | Match the detector's input size |
| `SETTLE_MS` | 120 | `robot.yaml` | No planner-grade capture within this of a slew |
| `MIN_INLIERS` | 8 | `robot.yaml` | Below this, the LK track is lost |
| `MATCH_THRESHOLD` | 0.75 | server | Below this, `unknown` |
| `APPROACH_STANDOFF_M` | 1.5 | `robot.yaml` | Approach ring radius |
| `MAX_APPROACH_M` | 3.0 | `robot.yaml` | Odom-only bound |
| `MIN_OBSTACLE_M` | 0.45 | `robot.yaml` | Authoritative; owned by `obstacle_node` |
| `FOLLOW_STOP_M` | 0.80 | `robot.yaml` | Must exceed `MIN_OBSTACLE_M` with margin |
| `FOLLOW_STANDOFF_M` | 1.20 | `robot.yaml` | Follow target distance |
| `REACQUIRE_S` | 5.0 | `robot.yaml` | Sweep duration before giving up |
