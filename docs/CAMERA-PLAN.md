# Camera upgrade — phased plan

Why: "detect Roshan and follow" answered **no**. The camera was not the root
cause: on the saved photos the face model (buffalo_l) scored Roshan **0.40–0.70**
against his own enrolment when the light or pose changed, under a match cut-off
of **0.75**. Only near-duplicates of the enrolment frames cleared it. The
enrolment itself was poor: in all 16 photos the top of the face was cut off by
the frame (box tops 51–111 px above the image), all in one pose, and backlit
(top third of the frame at ~20/255). Follow could only see a standing face
between ~1.5 m and ~2.5 m.

So: Phase 0 fixes the software for free, Phase 1 adds the HD camera for faces
and follow, Phase 2 adds the mono global-shutter camera for odometry, SLAM and
obstacles. **There is no 2-D lidar in this plan** — the LD14P is dropped (see
PLAN.md Phase 5). The TF-Luna on the head stays.

This amends `FOLLOW-DESIGN.md` (identity while following) and
`VISION-DESIGN.md` (`MATCH_THRESHOLD`), the way `EXPLORE-DESIGN.md` does.

| Phase | Hardware | What it buys | Status |
|---|---|---|---|
| 0 | none (Camera V2 on the ribbon) | Recognition that works across light and pose; enrolment of the whole face | **Implemented** |
| 1 | USB HD camera (innomaker PS5268, 1080p30, 120 dB HDR, 121° D) on the gimbal | HDR faces, ~1.7× wider view, faces from ~1.0 m to ~2.8 m; enrol auto-framing; follow keeps the person when their back is turned | **Implemented** (code); swap the camera to use it |
| 2 | Mono global-shutter camera (innomaker OV9281, 1280×800, 118° H) on the ribbon, fixed to the chassis | Visual odometry, SLAM, obstacle avoidance without a lidar | Planned |

---

## Phase 0 — recognition that works (no new hardware)

### What changed

| Where | Change | Why |
|---|---|---|
| `vision_service/settings.py` | `VS_MATCH_THRESHOLD` **0.75 → 0.45** | Same-person cosine across light/pose is ~0.4–0.7 for buffalo_l. Follow still needs 2 agreeing frames to lock (VISION-DESIGN Part 6), which is what makes the lower cut-off safe |
| `vision_service/settings.py`, `backends/real.py` | Face detector input **320 → 640** (`VS_FACE_DET_SIZE`) | Needed for 1080p frames (Phase 1): in a 1920 frame a 60 px face was missed at 320 and found at 640 (80 px and up: found by both) |
| `backends/real.py` `_face_quality` | Size scored in **pixels** (112 px = full marks), sharpness judged at 112 px | The old score was "share of the frame", which punishes every face in a 1080p frame |

### What you do

1. `make restart` on the Mac (new defaults; nothing to put in `.env`).
2. Drop the bad enrolment: `curl -X DELETE http://<mac>:8080/v1/faces/Roshan`.
3. Re-enrol by voice ("learn my face, I'm Roshan"). Stand ~1.5 m away, lit from
   the front (not a window behind you), and turn your head a little while it
   captures. The head now frames your face itself (Phase 1 code, works on the
   V2 too) and only keeps shots with the whole face in view.
4. Baseline test: "who am I" / "follow me" at 1.2, 2 and 3 m, front-lit and
   backlit. Note the match similarity in the Mac log. If strangers ever reach
   0.45, raise `VS_MATCH_THRESHOLD`; if you sit just under it, lower it.

---

## Phase 1 — USB HD camera for faces and follow

### Hardware swap checklist

1. Mount the USB camera on the gimbal, with a slack loop of thin USB cable (the
   SG90s are weak). Remove the V2 and its ribbon from the head — the ribbon is
   what kept snapping.
2. `robot.yaml` → `/explorer`:
   - `face_camera_size: [1920, 1080]`
   - `hfov_deg:` measured. Stand a 1 m wide target at distance d so it just
     fills the width: `hfov = 2·atan(0.5 / d)`. Expect ~105°.
   - `camera_inverted:` as mounted (true = upside down; flipped in software on USB).
   - `camera_backend: "auto"` already opens the USB camera when no ribbon
     camera answers. Set `"opencv"` in Phase 2.
3. Re-enrol (step 2–3 above): new sensor, new colours.

### What changed

| Where | Change |
|---|---|
| `robot_core/sensors/camera.py` | USB path asks for **MJPEG** (1080p30 fits USB 2.0) with a **1-frame buffer**, and drops the queued frame before each capture, so a picture taken after the head moves is from after it moved |
| `robot.yaml`, `voice_node.py`, `explorer_node.py` | `camera_backend`, `camera_index`, `face_camera_size`: one place says which camera is the head camera, for both the voice tasks and explore |
| `settings.FACE_WIRE_LONG_EDGE` (1920) | Faces and follow frames are sent at full resolution, not shrunk to 640. With the wide lens a face at 2 m is ~18 px at 640 and ~55 px at 1920 |
| `robot_core/face_tasks.py` **enrol auto-framing** | The head starts at `FACE_ENROLL_TILT_DEG` (20°), re-aims at the face whenever it is more than `FACE_FRAME_DEADBAND` off centre or cut by the frame edge (the detector's box runs past the edge, so its centre still points the right way), and keeps a shot only if the whole face is `FACE_EDGE_MARGIN` inside the frame. Never sweeps: the person comes to the robot. OLED shows "whole face not in view" while it is cut off |
| `vision_service/follow.py` **identity from behind** | See below |

### Follow: knowing who it is when their back is turned

The lock is unchanged: 2 frames in a row where the face matches the target. After
that, identity is carried by **continuity**, not the face:

1. **Where they should be.** After each tracked step the Mac stores the
   expected bearing for the next frame: last bearing + the turn it just told
   the robot to make (turning right moves every bearing left). A gate of 30°
   around it, wider by 15° per missed frame.
2. **Alone in the gate = still them**, even if their look changed (they turned
   round, a backpack appeared), as long as the clothes match ≥ 0.35. That new
   look is **learned**.
3. **A bank of looks.** Clothes are two colour histograms, torso and legs. The
   bank keeps up to 8 looks (front, side, back, ...); the first, seen with the
   face, is never dropped. A look nearly identical to a kept one refreshes it
   (lighting). Matching is against the best look in the bank.
4. **Several people in the gate:** best clothes match (≥ 0.5) plus a bonus for
   being nearer the expected bearing. It is learned only if clearly better than
   the runner-up.
5. **Lost, then seen anywhere else:** taken back only on a strong match (≥ 0.6)
   to a look in the bank — so a back view learned earlier lets the robot
   recognise them from behind after losing them. Nothing is learned here.
6. **Who it is not.** Every person's head crop still goes to the face model.
   Another enrolled person's face, or a clear face that scores < 0.25 against
   the target (`VS_FACE_REJECT_THRESHOLD`, "stranger"), excludes that person from
   clothes tracking — a lookalike in the same shirt is never followed once their
   face is seen. The target's face always wins and is always learned.

Known limit: if the target leaves view and a faceless bystander in similar
clothes steps into the gate, the robot can take them until it sees their face.

---

## Phase 2 — mono global-shutter camera: odometry, SLAM, obstacles (plan)

### Hardware

- innomaker CAM-MIPIOV9281 on the ribbon (CSI) port, **fixed to the chassis**,
  facing forward, slightly down. Raspberry Pi OS driver: `dtoverlay=ov9281`.
- Head camera stays on USB: set `camera_backend: "opencv"`.
- Global shutter = no rolling-shutter skew while the robot turns; mono = more
  light per pixel. 640×400 at 30–60 fps is plenty.
- Measure its height and pitch; calibrate the 148° D lens with a checkerboard
  (`cv2.fisheye`) once.

### What it is for

| Job | How | Where it runs |
|---|---|---|
| **Visual odometry** | FAST corners + LK optical flow on 640×400, essential-matrix rotation; wheel ticks give the scale (monocular has none). Disagreement between wheels and camera = slip/stall, feeding `/motion_health` | Pi (light; VISION-DESIGN already allows optical flow on the Pi) |
| **Obstacle avoidance** (replaces the LD14P `obstacle_node`) | Fixed camera at known height and pitch: each image row below the horizon is a known floor distance. Flag rows where the floor's appearance breaks (edges/texture from the floor model) or optical flow expands (time-to-contact) inside the drive corridor. Veto drives below `MIN_OBSTACLE_M`. TF-Luna on the head confirms range ahead; `/v1/depth` on the Mac for keyframes | Pi for the veto (must not depend on Wi-Fi); Mac for depth |
| **SLAM** | Keyframes (every ~0.3 m or 15°) with the VO + wheel pose go to the Mac. Mono SLAM with wheel-odometry scale and loop closure by place recognition, feeding the existing `vision_service/slam.py` map, replacing the head-sweep scans | Mac |
| Motion check | Move the existing `visual_motion` check from the head `lores` stream to this camera: it no longer moves with the gimbal | Pi |

### Exit criteria

- VO + wheels: < 5% distance error and < 5° heading error over a 5 m loop.
- The robot refuses to drive into a box, a chair leg and a wall, and says why.
- A room map that closes its loop and lets explore return to the start pose.

### Pi 4B budget

One ribbon port (mono) + one USB camera (HD) is fine: the Pi does capture, LK
flow and the obstacle veto; models stay on the Mac. Grab HD frames only when a
task needs them (no continuous 1080p decode); MJPEG can be forwarded without
re-encoding. Stereo would need two ribbon ports (Pi 5 or a multiplexer).
