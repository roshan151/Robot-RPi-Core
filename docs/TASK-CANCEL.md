# Cancelling a task — design

Every task the voice agent starts (`enroll_face`, `match_face`, `follow`,
`explore`) runs with the main Gemini Live session **closed**: the task owns the
camera and the head, and nothing may queue moves beside it. Until this change
that also meant nothing was listening — a follow could only end by losing the
person, by timing out (5 min) or by killing the voice node.

Now a task can be cancelled two ways, both feeding one flag. Nothing here runs
speech recognition on the Pi.

| Way | How | Needs | Idle cost on the Pi |
|---|---|---|---|
| **Say it** | "stop", "Robin stop", "that's enough", "quit following me", "come back"… heard by the **cancel listener**, a second, cancel-only Live session that runs only while a task does | Internet (Gemini) | Audio upload only (~32 KB/s), no on-device model |
| **Call the service** | `ros2 service call /task/stop std_srvs/srv/Trigger` from an SSH session on the Pi, or any machine on the home network with ROS (later: a Stop button in the web app) | Home network only, no internet | None: it waits for a call |

A physical button was considered and left out for now; it would be one more
setter of the same flag.

## Part 1 — The shared cancel flag

`robot_core/task_cancel.py`: one `threading.Event` for the process.

* `_run_task` (in `live/agent.py`) **clears** it as a task starts.
* The listener's `cancel_task` and the `task/stop` service **set** it and brake
  at once (`motion.stop()`, the out-of-band e-stop), so nothing waits for a
  task loop to notice.
* The tasks **check** it between steps:

| Task | On cancel |
|---|---|
| `follow` | Ends after the current step; a turn or drive in progress is braked and the next one is never sent. OLED: `follow cancelled` |
| `enroll_face` | Ends at once; nothing is saved. OLED: `enroll cancelled` |
| `match_face` | Ends at once, nobody matched. OLED: `match cancelled` |
| `explore` | The voice node calls `explore/stop`: the mission ends after the current step and **the map is saved** |

A cancelled task is answered with a **nod** ("yes, stopped"), whatever the task
had reached; `logs.json` has `task.cancel` (by `voice` or `service`) and
`task.end … cancelled: true`.

## Part 2 — The cancel listener

`robot_core/live/cancel_watch.py`. Started by `_run_task` beside the job,
shut down when the job ends (which frees the microphone before the main
session reconnects, exactly as before).

* **Reuses `LiveAgent`** (microphone, uplink, server-side voice detection, tool
  calls) with its own prompt (`ROBOT_CANCEL_PROMPT`), one tool (`cancel_task`)
  and no "connected" chime. `voice.connect` in the log carries `role: cancel`.
* **Only one tool**: no drive/turn, head, gesture, `run_task` or power-off, so
  it cannot fight the task for the wheels or the head. It has no motion
  backend, so the task's own moves are never reported to it.
* **Leaves the main session alone**: the main loop (`_run_supervised`, its
  braking reconnects, `_pump_results`) is unchanged.
* **Its failures are its own**: if it drops or cannot connect, it logs
  `task.error "cancel listener: …"` and reconnects with the usual backoff. It
  never brakes and never speaks, so a network blip does not stop a follow — but
  until it is back, `task/stop` is the way to cancel. A rejected configuration
  is not retried.
* **Prompt bias**: told that stopping by mistake is harmless, so it cancels when
  in doubt, but not for "don't stop", "keep going" or speech not addressed to
  the robot.

### Timing

* Connects in parallel with the task (~0.5–2 s, measure: `task.start` → the
  `voice.connect role=cancel` after it). The task does not wait for it; during
  that window only `task/stop` works.
* From the end of "stop" to the brake: ~1–2 s, most of it Gemini deciding you
  have finished speaking. The brake itself is immediate once the call arrives.
* The main session's reconnect after the task is unchanged.

### Cost and limits

* Audio is billed for the length of each task, and noise the server takes for
  speech starts a turn. Small for a 5-minute follow.
* Live sessions have a length limit (~15 min of audio). Fine for follow and the
  face tasks; a long explore may outlast one session and the listener simply
  reconnects (a few seconds without voice cancel), until session resumption is
  added.
* Motor noise: the server's recogniser copes far better than an on-device one,
  but a word lost under a drive is possible. Follow is stop-and-go, so saying it
  again usually lands while the robot stands still.

## Part 3 — Settings

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_CANCEL_LISTEN` | `1` | `0` turns the listener off; `task/stop` still works |
| `ROBOT_CANCEL_PROMPT` | see `settings.py` | What the listener is told; tune the trigger phrases here |

Same model and API key as the main session (`GEMINI_LIVE_MODEL`, `GEMINI_API_KEY`).

## Part 4 — Trying it

1. "Follow me", walk a few steps, then say "Robin stop". The robot brakes,
   nods, the OLED shows `follow cancelled`, and the main session reconnects.
2. Start a follow, then on the Pi:
   `ros2 service call /task/stop std_srvs/srv/Trigger` → same result.
3. "Go explore", then "stop": the mission ends after its current step and the
   map is on the Mac.
4. In `logs.json`: `task.start`, `voice.connect role=cancel`, `voice.heard`,
   `task.cancel by=voice`, `task.end cancelled=true`.
