# Hardware diagnostics

Scripts, not pytest tests — each needs a robot on the other end of the serial
port, and each is run by hand. They are named `check_*` rather than `test_*`
precisely so `pytest` does not try to collect them.

| Script | What it tells you |
|---|---|
| `check_protocol.py` | Frame encode/decode and checksums, no robot needed |
| `check_encoders.py` | **Run this after any harness change.** Hand-roll each wheel forward: both counters must go *up*, and rolling the left wheel must move `enc_left`. It is the only way to tell a swapped A/B pair from a mirror-mounted motor |
| `check_movements.py` | Closed-loop moves end to end |
| `check_timed.py` | Timing and sync-error behaviour across a run |
| `check_vision_stack.py` | **Run on the Pi.** Camera + vision service + local tracker without ROS: capture rate, detect p50/p95/p99, gateway counters. `--servos` sweeps the pan/tilt head so you can set trim/invert |
| `check_vision_service.ipynb` | **Run from the Pi.** Pings every vision-service endpoint on the Mac mini, verifies the response contract, and measures p50/p95/p99 detect latency. Needs the service up, not the robot |

```bash
python tests/hardware/check_encoders.py
```

Order matters: encoders first. Calibration numbers mean nothing until
`check_encoders.py` passes all four of its checks.

## The vision service check

The odd one out: a notebook, and the thing on the other end is the Mac mini
rather than the Arduino.

```bash
pip install requests pillow notebook
jupyter notebook tests/hardware/check_vision_service.ipynb
```

Set `BASE_URL` in the first cell to the mini's address, then Run All. It prints a
pass/fail summary and diagnoses the common failures (mini asleep, wrong address,
payload too large for the radio).

This is the Phase V1 diagnostic from `docs/VISION-DESIGN.md` — the phase whose
job is to measure real bandwidth and p99 latency before anything is built on
them. **Run it twice:** once on an idle Pi, then again with the voice session
live and the robot driving. The second number is the one the design constants
have to survive.
