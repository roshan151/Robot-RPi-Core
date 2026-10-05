# The hardware layer, from scratch

Everything from the design review, explained assuming no robotics or Raspberry Pi
background. You know Python and you know agents; this fills in the layer below.

---

## Part 1 — Why your stop button can be slow

### Processes and threads

A running program is a **process**. It has its own private memory. Your robot is
one process today.

Inside a process you can have several **threads** — workers that run at the same
time and share that memory. You already use these: one thread reads the serial
port, one runs the motion queue, one handles the microphone. Sharing memory makes
them easy to write, because they can just read each other's variables.

### The catch: Python's GIL

Python has a rule called the **Global Interpreter Lock**, or GIL:

> Inside one Python process, only **one** thread can run Python code at a time.

Not one per core. One, total — even though your Pi has four cores. Python
switches between threads very quickly so it *looks* simultaneous, but it never
actually is.

**The kitchen analogy.** Picture a kitchen with five cooks and exactly one knife.
Only the cook holding the knife can chop. They hand it around fast enough that
the kitchen looks busy, but if one cook grabs the knife and spends 200 ms dicing
an onion, the other four stand still and wait.

There's a second thing that stops everyone: Python periodically pauses to clean
up memory it no longer needs (**garbage collection**). That pause freezes *every*
thread in the process. Usually a few milliseconds. Occasionally much longer.

### Why this matters for obstacle avoidance

You've decided the obstacle logic runs on the Pi in Python. So the sequence is:

```
TF-Luna sees a wall → Python thread notices → Python decides "stop"
                    → writes to serial → Arduino stops the motors
```

Every one of those middle steps needs the knife. If the audio thread is mid-chop
when the lidar sees the wall, your stop *waits its turn*. That waiting is extra
distance travelled.

And this isn't hypothetical for you. Your `robot_tools.py` docstring documents
exactly this failure already: a slow operation in the receive loop stalled the
microphone uplink, audio got dropped, and the session died. Same mechanism, and
last time it only cost you a voice session. Now it would cost you stopping
distance.

### The fix: separate processes

Give each job its **own process**. Each process gets its own GIL — its own knife
— and Linux schedules them across your four cores genuinely in parallel. The
audio process can be busy for 200 ms and the obstacle process doesn't notice.

The cost: separate processes don't share memory, so they have to *send messages*
to each other instead of just reading each other's variables. That's precisely
what ROS topics and services are for, and it's the real reason ROS's extra
~120 MB is worth paying here.

Suggested split:

| Process | Contains | Why |
|---|---|---|
| `robot` | drivetrain, TF-Luna, obstacle policy, e-ink, battery | All light work. Nothing here can stall the stop |
| `perception` | camera capture, REST calls to the vision service | Network calls can hang for seconds. Isolate them |
| `voice` | the Gemini Live session | Needs steady timing, and it's the biggest CPU user |

The rule in one line: **the thing that stops the robot must not live in the same
process as anything slow.**

### Telling Linux what matters

Linux decides which process runs when. By default everything is equal. You can
say otherwise:

- **`nice -n -10 <command>`** — "prefer this process." Mild, safe, usually enough.
- **`SCHED_FIFO`** — real-time priority: "when this wants to run, run it now,
  ahead of nearly everything." Powerful, and mildly dangerous — a real-time
  process stuck in a tight loop can lock up the machine.

For a loop that reads a distance sensor 20 times a second, start with `nice`.
Only reach for `SCHED_FIFO` if measurement says you need it.

---

## Part 2 — How to know if it's fast enough

### What latency actually is

**Latency** here means the gap between "the sensor sees the wall" and "the motors
actually stop." It is not one number. It's different every single time,
depending on what else the Pi happened to be doing.

If you measure it 1000 times you get a spread of values — most clustered around
some typical value, a few much worse.

### p50, p99, and why you care about the tail

- **p50** (the median): half of measurements are faster than this. The typical case.
- **p99**: 99% of measurements are faster than this. The bad-but-not-freak case.

**Use p99, not the average.** Your robot doesn't hit the furniture on an average
day. It hits it on the one-run-in-a-hundred where something else was hogging the
CPU. Averages hide exactly the cases that break things.

**The commute analogy.** Your drive to work averages 20 minutes, but once a week
it's 45. You don't leave 20 minutes early — you leave 45 minutes early, because
the bad day is the one that makes you late.

### Actually measuring it

Not hard. Timestamp both ends and collect the difference:

```python
import time

samples = []
# ... in your obstacle loop, when a range reading says "too close":
t0 = time.monotonic()
drivetrain.emergency_stop()
samples.append(time.monotonic() - t0)

# after ~500 stops:
samples.sort()
p50 = samples[len(samples) // 2]
p99 = samples[int(len(samples) * 0.99)]
print(f"p50={p50*1000:.0f} ms   p99={p99*1000:.0f} ms")
```

Run those 500 stops **while the robot is doing everything else it normally does**
— voice session live, camera posting, all of it. A latency measured on an idle
Pi tells you nothing about the day it matters.

### Turning that into a speed limit

```
stopping distance = speed × p99 latency
```

| Speed | p99 | Distance travelled after the sensor fires |
|---|---|---|
| 0.3 m/s (slow walk) | 100 ms | 3 cm — fine |
| 0.5 m/s | 300 ms | 15 cm — not fine |

If p99 comes out bad, **slow the robot down.** Lower `DEFAULT_SPEED_PERCENT` and
the problem is solved permanently, in one line, with no cleverness required.
Trying to make Python reliably fast is a much worse use of your weekend.

One thing already in your favour: `emergency_stop()` writes to the serial port
*out of band* — it skips the lock that a running move is holding. So the last
step of the chain is already as fast as it can be. What you're measuring is
everything queued up in front of it.

---

## Part 3 — How the Pi talks to chips

Along one edge of the Pi are 40 pins. Most are **GPIO** ("general purpose
input/output") — you can turn them on and off from software. But raw on/off is
tedious, so the Pi has built-in hardware for several standard *protocols*. Each
sensor supports specific ones; they're not interchangeable.

### The four you'll use

**UART** — "serial." Two wires: TX (transmit) and RX (receive). Strictly
point-to-point: one device per UART. Both ends must agree on a speed, the **baud
rate** (e.g. 115200). Your Arduino link is a UART, tunnelled inside the USB
cable.

**I²C** — two wires, `SDA` (data) and `SCL` (clock), shared by *many* devices.
Each device has an **address**, a number like `0x10`. Like a party line: you call
out "device 0x10, what's your reading?" and only that one answers. Slower than
the others, but you can hang a dozen sensors off two pins.

**SPI** — four wires, much faster, plus one "chip select" wire per device. Used
when you need to push a lot of data. Your e-ink screen uses it because a screen
image is a lot of bytes.

**PWM** — not really a bus. One wire that the Pi switches on and off very fast.
The *ratio* of on-time to off-time carries the information. Servos read it to
decide their angle.

### Why TF-Luna should use I²C, not UART

The TF-Luna can speak either. Use I²C, for a specific and annoying reason.

The Pi 4 has two UARTs: a good one (called PL011) and a cheap one (the "mini
UART"). **Bluetooth claimed the good one** — and you use Bluetooth audio. So if
you wire the TF-Luna to the UART pins, you get the cheap one.

The cheap UART's timing is derived from the CPU clock speed. Your CPU clock
changes constantly, based on load and temperature. So the serial timing drifts,
and bytes arrive garbled — sometimes. Intermittently. Under load. That is the
single worst category of bug to chase on a robot.

I²C has its own clock and doesn't care. Wire it to I²C, address `0x10`, and the
whole problem never exists.

Before you rely on it, scan the bus:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

That prints a grid of every address that answers. Run it before plugging in the
TF-Luna and again after — `10` should appear. Your PiSugar battery board is
already on this bus at a different address, which is fine; that's what I²C is
for. You're just checking nothing collides.

---

## Part 4 — Servos, and why "hardware PWM" matters

### How a servo works

A servo doesn't take an angle as a number. It watches a pulse on its signal wire:

```
     ┌──┐              ┌──┐              ┌──┐
─────┘  └──────────────┘  └──────────────┘  └────
     │←→│                                        
     1–2 ms pulse       every 20 ms
```

A **1.0 ms** pulse means one end of the range, **1.5 ms** is centre, **2.0 ms**
is the other end. It re-reads the pulse ~50 times a second and holds position.

The important consequence: **the servo's accuracy is your timing accuracy.** If
your pulses are 0.2 ms too long sometimes, the head twitches.

### Software PWM vs hardware PWM

**Software PWM** means the CPU generates those pulses — a loop that turns the pin
on, waits, turns it off. But your CPU is busy running an audio stream, a network
connection, and an LLM session. When it's late, the pulse is the wrong length,
and the camera head jitters. Not a crash — just a head that never sits still.

**Hardware PWM** means a dedicated circuit inside the Pi's chip generates the
pulses. You tell it "1.5 ms pulses at 50 Hz" once and it does that forever,
perfectly, using **zero CPU**. Nothing your software does can disturb it.

Use hardware PWM. It isn't on by default:

```bash
# add to /boot/firmware/config.txt, then reboot
dtoverlay=pwm-2chan
```

That turns GPIO12 and GPIO13 into hardware PWM outputs. Drive it from Python with
the `rpi-hardware-pwm` package.

**Avoid GPIO18** — it can do PWM, but it's also the I²S audio clock pin, and you
are doing audio.

**Also note:** `RPi.GPIO`, the library in most older tutorials, is no longer the
right choice on current Pi OS. Use `gpiozero` (with the `lgpio` backend) for
ordinary GPIO work.

### Powering the servo — the classic beginner mistake

**Do not power a servo from the Pi's 5 V pin.** A servo under load pulls far more
current than the Pi can supply. What happens isn't a clean failure: the Pi's
voltage dips, and it reboots or corrupts the SD card mid-write.

Give the servo its own 5 V supply (or a buck converter off your LiPo, like the
motors have), and **connect its ground to the Pi's ground.** Signal wires need a
shared ground reference to mean anything — the same star-ground principle already
in your README for the motors.

Only three wires go to the servo: power, ground, and the signal from GPIO12.

---

## Part 5 — Reading the pin table

```
Function          Pins
────────────────────────────────────────────────────────
E-ink (SPI0)      GPIO 8 (CE0), 9 (MISO), 10 (MOSI), 11 (SCLK)
                  + GPIO 25 (DC), 17 (RST), 24 (BUSY)
TF-Luna (I2C1)    GPIO 2 (SDA), 3 (SCL)   — shared with PiSugar
Servo (PWM0)      GPIO 12
Camera            CSI ribbon connector — not GPIO at all
Arduino           USB
```

The assignments aren't arbitrary. Most GPIO pins are interchangeable, but the
built-in SPI, I²C and PWM controllers are **physically wired to specific pins**
inside the chip. If you want hardware SPI, it comes out on 8/9/10/11. That's why
these tables exist: to check nothing wants the same pin twice.

Reading the e-ink row: SPI itself needs four wires, but the display needs three
more ordinary GPIO pins for things SPI doesn't cover —

- **DC** (data/command): "is this byte an instruction, or picture data?"
- **RST** (reset): a wire to reboot the panel.
- **BUSY**: the panel's way of saying "still refreshing, don't send more yet."

The camera is the odd one out — it doesn't touch the 40-pin header at all. It has
its own flat ribbon connector wired straight into the Pi's image processor,
which is why it can move image data far faster than any GPIO bus.

**The verdict on this layout:** nothing collides, and you'll have around 15 GPIO
pins spare.

### One more thing that isn't a pin: e-ink is slow

E-ink holds its image with no power, which is lovely, but changing it is slow —
a **full refresh takes 2 to 15 seconds**, a partial refresh maybe 0.3 to 1
second. The screen must live where it can take its time without holding anything
else up, and if emotions arrive faster than it can draw them you want it to
**skip to the newest one** rather than work through a backlog. A face that's
thirty seconds behind the conversation is worse than no face.

---

## Part 6 — Two camera streams

`picamera2` can hand you the same moment at two different sizes simultaneously,
at no extra cost, because the camera hardware does the resizing on its way in:

```python
cfg = picam2.create_video_configuration(
    main={"size": (1280, 720)},    # the one you POST to the vision service
    lores={"size": (320, 240)},    # a cheap one for anything done on the Pi
)
```

### Why size matters so much

Cost scales with pixel count, and pixel count scales with *area*:

| Resolution | Pixels | Relative cost |
|---|---|---|
| 1920×1080 ("1080p") | 2,073,600 | 27× |
| 1280×720 ("720p") | 921,600 | 12× |
| 320×240 | 76,800 | 1× |

Halving width and height quarters the work. That's why 320×240 is the standard
choice for anything the Pi computes itself.

### Why not to send HD over the network

Your wifi is already carrying a **continuous** microphone stream to Gemini. That
stream is fragile — if it stutters, audio gets dropped and the session dies. You
have this in your logs already.

Now add photos:

| What you send | Size each | At 2 per second |
|---|---|---|
| 1080p JPEG | ~400 KB | ~800 KB/s |
| 720p JPEG | ~120 KB | ~240 KB/s |

The 1080p option competes with your audio for the radio, and audio loses. Send
720p or smaller.

**"HD camera" describes the sensor, not the payload.** Owning a high-resolution
camera and sending small pictures are two independent decisions. Capture high,
send low.

### And stop writing files

Your current code saves each photo to the SD card, reads it back, encodes it,
then uploads it — four times a second. Capture into memory instead
(`io.BytesIO`), and let the Pi's built-in JPEG hardware do the compression rather
than doing it in Python. Faster, and it stops slowly wearing out your SD card.

---

## Part 7 — The question you still need to answer

You said obstacle avoidance uses "HD camera + TF-Luna." Those two are very
different tools and it matters which one is actually doing the stopping.

**Sending a photo to a server and waiting** takes 100–500 ms, and returns nothing
at all if the wifi drops. Think about what that means at 0.4 m/s: by the time the
answer arrives you've travelled 20 cm, and on a bad wifi day the answer never
arrives and the robot just keeps going.

So they should be two different features with two different jobs:

| | TF-Luna | Camera via REST |
|---|---|---|
| Speed | ~10 ms | 100–500 ms |
| Works offline? | Yes | No |
| Job | **Stopping** | **Understanding** |
| Sees | one narrow point ahead | the whole scene, with meaning |

TF-Luna stops the robot. The camera tells the agent *what's there* so it can
decide what to do next. Don't ask the camera to stop anything.

**The third option** — real obstacle detection from the camera, on the Pi, using
the 320×240 `lores` stream — is possible, and it's the only item on your entire
list with meaningful CPU cost. Worth deciding on purpose now rather than
discovering halfway through.

### And one limitation of the TF-Luna itself

It measures **one point**, in a beam about 2° wide. It's a tape measure pointed
straight ahead, not a field of view. It will cleanly miss a chair leg 20 cm to
the left.

But it's mounted on your new tilting camera bracket — so **tilt it down and it
becomes a cliff detector**, spotting the top of a staircase or the edge of a
table. That's the failure that actually destroys robots, and you'd get it nearly
free. Good reason to build the servo before the camera work.
