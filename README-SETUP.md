# Setting up a new Pi

Follow the steps in order. Steps 1 and 2 are the only ones that take real time; the rest is copy-paste.

**You need:** a Raspberry Pi 4B running **64-bit Raspberry Pi OS Trixie** (Lite is fine), on your network, with internet. Check the OS with:

```bash
dpkg --print-architecture     # must print: arm64
. /etc/os-release && echo $VERSION_CODENAME    # must print: trixie
```

ROS 2 Jazzy has no 32-bit build, so a 32-bit or Bookworm image will not work

## 1. Get the code

```bash
sudo apt update && sudo apt install -y git
git clone <your-repo-url> ~/Robot-RPi-Core
cd ~/Robot-RPi-Core
```

## 2. Run the installer

```bash
./install_pi.sh
```

It takes a while (ROS plus the Python packages). It is safe to re-run, for example after a `git pull`. It does everything below for you:

1. Adds the **ROS 2 Jazzy apt repository** and installs ROS (see "About ROS on the Pi").
2. Installs every other apt package (camera, OpenCV, audio, Bluetooth).
3. Creates `.venv` (with access to the apt camera libraries), installs the Python packages, and checks they import.
4. Builds the ROS workspace (`ros2_ws`).
5. Adds your user to the `dialout`, `audio` and `bluetooth` groups and enables lingering (so audio works at boot).
6. Turns on the head-servo PWM overlay in `/boot/firmware/config.txt` and the hardware serial port for the TF-Luna.
7. Installs the `robot-voice` boot service, filled in with your username and this folder. It is **not started** yet.

If it stops with an error, fix the cause and run it again; finished steps are skipped.

## 3. Put your keys in `/etc/robot.env`

The installer cannot do this because it is your secret.

```bash
sudo nano /etc/robot.env
```

Paste (plain `KEY=value` lines, no quotes):

```
GEMINI_API_KEY=your-key-here
VISION_SERVICE_BASE_URL=http://mini.local:8080
BT_MAC=AA:BB:CC:DD:EE:FF
```

`VISION_SERVICE_BASE_URL` is the Mac mini running the vision service (step 6). `BT_MAC` is optional until you pair the buds (step 4). Then lock the file down:

```bash
sudo chown root:$USER /etc/robot.env && sudo chmod 640 /etc/robot.env
```

Never commit a real key.

## 4. Pair the Bluetooth buds (once)

Put the buds in pairing mode, then:

```bash
bluetoothctl
  power on
  agent on
  default-agent
  scan on                      # note the buds' MAC address, e.g. AA:BB:CC:DD:EE:FF
  pair AA:BB:CC:DD:EE:FF
  trust AA:BB:CC:DD:EE:FF      # so they reconnect by themselves at boot
  connect AA:BB:CC:DD:EE:FF
  exit
```

Put the same MAC in `/etc/robot.env` as `BT_MAC`. The wideband-microphone tweak is in [README.md, Audio](README.md#bluetooth-buds-microphone--speaker).

## 5. Reboot, then start the robot

```bash
sudo reboot
```

The reboot is what applies the new groups, the PWM overlay and the serial port. After it, with the buds on and nearby:

```bash
cd ~/Robot-RPi-Core
./start_robot.sh                           # try it by hand first (Ctrl+C to stop)
sudo systemctl enable --now robot-voice    # then start it now and at every boot
```

Stop the service before running `./start_robot.sh` by hand (`sudo systemctl stop robot-voice`); two copies fight over the serial port and microphone. More service commands are in [README.md](README.md#boot-service-stop-run-manually-re-enable).

## 6. The other two machines

**Mac mini (vision service).** In the `Vision-Microservice` folder on the Mac: `make deploy` (creates the venv, installs, runs tests, starts it), then `make status`. If the robot seems blind, check `GET /healthz`. For `/v1/ask`, set `GEMINI_API_KEY` on the Mac and/or run `ollama pull qwen3-vl:8b`.

**Arduino (drivetrain firmware).** Plug the Arduino into the **Mac** and run, once: `brew install arduino-cli && arduino-cli core install arduino:avr`. Then `./flash.sh` (override with `PORT=` and `FQBN=` if needed; the script header lists board types). The boot line must say `drv8871-v5-odo`. Or use the Arduino IDE on `firmware/drivetrain/drivetrain.ino`. You can test upload and serial without motors connected. Afterwards plug it into the Pi; the port is `/dev/ttyUSB0` or `/dev/ttyACM0` (set `ROBOT_SERIAL_PORT` in `/etc/robot.env` if it is the second).

## Audio comes out of the HDMI screen, or "PortAudio error"

The robot's audio library (PortAudio) talks to ALSA. Without `pipewire-alsa`, ALSA's default device is the Pi's HDMI output, so sound goes to the screen's speakers, and with no screen plugged in there is no device at all (the PortAudio error). The buds are only visible to PipeWire. Fix:

```bash
sudo apt install -y pipewire-alsa       # ./install_pi.sh now installs it
systemctl --user restart pipewire pipewire-pulse wireplumber
```

Then make the buds the default output and microphone (run as your normal user, not sudo, with the buds connected):

```bash
pactl list short sinks                  # find the bluez_output.* line
pactl list short sources                # find the bluez_input.* line (appears only in headset mode)
pactl set-default-sink   bluez_output.AA_BB_CC_DD_EE_FF.1
pactl set-default-source bluez_input.AA_BB_CC_DD_EE_FF.0
```

Use the exact names printed above. PipeWire remembers the choice. If there is no `bluez_input` line the buds are in music mode; run `./check_bt_audio.sh` and see the headset-mode notes in [README.md, Audio](README.md#bluetooth-buds-microphone--speaker). Finally `sudo systemctl restart robot-voice`.

## Day to day

After a `git pull`:

```bash
./install_pi.sh
sudo systemctl restart robot-voice
```

Because the workspace is built with `--symlink-install`, edits to existing Python files only need the restart. New packages and new `.msg`/`.action` files need the `./install_pi.sh` build step.

## Laptop (tests only, no Pi needed)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

## About ROS on the Pi

- The official ROS apt repository has no packages for Trixie, so the installer uses the community **rospian** repository (`https://rospian.github.io/rospian-repo`, suite `trixie-jazzy`).
- If that website is unreachable, the installer automatically clones the same repository from GitHub into `/opt/rospian-repo` (about 450 MB) and uses it as a local apt source. You do not need to do anything by hand.
- rospian has no `ros-jazzy-ros-base` bundle and no `ros-jazzy-ros-workspace` package, so the installer lists the individual ROS packages the robot uses, and generates `/opt/ros/jazzy/setup.bash` and registers ROS's shared libraries (`/etc/ld.so.conf.d/ros-jazzy.conf`) itself. Without that file, `source /opt/ros/jazzy/setup.bash` (used by `start_robot.sh`) fails.
- You never source ROS by hand: `start_robot.sh` does it. For your own shell, run `source /opt/ros/jazzy/setup.bash && source ~/Robot-RPi-Core/ros2_ws/install/setup.bash`.

## Starting over (clean up a failed install)

If an earlier attempt left ROS or the rospian source half-installed, wipe it and run the installer again:

```bash
./cleanup_pi.sh          # removes ROS, the rospian apt source and mirror, .venv and the ROS build
./install_pi.sh
```

It asks for confirmation first. It keeps your code, `/etc/robot.env`, Bluetooth pairing and the Pi settings. Add `--full` to also remove the `robot-voice` boot service.

## If something goes wrong

| Symptom | Fix |
|---------|-----|
| "This needs 64-bit Raspberry Pi OS Trixie" | Wrong OS image; re-flash (see top of this page) |
| `Unable to locate package ros-jazzy-...` | The rospian source is missing or stale: `cat /etc/apt/sources.list.d/rospian.list`, then `sudo apt update`. To start over: `./cleanup_pi.sh`, then `./install_pi.sh` |
| `/opt/ros/jazzy/setup.bash: No such file` | Re-run `./install_pi.sh`; it creates the file |
| `libddsc.so.0: cannot open shared object file` | The ROS library path is not registered: re-run `./install_pi.sh`, or just `echo -e "/opt/ros/jazzy/lib\n/opt/ros/jazzy/lib/aarch64-linux-gnu" \| sudo tee /etc/ld.so.conf.d/ros-jazzy.conf && sudo ldconfig` |
| `ros2: command not found` in your own shell | Source ROS first (see "About ROS on the Pi") |
| `Permission denied` on the serial port or audio | You have not rebooted since the install, or `id` does not show `dialout`/`audio` |
| Robot hears nothing at boot | Lingering is off: `sudo loginctl enable-linger $USER` and reboot; check the buds connect |
| Service crash-loops | `journalctl -u robot-voice -f`; clear with `sudo systemctl reset-failed robot-voice` |

# debug code
printf '%s\n' /opt/ros/jazzy/lib /opt/ros/jazzy/lib/aarch64-linux-gnu | sudo tee /etc/ld.so.conf.d/ros-jazzy.conf
sudo ldconfig
./start_robot.sh

# Port audio routing error - session is routed via screen's built in speaker
sudo apt install -y pipewire-alsa
systemctl --user restart pipewire pipewire-pulse wireplumber
pactl list short sinks      # find the line starting bluez_output...
pactl list short sources    # find the line starting bluez_input...
pactl set-default-sink   <the bluez_output name>
pactl set-default-source <the bluez_input name>
sudo systemctl restart robot-voice