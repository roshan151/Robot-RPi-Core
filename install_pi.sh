#!/usr/bin/env bash
# One-time setup of a Raspberry Pi for the robot: ROS 2, every apt package, the Python
# venv, and the ROS workspace build. Safe to re-run.
#
#   ./install_pi.sh
#
# Needs 64-bit Raspberry Pi OS Trixie. The steps it can't do for you are printed at the end.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

APT_PACKAGES=(
  # ROS 2 Jazzy (base only: no desktop, no GUI)
  ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp python3-colcon-common-extensions
  # camera, vision and config (apt-only: they can't be pip-installed)
  python3-picamera2 python3-libcamera python3-opencv python3-numpy python3-yaml
  # audio and Bluetooth buds
  libportaudio2 alsa-utils pulseaudio-utils bluez pipewire pipewire-pulse wireplumber libspa-0.2-bluetooth
  python3-venv
)
PIP_EXTRAS="voice,explore"     # pyserial, requests, google-genai, sounddevice, numpy, rpi-hardware-pwm

. /etc/os-release
[ "$(dpkg --print-architecture)" = arm64 ] && [ "${VERSION_CODENAME:-}" = trixie ] || {
  echo "This needs 64-bit Raspberry Pi OS Trixie (found: $(dpkg --print-architecture), ${VERSION_CODENAME:-unknown})." >&2
  exit 1
}

echo "== ROS 2 Jazzy apt repository (community 'rospian', the official one has no Trixie packages)"
REPO=https://rospian.github.io/rospian-repo
if [ ! -f /etc/apt/sources.list.d/rospian.list ]; then
  KEY=$(mktemp)
  if ! curl -fsSL "$REPO/rospian-archive-keyring.asc" -o "$KEY"; then
    echo "ERROR: cannot download $REPO/rospian-archive-keyring.asc (HTTP error)." >&2
    echo "The rospian repo site is unreachable or moved; see https://github.com/rospian/rospian-repo" >&2
    echo "Open that URL in a browser; re-run this script once it loads." >&2
    exit 1
  fi
  gpg --dearmor < "$KEY" | sudo tee /usr/share/keyrings/rospian-archive-keyring.gpg >/dev/null
  rm -f "$KEY"
  echo "deb [arch=arm64 signed-by=/usr/share/keyrings/rospian-archive-keyring.gpg] $REPO trixie-jazzy main" \
    | sudo tee /etc/apt/sources.list.d/rospian.list >/dev/null
fi

echo "== apt packages"
sudo apt update
sudo apt install -y "${APT_PACKAGES[@]}"

echo "== Python venv (.venv, sees the apt packages) + pip packages"
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e ".[$PIP_EXTRAS]"
.venv/bin/python -c "import picamera2, libcamera, cv2, yaml, serial, sounddevice, google.genai, rpi_hardware_pwm" \
  || { echo "A Python import failed (see above)." >&2; exit 1; }

echo "== ROS workspace"
set +u
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
(cd ros2_ws && colcon build --symlink-install)
set -u

cat <<'DONE'

Installed. Still to do by hand (see README, Installation):
  1. Secrets:   sudo nano /etc/robot.env          (GEMINI_API_KEY=...)
  2. Head + TF-Luna: add the pwm overlay to /boot/firmware/config.txt, enable the serial port, reboot
  3. Groups:    sudo usermod -aG dialout,audio,bluetooth $USER && sudo loginctl enable-linger $USER
  4. Buds:      pair them with bluetoothctl
  5. Service:   sudo cp robot-voice.service /etc/systemd/system/ && sudo systemctl enable --now robot-voice
DONE
