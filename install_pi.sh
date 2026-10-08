#!/usr/bin/env bash
# One-time setup of a Raspberry Pi for the robot. Safe to re-run (e.g. after a git pull).
#
#   ./install_pi.sh
#
# Does: ROS 2 Jazzy (apt), every other apt package, the Python venv, the ROS workspace
# build, group membership, the head/serial hardware config, and the boot service file.
# Does NOT do (needs you): the API key file and Bluetooth pairing. See README-SETUP.md.
#
# Needs 64-bit Raspberry Pi OS Trixie.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
REPO_DIR=$(pwd)
ROS=/opt/ros/jazzy

ROS_PACKAGES=(   # the rospian repo has no ros-base bundle, so list what the robot uses (no GUI)
  ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
  ros-jazzy-nav-msgs ros-jazzy-action-msgs ros-jazzy-tf2-ros ros-jazzy-tf2-ros-py
  ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime
  ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-launch-xml
  ros-jazzy-ros2cli ros-jazzy-ros2run ros-jazzy-ros2launch ros-jazzy-ros2action ros-jazzy-ros2param
  ros-jazzy-ros2topic ros-jazzy-ros2service ros-jazzy-ros2pkg ros-jazzy-rmw-cyclonedds-cpp
)
OTHER_PACKAGES=(
  curl gnupg git ca-certificates python3-venv
  python3-picamera2 python3-libcamera python3-opencv python3-numpy python3-yaml   # camera + vision
  libportaudio2 alsa-utils pulseaudio-utils bluez pipewire pipewire-pulse pipewire-alsa wireplumber libspa-0.2-bluetooth  # audio, buds (pipewire-alsa routes PortAudio to the buds)
)
PIP_EXTRAS="voice,explore,oled"

say() { printf '\n== %s\n' "$*"; }

. /etc/os-release
[ "$(dpkg --print-architecture)" = arm64 ] && [ "${VERSION_CODENAME:-}" = trixie ] || {
  echo "This needs 64-bit Raspberry Pi OS Trixie (found: $(dpkg --print-architecture), ${VERSION_CODENAME:-unknown})." >&2
  exit 1
}

# ---------------------------------------------------------------- 1. ROS apt repository
# The official ROS repo has no Trixie packages; the community "rospian" repo does.
# Normally it is used straight from the web. If the web site is unreachable, the same repo is
# cloned from GitHub into /opt/rospian-repo and used as a local apt source.
say "ROS 2 Jazzy apt repository (rospian)"
sudo apt-get update -qq || true
sudo apt-get install -y curl gnupg git ca-certificates
WEB=https://rospian.github.io/rospian-repo
KEYRING=/usr/share/keyrings/rospian-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/rospian.list
if [ ! -f "$LIST" ]; then
  KEY=$(mktemp)
  if curl -fsSL "$WEB/rospian-archive-keyring.asc" -o "$KEY"; then
    SRC="$WEB"
    echo "Using $WEB"
  else
    echo "Cannot reach $WEB; cloning the repo from GitHub instead (about 450 MB)."
    sudo rm -rf /opt/rospian-repo
    sudo git clone --depth 1 https://github.com/rospian/rospian-repo /opt/rospian-repo
    sudo chmod -R a+rX /opt/rospian-repo
    cp /opt/rospian-repo/public/rospian-archive-keyring.asc "$KEY"
    SRC="file:/opt/rospian-repo"
  fi
  gpg --dearmor < "$KEY" | sudo tee "$KEYRING" >/dev/null
  rm -f "$KEY"
  echo "deb [arch=arm64 signed-by=$KEYRING] $SRC trixie-jazzy main" | sudo tee "$LIST" >/dev/null
fi

# ---------------------------------------------------------------- 2. apt packages
say "apt packages"
sudo apt-get update
sudo apt-get install -y "${ROS_PACKAGES[@]}" "${OTHER_PACKAGES[@]}"

# rospian has no ros-workspace package, so $ROS/setup.bash does not exist and every
# "source /opt/ros/jazzy/setup.bash" fails. Generate it from ament_package's own templates
# (this is exactly what the missing package installs).
if [ ! -f "$ROS/setup.bash" ]; then
  say "Creating $ROS/setup.bash (missing from rospian)"
  T=$(ls -d "$ROS"/lib/python3*/site-packages/ament_package/template/prefix_level | head -1)
  sudo cp "$T"/{local_setup.bash,local_setup.zsh,setup.bash,setup.zsh,_local_setup_util.py} "$ROS/"
  for f in local_setup.sh setup.sh; do
    sed -e "s|@CMAKE_INSTALL_PREFIX@|$ROS|g" -e "s|@ament_package_PYTHON_EXECUTABLE@|/usr/bin/python3|g" \
        -e 's|@SKIP_PARENT_PREFIX_PATH@||g' "$T/$f.in" | sudo tee "$ROS/$f" >/dev/null
  done
fi

# The same missing package normally puts ROS's shared libraries on the loader path, so without
# it ros2 fails with "libddsc.so.0: cannot open shared object file". Register them system-wide.
say "Registering ROS shared libraries with the dynamic linker"
printf '%s\n' "$ROS/lib" "$ROS/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)" | sudo tee /etc/ld.so.conf.d/ros-jazzy.conf >/dev/null
sudo ldconfig

# ---------------------------------------------------------------- 3. Python venv + ROS build
say "Python venv (.venv, sees the apt packages) + pip packages"
python3 -m venv --system-site-packages .venv
.venv/bin/pip install colcon-common-extensions      # not packaged for Trixie
.venv/bin/pip install -e ".[$PIP_EXTRAS]"
.venv/bin/python -c "import picamera2, libcamera, cv2, yaml, serial, sounddevice, google.genai, rpi_hardware_pwm" \
  || { echo "A Python import failed (see above)." >&2; exit 1; }

say "ROS workspace build"
set +u
source "$ROS/setup.bash"
source .venv/bin/activate
# Every ROS module the robot imports (the C++ package alone is not enough: tf2_ros needs tf2-ros-py).
python - <<'PY2' || { echo "A ROS Python import failed (see above); install the matching ros-jazzy-* package." >&2; exit 1; }
import rclpy, tf2_ros, std_msgs, std_srvs, geometry_msgs, nav_msgs, rcl_interfaces, ament_index_python, launch, launch_ros
PY2
(cd ros2_ws && colcon build --symlink-install)
set -u

# ---------------------------------------------------------------- 4. Pi configuration
say "Groups and lingering (audio must exist at boot)"
sudo usermod -aG dialout,audio,bluetooth "$USER"
# Lets the voice "shut down" tool power the Pi off cleanly - this one command only, no password.
echo "$USER ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff" | sudo tee /etc/sudoers.d/robot-shutdown >/dev/null
sudo chmod 440 /etc/sudoers.d/robot-shutdown && sudo visudo -cf /etc/sudoers.d/robot-shutdown >/dev/null || sudo rm -f /etc/sudoers.d/robot-shutdown
sudo loginctl enable-linger "$USER"

say "Head servos (PWM overlay) and TF-Luna serial port"
OVERLAY="dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4"
grep -qxF "$OVERLAY" /boot/firmware/config.txt || echo "$OVERLAY" | sudo tee -a /boot/firmware/config.txt >/dev/null
sudo raspi-config nonint do_serial_cons 1     # 1 = no login shell on the serial port
sudo raspi-config nonint do_serial_hw 0       # 0 = serial hardware on
sudo raspi-config nonint do_spi 0              # 0 = SPI on (status OLED)

say "Boot service file (not started yet)"
UID_NOW=$(id -u)
sed -e "s|^User=.*|User=$USER|" -e "s|^Group=.*|Group=$(id -gn)|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=$REPO_DIR|" -e "s|^ExecStart=.*|ExecStart=$REPO_DIR/start_robot.sh|" \
    -e "s|/run/user/1000|/run/user/$UID_NOW|g" robot-voice.service | sudo tee /etc/systemd/system/robot-voice.service >/dev/null
sudo systemctl daemon-reload

cat <<DONE

Installed. Two things left for you (details in README-SETUP.md):
  1. API key:  sudo nano /etc/robot.env        (GEMINI_API_KEY=..., VISION_SERVICE_BASE_URL=...)
               sudo chown root:$USER /etc/robot.env && sudo chmod 640 /etc/robot.env
  2. Buds:     pair them with bluetoothctl
Then:          sudo reboot      (applies groups, PWM overlay and serial port)
After reboot:  sudo systemctl enable --now robot-voice     (or ./start_robot.sh to try it by hand)
DONE
