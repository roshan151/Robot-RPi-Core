#!/usr/bin/env bash
# Undo a failed or half-done install so ./install_pi.sh can start from scratch.
#
#   ./cleanup_pi.sh            remove ROS, the rospian apt source, .venv and the ROS build
#   ./cleanup_pi.sh --full     the above, plus the robot-voice boot service
#
# Removes:  every ros-jazzy-* apt package, /opt/ros, the rospian apt source and key,
#           /opt/rospian-repo (plus ~/rospian-repo and /tmp/rospian-repo from manual attempts),
#           .venv, ros2_ws/build, ros2_ws/install, ros2_ws/log.
# Keeps:    your code, /etc/robot.env, Bluetooth pairing, groups, /boot/firmware/config.txt,
#           and the other apt packages (camera, audio) — those are harmless to keep.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

FULL=0; [ "${1:-}" = "--full" ] && FULL=1

echo "This will remove ROS 2 Jazzy, the rospian apt source, .venv and the ROS build in $(pwd)."
[ $FULL = 1 ] && echo "It will also stop and remove the robot-voice boot service."
read -r -p "Continue? [y/N] " ans
[ "$ans" = y ] || [ "$ans" = Y ] || { echo "Cancelled."; exit 0; }

if [ $FULL = 1 ]; then
  echo "== boot service"
  sudo systemctl disable --now robot-voice 2>/dev/null || true
  sudo rm -f /etc/systemd/system/robot-voice.service
  sudo systemctl daemon-reload
else
  sudo systemctl stop robot-voice 2>/dev/null || true   # it holds the ROS files open
fi

echo "== ROS apt packages"
PKGS=$(dpkg-query -W -f='${Package}\n' 'ros-jazzy-*' 2>/dev/null || true)
[ -n "$PKGS" ] && sudo apt-get purge -y $PKGS
sudo apt-get autoremove -y

echo "== rospian apt source, key and local mirror"
sudo rm -f /etc/apt/sources.list.d/rospian.list /usr/share/keyrings/rospian-archive-keyring.gpg
sudo rm -rf /opt/rospian-repo ~/rospian-repo /tmp/rospian-repo
sudo apt-get update -qq || true

echo "== /opt/ros (includes the generated setup.bash)"
sudo rm -rf /opt/ros
sudo rm -f /etc/ld.so.conf.d/ros-jazzy.conf && sudo ldconfig

echo "== Python venv and ROS build"
rm -rf .venv ros2_ws/build ros2_ws/install ros2_ws/log

echo
echo "Clean. Now run: ./install_pi.sh"
