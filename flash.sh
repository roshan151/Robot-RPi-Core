#!/bin/bash
# Flash the drivetrain firmware from the repo — no copy-pasting.
#
#   ./flash.sh                 # compile + upload with the defaults below
#   PORT=/dev/cu.usbserial-0001 ./flash.sh   # override the port
#
# Board: ESP32 DevKit (ESP32-WROOM-32).  One-time setup:
#                  Mac: brew install arduino-cli
#                  Pi:  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
#                  then: arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
#                        arduino-cli core update-index && arduino-cli core install esp32:esp32
# If the upload stalls at "Connecting....", hold the board's BOOT button
# until it starts writing.
# On the Pi, stop the robot first (it holds the serial port):
#                  sudo systemctl stop robot-voice  ...flash...  sudo systemctl start robot-voice
#
set -euo pipefail

if [ "$(uname)" = Darwin ]; then DEFAULT_PORT=$(ls /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART /dev/cu.wchusbserial* 2>/dev/null | head -1 || true)
else DEFAULT_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 || true); fi
PORT="${PORT:-$DEFAULT_PORT}"
[ -n "$PORT" ] || { echo "No serial port found; set PORT=/dev/ttyUSB0" >&2; exit 1; }
FQBN="${FQBN:-esp32:esp32:esp32}"
SKETCH="$(dirname "$0")/firmware/drivetrain"

echo "Compiling + uploading $SKETCH  (board=$FQBN, port=$PORT)"
arduino-cli compile --fqbn "$FQBN" --upload --port "$PORT" "$SKETCH"

echo
echo "Done. Verify the build stamp: the next test run should print"
echo "  Arduino firmware: drv8871-v10-esp32-vel-luna built $(date '+%b %e %Y') <time>"
