#!/bin/bash
# Flash the drivetrain firmware from the repo — no copy-pasting.
#
#   ./flash.sh                 # compile + upload with the defaults below
#   FQBN=arduino:avr:nano ./flash.sh     # override the board type
#
# One-time setup:  Mac: brew install arduino-cli
#                  Pi:  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
#                  then: arduino-cli core install arduino:avr
# On the Pi, stop the robot first (it holds the serial port):
#                  sudo systemctl stop robot-voice  ...flash...  sudo systemctl start robot-voice
#
# FQBN = board type. Yours is FTDI-based, so if uno fails to upload try:
#   arduino:avr:nano                     (Nano, new bootloader)
#   arduino:avr:nano:cpu=atmega328old    (Nano clone, old bootloader)
#   arduino:avr:diecimila                (Duemilanove/Diecimila)
# Match whatever board selection worked in the Arduino IDE.
set -euo pipefail

if [ "$(uname)" = Darwin ]; then DEFAULT_PORT=/dev/cu.usbserial-A5069RR4
else DEFAULT_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1); fi
PORT="${PORT:-$DEFAULT_PORT}"
[ -n "$PORT" ] || { echo "No serial port found; set PORT=/dev/ttyUSB0" >&2; exit 1; }
FQBN="${FQBN:-arduino:avr:uno}"
SKETCH="$(dirname "$0")/firmware/drivetrain"

echo "Compiling + uploading $SKETCH  (board=$FQBN, port=$PORT)"
arduino-cli compile --fqbn "$FQBN" --upload --port "$PORT" "$SKETCH"

echo
echo "Done. Verify the build stamp: the next test run should print"
echo "  Arduino firmware: drv8871-v5-odo built $(date '+%b %e %Y') <time>"
