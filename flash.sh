#!/bin/bash
# Flash the drivetrain firmware from the repo — no copy-pasting.
#
#   ./flash.sh                 # compile + upload with the defaults below
#   FQBN=arduino:avr:nano ./flash.sh     # override the board type
#
# One-time setup:  brew install arduino-cli
#                  arduino-cli core install arduino:avr
#
# FQBN = board type. Yours is FTDI-based, so if uno fails to upload try:
#   arduino:avr:nano                     (Nano, new bootloader)
#   arduino:avr:nano:cpu=atmega328old    (Nano clone, old bootloader)
#   arduino:avr:diecimila                (Duemilanove/Diecimila)
# Match whatever board selection worked in the Arduino IDE.
set -euo pipefail

PORT="${PORT:-/dev/cu.usbserial-A5069RR4}"
FQBN="${FQBN:-arduino:avr:uno}"
SKETCH="$(dirname "$0")/firmware/drivetrain"

echo "Compiling + uploading $SKETCH  (board=$FQBN, port=$PORT)"
arduino-cli compile --fqbn "$FQBN" --upload --port "$PORT" "$SKETCH"

echo
echo "Done. Verify the build stamp: the next test run should print"
echo "  Arduino firmware: drv8871-v5-odo built $(date '+%b %e %Y') <time>"
