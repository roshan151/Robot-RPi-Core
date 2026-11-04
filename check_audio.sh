#!/usr/bin/env bash
# Which mic will the robot use, and does the camera mic work? (docs/USB-CAMERA-MIC.md)
# Run on the Pi as the robot user, from the repo folder. Buds: see check_bt_audio.sh.
set -uo pipefail
cd "$(dirname "$0")"

echo "== inputs";  pactl list short sources | grep -v '\.monitor'
echo "== outputs"; pactl list short sinks
echo "== default input now: $(pactl get-default-source)"
echo "== the robot would pick:"
python3 -c "from robot_core import audio_route; print(audio_route.current())"

SRC="$(pactl list short sources | awk '{print $2}' | grep -m1 -- "${ROBOT_ROOM_MIC:-alsa_input.usb-}")"
[ -n "$SRC" ] || { echo "!! no camera mic matches ROBOT_ROOM_MIC=${ROBOT_ROOM_MIC:-alsa_input.usb-}"; exit 1; }
echo "== recording 3 s from $SRC: speak from ~2 m away"
timeout 3 parecord --device="$SRC" /tmp/robot_mic.wav
echo "== playing it back (too quiet? pactl set-source-volume $SRC 100%)"
paplay /tmp/robot_mic.wav
