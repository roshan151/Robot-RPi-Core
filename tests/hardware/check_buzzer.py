"""Play the three buzzer sounds. Run on the Pi:  python tests/hardware/check_buzzer.py [BCM pin, default 26]

Stop the robot first (sudo systemctl stop robot-voice): the display node owns the buzzer pin.
"""

import sys
import time

from robot_core import buzzer

tone = buzzer.gpio_tone(int(sys.argv[1]) if len(sys.argv) > 1 else 26)
for name, tune in buzzer.TUNES.items():
    print(name)
    for hz, seconds in tune:
        tone(hz)
        time.sleep(seconds)
    tone(0)
    time.sleep(1.0)
