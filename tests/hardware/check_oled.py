"""Wiring check for the status OLED: draws text, then a countdown. Run on the Pi:

    python tests/hardware/check_oled.py

Wiring (OLED -> Pi): VCC 3.3V(pin 1)  GND(pin 25)  SCK GPIO11(23)  SDA GPIO10(19)
CS GPIO8(24)  DC GPIO25(22)  RES GPIO17(11).  SPI must be on (raspi-config -> Interface).
Stop robot-voice first if the display node is running (only one program can own the screen).
"""
import time

from robot_core import status
from robot_core.oled import Oled


def main() -> None:
    board = status.Board()
    oled = Oled(board)
    if not oled.start():
        raise SystemExit("OLED not found - see the warning above (wiring, SPI on, pip install '.[oled]')")
    board.apply({"k": "task", "v": "explore"})
    board.apply({"k": "tool", "v": "drive 0.5m"})
    board.apply({"k": "detail", "v": "plant_03 capturing 4/8"})
    board.apply({"k": "timer", "v": "enroll Sam", "n": 10})
    board.apply({"k": "error", "v": "P01", "t": "wheels drawing current"})
    board.apply({"k": "battery", "n": 82})
    time.sleep(12)
    board.apply({"k": "result", "v": "Sam enrolled", "n": 4})
    time.sleep(5)
    oled.stop()


if __name__ == "__main__":
    main()
