"""Three sounds on a passive buzzer, so the robot can be understood without its screen.

    online       three rising notes, once      the voice session is up: waiting for a command
    connecting   two short blips, every 2.5 s  booting / (re)connecting / a passing error
    failure      four falling notes, the last  something is broken and will not fix itself
                 one long

Nothing else in the robot calls this. The display node feeds it every status
message (the ones it already draws on the OLED) and `on_message` decides:

    sound "online"   (the voice agent, when its session connects)   -> online, once
    start-up, sound "connecting", or error L01 (voice session lost) -> connecting, until
                     "online", a failure, or CONNECT_FOR_S has passed
    errors in TRANSIENT (vision unreachable, motor command timeout) -> connecting, once
    errors in FAILURES  (fatal, Arduino lost, e-stop, head latched) -> failure

The tone is a software square wave (RPi.GPIO's PWM, which rpi-lgpio times in a
thread): both hardware PWM channels belong to the head servos, and GPIO18/19 are
those same two channels, so the buzzer must NOT go there. The pitch wobbles a
little under load; fine for beeps.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Optional

# (frequency Hz, seconds); 0 Hz = silence. Kept between 500 Hz and 2 kHz, where a small buzzer is loud.
TUNES = {
    "online": ((880, 0.09), (1175, 0.09), (1568, 0.16)),
    "connecting": ((660, 0.06), (0, 0.08), (660, 0.06)),
    "failure": ((988, 0.16), (740, 0.16), (554, 0.16), (0, 0.04), (523, 0.6)),
}
FAILURES = {"X02", "S01", "S03", "C01", "G04", "G05", "G06", "G07", "N03"}
RECONNECTING = {"L01"}                    # keeps blipping until the session is back
TRANSIENT = {"V01", "V02", "S04", "A01"}  # one pair of blips
CONNECT_EVERY_S = 2.5
CONNECT_FOR_S = 90.0                      # a lost "online" cannot leave it blipping for ever
REPEAT_GAP_S = 8.0                        # the same error reported over and over sounds once in this long


def gpio_tone(pin: int) -> Callable[[float], None]:
    """tone(hz) for a passive buzzer on BCM `pin`; tone(0) = silent."""
    import RPi.GPIO as GPIO                          # rpi-lgpio provides this on current Pi OS
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    pwm = GPIO.PWM(pin, 1000)

    def tone(hz: float) -> None:
        if hz > 0:
            pwm.ChangeFrequency(hz)
            pwm.start(50)
        else:
            pwm.stop()                               # the pin rests low: no current through the buzzer
    return tone


class Buzzer:
    def __init__(self, tone: Callable[[float], None], clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self._tone, self._clock, self._sleep = tone, clock, sleep
        self._q: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=4)
        self._online = False
        self._connecting_until = clock() + CONNECT_FOR_S      # booting: connecting until told otherwise
        self._last: dict = {}                                 # tune -> when it was last asked for
        self._thread: Optional[threading.Thread] = None

    # --- deciding -------------------------------------------------------

    def on_message(self, msg: dict) -> None:
        """One status message (robot_core.status). Never raises, never blocks."""
        k, v, now = msg.get("k"), str(msg.get("v", "")), self._clock()
        if k == "sound" and v == "online":
            self._connecting_until = 0.0
            if not self._online:                              # a silent re-connect (session length limit) stays silent
                self._online = True
                self.play("online")
        elif (k == "sound" and v == "connecting") or (k == "error" and v in RECONNECTING):
            self._online, self._connecting_until = False, now + CONNECT_FOR_S
        elif k == "error" and v in FAILURES:
            self._connecting_until = 0.0
            self._once("failure", now)
        elif k == "error" and v in TRANSIENT:
            self._once("connecting", now)
        elif k == "task" and v not in ("", "voice"):          # at work without a voice session (explore launch)
            self._connecting_until = 0.0

    def _once(self, name: str, now: float) -> None:
        if now - self._last.get(name, -1e9) >= REPEAT_GAP_S:
            self._last[name] = now
            self.play(name)

    def play(self, name: str) -> None:
        try:
            self._q.put_nowait(name)
        except queue.Full:                                    # sounds are piling up: drop, never block the robot
            pass

    # --- sounding -------------------------------------------------------

    def step(self, timeout: float = CONNECT_EVERY_S) -> bool:
        """Play the next sound asked for, or the connecting blips if that is the state. False = stop."""
        try:
            name = self._q.get(timeout=timeout)
        except queue.Empty:
            name = "connecting" if self._clock() < self._connecting_until else ""
        if name is None:
            return False
        try:
            for hz, seconds in TUNES.get(name, ()):
                self._tone(hz)
                self._sleep(seconds)
        finally:
            self._tone(0)
        return True

    def start(self) -> None:
        def run() -> None:
            try:
                while self.step():
                    pass
            except Exception:                                 # noqa: BLE001 — a buzzer must never break the robot
                pass
        self._thread = threading.Thread(target=run, name="buzzer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.play(None)                                       # type: ignore[arg-type]
        if self._thread:
            self._thread.join(timeout=2.0)
