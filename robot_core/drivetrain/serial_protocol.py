"""Framed, checksummed serial protocol (firmware v3).

Every line, both directions:  ``$BODY*CS\\n`` where CS is two hex digits,
the XOR of every byte of BODY.  Bytes outside a well-formed frame are
dropped and counted — line noise can corrupt at most one frame, and can
never inject a command or fake a reply.

Host -> robot bodies:
    P                              heartbeat (no reply)
    Q,<seq>                        ping                       -> A,<seq>
    V,<seq>,<pwm>                  set open-loop speed        -> A,<seq>
    I,<seq>,<dir>                  open-loop drive F/B/L/R    -> A,<seq>
    S,<seq>                        stop (active brake)        -> A,<seq>
    M,<seq>,<dir>,<pwm>,<ticks>    encoder-counted move       -> A now, D later

Robot -> host bodies:
    A,<seq>                        accepted
    N,<seq>,<reason>               rejected (BADARG | BUSY)
    D,<seq>,<status>,<el>,<er>     move done (OK|TIMEOUT|NOISE|STOP|LINK)
    E,<el>,<er>[,<ol>,<or>,<ms>]   encoder telemetry (100 ms); v5 firmware
                                   appends never-reset odometry totals + millis
    W,<code>[,...]                 warning (NOISE|MEMCORRUPT|RXBAD|LINK)
    B,<hex>,<build>                boot: reset cause + firmware build stamp
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

FRAME_START = ord("$")
FRAME_END = ord("*")
BODY_MAX = 64

# ---------------------------------------------------------------------------
# Reset-cause bits (AVR MCUSR), reported by the firmware in the B frame.
# ---------------------------------------------------------------------------
_RESET_CAUSE_BITS = (
    (0x1, "power-on (the Arduino's 5V supply dropped completely — "
          "USB power was interrupted: check the USB cable, connector, and port)"),
    (0x2, "external reset pin (normal when the serial port is opened via DTR; "
          "mid-move it means electrical noise reached the RESET pin)"),
    (0x4, "brown-out (the Arduino's 5V rail sagged below ~2.7V — "
          "something is dragging down or coupling into the 5V supply)"),
    (0x8, "watchdog flag (usually a bootloader side effect — "
          "ignore unless it appears alone)"),
)

# Human-readable text for D-frame terminal statuses (anything except OK).
MOVE_FAIL_TEXT = {
    "TIMEOUT": "the move ran past the firmware's 15 s limit without both "
               "encoders reaching the target (stalled wheel, disconnected "
               "encoder, or TICKS_PER_CM set too high)",
    "NOISE":   "one encoder channel repeatedly reported impossible counts "
               "(electrical pickup on the encoder wiring)",
    "STOP":    "the move was interrupted by a stop command",
    "LINK":    "the firmware's link watchdog stopped the motors — no valid "
               "frame (heartbeat) arrived for over a second",
}

# Human-readable text for W-frame warning codes.
WARN_TEXT = {
    "NOISE":      "Encoder noise repaired mid-move (raw counts {detail}) — "
                  "move continued; check encoder wire routing if frequent",
    "MEMCORRUPT": "Encoder memory corruption repaired mid-move (raw {detail}) "
                  "— a supply transient hit RAM; check grounds and motor EMI",
    "RXBAD":      "Arduino has dropped {detail} corrupted frame(s) — "
                  "electrical noise on the serial line (commands unaffected)",
    "LINK":       "Firmware link watchdog stopped the motors — the host "
                  "stopped sending frames while the robot was driving",
}


def describe_reset(cause_hex: str) -> str:
    """Plain-English reset cause from the B frame's hex field."""
    try:
        flags = int(cause_hex, 16)
    except ValueError:
        return f"reset cause unparseable: {cause_hex!r}"
    causes = [text for bit, text in _RESET_CAUSE_BITS if flags & bit]
    if not causes:
        return "no cause flags set (unexpected — possibly an old bootloader)"
    return "; ".join(causes)


def checksum(body: bytes) -> int:
    x = 0
    for b in body:
        x ^= b
    return x


def encode_frame(body: str) -> bytes:
    """Wrap a body string into a wire frame with checksum."""
    raw = body.encode("ascii")
    return b"$" + raw + b"*" + f"{checksum(raw):02X}".encode("ascii") + b"\n"


@dataclass
class Message:
    """A decoded robot->host body, split on commas."""
    kind: str
    args: Tuple[str, ...]

    @property
    def seq(self) -> Optional[int]:
        if self.kind in ("A", "N", "D") and self.args:
            try:
                return int(self.args[0])
            except ValueError:
                return None
        return None


def parse_body(body: str) -> Optional[Message]:
    if not body:
        return None
    parts = body.split(",")
    return Message(kind=parts[0], args=tuple(parts[1:]))


@dataclass
class FrameParser:
    """Streaming decoder: feed raw bytes, get back validated bodies.

    A ``$`` anywhere restarts the frame, so the parser resynchronises on
    the next frame no matter what garbage arrived in between.  Corrupted
    or malformed frames are dropped and counted in ``bad_frames``.
    """
    bad_frames: int = 0
    _state: int = 0            # 0 idle, 1 body, 2 cs1, 3 cs2
    _body: bytearray = field(default_factory=bytearray)
    _xor: int = 0
    _cs_hi: int = 0

    def feed(self, data: bytes) -> List[str]:
        out: List[str] = []
        for b in data:
            if b == FRAME_START:
                if self._state != 0:
                    self.bad_frames += 1     # previous frame never finished
                self._state = 1
                self._body = bytearray()
                self._xor = 0
                continue
            if self._state == 0:
                continue
            if self._state == 1:
                if b == FRAME_END:
                    self._state = 2
                elif b in (0x0A, 0x0D):
                    self._state = 0
                    self.bad_frames += 1
                elif len(self._body) >= BODY_MAX:
                    self._state = 0
                    self.bad_frames += 1
                else:
                    self._body.append(b)
                    self._xor ^= b
            elif self._state == 2:
                v = _hex_val(b)
                if v < 0:
                    self._state = 0
                    self.bad_frames += 1
                else:
                    self._cs_hi = v << 4
                    self._state = 3
            elif self._state == 3:
                v = _hex_val(b)
                self._state = 0
                if v < 0 or (self._cs_hi | v) != self._xor:
                    self.bad_frames += 1
                else:
                    try:
                        out.append(self._body.decode("ascii"))
                    except UnicodeDecodeError:
                        self.bad_frames += 1
        return out


def _hex_val(b: int) -> int:
    if 0x30 <= b <= 0x39:
        return b - 0x30
    if 0x41 <= b <= 0x46:
        return b - 0x41 + 10
    if 0x61 <= b <= 0x66:
        return b - 0x61 + 10
    return -1
