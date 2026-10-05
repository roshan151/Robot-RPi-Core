"""Offline self-test for the framed serial protocol (no hardware needed).

    python selftest_protocol.py

Exercises the exact byte formats both sides emit, including the noise
scenarios that broke the old plain-text protocol: flipped bits, inserted
control bytes, and frames chopped in half.
"""

from __future__ import annotations
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from robot_core.drivetrain.serial_protocol import (
    FrameParser,
    checksum,
    describe_reset,
    encode_frame,
    parse_body,
)


def firmware_frame(body: str, crlf: bool = True) -> bytes:
    """Byte-exact copy of the firmware's sendBody() output (uppercase hex,
    println CRLF)."""
    raw = body.encode("ascii")
    return b"$" + raw + b"*" + f"{checksum(raw):02X}".encode() + (b"\r\n" if crlf else b"")


def main() -> None:
    # --- round trip, host encoding -> parser --------------------------------
    p = FrameParser()
    assert p.feed(encode_frame("M,7,F,127,2500")) == ["M,7,F,127,2500"]

    # --- round trip, firmware-style CRLF + uppercase hex ---------------------
    p = FrameParser()
    assert p.feed(firmware_frame("E,-2047,-1726")) == ["E,-2047,-1726"]

    # --- lowercase checksum also accepted ------------------------------------
    p = FrameParser()
    body = b"A,7"
    frame = b"$" + body + b"*" + f"{checksum(body):02x}".encode() + b"\n"
    assert p.feed(frame) == ["A,7"]

    # --- a flipped bit is dropped, not misparsed ------------------------------
    p = FrameParser()
    frame = bytearray(firmware_frame("D,7,OK,2500,2500"))
    frame[5] ^= 0x02                       # corrupt one payload byte
    assert p.feed(bytes(frame)) == []
    assert p.bad_frames == 1

    # --- inserted control byte (the 'TIME\x01UT' failure mode) ----------------
    p = FrameParser()
    frame = bytearray(firmware_frame("D,9,TIMEOUT,171,0"))
    frame.insert(8, 0x01)                  # noise injects a byte mid-frame
    assert p.feed(bytes(frame)) == []
    assert p.bad_frames == 1

    # --- garbage between frames is ignored; parser resyncs on '$' -------------
    p = FrameParser()
    stream = b"\xff\x00garbage" + firmware_frame("E,10,12") + b"~~~" + \
        firmware_frame("A,3")
    assert p.feed(stream) == ["E,10,12", "A,3"]

    # --- frame split across arbitrary read boundaries --------------------------
    p = FrameParser()
    frame = firmware_frame("B,6,drv8871-v3 built Aug  9 2026 12:00:00")
    got = []
    for i in range(0, len(frame), 3):
        got += p.feed(frame[i:i + 3])
    assert got == ["B,6,drv8871-v3 built Aug  9 2026 12:00:00"]

    # --- a '$' mid-frame restarts cleanly (old frame counted bad) -------------
    p = FrameParser()
    stream = b"$E,1" + firmware_frame("E,5,6")
    assert p.feed(stream) == ["E,5,6"]
    assert p.bad_frames == 1

    # --- body parsing ----------------------------------------------------------
    m = parse_body("D,42,NOISE,-16645630,16777469")
    assert m is not None and m.kind == "D" and m.seq == 42
    assert m.args[1] == "NOISE"

    m = parse_body("B,6,drv8871-v3 built Aug  9 2026 12:00:00")
    assert m is not None and m.kind == "B" and m.args[0] == "6"

    # --- reset-cause decoding ---------------------------------------------------
    assert "brown-out" in describe_reset("6")
    assert "power-on" in describe_reset("1")
    assert "unparseable" in describe_reset("zz")

    print("All protocol self-tests passed.")


if __name__ == "__main__":
    main()
