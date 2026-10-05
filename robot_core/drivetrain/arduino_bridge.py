"""
Serial bridge to the drivetrain firmware (protocol v3: framed + checksummed).

Design notes — what this layer guarantees:

  * Every command travels in a ``$BODY*CS`` frame.  Corrupted bytes on the
    wire can drop a frame but can never execute as a different command or
    fake a reply (the failure mode behind 'TIME\\x01UT' and friends).
  * Every command carries a sequence number.  If the ACK is lost, the
    command is retransmitted with the SAME number and the firmware
    replays its cached reply instead of executing twice — retries are
    safe even for moves.
  * A background heartbeat frame goes out 4x/second.  If this process
    dies or the cable drops mid-move, the firmware's link watchdog
    brakes the motors on its own within a second.
  * Firmware resets are detected via the B (boot) frame and surfaced as
    errors on whatever call was in flight, with the decoded MCUSR cause.

Public API (unchanged from v2 — drivetrain_client and the test scripts
work without modification):

    ArduinoBridge(port, baud, timeout=..., on_encoder=..., on_error=...)
        .move(direction, speed_pwm, ticks)   # blocking encoder move
        .forward() .backward() .left() .right()
        .set_speed_pwm(v)
        .stop()
        .close()
        .is_connected
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Optional, Tuple

import serial  # pyserial

from robot_core import settings
from .serial_protocol import (
    FrameParser,
    Message,
    MOVE_FAIL_TEXT,
    WARN_TEXT,
    describe_reset,
    encode_frame,
    parse_body,
)

logger = logging.getLogger(__name__)

ACK_TIMEOUT_S = float(getattr(settings, "ACK_TIMEOUT_S", 0.35))
CMD_RETRIES = int(getattr(settings, "CMD_RETRIES", 3))
MOVE_TIMEOUT_S = float(getattr(settings, "MOVE_TIMEOUT_S", 20.0))
PING_INTERVAL_S = float(getattr(settings, "PING_INTERVAL_S", 0.25))
BOOT_WAIT_S = float(getattr(settings, "ARDUINO_DRAIN_WAIT_S", 2.5)) + 1.5

ESTOP_SEQ_MIN = int(getattr(settings, "ESTOP_SEQ_MIN", 240))
ESTOP_SEQ_MAX = int(getattr(settings, "ESTOP_SEQ_MAX", 255))
NORMAL_SEQ_MAX = int(getattr(settings, "NORMAL_SEQ_MAX", ESTOP_SEQ_MIN - 1))


class ProtocolError(RuntimeError):
    """The firmware rejected a command (N frame)."""


def _done_counts(msg: Message) -> Optional[Tuple[int, int]]:
    """Signed (left, right) counts carried by a D frame: D,<seq>,<status>,<el>,<er>.

    None rather than an exception on a short or unparseable frame — a move that
    the firmware reported OK did happen, and losing the diagnostic counts is not
    grounds for failing it.  The caller falls back to encoder telemetry.
    """
    if len(msg.args) < 4:
        return None
    try:
        return int(msg.args[2]), int(msg.args[3])
    except ValueError:
        logger.warning("D frame carried unparseable counts: %r", msg.args[2:4])
        return None


class ArduinoBridge:
    def __init__(
        self,
        port: str = settings.SERIAL_PORT,
        baud: int = settings.BAUD_RATE,
        timeout: float = settings.SERIAL_TIMEOUT,
        on_encoder: Optional[Callable[[int, int], None]] = None,
        on_error: Optional[Callable[[], None]] = None,
        on_odometry: Optional[Callable[[int, int, int], None]] = None,
    ) -> None:
        self._on_encoder = on_encoder
        # (total_left, total_right, firmware_ms) from v5+ E frames. Older
        # firmware sends only the per-move pair and this is never called.
        self._on_odometry = on_odometry
        self._on_error = on_error

        self._parser = FrameParser()
        self._seq = 0
        # Reserved sequence band for out-of-band emergency stops — see
        # emergency_stop().  Starts one below the band so the first e-stop
        # uses ESTOP_SEQ_MIN.
        self._estop_seq = ESTOP_SEQ_MAX

        # _cmd_lock serialises whole transactions (send -> reply) so two
        # callers can never interleave; _write_lock protects the raw port
        # writes shared with the heartbeat thread.
        self._cmd_lock = threading.Lock()
        self._write_lock = threading.Lock()

        self._reply_q: "queue.Queue[Message]" = queue.Queue()
        self._done_q: "queue.Queue[Message]" = queue.Queue()
        self._awaiting_reply = threading.Event()
        self._awaiting_done = threading.Event()

        self._boot_seen = threading.Event()
        self._connecting = True
        self._closed = False

        # Health counters (read by diagnostics; never reset).
        self.resets_seen = 0
        self.warnings_seen = 0
        self.firmware_build: Optional[str] = None

        logger.info("Opening serial port %s @ %d baud", port, baud)
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)

        self._reader_stop = threading.Event()
        self._reader = threading.Thread(
            target=self._read_loop, name="arduino-reader", daemon=True
        )
        self._reader.start()

        self._connect()
        self._connecting = False

        self._hb_stop = threading.Event()
        self._heartbeat = threading.Thread(
            target=self._heartbeat_loop, name="arduino-heartbeat", daemon=True
        )
        self._heartbeat.start()

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    @property
    def is_connected(self) -> bool:
        return not self._closed and self._ser.is_open

    def close(self) -> None:
        """Idempotent shutdown: stop motors, kill threads, close port."""
        if self._closed:
            return
        self._closed = True
        logger.debug("Closing ArduinoBridge")
        try:
            self.stop()
            time.sleep(0.15)
            self._raw_send("P")          # last watchdog feed while braking
        except Exception:
            pass
        if hasattr(self, "_hb_stop"):
            self._hb_stop.set()
        self._reader_stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass

    def _connect(self) -> None:
        """Wait for the boot frame (port-open DTR resets the board), then
        confirm two-way traffic with a ping."""
        port = self._ser.port
        if self._boot_seen.wait(timeout=BOOT_WAIT_S):
            logger.info("Connected on %s (boot frame received)", port)
        else:
            logger.info(
                "No boot frame on %s — assuming the board was already "
                "running; pinging", port,
            )
        try:
            self._transact("Q")
        except Exception as exc:
            raise RuntimeError(
                f"Arduino did not respond on {port!r}: {exc}. "
                "Check the USB cable, that no other app holds the port, "
                "that the baud is 115200, and that firmware v3 "
                "(sketches/drivetrain/drivetrain.ino) is flashed."
            ) from exc

    # ------------------------------------------------------------------ #
    # Public commands
    # ------------------------------------------------------------------ #

    def forward(self) -> None:
        self._transact("I", "F")

    def backward(self) -> None:
        self._transact("I", "B")

    def left(self) -> None:
        self._transact("I", "L")

    def right(self) -> None:
        self._transact("I", "R")

    def set_speed_pwm(self, value: int) -> None:
        v = max(0, min(255, int(value)))
        self._transact("V", str(v))

    def stop(self) -> None:
        """Send stop with a short budget — safe to call during teardown.

        NOTE: this takes _cmd_lock, so it BLOCKS until any in-flight
        encoder-counted move() completes.  It cannot interrupt a move.
        To halt a move already in progress, use emergency_stop().
        """
        try:
            self._transact("S", retries=1, ack_timeout=0.5)
        except Exception:
            logger.warning("stop(): no ACK received (already disconnected?)")

    def emergency_stop(self) -> None:
        """Out-of-band brake that works DURING a blocking move.

        move() holds _cmd_lock for its entire duration (up to MOVE_TIMEOUT_S),
        so anything routed through _transact() — including stop() — queues
        behind it and arrives only after the move has already finished.  That
        is not a halt, it is a very late no-op.

        This writes the S frame straight to the port under _write_lock only,
        the same path the heartbeat thread already uses concurrently with an
        active move, so it is safe to call from any thread.

        Firmware side (drivetrain.ino): on S with move_active it calls
        finishMove("STOP"), which emits the D frame the blocked move() is
        waiting on.  move() then raises RuntimeError — that exception IS the
        cancellation signal, and callers should treat it as such rather than
        as a fault.

        Sequence numbers come from a reserved band because the firmware
        deduplicates against the single previous seq: an e-stop reusing the
        in-flight move's seq would be swallowed as a retransmission.
        """
        span = ESTOP_SEQ_MAX - ESTOP_SEQ_MIN + 1
        self._estop_seq = ESTOP_SEQ_MIN + ((self._estop_seq - ESTOP_SEQ_MIN + 1) % span)
        try:
            self._raw_send(f"S,{self._estop_seq}")
            logger.warning("emergency_stop: out-of-band S sent (seq %d)", self._estop_seq)
        except Exception as e:
            logger.error("emergency_stop: raw write failed: %s", e)

    def move(self, direction: str, speed_pwm: int, ticks: int
             ) -> Optional[Tuple[int, int]]:
        """Blocking encoder-counted move.

        Sends M, waits for the acceptance ACK, then blocks until the
        firmware reports the move finished (D frame).  Raises on firmware
        reset, rejection, non-OK completion, or timeout.

        Returns the move's final signed encoder counts ``(left, right)``,
        taken from the D frame's own payload, or None if the firmware is old
        enough to send a D with no counts on it.

        Returning them matters for latency, not for convenience: the caller
        used to recover the same numbers by sleeping one telemetry interval
        and reading the last E frame, and that sleep sat between this move
        finishing and the next one being sent.  The counts were always in the
        D frame — the sleep was buying nothing the firmware had not already
        told us.

        Caveat for anyone comparing the two: these counts are latched in
        finishMove() at the instant softStop() is called, so they exclude the
        ticks the robot rolls while the ramp-down completes.  That coast is
        what TURN_COAST_TICKS exists to model; it is a few percent and none of
        the checks in _verify_encoder_delta() are sensitive to it.
        """
        if direction not in ("F", "B", "L", "R"):
            raise ValueError(f"move: bad direction {direction!r}")
        with self._cmd_lock:
            self._drain(self._done_q)
            self._awaiting_done.set()
            try:
                seq = self._transact_locked(
                    "M", direction, str(int(speed_pwm)), str(int(ticks))
                )
                deadline = time.monotonic() + MOVE_TIMEOUT_S
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(
                            f"No move-complete report from Arduino within "
                            f"{MOVE_TIMEOUT_S:.0f} s"
                        )
                    try:
                        msg = self._done_q.get(timeout=min(0.1, remaining))
                    except queue.Empty:
                        continue
                    if msg.kind == "BOOT":
                        raise RuntimeError(
                            f"Arduino reset mid-command — {msg.args[0]}"
                        )
                    if msg.kind == "D" and msg.seq == seq:
                        status = msg.args[1] if len(msg.args) > 1 else "?"
                        if status == "OK":
                            return _done_counts(msg)
                        detail = ",".join(msg.args[2:])
                        text = MOVE_FAIL_TEXT.get(
                            status, f"unrecognised status {status!r}"
                        )
                        raise RuntimeError(
                            f"Arduino returned ERR — {text} [{detail}]"
                        )
                    # stale D from an earlier aborted wait: ignore
            finally:
                self._awaiting_done.clear()

    # ------------------------------------------------------------------ #
    # Transaction machinery
    # ------------------------------------------------------------------ #

    def _next_seq(self) -> int:
        # Capped below ESTOP_SEQ_MIN so normal traffic never collides with the
        # emergency-stop band (see emergency_stop()).
        self._seq = (self._seq + 1) % (NORMAL_SEQ_MAX + 1)
        return self._seq

    def _transact(self, kind: str, *args: str,
                  retries: int = CMD_RETRIES,
                  ack_timeout: float = ACK_TIMEOUT_S) -> int:
        with self._cmd_lock:
            return self._transact_locked(
                kind, *args, retries=retries, ack_timeout=ack_timeout
            )

    def _transact_locked(self, kind: str, *args: str,
                         retries: int = CMD_RETRIES,
                         ack_timeout: float = ACK_TIMEOUT_S) -> int:
        """Send a command and wait for its A/N reply.  Retries re-send the
        SAME sequence number: the firmware deduplicates, so a command is
        executed at most once no matter how many times the ACK is lost."""
        seq = self._next_seq()
        body = ",".join((kind, str(seq)) + args)
        self._drain(self._reply_q)
        self._awaiting_reply.set()
        try:
            for attempt in range(1, retries + 2):
                self._raw_send(body)
                deadline = time.monotonic() + ack_timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        msg = self._reply_q.get(timeout=min(0.1, remaining))
                    except queue.Empty:
                        continue
                    if msg.kind == "BOOT":
                        raise RuntimeError(
                            f"Arduino reset mid-command — {msg.args[0]}"
                        )
                    if msg.seq != seq:
                        continue          # stale reply from a previous cmd
                    if msg.kind == "A":
                        return seq
                    if msg.kind == "N":
                        reason = msg.args[1] if len(msg.args) > 1 else "?"
                        raise ProtocolError(
                            f"Arduino rejected {kind!r}: {reason}"
                        )
                if attempt <= retries:
                    logger.debug(
                        "No ACK for %s (seq %d, attempt %d) — retrying",
                        kind, seq, attempt,
                    )
            # All retries exhausted.  If the board browned out and was
            # reset, its boot frame arrives seconds later (bootloader
            # delay) — wait briefly so the error names the real cause
            # instead of a generic timeout.
            boot_deadline = time.monotonic() + 3.0
            while True:
                remaining = boot_deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    msg = self._reply_q.get(timeout=min(0.2, remaining))
                except queue.Empty:
                    continue
                if msg.kind == "BOOT":
                    raise RuntimeError(
                        f"Arduino reset mid-command — {msg.args[0]}"
                    )
            raise TimeoutError(
                f"No ACK from Arduino within "
                f"{ack_timeout * (retries + 1):.1f} s and no boot frame "
                f"afterwards — the board is likely HUNG (a supply dip "
                f"with the brown-out detector disabled locks the chip up "
                f"instead of resetting it). Reopen the port or power-cycle."
            )
        finally:
            self._awaiting_reply.clear()

    def _raw_send(self, body: str) -> None:
        frame = encode_frame(body)
        with self._write_lock:
            self._ser.write(frame)
            self._ser.flush()

    @staticmethod
    def _drain(q: "queue.Queue[Message]") -> None:
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                break

    # ------------------------------------------------------------------ #
    # Background threads
    # ------------------------------------------------------------------ #

    def _heartbeat_loop(self) -> None:
        """Feed the firmware's link watchdog.  P frames are fire-and-forget
        — cheap enough to send always, essential while driving."""
        while not self._hb_stop.wait(PING_INTERVAL_S):
            if self._closed:
                return
            try:
                self._raw_send("P")
            except Exception:
                return

    def _read_loop(self) -> None:
        while not self._reader_stop.is_set():
            try:
                data = self._ser.read(256)
            except Exception:
                if not self._closed:
                    logger.error("Serial read failed — reader stopping")
                return
            if not data:
                continue
            for body in self._parser.feed(data):
                msg = parse_body(body)
                if msg is not None:
                    self._dispatch(msg)

    def _dispatch(self, msg: Message) -> None:
        logger.debug("RX: %s", msg)

        if msg.kind in ("A", "N"):
            if self._awaiting_reply.is_set():
                self._reply_q.put(msg)
            return

        if msg.kind == "D":
            if self._awaiting_done.is_set():
                self._done_q.put(msg)
            else:
                logger.warning(
                    "Unsolicited move-complete from firmware: %s "
                    "(link watchdog or stop during teardown)", msg.args,
                )
            return

        if msg.kind == "E":
            try:
                if self._on_encoder and len(msg.args) >= 2:
                    self._on_encoder(int(msg.args[0]), int(msg.args[1]))
                if self._on_odometry and len(msg.args) >= 5:
                    self._on_odometry(int(msg.args[2]), int(msg.args[3]),
                                      int(msg.args[4]))
            except ValueError:
                pass                        # corrupt-but-valid-checksum: drop
            return

        if msg.kind == "B":
            cause = msg.args[0] if msg.args else "?"
            build = msg.args[1] if len(msg.args) > 1 else "unknown build"
            self.firmware_build = build
            # Visible at WARNING so every run's output records exactly
            # which build is on the board.
            logger.warning("Arduino firmware: %s", build)
            self._boot_seen.set()
            if self._connecting:
                return                      # expected DTR reset on port open
            self.resets_seen += 1
            desc = describe_reset(cause)
            boot_marker = Message(kind="BOOT", args=(desc,))
            delivered = False
            if self._awaiting_reply.is_set():
                self._reply_q.put(boot_marker)
                delivered = True
            if self._awaiting_done.is_set():
                self._done_q.put(boot_marker)
                delivered = True
            if not delivered:
                logger.error("Arduino reset while idle — %s", desc)
                if self._on_error:
                    self._on_error()
            return

        if msg.kind == "W":
            self.warnings_seen += 1
            code = msg.args[0] if msg.args else "?"
            detail = ",".join(msg.args[1:])
            template = WARN_TEXT.get(code)
            if template:
                logger.warning(template.format(detail=detail))
            else:
                logger.warning("Firmware warning %s [%s]", code, detail)
            return

        logger.debug("Unhandled frame kind %r: %s", msg.kind, msg.args)
