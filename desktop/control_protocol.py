"""Tracking control channel, mirroring main/include/control_protocol.h.

The desktop reports WHERE the hand was, and when. Every decision about how the
gimbal moves is made on the board, so a late or lost packet degrades tracking
accuracy but can never produce a violent motion command.
"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct


COMMAND_MAGIC = 0x4743544C  # "GCTL"
STATUS_MAGIC = 0x47535441  # "GSTA"
# 2 added measurements; a version 1 board refuses these commands outright
# rather than misreading an angle as a speed.
VERSION = 2

COMMAND = struct.Struct("!IHHIhhHH")
STATUS = struct.Struct("!IHHIihhHH")

Q15_ONE = 32767
CENTIDEGREES = 100

# Command flags, desktop -> board.
FLAG_TRACKING = 1 << 0
FLAG_HAND_VISIBLE = 1 << 1
FLAG_RECENTER = 1 << 2
FLAG_MOTORS_OFF = 1 << 3
FLAG_MEASUREMENT = 1 << 4

# State flags, board -> desktop.
STATE_STEPPER_ENABLED = 1 << 0
STATE_TRACKING = 1 << 1
STATE_WATCHDOG_STOP = 1 << 2
STATE_PAN_AT_LIMIT = 1 << 3
STATE_TILT_AT_LIMIT = 1 << 4
STATE_RECENTERING = 1 << 5
STATE_PREDICTIVE = 1 << 6


@dataclass(frozen=True, slots=True)
class GimbalStatus:
    sequence: int
    pan_steps: int
    pan_rate: int
    tilt_degrees: float
    state_flags: int
    # Capture to arrival on the board for the newest measurement; 0 until one.
    vision_latency_ms: int = 0

    def describes(self, flag: int) -> bool:
        return bool(self.state_flags & flag)

    def summary(self) -> str:
        parts = []
        if self.describes(STATE_TRACKING):
            parts.append("predictive" if self.describes(STATE_PREDICTIVE) else "tracking")
        if self.describes(STATE_RECENTERING):
            parts.append("recentering")
        if self.describes(STATE_WATCHDOG_STOP):
            parts.append("WATCHDOG")
        if self.describes(STATE_PAN_AT_LIMIT):
            parts.append("pan limit")
        if self.describes(STATE_TILT_AT_LIMIT):
            parts.append("tilt limit")
        if not self.describes(STATE_STEPPER_ENABLED):
            parts.append("coils off")
        return ", ".join(parts) if parts else "idle"


def _to_q15(value: float) -> int:
    """Clamp to [-1, 1] and scale, so a vision bug cannot widen the range."""
    clamped = max(-1.0, min(1.0, value))
    return int(round(clamped * Q15_ONE))


def _to_centidegrees(degrees: float) -> int:
    return int(round(max(-327.67, min(327.67, degrees)) * CENTIDEGREES))


def encode_command(
    sequence: int, pan_error: float, tilt_error: float, flags: int
) -> bytes:
    """A manual speed command, as used by the jog and test tools."""
    return COMMAND.pack(
        COMMAND_MAGIC,
        VERSION,
        COMMAND.size,
        sequence & 0xFFFFFFFF,
        _to_q15(pan_error),
        _to_q15(tilt_error),
        flags & ~FLAG_MEASUREMENT & 0xFFFF,
        0,
    )


def encode_measurement(
    sequence: int,
    pan_degrees: float,
    tilt_degrees: float,
    flags: int,
    captured_at_us: int,
) -> bytes:
    """Where the hand was, in degrees from the camera axis, in the frame the
    board stamped captured_at_us (straight from the stream header)."""
    return COMMAND.pack(
        COMMAND_MAGIC,
        VERSION,
        COMMAND.size,
        sequence & 0xFFFFFFFF,
        _to_centidegrees(pan_degrees),
        _to_centidegrees(tilt_degrees),
        (flags | FLAG_MEASUREMENT) & 0xFFFF,
        (captured_at_us // 1000) & 0xFFFF,
    )


def receive_exactly(connection: socket.socket, byte_count: int) -> bytes:
    """Read exactly byte_count bytes or raise ConnectionError on disconnect."""
    chunks: list[bytes] = []
    remaining = byte_count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError("gimbal closed the control connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def receive_status(connection: socket.socket) -> GimbalStatus:
    (
        magic,
        version,
        message_bytes,
        sequence,
        pan_steps,
        pan_rate,
        tilt_decidegrees,
        state_flags,
        vision_latency_ms,
    ) = STATUS.unpack(receive_exactly(connection, STATUS.size))

    if magic != STATUS_MAGIC:
        raise ValueError(f"bad status magic 0x{magic:08x}")
    if version != VERSION or message_bytes != STATUS.size:
        raise ValueError(
            f"unsupported status version/size: {version}/{message_bytes} "
            f"(this desktop speaks version {VERSION}; reflash or update)"
        )

    return GimbalStatus(
        sequence=sequence,
        pan_steps=pan_steps,
        pan_rate=pan_rate,
        tilt_degrees=tilt_decidegrees / 10.0,
        state_flags=state_flags,
        vision_latency_ms=vision_latency_ms,
    )
