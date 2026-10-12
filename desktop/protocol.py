"""Wire protocol shared conceptually with main/include/stream_protocol.h."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import time


MAGIC = 0x4F43414D  # ASCII "OCAM"
VERSION = 1
HEADER = struct.Struct("!IHHIQIHHI")
MAX_JPEG_BYTES = 2 * 1024 * 1024
JPEG_START = b"\xff\xd8\xff"


@dataclass(frozen=True, slots=True)
class CameraFrame:
    sequence: int
    captured_at_us: int
    width: int
    height: int
    dropped_on_camera: int
    jpeg: bytes
    # time.monotonic() when the last byte arrived. Arrival minus capture time is
    # how camera_pose.BoardClock lines the board's clock up with this one.
    received_at: float = 0.0


def receive_exactly(connection: socket.socket, byte_count: int) -> bytes:
    """Read exactly byte_count bytes or raise ConnectionError on disconnect."""
    chunks: list[bytes] = []
    remaining = byte_count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError("camera closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def newest_jpeg_offset(jpeg: bytes) -> int:
    """Where the newest JPEG in a payload starts.

    The camera driver sometimes misses a frame boundary and delivers two to four
    complete JPEGs glued end to end. A decoder stops at the first end marker, so
    decoding from the start shows the OLDEST of them, up to three frames stale.
    The newest begins at the last start marker. That byte sequence cannot occur
    inside compressed image data, where every 0xFF byte is stuffed.
    """
    return max(0, jpeg.rfind(JPEG_START))


def receive_frame(connection: socket.socket) -> CameraFrame:
    """Receive and validate one complete JPEG frame."""
    raw_header = receive_exactly(connection, HEADER.size)
    (
        magic,
        version,
        header_bytes,
        sequence,
        captured_at_us,
        jpeg_bytes,
        width,
        height,
        dropped_on_camera,
    ) = HEADER.unpack(raw_header)

    if magic != MAGIC:
        raise ValueError(f"bad stream magic 0x{magic:08x}")
    if version != VERSION or header_bytes != HEADER.size:
        raise ValueError(
            f"unsupported protocol version/header: {version}/{header_bytes}"
        )
    if not 0 < jpeg_bytes <= MAX_JPEG_BYTES:
        raise ValueError(f"unreasonable JPEG size: {jpeg_bytes} bytes")

    return CameraFrame(
        sequence=sequence,
        captured_at_us=captured_at_us,
        width=width,
        height=height,
        dropped_on_camera=dropped_on_camera,
        jpeg=receive_exactly(connection, jpeg_bytes),
        received_at=time.monotonic(),
    )
