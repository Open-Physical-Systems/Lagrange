"""Length-prefixed frames of the viewer protocol.

A frame is a u32 little-endian length followed by that many payload bytes; the payload is a
serialised lagrange.viewer.v1.Envelope. This is the Python counterpart of
lagrange::proto::FrameReader and must stay byte-compatible with it.
"""

from __future__ import annotations

import struct

FRAME_HEADER = struct.Struct("<I")
DEFAULT_MAX_FRAME_SIZE = 64 * 1024 * 1024


class FrameError(Exception):
    """The byte stream is not a valid sequence of frames. The connection must be closed."""


def encode_frame(payload: bytes, max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> bytes:
    """Return header + payload. Raises FrameError if the payload is over the limit."""
    if len(payload) > max_frame_size:
        raise FrameError(f"payload of {len(payload)} bytes exceeds the limit of {max_frame_size}")
    return FRAME_HEADER.pack(len(payload)) + payload


class FrameReader:
    """Splits a byte stream into frames. Feed it bytes in pieces of any size."""

    def __init__(self, max_frame_size: int = DEFAULT_MAX_FRAME_SIZE) -> None:
        self._max = max_frame_size
        self._buffer = bytearray()
        self._failed = False

    def feed(self, data: bytes) -> None:
        if not self._failed:
            self._buffer += data

    def next(self) -> bytes | None:
        """Return the next complete payload, or None if more bytes are needed.

        Raises FrameError when a header announces a payload over the limit, and on every call
        after that.
        """
        if self._failed:
            raise FrameError("stream already failed")
        if len(self._buffer) < FRAME_HEADER.size:
            return None
        (length,) = FRAME_HEADER.unpack_from(self._buffer)
        if length > self._max:
            self._failed = True
            self._buffer.clear()
            raise FrameError(f"frame of {length} bytes exceeds the limit of {self._max}")
        end = FRAME_HEADER.size + length
        if len(self._buffer) < end:
            return None
        payload = bytes(self._buffer[FRAME_HEADER.size : end])
        del self._buffer[:end]
        return payload

    def frames(self):
        """Yield every complete payload currently buffered."""
        while (payload := self.next()) is not None:
            yield payload

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)
