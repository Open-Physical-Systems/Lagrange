#!/usr/bin/env python3
"""Writes frames.bin: one frame for every body type of lagrange.viewer.v1.Envelope.

Every field of every message is filled with a value derived from its field number, so the
file exercises all field types. The C++ and Python tests parse it, re-serialise each message
and require the same bytes back, which shows that both decoders read every field alike.

Run after an intentional protocol change:
    python3 proto/test/golden/generate_golden.py proto/test/golden/frames.bin
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

from google.protobuf.descriptor import FieldDescriptor as Fd
from lagrange.viewer.v1 import envelope_pb2

REPEATED_COUNT = 3


def _value(field: Fd, v: int):
    kind = field.cpp_type
    if kind == Fd.CPPTYPE_INT32:
        return -v
    if kind == Fd.CPPTYPE_INT64:
        return -(v << 40)
    if kind == Fd.CPPTYPE_UINT32:
        return v
    if kind == Fd.CPPTYPE_UINT64:
        return (1 << 63) | v
    if kind == Fd.CPPTYPE_DOUBLE:
        return v + 0.125  # exactly representable
    if kind == Fd.CPPTYPE_FLOAT:
        return v + 0.5
    if kind == Fd.CPPTYPE_BOOL:
        return True
    if kind == Fd.CPPTYPE_ENUM:
        return field.enum_type.values[-1].number
    if field.type == Fd.TYPE_BYTES:
        return b"\x00\xff" + str(v).encode()
    return f"左車輪_{v}"


def fill(message, depth: int = 0) -> None:
    """Set every field of `message`. In a oneof the last member stays set."""
    if depth > 16:
        raise RecursionError("unexpected recursive message type")
    for field in message.DESCRIPTOR.fields:
        repeated = field.label == Fd.LABEL_REPEATED
        for k in range(REPEATED_COUNT if repeated else 1):
            v = field.number * 10 + k + 1
            if field.cpp_type == Fd.CPPTYPE_MESSAGE:
                child = (
                    getattr(message, field.name).add() if repeated else getattr(message, field.name)
                )
                child.SetInParent()
                fill(child, depth + 1)
            elif repeated:
                getattr(message, field.name).append(_value(field, v))
            else:
                setattr(message, field.name, _value(field, v))


def golden_payloads() -> list[bytes]:
    payloads = []
    for field in envelope_pb2.Envelope.DESCRIPTOR.oneofs_by_name["body"].fields:
        envelope = envelope_pb2.Envelope()
        body = getattr(envelope, field.name)
        body.SetInParent()
        fill(body)
        payloads.append(envelope.SerializeToString(deterministic=True))
    payloads.append(b"")  # an Envelope with no body: a zero-length frame
    return payloads


def golden_bytes() -> bytes:
    return b"".join(struct.pack("<I", len(p)) + p for p in golden_payloads())


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    Path(sys.argv[1]).write_bytes(golden_bytes())
    print(f"wrote {sys.argv[1]}: {len(golden_payloads())} frames, {len(golden_bytes())} bytes")
