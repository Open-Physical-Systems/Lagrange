"""The committed golden frames, read by the Python decoder."""

import struct
import sys
from pathlib import Path

from lagrange.viewer.v1 import envelope_pb2

GOLDEN_DIR = Path(__file__).parent / "golden"
sys.path.insert(0, str(GOLDEN_DIR))
import generate_golden  # noqa: E402


def frames(data: bytes) -> list[bytes]:
    out, offset = [], 0
    while offset < len(data):
        (length,) = struct.unpack_from("<I", data, offset)
        out.append(data[offset + 4 : offset + 4 + length])
        offset += 4 + length
    assert offset == len(data)
    return out


def test_committed_file_matches_the_generator():
    # If this fails after a deliberate protocol change, regenerate the file (see the script).
    assert (GOLDEN_DIR / "frames.bin").read_bytes() == generate_golden.golden_bytes()


def test_every_frame_reserialises_to_the_same_bytes():
    payloads = frames((GOLDEN_DIR / "frames.bin").read_bytes())
    bodies = envelope_pb2.Envelope.DESCRIPTOR.oneofs_by_name["body"].fields
    assert len(payloads) == len(bodies) + 1
    for field, payload in zip(bodies, payloads, strict=False):
        envelope = envelope_pb2.Envelope.FromString(payload)
        assert envelope.WhichOneof("body") == field.name
        assert envelope.SerializeToString(deterministic=True) == payload, field.name
    assert payloads[-1] == b""


def test_known_values():
    payloads = frames((GOLDEN_DIR / "frames.bin").read_bytes())
    hello = envelope_pb2.Envelope.FromString(payloads[0]).client_hello
    assert hello.version.major == 11 and hello.version.minor == 21
    assert hello.client_name == "左車輪_21"
    assert list(hello.compression) == ["左車輪_41", "左車輪_42", "左車輪_43"]
