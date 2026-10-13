import pytest
from lagrange_mock_server.framing import FrameError, FrameReader, encode_frame


def test_header_is_little_endian_length():
    assert encode_frame(b"abc") == b"\x03\x00\x00\x00abc"
    assert encode_frame(b"") == b"\x00\x00\x00\x00"


def test_reads_frames_delivered_one_byte_at_a_time():
    stream = encode_frame(b"hello") + encode_frame(b"") + encode_frame(b"world!")
    reader = FrameReader()
    frames = []
    for byte in stream:
        reader.feed(bytes([byte]))
        frames.extend(reader.frames())
    assert frames == [b"hello", b"", b"world!"]
    assert reader.buffered_bytes == 0


def test_truncated_stream_yields_nothing():
    reader = FrameReader()
    reader.feed(encode_frame(b"truncated")[:-1])
    assert reader.next() is None
    assert reader.next() is None


def test_limit_is_inclusive_and_enforced_on_both_sides():
    assert encode_frame(b"12345678", max_frame_size=8)
    with pytest.raises(FrameError):
        encode_frame(b"123456789", max_frame_size=8)
    reader = FrameReader(max_frame_size=8)
    reader.feed(encode_frame(b"12345678"))
    assert reader.next() == b"12345678"


def test_oversize_header_fails_from_the_header_alone_and_stays_failed():
    reader = FrameReader(max_frame_size=1024)
    reader.feed(b"\xff\xff\xff\xff")
    with pytest.raises(FrameError):
        reader.next()
    reader.feed(encode_frame(b"ok"))
    assert reader.buffered_bytes == 0
    with pytest.raises(FrameError):
        reader.next()
