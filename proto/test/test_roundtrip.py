"""Round-trip tests for the generated Python protocol modules."""

import struct

from lagrange.viewer.v1 import (
    command_pb2,
    common_pb2,
    envelope_pb2,
    profile_pb2,
    scene_pb2,
    session_pb2,
    signals_pb2,
)


def roundtrip(message):
    parsed = type(message)()
    parsed.ParseFromString(message.SerializeToString())
    return parsed


def test_every_envelope_body_can_be_set_and_parsed():
    body = envelope_pb2.Envelope.DESCRIPTOR.oneofs_by_name["body"]
    assert len(body.fields) > 0
    for field in body.fields:
        envelope = envelope_pb2.Envelope()
        getattr(envelope, field.name).SetInParent()
        parsed = roundtrip(envelope)
        assert parsed.WhichOneof("body") == field.name


def test_handshake():
    hello = envelope_pb2.Envelope(
        client_hello=session_pb2.ClientHello(
            version=session_pb2.ProtocolVersion(major=0, minor=1),
            client_name="lagrange_viewer 0.1.0",
            wants_shm=True,
        )
    )
    parsed = roundtrip(hello)
    assert parsed.client_hello.version.minor == 1
    assert parsed.client_hello.wants_shm


def test_signal_run_raw_samples_are_bit_exact():
    samples = [0.0, 1e-300, 22.6, -51.3, 1.0 / 3.0]
    raw = struct.pack(f"<{len(samples)}d", *samples)
    block = signals_pb2.SignalBlock(
        catalog_generation=7,
        runs=[
            signals_pb2.SignalRun(
                signal=42,
                t0=common_pb2.SimTime(ns=12_340_000_000),
                dt_ns=100_000,
                sample_count=len(samples),
                components=1,
                encoding=signals_pb2.SAMPLE_ENCODING_F64_LE,
                samples=raw,
                first_index=123_400,
            )
        ],
    )
    parsed = roundtrip(block)
    assert parsed.runs[0].samples == raw
    assert list(struct.unpack(f"<{len(samples)}d", parsed.runs[0].samples)) == samples


def test_scene_snapshot_keeps_utf8_prim_paths():
    snapshot = scene_pb2.SceneSnapshot(
        sequence=100,
        prims=[scene_pb2.Prim(id=0, parent=0xFFFFFFFF, name="左車輪", path="/ロボット/左車輪")],
    )
    parsed = roundtrip(snapshot)
    assert parsed.prims[0].path == "/ロボット/左車輪"
    assert parsed.prims[0].parent == 0xFFFFFFFF


def test_command_and_result_share_request_id():
    command = command_pb2.Command(request_id=9, seek=command_pb2.Seek(to=common_pb2.SimTime(ns=5)))
    result = command_pb2.CommandResult(request_id=command.request_id, ok=True)
    assert roundtrip(command).WhichOneof("command") == "seek"
    assert roundtrip(result).request_id == 9


def test_profile_state_lists_domains():
    state = profile_pb2.ProfileState(
        schema_version=1,
        domains=[
            profile_pb2.DomainState(domain="electrical", enabled=True, dt_ns=100_000),
            profile_pb2.DomainState(
                domain="weather", enabled=False, substituted=True, active_module="clear"
            ),
        ],
    )
    parsed = roundtrip(state)
    assert [d.domain for d in parsed.domains] == ["electrical", "weather"]
    assert parsed.domains[1].substituted
