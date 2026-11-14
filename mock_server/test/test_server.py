"""The mock server and the conformance client against each other, over real sockets."""

import asyncio
import stat
from pathlib import Path

import numpy as np
import pytest
from lagrange.viewer.v1 import envelope_pb2, session_pb2, signals_pb2
from lagrange_mock_server.client import ProtocolClient, run_conformance
from lagrange_mock_server.endpoint import Endpoint, parse_endpoint
from lagrange_mock_server.replay import ReplayEngine
from lagrange_mock_server.sample import generate_sample
from lagrange_mock_server.server import MockServer


@pytest.fixture(scope="module")
def recording():
    return generate_sample(duration_s=1.0, seed=1)


def endpoints(tmp_path):
    return [
        Endpoint("unix", str(tmp_path / "lagrange" / "s.sock")),
        Endpoint("tcp", "127.0.0.1", 0),
    ]


async def started(recording, endpoint, loop=True):
    server = MockServer(ReplayEngine(recording, loop=loop), endpoint)
    return server, await server.start()


async def receive_until(client, body, limit=200):
    for _ in range(limit):
        envelope = await client.receive()
        assert envelope is not None, f"connection closed while waiting for {body}"
        if envelope.WhichOneof("body") == body:
            return envelope
    raise AssertionError(f"no {body} within {limit} messages")


@pytest.mark.parametrize("kind", ["unix", "tcp"])
def test_conformance_passes_on_both_transports(recording, tmp_path, kind):
    async def scenario():
        endpoint = next(e for e in endpoints(tmp_path) if e.kind == kind)
        server, bound = await started(recording, endpoint)
        try:
            return await run_conformance(bound, sim_seconds=0.4)
        finally:
            await server.stop()

    report = asyncio.run(scenario())
    assert report.errors == []
    assert report.server_name.startswith("lagrange_mock_server")
    assert report.signals_in_catalog == len(recording.signals)
    assert report.snapshots >= 2  # the requested one and the one after seek
    assert report.deltas > 5 and report.signal_blocks > 5
    # 0.4 s of a 10 kHz signal is at least 4000 samples, none missing.
    assert report.samples["motor/left_drive_motor/current_q"] >= 4000
    assert report.samples["joint/left_wheel_joint/velocity"] >= 400


def test_conformance_holds_across_the_loop_restart(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0])
        server.engine.set_rtf(4.0)
        try:
            client = await ProtocolClient.connect(bound)
            await client.hello()
            await client.command(play={})
            # Let the 1 s recording wrap at least once at 4x, then check a fresh client.
            await asyncio.sleep(0.4)
            await client.close()
            return await run_conformance(bound, sim_seconds=0.3)
        finally:
            await server.stop()

    assert asyncio.run(scenario()).errors == []


def test_decimated_subscriptions_pass_conformance(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0])
        try:
            return await run_conformance(
                bound,
                sim_seconds=0.3,
                subscribe={
                    "bus/dc_48v/voltage": (signals_pb2.SignalSubscription.MODE_MINMAX, 20),
                    "motor/left_drive_motor/current_q": (
                        signals_pb2.SignalSubscription.MODE_STRIDE,
                        5,
                    ),
                },
            )
        finally:
            await server.stop()

    report = asyncio.run(scenario())
    assert report.errors == []
    assert set(report.samples) == {"bus/dc_48v/voltage", "motor/left_drive_motor/current_q"}
    assert report.samples["bus/dc_48v/voltage"] >= 3000 // 20 - 1


def test_handshake_sends_profile_catalog_and_status(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0])
        try:
            client = await ProtocolClient.connect(bound)
            hello = await client.hello()
            first_three = [(await client.receive()).WhichOneof("body") for _ in range(3)]
            await client.close()
            return hello, first_three
        finally:
            await server.stop()

    hello, first_three = asyncio.run(scenario())
    assert hello.server_hello.server_kind == session_pb2.SERVER_KIND_MOCK
    assert hello.server_hello.capabilities.can_seek
    assert not hello.server_hello.shm_enabled
    assert hello.server_hello.session_id != 0
    assert first_three == ["profile_state", "catalog", "status"]


def test_wrong_major_version_is_refused_with_a_reason(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0])
        try:
            client = await ProtocolClient.connect(bound)
            hello = await client.hello(major=7)
            closed = await client.receive()
            await client.close()
            return hello, closed
        finally:
            await server.stop()

    hello, closed = asyncio.run(scenario())
    assert "7.1" in hello.server_hello.refusal and "0.1" in hello.server_hello.refusal
    assert closed is None  # the server closed the connection after refusing


def test_first_message_must_be_client_hello(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0])
        try:
            client = await ProtocolClient.connect(bound)
            ping = envelope_pb2.Envelope()
            ping.ping.nonce = 1
            await client.send(ping)
            reply = await client.receive()
            closed = await client.receive()
            await client.close()
            return reply, closed
        finally:
            await server.stop()

    reply, closed = asyncio.run(scenario())
    assert reply.server_hello.refusal
    assert closed is None


def test_ping_seek_step_and_unsupported_commands(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[0], loop=False)
        try:
            client = await ProtocolClient.connect(bound)
            await client.hello()
            ping = envelope_pb2.Envelope()
            ping.ping.nonce = 99
            await client.send(ping)
            pong = await receive_until(client, "pong")

            seek = await client.command(seek={"to": {"ns": 500_000_000}})
            seek_result = await receive_until(client, "command_result")
            snapshot = await receive_until(client, "snapshot")
            step = await client.command(step={"durationNs": 2_000_000})
            await receive_until(client, "command_result")
            status = await receive_until(client, "status")

            await client.command(load_stage={"path": "/nope.usda"})
            unsupported = await receive_until(client, "command_result")
            await client.command(remove_robot={"name": "amr_diffdrive"})
            removed = await receive_until(client, "command_result")
            await client.command(
                command_joint={"joint": "left_wheel_joint", "mode": "position", "value": 1.0}
            )
            joint = await receive_until(client, "command_result")
            await client.close()
            return pong, seek, seek_result, snapshot, step, status, unsupported, removed, joint
        finally:
            await server.stop()

    pong, seek, seek_result, snapshot, _step, status, unsupported, removed, joint = asyncio.run(
        scenario()
    )
    assert pong.pong.nonce == 99
    assert seek_result.command_result.request_id == seek and seek_result.command_result.ok
    assert snapshot.snapshot.sim_time.ns == 500_000_000
    assert status.status.sim_time.ns == 502_000_000
    assert status.status.state == session_pb2.RUN_STATE_PAUSED
    assert not unsupported.command_result.ok
    assert "load_stage" in unsupported.command_result.error
    assert not removed.command_result.ok
    assert "no spawned robot" in removed.command_result.error
    assert not joint.command_result.ok
    assert "take no commands" in joint.command_result.error


def test_garbage_from_one_client_does_not_affect_another(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, endpoints(tmp_path)[1])
        try:
            reader, writer = await asyncio.open_connection(bound.address, bound.port)
            writer.write(b"\xff\xff\xff\xff" + b"not a frame at all")  # oversize header
            await writer.drain()
            assert await reader.read() == b""  # server closes it
            writer.close()
            reader, writer = await asyncio.open_connection(bound.address, bound.port)
            writer.write(b"\x03\x00\x00\x00\xff\xff\xff")  # a frame that is not a protobuf
            await writer.drain()
            await reader.read()
            writer.close()
            return await run_conformance(bound, sim_seconds=0.1)
        finally:
            await server.stop()

    assert asyncio.run(scenario()).errors == []


def test_unix_socket_is_private_removed_on_stop_and_not_stolen(recording, tmp_path):
    async def scenario():
        endpoint = endpoints(tmp_path)[0]
        path = Path(endpoint.address)
        server, _ = await started(recording, endpoint)
        mode = stat.S_IMODE(path.stat().st_mode)
        parent_mode = stat.S_IMODE(path.parent.stat().st_mode)
        second = MockServer(ReplayEngine(recording), endpoint)
        with pytest.raises(OSError, match="another server"):
            await second.start()
        still_there = path.exists()
        await server.stop()
        return mode, parent_mode, still_there, path.exists()

    mode, parent_mode, still_there, after_stop = asyncio.run(scenario())
    assert mode == 0o600
    assert parent_mode == 0o700
    assert still_there
    assert not after_stop


def test_endpoint_strings_match_the_cpp_forms():
    assert parse_endpoint("unix:/run/user/1000/lagrange/default.sock") == Endpoint(
        "unix", "/run/user/1000/lagrange/default.sock"
    )
    assert parse_endpoint("/tmp/x.sock") == Endpoint("unix", "/tmp/x.sock")
    assert parse_endpoint("tcp://127.0.0.1:5000") == Endpoint("tcp", "127.0.0.1", 5000)
    assert parse_endpoint("tcp://sim-host") == Endpoint("tcp", "sim-host", 47610)
    assert parse_endpoint("tcp://[::1]:47610") == Endpoint("tcp", "::1", 47610)
    for text in ("unix:/tmp/x.sock", "tcp://127.0.0.1:5000", "tcp://[::1]:47610"):
        assert str(parse_endpoint(text)) == text
    for bad in (
        "",
        "unix:",
        "tcp://",
        "tcp://:5000",
        "tcp://host:",
        "tcp://host:0",
        "tcp://host:65536",
        "tcp://host:abc",
        "tcp://[::1",
        "http://host:80",
        "relative.sock",
    ):
        with pytest.raises(ValueError):
            parse_endpoint(bad)


def test_inspector_attributes_and_console_log(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, Endpoint("unix", str(tmp_path / "s.sock")))
        client = await ProtocolClient.connect(bound)
        try:
            await client.hello()
            # The console history is sent to a viewer when it connects.
            logs = []
            while len(logs) < 3:
                envelope = await client.receive()
                if envelope.WhichOneof("body") == "log":
                    logs.append(envelope.log)
            await client.command(get_attributes={"prims": [8, 10, 999]})
            result = (await receive_until(client, "command_result")).command_result
            return logs, result
        finally:
            await client.close()
            await server.stop()

    logs, result = asyncio.run(scenario())
    assert logs[0].source == "stage" and "15 prims" in logs[0].text
    assert logs[0].level == session_pb2.LOG_LEVEL_INFO
    assert "domains on: rigid_body" in logs[2].text

    assert result.ok and [item.prim for item in result.attributes.items] == [8, 10, 999]
    motor = {attribute.name: attribute for attribute in result.attributes.items[0].attributes}
    assert motor["elec:windingResistance"].real == pytest.approx(0.12)
    assert motor["elec:windingResistance"].unit.display == "\u2126"
    assert motor["elec:behaviorModel"].asset.endswith(".fmu")
    assert list(motor["elec:powerPort"].targets.prims) == [7]
    assert motor["thermal:heatCapacity"].unit.display == "J/K"
    # A prim without attributes, and one that does not exist, answer with an empty list.
    assert not result.attributes.items[1].attributes and not result.attributes.items[2].attributes


def test_lidar_frames_are_sent_to_viewers_that_ask(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, Endpoint("unix", str(tmp_path / "s.sock")))
        client = await ProtocolClient.connect(bound)
        try:
            await client.hello()
            await client.command(
                subscribe_sensor={"sensor": "/amr/sensors/lidar_front", "enabled": True}
            )
            await client.command(play={})
            return (await receive_until(client, "sensor_frame", limit=2000)).sensor_frame
        finally:
            await client.close()
            await server.stop()

    frame = asyncio.run(scenario())
    assert frame.sensor == "/amr/sensors/lidar_front"
    assert list(frame.points.fields) == ["x", "y", "z", "intensity"]
    assert frame.points.stride == 16
    assert frame.points.data.size == frame.points.count * 16 == len(frame.points.data.inline)
    # Four rings of 720 beams, nearly all of which hit a wall of the aisle.
    assert 2500 < frame.points.count <= 2880
    points = np.frombuffer(frame.points.data.inline, dtype="<f4").reshape(-1, 4)
    ranges = np.hypot(points[:, 0], points[:, 1])
    assert ranges.min() > 0.05 and ranges.max() < 25.0
    assert points[:, 3].min() > 0.0 and points[:, 3].max() <= 1.0


def test_run_facts_events_and_statistics_reach_a_viewer(tmp_path):
    recording = generate_sample(duration_s=10.0, seed=1)  # a whole figure of eight

    async def scenario():
        server, bound = await started(recording, Endpoint("unix", str(tmp_path / "s.sock")))
        client = await ProtocolClient.connect(bound)
        try:
            await client.hello()
            info = (await receive_until(client, "recording_info")).recording_info
            events = (await receive_until(client, "event_list")).event_list
            await client.command(play={})
            stats = (await receive_until(client, "server_stats", limit=5000)).server_stats
            return info, events, stats
        finally:
            await client.close()
            await server.stop()

    info, events, stats = asyncio.run(scenario())
    assert info.synthetic and info.path == "" and info.size_bytes == 0
    assert info.duration.ns == recording.duration_ns
    assert len(info.channels) == len(recording.signals)
    assert info.messages == sum(channel.messages for channel in info.channels)
    fast = next(c for c in info.channels if c.topic == "bus/dc_48v/voltage")
    assert fast.rate_hz == pytest.approx(10_000.0) and fast.unit == "V"

    assert events.complete and len(events.events) >= 2
    assert [e.id for e in events.events] == list(range(1, len(events.events) + 1))
    assert [e.begin.ns for e in events.events] == sorted(e.begin.ns for e in events.events)
    kinds = {e.kind for e in events.events}
    assert "hard_turn" in kinds
    for event in events.events:
        assert event.end.ns > event.begin.ns and event.text
        assert event.signal in {s.info.name for s in recording.signals}

    assert stats.estimated and stats.viewers == 1
    assert stats.memory_bytes > 1_000_000 and stats.tick_ms > 0.0
    assert stats.bytes_per_second_out > 0.0
    modules = {m.domain: m for m in stats.modules}
    assert modules["electrical"].dt_ns == 100_000 and modules["electrical"].cpu_ms_per_step > 0


def test_a_regen_event_names_the_signal_that_shows_it():
    from lagrange_mock_server.events import derive_events

    recording = generate_sample(duration_s=10.0, seed=1)
    regen = [e for e in derive_events(recording) if e.kind == "regen"]
    current = recording.signal("battery/battery/current")
    for event in regen:
        assert event.domain == "electrical" and event.signal == "battery/battery/current"
        begin, end = event.begin.ns // current.dt_ns, event.end.ns // current.dt_ns
        # Most of the span is below zero: that is what regeneration means.
        assert (current.data[begin:end, 0] < 0.0).mean() > 0.5


def test_environment_fields_ros_graph_and_world_reach_a_viewer_that_asks(recording, tmp_path):
    async def scenario():
        server, bound = await started(recording, Endpoint("unix", str(tmp_path / "s.sock")))
        client = await ProtocolClient.connect(bound)
        try:
            await client.hello()
            surroundings = (await receive_until(client, "environment")).environment
            await client.command(
                subscribe_field={"field": "emi/flux_density", "enabled": True, "heightM": 0.1}
            )
            await client.command(set_ros_graph={"enabled": True})
            await client.command(set_world={"enabled": True})
            await client.command(play={})
            seen = {}
            for _ in range(8000):
                envelope = await client.receive()
                body = envelope.WhichOneof("body")
                if body in ("field_slice", "ros_graph", "world") and body not in seen:
                    seen[body] = envelope
                if len(seen) == 3:
                    break
            return surroundings, seen
        finally:
            await client.close()
            await server.stop()

    surroundings, seen = asyncio.run(scenario())
    assert surroundings.synthetic
    fields = {field.name: field for field in surroundings.fields}
    assert fields["emi/flux_density"].available and fields["emi/flux_density"].unit.display == "µT"
    assert not fields["weather/wind"].available and fields["weather/wind"].unavailable_reason
    assert surroundings.emi_sources[0].frequency_hz == 20_000.0

    field = seen["field_slice"].field_slice
    assert (field.width, field.height, field.components) == (64, 48, 1)
    values = np.frombuffer(field.values.inline, dtype="<f4").reshape(field.height, field.width)
    assert field.values.size == values.nbytes
    assert (
        values.min() == pytest.approx(field.value_min) and values.max() <= field.value_max * 1.0001
    )
    assert values.min() > 0.0
    # The field is strongest in the cell nearest the robot, which is near the middle early on.
    row, column = np.unravel_index(values.argmax(), values.shape)
    assert 8 < column < 56 and 6 < row < 42

    graph = seen["ros_graph"].ros_graph
    assert graph.note and graph.distro == "lyrical" and graph.use_sim_time
    rates = {topic.name: topic.rate_hz for topic in graph.topics}
    # The same rates as the sensors and signals the viewer shows elsewhere.
    assert rates["/lidar_front/points"] == pytest.approx(10.0)
    assert rates["/imu/data"] == pytest.approx(1000.0)
    assert rates["/joint_states"] == pytest.approx(1000.0)
    assert {node.name for node in graph.node_info} >= {
        "/lagrange_ros2_bridge",
        "/controller_manager",
    }
    assert graph.control.active and len(graph.control.joints) == 2

    world = seen["world"].world
    assert world.synthetic and not world.geo_origin.valid and world.tile_size_m == 256.0
    assert len(world.tile_x) == len(world.tile_y) == len(world.tile_lod) == 13 * 13
    assert world.tile_lod.count(0) == 1  # only the tile the robot is in has full physics
    assert world.agents[0].name == "amr" and world.agents[0].focus
    assert world.floating_origin.anchor == "amr"
