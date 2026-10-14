import numpy as np
import pytest
from lagrange.viewer.v1 import common_pb2
from lagrange_mock_server.recording import RecordingError, read_recording, write_recording
from lagrange_mock_server.sample import DT_ELECTRICAL, DT_MECHANICAL, DT_THERMAL, generate_sample


@pytest.fixture(scope="module")
def sample():
    return generate_sample(duration_s=2.0, seed=1)


def test_sample_has_signals_at_their_native_rates(sample):
    assert sample.meta.synthetic
    assert sample.duration_ns == 2_000_000_000
    assert sample.signal("motor/left_drive_motor/current_q").sample_count == 20_000
    assert sample.signal("motor/left_drive_motor/current_q").dt_ns == DT_ELECTRICAL
    assert sample.signal("joint/left_wheel_joint/velocity").sample_count == 2_000
    assert sample.signal("joint/left_wheel_joint/velocity").dt_ns == DT_MECHANICAL
    assert sample.signal("thermal/left_drive_motor/temperature").sample_count == 20
    assert sample.signal("thermal/left_drive_motor/temperature").dt_ns == DT_THERMAL


def test_sample_values_are_physically_plausible(sample):
    voltage = sample.signal("bus/dc_48v/voltage").data
    assert voltage.min() > 44.8 and voltage.max() < 60.0  # between the cut-off and trip levels
    assert np.abs(sample.signal("motor/left_drive_motor/current_q").data).max() < 30.0
    soc = sample.signal("battery/battery/soc").data[:, 0]
    # Starts at 82 % and barely moves in two seconds; braking may raise it slightly.
    assert soc.min() > 0.81 and soc.max() < 0.83
    assert sample.signal("thermal/left_drive_motor/temperature").data.min() > 293.15
    pose = sample.signal("body/base_link/pose").data
    assert np.allclose(np.linalg.norm(pose[:, 3:7], axis=1), 1.0)  # unit quaternions
    assert np.all(np.isfinite(pose))


def test_sample_units_are_si_with_a_kind(sample):
    for signal in sample.signals:
        assert signal.info.unit.scale == 1.0, signal.info.name
        assert signal.info.unit.kind != 0, signal.info.name
        assert signal.info.unit.display, signal.info.name
    torque = sample.signal("joint/left_wheel_joint/torque").info.unit
    assert (torque.length, torque.mass, torque.time) == (2, 1, -2)
    assert sample.signal("body/base_link/pose").info.type == common_pb2.SIGNAL_TYPE_POSE


def test_sample_scene_is_consistent(sample):
    snapshot = sample.snapshot
    assert len(snapshot.transforms.inline) == 52 * len(snapshot.prims)
    ids = {prim.id for prim in snapshot.prims}
    for prim in snapshot.prims:
        assert prim.parent == 0xFFFFFFFF or prim.parent in ids
        assert prim.mesh == 0xFFFFFFFF or prim.mesh < len(snapshot.meshes)
    for mesh in snapshot.meshes:
        assert len(mesh.positions.inline) == 12 * mesh.vertex_count
        assert len(mesh.normals.inline) == 12 * mesh.vertex_count
        indices = np.frombuffer(mesh.indices.inline, dtype="<u4")
        assert len(indices) == mesh.index_count and len(indices) % 3 == 0
        assert indices.max() < mesh.vertex_count
    names = {signal.info.name for signal in sample.signals}
    for net in snapshot.nets:
        assert net.voltage_signal in names
        assert all(terminal.current_signal in names for terminal in net.terminals)


def test_generation_is_deterministic():
    a, b = generate_sample(1.0, seed=7), generate_sample(1.0, seed=7)
    for x, y in zip(a.signals, b.signals, strict=True):
        assert np.array_equal(x.data, y.data)
    other = generate_sample(1.0, seed=8)
    assert not np.array_equal(
        a.signal("bus/dc_48v/voltage").data, other.signal("bus/dc_48v/voltage").data
    )


def test_file_round_trip_is_bit_exact(sample, tmp_path):
    path = tmp_path / "sample.mcap"
    write_recording(sample, path)
    loaded = read_recording(path)
    assert loaded.meta == sample.meta
    assert loaded.snapshot == sample.snapshot
    assert len(loaded.signals) == len(sample.signals)
    for original, restored in zip(sample.signals, loaded.signals, strict=True):
        assert restored.info == original.info
        assert np.array_equal(restored.data, original.data)


def test_writing_twice_gives_identical_files(sample, tmp_path):
    write_recording(sample, tmp_path / "a.mcap")
    write_recording(sample, tmp_path / "b.mcap")
    assert (tmp_path / "a.mcap").read_bytes() == (tmp_path / "b.mcap").read_bytes()


def test_file_is_plain_mcap_with_protobuf_schemas(sample, tmp_path):
    from mcap.reader import make_reader

    path = tmp_path / "sample.mcap"
    write_recording(sample, path)
    with path.open("rb") as stream:
        summary = make_reader(stream).get_summary()
    topics = {channel.topic for channel in summary.channels.values()}
    assert {"/lagrange/meta", "/lagrange/catalog", "/lagrange/scene/snapshot"} <= topics
    assert f"/lagrange/signals/{DT_ELECTRICAL}ns" in topics
    assert {schema.encoding for schema in summary.schemas.values()} == {"protobuf"}
    assert "lagrange.viewer.v1.SignalBlock" in {s.name for s in summary.schemas.values()}


def test_unreadable_files_are_reported(tmp_path):
    garbage = tmp_path / "garbage.mcap"
    garbage.write_bytes(b"this is not an mcap file")
    with pytest.raises(RecordingError):
        read_recording(garbage)


def test_branch_currents_of_the_sample_add_up_to_the_battery_current():
    """Design acceptance (04-electrical): the schematic shows both, so they must agree."""
    recording = generate_sample(duration_s=1.0, seed=3)
    net = recording.snapshot.nets[0]
    source = [t for t in net.terminals if t.port.endswith("_out")]
    loads = [t for t in net.terminals if t.port.endswith("_in")]
    assert len(source) == 1 and len(loads) == 3
    drawn = sum(recording.signal(t.current_signal).data[:, 0] for t in loads)
    supplied = recording.signal(source[0].current_signal).data[:, 0]
    assert drawn == pytest.approx(supplied, abs=1e-9)
    # Every terminal names a prim of the scene and a signal of the catalog.
    for terminal in net.terminals:
        assert terminal.prim < len(recording.snapshot.prims)
