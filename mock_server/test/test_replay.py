import numpy as np
import pytest
from lagrange.viewer.v1 import session_pb2
from lagrange_mock_server.replay import (
    MODE_FULL,
    MODE_MINMAX,
    MODE_STRIDE,
    ReplayEngine,
    Subscription,
    Window,
)
from lagrange_mock_server.sample import generate_sample


@pytest.fixture(scope="module")
def recording():
    return generate_sample(duration_s=1.0, seed=1)


def samples(run):
    return np.frombuffer(run.samples, dtype="<f8").reshape(run.sample_count, -1)


def test_paused_engine_does_not_advance(recording):
    engine = ReplayEngine(recording)
    assert engine.advance(20_000_000) is None
    assert engine.sim_time_ns == 0


def test_advance_scales_wall_time_by_the_real_time_factor(recording):
    engine = ReplayEngine(recording)
    engine.play()
    assert engine.advance(20_000_000) == Window(0, 20_000_000)
    engine.set_rtf(0.5)
    assert engine.advance(20_000_000) == Window(20_000_000, 30_000_000)
    with pytest.raises(ValueError):
        engine.set_rtf(-1.0)


def test_consecutive_windows_give_gap_free_full_runs(recording):
    engine = ReplayEngine(recording)
    engine.play()
    signal = recording.signal("motor/left_drive_motor/current_q")
    subscription = [Subscription(signal.info.id, MODE_FULL)]
    collected, expected_index = [], 0
    for wall_ns in (20_000_000, 13_337_000, 1, 99, 50_000, 26_612_900):  # irregular ticks
        window = engine.advance(wall_ns)
        for run in engine.runs(window, subscription):
            assert run.first_index == expected_index
            assert run.t0.ns == run.first_index * run.dt_ns
            expected_index += run.sample_count
            collected.append(samples(run))
    data = np.concatenate(collected)
    assert np.array_equal(data, signal.data[: len(data)])
    assert expected_index == -(-engine.sim_time_ns // signal.dt_ns)  # every sample before now


def test_minmax_matches_brute_force_and_sends_each_bucket_once(recording):
    engine = ReplayEngine(recording)
    engine.play()
    signal = recording.signal("bus/dc_48v/voltage")
    n = 16
    subscription = [Subscription(signal.info.id, MODE_MINMAX, n)]
    buckets, expected_bucket = [], 0
    for wall_ns in (20_000_000, 333_000, 7_777_700, 41_000_000):
        for run in engine.runs(engine.advance(wall_ns), subscription):
            assert run.dt_ns == n * signal.dt_ns
            assert run.first_index == expected_bucket
            expected_bucket += run.sample_count
            buckets.append(samples(run))
    got = np.concatenate(buckets)
    reference = signal.data[: len(got) * n].reshape(len(got), n)
    assert np.array_equal(got[:, 0], reference.min(axis=1))
    assert np.array_equal(got[:, 1], reference.max(axis=1))
    assert len(got) == -(-engine.sim_time_ns // signal.dt_ns) // n  # every complete bucket


def test_stride_takes_every_nth_sample(recording):
    engine = ReplayEngine(recording)
    signal = recording.signal("joint/left_wheel_joint/velocity")
    run = engine.runs(Window(0, 100_000_000), [Subscription(signal.info.id, MODE_STRIDE, 10)])[0]
    assert run.sample_count == 10 and run.first_index == 0
    assert np.array_equal(samples(run), signal.data[0:100:10])
    later = engine.runs(
        Window(100_000_000, 205_000_000), [Subscription(signal.info.id, MODE_STRIDE, 10)]
    )[0]
    assert later.first_index == 10
    assert np.array_equal(samples(later), signal.data[100:205:10])


def test_looping_replay_wraps_and_non_looping_pauses_at_the_end(recording):
    looping = ReplayEngine(recording, loop=True)
    looping.play()
    looping.seek(990_000_000)
    assert looping.advance(20_000_000) == Window(990_000_000, 1_000_000_000, wrapped=True)
    assert looping.sim_time_ns == 0
    assert looping.state == session_pb2.RUN_STATE_RUNNING

    once = ReplayEngine(recording, loop=False)
    once.play()
    once.seek(990_000_000)
    assert once.advance(20_000_000) == Window(990_000_000, 1_000_000_000)
    assert once.state == session_pb2.RUN_STATE_PAUSED
    assert once.advance(20_000_000) is None


def test_seek_is_clamped_and_step_works_while_paused(recording):
    engine = ReplayEngine(recording, loop=False)
    engine.seek(-5)
    assert engine.sim_time_ns == 0
    engine.seek(10**12)
    assert engine.sim_time_ns == recording.duration_ns
    engine.seek(0)
    assert engine.step(1_000_000) == Window(0, 1_000_000)
    assert engine.state == session_pb2.RUN_STATE_PAUSED


def test_delta_and_snapshot_carry_the_pose_at_the_requested_time(recording):
    engine = ReplayEngine(recording)
    pose = recording.signal("body/base_link/pose").data[250]
    delta = engine.delta(250_000_000)
    assert len(delta.transform_data) == 40 * len(delta.transform_prims)
    slot = list(delta.transform_prims).index(1)  # prim 1 is base_link
    position = np.frombuffer(delta.transform_data, dtype="<f8", count=3, offset=40 * slot)
    assert np.array_equal(position, pose[:3])

    snapshot = engine.snapshot(250_000_000)
    assert len(snapshot.transforms.inline) == 52 * len(snapshot.prims)
    position = np.frombuffer(snapshot.transforms.inline, dtype="<f8", count=3, offset=52 * 1)
    assert np.array_equal(position, pose[:3])
    assert snapshot.sim_time.ns == 250_000_000
    assert recording.snapshot.sim_time.ns == 0  # the stored snapshot is not modified
