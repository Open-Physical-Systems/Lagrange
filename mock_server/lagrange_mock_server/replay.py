"""Replay engine: turns a recording and a clock into protocol messages.

It has no sockets and no real clock, so it can be tested exactly. The server calls advance()
with the wall time that passed and sends what the engine returns.

Sample k of a signal is at time k * dt. A window [t0, t1) contains the samples with
t0 <= k * dt < t1, so consecutive windows give gap-free, non-overlapping runs.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from lagrange.viewer.v1 import scene_pb2, session_pb2, signals_pb2

from .recording import Recording, Signal, signal_run

MODE_FULL = signals_pb2.SignalSubscription.MODE_FULL
MODE_MINMAX = signals_pb2.SignalSubscription.MODE_MINMAX
MODE_STRIDE = signals_pb2.SignalSubscription.MODE_STRIDE

TRANSFORM_DELTA_BYTES = 40  # 3 f64 position + 4 f32 quaternion
TRANSFORM_SNAPSHOT_BYTES = 52  # + 3 f32 scale


@dataclass(frozen=True)
class Window:
    """Simulation time covered by one advance: [t0, t1)."""

    t0: int
    t1: int
    wrapped: bool = False  # the replay reached the end and restarted at 0 after this window


@dataclass(frozen=True)
class Subscription:
    signal: int
    mode: int = MODE_FULL
    decimation: int = 1


def _ceil_div(a: int, b: int) -> int:
    return -(-a // b)


class ReplayEngine:
    def __init__(self, recording: Recording, loop: bool = True) -> None:
        self.recording = recording
        self.loop = loop
        self.sim_time_ns = 0
        self.state = session_pb2.RUN_STATE_PAUSED
        self.rtf_target = 1.0
        self._bindings = [
            (binding.prim, recording.signal(binding.signal))
            for binding in recording.meta.pose_bindings
        ]

    # --- transport controls ----------------------------------------------------------------

    @property
    def duration_ns(self) -> int:
        return self.recording.duration_ns

    def play(self) -> None:
        if self.sim_time_ns >= self.duration_ns and not self.loop:
            self.sim_time_ns = 0
        self.state = session_pb2.RUN_STATE_RUNNING

    def pause(self) -> None:
        self.state = session_pb2.RUN_STATE_PAUSED

    def set_rtf(self, target: float) -> None:
        """Real-time factor. 0 is 'as fast as possible', which a replay treats as 1."""
        if target < 0 or not np.isfinite(target):
            raise ValueError("real-time factor must be a non-negative number")
        self.rtf_target = target if target > 0 else 1.0

    def seek(self, time_ns: int) -> None:
        self.sim_time_ns = min(max(0, time_ns), self.duration_ns)

    def advance(self, wall_dt_ns: int) -> Window | None:
        """Move the clock by wall time while running. None when paused."""
        if self.state != session_pb2.RUN_STATE_RUNNING or self.duration_ns <= 0:
            return None  # paused, or a static scene with no timeline
        return self._move(round(wall_dt_ns * self.rtf_target))

    def step(self, duration_ns: int) -> Window:
        """Advance by exactly `duration_ns` of simulation time, whatever the run state."""
        return self._move(max(0, duration_ns))

    def _move(self, sim_dt_ns: int) -> Window:
        t0 = self.sim_time_ns
        t1 = t0 + sim_dt_ns
        if t1 < self.duration_ns:
            self.sim_time_ns = t1
            return Window(t0, t1)
        if self.loop:
            self.sim_time_ns = 0
            return Window(t0, self.duration_ns, wrapped=True)
        self.sim_time_ns = self.duration_ns
        self.state = session_pb2.RUN_STATE_PAUSED
        return Window(t0, self.duration_ns)

    # --- signals ---------------------------------------------------------------------------

    def runs(
        self, window: Window, subscriptions: list[Subscription]
    ) -> list[signals_pb2.SignalRun]:
        """Signal runs for `window`, one per subscription that has samples in it."""
        out = []
        for subscription in subscriptions:
            signal = self.recording.signals[subscription.signal]
            first = min(_ceil_div(window.t0, signal.dt_ns), signal.sample_count)
            last = min(_ceil_div(window.t1, signal.dt_ns), signal.sample_count)
            if first >= last:
                continue
            n = max(1, subscription.decimation)
            if subscription.mode == MODE_MINMAX and n > 1:
                run = self._minmax(signal, first, last, n)
            elif subscription.mode == MODE_STRIDE and n > 1:
                run = self._stride(signal, first, last, n)
            else:
                run = signal_run(signal, first, last)
            if run is not None:
                out.append(run)
        return out

    @staticmethod
    def _minmax(signal: Signal, first: int, last: int, n: int) -> signals_pb2.SignalRun | None:
        # Bucket b covers samples [b*n, (b+1)*n). A bucket is sent in the window in which its
        # last sample falls, so each bucket is sent exactly once.
        bucket_first = _ceil_div(first + 1, n) - 1
        bucket_last = last // n
        if bucket_first >= bucket_last:
            return None
        data = signal.data[bucket_first * n : bucket_last * n]
        shaped = data.reshape(bucket_last - bucket_first, n, signal.info.components)
        packed = np.concatenate([shaped.min(axis=1), shaped.max(axis=1)], axis=1)
        run = signals_pb2.SignalRun(
            signal=signal.info.id,
            dt_ns=signal.dt_ns * n,
            sample_count=packed.shape[0],
            components=signal.info.components,
            encoding=signals_pb2.SAMPLE_ENCODING_F64_LE,
            samples=np.ascontiguousarray(packed, dtype="<f8").tobytes(),
            first_index=bucket_first,
            mode=MODE_MINMAX,
        )
        run.t0.ns = bucket_first * n * signal.dt_ns
        return run

    @staticmethod
    def _stride(signal: Signal, first: int, last: int, n: int) -> signals_pb2.SignalRun | None:
        start = _ceil_div(first, n)
        stop = _ceil_div(last, n)
        if start >= stop:
            return None
        data = signal.data[start * n : (stop - 1) * n + 1 : n]
        run = signals_pb2.SignalRun(
            signal=signal.info.id,
            dt_ns=signal.dt_ns * n,
            sample_count=data.shape[0],
            components=signal.info.components,
            encoding=signals_pb2.SAMPLE_ENCODING_F64_LE,
            samples=np.ascontiguousarray(data, dtype="<f8").tobytes(),
            first_index=start,
            mode=MODE_STRIDE,
        )
        run.t0.ns = start * n * signal.dt_ns
        return run

    # --- scene -----------------------------------------------------------------------------

    def _pose_at(self, signal: Signal, time_ns: int) -> np.ndarray:
        index = min(time_ns // signal.dt_ns, signal.sample_count - 1)
        return signal.data[index]

    def delta(self, time_ns: int) -> scene_pb2.SceneDelta:
        """Transforms of all bound prims at `time_ns`. The caller sets the sequence fields."""
        delta = scene_pb2.SceneDelta()
        delta.sim_time.ns = time_ns
        data = bytearray()
        for prim, signal in self._bindings:
            if signal.sample_count == 0:
                continue
            pose = self._pose_at(signal, time_ns)
            delta.transform_prims.append(prim)
            data += pose[:3].astype("<f8").tobytes()
            data += pose[3:7].astype("<f4").tobytes()
        delta.transform_data = bytes(data)
        return delta

    def snapshot(self, time_ns: int) -> scene_pb2.SceneSnapshot:
        """The recorded scene with transforms as of `time_ns`. The caller sets the sequence."""
        snapshot = scene_pb2.SceneSnapshot()
        snapshot.CopyFrom(self.recording.snapshot)
        snapshot.sim_time.ns = time_ns
        transforms = bytearray(snapshot.transforms.inline)
        for prim, signal in self._bindings:
            if signal.sample_count == 0:
                continue
            pose = self._pose_at(signal, time_ns)
            offset = prim * TRANSFORM_SNAPSHOT_BYTES
            transforms[offset : offset + 24] = pose[:3].astype("<f8").tobytes()
            transforms[offset + 24 : offset + 40] = pose[3:7].astype("<f4").tobytes()
        snapshot.transforms.inline = bytes(transforms)
        return snapshot
