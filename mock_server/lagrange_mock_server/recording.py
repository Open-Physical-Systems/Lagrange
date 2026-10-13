"""In-memory recordings and their MCAP file form.

File layout (see proto/lagrange/record/v1/record.proto):

    /lagrange/meta            RunMeta          one message at time 0
    /lagrange/catalog         SignalCatalog    one message at time 0
    /lagrange/scene/snapshot  SceneSnapshot    one message at time 0
    /lagrange/signals/<dt>ns  SignalBlock      one channel per step size, in time order

v0.1 limitation: a recording is loaded into memory as a whole, and every signal is uniformly
sampled from t = 0. Event-driven signals and streaming reads come later.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from google.protobuf import descriptor_pb2
from lagrange.record.v1 import record_pb2
from lagrange.viewer.v1 import scene_pb2, signals_pb2
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

FORMAT_VERSION = 1
TOPIC_META = "/lagrange/meta"
TOPIC_CATALOG = "/lagrange/catalog"
TOPIC_SNAPSHOT = "/lagrange/scene/snapshot"
TOPIC_SIGNALS_PREFIX = "/lagrange/signals/"
DEFAULT_BLOCK_NS = 100_000_000  # 100 ms of samples per SignalBlock message


class RecordingError(Exception):
    """The file is not a usable Lagrange recording."""


@dataclass
class Signal:
    """One uniformly sampled signal: sample k is at time k * dt_ns."""

    info: signals_pb2.SignalInfo
    data: np.ndarray  # float64, shape (sample_count, components)

    @property
    def dt_ns(self) -> int:
        return self.info.native_dt_ns

    @property
    def sample_count(self) -> int:
        return self.data.shape[0]


@dataclass
class Recording:
    meta: record_pb2.RunMeta
    snapshot: scene_pb2.SceneSnapshot
    signals: list[Signal] = field(default_factory=list)  # list index == SignalInfo.id

    @property
    def duration_ns(self) -> int:
        return self.meta.duration.ns

    def catalog(self, generation: int = 1) -> signals_pb2.SignalCatalog:
        return signals_pb2.SignalCatalog(
            generation=generation, signals=[s.info for s in self.signals]
        )

    def signal(self, name: str) -> Signal:
        for candidate in self.signals:
            if candidate.info.name == name:
                return candidate
        raise KeyError(name)

    def validate(self) -> None:
        """Raise RecordingError if the recording is internally inconsistent."""
        for index, signal in enumerate(self.signals):
            info = signal.info
            if info.id != index:
                raise RecordingError(f"signal {info.name!r} has id {info.id}, expected {index}")
            if info.native_dt_ns <= 0:
                raise RecordingError(f"signal {info.name!r} has no step size")
            if signal.data.ndim != 2 or signal.data.shape[1] != info.components:
                raise RecordingError(f"signal {info.name!r} has the wrong shape")
            if signal.data.dtype != np.float64:
                raise RecordingError(f"signal {info.name!r} is not float64")
        names = [s.info.name for s in self.signals]
        if len(set(names)) != len(names):
            raise RecordingError("duplicate signal names")
        known = set(names)
        for binding in self.meta.pose_bindings:
            if binding.signal not in known:
                raise RecordingError(f"pose binding refers to unknown signal {binding.signal!r}")


def _serialize(message) -> bytes:
    return message.SerializeToString(deterministic=True)


def _descriptor_set(message_class) -> bytes:
    """A FileDescriptorSet with the message's file and everything it imports."""
    files: dict[str, descriptor_pb2.FileDescriptorProto] = {}

    def add(file_descriptor) -> None:
        if file_descriptor.name in files:
            return
        for dependency in file_descriptor.dependencies:
            add(dependency)
        proto = descriptor_pb2.FileDescriptorProto()
        file_descriptor.CopyToProto(proto)
        files[file_descriptor.name] = proto

    add(message_class.DESCRIPTOR.file)
    return _serialize(descriptor_pb2.FileDescriptorSet(file=list(files.values())))


def signal_run(signal: Signal, first: int, last: int) -> signals_pb2.SignalRun:
    """A MODE_FULL run with samples [first, last) of `signal`."""
    chunk = np.ascontiguousarray(signal.data[first:last], dtype="<f8")
    run = signals_pb2.SignalRun(
        signal=signal.info.id,
        dt_ns=signal.dt_ns,
        sample_count=chunk.shape[0],
        components=signal.info.components,
        encoding=signals_pb2.SAMPLE_ENCODING_F64_LE,
        samples=chunk.tobytes(),
        first_index=first,
        mode=signals_pb2.SignalSubscription.MODE_FULL,
    )
    run.t0.ns = first * signal.dt_ns
    return run


def write_recording(recording: Recording, path: Path, block_ns: int = DEFAULT_BLOCK_NS) -> None:
    """Write `recording` to an MCAP file. The output bytes depend only on the recording."""
    recording.validate()
    with Path(path).open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.ZSTD)
        writer.start(profile="", library="lagrange_mock_server")

        def channel(topic: str, message_class) -> int:
            schema_id = writer.register_schema(
                name=message_class.DESCRIPTOR.full_name,
                encoding="protobuf",
                data=_descriptor_set(message_class),
            )
            return writer.register_channel(
                topic=topic, message_encoding="protobuf", schema_id=schema_id
            )

        def put(channel_id: int, time_ns: int, message) -> None:
            writer.add_message(
                channel_id=channel_id,
                log_time=time_ns,
                publish_time=time_ns,
                data=_serialize(message),
            )

        put(channel(TOPIC_META, record_pb2.RunMeta), 0, recording.meta)
        put(channel(TOPIC_CATALOG, signals_pb2.SignalCatalog), 0, recording.catalog())
        put(channel(TOPIC_SNAPSHOT, scene_pb2.SceneSnapshot), 0, recording.snapshot)

        groups: dict[int, list[Signal]] = {}
        for signal in recording.signals:
            groups.setdefault(signal.dt_ns, []).append(signal)
        channels = {
            dt: channel(f"{TOPIC_SIGNALS_PREFIX}{dt}ns", signals_pb2.SignalBlock)
            for dt in sorted(groups)
        }
        # Interleave the rate groups in time order so that a streaming reader sees them together.
        blocks: list[tuple[int, int, signals_pb2.SignalBlock]] = []
        for dt, members in groups.items():
            per_block = max(1, block_ns // dt)
            longest = max(s.sample_count for s in members)
            for first in range(0, longest, per_block):
                block = signals_pb2.SignalBlock(catalog_generation=1)
                for signal in members:
                    last = min(first + per_block, signal.sample_count)
                    if first < last:
                        block.runs.append(signal_run(signal, first, last))
                blocks.append((first * dt, dt, block))
        for time_ns, dt, block in sorted(blocks, key=lambda item: (item[0], item[1])):
            put(channels[dt], time_ns, block)
        writer.finish()


def read_recording(path: Path) -> Recording:
    """Load a recording written by write_recording()."""
    meta: record_pb2.RunMeta | None = None
    catalog: signals_pb2.SignalCatalog | None = None
    snapshot: scene_pb2.SceneSnapshot | None = None
    chunks: dict[int, list[np.ndarray]] = {}
    next_index: dict[int, int] = {}

    with Path(path).open("rb") as stream:
        try:
            reader = make_reader(stream)
            messages = list(reader.iter_messages())
        except Exception as error:  # the mcap library raises several unrelated types
            raise RecordingError(f"{path}: not a readable MCAP file: {error}") from error

    for _schema, channel, message in messages:
        if channel.topic == TOPIC_META:
            meta = record_pb2.RunMeta.FromString(message.data)
        elif channel.topic == TOPIC_CATALOG:
            catalog = signals_pb2.SignalCatalog.FromString(message.data)
        elif channel.topic == TOPIC_SNAPSHOT:
            snapshot = scene_pb2.SceneSnapshot.FromString(message.data)
        elif channel.topic.startswith(TOPIC_SIGNALS_PREFIX):
            block = signals_pb2.SignalBlock.FromString(message.data)
            for run in block.runs:
                if run.encoding != signals_pb2.SAMPLE_ENCODING_F64_LE:
                    raise RecordingError(f"signal {run.signal}: unsupported sample encoding")
                expected = next_index.get(run.signal, 0)
                if run.first_index != expected:
                    raise RecordingError(
                        f"signal {run.signal}: samples start at {run.first_index}, "
                        f"expected {expected} (gap or overlap in the file)"
                    )
                if len(run.samples) != run.sample_count * run.components * 8:
                    raise RecordingError(f"signal {run.signal}: sample bytes do not match count")
                data = np.frombuffer(run.samples, dtype="<f8").reshape(
                    run.sample_count, run.components
                )
                chunks.setdefault(run.signal, []).append(data)
                next_index[run.signal] = expected + run.sample_count

    if meta is None or catalog is None or snapshot is None:
        raise RecordingError(f"{path}: missing meta, catalog or snapshot; not a Lagrange recording")
    if meta.format_version != FORMAT_VERSION:
        raise RecordingError(
            f"{path}: format version {meta.format_version}, this reader supports {FORMAT_VERSION}"
        )

    signals = []
    for info in catalog.signals:
        parts = chunks.get(info.id, [])
        data = (
            np.concatenate(parts).astype(np.float64)
            if parts
            else np.zeros((0, info.components), dtype=np.float64)
        )
        signals.append(Signal(info=info, data=data))
    recording = Recording(meta=meta, snapshot=snapshot, signals=signals)
    recording.validate()
    return recording
