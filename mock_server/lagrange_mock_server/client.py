"""A small protocol client and the conformance check built on it.

The conformance check connects to any server that speaks the viewer protocol (the mock server
now, the simulation server later) and verifies the rules a viewer relies on:

  * the handshake order and version,
  * scene sequence numbers: every delta's base is the previous scene message,
  * signal runs: sizes match their headers, timestamps match their indices, and consecutive
    runs of a signal are contiguous (no gap, no overlap) between discontinuities,
  * every command gets exactly one result.
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass, field

from google.protobuf import json_format
from lagrange.viewer.v1 import envelope_pb2, signals_pb2

from .endpoint import Endpoint
from .framing import FrameReader, encode_frame

PROTOCOL_MAJOR = 0
PROTOCOL_MINOR = 1


class ProtocolClient:
    """Blocking-style helper over asyncio streams: send an Envelope, receive Envelopes."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self._reader = reader
        self._writer = writer
        self._frames = FrameReader()
        self._next_request = 1

    @classmethod
    async def connect(cls, endpoint: Endpoint) -> ProtocolClient:
        if endpoint.kind == "unix":
            reader, writer = await asyncio.open_unix_connection(endpoint.address)
        else:
            reader, writer = await asyncio.open_connection(endpoint.address, endpoint.port)
        return cls(reader, writer)

    async def send(self, envelope: envelope_pb2.Envelope) -> None:
        self._writer.write(encode_frame(envelope.SerializeToString()))
        await self._writer.drain()

    async def command(self, **fields) -> int:
        """Send a Command with one field set, e.g. command(play={}). Returns the request id."""
        envelope = envelope_pb2.Envelope()
        envelope.command.request_id = self._next_request
        self._next_request += 1
        for name, value in fields.items():
            sub = getattr(envelope.command, name)
            sub.SetInParent()
            json_format.ParseDict(value, sub)
        await self.send(envelope)
        return envelope.command.request_id

    async def receive(self, timeout: float = 5.0) -> envelope_pb2.Envelope | None:
        """Next Envelope, or None when the server closed the connection."""
        while True:
            payload = self._frames.next()
            if payload is not None:
                envelope = envelope_pb2.Envelope()
                envelope.ParseFromString(payload)
                return envelope
            data = await asyncio.wait_for(self._reader.read(65536), timeout)
            if not data:
                return None
            self._frames.feed(data)

    async def hello(self, major: int = PROTOCOL_MAJOR, minor: int = PROTOCOL_MINOR):
        envelope = envelope_pb2.Envelope()
        envelope.client_hello.version.major = major
        envelope.client_hello.version.minor = minor
        envelope.client_hello.client_name = "lagrange conformance client"
        await self.send(envelope)
        return await self.receive()

    async def close(self) -> None:
        self._writer.close()
        with contextlib.suppress(OSError):
            await self._writer.wait_closed()


@dataclass
class ConformanceReport:
    errors: list[str] = field(default_factory=list)
    server_name: str = ""
    signals_in_catalog: int = 0
    snapshots: int = 0
    deltas: int = 0
    signal_blocks: int = 0
    samples: dict[str, int] = field(default_factory=dict)  # per signal name
    final_sim_time_ns: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors


class _RunChecker:
    """Checks the runs of one signal for size, timing and contiguity."""

    def __init__(self, info: signals_pb2.SignalInfo, report: ConformanceReport) -> None:
        self.info = info
        self.report = report
        self.next_index: int | None = None
        self.dt_ns: int | None = None

    def reset(self) -> None:
        self.next_index = None

    def check(self, run: signals_pb2.SignalRun) -> None:
        name = self.info.name
        error = self.report.errors.append
        values_per_sample = run.components * (
            2 if run.mode == signals_pb2.SignalSubscription.MODE_MINMAX else 1
        )
        if run.encoding != signals_pb2.SAMPLE_ENCODING_F64_LE:
            error(f"{name}: unexpected sample encoding {run.encoding}")
            return
        if len(run.samples) != run.sample_count * values_per_sample * 8:
            error(f"{name}: {len(run.samples)} sample bytes for {run.sample_count} samples")
        if run.components != self.info.components:
            error(
                f"{name}: run has {run.components} components, catalog says {self.info.components}"
            )
        if run.dt_ns <= 0 or run.dt_ns % self.info.native_dt_ns != 0:
            error(f"{name}: run step {run.dt_ns} ns is not a multiple of the native step")
            return
        if run.t0.ns != run.first_index * run.dt_ns:
            error(f"{name}: t0 {run.t0.ns} does not match first_index {run.first_index}")
        if self.dt_ns is not None and run.dt_ns != self.dt_ns:
            error(f"{name}: step changed from {self.dt_ns} to {run.dt_ns} ns mid-stream")
        self.dt_ns = run.dt_ns
        if self.next_index is not None and run.first_index != self.next_index:
            kind = "gap" if run.first_index > self.next_index else "overlap"
            error(f"{name}: {kind}: expected index {self.next_index}, got {run.first_index}")
        self.next_index = run.first_index + run.sample_count
        self.report.samples[name] = self.report.samples.get(name, 0) + run.sample_count


async def run_conformance(
    endpoint: Endpoint,
    sim_seconds: float = 0.5,
    subscribe: dict[str, tuple[int, int]] | None = None,
    timeout: float = 20.0,
) -> ConformanceReport:
    """Connect, play for `sim_seconds` of simulation time, and check everything received.

    `subscribe` maps signal names to (mode, decimation). By default every signal in the
    catalog is subscribed in full.
    """
    report = ConformanceReport()
    error = report.errors.append
    client = await ProtocolClient.connect(endpoint)
    try:
        hello = await client.hello()
        if hello is None or hello.WhichOneof("body") != "server_hello":
            error("first message from the server is not ServerHello")
            return report
        if hello.server_hello.refusal:
            error(f"server refused the connection: {hello.server_hello.refusal}")
            return report
        if hello.server_hello.version.major != PROTOCOL_MAJOR:
            error(f"server speaks protocol major {hello.server_hello.version.major}")
        report.server_name = hello.server_hello.server_name

        catalog = None
        while catalog is None:
            envelope = await client.receive()
            if envelope is None:
                error("connection closed before the signal catalog arrived")
                return report
            if envelope.WhichOneof("body") == "catalog":
                catalog = envelope.catalog
        report.signals_in_catalog = len(catalog.signals)
        names = [info.name for info in catalog.signals]
        if len(set(names)) != len(names):
            error("duplicate signal names in the catalog")
        for index, info in enumerate(catalog.signals):
            if info.id != index:
                error(f"catalog entry {index} has id {info.id}")
            if info.native_dt_ns <= 0:
                error(f"{info.name}: no native step in the catalog")

        wanted = (
            subscribe
            if subscribe is not None
            else {name: (signals_pb2.SignalSubscription.MODE_FULL, 1) for name in names}
        )
        by_name = {info.name: info for info in catalog.signals}
        request = envelope_pb2.Envelope()
        request.subscribe.catalog_generation = catalog.generation
        checkers: dict[int, _RunChecker] = {}
        for name, (mode, decimation) in wanted.items():
            info = by_name[name]
            request.subscribe.signals.add(signal=info.id, mode=mode, decimation=decimation)
            checkers[info.id] = _RunChecker(info, report)
        await client.send(request)

        pending = {
            await client.command(request_snapshot={}),
            await client.command(seek={"to": {"ns": 0}}),
            await client.command(set_rtf={"target": 1.0}),
            await client.command(play={}),
        }
        target_ns = round(sim_seconds * 1e9)
        sequence: int | None = None
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        done = False
        while not done:
            if loop.time() > deadline:
                error(f"timed out; simulation time reached {report.final_sim_time_ns} ns")
                break
            envelope = await client.receive()
            if envelope is None:
                error("server closed the connection")
                break
            body = envelope.WhichOneof("body")
            if body == "snapshot":
                report.snapshots += 1
                sequence = envelope.snapshot.sequence
                for checker in checkers.values():
                    checker.reset()  # a snapshot marks a discontinuity in time
            elif body == "delta":
                report.deltas += 1
                delta = envelope.delta
                if sequence is None:
                    error("delta received before any snapshot")
                elif delta.base_sequence != sequence:
                    error(f"delta base {delta.base_sequence} does not follow sequence {sequence}")
                if delta.sequence != delta.base_sequence + 1:
                    error(f"delta sequence {delta.sequence} is not base + 1")
                if len(delta.transform_data) != 40 * len(delta.transform_prims):
                    error("delta transform bytes do not match the prim count")
                sequence = delta.sequence
            elif body == "signal_block":
                report.signal_blocks += 1
                if envelope.signal_block.catalog_generation != catalog.generation:
                    error("signal block for a different catalog generation")
                for run in envelope.signal_block.runs:
                    if run.signal not in checkers:
                        error(f"run for signal {run.signal}, which was not subscribed")
                    else:
                        checkers[run.signal].check(run)
            elif body == "command_result":
                result = envelope.command_result
                if result.request_id not in pending:
                    error(f"result for unknown or repeated request {result.request_id}")
                pending.discard(result.request_id)
                if not result.ok:
                    error(f"command {result.request_id} failed: {result.error}")
            elif body == "status":
                report.final_sim_time_ns = envelope.status.sim_time.ns
                # Status messages sent before our seek and play took effect do not count.
                done = not pending and envelope.status.sim_time.ns >= target_ns
        if pending:
            error(f"no result for requests {sorted(pending)}")
        pause = await client.command(pause={})
        while True:
            envelope = await client.receive()
            if envelope is None:
                error("server closed the connection before answering pause")
                break
            if (
                envelope.WhichOneof("body") == "command_result"
                and envelope.command_result.request_id == pause
            ):
                break
    except (TimeoutError, OSError) as failure:
        error(f"connection failed: {failure!r}")
    finally:
        await client.close()
    return report
