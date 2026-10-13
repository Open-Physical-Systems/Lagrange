"""asyncio server that speaks the viewer protocol from a ReplayEngine.

One engine (one timeline) is shared by all connected viewers. Each viewer has its own
subscriptions and its own scene sequence numbers.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import stat
import time
from pathlib import Path

from lagrange.viewer.v1 import (
    command_pb2,
    envelope_pb2,
    profile_pb2,
    session_pb2,
    signals_pb2,
)

from . import __version__
from .endpoint import Endpoint
from .framing import DEFAULT_MAX_FRAME_SIZE, FrameError, FrameReader, encode_frame
from .recording import Recording, RecordingError, read_recording
from .replay import MODE_FULL, ReplayEngine, Subscription, Window
from .urdf import UrdfError, import_urdf

PROTOCOL_MAJOR = 0
PROTOCOL_MINOR = 1
TICK_SECONDS = 0.02  # 50 Hz: scene deltas and 20 ms signal blocks
STATUS_EVERY_TICKS = 5  # 10 Hz
# A viewer that does not read gets its updates dropped beyond this much queued output.
MAX_QUEUED_BYTES = 32 * 1024 * 1024
CLIENT_MAX_FRAME_SIZE = 4 * 1024 * 1024  # commands are small; refuse anything larger

log = logging.getLogger("lagrange_mock_server")


class _Session:
    def __init__(self, writer: asyncio.StreamWriter) -> None:
        self.writer = writer
        self.subscriptions: dict[int, Subscription] = {}
        self.sequence = 0  # sequence of the last scene message sent
        self.has_snapshot = False
        self.needs_snapshot = False
        self.dropped_deltas = 0
        self.dropped_blocks = 0

    def congested(self) -> bool:
        return self.writer.transport.get_write_buffer_size() > MAX_QUEUED_BYTES

    def send(self, envelope: envelope_pb2.Envelope) -> None:
        self.writer.write(encode_frame(envelope.SerializeToString()))


class MockServer:
    def __init__(self, engine: ReplayEngine, endpoint: Endpoint) -> None:
        self.engine = engine
        self.endpoint = endpoint
        self.catalog_generation = 1
        self._sessions: set[_Session] = set()
        self._server: asyncio.AbstractServer | None = None
        self._ticker: asyncio.Task | None = None
        self._next_session_id = 1
        self._tick_count = 0
        self._loading: set[asyncio.Task] = set()

    # --- lifecycle -------------------------------------------------------------------------

    async def start(self) -> Endpoint:
        """Start listening. Returns the bound endpoint (with the real port for tcp port 0)."""
        if self.endpoint.kind == "unix":
            path = Path(self.endpoint.address)
            path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            if path.exists() or path.is_symlink():
                if not stat.S_ISSOCK(path.lstat().st_mode):
                    raise FileExistsError(f"{path} exists and is not a socket")
                if await self._is_live(path):
                    raise OSError(f"another server is listening on {path}")
                path.unlink()
            previous = os.umask(0o177)  # create the socket file with mode 0600
            try:
                self._server = await asyncio.start_unix_server(self._serve, path=str(path))
            finally:
                os.umask(previous)
            bound = self.endpoint
        else:
            self._server = await asyncio.start_server(
                self._serve, host=self.endpoint.address, port=self.endpoint.port
            )
            port = self._server.sockets[0].getsockname()[1]
            bound = Endpoint("tcp", self.endpoint.address, port)
        self.endpoint = bound
        self._ticker = asyncio.create_task(self._tick_loop())
        log.info("listening on %s", bound)
        return bound

    @staticmethod
    async def _is_live(path: Path) -> bool:
        try:
            _, writer = await asyncio.open_unix_connection(str(path))
        except OSError:
            return False
        writer.close()
        with contextlib.suppress(OSError):
            await writer.wait_closed()
        return True

    async def stop(self) -> None:
        if self._ticker is not None:
            self._ticker.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._ticker
        for session in list(self._sessions):
            session.writer.close()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        if self.endpoint.kind == "unix":
            with contextlib.suppress(OSError):
                Path(self.endpoint.address).unlink()

    # --- per-connection --------------------------------------------------------------------

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        session = _Session(writer)
        frames = FrameReader(CLIENT_MAX_FRAME_SIZE)
        greeted = False
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                frames.feed(data)
                for payload in frames.frames():
                    envelope = envelope_pb2.Envelope()
                    envelope.ParseFromString(payload)
                    if not greeted:
                        if not self._handshake(session, envelope):
                            await writer.drain()
                            return
                        greeted = True
                        self._sessions.add(session)
                    else:
                        self._dispatch(session, envelope)
                await writer.drain()
        except (FrameError, ConnectionError) as error:
            log.info("client dropped: %s", error)
        except Exception:  # a malformed message must not take the server down
            log.exception("client dropped after an invalid message")
        finally:
            self._sessions.discard(session)
            writer.close()

    def _handshake(self, session: _Session, envelope: envelope_pb2.Envelope) -> bool:
        reply = envelope_pb2.Envelope()
        hello = reply.server_hello
        hello.version.major = PROTOCOL_MAJOR
        hello.version.minor = PROTOCOL_MINOR
        hello.server_kind = session_pb2.SERVER_KIND_MOCK
        hello.server_name = f"lagrange_mock_server {__version__}"
        if envelope.WhichOneof("body") != "client_hello":
            hello.refusal = "the first message must be ClientHello"
            session.send(reply)
            return False
        client = envelope.client_hello
        if client.version.major != PROTOCOL_MAJOR:
            hello.refusal = (
                f"protocol version mismatch: the client speaks {client.version.major}."
                f"{client.version.minor}, this server speaks {PROTOCOL_MAJOR}.{PROTOCOL_MINOR}"
            )
            session.send(reply)
            return False
        hello.session_id = self._next_session_id
        self._next_session_id += 1
        hello.shm_enabled = False  # the mock server sends everything in-band for now
        capabilities = hello.capabilities
        capabilities.can_seek = True
        capabilities.can_step = True
        capabilities.can_import = True
        capabilities.tier = 0
        session.send(reply)

        session.send(envelope_pb2.Envelope(profile_state=self._profile_state()))
        catalog = envelope_pb2.Envelope()
        catalog.catalog.CopyFrom(self.engine.recording.catalog(self.catalog_generation))
        session.send(catalog)
        session.send(self._status())
        return True

    def _profile_state(self) -> profile_pb2.ProfileState:
        state = profile_pb2.ProfileState(
            yaml=self.engine.recording.meta.profile_yaml, schema_version=1
        )
        writers: dict[str, str] = {}
        for signal in self.engine.recording.signals:
            domain, _, module = signal.info.writer.partition("/")
            writers.setdefault(domain, module)
        for domain, module in sorted(writers.items()):
            state.domains.add(
                domain=domain,
                enabled=False,
                available=True,
                active_module="replay",
                substituted=True,
            ).substitutes.add(
                token="replay",
                kind="replay",
                description=f"recorded {module} data",
                needs_source=True,
            )
        return state

    def _status(self) -> envelope_pb2.Envelope:
        envelope = envelope_pb2.Envelope()
        status = envelope.status
        status.sim_time.ns = self.engine.sim_time_ns
        status.state = self.engine.state
        status.real_time_factor_target = self.engine.rtf_target
        running = self.engine.state == session_pb2.RUN_STATE_RUNNING
        status.real_time_factor = self.engine.rtf_target if running else 0.0
        status.replay_begin.ns = 0
        status.replay_end.ns = self.engine.duration_ns
        return envelope

    def _send_snapshot(self, session: _Session) -> None:
        envelope = envelope_pb2.Envelope()
        envelope.snapshot.CopyFrom(self.engine.snapshot(self.engine.sim_time_ns))
        session.sequence += 1
        envelope.snapshot.sequence = session.sequence
        session.send(envelope)
        session.has_snapshot = True
        session.needs_snapshot = False

    # --- commands --------------------------------------------------------------------------

    def _dispatch(self, session: _Session, envelope: envelope_pb2.Envelope) -> None:
        body = envelope.WhichOneof("body")
        if body == "ping":
            reply = envelope_pb2.Envelope()
            reply.pong.nonce = envelope.ping.nonce
            reply.pong.client_monotonic_ns = envelope.ping.client_monotonic_ns
            reply.pong.sim_time.ns = self.engine.sim_time_ns
            session.send(reply)
        elif body == "subscribe":
            self._subscribe(session, envelope.subscribe)
        elif body == "unsubscribe":
            for signal in envelope.unsubscribe.signals:
                session.subscriptions.pop(signal, None)
        elif body == "command":
            self._command(session, envelope.command)
        # Anything else from a client is ignored: newer clients may send messages this
        # server does not know.

    def _subscribe(self, session: _Session, request: signals_pb2.Subscribe) -> None:
        if request.catalog_generation != self.catalog_generation:
            return  # stale request; the client will re-subscribe after the new catalog
        count = len(self.engine.recording.signals)
        for item in request.signals:
            if item.signal < count:
                session.subscriptions[item.signal] = Subscription(
                    signal=item.signal, mode=item.mode or MODE_FULL, decimation=item.decimation or 1
                )

    def _command(self, session: _Session, command: command_pb2.Command) -> None:
        result = envelope_pb2.Envelope()
        result.command_result.request_id = command.request_id
        result.command_result.ok = True
        kind = command.WhichOneof("command")
        resync = False
        try:
            if kind == "play":
                self.engine.play()
            elif kind == "pause":
                self.engine.pause()
            elif kind == "set_rtf":
                self.engine.set_rtf(command.set_rtf.target)
            elif kind == "seek":
                self.engine.seek(command.seek.to.ns)
                resync = True
            elif kind == "reset":
                self.engine.pause()
                self.engine.seek(0)
                resync = True
            elif kind == "step":
                duration = command.step.duration_ns or command.step.ticks * 1_000_000
                self._emit(self.engine.step(duration))
            elif kind == "request_snapshot":
                self._send_snapshot(session)
            elif kind == "get_profile":
                session.send(envelope_pb2.Envelope(profile_state=self._profile_state()))
            elif kind in ("import_robot", "open_recording"):
                # Reading files can take a moment: answer when it is done, off the event loop.
                task = asyncio.create_task(self._load(session, command))
                self._loading.add(task)
                task.add_done_callback(self._loading.discard)
                return
            elif kind == "get_attributes":
                result.command_result.attributes.SetInParent()  # no attribute data yet
            else:
                result.command_result.ok = False
                result.command_result.error = f"'{kind}' is not supported by the mock server yet"
        except ValueError as error:
            result.command_result.ok = False
            result.command_result.error = str(error)
        session.send(result)
        if resync:
            # A snapshot marks the discontinuity for every viewer, including ones that only
            # watch signals: sample indices restart from the new time after it.
            for other in self._sessions:
                self._send_snapshot(other)
        if kind in ("play", "pause", "set_rtf", "seek", "reset", "step"):
            status = self._status()
            for other in self._sessions:
                other.send(status)

    # --- loading another scene -----------------------------------------------------------

    def replace_recording(self, recording: Recording) -> None:
        """Serve a different recording or scene. Every viewer gets the new catalog and scene."""
        was_running = self.engine.state == session_pb2.RUN_STATE_RUNNING
        self.engine = ReplayEngine(recording, loop=self.engine.loop)
        if was_running:
            self.engine.play()
        self.catalog_generation += 1
        for session in self._sessions:
            session.subscriptions.clear()  # signal ids belong to the old catalog
            session.send(envelope_pb2.Envelope(profile_state=self._profile_state()))
            catalog = envelope_pb2.Envelope()
            catalog.catalog.CopyFrom(recording.catalog(self.catalog_generation))
            session.send(catalog)
            self._send_snapshot(session)
            session.send(self._status())

    async def _load(self, session: _Session, command: command_pb2.Command) -> None:
        result = envelope_pb2.Envelope()
        result.command_result.request_id = command.request_id
        kind = command.WhichOneof("command")
        path = command.import_robot.path if kind == "import_robot" else command.open_recording.path
        try:
            if not path:
                raise UrdfError("no file path given (uploading file content is not supported yet)")
            if kind == "open_recording":
                recording = await asyncio.to_thread(read_recording, Path(path))
                notes: list[str] = []
            else:
                args = tuple(command.import_robot.xacro_args)
                recording, report = await asyncio.to_thread(import_urdf, path, args)
                notes = report.notes
            self.replace_recording(recording)
            result.command_result.ok = True
            result.command_result.imported_stage_path = path
            for note in notes:
                log.info("import: %s", note)
                record = envelope_pb2.Envelope()
                record.log.level = session_pb2.LOG_LEVEL_WARN
                record.log.source = "import"
                record.log.text = note
                session.send(record)
        except (UrdfError, RecordingError, OSError) as error:
            result.command_result.ok = False
            result.command_result.error = str(error)
        if not session.writer.is_closing():
            session.send(result)

    # --- time ------------------------------------------------------------------------------

    async def _tick_loop(self) -> None:
        last = time.monotonic_ns()
        while True:
            await asyncio.sleep(TICK_SECONDS)
            now = time.monotonic_ns()
            window = self.engine.advance(now - last)
            last = now
            if window is not None:
                self._emit(window)
            self._tick_count += 1
            if self._tick_count % STATUS_EVERY_TICKS == 0:
                status = self._status()
                for session in self._sessions:
                    status.status.dropped_deltas = session.dropped_deltas
                    status.status.dropped_signal_blocks = session.dropped_blocks
                    session.send(status)

    def _emit(self, window: Window) -> None:
        """Send one window of signals and the scene at its end to every session."""
        for session in list(self._sessions):
            if session.writer.is_closing():
                continue
            if session.congested():
                # Dropping a delta breaks the sequence on purpose: the next delta the viewer
                # sees has a base it does not know, so it asks for a snapshot.
                session.dropped_blocks += 1
                if session.has_snapshot:
                    session.dropped_deltas += 1
                    session.sequence += 1
                continue
            if session.subscriptions:
                runs = self.engine.runs(window, list(session.subscriptions.values()))
                if runs:
                    envelope = envelope_pb2.Envelope()
                    envelope.signal_block.catalog_generation = self.catalog_generation
                    envelope.signal_block.runs.extend(runs)
                    session.send(envelope)
            if window.wrapped:
                self._send_snapshot(session)  # the timeline restarted at 0
            elif session.has_snapshot:
                envelope = envelope_pb2.Envelope()
                envelope.delta.CopyFrom(self.engine.delta(max(window.t1 - 1, 0)))
                envelope.delta.base_sequence = session.sequence
                session.sequence += 1
                envelope.delta.sequence = session.sequence
                session.send(envelope)


__all__ = ["DEFAULT_MAX_FRAME_SIZE", "MockServer"]
