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
from collections import deque
from pathlib import Path

import numpy as np
from lagrange.viewer.v1 import (
    command_pb2,
    envelope_pb2,
    profile_pb2,
    session_pb2,
    signals_pb2,
)

from . import __version__, environment, profile, sensors
from .endpoint import Endpoint
from .events import derive_events
from .framing import DEFAULT_MAX_FRAME_SIZE, FrameError, FrameReader, encode_frame
from .recording import Recording, RecordingError, read_recording
from .replay import MODE_FULL, ReplayEngine, Subscription, Window
from .urdf import UrdfError, import_urdf

PROTOCOL_MAJOR = 0
PROTOCOL_MINOR = 1
TICK_SECONDS = 0.02  # 50 Hz: scene deltas and 20 ms signal blocks
STATUS_EVERY_TICKS = 5  # 10 Hz
STATS_PERIOD_S = 1.0
LIDAR_PERIOD_NS = round(1e9 / sensors.LIDAR_RATE_HZ)
FIELD_PERIOD_NS = round(1e9 / environment.SLICE_RATE_HZ)
# A viewer that does not read gets its updates dropped beyond this much queued output.
MAX_QUEUED_BYTES = 32 * 1024 * 1024
MAX_LOG_RECORDS = 500
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
        self.bytes_out = 0  # since the last statistics message
        self.sensors: set[str] = set()
        self.fields: dict[str, float] = {}  # field name -> height of the slice wanted
        self.ros_graph = False
        self.world = False
        self.world_tiles_sent = False  # sensor names (prim paths) this viewer watches

    def congested(self) -> bool:
        return self.writer.transport.get_write_buffer_size() > MAX_QUEUED_BYTES

    def send(self, envelope: envelope_pb2.Envelope) -> None:
        frame = encode_frame(envelope.SerializeToString())
        self.bytes_out += len(frame)
        self.writer.write(frame)


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
        self._restart_required = False
        self._last_lidar_ns = 0
        self._events = derive_events(self.engine.recording)
        self._tick_ms: list[float] = []  # wall time of each tick since the last statistics
        self._stats_at = time.monotonic()
        self._stats_sim_ns = 0
        self._last_poses: dict[int, np.ndarray] = {}
        self._last_field_ns = 0
        self._reset_profile()
        # What the console shows. Kept so that a viewer that connects later sees it too.
        self._logs: deque[session_pb2.LogRecord] = deque(maxlen=MAX_LOG_RECORDS)
        self._log_stage()

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
        for record in self._logs:
            session.send(envelope_pb2.Envelope(log=record))
        self._send_run_facts(session)
        return True

    # --- what a run is and what happened in it ------------------------------------------------

    def _recording_info(self) -> session_pb2.RecordingInfo:
        recording = self.engine.recording
        info = session_pb2.RecordingInfo(
            path=recording.path,
            size_bytes=recording.size_bytes,
            compression="zstd" if recording.path else "",
            synthetic=recording.meta.synthetic,
            generator=recording.meta.generator,
            description=recording.meta.description,
        )
        info.duration.ns = recording.duration_ns
        for signal in recording.signals:
            info.channels.add(
                topic=signal.info.name,
                rate_hz=1e9 / signal.dt_ns if signal.dt_ns > 0 else 0.0,
                messages=signal.sample_count,
                unit=signal.info.unit.display,
            )
            info.messages += signal.sample_count
        return info

    def _environment(self) -> session_pb2.EnvironmentState:
        on = {name for name, d in self._profile_data["domains"].items() if d.get("enabled")}
        return environment.environment_state(self.engine.recording, on)

    def _send_run_facts(self, session: _Session) -> None:
        session.send(envelope_pb2.Envelope(recording_info=self._recording_info()))
        session.send(envelope_pb2.Envelope(environment=self._environment()))
        session.world_tiles_sent = False
        events = envelope_pb2.Envelope()
        events.event_list.events.extend(self._events)
        events.event_list.complete = True  # a replay knows its whole recording
        session.send(events)

    def _root_poses(self) -> dict[int, np.ndarray]:
        poses = {}
        for prim in self.engine.recording.snapshot.prims:
            pose = self.engine.pose_of(prim.id, self.engine.sim_time_ns)
            if pose is not None:
                poses[prim.id] = pose
        return poses

    def _send_slow_views(self, elapsed: float) -> None:
        """Once a second, to the viewers that asked: the ROS graph and the world."""
        recording = self.engine.recording
        time_ns = self.engine.sim_time_ns
        poses = self._root_poses()
        # Speed of each posed prim over the last period, from where it was then.
        speeds = {}
        for prim, pose in poses.items():
            before = self._last_poses.get(prim)
            if before is not None and elapsed > 0.0:
                speeds[prim] = float(np.hypot(pose[0] - before[0], pose[1] - before[1])) / elapsed
        self._last_poses = {prim: pose.copy() for prim, pose in poses.items()}
        graph = None
        for session in self._sessions:
            if session.ros_graph:
                if graph is None:
                    graph = envelope_pb2.Envelope(
                        ros_graph=environment.ros_graph(recording, time_ns, sensors.LIDAR_RATE_HZ)
                    )
                session.send(graph)
            if session.world:
                world = environment.world_state(
                    recording, poses, speeds, time_ns, with_tiles=not session.world_tiles_sent
                )
                session.world_tiles_sent = True
                session.send(envelope_pb2.Envelope(world=world))

    def _emit_fields(self) -> None:
        """Slices of the fields viewers watch, a few times a second of simulation time."""
        recording = self.engine.recording
        time_ns = self.engine.sim_time_ns
        pose = next(iter(self._root_poses().values()), None)
        for session in self._sessions:
            height = session.fields.get(environment.FIELD_FLUX)
            if height is None or session.congested():
                continue
            field = environment.flux_slice(recording, pose, time_ns, height)
            if field is not None:
                session.send(envelope_pb2.Envelope(field_slice=field))

    def _send_stats(self) -> None:
        """Once a second: what the server costs. Module costs are the profile's estimates."""
        now = time.monotonic()
        elapsed = max(now - self._stats_at, 1e-6)
        sim_ns = self.engine.sim_time_ns
        stats = session_pb2.ServerStats(estimated=True, viewers=len(self._sessions))
        stats.sim_time.ns = sim_ns
        advanced = sim_ns - self._stats_sim_ns
        stats.real_time_factor = max(advanced, 0) / 1e9 / elapsed
        if self._tick_ms:
            stats.tick_ms = sum(self._tick_ms) / len(self._tick_ms)
            stats.tick_ms_max = max(self._tick_ms)
        with contextlib.suppress(OSError, ValueError):
            # Resident pages of this process, from the kernel.
            pages = int(Path("/proc/self/statm").read_text(encoding="ascii").split()[1])
            stats.memory_bytes = pages * os.sysconf("SC_PAGE_SIZE")
        state = self._profile_state()
        for domain in state.domains:
            if not domain.enabled or domain.dt_ns <= 0:
                continue
            steps_per_sim_s = 1e9 / domain.dt_ns
            stats.modules.add(
                module=f"{domain.domain}/{domain.active_module}",
                domain=domain.domain,
                dt_ns=domain.dt_ns,
                steps_per_second=steps_per_sim_s * stats.real_time_factor,
                cpu_ms_per_step=domain.cpu_ms_per_sim_s / steps_per_sim_s,
                cpu_ms_per_step_max=1.6 * domain.cpu_ms_per_sim_s / steps_per_sim_s,
            )
        for session in self._sessions:
            stats.bytes_per_second_out = session.bytes_out / elapsed
            session.bytes_out = 0
            session.send(envelope_pb2.Envelope(server_stats=stats))
        self._tick_ms.clear()
        self._stats_at = now
        self._stats_sim_ns = sim_ns

    # --- console -----------------------------------------------------------------------------

    def emit_log(self, level: int, source: str, text: str) -> None:
        """Record a line for the console and send it to every connected viewer."""
        record = session_pb2.LogRecord(
            level=level, source=source, text=text, wall_unix_ns=time.time_ns()
        )
        record.sim_time.ns = self.engine.sim_time_ns
        self._logs.append(record)
        envelope = envelope_pb2.Envelope(log=record)
        for session in self._sessions:
            session.send(envelope)

    def _log_stage(self) -> None:
        recording = self.engine.recording
        stage = recording.meta.stage_path or recording.snapshot.stage_path or "untitled"
        info = session_pb2.LOG_LEVEL_INFO
        self.emit_log(info, "stage", f"loaded {stage} · {len(recording.snapshot.prims)} prims")
        self.emit_log(
            info,
            "replay",
            f"{len(recording.signals)} signals · {recording.duration_ns / 1e9:.3f} s"
            + (" · synthetic data" if recording.meta.synthetic else ""),
        )
        enabled = [name for name, d in self._profile_data["domains"].items() if d.get("enabled")]
        self.emit_log(info, "profile", "domains on: " + (", ".join(enabled) or "none"))
        if recording.meta.synthetic and any(
            prim.type_name == "Lidar" for prim in recording.snapshot.prims
        ):
            self.emit_log(
                session_pb2.LOG_LEVEL_WARN,
                "sensors",
                "lidar frames are synthetic: a fixed aisle scanned from the robot's pose",
            )
        if recording.meta.synthetic:
            self.emit_log(
                session_pb2.LOG_LEVEL_WARN,
                "replay",
                "this run was generated by a script, not by a simulation or a robot",
            )

    def _reset_profile(self) -> None:
        """Take the profile the recording was made with, or a default one for its stage."""
        meta = self.engine.recording.meta
        checked = profile.check(meta.profile_yaml) if meta.profile_yaml else None
        if checked is None or not checked.valid:
            stage = meta.stage_path or self.engine.recording.snapshot.stage_path
            checked = profile.check(profile.default_profile(stage))
        assert checked.data is not None
        self._profile_data = checked.data
        self._profile_text = profile.canonical(checked.data)
        self._restart_required = False

    def _profile_state(self) -> profile_pb2.ProfileState:
        return profile.state(
            self._profile_text, self._profile_data, restart_required=self._restart_required
        )

    def _apply_profile(self, request: command_pb2.ApplyProfile) -> bool:
        """Adopt a new profile. Returns True when the run restarted. Raises ValueError."""
        checked = profile.check(request.yaml)
        if not checked.valid or checked.data is None:
            first = next(issue for issue in checked.issues if issue.error)
            where = f"{first.pointer}: " if first.pointer else ""
            raise ValueError(f"profile not applied: {where}{first.message}")
        self._profile_data = checked.data
        self._profile_text = profile.canonical(checked.data)
        # A real server rebuilds its wiring here. The replay has nothing to rebuild, so a
        # restart is the whole effect: the run starts again at t = 0.
        self._restart_required = not request.restart
        if request.restart:
            self.engine.seek(0)
        return request.restart

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
        profile_changed = False
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
            elif kind == "subscribe_sensor":
                if command.subscribe_sensor.enabled:
                    session.sensors.add(command.subscribe_sensor.sensor)
                else:
                    session.sensors.discard(command.subscribe_sensor.sensor)
            elif kind == "subscribe_field":
                request = command.subscribe_field
                if request.enabled:
                    session.fields[request.field] = request.height_m
                else:
                    session.fields.pop(request.field, None)
            elif kind == "set_ros_graph":
                session.ros_graph = command.set_ros_graph.enabled
                if session.ros_graph:  # answer now; the once-a-second updates follow
                    session.send(
                        envelope_pb2.Envelope(
                            ros_graph=environment.ros_graph(
                                self.engine.recording,
                                self.engine.sim_time_ns,
                                sensors.LIDAR_RATE_HZ,
                            )
                        )
                    )
            elif kind == "set_world":
                session.world = command.set_world.enabled
                session.world_tiles_sent = False
                if session.world:
                    world = environment.world_state(
                        self.engine.recording,
                        self._root_poses(),
                        {},
                        self.engine.sim_time_ns,
                        with_tiles=True,
                    )
                    session.world_tiles_sent = True
                    session.send(envelope_pb2.Envelope(world=world))
            elif kind == "set_camera_interest":
                pass  # a hint for streamed worlds; the mock server streams nothing
            elif kind == "validate_profile":
                result.command_result.validation.CopyFrom(
                    profile.check(command.validate_profile.yaml).to_proto()
                )
            elif kind == "apply_profile":
                resync = self._apply_profile(command.apply_profile)
                profile_changed = True
                self.emit_log(
                    session_pb2.LOG_LEVEL_INFO,
                    "profile",
                    "applied · run restarted at t = 0" if resync else "applied · restart required",
                )
            elif kind in ("import_robot", "open_recording"):
                # Reading files can take a moment: answer when it is done, off the event loop.
                task = asyncio.create_task(self._load(session, command))
                self._loading.add(task)
                task.add_done_callback(self._loading.discard)
                return
            elif kind == "remove_robot":
                # One scene, replaced whole by import_robot: no robot is spawned beside another.
                result.command_result.ok = False
                result.command_result.error = (
                    "the mock server replays one scene: it has no spawned robot to remove"
                )
            elif kind == "command_joint":
                # A replay: the recorded motion plays whatever is commanded (RFC 0016 §7.3).
                result.command_result.ok = False
                result.command_result.error = (
                    "the mock server replays a recording: its joints take no commands"
                )
            elif kind == "get_attributes":
                items = result.command_result.attributes
                items.SetInParent()
                known = self.engine.recording.attributes
                for prim in command.get_attributes.prims:
                    items.items.add(prim=prim).attributes.extend(known.get(prim, []))
            else:
                result.command_result.ok = False
                result.command_result.error = f"'{kind}' is not supported by the mock server yet"
        except ValueError as error:
            result.command_result.ok = False
            result.command_result.error = str(error)
        session.send(result)
        if profile_changed:
            state = envelope_pb2.Envelope(profile_state=self._profile_state())
            surroundings = envelope_pb2.Envelope(environment=self._environment())
            for other in self._sessions:
                other.send(state)
                other.send(surroundings)
        if resync:
            # A snapshot marks the discontinuity for every viewer, including ones that only
            # watch signals: sample indices restart from the new time after it.
            for other in self._sessions:
                self._send_snapshot(other)
        if kind in ("play", "pause", "set_rtf", "seek", "reset", "step", "apply_profile"):
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
        self._reset_profile()
        self._log_stage()
        self._events = derive_events(recording)
        self._stats_sim_ns = 0
        for session in self._sessions:
            session.subscriptions.clear()  # signal ids belong to the old catalog
            session.send(envelope_pb2.Envelope(profile_state=self._profile_state()))
            catalog = envelope_pb2.Envelope()
            catalog.catalog.CopyFrom(recording.catalog(self.catalog_generation))
            session.send(catalog)
            self._send_snapshot(session)
            session.send(self._status())
            self._send_run_facts(session)

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
                self.emit_log(session_pb2.LOG_LEVEL_WARN, "import", note)
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
            started = time.perf_counter()
            if window is not None:
                self._emit(window)
                # Sensors run on simulation time, so their rate follows the replay speed and
                # does not depend on how often this loop happens to wake up.
                since = self.engine.sim_time_ns - self._last_lidar_ns
                if since >= LIDAR_PERIOD_NS or since < 0:
                    self._last_lidar_ns = self.engine.sim_time_ns
                    self._emit_sensors()
                since = self.engine.sim_time_ns - self._last_field_ns
                if since >= FIELD_PERIOD_NS or since < 0:
                    self._last_field_ns = self.engine.sim_time_ns
                    self._emit_fields()
            self._tick_count += 1
            self._tick_ms.append((time.perf_counter() - started) * 1e3)
            if time.monotonic() - self._stats_at >= STATS_PERIOD_S:
                self._send_slow_views(time.monotonic() - self._stats_at)
                self._send_stats()
            if self._tick_count % STATUS_EVERY_TICKS == 0:
                status = self._status()
                for session in self._sessions:
                    status.status.dropped_deltas = session.dropped_deltas
                    status.status.dropped_signal_blocks = session.dropped_blocks
                    session.send(status)

    def _emit_sensors(self) -> None:
        """Send a lidar frame to every viewer that watches a lidar of the sample scene."""
        recording = self.engine.recording
        if not recording.meta.synthetic or not any(s.sensors for s in self._sessions):
            return
        time_ns = self.engine.sim_time_ns
        for prim in recording.snapshot.prims:
            if prim.type_name != "Lidar":
                continue
            watchers = [s for s in self._sessions if prim.path in s.sensors and not s.congested()]
            pose = self.engine.pose_of(prim.id, time_ns)
            if not watchers or pose is None:
                continue
            qw, qz = float(pose[3]), float(pose[6])
            points = sensors.lidar_scan(
                float(pose[0]), float(pose[1]), 2.0 * float(np.arctan2(qz, qw)), time_ns
            )
            envelope = envelope_pb2.Envelope()
            frame = envelope.sensor_frame
            frame.sensor = prim.path
            frame.sim_time.ns = time_ns
            frame.points.count = len(points)
            frame.points.stride = sensors.POINT_STRIDE
            frame.points.fields.extend(sensors.POINT_FIELDS)
            data = points.astype("<f4").tobytes()
            frame.points.data.inline = data
            frame.points.data.size = len(data)
            for session in watchers:
                session.send(envelope)

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
