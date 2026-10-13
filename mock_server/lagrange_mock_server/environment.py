"""Environment, ROS graph and world state for a replayed run.

A simulation server takes these from its environment layer, its ROS 2 bridge and its tile
streamer. The mock server has none of them, so it describes what it can derive from the
recording and marks the rest as synthetic. Nothing here pretends to be a measurement.
"""

from __future__ import annotations

import numpy as np
from lagrange.viewer.v1 import common_pb2, session_pb2

from .recording import Recording, Signal

NO_PRIM = 0xFFFFFFFF
KIND_MAGNETIC_FLUX_DENSITY = 18  # lagrange::QuantityKind

# --- environment -----------------------------------------------------------------------------

FIELD_FLUX = "emi/flux_density"
FIELD_WIND = "weather/wind"
FIELD_FOG = "weather/fog_density"
SLICE_WIDTH = 64  # cells along +X
SLICE_HEIGHT = 48
SLICE_CELL_M = 0.25
SLICE_RATE_HZ = 2.0
# Flux density of the drive currents, as a line-current estimate B = mu0 I / (2 pi r),
# with the distance kept above the size of the robot so that the field stays finite on it.
MU0_OVER_2PI = 2.0e-7  # T·m/A
ROBOT_RADIUS_M = 0.3
PWM_FREQUENCY_HZ = 20_000.0  # of the sample robot's motor drives

MICROTESLA = common_pb2.Unit(
    display="µT", scale=1e-6, kind=KIND_MAGNETIC_FLUX_DENSITY, mass=1, time=-2, current=-1
)


def _source_current(recording: Recording) -> Signal | None:
    """The current of the first electrical source: what the robot's own field follows."""
    by_name = {signal.info.name: signal for signal in recording.signals}
    for net in recording.snapshot.nets:
        for terminal in net.terminals:
            if terminal.port.endswith("_out") and terminal.current_signal in by_name:
                return by_name[terminal.current_signal]
    return None


def environment_state(recording: Recording, domains_on: set[str]) -> session_pb2.EnvironmentState:
    """What the environment is for this run. Values the recording does not hold are neutral."""
    state = session_pb2.EnvironmentState(
        synthetic=True,
        local_time_hours=12.0,
        sun_elevation_deg=0.0,  # indoors
        ambient_temperature_k=293.15,
    )
    emi_on = "emi" in domains_on
    has_current = _source_current(recording) is not None
    if has_current and recording.snapshot.prims:
        state.emi_sources.add(
            name=f"{recording.snapshot.prims[0].name} drive inverters",
            prim=0,
            frequency_hz=PWM_FREQUENCY_HZ,
            detail="robot self-emission, estimated from the battery current",
        )
    flux = state.fields.add(
        name=FIELD_FLUX,
        label="|B|",
        domain="emi",
        unit=MICROTESLA,
        components=1,
        available=has_current,
    )
    if not has_current:
        flux.unavailable_reason = "this run has no electrical source to estimate it from"
    elif not emi_on:
        # Shown all the same: it is an estimate from the recording, not the EMI module.
        flux.label = "|B| (estimate)"
    wind = state.fields.add(name=FIELD_WIND, label="wind", domain="weather", components=2)
    wind.unit.display = "m/s"
    wind.unavailable_reason = "indoor stage: no wind field"
    fog = state.fields.add(name=FIELD_FOG, label="fog density", domain="weather", components=1)
    fog.unit.display = "1/m"
    fog.unavailable_reason = "no fog in this run"
    return state


def flux_slice(
    recording: Recording, pose: np.ndarray | None, time_ns: int, height_m: float
) -> session_pb2.FieldSlice | None:
    """|B| around the robot at `time_ns`, on a grid centred on the region's origin."""
    source = _source_current(recording)
    if source is None or pose is None or source.sample_count == 0:
        return None
    current = abs(float(source.data[min(time_ns // source.dt_ns, source.sample_count - 1), 0]))
    xs = (np.arange(SLICE_WIDTH) - (SLICE_WIDTH - 1) / 2.0) * SLICE_CELL_M
    ys = (np.arange(SLICE_HEIGHT) - (SLICE_HEIGHT - 1) / 2.0) * SLICE_CELL_M
    dx = xs[None, :] - float(pose[0])
    dy = ys[:, None] - float(pose[1])
    dz = height_m - float(pose[2])
    distance = np.sqrt(dx * dx + dy * dy + dz * dz + ROBOT_RADIUS_M**2)
    microtesla = (MU0_OVER_2PI * current / distance * 1e6).astype("<f4")
    result = session_pb2.FieldSlice(
        field=FIELD_FLUX,
        width=SLICE_WIDTH,
        height=SLICE_HEIGHT,
        components=1,
        origin_x=float(xs[0]),
        origin_y=float(ys[0]),
        cell_m=SLICE_CELL_M,
        height_m=height_m,
        value_min=float(microtesla.min()),
        value_max=float(microtesla.max()),
    )
    result.sim_time.ns = time_ns
    data = microtesla.tobytes()
    result.values.inline = data
    result.values.size = len(data)
    return result


# --- ROS 2 -----------------------------------------------------------------------------------


def _rate(signal: Signal) -> float:
    return 1e9 / signal.dt_ns if signal.dt_ns > 0 else 0.0


def ros_graph(recording: Recording, time_ns: int, lidar_rate_hz: float) -> session_pb2.RosGraph:
    """The graph a ROS 2 bridge would publish for this run. No bridge is running."""
    graph = session_pb2.RosGraph(
        rmw="rmw_fastrtps_cpp",
        distro="lyrical",
        domain_id=0,
        use_sim_time=True,
        note="described by the mock server: no ROS 2 bridge is running",
    )
    bridge = graph.node_info.add(name="/lagrange_ros2_bridge", detail="simulator bridge")
    signals = {signal.info.name: signal for signal in recording.signals}

    def topic(name: str, kind: str, rate_hz: float, bytes_per_message: int) -> None:
        graph.topics.add(
            name=name,
            type=kind,
            publishers=1,
            rate_hz=rate_hz,
            from_bridge=True,
            bandwidth_bps=rate_hz * bytes_per_message * 8.0,
        )
        bridge.publishes.append(name)

    poses = [s for s in recording.signals if s.info.name.endswith("/pose")]
    joints = sorted(
        name.split("/")[1] for name in signals if name.startswith("joint/") and "/velocity" in name
    )
    mechanical = max((_rate(s) for s in poses), default=0.0)
    topic("/clock", "rosgraph_msgs/msg/Clock", mechanical or 100.0, 16)
    if poses:
        topic("/tf", "tf2_msgs/msg/TFMessage", mechanical, 96 * len(poses))
    if joints:
        topic("/joint_states", "sensor_msgs/msg/JointState", mechanical, 64 + 48 * len(joints))
    for prim in recording.snapshot.prims:
        if prim.type_name == "Lidar":
            topic(f"/{prim.name}/points", "sensor_msgs/msg/PointCloud2", lidar_rate_hz, 46_000)
        elif prim.type_name == "Imu":
            imu = [s for s in recording.signals if s.info.prim == prim.id]
            if imu:
                topic(f"/{prim.name}/data", "sensor_msgs/msg/Imu", _rate(imu[0]), 320)
    graph.nodes.extend(node.name for node in graph.node_info)

    if joints:
        graph.topics.add(
            name="/cmd_vel",
            type="geometry_msgs/msg/Twist",
            subscribers=1,
            from_bridge=False,
            status="no publisher",
        )
        manager = graph.node_info.add(
            name="/controller_manager", detail="diff_drive_controller, joint_state_broadcaster"
        )
        manager.subscribes.append("/cmd_vel")
        state_publisher = graph.node_info.add(name="/robot_state_publisher", detail="URDF to /tf")
        state_publisher.subscribes.append("/joint_states")
        graph.nodes.extend(["/controller_manager", "/robot_state_publisher"])

        control = graph.control
        control.hardware = "LagrangeSystem"
        control.plugin = "lagrange_ros2_control/LagrangeSystem"
        control.rate_hz = mechanical
        control.active = True
        for joint in joints:
            torque = signals.get(f"joint/{joint}/torque")
            velocity = signals[f"joint/{joint}/velocity"]

            def at(signal: Signal | None) -> float:
                if signal is None or signal.sample_count == 0:
                    return 0.0
                return float(signal.data[min(time_ns // signal.dt_ns, signal.sample_count - 1), 0])

            control.joints.add(
                name=joint,
                interface="effort",
                command=at(torque),
                state=at(velocity),
                unit="N·m → rad/s",
            )
        control.controllers.add(name="diff_drive_controller", rate_hz=100.0, state="active")
        control.controllers.add(name="joint_state_broadcaster", rate_hz=mechanical, state="active")
    return graph


# --- world -----------------------------------------------------------------------------------

TILE_SIZE_M = 256.0
ACTIVE_RADIUS_M = 150.0
# Level of detail by distance from the focus agent, in tiles (Chebyshev). Beyond: not loaded.
LOD_RINGS = (0, 1, 3, 6)
RECENTER_THRESHOLD_M = 1000.0


def world_state(
    recording: Recording,
    poses: dict[int, np.ndarray],
    speeds: dict[int, float],
    time_ns: int,
    *,
    with_tiles: bool,
) -> session_pb2.WorldState:
    """A small synthetic world around the run's robots: what a streamer would hold loaded."""
    world = session_pb2.WorldState(synthetic=True, tile_size_m=TILE_SIZE_M, tiles_generation=1)
    world.sim_time.ns = time_ns
    world.geo_origin.valid = False  # the sample stage is not georeferenced
    prims = recording.snapshot.prims
    roots = [
        prim
        for prim in prims
        if prim.parent == NO_PRIM and any(child.parent == prim.id for child in prims)
    ]
    focus_tile = (0, 0)
    for index, prim in enumerate(roots):
        pose = poses.get(prim.id)
        x, y = (float(pose[0]), float(pose[1])) if pose is not None else (0.0, 0.0)
        focus = index == 0
        world.agents.add(
            name=prim.name,
            prim=prim.id,
            kind="robot",
            x=x,
            y=y,
            speed_m_per_s=speeds.get(prim.id, 0.0),
            status="focus" if focus else "active",
            active_radius_m=ACTIVE_RADIUS_M,
            focus=focus,
        )
        if focus:
            focus_tile = (int(np.floor(x / TILE_SIZE_M)), int(np.floor(y / TILE_SIZE_M)))
            world.floating_origin.anchor = prim.name
    world.floating_origin.recenter_threshold_m = RECENTER_THRESHOLD_M
    extent = LOD_RINGS[-1]
    if with_tiles:
        lods = bytearray()
        for ty in range(-extent, extent + 1):
            for tx in range(-extent, extent + 1):
                ring = max(abs(tx), abs(ty))
                lod = next(level for level, limit in enumerate(LOD_RINGS) if ring <= limit)
                world.tile_x.append(focus_tile[0] + tx)
                world.tile_y.append(focus_tile[1] + ty)
                lods.append(lod)
        world.tile_lod = bytes(lods)
    half = (extent + 0.5) * TILE_SIZE_M
    centre_x = (focus_tile[0] + 0.5) * TILE_SIZE_M
    centre_y = (focus_tile[1] + 0.5) * TILE_SIZE_M
    world.regions.add(
        name="R1",
        min_x=centre_x - half,
        min_y=centre_y - half,
        max_x=centre_x + half,
        max_y=centre_y + half,
    )
    return world
