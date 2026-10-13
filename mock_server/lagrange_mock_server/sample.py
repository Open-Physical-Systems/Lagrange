"""A synthetic sample recording: a differential-drive AMR driving a figure-eight.

Nothing here is simulated or measured. The signals are computed from closed-form motion and
simple first-order models so that the viewer has plausible data at the right rates (10 kHz
electrical, 1 kHz mechanical, 10 Hz thermal). The recording is marked `synthetic`.

All parameters match the illustrative sample robot in docs/design/formats.md section 4.
"""

from __future__ import annotations

import numpy as np
from lagrange.record.v1 import record_pb2
from lagrange.viewer.v1 import common_pb2, scene_pb2, signals_pb2

from . import units
from .recording import FORMAT_VERSION, Recording, Signal

# Step sizes, ns.
DT_ELECTRICAL = 100_000  # 10 kHz
DT_MECHANICAL = 1_000_000  # 1 kHz
DT_THERMAL = 100_000_000  # 10 Hz

# Robot (illustrative values from the sample AMR).
WHEEL_RADIUS = 0.08  # m
WHEEL_SEPARATION = 0.50  # m
IMU_ACCEL_NOISE = 0.05  # m/s², white, per sample: a typical MEMS IMU at 1 kHz
IMU_GYRO_NOISE = 0.004  # rad/s
ROBOT_MASS = 191.1  # kg: the 41.1 kg robot carrying a 150 kg load
YAW_INERTIA = 12.0  # kg·m², rough box estimate with the load
GEAR_RATIO = 20.0
GEAR_EFFICIENCY = 0.92
TORQUE_CONSTANT = 0.085  # N·m/A
WINDING_RESISTANCE = 0.12  # ohm
ROLLING_RESISTANCE = 0.02  # 1
GRAVITY = 9.80665  # m/s², standard gravity
MOTOR_HEAT_CAPACITY = 450.0  # J/K
MOTOR_CONDUCTANCE = 1.5  # W/K to ambient
AMBIENT = 293.15  # K
BATTERY_OPEN_CIRCUIT = 52.5  # V
BATTERY_RESISTANCE = 0.05  # ohm, pack + harness
BATTERY_CAPACITY = 144_000.0  # C (40 Ah)
COMPUTER_POWER = 45.0  # W
CURRENT_LOOP_BANDWIDTH = 2.0 * np.pi * 1000.0  # rad/s

# Path: a lemniscate that closes after one period, so a looping replay has no jump.
PATH_AMPLITUDE = 2.0  # m
PATH_PERIOD = 10.0  # s

NO_PARENT = 0xFFFFFFFF


def _yaw_quaternion(yaw: np.ndarray) -> np.ndarray:
    """Quaternions (w, x, y, z) for rotations about +Z."""
    half = 0.5 * yaw
    zeros = np.zeros_like(yaw)
    return np.stack([np.cos(half), zeros, zeros, np.sin(half)], axis=1)


def _multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two arrays of quaternions (w, x, y, z)."""
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def _pose(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(np.hstack([position, quaternion]), dtype=np.float64)


def _box(sx: float, sy: float, sz: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangle mesh of an axis-aligned box centred at the origin."""
    positions, normals, indices = [], [], []
    half = np.array([sx, sy, sz]) / 2.0
    for axis in range(3):
        u, v = (axis + 1) % 3, (axis + 2) % 3
        for sign in (-1.0, 1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            base = len(positions)
            for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                corner = np.zeros(3)
                corner[axis] = sign * half[axis]
                corner[u] = du * half[u]
                corner[v] = dv * half[v]
                positions.append(corner)
                normals.append(normal)
            quad = (0, 1, 2, 0, 2, 3) if sign > 0 else (0, 2, 1, 0, 3, 2)
            indices.extend(base + i for i in quad)
    return (
        np.array(positions, dtype="<f4"),
        np.array(normals, dtype="<f4"),
        np.array(indices, dtype="<u4"),
    )


def _cylinder_y(radius: float, height: float, segments: int = 24):
    """Triangle mesh of a cylinder whose axis is Y (a wheel), centred at the origin."""
    positions, normals, indices = [], [], []
    angles = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.stack([np.cos(angles), np.sin(angles)], axis=1)  # (x, z)
    for side in (-1.0, 1.0):  # side wall vertices
        for x, z in ring:
            positions.append([radius * x, side * height / 2.0, radius * z])
            normals.append([x, 0.0, z])
    for i in range(segments):
        j = (i + 1) % segments
        a, b, c, d = i, j, segments + i, segments + j
        indices.extend([a, c, b, b, c, d])
    for side in (-1.0, 1.0):  # caps
        centre = len(positions)
        positions.append([0.0, side * height / 2.0, 0.0])
        normals.append([0.0, side, 0.0])
        first = len(positions)
        for x, z in ring:
            positions.append([radius * x, side * height / 2.0, radius * z])
            normals.append([0.0, side, 0.0])
        for i in range(segments):
            j = (i + 1) % segments
            tri = (centre, first + j, first + i) if side > 0 else (centre, first + i, first + j)
            indices.extend(tri)
    return (
        np.array(positions, dtype="<f4"),
        np.array(normals, dtype="<f4"),
        np.array(indices, dtype="<u4"),
    )


def _mesh(mesh_id: int, geometry) -> scene_pb2.Mesh:
    positions, normals, indices = geometry
    mesh = scene_pb2.Mesh(id=mesh_id, vertex_count=len(positions), index_count=len(indices))
    for ref, array in ((mesh.positions, positions), (mesh.normals, normals), (mesh.indices, indices)):
        data = array.tobytes()
        ref.inline = data
        ref.size = len(data)
    mesh.aabb.extend([*positions.min(axis=0).tolist(), *positions.max(axis=0).tolist()])
    return mesh


def _first_order_lag(target: np.ndarray, alpha: float) -> np.ndarray:
    """y[k] = y[k-1] + alpha * (target[k] - y[k-1]), y[-1] = target[0]."""
    out = np.empty_like(target)
    state = target[0]
    for k, value in enumerate(target):
        state += alpha * (value - state)
        out[k] = state
    return out


def _real(name: str, value: float, unit: common_pb2.Unit, doc: str) -> scene_pb2.Attribute:
    return scene_pb2.Attribute(name=name, real=value, unit=unit, authored=True, doc=doc)


def _motor_attributes() -> list[scene_pb2.Attribute]:
    # Values of the reference motor in CLAUDE.md §11.3.
    attributes = [
        _real("elec:windingResistance", 0.12, units.OHM, "phase resistance at 25 °C"),
        _real("elec:torqueConstant", 0.085, units.NEWTON_METRE_PER_AMPERE, "torque per amp"),
        _real("elec:inductance", 0.00031, units.HENRY, "phase inductance"),
        _real("elec:currentLimit", 18.0, units.AMPERE, "drive current limit"),
        scene_pb2.Attribute(
            name="elec:behaviorModel", asset="models/bldc_drive.fmu", authored=True,
            doc="behaviour model (FMI 3.0)",
        ),
        scene_pb2.Attribute(name="elec:powerPort", authored=True, doc="supply bus"),
        _real("thermal:heatCapacity", 450.0, units.JOULE_PER_KELVIN, "lumped heat capacity"),
        _real("thermal:conductanceToBase", 1.8, units.WATT_PER_KELVIN, "to the chassis"),
        _real("thermal:emissivity", 0.85, units.ONE, "surface emissivity"),
    ]
    attributes[5].targets.prims.append(7)  # /amr/power/dc_48v
    return attributes


def _sample_attributes() -> dict[int, list[scene_pb2.Attribute]]:
    """What the inspector shows for the sample robot's prims (prim ids as in generate_sample)."""
    return {
        1: [_real("physics:mass", ROBOT_MASS, units.KILOGRAM, "robot with its load")],
        6: [
            _real("elec:capacity", 40.0, units.AMPERE_HOUR, "rated capacity"),
            _real("elec:nominalVoltage", 48.0, units.VOLT, "nominal pack voltage"),
            _real("elec:internalResistance", 0.045, units.OHM, "pack resistance at 25 °C"),
            _real("thermal:heatCapacity", 9500.0, units.JOULE_PER_KELVIN, "lumped heat capacity"),
        ],
        7: [_real("elec:nominalVoltage", 48.0, units.VOLT, "bus voltage")],
        8: _motor_attributes(),
        9: _motor_attributes(),
    }


def generate_sample(duration_s: float = PATH_PERIOD, seed: int = 1) -> Recording:
    """Build the sample recording. Deterministic for a given (duration, seed)."""
    rng = np.random.default_rng(seed)
    duration_ns = round(duration_s * 1e9)
    n_mech = duration_ns // DT_MECHANICAL
    n_elec = duration_ns // DT_ELECTRICAL
    n_therm = max(1, duration_ns // DT_THERMAL)

    # --- motion at 1 kHz -----------------------------------------------------------------
    t = np.arange(n_mech) * (DT_MECHANICAL * 1e-9)
    w = 2.0 * np.pi / PATH_PERIOD
    x = PATH_AMPLITUDE * np.sin(w * t)
    y = PATH_AMPLITUDE * np.sin(w * t) * np.cos(w * t)
    vx = PATH_AMPLITUDE * w * np.cos(w * t)
    vy = PATH_AMPLITUDE * w * np.cos(2.0 * w * t)
    ax = -PATH_AMPLITUDE * w * w * np.sin(w * t)
    ay = -2.0 * PATH_AMPLITUDE * w * w * np.sin(2.0 * w * t)
    speed = np.hypot(vx, vy)
    yaw = np.unwrap(np.arctan2(vy, vx))
    yaw_rate = (vx * ay - vy * ax) / np.maximum(speed * speed, 1e-9)
    accel = (vx * ax + vy * ay) / np.maximum(speed, 1e-9)
    yaw_accel = np.gradient(yaw_rate, DT_MECHANICAL * 1e-9)

    omega_left = (speed - yaw_rate * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    omega_right = (speed + yaw_rate * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    angle_left = np.cumsum(omega_left) * (DT_MECHANICAL * 1e-9)
    angle_right = np.cumsum(omega_right) * (DT_MECHANICAL * 1e-9)

    # Wheel torques from the planar rigid-body equations plus rolling resistance.
    force = ROBOT_MASS * accel + ROLLING_RESISTANCE * ROBOT_MASS * GRAVITY
    moment = YAW_INERTIA * yaw_accel
    torque_left = (force / 2.0 - moment / WHEEL_SEPARATION) * WHEEL_RADIUS
    torque_right = (force / 2.0 + moment / WHEEL_SEPARATION) * WHEEL_RADIUS

    heading = _yaw_quaternion(yaw)
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

    def offset(dx: float, dy: float, z: float) -> np.ndarray:
        return np.stack([x + cos_yaw * dx - sin_yaw * dy, y + sin_yaw * dx + cos_yaw * dy,
                         np.full_like(x, z)], axis=1)

    def spin(angle: np.ndarray) -> np.ndarray:  # rotation about the wheel's own Y axis
        half = 0.5 * angle
        zeros = np.zeros_like(angle)
        return np.stack([np.cos(half), zeros, np.sin(half), zeros], axis=1)

    pose_base = _pose(offset(0.0, 0.0, 0.10), heading)
    pose_left = _pose(offset(0.0, 0.25, WHEEL_RADIUS), _multiply(heading, spin(angle_left)))
    pose_right = _pose(offset(0.0, -0.25, WHEEL_RADIUS), _multiply(heading, spin(angle_right)))
    pose_caster = _pose(offset(-0.24, 0.0, 0.03), heading)

    # --- electrical at 10 kHz ------------------------------------------------------------
    hold = DT_MECHANICAL // DT_ELECTRICAL
    te = np.arange(n_elec) * (DT_ELECTRICAL * 1e-9)
    alpha = 1.0 - np.exp(-CURRENT_LOOP_BANDWIDTH * DT_ELECTRICAL * 1e-9)

    def motor_current(torque: np.ndarray, phase: float) -> np.ndarray:
        command = np.repeat(torque, hold)[:n_elec] / (GEAR_RATIO * GEAR_EFFICIENCY * TORQUE_CONSTANT)
        ripple = 0.25 * np.sin(2.0 * np.pi * 1500.0 * te + phase)
        noise = rng.normal(0.0, 0.05, n_elec)
        return _first_order_lag(command, alpha) + ripple + noise

    current_left = motor_current(torque_left, 0.0)
    current_right = motor_current(torque_right, 1.3)

    omega_left_e = np.repeat(omega_left, hold)[:n_elec] * GEAR_RATIO
    omega_right_e = np.repeat(omega_right, hold)[:n_elec] * GEAR_RATIO
    # Electrical power per motor: mechanical + copper loss. Negative means regeneration.
    power_left = TORQUE_CONSTANT * current_left * omega_left_e + WINDING_RESISTANCE * current_left**2
    power_right = (
        TORQUE_CONSTANT * current_right * omega_right_e + WINDING_RESISTANCE * current_right**2
    )
    # What each branch draws from the DC bus. The bus is taken at its open-circuit voltage
    # here, so that the branch currents add up to the battery current exactly (Kirchhoff).
    dc_left = power_left / BATTERY_OPEN_CIRCUIT
    dc_right = power_right / BATTERY_OPEN_CIRCUIT
    dc_computer = np.full(n_elec, COMPUTER_POWER / BATTERY_OPEN_CIRCUIT)
    battery_current = dc_left + dc_right + dc_computer
    bus_voltage = BATTERY_OPEN_CIRCUIT - BATTERY_RESISTANCE * battery_current
    bus_voltage = bus_voltage + rng.normal(0.0, 0.01, n_elec)

    # --- thermal and state of charge at 10 Hz --------------------------------------------
    per_thermal = DT_THERMAL // DT_ELECTRICAL
    usable = (n_elec // per_thermal) * per_thermal

    def block_mean(values: np.ndarray) -> np.ndarray:
        if usable == 0:
            return np.array([values.mean() if len(values) else 0.0])
        return values[:usable].reshape(-1, per_thermal).mean(axis=1)

    def temperature(current: np.ndarray) -> np.ndarray:
        loss = WINDING_RESISTANCE * block_mean(current**2)
        out = np.empty(n_therm)
        value = AMBIENT + 12.0  # already warm from earlier work
        dt = DT_THERMAL * 1e-9
        for k in range(n_therm):
            p = loss[min(k, len(loss) - 1)]
            value += dt * (p - MOTOR_CONDUCTANCE * (value - AMBIENT)) / MOTOR_HEAT_CAPACITY
            out[k] = value
        return out

    mean_current = block_mean(battery_current)
    charge_used = np.cumsum(np.resize(mean_current, n_therm)) * (DT_THERMAL * 1e-9)
    soc = 0.82 - charge_used / BATTERY_CAPACITY

    # --- scene -----------------------------------------------------------------------------
    prim_specs = [
        # name, parent, type, schemas, mesh, material
        ("amr", NO_PARENT, "Xform", [], NO_PARENT, NO_PARENT),
        ("base_link", 0, "Xform", ["PhysicsRigidBodyAPI", "AeroAPI_1", "ThermalAPI_1"], 0, 0),
        ("left_wheel", 0, "Xform", ["PhysicsRigidBodyAPI"], 1, 1),
        ("right_wheel", 0, "Xform", ["PhysicsRigidBodyAPI"], 1, 1),
        ("caster", 0, "Xform", ["PhysicsRigidBodyAPI"], 2, 1),
        ("power", 0, "Scope", [], NO_PARENT, NO_PARENT),
        ("battery", 5, "Xform", ["BatteryAPI_1", "ThermalAPI_1", "PortAPI_1:dc_out"], NO_PARENT, NO_PARENT),
        ("dc_48v", 5, "Xform", ["ElectricalBusAPI_1"], NO_PARENT, NO_PARENT),
        ("left_drive_motor", 0, "Xform", ["ElectricalMotorAPI_1", "ThermalAPI_1"], NO_PARENT, NO_PARENT),
        ("right_drive_motor", 0, "Xform", ["ElectricalMotorAPI_1", "ThermalAPI_1"], NO_PARENT, NO_PARENT),
        ("ground", NO_PARENT, "Mesh", [], 3, 2),
        # Appended so that the ids above stay what the tests and bindings expect.
        ("computer", 5, "Xform", ["PortAPI_1:dc_in"], NO_PARENT, NO_PARENT),
        ("sensors", 0, "Scope", [], NO_PARENT, NO_PARENT),
        ("lidar_front", 12, "Lidar", ["SensorLidarAPI_1"], NO_PARENT, NO_PARENT),
        ("imu", 12, "Imu", ["SensorImuAPI_1"], NO_PARENT, NO_PARENT),
    ]
    paths: list[str] = []
    snapshot = scene_pb2.SceneSnapshot(sequence=1, stage_path="synthetic:amr_figure_eight")
    for prim_id, (name, parent, type_name, schemas, mesh, material) in enumerate(prim_specs):
        path = f"/{name}" if parent == NO_PARENT else f"{paths[parent]}/{name}"
        paths.append(path)
        children = sum(1 for spec in prim_specs if spec[1] == prim_id)
        snapshot.prims.add(
            id=prim_id, parent=parent, name=name, path=path, type_name=type_name,
            api_schemas=schemas, mesh=mesh, material=material, visible=True, child_count=children,
        )
    snapshot.meshes.extend(
        [
            _mesh(0, _box(0.60, 0.44, 0.20)),
            _mesh(1, _cylinder_y(WHEEL_RADIUS, 0.04)),
            _mesh(2, _cylinder_y(0.03, 0.02)),
            _mesh(3, _box(12.0, 8.0, 0.02)),
        ]
    )
    snapshot.materials.add(id=0, name="painted_steel", base_color=[0.36, 0.43, 0.56, 1.0],
                           metallic=0.6, roughness=0.5)
    snapshot.materials.add(id=1, name="rubber", base_color=[0.05, 0.05, 0.05, 1.0], roughness=0.9)
    snapshot.materials.add(id=2, name="floor", base_color=[0.004, 0.008, 0.018, 1.0],
                           roughness=0.8)
    net = snapshot.nets.add(id=0, name="dc_48v", bus_prim=7, voltage_signal="bus/dc_48v/voltage")
    net.terminals.add(prim=6, port="dc_out", current_signal="battery/battery/current")
    net.terminals.add(prim=8, port="dc_in", current_signal="motor/left_drive_motor/current_dc")
    net.terminals.add(prim=9, port="dc_in", current_signal="motor/right_drive_motor/current_dc")
    net.terminals.add(prim=11, port="dc_in", current_signal="load/computer/current")

    # --- signals ---------------------------------------------------------------------------
    scalar = common_pb2.SIGNAL_TYPE_SCALAR
    pose = common_pb2.SIGNAL_TYPE_POSE
    pose_names = ["x", "y", "z", "qw", "qx", "qy", "qz"]
    specs = [
        # name, type, unit, dt, data, writer, prim
        ("body/base_link/pose", pose, units.METRE, DT_MECHANICAL, pose_base, "rigid_body", 1),
        ("body/left_wheel/pose", pose, units.METRE, DT_MECHANICAL, pose_left, "rigid_body", 2),
        ("body/right_wheel/pose", pose, units.METRE, DT_MECHANICAL, pose_right, "rigid_body", 3),
        ("body/caster/pose", pose, units.METRE, DT_MECHANICAL, pose_caster, "rigid_body", 4),
        ("joint/left_wheel_joint/position", scalar, units.RADIAN, DT_MECHANICAL, angle_left, "rigid_body", 2),
        ("joint/left_wheel_joint/velocity", scalar, units.RADIAN_PER_SECOND, DT_MECHANICAL, omega_left, "rigid_body", 2),
        ("joint/left_wheel_joint/torque", scalar, units.NEWTON_METRE, DT_MECHANICAL, torque_left, "electrical", 2),
        ("joint/right_wheel_joint/position", scalar, units.RADIAN, DT_MECHANICAL, angle_right, "rigid_body", 3),
        ("joint/right_wheel_joint/velocity", scalar, units.RADIAN_PER_SECOND, DT_MECHANICAL, omega_right, "rigid_body", 3),
        ("joint/right_wheel_joint/torque", scalar, units.NEWTON_METRE, DT_MECHANICAL, torque_right, "electrical", 3),
        ("motor/left_drive_motor/current_q", scalar, units.AMPERE, DT_ELECTRICAL, current_left, "electrical", 8),
        ("motor/right_drive_motor/current_q", scalar, units.AMPERE, DT_ELECTRICAL, current_right, "electrical", 9),
        ("bus/dc_48v/voltage", scalar, units.VOLT, DT_ELECTRICAL, bus_voltage, "electrical", 7),
        ("battery/battery/current", scalar, units.AMPERE, DT_ELECTRICAL, battery_current, "electrical", 6),
        ("battery/battery/soc", scalar, units.ONE, DT_THERMAL, soc, "electrical", 6),
        ("thermal/left_drive_motor/temperature", scalar, units.KELVIN, DT_THERMAL, temperature(current_left), "thermal", 8),
        ("thermal/right_drive_motor/temperature", scalar, units.KELVIN, DT_THERMAL, temperature(current_right), "thermal", 9),
        # DC-side branch currents: what the schematic draws on its wires.
        ("motor/left_drive_motor/current_dc", scalar, units.AMPERE, DT_ELECTRICAL, dc_left, "electrical", 8),
        ("motor/right_drive_motor/current_dc", scalar, units.AMPERE, DT_ELECTRICAL, dc_right, "electrical", 9),
        ("load/computer/current", scalar, units.AMPERE, DT_ELECTRICAL, dc_computer, "electrical", 11),
        # What the IMU measures in the robot's frame: forward and sideways acceleration, turn rate.
        ("sensor/imu/accel_x", scalar, units.METRE_PER_SECOND_SQUARED, DT_MECHANICAL, accel + rng.normal(0.0, IMU_ACCEL_NOISE, n_mech), "sensors", 14),
        ("sensor/imu/accel_y", scalar, units.METRE_PER_SECOND_SQUARED, DT_MECHANICAL, speed * yaw_rate + rng.normal(0.0, IMU_ACCEL_NOISE, n_mech), "sensors", 14),
        ("sensor/imu/gyro_z", scalar, units.RADIAN_PER_SECOND, DT_MECHANICAL, yaw_rate + rng.normal(0.0, IMU_GYRO_NOISE, n_mech), "sensors", 14),
    ]
    signals = []
    for signal_id, (name, kind, unit, dt, data, writer, prim) in enumerate(specs):
        array = np.ascontiguousarray(np.asarray(data, dtype=np.float64).reshape(len(data), -1))
        info = signals_pb2.SignalInfo(
            id=signal_id, name=name, type=kind, unit=unit, components=array.shape[1],
            native_dt_ns=dt, writer=f"{writer}/synthetic", prim=prim,
            component_names=pose_names if kind == pose else [],
        )
        signals.append(Signal(info=info, data=array))

    # Initial transforms: every prim at its pose at t = 0 (or the origin for static prims).
    bindings = {1: "body/base_link/pose", 2: "body/left_wheel/pose", 3: "body/right_wheel/pose",
                4: "body/caster/pose", 0: "body/base_link/pose", 5: "body/base_link/pose",
                6: "body/base_link/pose", 7: "body/base_link/pose", 8: "body/left_wheel/pose",
                9: "body/right_wheel/pose", 11: "body/base_link/pose",
                12: "body/base_link/pose", 13: "body/base_link/pose", 14: "body/base_link/pose"}
    by_name = {s.info.name: s for s in signals}
    transforms = bytearray()
    for prim_id in range(len(prim_specs)):
        if prim_id in bindings:
            first = by_name[bindings[prim_id]].data[0]
            position, quaternion = first[:3], first[3:]
        else:
            position, quaternion = np.array([0.0, 0.0, -0.02]), np.array([1.0, 0.0, 0.0, 0.0])
        transforms += np.asarray(position, dtype="<f8").tobytes()
        transforms += np.asarray(quaternion, dtype="<f4").tobytes()
        transforms += np.ones(3, dtype="<f4").tobytes()
    snapshot.transforms.inline = bytes(transforms)
    snapshot.transforms.size = len(transforms)

    meta = record_pb2.RunMeta(
        format_version=FORMAT_VERSION,
        stage_path=snapshot.stage_path,
        seed=seed,
        generator="lagrange_mock_server.sample",
        synthetic=True,
        description=(
            "Synthetic data, not a simulation: a differential-drive AMR on a figure-eight path "
            "with closed-form motion and first-order electrical and thermal models."
        ),
        duration=common_pb2.SimTime(ns=duration_ns),
        pose_bindings=[
            record_pb2.PoseBinding(prim=prim, signal=signal) for prim, signal in sorted(bindings.items())
        ],
    )
    recording = Recording(meta=meta, snapshot=snapshot, signals=signals)
    recording.attributes = _sample_attributes()
    recording.validate()
    return recording
