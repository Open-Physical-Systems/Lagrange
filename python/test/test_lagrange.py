"""Roadmap 0.2-H5. Acceptance: a Python script steps the AMR and reads signals."""

from __future__ import annotations

import math

import lagrange
import pytest


@pytest.fixture(scope="module")
def sim() -> lagrange.Simulation:
    return lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())


def test_the_amr_drives_and_its_signals_read(sim: lagrange.Simulation) -> None:
    start = sim.read("body/base_link/pose")
    sim.step_until(3.0)
    assert sim.time == pytest.approx(3.0)
    pose = sim.read("body/base_link/pose")
    assert len(pose) == 7  # position, then orientation (w, x, y, z)
    moved = math.hypot(pose[0] - start[0], pose[1] - start[1])
    assert moved > 0.5, "the demo drive moved the robot"
    assert sim.read("base/command/linear_velocity")[0] == pytest.approx(0.6)


def test_signals_carry_units(sim: lagrange.Simulation) -> None:
    by_name = {s["name"]: s for s in sim.signals()}
    assert by_name["joint/left_wheel_joint/velocity"]["unit"] == "rad/s"
    assert by_name["body/base_link/pose"]["components"] == 7
    with pytest.raises(KeyError):
        sim.read("no/such/signal")


def test_robots_spawn_into_an_empty_run_and_go() -> None:
    sim = lagrange.Simulation(None, world=lagrange.sample_world())
    assert "base_link" not in sim.bodies()
    sim.step_until(0.5)
    assert sim.spawn(lagrange.sample_robot(), at=[1.0, 0.0]) == "/amr_diffdrive"
    assert sim.time == pytest.approx(0.5), "time goes on"
    assert sim.spawn(lagrange.sample_robot(), "mate", at=[1.0, 3.0, 0.0, math.pi]) == "/mate"
    assert "mate_base_link" in sim.bodies()
    sim.step_until(2.0)
    sim.delete("/mate")
    assert "mate_base_link" not in sim.bodies()
    with pytest.raises(RuntimeError, match="no robot named"):
        sim.delete("/mate")


def test_times_and_poses_that_are_not_finite_are_refused() -> None:
    # QA 2026-11-12: step_until(nan) stepped nothing silently, spawn(at=[nan, 0]) broke the
    # physics and command_joint on no joint said True.
    sim = lagrange.Simulation(lagrange.sample_robot(), world=None, joint_control=True)
    for bad in (math.nan, math.inf, 1e300):
        with pytest.raises(ValueError):
            sim.step_until(bad)
    with pytest.raises(ValueError):
        sim.spawn(lagrange.sample_robot(), "mate", at=[math.nan, 0.0])
    with pytest.raises(ValueError):
        lagrange.Simulation(lagrange.sample_robot(), world=None, spawn=[0.0, math.inf])
    assert not sim.command_joint("no_such_joint", "velocity", 1.0)
    assert not sim.command_joint("left_wheel_joint", "velocity", math.nan)
    assert sim.command_joint("left_wheel_joint", "velocity", 1.0)
    sim.step_until(0.5)
    assert all(math.isfinite(x) for x in sim.read("body/base_link/pose"))


def test_world_queries() -> None:
    sim = lagrange.Simulation(lagrange.sample_robot(), world=None)
    assert "base_link" in sim.bodies()
    assert "left_wheel_joint" in sim.joints()


def test_two_runs_are_identical() -> None:
    a = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
    b = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
    a.step_until(1.0)
    b.step_until(1.0)
    assert a.state_hash() == b.state_hash()


def test_the_protocol_messages_import_alongside_the_bindings():
    """lagrange_proto installs lagrange.viewer.v1 and lagrange.record.v1 next to this package's
    lagrange; with the bindings first on the path (as a sourced workspace has them), both
    import. In a fresh interpreter: an extension module cannot be imported twice in one."""
    import os
    import subprocess
    import sys
    from pathlib import Path

    import lagrange

    bindings = str(Path(lagrange.__file__).parent.parent)
    path = os.pathsep.join([bindings, *sys.path])
    check = (
        "import lagrange, lagrange.viewer.v1.signals_pb2 as s, lagrange.record.v1.record_pb2 as r;"
        "assert lagrange.Simulation and s.SignalCatalog and r.RunMeta().format_version == 0"
    )
    result = subprocess.run(
        [sys.executable, "-c", check],
        env={**os.environ, "PYTHONPATH": path},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_the_sample_quadcopter_takes_off_and_hovers() -> None:
    # v0.4: rotors from coefficients, the electrical network and the built-in controller.
    sim = lagrange.Simulation(
        lagrange.sample_quadcopter(), world=None, profile=lagrange.sample_profile("quad_x500_hover")
    )
    start = sim.read("body/base_link/pose")
    sim.step_until(8.0)
    pose = sim.read("body/base_link/pose")
    assert pose[2] - start[2] == pytest.approx(2.0, abs=0.01), "hovering 2 m above the start"
    assert math.hypot(pose[0] - start[0], pose[1] - start[1]) < 0.01
    assert pose[3] == pytest.approx(1.0, abs=1e-3), "level"
    speeds = [sim.read(f"joint/rotor_{i}_joint/velocity")[0] for i in range(1, 5)]
    assert speeds[0] > 0 and speeds[1] > 0 and speeds[2] < 0 and speeds[3] < 0  # ccw, ccw, cw, cw
    # Momentum check: four rotors carry the weight (1.9 + 4 x 0.025 kg).
    thrust = sum(sim.read(f"rotor/rotor_{i}_joint/thrust")[0] for i in range(1, 5))
    assert thrust == pytest.approx(2.0 * 9.80665, rel=0.01)
    power = sim.read("battery/battery/power")[0]
    assert 150.0 < power < 250.0, f"hover power {power:.0f} W"


def test_the_quadcopter_holds_its_position_in_gusty_wind() -> None:
    # v0.4: link drag, the logarithmic wind profile and Dryden turbulence.
    sim = lagrange.Simulation(
        lagrange.sample_quadcopter(), world=None, profile=lagrange.sample_profile("quad_x500_gusty")
    )
    start = sim.read("body/base_link/pose")
    gusts = []
    for t in range(5, 21):
        sim.step_until(float(t))
        pose = sim.read("body/base_link/pose")
        assert math.hypot(pose[0] - start[0], pose[1] - start[1]) < 0.3
        assert pose[2] - start[2] == pytest.approx(2.0, abs=0.1)
        gusts.append(sim.read("env/wind/gust")[0])
    assert max(gusts) - min(gusts) > 1.0, "the turbulence varies"
    assert sim.read("body/base_link/airspeed")[0] > 2.0, "the wind blows on it"


def _usd_arm(folder, joints: int, hinges_driven: bool = True) -> str:
    """A chain of links in USD with no joint to the world, as CAD exports it."""
    drive = '( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )' if hinges_driven else ""
    force = "float drive:angular:physics:maxForce = 500" if hinges_driven else ""
    body = ['#usda 1.0\n(\n    defaultPrim = "arm"\n    metersPerUnit = 1\n    upAxis = "Z"\n)']
    body.append('def Xform "arm"\n{')
    for i in range(joints + 1):
        body.append(
            f'def Xform "link{i}" '
            '( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )'
            f"\n{{\nfloat physics:mass = 1\ndouble3 xformOp:translate = (0, 0, {0.5 + i * 0.2})\n"
            'uniform token[] xformOpOrder = ["xformOp:translate"]\n}'
        )
    for i in range(joints):
        body.append(
            f'def PhysicsRevoluteJoint "joint{i}" {drive}\n{{\n'
            f"rel physics:body0 = </arm/link{i}>\nrel physics:body1 = </arm/link{i + 1}>\n"
            f'uniform token physics:axis = "Z"\n{force}\n}}'
        )
    body.append("}")
    path = folder / "arm.usda"
    path.write_text("\n".join(body) + "\n")
    return str(path)


def test_a_usd_arm_with_no_world_joint_stands(tmp_path) -> None:
    sim = lagrange.Simulation(_usd_arm(tmp_path, 4), world=None)
    sim.step_until(1.0)
    pose = sim.read("body/link1/pose")  # its base is welded: only what moves has a pose
    assert abs(pose[2] - 0.7) < 1e-3, pose


def test_a_usd_chain_too_short_or_undriven_still_falls(tmp_path) -> None:
    sim = lagrange.Simulation(_usd_arm(tmp_path, 2), world=None)
    sim.step_until(0.3)
    assert sim.read("body/link0/pose")[2] < 0.4
    sim = lagrange.Simulation(_usd_arm(tmp_path, 4, hinges_driven=False), world=None)
    sim.step_until(0.3)
    assert sim.read("body/link0/pose")[2] < 0.4


def test_save_state_goes_back_and_repeats() -> None:
    sim = lagrange.Simulation(lagrange.sample_robot(), world=None)
    sim.step_until(0.2)
    state = sim.save_state()
    sim.step_until(0.5)
    first = sim.state_hash()
    sim.restore_state(state)
    assert abs(sim.time - 0.2) < 1e-9
    sim.step_until(0.5)
    assert sim.state_hash() == first


def test_contact_forces_on_a_robot_at_rest_carry_its_weight() -> None:
    sim = lagrange.Simulation(lagrange.sample_robot(), world=None)
    sim.step_until(0.0)
    names = [s["name"] for s in sim.signals() if s["name"].endswith("/contact_force")]
    assert names
    gravity = 9.80665
    robot_kg = 20.0 + 2 * 1.5 + 2 * (0.1 + 0.2) + 0.3 + 0.5  # the bodies of amr_diffdrive.urdf
    sim.step_until(0.5)
    # The sample's first half second is spent still: its demo starts after that.
    up = sum(sim.read(name)[2] for name in names)
    assert up == pytest.approx(robot_kg * gravity, rel=0.1)


def test_a_joints_reaction_is_what_it_carries() -> None:
    # A hinge about y with 2 kg held 0.5 m out sideways, in a URDF written here.
    import tempfile
    from pathlib import Path

    urdf = """<robot name="r"><link name="world"/>
      <joint name="m" type="fixed"><parent link="world"/><child link="base"/>
        <origin xyz="0 0 1"/></joint>
      <link name="base"><inertial><mass value="1"/>
        <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="100" velocity="9"/></joint>
      <link name="arm"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
        <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>
    </robot>"""
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "r.urdf"
        path.write_text(urdf)
        sim = lagrange.Simulation(str(path), world=None, joint_control=True)
        sim.command_joint("hinge", "position", 0.0)
        sim.step_until(3.0)
    fx, fy, fz, tx, ty, tz = sim.read("joint/hinge/reaction")
    weight = 2.0 * 9.80665
    assert fz == pytest.approx(weight, rel=0.01)
    assert abs(fx) < 0.1 and abs(fy) < 0.1
    # The motor's torque, about y, holds the weight's moment.
    assert abs(ty) == pytest.approx(0.5 * weight, rel=0.02)
    assert abs(tx) < 0.1 and abs(tz) < 0.1


def test_sensor_faults_are_what_python_reads() -> None:
    # sensors.md "Sensor faults (as built)": the faulted value, the fault's status, and the
    # same faults again after a restore.
    faults = ["imu:angular_rate=bias,value=0.5,at=0.2,component=2", "front_lidar=disconnect,at=0.3"]
    sim = lagrange.Simulation(lagrange.sample_robot(), world=None, sensor_faults=faults)
    sim.step_until(0.1)
    assert sim.read("sensor/imu/fault/active") == [0.0]
    state = sim.save_state()
    sim.step_until(1.0)
    assert sim.read("sensor/imu/fault/active") == [1.0]
    assert sim.read("sensor/front_lidar/fault/dropped")[0] > 0
    rate = sim.read("sensor/imu/angular_rate")[2]
    sim.restore_state(state)
    sim.step_until(1.0)
    assert sim.read("sensor/imu/angular_rate")[2] == rate
    with pytest.raises(RuntimeError, match="no sensor compass"):
        lagrange.Simulation(lagrange.sample_robot(), world=None, sensor_faults=["compass=stuck"])
    with pytest.raises(RuntimeError, match="sensor-fault needs"):
        lagrange.Simulation(lagrange.sample_robot(), world=None, sensor_faults=["imu=broken"])
