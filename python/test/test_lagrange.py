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
