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
