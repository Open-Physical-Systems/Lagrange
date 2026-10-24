"""Lagrange simulator from Python (roadmap 0.2-H5).

import lagrange

sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
sim.step_until(2.0)
print(sim.read("body/base_link/pose"))
"""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory

from lagrange._lagrange import Simulation

__all__ = ["Simulation", "sample_robot", "sample_world"]


def sample_robot() -> str:
    """The sample differential-drive AMR (URDF) from lagrange_assets."""
    return (
        get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive/amr_diffdrive.urdf"
    )


def sample_world() -> str:
    """The sample warehouse (URDF) from lagrange_assets."""
    return (
        get_package_share_directory("lagrange_assets")
        + "/worlds/warehouse_small/warehouse_small.urdf"
    )
