"""Lagrange simulator from Python (roadmap 0.2-H5).

import lagrange

sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
sim.step_until(2.0)
print(sim.read("body/base_link/pose"))
"""

from __future__ import annotations

import pkgutil

from ament_index_python.packages import get_package_share_directory

# Other packages also install modules under `lagrange` (lagrange_proto: lagrange.viewer.v1,
# lagrange.record.v1). This package is a regular one, so without this whichever comes first on
# the path would hide the others (the mock server then fails to import its messages).
__path__ = pkgutil.extend_path(__path__, __name__)

from lagrange._lagrange import Simulation

__all__ = [
    "Simulation",
    "sample_city",
    "sample_profile",
    "sample_quadcopter",
    "sample_robot",
    "sample_sedan",
    "sample_tiled_town",
    "sample_town",
    "sample_world",
]


def sample_robot() -> str:
    """The sample differential-drive AMR (URDF) from lagrange_assets."""
    return (
        get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive/amr_diffdrive.urdf"
    )


def sample_quadcopter() -> str:
    """The sample quadcopter (USD, with its rotors, motors and battery) from lagrange_assets."""
    return get_package_share_directory("lagrange_assets") + "/robots/quad_x500/quad_x500.usda"


def sample_sedan() -> str:
    """The sample sedan (USD, a vehicle on Magic Formula tyres) from lagrange_assets."""
    return get_package_share_directory("lagrange_assets") + "/robots/sedan/sedan.usda"


def sample_town() -> str:
    """The sample town (USD: a road grid, crossings, buildings, a park) from lagrange_assets."""
    return get_package_share_directory("lagrange_assets") + "/worlds/town/town.usda"


def sample_city() -> str:
    """The sample city block (USD: a junction, pavements, buildings) from lagrange_assets."""
    return get_package_share_directory("lagrange_assets") + "/worlds/city_block/city_block.usda"


def sample_profile(name: str) -> str:
    """An example profile from lagrange_profiles, e.g. "quad_x500_hover"."""
    return get_package_share_directory("lagrange_profiles") + f"/examples/{name}.yaml"


def sample_tiled_town() -> str:
    """The tiled town (USD, streamed in 68 tiles: huge-worlds.md section 2) from lagrange_assets."""
    return get_package_share_directory("lagrange_assets") + "/worlds/tiled_town/tiled_town.usda"


def sample_world() -> str:
    """The sample warehouse (URDF) from lagrange_assets."""
    return (
        get_package_share_directory("lagrange_assets")
        + "/worlds/warehouse_small/warehouse_small.urdf"
    )
