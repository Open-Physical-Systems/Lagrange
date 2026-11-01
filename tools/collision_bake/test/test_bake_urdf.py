"""A URDF's concave collision mesh becomes convex parts, and something can pass through its gap."""

from __future__ import annotations

from pathlib import Path

import lagrange
import pytest
from lagrange_collision_bake.bake import Settings
from lagrange_collision_bake.bake_urdf import bake_urdf, read_obj

pytest.importorskip("coacd")

# A staple seen from the side, 1 m wide with a 0.6 m gap: two legs and a bridge.
BOXES = [(-0.5, -0.3, 0.0, 1.0), (0.3, 0.5, 0.0, 1.0), (-0.5, 0.5, 0.8, 1.0)]


def staple_obj() -> str:
    lines = []
    count = 0
    for x0, x1, z0, z1 in BOXES:
        for x in (x0, x1):
            for y in (-0.2, 0.2):
                for z in (z0, z1):
                    lines.append(f"v {x} {y} {z}")
        for a, b, c, d in [
            (1, 2, 4, 3),
            (5, 7, 8, 6),
            (1, 5, 6, 2),
            (3, 4, 8, 7),
            (1, 3, 7, 5),
            (2, 6, 8, 4),
        ]:
            lines.append(f"f {a + count} {b + count} {c + count}")
            lines.append(f"f {a + count} {c + count} {d + count}")
        count += 8
    return "\n".join(lines) + "\n"


URDF = """<robot name="staple">
  <link name="world"/>
  <joint name="mount" type="fixed"><parent link="world"/><child link="body"/></joint>
  <link name="body">
    <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry><mesh filename="staple.obj" scale="2 1 1"/></geometry>
    </collision>
  </link>
</robot>
"""


def test_a_concave_mesh_becomes_parts_and_the_rest_of_the_file_stays(tmp_path: Path) -> None:
    (tmp_path / "staple.obj").write_text(staple_obj())
    source = tmp_path / "robot.urdf"
    source.write_text(URDF)
    target = tmp_path / "robot_baked.urdf"
    report = bake_urdf(source, target, Settings(threshold=0.05, seed=1))
    assert report[0][0] == "body"
    assert report[0][1] >= 3, report
    assert source.read_text() == URDF
    text = target.read_text()
    assert text.count("<collision>") == report[0][1]
    assert 'xyz="0 0 0.05"' in text
    # The scale went into the parts: 2 m wide.
    xs = []
    for part in (tmp_path / "robot_baked_collision").glob("*.obj"):
        points, _faces = read_obj(part)
        xs += list(points[:, 0])
    assert max(xs) == pytest.approx(1.0, abs=0.05)
    assert min(xs) == pytest.approx(-1.0, abs=0.05)
    # And it loads: one collider per part, besides the ground.
    sim = lagrange.Simulation(str(target), world=None)
    assert sim.colliders() >= report[0][1]
