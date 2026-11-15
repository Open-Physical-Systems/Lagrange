"""Renaming in the Builder (RFC 0016; 12-builder SPEC "Robot (tree)" and "Properties"), headless.

F2 makes the selection's tree row its name field: Enter renames (one undo step, every reference
follows: joints' parent and child, a sensor's link, a mimic leader, <ros2_control> kept as
written), Esc leaves it. A power part's name field in Properties renames the part and what
connects to it. The tree's Add menu moves a link under another (Joint between…). No server.
Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROVER = HERE / "builder" / "rover_mk1.urdf"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
ROBOT_ROW = "30,177"  # the robot in the tree: Properties shows its Power section
FIRST_BUS_NAME = "1440,537"  # with RAIL open and the robot selected: Bus · dc_48v's name field
TREE_ADD = "237,144"  # the tree's Add button
JOINT_BETWEEN = "290,207"  # its menu: "Joint between…"
JOINT_BETWEEN_MAST = "390,290"  # that submenu's "mast" (rover_mk1, tilt_link selected)

# A two-bus rail (the test_builder_power.py rover's, shortened): battery → dc_48v → converter →
# dc_12v → load. Values illustrative, not a product's.
RAIL = """\
schema_version: 1
apply:
  - create: power/dc_48v
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.0047
        elec:bus:undervoltageTrip: 42.0
        elec:bus:overvoltageTrip: 60.0
  - create: power/battery
    schemas:
      BatteryAPI_1:
        elec:battery:chemistry: lfp
        elec:battery:seriesCells: 16
        elec:battery:capacity: 72000.0
        elec:battery:ocvSoc: [0.0, 0.1, 0.5, 0.9, 1.0]
        elec:battery:ocvVoltage: [2.80, 3.20, 3.28, 3.33, 3.45]
        elec:battery:r0: 0.0015
        elec:battery:cutoffVoltage: 44.8
        elec:battery:maxChargeCurrent: 10.0
      PortAPI_1:dc_out:
        elec:port:dc_out:connectedTo: power/dc_48v
  - create: power/dc_12v
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.001
        elec:bus:undervoltageTrip: 10.0
        elec:bus:overvoltageTrip: 14.0
  - create: power/dcdc
    schemas:
      ElectricalConverterAPI_1:
        elec:conv:outputVoltage: 12.0
        elec:conv:efficiency: 0.9
        elec:conv:maxOutputCurrent: 5.0
        elec:conv:input: power/dc_48v
        elec:conv:output: power/dc_12v
  - create: power/computer
    schemas:
      ElectricalLoadAPI_1:
        elec:load:power: 20.0
        elec:powerPort: power/dc_12v
"""

# A pan joint with a mimic follower and a <ros2_control> block naming it.
INERTIA = 'ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"'
CONTROLLED = f"""\
<?xml version="1.0"?>
<robot name="head">
  <link name="base_link">
    <inertial><mass value="1"/><inertia {INERTIA}/></inertial>
    <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision>
  </link>
  <link name="pan_link">
    <inertial><mass value="0.2"/><inertia {INERTIA}/></inertial>
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="twin_link">
    <inertial><mass value="0.2"/><inertia {INERTIA}/></inertial>
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <joint name="pan_joint" type="revolute">
    <parent link="base_link"/><child link="pan_link"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="2" velocity="3"/>
  </joint>
  <joint name="twin_joint" type="revolute">
    <parent link="base_link"/><child link="twin_link"/>
    <origin xyz="0.1 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="2" velocity="3"/>
    <mimic joint="pan_joint" multiplier="1" offset="0"/>
  </joint>
  <ros2_control name="head_control" type="system">
    <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
    <joint name="pan_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(tmp_path: Path, *extra: str, frames: int = 16) -> dict:
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
    }
    done = subprocess.run(
        [
            viewer(),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--workspace",
            "builder",
            "--frames",
            str(frames),
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=150,
        env=environment,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return json.loads(lines[-1])["builder"]


def rover(tmp_path: Path, overlay: str | None = None) -> Path:
    urdf = tmp_path / "rover_mk1.urdf"
    shutil.copy(ROVER, urdf)
    if overlay is not None:
        (tmp_path / "rover_mk1.overlay.yaml").write_text(overlay)
    return urdf


def test_f2_renames_a_link_in_its_tree_row(tmp_path):
    out = tmp_path / "out" / "rover_mk1.urdf"
    out.parent.mkdir()
    builder = run(
        tmp_path,
        "--builder-open",
        str(rover(tmp_path)),
        "--input",
        "3:select:mast;5:key:f2;8:text:tower;10:key:enter",
        "--builder-save",
        str(out),
    )
    assert builder["history"]["labels"] == ["Open rover_mk1.urdf", "Rename mast to tower"]
    assert builder["selected"] == "tower"
    written = out.read_text()
    # The link, its joint's child and the lidar's link (a <gazebo reference>) follow.
    assert 'link name="mast"' not in written
    assert 'link name="tower"' in written
    assert '<child link="tower"/>' in written
    assert 'reference="tower"' in written


def test_esc_leaves_the_name_and_ctrl_z_undoes_a_rename(tmp_path):
    urdf = rover(tmp_path)
    # Typed, then Esc: no step, the name as it was.
    builder = run(
        tmp_path,
        "--builder-open",
        str(urdf),
        "--input",
        "3:select:mast;5:key:f2;8:text:x;10:key:escape",
    )
    assert builder["history"]["labels"] == ["Open rover_mk1.urdf"]
    assert builder["selected"] == "mast"
    # Renamed, then Ctrl Z: back to the opened document.
    builder = run(
        tmp_path,
        "--builder-open",
        str(urdf),
        "--input",
        "3:select:mast;5:key:f2;8:text:tower;10:key:enter;13:key:ctrl+z",
    )
    assert builder["history"]["position"] == 0
    assert "mast_joint" in builder["joint_types"]


def test_a_name_in_use_is_refused_and_said(tmp_path):
    builder = run(
        tmp_path,
        "--builder-open",
        str(rover(tmp_path)),
        "--input",
        "3:select:mast;5:key:f2;8:text:caster;10:key:enter",
    )
    assert builder["history"]["labels"] == ["Open rover_mk1.urdf"]
    assert any("not renamed" in p and "caster" in p for p in builder["problems"]), builder[
        "problems"
    ]


def test_f2_on_a_joint_renames_its_mimic_and_ros2_control_references(tmp_path):
    urdf = tmp_path / "head.urdf"
    urdf.write_text(CONTROLLED)
    out = tmp_path / "out" / "head.urdf"
    out.parent.mkdir()
    builder = run(
        tmp_path,
        "--builder-open",
        str(urdf),
        "--input",
        "3:select:pan_joint;5:key:f2;8:text:yaw_joint;10:key:enter",
        "--builder-save",
        str(out),
    )
    assert builder["history"]["labels"][-1] == "Rename pan_joint to yaw_joint"
    written = out.read_text()
    assert "pan_joint" not in written
    assert '<mimic joint="yaw_joint"' in written
    # The <ros2_control> block, kept as written, names the new joint.
    control = written[written.index("<ros2_control") :]
    assert '<joint name="yaw_joint">' in control


def test_renaming_a_bus_renames_what_connects_to_it(tmp_path):
    out = tmp_path / "out" / "rover_mk1.urdf"
    out.parent.mkdir()
    builder = run(
        tmp_path,
        "--builder-open",
        str(rover(tmp_path, RAIL)),
        "--input",
        f"3:click:{ROBOT_ROW};7:click:{FIRST_BUS_NAME};10:text:main_bus;12:key:enter",
        "--builder-save",
        str(out),
    )
    assert builder["history"]["labels"] == [
        "Open rover_mk1.urdf",
        "Rename bus dc_48v to main_bus",
    ]
    assert builder["overlay"]["ElectricalBusAPI_1"] == ["main_bus", "dc_12v"]
    assert builder["errors"] == 0
    written = yaml.safe_load(out.with_name("rover_mk1.overlay.yaml").read_text())
    by_prim = {entry.get("create"): entry["schemas"] for entry in written["apply"]}
    assert "power/dc_48v" not in by_prim
    assert "power/main_bus" in by_prim
    battery = by_prim["power/battery"]["PortAPI_1:dc_out"]
    assert battery["elec:port:dc_out:connectedTo"] == "power/main_bus"
    assert by_prim["power/dcdc"]["ElectricalConverterAPI_1"]["elec:conv:input"] == "power/main_bus"


def test_a_power_name_in_use_is_refused(tmp_path):
    builder = run(
        tmp_path,
        "--builder-open",
        str(rover(tmp_path, RAIL)),
        "--input",
        f"3:click:{ROBOT_ROW};7:click:{FIRST_BUS_NAME};10:text:battery;12:key:enter",
    )
    assert builder["history"]["labels"] == ["Open rover_mk1.urdf"]
    assert builder["overlay"]["ElectricalBusAPI_1"] == ["dc_48v", "dc_12v"]


def test_joint_between_moves_a_link_under_another(tmp_path):
    builder = run(
        tmp_path,
        "--builder-open",
        str(rover(tmp_path)),
        "--input",
        f"3:select:tilt_link;5:click:{TREE_ADD};8:move:{JOINT_BETWEEN};10:move:390,207;"
        f"11:move:{JOINT_BETWEEN_MAST};12:click:{JOINT_BETWEEN_MAST}",
    )
    assert builder["history"]["labels"] == ["Open rover_mk1.urdf", "Move tilt_link to mast"]
