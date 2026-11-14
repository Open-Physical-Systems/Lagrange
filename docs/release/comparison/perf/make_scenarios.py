#!/usr/bin/env python3
# ruff: noqa: E501, RUF002, RUF003  (comparison scripts: generated XML, units in text)
"""Writes the performance scenarios of docs/comparison.md for Lagrange and Gazebo Jetty.

    make_scenarios.py REPO OUT_DIR

REPO is the Lagrange source tree (for the sample AMR and warehouse); `gz` must be on the PATH
(it converts the AMR's URDF to SDFormat). Writes into OUT_DIR:

- amr.sdf                      the sample AMR as `gz sdf -p` converts it
- amr_physics.sdf, amr_sensors.sdf   Gazebo worlds: the warehouse as one static model, the AMR,
                               1 ms step; with the Sensors (ogre2) and IMU systems or without
- amr_camera.urdf              the sample AMR with its lidars replaced by one 640×480 camera at
                               30 Hz (Gazebo's <sensor type="camera"> form, read by both)
- amr_camera.sdf, gz_amr_camera.sdf  its conversion and its Gazebo world
- boxes.xml, boxes.sdf, boxes.json   200 boxes dropped onto the ground into a pile
- profile_*.yaml               Lagrange profiles (physics only, sensors, camera, boxes, quad)
- gz_quad.sdf                  Gazebo's multicopter example's X3 alone (its model from Fuel:
                               `gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4"`)
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

DOMAINS_OFF = """  electrical:   {{ enabled: false, substitute: ideal }}
  thermal:      {{ enabled: false, substitute: constant }}
  aerodynamics: {{ enabled: false, substitute: zero }}
  weather:      {{ enabled: false, substitute: clear }}
  emi:          {{ enabled: false, substitute: zero }}
  radiation:    {{ enabled: false, substitute: zero }}
"""

PROFILE = (
    """schema_version: 1
stage: {stage}
domains:
  rigid_body:   {{ enabled: true,  backend: {backend}, dt: 0.001 }}
"""
    + DOMAINS_OFF
    + "  sensors:      {sensors}\n"
)

CAMERA = """  <gazebo reference="base_link">
    <sensor name="front_camera" type="camera">
      <pose>0.25 0 0.3 0 0 0</pose>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>100</far></clip>
      </camera>
    </sensor>
  </gazebo>
"""

# 200 boxes of 0.2 m, 1 kg, in 8 layers of 5 × 5 spaced 0.3 m, each layer shifted and turned a
# little (fixed numbers, no randomness) so that they tumble into a pile on the ground.
BOX = 0.2
BOX_M = 1.0
LAYERS, ROWS = 8, 5


def boxes() -> list[tuple[float, float, float, float]]:
    out = []
    for k in range(LAYERS):
        for i in range(ROWS):
            for j in range(ROWS):
                x = (i - 2) * 0.3 + 0.05 * math.sin(1.7 * k + i)
                y = (j - 2) * 0.3 + 0.05 * math.cos(1.3 * k + j)
                z = 0.5 + 0.35 * k
                yaw = 0.3 * math.sin(k + 2.0 * i + 3.0 * j)
                out.append((x, y, z, yaw))
    return out


def boxes_mjcf(items: list[tuple[float, float, float, float]]) -> str:
    h = BOX / 2
    bodies = "\n".join(
        f'    <body name="box{n}" pos="{x:.6f} {y:.6f} {z:.6f}" axisangle="0 0 1 {yaw:.6f}">'
        f'<freejoint/><geom type="box" size="{h} {h} {h}" mass="{BOX_M}"/></body>'
        for n, (x, y, z, yaw) in enumerate(items)
    )
    return f"""<mujoco model="boxes">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="ground" type="box" size="20 20 0.1" pos="0 0 -0.1"/>
{bodies}
  </worldbody>
</mujoco>
"""


def boxes_sdf(items: list[tuple[float, float, float, float]]) -> str:
    i = BOX_M * BOX * BOX / 6.0
    models = "\n".join(
        f'    <model name="box{n}"><pose>{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}</pose><link name="l">'
        f"<inertial><mass>{BOX_M}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f'<collision name="c"><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry>'
        f"</collision></link></model>"
        for n, (x, y, z, yaw) in enumerate(items)
    )
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="boxes">
    <physics name="1ms" type="ode"><max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <model name="ground"><static>true</static><pose>0 0 -0.1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>40 40 0.2</size></box></geometry></collision>
    </link></model>
{models}
  </world>
</sdf>
"""


def camera_world(amr_sdf: Path) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="amr_camera">
    <physics name="1ms" type="ode"><max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 -9.80665</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    __WAREHOUSE__
    <include><uri>file://{amr_sdf}</uri><pose>0 0 0.11 0 0 0</pose></include>
  </world>
</sdf>
"""


def gz_share() -> Path:
    """gz-sim's share directory (worlds/), from `gz sim --versions`' installation."""
    for root in (
        Path("/opt/ros/lyrical/opt/gz_sim_vendor/share/gz/gz-sim"),
        Path("/usr/share/gz/gz-sim"),
    ):
        if (root / "worlds/multicopter_velocity_control.sdf").exists():
            return root
    raise FileNotFoundError("gz-sim's worlds/multicopter_velocity_control.sdf")


def main() -> int:
    repo, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    amr_urdf = repo / "assets/robots/amr_diffdrive/amr_diffdrive.urdf"
    warehouse = repo / "assets/worlds/warehouse_small/warehouse_small.urdf"
    make_world = HERE.parent / "make_gazebo_world.py"

    amr_sdf = out / "amr.sdf"
    amr_sdf.write_text(subprocess.check_output(["gz", "sdf", "-p", str(amr_urdf)], text=True))
    py = sys.executable
    subprocess.check_call(
        [py, str(make_world), str(warehouse), str(amr_sdf), str(out / "amr_sensors.sdf")]
    )
    subprocess.check_call(
        [
            py,
            str(make_world),
            str(warehouse),
            str(amr_sdf),
            str(out / "amr_physics.sdf"),
            "--no-sensors",
        ]
    )

    # The camera AMR: the sample with its two lidars' <gazebo> blocks replaced by one camera.
    text = amr_urdf.read_text(encoding="utf-8")
    text, n = re.subn(
        r'  <gazebo reference="(lidar_link|top_lidar_link)">.*?</gazebo>\n', "", text, flags=re.S
    )
    assert n == 2, f"expected the two lidar blocks, found {n}"
    text = text.replace("</robot>", CAMERA + "</robot>")
    cam_urdf = out / "amr_camera.urdf"
    cam_urdf.write_text(text, encoding="utf-8")
    cam_sdf = out / "amr_camera.sdf"
    cam_sdf.write_text(subprocess.check_output(["gz", "sdf", "-p", str(cam_urdf)], text=True))
    # The warehouse model of amr_physics.sdf, reused for the camera world.
    physics = (out / "amr_physics.sdf").read_text(encoding="utf-8")
    ground_and_warehouse = physics[
        physics.index('<model name="ground">') : physics.index("<include>")
    ]
    (out / "gz_amr_camera.sdf").write_text(
        camera_world(cam_sdf).replace("__WAREHOUSE__", ground_and_warehouse.strip()),
        encoding="utf-8",
    )

    items = boxes()
    (out / "boxes.xml").write_text(boxes_mjcf(items), encoding="utf-8")
    (out / "boxes.sdf").write_text(boxes_sdf(items), encoding="utf-8")
    (out / "boxes.json").write_text(
        json.dumps({"size": BOX, "mass": BOX_M, "boxes": items}), encoding="utf-8"
    )

    for name, backend, stage, sensors in (
        ("profile_physics", "mujoco", amr_urdf, "{ enabled: false, substitute: ground_truth }"),
        ("profile_physics_jolt", "jolt", amr_urdf, "{ enabled: false, substitute: ground_truth }"),
        ("profile_sensors", "mujoco", amr_urdf, "{ enabled: true }"),
        ("profile_camera", "mujoco", cam_urdf, "{ enabled: true }"),
        (
            "profile_boxes",
            "mujoco",
            out / "boxes.xml",
            "{ enabled: false, substitute: ground_truth }",
        ),
        (
            "profile_boxes_jolt",
            "jolt",
            out / "boxes.xml",
            "{ enabled: false, substitute: ground_truth }",
        ),
    ):
        (out / f"{name}.yaml").write_text(
            PROFILE.format(stage=stage, backend=backend, sensors=sensors), encoding="utf-8"
        )
    # Quadcopter hover. Lagrange: the sample quadcopter's own profile (electrical at 50 µs,
    # sensors), and a lean one closer to Gazebo's: no sensors. Its electrical network stays on:
    # with electrical substituted (ideal) the rotors do not turn in v1.3.0 and it never takes off.
    quad = repo / "assets/robots/quad_x500/quad_x500.usda"
    full = (repo / "profiles/examples/quad_x500_hover.yaml").read_text(encoding="utf-8")
    (out / "profile_quad_full.yaml").write_text(
        full.replace("stage: robots/quad_x500/quad_x500.usda", f"stage: {quad}"), encoding="utf-8"
    )
    (out / "profile_quad_lean.yaml").write_text(
        PROFILE.format(
            stage=quad, backend="mujoco", sensors="{ enabled: false, substitute: ground_truth }"
        ).replace(
            "electrical:   { enabled: false, substitute: ideal }",
            "electrical:   { enabled: true,  fidelity: averaged, dt: 0.00005 }",
        )
        + "  rotor_aero:   { enabled: true,  fidelity: coefficients, ground_effect: true, air_density: 1.225 }\n",
        encoding="utf-8",
    )
    # Gazebo: its multicopter example's X3 (MulticopterMotorModel ×4, MulticopterVelocityControl,
    # from Fuel) alone, 2 m up, at a 1 ms step instead of the example's 4 ms, without rendering.
    src = (gz_share() / "worlds/multicopter_velocity_control.sdf").read_text(encoding="utf-8")
    a = src.index("    <include>")
    b = src.index("    </include>", a) + len("    </include>")
    x3 = src[a:b].replace("    <include>\n", "    <include>\n      <pose>0 0 2 0 0 0</pose>\n", 1)
    g = src.index('<model name="ground_plane">')
    ground = src[g : src.index("</model>", g) + len("</model>")]
    (out / "gz_quad.sdf").write_text(
        f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="multicopter">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    {ground}
{x3}
  </world>
</sdf>
""",
        encoding="utf-8",
    )
    print(f"scenarios in {out}: {len(items)} boxes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
