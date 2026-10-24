#!/usr/bin/env python3
"""Writes the Gazebo Jetty world for the v0.2 comparison (docs/release/v0.2.0-comparison.md).

The sample AMR converts with `gz sdf -p amr_diffdrive.urdf`. The warehouse does not: its URDF
is rooted at a link named "world", which gz's converter rejects. Its links are boxes bolted to
the world, so this script writes them as one static SDFormat model, with the same physics step
(1 ms) and gravity as Lagrange.

    make_gazebo_world.py WAREHOUSE.urdf AMR.sdf OUT.sdf [--no-sensors]
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET


def main() -> int:
    warehouse, amr, out = sys.argv[1:4]
    sensors = "--no-sensors" not in sys.argv
    root = ET.parse(warehouse).getroot()
    origins = {}
    for joint in root.findall("joint"):
        if joint.find("parent").get("link") == "world":
            origin = joint.find("origin")
            xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            origins[joint.find("child").get("link")] = f"{xyz} {rpy}"
    links = []
    for link in root.findall("link"):
        name = link.get("name")
        box = link.find("collision/geometry/box")
        if name not in origins or box is None:
            continue
        size = box.get("size")
        links.append(
            f'      <link name="{name}"><pose>{origins[name]}</pose>'
            f"<collision name=\"c\"><geometry><box><size>{size}</size></box></geometry>"
            f"</collision><visual name=\"v\"><geometry><box><size>{size}</size></box>"
            f"</geometry></visual></link>"
        )
    sensor_systems = (
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
        "<render_engine>ogre2</render_engine></plugin>\n"
        '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n'
        if sensors
        else ""
    )
    with open(out, "w", encoding="utf-8") as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="warehouse_small">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gravity>0 0 -9.80665</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
{sensor_systems}    <model name="ground"><static>true</static>
      <link name="ground"><collision name="c"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry></collision></link>
    </model>
    <model name="warehouse"><static>true</static>
{chr(10).join(links)}
    </model>
    <include><uri>file://{amr}</uri><pose>0 0 0.11 0 0 0</pose></include>
  </world>
</sdf>
""")
    print(f"{out}: {len(links)} warehouse links, sensors {'on' if sensors else 'off'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
