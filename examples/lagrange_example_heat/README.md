# Heat inside the parts

Two LEDs and their driver on a circuit board, on a finned aluminium heat sink (`assets/robots/led_heatsink/led_heatsink.urdf`). The materials and heat sources are in the URDF; Lagrange works out the temperature inside every part (RFC 0012). A node reads the LED's, the board's hottest and the sink's temperatures on ROS 2 as they rise; the viewer opens in the Thermal shading, each part coloured cell by cell.

```bash
ros2 launch lagrange_example_heat heat.launch.py
ros2 launch lagrange_example_heat heat.launch.py viewer:=false once:=true
```

Where to look: the URDF's `<thermal_material>` and `<heat_source>` elements, `config/topics.yaml` (signals on topics), `launch/heat.launch.py`, `scripts/read_heat.py`.

What it shows: `/heat/led_left`, `/heat/board_max`, `/heat/sink` (`sensor_msgs/Temperature`, °C), from `thermal/<source>/temperature` and `thermal/<link>/max_temperature`. More: docs/physics/heat-conduction.md.
