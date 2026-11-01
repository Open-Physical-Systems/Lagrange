# Your own wheeled robot

A URDF written here (a box on two wheels, its `<ros2_control>` block saying which joints are driven) on an empty ground. A node drives a square over `/cmd_vel`, reading `/odom`.

```bash
ros2 launch lagrange_example_diff_drive drive.launch.py                         # with the viewer
ros2 launch lagrange_example_diff_drive drive.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: drove 4 sides of 1.5 m, ended 0.20 m from where it started
```

Where to look: `robot/two_wheels.urdf`, `launch/drive.launch.py`, `scripts/drive_square.py`.

What it shows: `--robot FILE`, `--world none`, `--no-demo`, `/cmd_vel`, `/odom`.
