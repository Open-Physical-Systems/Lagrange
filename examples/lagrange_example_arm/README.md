# An arm from a URDF

An arm with no `world` link, as CAD exports it: one chain of three driven joints, so its base stays where it is. A node sends it to poses over `/joint_commands`.

```bash
ros2 launch lagrange_example_arm arm.launch.py                         # with the viewer
ros2 launch lagrange_example_arm arm.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: reached 3 poses, within 0.05 rad
```

Where to look: `robot/three_joints.urdf`, `launch/arm.launch.py`, `scripts/wave.py`.

What it shows: `--joint-control`, `/joint_commands`, `/joint_states`.
