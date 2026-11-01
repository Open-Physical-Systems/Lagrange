# A trotting quadruped

A four-legged robot from a URDF with three driven joints a leg. Its legs branch, so its base stays free. A node settles it into a stance and trots: feet on an ellipse, leg angles by inverse kinematics, sent as positions on `/joint_commands`.

```bash
ros2 launch lagrange_example_quadruped trot.launch.py
ros2 launch lagrange_example_quadruped trot.launch.py viewer:=false once:=true
```

The result it logs, on the machine it was checked on:

```
RESULT: trotted 1.00 m forward in 11.0 s, -0.94 m sideways, body at 0.29 m
```

It is an illustration, not a locomotion controller: open loop, with no balance or heading feedback, so it veers and goes at a fifth of what the stride asks. Put your own controller in `scripts/trot.py`.

Where to look: `robot/quadruped.urdf`, `launch/trot.launch.py`, `scripts/trot.py`.

What it shows: `--joint-control`, `--joint-loop-hz`, `/joint_commands`, a robot with no built-in controller.
