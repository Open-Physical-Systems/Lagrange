# A trotting quadruped

A four-legged robot from a URDF with three driven joints a leg. Its legs branch, so its base stays free. A node settles it into a stance and trots: feet on an ellipse, leg angles by inverse kinematics, sent as positions on `/joint_commands`.

```bash
ros2 launch lagrange_example_quadruped trot.launch.py
ros2 launch lagrange_example_quadruped trot.launch.py viewer:=false once:=true
```

The result it logs, one run on the machine it was checked on:

```
RESULT: trotted 1.00 m forward in 8.2 s, -0.02 m sideways, body at 0.29 m
```

Expect the time to differ from run to run, about 7.5 to 10 s, and the sideways drift by up to
0.16 m either way. The physics repeats exactly; the node does not. It answers each
`/joint_states` (50 Hz) in real time, so its commands land a few milliseconds later or earlier
each run, and a gait without balance control is sensitive to that: the same trot stepped in
lockstep without ROS (commands every 20 ms, the same 25 Hz joint loops) takes 9.9 s every time,
and anywhere from 6.3 s to 10.2 s when its commands are sent 5 to 40 ms late. On a busy machine the
answers come later still.

It is an illustration, not a locomotion controller: only its heading is fed back, by lengthening the strides on one side; there is no balance control, and it goes at a third of what the stride asks. Put your own controller in `scripts/trot.py`.

Where to look: [`assets/robots/quadruped/quadruped.urdf`](../../assets/robots/quadruped/quadruped.urdf) (Lagrange's sample quadruped, installed with `lagrange_assets`; [`quadruped.overlay.yaml`](../../assets/robots/quadruped/quadruped.overlay.yaml) beside it gives it a battery and a drive on each joint, and [`profiles/examples/quadruped_trot.yaml`](../../profiles/examples/quadruped_trot.yaml) is its profile), `launch/trot.launch.py`, `scripts/trot.py`.

What it shows: `--joint-control`, `--joint-loop-hz`, `/joint_commands`, a robot with no built-in controller.
