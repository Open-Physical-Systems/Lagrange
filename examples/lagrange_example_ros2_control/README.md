# An arm through ros2_control

The sample arm with controllers spawned as on real hardware, and a trajectory sent to `joint_trajectory_controller`. Your stack sees the hardware interface and nothing else.

```bash
ros2 launch lagrange_example_ros2_control arm.launch.py                         # with the viewer
ros2 launch lagrange_example_ros2_control arm.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: trajectory of 3 s ended with error code 0 (success)
```

Where to look: `config/controllers.yaml`, `launch/arm.launch.py`, `scripts/send_trajectory.py`.

What it shows: `--ros2-control`, `controller_manager` spawner, `FollowJointTrajectory`.
