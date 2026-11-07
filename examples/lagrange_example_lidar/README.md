# A simulated lidar

The sample robot in the sample warehouse. A node reads `/scan` and drives until something is within a metre.

```bash
ros2 launch lagrange_example_lidar lidar.launch.py                         # with the viewer
ros2 launch lagrange_example_lidar lidar.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: stopped with something 0.98 m ahead
```

Where to look: `launch/lidar.launch.py`, `scripts/stop_at_wall.py`.

What it shows: `/scan`, `lagrange_assets`.
