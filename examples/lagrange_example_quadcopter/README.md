# A quadcopter

The sample quadcopter over the open field. It takes off and hovers by itself; a node flies it forward over `/cmd_vel`.

```bash
ros2 launch lagrange_example_quadcopter fly.launch.py                         # with the viewer
ros2 launch lagrange_example_quadcopter fly.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: flew 4.02 m forward at 2.00 m above the start
```

Where to look: `launch/fly.launch.py`, `scripts/fly.py`.

What it shows: `--air-density`, `--geo-origin`, `/cmd_vel`.
