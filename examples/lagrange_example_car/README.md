# A car in the town

The sample sedan in the town, with traffic, working lights and walking people. A node asks for a speed over `/cmd_vel`.

```bash
ros2 launch lagrange_example_car car.launch.py                         # with the viewer
ros2 launch lagrange_example_car car.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: went 12.0 m, at 3.98 m/s (asked 4.00)
```

Where to look: `launch/car.launch.py`, `scripts/drive_car.py`.

What it shows: `--spawn`, `--traffic`, `/cmd_vel`.
