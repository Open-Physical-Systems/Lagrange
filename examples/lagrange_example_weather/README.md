# Fog on a sensor

The same warehouse with a visibility you choose: `visibility:=8` or `visibility:=1000`. A node reports how far the lidar sees.

```bash
ros2 launch lagrange_example_weather weather.launch.py                         # with the viewer
ros2 launch lagrange_example_weather weather.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: with `visibility:=8` farthest return 3.6 m, 54 % of the rays; with `1000`, 10.2 m and 100 %
```

Where to look: `launch/weather.launch.py`, `scripts/read_range.py`.

What it shows: `--weather`.
