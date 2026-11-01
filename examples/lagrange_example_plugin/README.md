# Your own module in C

A simulation module of your own, compiled here and loaded by name, which reads the robot's pose and writes a signal; that signal on a ROS topic. It links nothing of Lagrange's.

```bash
ros2 launch lagrange_example_plugin plugin.launch.py                         # with the viewer
ros2 launch lagrange_example_plugin plugin.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: 300 samples from the plugin: 0.003 m, then 1.250 m
```

Where to look: `src/distance_sensor.c`, `CMakeLists.txt`, `config/topics.yaml`, `launch/plugin.launch.py`.

What it shows: `--plugin`, `--ros-topics`, the C ABI (`lagrange_plugin.h`).
