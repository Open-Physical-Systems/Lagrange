# Built-in sensor kinds

The sample robot with the sensors an AMR carries besides its lidars: front and rear bumpers, four cliff sensors, wheel odometry from 16384-count encoders, a safety laser scanner with a protective and a warning field, and torque sensors in the wheel joints (`amr_diffdrive_safety.urdf` in `lagrange_assets`). Lagrange simulates them itself, with no plugin. `config/topics.yaml` puts their signals on ROS 2 topics with `--ros-topics`. A node drives towards a wall, slows when the scanner's warning field sees it and stops when its protective field does, then checks the wheel odometry against the true distance, the cliff sensor against the floor, and that the bumper was never pressed.

```bash
ros2 launch lagrange_example_sensors sensors.launch.py                         # with the viewer
ros2 launch lagrange_example_sensors sensors.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: slowed at 7.25 m and stopped by the safety scanner (nearest return 0.58 m); wheel odometry 8.164 m for 8.114 m driven; cliff sensor 6.0 cm over the floor; bumper not pressed; 9907 wheel torque readings
```

Where to look: `launch/sensors.launch.py`, `config/topics.yaml`, `scripts/check_sensors.py`, and the robot's `<sensor>` blocks at the end of `assets/robots/amr_diffdrive/amr_diffdrive_safety.urdf`.

What it shows: `<sensor type="bumper|cliff|wheel_odometry|safety_scanner|joint_torque">`, `--ros-topics`. Every sensor Lagrange simulates, and how to add, read and break one: `docs/guide/sensors.md`.
