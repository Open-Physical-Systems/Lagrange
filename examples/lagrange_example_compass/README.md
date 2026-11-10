# A compass next to the battery leads

The sample quadcopter with a battery under its frame and a lead from it to each motor, carrying the motor's bus current, its return 5 mm above (`--emi-motor-leads 0.005,-0.06,0,-0.06`, RFC 0013). It takes off and hovers; a node reads on ROS 2 how far its compass's heading is off and the disturbance's share of the Earth's field, the percentage ArduPilot's compassmot reports (under 30 % fine, over 60 % move the compass).

```bash
ros2 launch lagrange_example_compass compass.launch.py
ros2 launch lagrange_example_compass compass.launch.py viewer:=false once:=true
```

Where to look: `launch/compass.launch.py` (the leads), `config/topics.yaml`, `scripts/read_compass.py`. With the viewer, the Environment workspace slices |B| through the quadcopter and lists the effects under Overlays → Coupled into.

What it shows: `/compass/heading_error` (rad), `/compass/interference` (1), `/compass/lead_current` (A), from `sensor/magnetometer/heading_error`, `sensor/magnetometer/interference` and `motor/rotor_1_motor/bus_current`. More: docs/physics/emi-and-radio.md.
