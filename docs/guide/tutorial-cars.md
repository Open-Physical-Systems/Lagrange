# Tutorial: a car in a city block

In this tutorial you look at how the sample sedan and its city are made, watch it drive among
traffic, read its radar and camera, run an OpenSCENARIO cut-in, and drive it yourself over ROS 2
the way an autonomy stack does. It takes about 30 minutes.

`tools/ci/tutorial_cars.sh` runs every step below without the viewer and checks the results (the
Autoware step only when `lagrange_autoware` is built), so the commands here are known to work.

## Before you start

You need Lagrange built as in the [README](../../README.md#build). `rosdep install` brings
`ackermann_msgs`, which the last steps use. In every terminal, from the source tree:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/lagrange_ws/src/lagrange && source ../../install/setup.bash
alias lagrange='ros2 run lagrange_server lagrange'
```

## 1. The sedan in two layers

The sedan starts as an ordinary URDF, `assets/robots/sedan/sedan.urdf`: a chassis, four wheels
on spin joints, two steer joints, a roof lidar, a front camera and an IMU. The importer turns it
into USD:

```bash
ros2 run lagrange_server lagrange_import assets/robots/sedan/sedan.urdf /tmp/sedan_base.usda
```

That file is `sedan_base.usda` in the repository. What makes it a car is the second layer,
`sedan.usda`, which sublayers the first and adds:

- `VehicleAPI_1` on the chassis: an electric powertrain with 2000 N·m at the rear wheels,
  Ackermann steering up to 0.6 rad, brakes with ABS, anti-roll bars;
- `WheelAPI_1` on each spin joint: radius, which wheels steer and drive, the tyre file
  (`assets/tyres/generic_205_55R16.tir`, Magic Formula 5.2) and the suspension;
- a 77 GHz front radar (`RadarAPI_1`) and the front camera's sensor model: rolling shutter,
  auto exposure, a mild barrel lens and vignetting.

All of these are illustrative values, not a product's. The run's profile validates like any
other:

```bash
lagrange validate profiles/examples/sedan_city.yaml
```

## 2. Roads from OpenDRIVE

The city block's roads are an OpenDRIVE file, `assets/worlds/city_block/city_block.xodr`: four
arms meeting at a junction, written by `city_block_xodr.py`. The importer turns its roads into road
surfaces, road marks and one `RoadLaneAPI_1` prim per lane, which traffic and scenarios drive on:

```bash
ros2 run lagrange_opendrive lagrange_opendrive_import \
    assets/worlds/city_block/city_block.xodr /tmp/roads.usda
```

It reports 16 roads and 40 lanes. `city_block.usda` puts those roads together with the buildings.

## 3. The demo drive, traffic and the sensors

Run the sedan in the city, and the viewer in a second terminal:

```bash
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --world assets/worlds/city_block/city_block.usda
ros2 run lagrange_viewer lagrange_viewer --connect unix:$XDG_RUNTIME_DIR/lagrange/default.sock
```

Until it gets its first command, the sedan drives a 5 m circle round the junction's centre at
3 m/s. Eight cars and six pedestrians share the junction. The cars follow their lanes with the
intelligent driver model, stop at the fixed-cycle lights and give way at the junction; the
pedestrians walk the sidewalks. The same run in Python:

```python
import lagrange

sim = lagrange.Simulation(lagrange.sample_sedan(), world=lagrange.sample_city())
sim.step_until(20.0)
sim.read("vehicle/sedan/speed")  # [3.00]
len(sim.read("traffic/agents")) // 8  # 14 agents, 8 numbers each
sim.read("sensor/front_radar/count")  # radar detections, here 48
sim.read("sensor/front_camera/exposure_time")  # what auto exposure chose, about 5 ms
```

The radar's detections (`sensor/front_radar/detections`: range, azimuth, elevation, radial
velocity, cross-section, SNR) are mostly the buildings' walls, one per range cell, and the cars;
over ROS they are a PointCloud2 on `/front_radar/detections`. In the viewer, Overlays → Radar
detections draws them in the 3D view coloured by radial velocity (blue closing, orange moving
away), and the Sensors workspace shows them over range and azimuth. The camera's
`/front_camera/camera_info` carries its lens distortion.

`--domain vehicle_dynamics=off` swaps the tyre and suspension model for a kinematic bicycle,
and `--traffic vehicles=0,pedestrians=0` empties the streets.

## 4. A cut-in scenario

`assets/scenarios/cut_in.xosc` is an OpenSCENARIO file: the ego (the sedan) stands in its lane
while a target car in the next lane passes at 10 m/s, changes into the ego's lane ahead of it
and slows to 6 m/s; the scenario stops after 12 s. With `--scenario` and no `--world`, the scenario's own OpenDRIVE
roads are the world:

```bash
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --scenario assets/scenarios/cut_in.xosc --rtf 0
```

The log ends with `the scenario ended at its stop trigger, t = 12.01 s`. `--rtf 0` runs it as
fast as the machine can. In the viewer the target is the box called `scenario_Target`.

## 5. Drive it over ROS 2

An autonomy stack drives a car by steering angle and speed. The server takes these as
`ackermann_msgs/AckermannDriveStamped` on `/ackermann_cmd` (and speed with turn rate on
`/cmd_vel`). Start it without the demo and without traffic:

```bash
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --world assets/worlds/city_block/city_block.usda --no-demo --traffic vehicles=0,pedestrians=0
```

and steer 0.1 rad at 4 m/s:

```bash
ros2 topic pub -r 20 /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {speed: 4.0, steering_angle: 0.1}}"
```

`ros2 topic echo /odom` shows the car settle at 3.99 m/s, turning at 0.148 rad/s. The bicycle
model's yaw rate is v·tan δ / L = 4 × tan 0.1 / 2.7 = 0.149 rad/s, so the tyres slip very little
at this speed. It stops half a second after the last command.

## 6. Autoware

[Autoware with a simulated car](autoware.md) has the vehicle interface, `lagrange_autoware`,
that turns Autoware's control commands into `/ackermann_cmd` and reports the car's state back.
It needs the Autoware messages, which are not all released for Lyrical yet; that page explains
how to build them and what to run while Autoware itself is not on Lyrical.
