# Autoware with a simulated car

Lagrange's sample sedan can be driven by [Autoware](https://autoware.org) the way a real car is:
through a vehicle interface that takes Autoware's control commands and reports the car's state
back. This page covers that interface (`lagrange_autoware`), how to build it on Lyrical, and
what to do while Autoware itself is not released for Lyrical.

## What the interface does

`lagrange_autoware_interface` sits between Autoware and `lagrange_server`, where a real car's
drive-by-wire gateway would be:

| Autoware topic | Type | Direction | In the simulator |
|---|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/Control` | in | `/ackermann_cmd`: the steering tire angle and the velocity (signed by the gear), acceleration and jerk passed along |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/GearCommand` | in | park and neutral stop the car, reverse backs it up |
| `/control/command/turn_indicators_cmd`, `hazard_lights_cmd` | `autoware_vehicle_msgs/…Command` | in | echoed as their reports (the car has no lamps yet) |
| `/control/control_mode_request` | service `autoware_vehicle_msgs/ControlModeCommand` | in | AUTONOMOUS (the start) forwards the commands; MANUAL stops forwarding them, and the car stops after 0.5 s |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | out | `/odom`'s body-frame twist |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | out | the two steer joints of `/joint_states` as one bicycle-model angle |
| `/vehicle/status/gear_status`, `control_mode` | `autoware_vehicle_msgs/…Report` | out | the gear and mode as last commanded |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | out | ground truth in the `map` frame, standing in for Autoware's localization (`publish_kinematic_state:=false` to run Autoware's own) |

The server takes `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`) for any road vehicle
(`VehicleAPI_1`), beside `/cmd_vel`: its car controller then steers to the commanded angle and
holds the commanded speed with the throttle and brakes.

The topic names follow Autoware's [vehicle interface
design](https://docs.autoware.org/main/design/autoware-architecture-v1/interfaces/components/vehicle-interface/)
and the messages are `autoware_msgs` 1.11.0.

## Building it

`autoware_control_msgs` is released for Lyrical (`ros-lyrical-autoware-control-msgs`), but
`autoware_vehicle_msgs` is not (as of 2026-10-25). Without both, `colcon build` skips
`lagrange_autoware` with a status message and everything else builds as usual. To build it,
put the messages in an underlay workspace:

```bash
mkdir -p ~/autoware_msgs_ws/src && cd ~/autoware_msgs_ws/src
git clone --depth 1 --branch 1.11.0 https://github.com/autowarefoundation/autoware_msgs.git
cd .. && source /opt/ros/lyrical/setup.bash
colcon build --packages-up-to autoware_control_msgs autoware_vehicle_msgs
```

then build Lagrange on top of it with the dependencies switched on:

```bash
source ~/autoware_msgs_ws/install/setup.bash
export LAGRANGE_AUTOWARE=on   # package.xml asks for the Autoware messages only with this
colcon build --symlink-install --packages-up-to lagrange_autoware
```

## Running it

```bash
ros2 launch lagrange_autoware sedan_autoware.launch.py [viewer:=true] [kinematic_state:=false]
```

starts the server with the sedan in the city block (no demo drive, ROS time on `/clock`, the
lidar, IMU and front camera renamed into `/sensing/...`: see `SENSOR_REMAPS` in the launch file,
and check them against your sensor kit) and the interface. Start Autoware separately with its
own vehicle and sensor-kit descriptions (wheelbase 2.7 m, track 1.56 m, wheel radius 0.316 m,
maximum steering 0.6 rad at the inner wheel for the sample) and `use_sim_time:=true`.

## While Autoware is not on Lyrical

Autoware's own packages are not released for Lyrical yet. Until they are:

- **Stand-in** (what Lagrange tests): `test_autoware_interface` drives the interface from a
  stand-in node that publishes what Autoware's control and the simulator publish, and checks the
  Ackermann command, the reports and the control-mode service. By hand, the same stand-in
  publishing a steering angle of 0.1 rad at 4 m/s made the sedan hold 3.99 m/s at a yaw rate of
  0.148 rad/s, the bicycle model's v·tan δ / L.
- **Autoware in its own container** on the distribution it is released for, talking to the
  simulator over DDS on the same domain. The message definitions are the same `autoware_msgs`
  release on both sides; whether type hashes match across ROS distributions is TODO(verify) for
  the Autoware release you use.
- **Without Autoware**, any Ackermann stack can publish `/ackermann_cmd` directly.
