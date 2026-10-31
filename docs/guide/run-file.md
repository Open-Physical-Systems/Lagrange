# The run file

Your robot is all you need:

```bash
lagrange my_robot.urdf            # URDF, or USD, SDFormat, MJCF
lagrange my_robot.usda world.usda # and a world
```

It starts on an empty ground (or in the world you named), still until you command it. Lagrange is
the low level: your code drives each joint, by position, velocity or effort:

```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: [left_wheel, elbow], velocity: [3.0, .nan], position: [.nan, 0.5]}"
```

For each name it takes the position if given (not NaN), else the velocity, else the effort. Position
and velocity loops are built in (8 Hz, critically damped on the joint's inertia, so no tuning, and a
position loop holding against gravity sags slightly); effort goes straight to the joint, limited by
its effort limit. A joint not yet commanded gets no torque. Road vehicles and multicopters keep
their own controllers: `/cmd_vel` or `/ackermann_cmd`.

To keep a run's settings, put them in one file and run that:

```bash
lagrange run.yaml
```

```yaml
# Only `robot` or `world` is required. Paths are relative to this file.
robot: my_robot.urdf
world: warehouse.usda       # none: an empty ground
spawn: [2.0, 0.0, 0.0, 1.57]   # x, y, z, yaw

simulate:                   # what you leave out runs if the robot has what it needs
  electrical: false         # the ideal torque source: pure physics and navigation testing
  sensors: true

site:
  temperature: 293.15       # K
  air_density: 1.225        # kg/m³: rotors fly
  pressure: 101325          # Pa, with temperature and relative_humidity: the weather runs
  relative_humidity: 0.5
  geo_origin: [47.3977, 8.5456, 488]  # latitude, longitude, height: GNSS and the compass run

physics: mujoco             # or jolt
control: joints             # each joint on /joint_commands; builtin: the robot's own controller
time: { real_time_factor: 1.0, paused: false }
ros: true
ros_topics: topics.yaml     # plugin signals on ROS: writing-a-plugin.md
plugins: [lagrange_tof]
humans: 6,radius=7          # people: tutorial-humans.md
record: run.mcap
```

Keys under `simulate` are `physics`, `sensors`, `electrical`, `thermal`, `weather`, `emi`, `comms`,
`vehicle_dynamics`, `traffic` and `humans`. `false` puts in the domain's substitute. A mistake in the
file stops the start and says which key.

Electrical needs a power system on the robot: an overlay (`overlay: power.yaml`,
[formats.md §9](../design/formats.md#9-importers)) or USD electrical schemas.

A [profile](profiles.md) is the detailed form, for step sizes and fidelity per domain, run with
`lagrange run profile.yaml`.
