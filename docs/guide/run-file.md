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

site:                       # also changed while running: lagrange set gravity=1.62 wind_speed=5
  gravity: 9.80665          # m/s², downward
  wind_speed: 3.0           # m/s; acts on links with AeroAPI_1 and on rotors
  wind_from: 270            # degrees, compass: where it blows from
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
sensor_timing:              # how late a sensor's data reaches the robot, in s
  imu: {latency: 0.002, jitter: 0.0005}
water:                      # still water: it floats what enters it, and lidars see it as water
  - {at: [0, 0, 0.5], size: [12, 8], depth: 0.5}    # x, y, surface height
record: run.mcap
```

Keys under `simulate` are `physics`, `sensors`, `electrical`, `thermal`, `weather`, `emi`, `comms`,
`vehicle_dynamics`, `traffic` and `humans`. `false` puts in the domain's substitute. A mistake in the
file stops the start and says which key.

Electrical needs a power system on the robot: an overlay (`overlay: power.yaml`,
[formats.md §9](../design/formats.md#9-importers)) or USD electrical schemas.

## Changing the site while it runs

From a terminal `lagrange set gravity=1.62 wind_speed=5`, or from any ROS tool `ros2 param set
/lagrange_ros2_bridge gravity 1.62`, or in the viewer's Environment workspace. The run goes on from
where it is. A value that brings a domain in (the first air density starts the rotors) rebuilds the
run keeping time, poses and joint positions, with velocities from rest. `wind_speed` alone is enough:
unless given, a wind comes from the north (`wind_from`, degrees), measured 10 m up over open
grassland (the aerodynamics profile's `wind_reference_height`, `roughness_length`). A checkpoint
(`Server::saveState`) holds these values, and restoring it puts them back.

A [profile](profiles.md) is the detailed form, for step sizes and fidelity per domain, run with
`lagrange run profile.yaml`.
