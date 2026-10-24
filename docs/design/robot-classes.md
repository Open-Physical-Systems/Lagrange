# Robot classes

Status: draft for review. For each class: required modules, integrations, sample assets, and the
release in which it becomes usable. Package versions are those found on Ubuntu 26.04 with ROS 2
Lyrical on 2026-10-13 (`docs/plan/verification.md` §4).

## 1. Summary

| Class | Release | Required domains | Key integrations | Status of integrations on Lyrical |
|---|---|---|---|---|
| Mobile robot / AMR | v0.2 | rigid_body, sensors (lidar, IMU, encoders) | ros2_control, Nav2, simulation_interfaces, Open-RMF (demo) | ros2_control 6.10.1, Nav2 1.5.1 available; RMF partly (no `rmf-fleet-adapter` apt candidate) |
| Robot arm | v0.3 | rigid_body, electrical, thermal, sensors (camera) | ros2_control, MoveIt 2 | MoveIt 2.15.2 available |
| Drone | v0.4 | rigid_body, rotor_aero, aerodynamics, electrical, sensors (IMU, GNSS, mag, baro) | PX4 and ArduPilot SITL over MAVLink; MAVROS | MAVROS 2.15.1 available; PX4 and ArduPilot ROS 2 guides target Jazzy/Humble (V8) |
| Car | v0.6 | rigid_body, vehicle_dynamics, aerodynamics | ros2_control (Ackermann), OpenDRIVE | available |
| Autonomous car | v0.6 | car + sensors (lidar, camera, radar, GNSS/IMU), traffic, weather | Autoware, OpenSCENARIO, SUMO | Autoware **not** released into Lyrical (V7); SUMO 1.25.0 in apt |
| Huge-world operations (any class) | v0.7 | + streaming, geospatial | 3D Tiles, GeoTIFF, OSM | GDAL 3.12.2, PROJ 9.7.1 in apt |
| Legged, marine, climbing | post-1.0 | — | — | — |

## 2. Common integration layer (all classes, from v0.2)

### 2.1 ROS 2 bridge (`bridges/ros2/`, package `lagrange_ros2`)

An `rclcpp` component inside the server process.

| Interface | Detail |
|---|---|
| `/clock` | `rosgraph_msgs/Clock` every base tick or at a configured rate (default 1 kHz cap). All nodes use `use_sim_time` |
| TF | `/tf`, `/tf_static` from body poses; frame names from prim names or `sensor:frameId` |
| Joint states | `sensor_msgs/JointState` |
| Sensors | `sensor_msgs/LaserScan`, `PointCloud2`, `Image`, `CameraInfo`, `Imu`, `NavSatFix`, `MagneticField`; `nav_msgs/Odometry` for ground truth |
| GPU transport | With `bridges.ros2.gpu_buffers: true`, image and point-cloud payloads are published through `rosidl::Buffer` with the CUDA backend (`ros-lyrical-cuda-buffer`), avoiding a CPU copy for GPU consumers (V5). Off by default |
| Standard simulator API | `simulation_interfaces` services and actions (spawn, delete, get/set entity state, step, reset, simulation state). It is in the Lyrical rosdistro (V6); the exact version and coverage are checked when implementing |
| Signals | Any bus signal can be exposed as a `std_msgs/Float64` (or typed) topic through a mapping file, so electrical and thermal values are visible to ROS tools |
| QoS | Sensor data: best effort, keep last 5. State: reliable. Configurable per topic |
| RMW | Tested with `rmw_fastrtps_cpp` (installed default) and `rmw_zenoh_cpp` 0.10.6 |

Lockstep: with `time.real_time_factor: 0` or when a controller must not miss a cycle, the server
can wait for a named set of subscribers to acknowledge each control period. This is implemented
through ros2_control's own cycle (section 2.2), not through a custom ROS protocol.

### 2.2 ros2_control (`bridges/ros2_control/`, package `lagrange_ros2_control`)

Top-priority integration. A `hardware_interface::SystemInterface` plugin loaded by the user's
`controller_manager`, as with `gz_ros2_control` (3.0.9 on Lyrical) and `mujoco_ros2_control`
(0.1.2 on Lyrical). Those two packages are the behavioural references.

| Item | Design |
|---|---|
| Where the controller manager runs | Inside the server process, stepped by the scheduler at the controller update rate. This gives exact lockstep: read state → update controllers → write commands, once per control period, with no dependence on wall-clock timing |
| State interfaces | `position`, `velocity`, `effort` per joint from `joint/{j}/*`; plus optional `current`, `temperature`, `bus_voltage` from electrical and thermal signals |
| Command interfaces | `effort` → `joint/{j}/torque_command`; `velocity` and `position` → a built-in joint servo (PID with parameters from URDF `<ros2_control>` or the overlay) that produces `torque_command`. With electrical enabled and `elec:controlMode = velocity`, the velocity command goes to the drive model instead |
| URDF | `<ros2_control>` blocks name the plugin `lagrange_ros2_control/LagrangeSystem`. The importer keeps the block so the same URDF drives a real robot by changing the plugin name |
| Encoders | `<param name="encoder_resolution">` on a joint (counts per revolution, after quadrature) makes its `position` state a whole number of counts, rounded down as a counter is, and its `velocity` state the count change over the control period, as a driver computes it (`Encoder.hpp`, `test_encoder`). Without it the states are exact. The sample AMR's wheels have 16384 counts per revolution |
| Encoder effects | When EMI is enabled, `emi/{p}/glitch_probability` corrupts the position state read by controllers |

### 2.3 Scripting and RL

Python bindings (nanobind) for `Scheduler`, `SignalBus` and world queries. A gRPC control API is
planned for v0.2 at the level of start/stop/step/get/set, mirroring `simulation_interfaces`.
Batched environments (`batch: N`) expose signals as arrays; a Gymnasium-style wrapper is a v0.3+
item and is not required for any acceptance criterion before 1.0.

## 3. AMR / mobile robot (v0.2)

| Need | Provided by |
|---|---|
| Differential, Ackermann, mecanum, omni bases | rigid_body + ros2_control controllers (`diff_drive_controller` and others from `ros2_controllers` 6.9.0) |
| 2D and 3D lidar, IMU, wheel encoders | sensors (Tier 0 Embree, Tier 1 compute BVH) |
| Navigation | Nav2 1.5.1 against `/scan`, `/odom`, `/tf`, `/cmd_vel` |
| Fleet demo | Open-RMF with a small warehouse; depends on RMF package availability (R8) |
| Battery behaviour | electrical (v0.3; thin slice proposed for v0.2, Q3): state of charge, sag under load, brownout |
| Docking, lifts, doors | scripted kinematic bodies in the world; RMF door/lift adapters later |

Sample assets: `assets/robots/amr_diffdrive/` (the file in `docs/design/formats.md` §4),
`assets/robots/amr_mecanum/` (v0.3), `assets/worlds/warehouse_small/` (20 m × 30 m, shelves,
a ramp, a narrow aisle), `assets/worlds/empty_flat/`.

Acceptance scenario for v0.2: the sample AMR navigates between four waypoints in the warehouse
with Nav2, using `diff_drive_controller` through `lagrange_ros2_control`, with lidar on Tier 0 and
Tier 1, for 10 simulated minutes without collision, at a real-time factor of at least 1.0 on the
reference desktop.

## 4. Robot arm (v0.3)

| Need | Provided by |
|---|---|
| 6- and 7-DoF serial arms, grippers | rigid_body (MuJoCo first; contact-rich grasping checked on Jolt and PhysX) |
| Joint torque, position and velocity control at 500 Hz–1 kHz | ros2_control + `joint_trajectory_controller` |
| Motion planning | MoveIt 2 2.15.2; the planning scene comes from the same USD through the URDF/SRDF the user already has |
| Motor currents, bus voltage, regeneration during deceleration | electrical |
| Winding temperature and derating during duty cycles | thermal |
| Wrist camera, depth camera | sensors |
| Force–torque sensor | `joint/{j}/constraint_torque` and a body wrench sensor |

Sample assets: `assets/robots/arm6/` (a generic 6-DoF arm with illustrative motor parameters;
no vendor model is redistributed without checking its license), `assets/worlds/workcell_arm6/`.

Acceptance scenario for v0.3: MoveIt 2 plans and executes a pick-and-place cycle; the scope shows
phase currents and bus voltage; repeating the cycle at full speed raises the winding temperature
until derating limits torque and the trajectory controller reports a tolerance violation.
Disabling electrical (ideal torque source) makes the same cycle run without that effect.

## 5. Drone (v0.4)

| Need | Provided by |
|---|---|
| Multirotor airframes | rigid_body + rotor_aero + aerodynamics |
| Motor and ESC dynamics, battery sag affecting thrust | electrical at 50 µs |
| IMU, barometer, magnetometer, GNSS | sensors |
| Autopilot in the loop | `bridges/mavlink/`: PX4 SITL through its simulator MAVLink interface (`HIL_SENSOR`, `HIL_GPS`, `HIL_ACTUATOR_CONTROLS`) with lockstep; ArduPilot SITL through its JSON simulator interface. Both are independent of the ROS distro, which matters because their ROS 2 guides do not target Lyrical yet (V8). Interface details TODO(verify) against the PX4 and ArduPilot versions pinned at v0.4 |
| ROS 2 access | MAVROS 2.15.1 (available), or PX4's uXRCE-DDS bridge with `px4_msgs` built from source |
| Wind and gusts | weather + aerodynamics |

Sample assets: `assets/robots/quad_x500/` (a generic 500 mm quadcopter with illustrative
parameters), `assets/worlds/open_field/`.

Acceptance scenario for v0.4: PX4 SITL in lockstep takes off, flies a square mission in 8 m/s
gusty wind and lands; a low battery scenario shows voltage sag reducing available thrust.

## 6. Car and autonomous car (v0.6)

| Need | Provided by |
|---|---|
| Vehicle model | rigid_body (PhysX or MuJoCo chassis and wheels) + vehicle_dynamics |
| Roads | `io/opendrive/`: OpenDRIVE → USD road meshes, lane geometry tables, traffic-light prims |
| Scenarios | `io/openscenario/`: OpenSCENARIO 1.x runner driving traffic and ego triggers |
| Traffic | traffic (SUMO co-simulation) |
| Sensor suite | sensors: multi-lidar, cameras, radar, GNSS/IMU on Tier 2; weather effects |
| Autonomy stack | `bridges/autoware/`: Autoware's vehicle and sensor interfaces. Blocked on Autoware supporting Lyrical (V7, R7). Fallbacks: run Autoware in a container on its supported distro and connect over the network with a compatible RMW (cross-distro message compatibility must be tested), or demonstrate with a smaller open stack on Lyrical |
| Zero-copy sensor delivery | `rosidl::Buffer` with CUDA backend (V5) |

Sample assets: `assets/robots/sedan/`, `assets/worlds/city_block/` (four-way junction with
lights, from an OpenDRIVE file authored for the project), `assets/worlds/skidpad/`.

Acceptance scenario for v0.6: an OpenSCENARIO cut-in runs in fog with SUMO traffic; the ego
vehicle is driven by a ros2_control Ackermann controller following a recorded path (the Autoware
demonstration is a stretch goal that depends on R7).

## 7. Huge-world operations (v0.7)

See `docs/design/huge-worlds.md`. Sample: a 40 km coastal tile set built by the geospatial
pipeline from open elevation and map data, with the drone survey profile in
`docs/design/formats.md` §5.3. Source data licenses (elevation models, OpenStreetMap's ODbL) are
checked and attributed when the sample is built; ODbL data is not committed to the Apache-2.0
repository, it is fetched by the build script.

## 8. Sample asset policy

- In this repo: only files under 1 MB each (CLAUDE.md §9), authored for the project under
  Apache-2.0 or CC0, with primitive or low-polygon geometry.
- In `lagrange-assets` (separate repo or release bundles): detailed meshes and textures, each with
  a `LICENSE` and a source note. Fetched by `tools/fetch_assets` with SHA-256 verification.
- No third-party robot model is redistributed until its license is recorded in
  `docs/design/dependencies.md`.
- Every sample robot states in its file that its parameters are illustrative unless a datasheet
  source is cited next to the value.
