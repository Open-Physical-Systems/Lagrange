# Examples

Each folder is a ROS 2 package that runs Lagrange through a launch file, with an ordinary node beside it. Copy one into your own workspace to start from it.

Build them with the rest of the workspace, then run one:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch lagrange_example_diff_drive drive.launch.py
```

| Package | What it is | What it shows |
|---|---|---|
| [`lagrange_example_diff_drive`](lagrange_example_diff_drive) | Your own wheeled robot | `--robot FILE`, `--world none`, `--no-demo`, `/cmd_vel`, `/odom` |
| [`lagrange_example_arm`](lagrange_example_arm) | An arm from a URDF | `--joint-control`, `/joint_commands`, `/joint_states` |
| [`lagrange_example_power`](lagrange_example_power) | A power system from a schematic | `<lagrange><schematic>`, `--ros-topics`, `bus/dc/voltage` |
| [`lagrange_example_lidar`](lagrange_example_lidar) | A simulated lidar | `/scan`, `lagrange_assets` |
| [`lagrange_example_sensors`](lagrange_example_sensors) | Bumpers, cliff sensors, wheel odometry, a safety scanner | Built-in sensor kinds in a URDF, `--ros-topics` |
| [`lagrange_example_camera`](lagrange_example_camera) | A simulated camera | Colour, depth and instance images on ROS 2 |
| [`lagrange_example_compass`](lagrange_example_compass) | A compass next to the battery leads | What a quadcopter's own wiring does to its heading, on ROS 2 |
| [`lagrange_example_heat`](lagrange_example_heat) | Heat inside the parts | LEDs on a board on a heat sink: where the heat goes, on ROS 2 and in the viewer |
| [`lagrange_example_weather`](lagrange_example_weather) | Fog on a sensor | `--weather` |
| [`lagrange_example_quadcopter`](lagrange_example_quadcopter) | A quadcopter | `--air-density`, `--geo-origin`, `/cmd_vel` |
| [`lagrange_example_holybro`](lagrange_example_holybro) | Your own drone (Holybro X500 V2) by keyboard | `--overlay`, `--follow`, `teleop_twist_keyboard` in its own window |
| [`lagrange_example_car`](lagrange_example_car) | A car in the town | `--spawn`, `--traffic`, `/cmd_vel` |
| [`lagrange_example_spawn`](lagrange_example_spawn) | Adding and removing robots | `--robot none`, `--humans`, `/spawn_entity`, `/get_entities`, `/delete_entity` |
| [`lagrange_example_ros2_control`](lagrange_example_ros2_control) | An arm through ros2_control | `--ros2-control`, `controller_manager` spawner, `FollowJointTrajectory` |
| [`lagrange_example_quadruped`](lagrange_example_quadruped) | A trotting quadruped | `--joint-loop-hz`, position commands from a node |
| [`lagrange_example_wind_tunnel`](lagrange_example_wind_tunnel) | A wind tunnel, as in CFD | `lagrange tunnel`: drag, lift, streamlines, surface pressure, ParaView |
| [`lagrange_example_rl`](lagrange_example_rl) | Learning to control an arm | `lagrange.envs`: Gymnasium's interface, parallel processes, randomized gravity |
| [`lagrange_example_plugin`](lagrange_example_plugin) | Your own module in C | `--plugin`, `--ros-topics`, the C ABI (`lagrange_plugin.h`) |

## How they are made

Each launch file does the same three things:

1. Runs Lagrange as a node: `package="lagrange_server", executable="lagrange_server"`, with its command-line options as `arguments`.
2. Runs your node, `on_exit=Shutdown()`.
3. Runs the viewer as a plain process unless `viewer:=false`. It finds the server on the socket given by `--listen`.

Each takes two launch arguments: `viewer` (default `true`) and `once` (default `false`). With `once:=true` the launch ends when the node has its result; without it the run stays up to look at. Every node logs one line starting `RESULT:`.

## Checking them

```bash
tools/ci/check_examples.py              # all, without a viewer
tools/ci/check_examples.py arm power    # some
```

It runs each launch file and fails when one logs no result or does not end in time. `colcon test --packages-select lagrange_examples_check` runs the same. A result is only logged when the example did its job, but the check does not compare numbers.

The `lagrange_example_*` packages are not released as binary packages: they are for reading and copying.
