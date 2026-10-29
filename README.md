<p align="center">
  <img src="docs/imgs/repo-logo/lagrange-lockup@2x.png" alt="Lagrange" width="480">
</p>

# Lagrange

Lagrange is an open-source (Apache-2.0) robot simulator for ROS 2 Lyrical Luth on Ubuntu 26.04.
Its aim is multi-physics that can be switched on and off: rigid-body motion, electrical, thermal,
aerodynamics, weather, EMI, comms, vehicle dynamics, traffic and sensors are separate domains on a
unit-checked signal bus, and any of them can be disabled or replaced by ideal, constant or
replayed data. A native Vulkan viewer shows the run; it talks to the simulator only through a
protocol, never by linking it.

**Status: v1.0.0, stable** ([release notes](docs/release/v1.0.0.md)). The USD schemas, the
plugin C ABI, the viewer protocol, profiles and recordings are frozen for 1.x and change only by
addition ([RFC 0007](rfcs/0007-v1-freeze.md)); 1.x is supported on `release/1.x` until ROS 2
Lyrical's end of life in May 2031 ([GOVERNANCE.md](GOVERNANCE.md)). What each model is checked
against: the [validation report](docs/validation/report.md).

<p align="center">
  <img src="docs/imgs/sedan-city.gif" alt="The sample sedan driving round the city block junction among simulated traffic, in the Lagrange viewer's follow camera" width="646">
  <br><b><a href="https://github.com/Open-Physical-Systems/Lagrange/releases/download/v1.0.0/lagrange-sedan-city.mp4">▶ Watch 20 s of the whole viewer (MP4, 6 MB)</a></b>
  <br><sub>The sample sedan on Magic Formula tyres, among IDM traffic in an OpenDRIVE city block
  (<code>lagrange_server --robot assets/robots/sedan/sedan.usda --world assets/worlds/city_block/city_block.usda</code>).</sub>
</p>

<p align="center">
  <img src="docs/imgs/viewer-sedan.png" alt="The Lagrange viewer: scene tree, the viewport with the sedan and its roof lidar's returns, timeline and console" width="820">
</p>

## Real robots in real places

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/imgs/lexus-tokyo.gif" alt="A Lexus RX 450h driving down Koen-dori in Nishi-Shinjuku, Tokyo, among the district's buildings" width="100%">
      <br><sub><b>Lexus RX 450h on Koen-dori, Nishi-Shinjuku, Tokyo</b>, at 10 m/s along the
      street's OpenStreetMap centreline. Body mesh and dimensions from Autoware's sample vehicle
      (Apache-2.0); buildings and roads © OpenStreetMap contributors (ODbL).</sub>
    </td>
    <td align="center" width="50%">
      <img src="docs/imgs/x500-zurich.gif" alt="A Holybro X500 V2 quadcopter flying over Zurich's streets and buildings" width="100%">
      <br><sub><b>Holybro X500 V2 over Zurich</b>, from PX4's default home, flown by
      <code>/cmd_vel</code>. Frame, motor and propeller meshes, mass and inertia from PX4's model
      (BSD-3-Clause); terrain produced using Copernicus WorldDEM-30 (notice above); buildings and
      roads © OpenStreetMap contributors (ODbL).</sub>
    </td>
  </tr>
</table>

<p align="center">
  <img src="docs/imgs/amr-fms-warehouse.png" alt="The AMR_FMS swerve AMR in its 60 by 40 m warehouse" width="520">
  <br><sub><b>The AMR_FMS swerve AMR</b> in its 60 m × 40 m warehouse: four steered wheels on the
  built-in swerve controller (<code>/cmd_vel</code> with sideways speed).</sub>
</p>

```bash
ros2 run lagrange_assets fetch_assets        # the large meshes: pinned upstream commits, SHA-256 checked
tools/ci/build_sample_world.sh --place tokyo # or zurich: worlds from open data (network)
ros2 run lagrange_server lagrange_server --robot assets/robots/lexus_rx450h/lexus_rx450h.urdf \
    --overlay assets/robots/lexus_rx450h/lexus_rx450h.overlay.yaml \
    --world ~/.cache/lagrange/worlds/tokyo_shinjuku_10km/tokyo_shinjuku_10km.usda \
    --geo-origin 35.689568,139.691717,68.5 --spawn -124.86,294.15,0.3,-1.3746
ros2 run lagrange_server lagrange_server --robot assets/robots/holybro_x500_v2/holybro_x500_v2.urdf \
    --overlay assets/robots/holybro_x500_v2/holybro_x500_v2.overlay.yaml \
    --world ~/.cache/lagrange/worlds/zurich_40km/zurich_40km.usda \
    --geo-origin 47.397742,8.545594,535.82 --air-density 1.225
ros2 run lagrange_server lagrange_server --robot assets/robots/amr_fms/amr_fms.urdf \
    --world assets/worlds/amr_fms_warehouse/warehouse.sdf --spawn 26,8,0,3.1416
```

## What it simulates

| Robot class | Since | Sample | Highlights |
|---|---|---|---|
| Mobile robots (AMR) | v0.2 | `amr_diffdrive` in the small warehouse | MuJoCo or Jolt physics, IMU, 2D/3D lidar on CPU or GPU, Nav2, ros2_control |
| Robot arms | v0.3 | `arm6` work cell | MoveIt 2, motor drives, battery and bus at 100 µs, motor heating and derating |
| Drones | v0.4 | `quad_x500` on the open field | Rotor aerodynamics (coefficients or blade element), wind, PX4 and ArduPilot SITL in lockstep |
| Any robot outdoors | v0.5 | `amr_fog.yaml`, the power line | Weather (fog, rain, dust, NanoVDB fields), EMI from power lines and drives, Wi-Fi and CAN links |
| Cars, autonomous cars | v0.6 | `sedan` in the city block | Tyres, suspension, powertrain, OpenDRIVE roads, traffic and pedestrians, OpenSCENARIO, radar, camera sensor model, Autoware interface |
| Any robot, anywhere | v0.7 | the tiled town, the 40 km Zurich world | Worlds streamed in tiles, a floating origin that follows the robot, robots far apart in regions of their own, worlds built from elevation models and OpenStreetMap, terrain as height fields |

<p align="center">
  <img src="docs/imgs/zurich-40km.png" alt="The sample AMR in the 40 km Zurich world: terrain, buildings and roads streamed around it" width="820">
  <br><sub>The 40 km Zurich world, built by <code>tools/ci/build_sample_world.sh</code>. Terrain produced using
  Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018
  provided under COPERNICUS by the European Union and ESA; all rights reserved. Buildings and
  roads © OpenStreetMap contributors (ODbL).</sub>
</p>

Everything runs headless too: from Python (`import lagrange`), from the `lagrange` command, or
recorded to MCAP and replayed.

## Where to read

| Topic | File |
|---|---|
| **User guide: tutorial, concepts, profiles, writing a module, CLI** | [docs/guide/](docs/guide/README.md) |
| Vision, scope and release metrics | [docs/plan/00-overview.md](docs/plan/00-overview.md) |
| Roadmap and progress | [docs/plan/roadmap.md](docs/plan/roadmap.md), [docs/plan/progress.md](docs/plan/progress.md) |
| Release notes | [docs/release/](docs/release/) |
| Architecture | [docs/design/architecture.md](docs/design/architecture.md) |
| Formats, USD schemas, profiles | [docs/design/formats.md](docs/design/formats.md) |
| Viewer and protocol | [docs/design/viewer.md](docs/design/viewer.md), [docs/design/protocol.md](docs/design/protocol.md) |
| Design proposals | [rfcs/](rfcs/) |
| Decisions and open questions | [docs/plan/open-questions.md](docs/plan/open-questions.md) |

## Install

The packages are built as `ros-lyrical-*` debs; until they are published on the ROS buildfarm,
build them yourself from a clone of this repository and install them on any Ubuntu 26.04 machine
with ROS 2 Lyrical (about an hour, with podman; `packaging/README.md`):

```bash
packaging/debs/build_debs.sh ~/lagrange_debs          # every package, in a clean container
sudo apt install ~/lagrange_debs/*.deb
PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install -c packaging/pip-constraints.txt mcap coacd
packaging/test_install.sh ~/lagrange_debs              # optional: the clean-machine test
```

Or build from source:

## Build

Requirements: Ubuntu 26.04 with ROS 2 Lyrical installed.

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ~/lagrange_ws/src && cd ~/lagrange_ws
git clone -b dev https://github.com/Open-Physical-Systems/Lagrange.git src/lagrange

# One dependency (the Python MCAP library) is not packaged for Ubuntu. Register the project's
# extra rosdep keys once, then let rosdep install it with pip.
echo "yaml file://$PWD/src/lagrange/packaging/rosdep/lagrange.yaml" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-lagrange.list
rosdep update
PIP_BREAK_SYSTEM_PACKAGES=1 PIP_CONSTRAINT=$PWD/src/lagrange/packaging/pip-constraints.txt \
  rosdep install --from-paths src --ignore-src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test && colcon test-result --verbose
```

If you prefer not to install pip packages into the system Python, create a virtual environment
with `--system-site-packages`, install `mcap` there, and pass
`-DPython3_EXECUTABLE=<venv>/bin/python` to `colcon build`.

Optional: `lagrange_autoware` builds only with the Autoware messages
([docs/guide/autoware.md](docs/guide/autoware.md)); the PX4 and ArduPilot missions need their
SITL builds ([drone tutorial](docs/guide/tutorial-drone.md)).

Packages:

| Package | Directory | Content |
|---|---|---|
| `lagrange_cmake` | `cmake/` | Shared build options |
| `lagrange_core` | `core/` | Units, signal bus, multirate scheduler, time, random numbers, world state, coordinates, plugin C ABI |
| `lagrange_schemas` | `schemas/` | The OpenUSD applied schemas (electrical, thermal, aero, weather, EMI, comms, sensors, vehicle, road, world) |
| `lagrange_profiles` | `profiles/` | Profile schema and examples |
| `lagrange_proto` | `proto/` | Viewer protocol messages, framing, socket transport, shared memory |
| `lagrange_gpu` | `gpu/` | Vulkan device layer, capability tiers, Slang kernels, the GPU ray caster |
| `lagrange_stage` | `io/stage/` | USD stage → world tables and scene, and back |
| `lagrange_importers` | `io/importers/` | SDFormat and MJCF readers (URDF in `lagrange_stage`) |
| `lagrange_meshes` | `io/meshes/` | glTF, OBJ, STL and Collada import |
| `lagrange_recording` | `io/recording/` | MCAP reading and the replay substitute |
| `lagrange_fields` | `io/fields/` | NanoVDB fields, OpenFOAM wind import |
| `lagrange_opendrive` | `io/opendrive/` | OpenDRIVE reader and road import to USD |
| `lagrange_geospatial` | `io/geospatial/` | Worlds from elevation models, OpenStreetMap and 3D Tiles (`lagrange_build_world`) |
| `lagrange_openscenario` | `io/openscenario/` | OpenSCENARIO 1.x subset runner |
| `lagrange_rigid_body_mujoco`, `_jolt`, `_substitutes` | `modules/rigid_body/` | The rigid-body domain on MuJoCo or Jolt, and its substitutes |
| `lagrange_electrical` | `modules/electrical/` | Batteries, buses, motor drives, FMU host |
| `lagrange_thermal` | `modules/thermal/` | Lumped thermal networks |
| `lagrange_aerodynamics`, `lagrange_rotor_aero` | `modules/` | Link drag, wind and turbulence; rotors |
| `lagrange_weather`, `lagrange_emi`, `lagrange_comms` | `modules/` | Weather, EMI fields, radio and CAN links |
| `lagrange_vehicle_dynamics`, `lagrange_traffic` | `modules/` | Road vehicles; traffic agents and pedestrians |
| `lagrange_sensors` | `modules/sensors/` | IMU, lidar, camera, radar, GNSS, magnetometer, barometer |
| `lagrange_models` | `models/` | Component FMUs |
| `lagrange_ros2`, `lagrange_ros2_control` | `bridges/` | The ROS 2 bridge; the ros2_control hardware plugin |
| `lagrange_mavlink`, `lagrange_autoware` | `bridges/` | PX4 and ArduPilot SITL; the Autoware vehicle interface |
| `lagrange_server` | `server/` | The simulator (`lagrange_server`) and the `lagrange` command |
| `lagrange_viewer` | `viewer/` | The desktop viewer |
| `lagrange_mock_server` | `mock_server/` | Replay server for recordings; a test tool |
| `lagrange_python` | `python/` | Python bindings: run a simulation headless, step it, read signals |
| `lagrange_assets`, `lagrange_demos` | `assets/`, `demos/` | Sample robots and worlds; the Nav2 and MoveIt demos |
| `lagrange_collision_bake`, `lagrange_migrate` | `tools/` | Convex decomposition; schema and profile migrations |
| `lagrange_tutorial`, `lagrange_benchmarks` | `tests/` | The user guide's code; performance benchmarks |
| `*_vendor` | `vendor/` | Pinned OpenUSD, Jolt, FMI Library, Dear ImGui and ImPlot |

## Run the simulator

```bash
source install/setup.bash          # or /opt/ros/lyrical/setup.bash with the debs installed
lagrange                           # the server and the viewer: the sample AMR in the warehouse
lagrange sedan                     # a sample by name: amr, sedan, quad, lexus, x500, swerve
lagrange robot.urdf world.sdf      # your own robot and world (URDF, SDFormat, MJCF or USD)
lagrange --server-only sedan       # headless, like gzserver; `lagrange viewer` is the viewer alone
```

Closing the viewer stops the server. Options before `--` go to the server (`--spawn`,
`--physics jolt`, `--no-ros`, ...; `lagrange_server --help`), after it to the viewer
(`lagrange sedan -- --follow 20`). The robot drives a figure-eight until it gets `/cmd_vel`.
The same samples with the server alone:

```bash
# the sedan in the city block, with traffic (drive it with /ackermann_cmd or /cmd_vel)
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --world assets/worlds/city_block/city_block.usda
# the quadcopter, hovering 2 m up (fly it with /cmd_vel)
ros2 run lagrange_server lagrange_server --robot assets/robots/quad_x500/quad_x500.usda \
    --world assets/worlds/open_field/open_field.usda --air-density 1.225 \
    --geo-origin 47.397742,8.545594,488
# the arm work cell with MoveIt 2
ros2 launch lagrange_demos arm6_workcell.launch.py viewer:=true
```

One tutorial per robot class walks through everything (ROS 2, profiles, recording, substitutes,
the stacks that drive it):
[AMR and Nav2](docs/guide/tutorial-amr.md) ·
[arm and MoveIt 2](docs/guide/tutorial-arm.md) ·
[drone and PX4/ArduPilot](docs/guide/tutorial-drone.md) ·
[weather, EMI and comms](docs/guide/tutorial-environment.md) ·
[cars, scenarios and Autoware](docs/guide/tutorial-cars.md) ·
[huge worlds](docs/guide/tutorial-huge-worlds.md).
The scripts in `tools/ci/` (`tutorial_amr.sh`, `tutorial_drone.sh`, `tutorial_environment.sh`,
`tutorial_cars.sh`, `tutorial_huge_worlds.sh`, and `arm_scenario.sh` for the arm) run them headless and check every step.

To record a clip like the one above: `lagrange_viewer --headless --follow 30,55
--record-frames DIR --record-fps 10 --frames 100` writes PNG frames of a running simulation;
the video is 500 frames at 25 fps, `ffmpeg -framerate 25 -i DIR/frame_%05d.png -c:v libx264
-pix_fmt yuv420p -crf 23 clip.mp4`.

## The viewer and the mock server

The viewer also works without the simulator: the mock server replays recordings and a built-in
synthetic sample. Start the mock server with the sample, then the viewer:

```bash
source install/setup.bash
ros2 run lagrange_mock_server lagrange_mock_server --sample &
ros2 run lagrange_viewer lagrange_viewer
```

Both use the same default Unix socket, so no address is needed on one machine. Drag with the
left mouse button to orbit, the right or middle button to pan, and the wheel to zoom. Panels can
be dragged and docked; the layout is saved in `~/.config/lagrange/viewer/layout.ini`.

The Scope panel plots signals at their full rate (the sample has four at 10 kHz). Use
**+ Signal** to add one; signals with the same unit share a plot. **Freeze** stops the plot
from following time so that you can pan and zoom it. To open signals from the command line:

```bash
ros2 run lagrange_viewer lagrange_viewer \
  --scope motor/left_drive_motor/current_q,bus/dc_48v/voltage,joint/left_wheel_joint/velocity
```

`--sample` serves a built-in synthetic recording (an AMR on a figure-eight path; computed, not
simulated). `--write-sample file.mcap` writes it to disk and `--recording file.mcap` serves a
file.

### Open your own robot

With the viewer and a server running, open a URDF or xacro file in any of these ways:
drag the file onto the window, use **File > Open robot or recording...** (Ctrl+O), or start
the viewer with `--open`:

```bash
ros2 run lagrange_viewer lagrange_viewer --open ~/my_robot/urdf/my_robot.urdf
```

The robot is shown with every joint at zero. Meshes in STL, OBJ and DAE are read, and
`package://` paths are looked up in installed ROS packages and in the folders above the file.
Anything the importer could not use (textures, a mesh it could not find) is listed in the
Timeline panel; a missing mesh is drawn as a magenta box. A `.mcap` recording opens the same
way. To serve a robot directly: `lagrange_mock_server --urdf robot.urdf`.

Other useful commands:

```bash
ros2 run lagrange_server lagrange info                        # version, paths, GPUs and tiers
ros2 run lagrange_gpu lagrange_gpu_info                      # GPUs and their capability tier
ros2 run lagrange_viewer lagrange_viewer --gpu Intel         # pick a GPU by name or index
ros2 run lagrange_viewer lagrange_viewer --headless --screenshot out.png   # no window
ros2 run lagrange_mock_server lagrange_mock_server --check unix:$XDG_RUNTIME_DIR/lagrange/default.sock
```

## Contributing

Development happens on the `dev` branch; `main` holds releases only. See
[CONTRIBUTING.md](CONTRIBUTING.md).

## License

Apache-2.0. See [LICENSE](LICENSE).

Lagrange is a project of Open Physical Systems.
