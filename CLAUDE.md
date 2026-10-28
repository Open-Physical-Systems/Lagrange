# CLAUDE.md

Guidance for Claude Code working in this repository. Read this fully before any task.
Detailed design lives in `docs/design/` and `rfcs/`. This file holds the rules, invariants,
and canonical reference examples (§11–§12) that apply to every session. The examples are the
agreed starting point for interfaces and config files; extend them, don't contradict them
without an RFC.

> Project name: **Lagrange** (organisation: Open Physical Systems). Identifiers use `lagrange` / `LAGRANGE_`.

## 1. What this project is

An open-source (Apache-2.0), GPU-accelerated robot simulator in the spirit of Gazebo, with:

- **Toggleable multi-physics domains**: rigid-body physics, electrical (voltage/current),
  thermal, aerodynamics/wind, weather (rain, fog, dust), EMI, radiation, sensors.
  Any domain can be enabled, disabled, or replaced by a substitute.
- **Robot classes**: drones, cars, autonomous cars, mobile robots/AMRs, robot arms
  (later: legged, marine, climbing).
- **Huge worlds**: double-precision world coordinates, floating origin, tile streaming,
  geospatial import.
- **Native GPU desktop viewer**: a real desktop window drawn with Vulkan. No browser, no web
  engine, no Electron/Tauri, no JavaScript/Node toolchain anywhere in the product.
- **Complete and usable** for end users: packaged installs, docs, sample robots/worlds,
  working ROS 2 integration on first run.
- **Single target platform for now: ROS 2 Lyrical Luth on Ubuntu 26.04 (Resolute)** — see §1a.
  The architecture keeps portability seams (HAL, Slang, C ABI) so other platforms can be added
  later without a rewrite, but nothing outside §1a is built, tested, or supported yet.

## 1a. Target platform (binding)

| Item | Target |
|---|---|
| OS | Ubuntu 26.04 LTS (Resolute Raccoon) |
| Architectures | amd64 (primary, GPU CI); arm64 (build + CPU tests) |
| ROS | ROS 2 Lyrical Luth (LTS, supported until May 2031), `ros-lyrical-*` packages |
| C++ standard | C++20, matching Lyrical's language baseline |
| Build | `ament_cmake` packages built with `colcon`; system deps via `rosdep` / apt |
| Default RMW | `rmw_fastrtps_cpp` (Lyrical default); also test `rmw_zenoh_cpp` |
| GPU | NVIDIA with Vulkan (+ optional CUDA fast path). AMD/Intel Vulkan best-effort |
| Python | Ubuntu 26.04 system Python (the one Lyrical's rclpy is built against) |
| Viewer | Native C++20 app: Vulkan renderer + Dear ImGui (docking branch); window via SDL3 (RFC 0006, accepted) |
| Comparison baseline | Gazebo Jetty (gz-sim 10), the Gazebo release paired with Lyrical |

Consequences:
- Don't add Windows, macOS, Metal, WebGPU, browser, or mobile code paths. Don't add
  `#ifdef _WIN32` / `__APPLE__` branches. Keep platform-specific code inside `gpu/hal/` and
  `core/platform/` so it can be extended later.
- `ament_target_dependencies` is removed in the ament_cmake shipped with Lyrical. Use
  `target_link_libraries` with exported CMake targets.
- Dependencies not packaged for Ubuntu 26.04 / rosdep (likely OpenUSD, PhysX 5, Slang;
  TODO(verify)) are built through vendor packages (`*_vendor`, ExternalProject) pinned to an
  exact version. Don't use vcpkg alongside rosdep.
- Jetson: arm64 ROS packages exist, but NVIDIA JetPack's Ubuntu base may lag 26.04.
  TODO(verify) before promising Jetson support.
- The viewer must run on both Wayland (Ubuntu's default session) and X11/XWayland.

## 2. Current phase

**v0.2 (simulation core + AMR), v0.3 (arms, electrical, thermal), v0.4 (drones), v0.5
(weather, EMI, comms), v0.6 (cars, autonomous cars) and v0.7 (huge worlds) are released** as
`v0.2.0` to `v0.7.0` (v0.2 to v0.6 on 2026-10-25, v0.7 on 2026-10-29; `docs/release/`; v0.1.0 on
2026-10-14). Status and owed items: `docs/plan/progress.md`. Next by the roadmap: **v1.0
stability** (`docs/plan/roadmap.md` §9).

- The protocol stays v1; the v0.1 viewer connects to the C++ server unchanged.
- RFC 0001 (module interface), 0002 (USD schemas) and 0005 (profile format) are accepted; work
  that depends on a still-Draft RFC waits until the owner accepts it.
- Replay is in the C++ server (`--replay`, E8); the Python mock server is a test tool (Q15).
- Update this section when the phase changes.

Roadmap: v0.1 UI/protocol/mock → v0.2 sim core + AMR (ROS 2, ros2_control, Nav2) →
v0.3 arms + electrical (MoveIt 2) → v0.4 drones (PX4/ArduPilot SITL) → v0.5 weather + EMI →
v0.6 cars + autonomous cars (OpenDRIVE, OpenSCENARIO, Autoware, SUMO) → v0.7 huge worlds →
v1.0 stable schemas + plugin ABI, released as `ros-lyrical-*` debs on Ubuntu 26.04.
Other platforms: post-1.0, only through an RFC.

## 3. Architecture invariants (do not violate without an accepted RFC)

1. **Modules communicate only through the signal bus.** No module includes or calls another
   module directly. This is what makes enable/disable work.
2. **Every module declares inputs/outputs as `SignalSpec` with units.** Units are checked at
   wiring time; a unit mismatch must fail at startup, never silently convert.
3. **Every domain has substitutes** (ideal / constant / replay-from-MCAP) so disabling it
   never breaks dependents.
4. **The scheduler is a multirate co-simulation master.** Each module has its own step size
   (e.g. electrical ~100 µs, physics ~1 ms, thermal ~100 ms). Disabled modules cost zero.
5. **USD is for authoring, not per-step access.** `core/stage/` converts the USD stage into
   flat SoA buffers in `core/world/` at load. Never read USD attributes inside `step()`.
6. **World positions are `double`.** Physics/rendering run in local frames (floating origin).
   Never store absolute world positions as `float`.
7. **A deterministic CPU reference mode must always exist** and stay bit-reproducible on a
   given platform. GPU paths may be non-deterministic; CPU reference may not.
8. **GPU kernels are written once in Slang**, compiled for now to SPIR-V (Vulkan) and CUDA only.
   Backend-specific code lives only in `gpu/hal/`. Optional CUDA fast paths sit behind the same
   interface. (Metal/WGSL backends are a later RFC; don't write kernels that preclude them.)
9. **Capability tiers, all on Ubuntu 26.04**: Tier 0 CPU-only (GPU-less CI runners, servers),
   Tier 1 raster GPU (no hardware RT, e.g. integrated or older GPUs; compute-shader BVH lidar),
   Tier 2 full (hardware RT + batched GPU physics, NVIDIA RTX). Every feature defines its
   behavior on each tier, even if "unsupported".
10. **Third-party plugins use the versioned C ABI** in `core/plugin_abi/`, never the C++ API.
11. **Schemas are versioned** (`ElectricalMotorAPI_v1`). Any schema change needs a migration
    in `tools/` and core-maintainer review.
12. **Integrate before building**: use existing engines (PhysX 5, MuJoCo, Jolt, Embree,
    ngspice offline, OpenFOAM offline). Custom solvers only where they are the project's wedge.
13. **The viewer is a native process with no web technology.** It renders through the same
    Vulkan HAL (`gpu/hal/`) and Slang shaders as the sensor simulation, so a camera-sensor view
    and the 3D viewport are drawn by the same code. Never add a browser engine, webview,
    Electron, Tauri, or Node dependency.
14. **The viewer talks to the sim only through the protocol** (`proto/`), even on the same
    machine. It never links simulation modules. Large payloads (point clouds, images, field
    volumes) may use a shared-memory fast path when local, negotiated through the protocol.

## 4. Format stack (one home per kind of data)

| Data | Format |
|---|---|
| Robot + world, component placement, electrical topology, weather/EMI sources | OpenUSD + custom schemas (`schemas/`) |
| Component behavior (battery, motor/drive, thermal network) | FMU (FMI 3.0), authored in Modelica (`models/`) |
| Volumetric fields (fog, dust, wind, EMI, dose) | OpenVDB, NanoVDB on GPU, referenced from USD |
| Simulation profile (domains on/off, fidelity, dt, substitutes) | YAML validated by `profiles/schema.json` |
| Recordings / replay | MCAP |
| Mesh/material import | glTF 2.0, OBJ, STL, DAE (via importers → USD); never a runtime format |
| Collision geometry | Separate convex decompositions (V-HACD/CoACD), never render meshes |
| UI↔server protocol | Protobuf (proto3), `proto/`, package `lagrange.viewer.v1`. Transport: length-prefixed frames over a Unix socket (local) or TCP (remote), plus local shared memory (`memfd`). Decided in RFC 0003. Zenoh deferred. Not WebSocket/WebRTC |

Importers (URDF/xacro, SDFormat, MJCF → USD) are mandatory; users must not rewrite robots.

## 5. Languages

| Layer | Language |
|---|---|
| Core, scheduler, adapters, modules | C++20 (Lyrical's baseline; no C++23-only features) |
| GPU kernels | Slang |
| Domain models | Modelica → FMU |
| Bindings, scripting, tests, mock server (initially) | Python via nanobind |
| Viewer (native desktop app) | C++20: Vulkan (via `gpu/hal/`), Dear ImGui docking branch, ImPlot (scopes), imgui-node-editor or equivalent (schematic), SDL3 or GLFW (window/input) |

## 6. Repository layout

```
lagrange/
├── CMakeLists.txt
├── CMakePresets.json           # dev, release, asan, tsan, ci-*
├── colcon.meta / colcon.defaults  # colcon build settings
├── vendor/                     # *_vendor packages for deps not in rosdep (OpenUSD, PhysX, Slang…)
├── cmake/                      # shared CMake modules (no cross-platform toolchains yet)
│   # every top-level C++ dir below is one or more ament_cmake packages with a package.xml
│
├── schemas/                    # USD custom schemas — the data model (package `lagrange_schemas`)
│   ├── electrical/schema.usda      # ElectricalMotorAPI, BatteryAPI, PortAPI...
│   ├── thermal/  aero/  weather/  emi/  radiation/  sensor_material/
│   └── generated/              # usdGenSchema output (C++ + Python) — never edit by hand
├── profiles/
│   ├── schema.json             # JSON Schema for YAML sim profiles
│   └── examples/               # validated example profiles (see §11.1); package `lagrange_profiles`
├── proto/                      # UI↔server protocol (Protobuf/FlatBuffers)
│
├── core/                       # no GPU, no physics — pure framework
│   ├── module/                 # IModule interface, registry, plugin loading
│   ├── scheduler/              # multirate co-sim master, dependency graph
│   ├── signals/                # typed signal bus (torque, current, temp...)
│   ├── time/                   # sim clock, lockstep, real-time factor
│   ├── random/                 # counter-based RNG (Philox 4x32-10) for deterministic noise
│   ├── world/                  # runtime world state (flat SoA arrays)
│   │   ├── coords/             # double-precision world, floating origin, WGS84↔ENU
│   │   ├── streaming/          # tile loading, LOD, USD payload management
│   │   └── regions/            # active-region management, future distributed handoff
│   ├── stage/                  # (planned here; lives in io/stage, since core/ is one colcon package)
│   └── plugin_abi/             # versioned C ABI for third-party plugins
│
├── gpu/
│   ├── hal/                    # device abstraction: Vulkan + CUDA now (Metal later, by RFC)
│   ├── kernels/                # .slang files
│   │   ├── lidar/  camera/  radar/
│   │   ├── weather/            # fog/rain/dust scattering
│   │   └── fields/             # EMI, wind, dose sampling from NanoVDB
│   └── capability/             # tier detection (CPU / raster / full RT)
│
├── modules/                    # each = one toggleable domain, built as a plugin
│   ├── rigid_body/
│   │   ├── physx_backend/  mujoco_backend/  substitutes/
│   ├── electrical/
│   │   ├── network/            # builds circuit graph from USD ports
│   │   ├── fmu_host/           # FMI 3.0 co-sim
│   │   └── substitutes/        # ideal torque source, replay
│   ├── thermal/  aerodynamics/  weather/  emi/  radiation/
│   ├── vehicle_dynamics/       # tires (Pacejka), suspension, powertrain, brakes
│   ├── rotor_aero/             # thrust/torque, BEM, ground effect
│   ├── hydrodynamics/          # marine (Fossen model)
│   ├── traffic/                # pedestrians, vehicles, traffic lights, SUMO co-sim
│   ├── comms/                  # Wi-Fi/radio link quality, coupled with EMI
│   └── sensors/                # lidar, camera, imu, gnss, magnetometer...
│
├── io/
│   ├── stage/                  # `lagrange_stage`: USD stage → world tables and scene, and back
│   ├── importers/              # urdf, xacro, sdformat, mjcf → usd
│   ├── recording/              # MCAP writer/reader, replay substitutes
│   ├── fields/                 # OpenVDB loading, offline CFD import
│   ├── geospatial/             # 3D Tiles, GeoTIFF/DEM, OSM import
│   ├── opendrive/              # road network → USD
│   └── openscenario/           # scenario runner
│
├── bridges/
│   ├── ros2/                   # rclcpp node: /clock, topics, services
│   ├── ros2_control/           # simulated SystemInterface plugin
│   ├── mavlink/                # PX4 / ArduPilot SITL
│   ├── autoware/               # sensor/vehicle interface for Autoware
│   ├── grpc/                   # control API for any language
│   └── zenoh/
│
├── server/                     # headless sim server; serves the protocol to viewers
├── mock_server/                # v0.1 mock/replay server (test tool; replay is lagrange_server --replay)
├── demos/                      # runnable demos (`lagrange_demos`): launch files, Nav2 config, maps
├── python/                     # nanobind bindings, `pip install lagrange`
├── viewer/                     # native C++ app (ament_cmake package `lagrange_viewer`)
│   ├── app/                    # main(), window (SDL3/GLFW), Vulkan swapchain, frame loop
│   │   ├── theme/              # design tokens as ImGui/ImPlot style, font loading
│   │   └── widgets/            # shared widgets of the design system (chips, pills, switch…)
│   ├── render/                 # viewport renderer on gpu/hal: meshes, gizmos, grid, overlays
│   ├── panels/                 # ImGui panels: scene tree, inspector, domains, timeline,
│   │                           #   scopes (ImPlot), schematic, fields, console, ROS 2 topics
│   ├── layout/                 # docking layouts, saved presets ("Electrical debugging"…)
│   ├── client/                 # protocol client, local shared-memory fast path
│   └── assets/                 # fonts, icons (small, license-checked)
├── models/                     # Modelica sources + prebuilt FMUs (battery, BLDC...)
├── assets/                     # small test assets only (<1 MB each)
│   ├── robots/                 # sample: quadcopter, sedan, AMR, 6-DoF arm, ...
│   └── worlds/                 # warehouse, city block, open terrain, bridge...
├── tests/                      # unit, integration, determinism, sim-vs-real validation
├── tools/                      # schema migration, profiling, asset converters
├── packaging/                  # bloom/ROS buildfarm release, docker (ubuntu:resolute), pip wheel
├── docs/                       # design/, plan/, docs site source, tutorials per robot class
├── rfcs/                       # numbered design proposals: 0001-module-interface.md ...
└── .github/                    # CI, CODEOWNERS, templates, labels, renovate
```

Each module directory contains its backend(s) and a `substitutes/` subdirectory. Large sample
robots/worlds live in a separate `lagrange-assets` repo or release bundles fetched with checksum
verification, not in this repo.

## 7. Commands

<!-- Fill in once the scaffold exists. Do not guess commands that are not in the repo. -->
- Environment: `source /opt/ros/lyrical/setup.bash`
- Deps: `rosdep install --from-paths . --ignore-src -y`
- Build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` (TBD)
- Sanitizer build: `colcon build --build-base build-asan --install-base install-asan --mixin asan` (TBD; verify mixin availability)
- C++ tests: `colcon test && colcon test-result --verbose` (TBD)
- Python: `ruff check . && ruff format --check . && mypy python/ && pytest` (TBD)
- Python API: `import lagrange; sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world()); sim.step_until(2.0); sim.read("body/base_link/pose")` (package `lagrange_python`, nanobind; headless, no ROS; also `signals()`, `bodies()`, `joints()`, `state_hash()`, `profile=` and `lidar=`)
- Simulator: `ros2 run lagrange_server lagrange_server [--robot FILE] [--world FILE|none] [--overlay FILE.yaml|none] [--listen ENDPOINT] [--spawn X,Y[,Z[,YAW]]] [--lidar auto|cpu|gpu] [--gpu DEVICE] [--physics mujoco|jolt] [--session FILE.usda] [--rtf X] [--paused] [--no-ros] [--ros2-control PARAMS.yaml] [--ambient-temperature K] [--air-density KG_M3] [--geo-origin LAT,LON,HEIGHT] [--weather temperature=K,pressure=PA,relative_humidity=X[,visibility=M,rain_rate=MPS,...]] [--mavlink px4[:PORT]|ardupilot[:PORT]] [--domain rigid_body|electrical|thermal|weather|emi|comms|vehicle_dynamics|traffic|sensors=off] [--link-topics FILE.yaml] [--traffic vehicles=N,pedestrians=M,seed=S] [--scenario FILE.xosc [--scenario-ego NAME]] [--cmd-vel-unstamped] [--no-demo] [--record FILE.mcap] [--export-map FILE.yaml]` (FILE: URDF, SDFormat, MJCF or USD; MuJoCo physics (or Jolt with `--physics jolt` or a profile's `backend: jolt`), the electrical network (battery, bus, motor drives at 100 µs, for robots with motor prims), the thermal network (100 ms, for thermal prims, only when the site ambient temperature is given by the profile or `--ambient-temperature`), the rotors (`RotorAPI_1` joints: coefficient thrust and torque, ground effect, 1 ms, only when the site air density is given by the profile's `rotor_aero.air_density` or `--air-density`) with a built-in multicopter controller that hovers 2 m above the start when nothing else flies it, the weather (50 ms, `env/*`: air density, extinction from fog, rain and dust, wet friction, lens contamination; only when the site's temperature, pressure and humidity are given by `--weather`, a WeatherAPI_1 prim or the profile; the lidars then lose range and see fog and rain clutter, the cameras a fog veil; example `profiles/examples/amr_fog.yaml`; NanoVDB fields with `--weather fog_field=FILE.nvdb,wind_field=FILE.nvdb` or the profile's weather keys, samples in `assets/fields/`, made by `ros2 run lagrange_fields lagrange_make_fields fog_bank|wind_shear OUT.nvdb`), and IMU, lidar, camera (with `CameraSensorAPI_1`'s sensor model when asked: lens distortion, vignetting, rolling shutter, exposure and noise, auto exposure), radar (`RadarAPI_1`: detections by ray launching, the sedan's `front_radar`), GNSS, magnetometer (World Magnetic Model 2025) and barometer (ICAO standard atmosphere) on the scheduler (the last three only when the site's geo origin is given by the profile's `world.geo_origin` or `--geo-origin`), ROS 2 bridge publishing /clock /tf /odom /joint_states /scan /points /imu (per GNSS, magnetometer and barometer /<name>/fix NavSatFix, /<name>/field MagneticField, /<name>/pressure FluidPressure and /<name>/temperature, with covariances from their noise; and per camera /<name>/image_raw, /<name>/depth/image_raw and their camera_info, in <name>_optical; per radar /<name>/detections PointCloud2 with x, y, z, velocity, rcs, snr) and taking /cmd_vel as TwistStamped, plus the `simulation_interfaces` services: get/set_simulation_state, step_simulation, reset_simulation, get_entities, get_entity_state, ...; default: the sample AMR with its power-system overlay in the sample warehouse on the default socket; `--export-map` writes a Nav2 occupancy map of the fixed scenery and exits)
- Scene editing (E6): the inspector's editable numbers (schema values, body mass) go to the server as `SetAttribute` on Apply; the server checks them as the loader would, rebuilds the run and carries its state over, so the edit applies from the next step; with `save_to_session_layer` it writes the edited model as a USD layer (`--session`, default `$XDG_STATE_HOME/lagrange/<robot>.session.usda`, run it with `--robot FILE --world none`). `SetTransform` is refused (not supported yet)
- ros2_control: `lagrange_server --ros2-control PARAMS.yaml` runs a controller manager (`/controller_manager`) in lockstep with `lagrange_ros2_control/LagrangeSystem` as hardware; spawn controllers with `ros2 run controller_manager spawner NAME --param-file PARAMS.yaml`; `/odom`, `/joint_states` and `/cmd_vel` then belong to the controllers, and ground truth stays on `/ground_truth/odom`; a joint's `<param name="encoder_resolution">` (counts per revolution) quantises its position and velocity states like a real encoder
- CLI: `ros2 run lagrange_server lagrange run PROFILE | validate PROFILE... | graph [PROFILE] | determinism [PROFILE] [--seconds N] | info [--version]` (`determinism` runs twice without ROS and with the CPU lidar and compares state hashes every 100 ms; exit 1 on a difference) (a profile's `stage` names the URDF robot unless `--robot` is given; options as `lagrange_server`; `docs/guide/cli.md`; the example for the sample AMR is `profiles/examples/amr_sample_urdf.yaml`)
- User guide and tutorial: `docs/guide/` (tutorial, concepts, profiles, writing a module, CLI); `tools/ci/tutorial_amr.sh` runs the AMR tutorial headless and checks each step (Nav2 step only when Nav2 is installed); the module chapter's code is package `lagrange_tutorial` in `tests/tutorial/`
- Record and replay: `lagrange_server --record run.mcap` writes the run until the server stops or is reset (signals at full rate, about 2.8 MB/s for the sample AMR with its 10 kHz electrical signals; arrays over 1024 components left out); `lagrange_server --replay run.mcap` replays it to the viewer (every domain a replay substitute, the scene from the recording, seeking by restoring checkpoints saved every second; no ROS), and the viewer's Open recording does the same on a running server; the Python `lagrange_mock_server --recording` is a test tool only (Q15)
- Bench mode: `lagrange_server --bench run.mcap` (or a profile's `rigid_body: {enabled: false, substitute: replay, source: run.mcap}`) replaces physics by the recording's joint and body motion, open loop, while electrical, thermal and sensors run live (`lagrange_recording`'s `ReplaySubstitute`)
- Import to USD: `ros2 run lagrange_server lagrange_import robot.urdf|robot.sdf|robot.xml robot.usda [--overlay FILE.yaml]` (URDF, SDFormat or MJCF, told apart by the root element; writes what the server reads back; the sample `.usda` assets are made this way and `test_usd_assets` keeps them in step with the URDFs; `lagrange_importers` holds the SDFormat and MJCF readers, and the sample AMR also ships as `.sdf` and MJCF `.xml`; an overlay file (formats.md §9) adds motors, battery and bus, as `amr_diffdrive.overlay.yaml` does for the sample, whose hand-written equivalent is `amr_diffdrive_powered.usda`)
- Migrations (formats.md §10): `ros2 run lagrange_migrate migrate_schema FILE.usda --in-place|-o OUT|--check` and `ros2 run lagrange_migrate migrate_profile FILE.yaml --in-place|-o OUT|--check [--schema profiles/schema.json]` (one function per schema family and version step in `tools/migrate/lagrange_migrate/`; profiles without `schema_version` are version 0)
- Collision bake (B8): `ros2 run lagrange_collision_bake collision_bake IN.usda -o OUT.usda [--threshold 0.05] [--preprocess auto|on|off] [--seed N] [--report FILE.json]` decomposes every mesh collider that is not already a convex hull with CoACD into `convexHull` meshes under `<mesh>_collision` (the mesh stays drawn); the loader reads convex `convexHull` meshes as hull colliders (MuJoCo, Jolt, both lidar tiers). Sample: `assets/worlds/rack_sample/rack.usda` and `rack_baked.usda`
- Electrical export: `ros2 run lagrange_server lagrange export spice|modelica [PROFILE] [options] [--out FILE]` writes the robot's power system as an ngspice netlist or a Modelica model (drives as averaged power sinks; electrical.md §7); `test_export` runs a netlist through ngspice when installed
- Quadcopter (v0.4): `ros2 run lagrange_server lagrange_server --robot assets/robots/quad_x500/quad_x500.usda --world assets/worlds/open_field/open_field.usda --air-density 1.225 --geo-origin 47.397742,8.545594,488` (or `--world none`; the open field is a 50 m field with a landing pad, trees and a shed, where the PX4 and ArduPilot missions fly) (or `lagrange run profiles/examples/quad_x500_hover.yaml --robot ... --world none`): the sample 500 mm quadcopter (URDF + `quad_x500.overlay.yaml` for rotors, motors and a 4S battery; the `.usda` is both, made by `lagrange_import --overlay`) takes off and hovers, and with ROS `/cmd_vel` flies it (forward, left, up in m/s in its heading frame, yaw rate; it holds without a command for 0.5 s; `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`); in Python `lagrange.Simulation(lagrange.sample_quadcopter(), world=None, profile=lagrange.sample_profile("quad_x500_hover"))`; its rotors name propeller files (`assets/propellers/*.prop.yaml`, formats.md §3.3.1: blade geometry, sections, hand), drawn as blades, and `quad_x500_bem.yaml` flies them by blade-element momentum (`rotor_aero.fidelity: bem`, with `aero:rotor:mount` puller/pusher; checked against NACA Report 640 in `test_bem`); variants: `--robot assets/robots/quad_x500/quad_x500.urdf --overlay assets/robots/quad_x500/quad_x500_folding.overlay.yaml` (folding blades, `rotor/<j>/blade_angle`) and `assets/robots/quad_x500_inverted/quad_x500_inverted.usda` (motors under the arms, pushers)
- EMI (v0.5): `lagrange run profiles/examples/quad_x500_power_line.yaml --robot assets/robots/quad_x500/quad_x500_emi.usda --world assets/worlds/power_line/power_line.usda`: the quadcopter hovers next to a 400 A, 60 Hz power line (`EmiSourceAPI_1`; the grid frequency is a site parameter in the scene) and its magnetometer and IMU (`EmiSusceptibilityAPI_1`) see the field; `lagrange_emi` runs whenever the stage has EMI prims, `emi.coupling` picks the victims, `--domain emi=off` is the zero substitute; the viewer's Environment workspace shows the sources and |B| slices (emi.md §8)
- Comms (v0.5): `lagrange_server --robot assets/robots/amr_diffdrive/amr_diffdrive_wifi.usda --world assets/worlds/warehouse_small/warehouse_small.usda --link-topics assets/robots/amr_diffdrive/amr_diffdrive_wifi.links.yaml`: `lagrange_comms` (runs for `RadioLinkAPI_1`/`CanBusAPI_1` prims) computes `link/<l>/*` (received power, SNR, packet loss after retries, latency, throughput, up; walls from the fixed scenery, shadowing, EMI interference) and `can/<c>/*` (ISO 11898-1 fault confinement from EMI bit errors); the bridge loses and delays the mapped topics and /cmd_vel (comms.md §8); `--domain comms=off` is the ideal substitute
- CFD wind import (v0.5): `ros2 run lagrange_fields lagrange_foam_to_vdb CASE/TIME/C CASE/TIME/U OUT.nvdb [--voxel M] [--offset X,Y,Z]` (C from `postProcess -func writeCellCentres`, ASCII files) writes a NanoVDB wind field for `--weather wind_field=OUT.nvdb` or the profile's `weather.wind_field`
- Cars (v0.6): `ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda --world assets/worlds/city_block/city_block.usda` (or `lagrange run profiles/examples/sedan_city.yaml --robot ... --world ...`; Python `lagrange.Simulation(lagrange.sample_sedan(), world=lagrange.sample_city())`): the sample sedan (`sedan.urdf` → `sedan_base.usda` by `lagrange_import`, `sedan.usda` adds `VehicleAPI_1`/`WheelAPI_1` and the ride height) on `lagrange_vehicle_dynamics` (raycast suspension, MF-Tyre 5.2 tyres from `assets/tyres/*.tir`, brakes, Ackermann steering, electric or engine-map powertrain; vehicle_dynamics.md §8) drives a demo circle round the junction until the first `/cmd_vel` (speed and turn rate through the car controller); `vehicle/sedan/*` and `wheel/<j>/*` signals; `--domain vehicle_dynamics=off` is the kinematic bicycle substitute
- OpenDRIVE (v0.6): `ros2 run lagrange_opendrive lagrange_opendrive_import IN.xodr OUT.usda [--root NAME] [--step M]` writes road surfaces, road marks and `RoadLaneAPI_1` lanes (own reader: line, arc, spiral, poly3, paramPoly3, elevation, lane sections, links, junctions; formats.md §3.11); the city block is `city_block.usda` = `city_block_buildings.usda` (from the URDF) + `city_block_roads.usda` (from `city_block.xodr`, made by `city_block_xodr.py`)
- Traffic (v0.6): on a world with `RoadLaneAPI_1` lanes the server adds `--traffic vehicles=N,pedestrians=M,seed=S` agents (default 8, 6, 1) as kinematic bodies (`World::Bodies::kinematic`; MuJoCo mocap, Jolt kinematic, moved by `body/<b>/pose_command`); `lagrange_traffic` drives them: IDM lane following behind agents and robots, fixed-cycle lights at junctions, first-come right of way, social-force pedestrians on sidewalks (traffic.md §8); `traffic/agents`, `traffic/lights`; `--domain traffic=off` hides them
- Scenarios (v0.6): `lagrange_server --robot assets/robots/sedan/sedan.usda --scenario assets/scenarios/cut_in.xosc [--scenario-ego NAME]` runs an OpenSCENARIO 1.x subset (own runner `lagrange_openscenario`: parameters, entities, Init teleports and speeds, speed and lane-change actions, time/position/distance/speed/travelled conditions, stop trigger; traffic.md §8.1); the scenario's OpenDRIVE is the world unless `--world` is given, its `Ego` is the robot, other entities are kinematic `scenario_<name>` bodies; `scenario/state` turns 1 at the stop trigger
- Autoware (v0.6): `ros2 launch lagrange_autoware sedan_autoware.launch.py [viewer:=true] [kinematic_state:=false]` runs the sedan in the city block with `lagrange_autoware_interface` (`bridges/autoware`): Autoware's `/control/command/control_cmd`, `gear_cmd`, indicator commands and `/control/control_mode_request` to the server's `/ackermann_cmd` (taken for any `VehicleAPI_1` robot), and `/vehicle/status/*` and ground-truth `/localization/kinematic_state` back; sensors remapped into `/sensing/...`. It builds only when `autoware_control_msgs` and `autoware_vehicle_msgs` are found (`LAGRANGE_AUTOWARE=on`; the vehicle messages have no Lyrical binary, so build `autoware_msgs` 1.11.0 in an underlay; docs/guide/autoware.md)
- Huge worlds (v0.7): a world stage whose tiles are `WorldTileAPI_1` prims with payloads is streamed: `lagrange_server --world assets/worlds/tiled_town/tiled_town.usda --spawn 1036,1036` loads the tiles within `world.streaming.load_radius` (default 600 m) of the robot on worker threads, drops them beyond `unload_radius` (800 m), and holds the run while the robot's ground is not in physics (huge-worlds.md §2.1); `--add-robot NAME=FILE@X,Y[,Z[,YAW]]` adds passive robots (names prefixed `NAME_`); `server::RegionHost` runs robots far apart in separate regions, each with its own origin and tiles (§3.1, headless); the floating origin moves with the robot every 1000 m during the run (§1.4: world tables shifted, run rebuilt keeping its state, `IModule::shiftOrigin`; ROS `/odom` stays continuous)
- Worlds from open data (v0.7): `ros2 run lagrange_geospatial lagrange_build_world OUT_DIR --origin LAT,LON,HEIGHT|ground --size M [--dem FILE --geoid /usr/share/proj/egm96_15.gtx | --assume-ellipsoidal] [--osm FILE.osm] [--tiles3d TILESET.json] [--fine-tile M --fine-cell M --collision-cell M --coarse-tile M --coarse-cell M]` writes a streamed world (terrain prisms, OSM buildings and roads, 3D Tiles for show, ATTRIBUTION.md; huge-worlds.md §4.1); run it with `--geo-origin` at the same point; `tools/ci/build_sample_world.sh [OUT]` builds the 40 km Zurich sample from the Copernicus DEM and OpenStreetMap (network, not committed)
- Huge worlds tutorial (v0.7): `docs/guide/tutorial-huge-worlds.md`; `tools/ci/tutorial_huge_worlds.sh [--network]` runs it headless (the 40 km sample only with `--network`); Python `lagrange.Simulation(..., spawn=[x, y])`, `region_origin()`, `colliders()`, `lagrange.sample_tiled_town()`
- Cars tutorial (v0.6): `docs/guide/tutorial-cars.md`; `tools/ci/tutorial_cars.sh` runs it headless (the Autoware step when `lagrange_autoware` is built)
- Drone tutorial (v0.4): `docs/guide/tutorial-drone.md`; `tools/ci/tutorial_drone.sh` runs it headless (the PX4 step when PX4 SITL is built)
- PX4 mission (v0.4): `tools/ci/px4_mission.sh [--wind]` builds nothing: it needs PX4 SITL built in `$PX4_DIR` (default `~/PX4-Autopilot`, v1.17.0, `make px4_sitl_default`); it installs the airframe `bridges/mavlink/px4/22100_lagrange_quad_x500`, starts `lagrange_server --mavlink px4` with the sample quadcopter, starts PX4 in lockstep, flies `bridges/mavlink/px4/fly_square_mission.py` (pymavlink, take-off, 10 m square, land; exit 3 on a missed waypoint), and stops both
- ArduPilot mission (v0.4): `tools/ci/ardupilot_mission.sh [--wind]` needs ArduCopter SITL built in `$ARDUPILOT_DIR` (default `~/ardupilot`, Copter-4.7.2, `./waf configure --board sitl && ./waf copter`, pymavlink in its `.venv`); it starts `lagrange_server --mavlink ardupilot` (JSON interface, UDP 9002, lockstep at 1 kHz), runs `arducopter --model JSON` with `bridges/mavlink/ardupilot/lagrange_quad_x500.parm`, flies `bridges/mavlink/ardupilot/fly_square_mission.py` over MAVLink TCP 5760, and stops both
- Arm demo (v0.3): `ros2 launch lagrange_demos arm6_workcell.launch.py [motors:=sized|undersized] [electrical:=on|off] [ambient:=K] [rtf:=X] [record:=FILE.mcap] [viewer:=true]`, then `ros2 run lagrange_demos pick_place [--cycles N] [--speed 0..1]` (MoveIt 2 via moveit_py through joint_trajectory_controller at 1 kHz; exit 3 when a move fails); `tools/ci/arm_scenario.sh` runs the derating scenario of robot-classes.md §4 (about ten minutes)
- Nav2 demo: `ros2 launch lagrange_demos amr_warehouse.launch.py [viewer:=false] [control:=ros2_control|builtin] [lidar:=auto|cpu|gpu]`, then `ros2 run lagrange_demos patrol [--minutes N]` (fails on a missed waypoint or a collision; nightly CI runs 10 minutes via `tools/ci/nav2_demo.sh 10`)
- Profiles: the server validates profiles from the viewer against `profiles/schema.json` (jsoncons, JSON Schema 2020-12) and what it can run, and applies them by rebuilding the run at t = 0 (domain on/off with substitutes, `rigid_body.dt`, `time.real_time_factor`); other keys are warnings
- Viewer: built by colcon with everything else; run `ros2 run lagrange_viewer lagrange_viewer` (starts on the launcher, whose Start local simulator runs `lagrange_server`; `--connect <endpoint>` goes straight to a server; `--follow M[,DEG]` starts with the follow camera; headless `--record-frames DIR [--record-fps N] --frames N` writes a PNG frame sequence, which is how `docs/imgs/sedan-city.gif` was made)
- Viewer headless test: `colcon test --packages-select lagrange_viewer` (runs on the software Vulkan device, llvmpipe; `LAGRANGE_TEST_GPU=<name>` picks another)
- GPU and tier of this machine: `ros2 run lagrange_gpu lagrange_gpu_info`
- Container: `podman build -f packaging/container/Containerfile -t lagrange:dev .` then `podman run --rm lagrange:dev` runs the Nav2 demo headless (built-in controller; `-e CONTROL=ros2_control` for ros2_control), and `podman run --rm lagrange:dev tools/ci/tutorial_drone.sh` the drone tutorial (without its PX4 and ArduPilot steps); Docker reads the same file. Local `.deb` dry runs: `bloom-generate rosdebian --ros-distro lyrical` with `packaging/rosdep/lagrange-self.yaml` registered (docs/plan/spikes/0.2-H8-packaging.md)
- Benchmarks: `tools/ci/benchmarks.sh MACHINE` (scheduler, bus, lidar per tier; medians against `tests/benchmarks/baseline.json`, see `docs/design/benchmarks.md`)
- All hooks: `pre-commit run --all-files`
- Protocol: `buf lint && buf format --diff --exit-code && buf breaking --against '.git#ref=origin/dev'`

## 8. Code conventions

- Format with the **pinned** clang-format version in `.clang-format`; never reformat code you
  did not change.
- clang-tidy must pass; `bugprone-*`, `concurrency-*`, `performance-*` are errors.
- No raw `new`/`delete`; explicit ownership at module boundaries; no exceptions across the C ABI.
- Physics constants: name them and comment units + source; magic-number lint is disabled
  for physics code, documentation is not.
- Every physical quantity in an interface carries a unit (type or `Unit` field), SI by default.
- Python: ruff + mypy strict on new code.
- Viewer: ImGui calls only on the UI thread; no blocking network I/O in the frame loop;
  protocol decoding on a worker thread with a lock-free handoff to the frame.
- Sanitizer builds (ASan/UBSan/TSan) must stay green.

## 9. Git workflow

- **`main` is for releases only.** It changes only by a release PR from `dev` (or a `release/x.y`
  or `hotfix/…` branch). Never branch feature work from `main` and never open a feature PR
  against it.
- **`dev` is the integration branch.** Create work branches from `dev` and open PRs against `dev`.
- **Conventional branch names**: `<type>/<kebab-description>` with type one of `feat`, `fix`,
  `docs`, `ci`, `chore`, `refactor`, `perf`, `test`, `build`, `revert`. Example:
  `feat/viewer-scope-panel`.
- **Conventional Commits**, scopes matching the tree: `core`, `gpu`, `mod/<domain>`, `viewer`,
  `proto`, `bridge/<name>`, `io`, `schemas`, `docs`, `ci`, `packaging`.
  Example: `feat(viewer): add waveform scope panel`.
- **DCO sign-off on every commit** (`git commit -s`).
- Squash merge into `dev` (the PR title becomes the commit, so it must be a Conventional Commit).
  Releases merge `dev` into `main` with a merge commit, so the two branches never diverge.
- Never force-push `main` or `dev`. Never commit directly to `main` or `dev`.
- Commit messages carry only the author's sign-off. Do not add tool or AI co-author trailers.
- Changes to `schemas/`, `core/plugin_abi/`, or `proto/` need an RFC or an explicit note in the
  PR and 2 approvals.
- No files >1 MB in the repo; large assets go to the separate assets repo/release bundles.

## 10. Rules for Claude Code specifically

- **Do not invent versions, licenses, or API details.** Library versions, GitHub Action SHAs,
  pre-commit `rev:` pins, license terms, and tool maturity (Dear ImGui docking/multi-viewport on
  Wayland, ImPlot and node-editor versions, SDL3/GLFW Wayland support, Rust USD bindings,
  Newton/MuJoCo Warp/Isaac Sim status)
  must be verified from the source. If you cannot verify, write `TODO(verify): …` instead.
- Before adding a dependency, check its license is compatible with Apache-2.0 (no GPL in core)
  and record it in `docs/design/dependencies.md`.
- Prefer small, reviewable changes. Plan first for anything touching more than one top-level
  directory.
- When a task conflicts with an invariant in §3, stop and say so instead of working around it.
- Realism claims need validation: any new physical model ships with a test and, where
  possible, a reference dataset or published reference it is checked against.
- Note: site-specific constants (e.g. grid frequency 50 vs 60 Hz) are scenario parameters,
  never hard-coded defaults.
- Update this file when an invariant, command, or the current phase changes.

## 11. Reference examples (canonical starting points)

These come from the agreed design. They are illustrative, not frozen APIs. Real versions get
written and reviewed through the RFCs in `rfcs/`.

### 11.1 Simulation profile (YAML)

Full multi-domain run (bridge inspection, heavy rain at night, sensors replaced by ground truth):

```yaml
# profiles/examples/bridge_rain_electrical.yaml
stage: worlds/bridge_inspection.usda
environment: environments/heavy_rain_night.usda

domains:
  rigid_body:   { enabled: true,  backend: physx, dt: 0.001 }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: true,  fidelity: drag_plus_field }
  weather:      { enabled: true }
  emi:          { enabled: true,  coupling: [magnetometer, can_bus, wifi] }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: false, substitute: ground_truth }

views: [electrical_schematic, waveforms, physics_3d]
record: { format: mcap, path: runs/run_001.mcap }
```

Electrical-only "bench mode" (physics replaced by a replayed real trajectory). This shows the
toggle concept in the other direction:

```yaml
# profiles/examples/bench_electrical_replay.yaml
stage: robots/amr_diffdrive.usda

domains:
  rigid_body:   { enabled: false, substitute: replay, source: logs/real_amr_run.mcap }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: false, substitute: zero }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: false, substitute: zero }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: false, substitute: ground_truth }

views: [electrical_schematic, waveforms]
record: { format: mcap, path: runs/bench_001.mcap }
```

Rules: every disabled domain names a substitute, and every profile validates against
`profiles/schema.json` in CI.

### 11.2 Substitutes when a domain is disabled

| Disabled domain | Replaced by | Example use |
|---|---|---|
| Electrical | Ideal torque source (commanded torque delivered exactly) | Pure physics/navigation testing |
| Rigid-body physics | Fixed robot, mechanical load profile (torque vs. speed), or replayed real data | Bench mode: currents/voltages on a stationary or virtual-load robot |
| Thermal | Constant ambient temperature | Short tests where heating doesn't matter |
| Weather | Clear conditions | Baseline sensor testing |
| EMI | Zero field | Baseline |
| Radiation | Zero dose | Non-nuclear scenarios |
| Sensors | No rendering; ground-truth state published | High-speed planner/controller testing |

There are three substitute kinds: **ideal**, **constant**, and **replay** (from MCAP).

### 11.3 USD custom schema usage (electrical, thermal, EMI)

```usda
def Xform "left_drive_motor" (
    prepend apiSchemas = ["ElectricalMotorAPI", "ThermalAPI", "EmiSusceptibilityAPI"]
)
{
    float elec:windingResistance = 0.12      # ohm @ 25 °C
    float elec:torqueConstant = 0.085        # N·m/A
    float elec:inductance = 0.00031          # H
    asset elec:behaviorModel = @models/bldc_drive.fmu@
    rel elec:powerPort = </robot/power_bus/dc_48v>
    float thermal:heatCapacity = 450.0       # J/K
}

def Xform "power_line_01" ( prepend apiSchemas = ["EmiSourceAPI"] )
{
    float emi:frequency = 60.0               # Hz — site parameter (Kyushu/western Japan: 60 Hz,
                                             # eastern Japan/Europe: 50 Hz). Never a code default.
    float emi:current = 400.0                # A
    asset emi:fieldMap = @fields/power_line_01.vdb@
}
```

Planned applied schemas (all versioned `_v1`): `ElectricalMotorAPI`, `BatteryAPI`, `PortAPI`,
`ThermalAPI`, `AeroAPI`, `EmiSourceAPI`, `EmiSusceptibilityAPI`, `RadiationAPI`,
`SensorMaterialAPI` (non-visual properties: lidar reflectivity at 905/1550 nm, radar cross-section,
thermal emissivity, conductivity, magnetic permeability).

Electrical **topology** (ports + connection relationships) lives on the same prims as the 3D
components, so the 3D view and schematic view cannot disagree. It is exportable to SPICE netlist
and Modelica.

### 11.4 Module interface (C++)

```cpp
// core/module/IModule.hpp
struct SignalSpec {
    std::string_view name;   // e.g. "joint/left_wheel/torque"
    SignalType type;         // Scalar, Vec3, Field3D, ...
    Unit unit;               // N·m, A, K, V/m — checked at wiring time
};

class IModule {
public:
    virtual ~IModule() = default;
    virtual std::span<const SignalSpec> inputs()  const = 0;
    virtual std::span<const SignalSpec> outputs() const = 0;
    virtual Duration preferredStep() const = 0;     // 100 µs electrical, 1 ms physics...

    virtual void init(const WorldView&, const ModuleConfig&) = 0;
    virtual void step(SimTime t, Duration dt, SignalBus& bus) = 0;
    virtual void shutdown() = 0;
};
```

### 11.5 Substitute implementation (C++)

```cpp
// modules/electrical/substitutes/IdealTorqueSource.cpp
// Used when electrical is disabled: commanded torque passes straight through.
void IdealTorqueSource::step(SimTime, Duration, SignalBus& bus) {
    for (auto& j : joints_)
        bus.write(j.torqueOut, bus.read(j.torqueCommand));
}
```

The scheduler builds a dependency graph from enabled modules and substitutes at every run start.
If the electrical module outputs N·m and physics expects N, the scheduler refuses to start.

### 11.6 Architecture: server and clients

```
[Sim Server or Mock Server] ─ physics + sensor sim + world state (headless, Ubuntu 26.04 + GPU)
     │
     ├── ROS 2 bridge (rclcpp, rmw_fastrtps / rmw_zenoh) ── your robot stack
     ├── gRPC/Zenoh API ── Python/C++ scripting, RL training
     └── Viewer protocol (Protobuf: snapshot + deltas + signal streams)
              │   same machine: + shared-memory fast path for big buffers
              │   other machine: over the network
              ▼
        lagrange_viewer ─ native window, Vulkan renderer (gpu/hal + Slang), Dear ImGui panels
```

### 11.7 Domain time scales (drives step sizes and fidelity tiers)

| Domain | Natural time scale | Real-time at full detail? | Real-time approach |
|---|---|---|---|
| Power electronics switching | 10 ns – 10 µs | No | Averaged PWM; SPICE export offline |
| Motor currents, battery | 10 µs – 1 ms | Yes, averaged | dq-frame motor, ECM battery |
| Rigid-body mechanics, contacts | 0.1 – 2 ms | Yes | Physics engine |
| Aerodynamics (true CFD) | µs steps on fine grids | No | Drag models + baked OpenFOAM fields; GPU LBM for small volumes |
| Thermal | 0.1 s – minutes | Yes | Lumped networks |
| Radiation dose | Seconds – hours | Yes | Ray-traced dose fields; Geant4 offline |

## 12. Tooling reference configs

`rev:` pins and tool versions are intentionally left unpinned. Verify and pin them when the
files are created (`TODO(verify)`).

### 12.1 `.clang-format`

```yaml
BasedOnStyle: Google
Language: Cpp
Standard: c++20            # matches ROS 2 Lyrical's C++ baseline
ColumnLimit: 100
IndentWidth: 4
AccessModifierOffset: -4
PointerAlignment: Left
DerivePointerAlignment: false
IncludeBlocks: Regroup
IncludeCategories:          # first matching regex wins, so order matters
  - Regex: '^<lagrange/'      # project headers last
    Priority: 3
  - Regex: '^<(pxr|PxPhysics|mujoco|openvdb)'  # third-party
    Priority: 2
  - Regex: '^<.*>'           # std
    Priority: 1
AllowShortFunctionsOnASingleLine: Inline
BreakConstructorInitializers: BeforeComma
SortIncludes: CaseSensitive
```

### 12.2 `.clang-tidy`

```yaml
Checks: >
  -*,
  bugprone-*,
  cppcoreguidelines-*,
  modernize-*,
  performance-*,
  readability-*,
  concurrency-*,
  -modernize-use-trailing-return-type,
  -readability-magic-numbers,
  -cppcoreguidelines-avoid-magic-numbers
WarningsAsErrors: 'bugprone-*,concurrency-*,performance-*'
HeaderFilterRegex: '^(core|gpu|modules|bridges|io|viewer|server|mock_server)/'
```

Magic-number checks are disabled on purpose, because physics code is full of legitimate
constants. Name and document constants instead (§8).

### 12.3 `.pre-commit-config.yaml`

```yaml
repos:
  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: <pinned>
    hooks: [{ id: clang-format, types_or: [c++, c] }]
  - repo: https://github.com/BlankSpruce/gersemi-pre-commit
    rev: <pinned>
    hooks: [{ id: gersemi }]
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: <pinned>
    hooks: [{ id: ruff, args: [--fix] }, { id: ruff-format }]
  - repo: https://github.com/crate-ci/typos
    rev: <pinned>
    hooks: [{ id: typos }]
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: <pinned>
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-added-large-files   # blocks accidental big assets
        args: [--maxkb=1024]
      - id: check-merge-conflict
      - id: detect-private-key
  # Also add: buf format/lint, yamllint, markdownlint-cli2 or mdformat, shfmt, shellcheck,
  # REUSE lint. No Node/JS formatters are needed: the project has no TypeScript.
  # TODO(verify): markdownlint-cli2 needs Node; prefer mdformat (Python) if avoiding Node in dev.
```

CI runs the same hooks, so skipping them locally never lets bad formatting land.

### 12.4 Formatters and linters per language

| Language / file | Formatter | Linter |
|---|---|---|
| C++ | clang-format (pinned major version) | clang-tidy |
| Slang shaders | clang-format (verify output looks sane) | slangc compile check |
| CMake | gersemi | — |
| Python | ruff format | ruff + mypy |
| Protobuf | buf format | buf lint + buf breaking |
| YAML | yamlfmt (TODO(verify)) | yamllint |
| Markdown | mdformat (TODO(verify)) | markdownlint or mdformat --check |
| Shell | shfmt | shellcheck |
| Everything | — | typos, REUSE |

### 12.5 Repository files

```
.editorconfig  .gitattributes  .gitignore
.clang-format  .clang-tidy  .pre-commit-config.yaml
CMakePresets.json
LICENSE (Apache-2.0)  LICENSES/  REUSE.toml
README.md  CONTRIBUTING.md  CODE_OF_CONDUCT.md  SECURITY.md  GOVERNANCE.md  CHANGELOG.md
.github/
├── CODEOWNERS             # schemas/, core/plugin_abi/, proto/ require core-maintainer review
├── ISSUE_TEMPLATE/        # bug (OS/GPU/driver required), feature, robot-class request
├── PULL_REQUEST_TEMPLATE.md
├── labels.yml             # area/*, type/*, good-first-issue, needs-rfc
├── renovate.json
└── workflows/
```

### 12.6 CI pipeline

| Stage | Runs | Trigger |
|---|---|---|
| Lint | pre-commit (all hooks), commitlint, DCO, buf breaking, REUSE | Every PR |
| Build | Ubuntu 26.04 amd64 + arm64, inside `ros:lyrical` containers (TODO(verify) image tag) | Every PR |
| Test | colcon test (gtest), pytest | Every PR |
| Sanitizers | ASan+UBSan, TSan (Linux) | Every PR |
| Viewer | Headless viewer against the mock server on software Vulkan (Mesa lavapipe): renders N frames, checks frame-time budget, golden-image comparison of the viewport, ImGui Test Engine panel tests (TODO(verify) license) | Every PR |
| GPU tests | Sensor/kernel tests on a self-hosted NVIDIA runner | Merge queue + nightly |
| Determinism | Same scenario twice → bit-identical CPU output | Merge queue |
| Validation | Sim-vs-real regression tolerances | Nightly |
| Performance | Benchmarks; fail on regression over threshold | Nightly |
| Docs | Build docs site, check links | Every PR |

Pin all GitHub Actions to commit SHAs. Cache vendor-package builds and use
ccache/sccache (OpenUSD builds are slow). Releases use release-please and produce
`ros-lyrical-*` debs via bloom and the ROS buildfarm, Docker images on ubuntu:resolute,
a pip wheel, an SBOM, and Sigstore signatures. Security: CodeQL, secret scanning with push protection, an OpenSSF
Scorecard workflow, and Renovate grouped weekly.

## 13. Viewer (native GPU desktop application)

The viewer is `lagrange_viewer`, a native C++20 program. It opens real OS windows and draws
everything on the GPU with Vulkan. There is no browser or web engine involved.

**The UI design is in `docs/design/ui/` and is binding**: shared conventions, design tokens, the
ImGui theme, and one `SPEC.md` + `reference.png` per screen. Read it before any viewer UI work;
the rules are in `.claude/rules/viewer-ui.md`. It replaces the earlier panel sketches in
`docs/design/viewer.md` where they differ.

### 13.1 Stack

| Concern | Choice |
|---|---|
| Window, input, Vulkan surface | SDL3 (RFC 0006, accepted). Must work on Wayland and X11 |
| 3D rendering | Own Vulkan renderer in `viewer/render/`, built on `gpu/hal/` and Slang shaders |
| UI panels | Dear ImGui, docking branch (dockable panels, saved layouts, optional multi-viewport) |
| Waveform scopes | ImPlot, fed from ring buffers; decimation (min/max per pixel) for 10 kHz+ signals |
| Electrical schematic | imgui-node-editor or equivalent node graph (TODO(verify) license + maintenance) |
| Gizmos | ImGuizmo or own implementation (TODO(verify)) |
| Fonts | Bundled, license-checked (e.g. a SIL-OFL font), Japanese glyph range supported |
| HiDPI | Per-monitor scale factor applied to ImGui style and font atlas |

### 13.2 v0.1 panels

3D viewport (orbit/fly camera, grid, axes, selection, transform gizmo, contact/force arrows,
field overlays) · scene tree (USD hierarchy) · property inspector (incl. custom schemas) ·
domain toggle panel (writes a profile that validates against `profiles/schema.json`) ·
timeline (play/pause/step, real-time factor, replay scrubbing) · waveform scopes ·
electrical schematic with animated current · console/log · ROS 2 topic browser ·
layout presets.

### 13.3 Frame loop and threading

```cpp
// viewer/app/FrameLoop.hpp (illustrative)
class FrameLoop {
public:
    void run();                         // until window closes
private:
    void pollEvents();                  // SDL3/GLFW events → ImGui IO
    void applyIncomingState();          // swap latest decoded snapshot/deltas (lock-free)
    void buildUi();                     // all ImGui panel code, UI thread only
    void renderViewport(VkCommandBuffer);  // 3D scene into an offscreen image
    void renderUi(VkCommandBuffer);     // ImGui draw data (viewport shown as an ImGui image)
    void present();
};
// Network receive + protobuf decode run on a worker thread (viewer/client/).
// The UI thread never blocks on I/O.
```

### 13.4 Performance budgets (v0.1 targets; refine in docs/design/viewer.md)

- 60 fps (≤16.6 ms frame) on an NVIDIA RTX-class GPU with a sample AMR scene and 8 open scopes.
- Scopes: ≥16 signals at 10 kHz each without dropped samples in the ring buffer.
- Cold start to first rendered frame with a sample URDF: under 5 s on a warm disk.
- Must still run (degraded) on Tier 1 GPUs without hardware ray tracing.

### 13.5 Known limitations to design around

- ImGui multi-viewport (panels torn off into separate OS windows) has historically had gaps on
  Wayland because clients cannot position their own windows. [Medium confidence] Make torn-off
  windows optional, and keep a fully docked single-window mode as the default.
- An immediate-mode UI rebuilds every frame. Keep panels cheap: no per-frame allocation in the
  hot path, virtualized lists for large scene trees (ImGuiListClipper).
