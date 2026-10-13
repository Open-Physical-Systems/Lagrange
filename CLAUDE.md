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

**v0.1 = native viewer + UI↔server protocol + mock/replay server.** No physics yet.

- The viewer is a native Vulkan + Dear ImGui desktop application (§13). It is not a web app.
- The viewer must be built against the real protocol, fed by the mock server. Never hard-code
  fake data inside viewer panels.
- The mock server becomes the product's replay mode later — write it as production code.
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
├── schemas/                    # USD custom schemas — the data model
│   ├── electrical/schema.usda      # ElectricalMotorAPI, BatteryAPI, PortAPI...
│   ├── thermal/  aero/  weather/  emi/  radiation/  sensor_material/
│   └── generated/              # usdGenSchema output (C++ + Python) — never edit by hand
├── profiles/
│   ├── schema.json             # JSON Schema for YAML sim profiles
│   └── examples/               # validated example profiles (see §11.1)
├── proto/                      # UI↔server protocol (Protobuf/FlatBuffers)
│
├── core/                       # no GPU, no physics — pure framework
│   ├── module/                 # IModule interface, registry, plugin loading
│   ├── scheduler/              # multirate co-sim master, dependency graph
│   ├── signals/                # typed signal bus (torque, current, temp...)
│   ├── time/                   # sim clock, lockstep, real-time factor
│   ├── world/                  # runtime world state (flat SoA arrays)
│   │   ├── coords/             # double-precision world, floating origin, WGS84↔ENU
│   │   ├── streaming/          # tile loading, LOD, USD payload management
│   │   └── regions/            # active-region management, future distributed handoff
│   ├── stage/                  # USD stage loading → runtime world conversion
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
├── mock_server/                # v0.1 mock/replay server (becomes replay mode)
├── python/                     # nanobind bindings, `pip install lagrange`
├── viewer/                     # native C++ app (ament_cmake package `lagrange_viewer`)
│   ├── app/                    # main(), window (SDL3/GLFW), Vulkan swapchain, frame loop
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
- Viewer: built by colcon with everything else; run `ros2 run lagrange_viewer lagrange_viewer --connect <endpoint>`
- Viewer headless test: `colcon test --packages-select lagrange_viewer` (runs on the software Vulkan device, llvmpipe; `LAGRANGE_TEST_GPU=<name>` picks another)
- GPU and tier of this machine: `ros2 run lagrange_gpu lagrange_gpu_info`
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
