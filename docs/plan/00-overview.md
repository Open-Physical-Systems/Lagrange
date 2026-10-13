# Project overview

Project name: **Lagrange** (Open Physical Systems). Status: plan for review. Facts
about other projects come from `docs/plan/verification.md` (checked 2026-10-13).

## 1. Vision

An Apache-2.0 robot simulator for ROS 2 Lyrical on Ubuntu 26.04 in which a robot is more than a
set of rigid bodies. The same model carries its electrical system, its heat, the air around it,
the weather, and the interference fields it moves through, and each of those domains can be
switched on, switched off, or replaced by recorded data without breaking the rest.

Three user stories define success:

1. **"Why did the robot reset on the ramp?"** An AMR climbs a ramp with a low battery. Bus
   voltage sags, the computer browns out, navigation stops. The user sees the voltage on a scope,
   the current animated in the schematic, and the robot in 3D, all on one timeline.
2. **"Test my drive firmware without the robot."** Physics is replaced by a replayed real
   trajectory. Only the electrical and thermal domains run (bench mode), at 10 kHz, on a laptop.
3. **"Does my planner survive fog and a noisy magnetometer?"** The same scenario runs with
   weather and EMI enabled, then with both replaced by clear and zero substitutes, and the
   difference is attributable.

## 2. Where it sits among existing simulators

| | Gazebo Jetty | Isaac Sim 6 | MuJoCo 3 | Newton 1.x | Genesis | This project |
|---|---|---|---|---|---|---|
| License | open (line not re-verified) | source Apache-2.0, but needs Omniverse Kit under NVIDIA terms | Apache-2.0 | Apache-2.0 | Apache-2.0 | Apache-2.0 |
| ROS 2 Lyrical debs | yes (`gz-jetty`, `ros-lyrical-ros-gz`) | no (own distribution) | vendor package + `mujoco_ros2_control` | no | no | target: `ros-lyrical-lagrange-*` |
| ros2_control | `gz_ros2_control` | through Isaac ROS bridges | `mujoco_ros2_control` | not found | not found | first-class, in-process lockstep |
| GPU-batched physics | no (not in the sources checked) | yes (PhysX GPU, Newton) | via MuJoCo Warp/MJX (Python) | yes (Python/Warp) | yes (Python) | Tier 2, through backends |
| Electrical domain (battery, drives, bus) | no | no | no | no | no | **yes** |
| Thermal, EMI, ionizing dose | no | no | no | no | no | **yes** |
| Per-domain toggle with substitutes and replay | no | no | no | no | no | **yes** |
| Unit-checked signals | no | no | no | no | no | **yes** |
| Native scope and schematic views | plots only | no schematic | no | no | no | **yes** |
| Viewer technology | Qt + OGRE | Omniverse Kit | GLFW/OpenGL viewer | — | — | native Vulkan + Dear ImGui |

"no" for other projects means "not found in the sources checked for this plan"
(`verification.md` §5), not a proof of absence. The table must be re-checked before it is used in
public material.

### The wedge

1. **Multi-physics that robotics engineers actually debug**: power, heat and interference,
   coupled to motion, at real-time rates with averaged models.
2. **Toggle and substitute**: any domain off, ideal, constant, or replayed from MCAP. This also
   gives bench mode and makes performance scale down to what the user needs.
3. **One model, several views**: the 3D scene, the schematic and the scopes are projections of
   one USD stage and one signal bus.
4. **ROS-native packaging**: `apt install`, ros2_control lockstep, `simulation_interfaces`,
   zero-copy sensor output through Lyrical's `rosidl::Buffer`.

### What is not the wedge (integrate, do not build)

Rigid-body solvers (MuJoCo, Jolt, PhysX), ray casting (Embree, Vulkan ray query), traffic (SUMO),
scenario formats, CFD and particle transport (offline tools), planning and control stacks (Nav2,
MoveIt 2, PX4, Autoware).

## 3. Non-goals

- Windows, macOS, browsers, mobile, Metal, WebGPU. Post-1.0, by RFC only.
- A photorealistic renderer or a synthetic-data product. Camera simulation targets sensor
  behaviour (exposure, noise, distortion, weather), not film-quality images.
- Switching-level power electronics, full-wave electromagnetics, CFD or particle transport in the
  loop. These stay offline, exchanged through files.
- An EMC or radiation-safety certification tool. EMI and dose models are for robot software
  testing, with stated accuracy.
- A reinforcement-learning framework. Batched environments and Python bindings are provided;
  training loops are not.
- A robot description format of our own. Robots come from URDF, SDFormat, MJCF or USD.
- Legged, marine and climbing robots before 1.0.
- Jetson support, until JetPack ships on Ubuntu 26.04 (verification V9).

## 4. Principles that follow from CLAUDE.md

- Modules talk only through the unit-checked signal bus.
- A deterministic CPU reference mode always exists.
- Every feature states its behaviour on Tier 0, 1 and 2.
- Every physical model ships with a test and a reference.
- The viewer is a protocol client and nothing else.
- Nothing is pinned or claimed without verification; unknowns are written as `TODO(verify)`.

## 5. Success metrics per release

Metrics are measured on the reference desktop (to be fixed in v0.1; proposal: 8-core x86-64 CPU,
32 GB RAM, NVIDIA RTX 4060-class GPU, Ubuntu 26.04, driver 595) unless stated.

| Release | Theme | Acceptance (summary; full criteria in `roadmap.md`) | Quantitative metrics |
|---|---|---|---|
| v0.1 | Viewer, protocol, mock server | Load a sample URDF and see it in 3D within a minute of first launch; replay an MCAP at 60 fps with scopes; toggles produce valid YAML; runs on Wayland and X11 | Frame ≤ 16.6 ms p99 with 8 scopes; 16 signals × 10 kHz with zero drops for 60 s; cold start < 5 s; headless test green on lavapipe |
| v0.2 | Sim core + AMR | Sample AMR navigates a warehouse with Nav2 through ros2_control; domains toggle at start; deterministic CPU mode | RTF ≥ 1.0 with lidar on Tier 1; determinism test bit-identical over 10 s; unit mismatch refused at startup (test); `colcon build` from clean < 30 min with cached USD |
| v0.3 | Arms + electrical + thermal | MoveIt 2 pick-and-place with visible currents, sag and derating; bench mode from a recorded trajectory | Electrical at 100 µs in real time for a 7-DoF arm; coupling tests within 2 % RMS; energy balance within 0.5 % |
| v0.4 | Drones | PX4 SITL lockstep mission in gusts; battery sag reduces thrust | 250 Hz+ autopilot loop in lockstep at RTF ≥ 1.0; thrust model within 5 % of reference data |
| v0.5 | Weather + EMI + comms | Fog and rain reduce lidar range as published; magnetometer disturbed near a power line; packet loss near a transmitter | Detection-range error within 20 % of reference measurements; closed-form field tests exact |
| v0.6 | Cars + autonomous cars | OpenSCENARIO cut-in with SUMO traffic and a full sensor suite on Tier 2 | 3 lidars + 6 cameras + radar at RTF ≥ 0.5 on the reference GPU; tyre curves within 0.5 % of the parameter set |
| v0.7 | Huge worlds | 40 km tiled world streams within memory budgets; no entry into unloaded physics tiles | Viewer frame budget held during streaming; identical trajectories at 0 km and 500 km offsets |
| v1.0 | Stability | Schemas, protocol and plugin ABI frozen with migration tools; `ros-lyrical-*` debs installable | `apt install` to first simulation in under 10 minutes on a clean machine; all tutorials run in CI |

Project health metrics (tracked from the first public commit): CI time on a PR under 30 minutes
at the median; sanitizer builds green; every merged physical model has a validation test; time
from a new contributor's clone to a successful build under one hour using the prebuilt underlay.

## 6. Document map

| Topic | File |
|---|---|
| What was verified | `docs/plan/verification.md` |
| Decisions needed and flaws found | `docs/plan/open-questions.md` |
| Risks | `docs/plan/risks.md` |
| Releases, epics, issues | `docs/plan/roadmap.md` |
| Git, CI, release, security, config files | `docs/plan/repo-tooling.md` |
| Architecture | `docs/design/architecture.md` |
| Formats, schemas, profiles | `docs/design/formats.md` |
| Domains | `docs/design/domains/*.md` |
| Robot classes | `docs/design/robot-classes.md` |
| Huge worlds | `docs/design/huge-worlds.md` |
| Protocol | `docs/design/protocol.md` |
| Viewer | `docs/design/viewer.md` |
| Dependencies | `docs/design/dependencies.md` |
| RFC drafts | `rfcs/0001` … `rfcs/0006` |
