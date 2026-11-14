# Lagrange compared with Gazebo, MuJoCo, PyBullet and others

This page compares Lagrange v1.3.0 with Gazebo Jetty (gz-sim 10.5.0), the simulator most ROS 2
users would otherwise pick, and with standalone MuJoCo and PyBullet on physics. Isaac Sim, Webots
and CARLA appear in the feature table only.

Every number in the measured tables comes from a run on one machine on 2026-11-14. The scripts,
the exact commands and the raw output are in
[`docs/release/comparison/`](https://github.com/Open-Physical-Systems/Lagrange/tree/main/docs/release/comparison).
Facts about the other simulators that were not measured are cited and marked *from their
documentation, not measured here*. Where nothing was found, the table says "not found".

## Summary: when to pick which

- **Gazebo** if you want the mature, standard ROS simulator: two decades of use, thousands of
  models, a community, a GUI editor, more platforms (Ubuntu 24.04 and macOS too) and robot
  descriptions that already carry Gazebo plugins. In these measurements it was also slightly
  faster with a camera, and its DART engine held a box on a slope exactly. It was much slower on
  robot physics and lidars, and on a pile of 200 boxes it did not finish.
- **Lagrange** if you need the robot's power, heat, interference, radio and weather simulated
  together with its motion. It is also the pick if you want fast headless runs (6.5 times
  Gazebo's real-time factor on the AMR, a tenth of a second to start) or bit-for-bit repeatable
  runs. Accept a project a month old, with no community, on Ubuntu 26.04 and ROS 2 Lyrical only,
  checked against closed forms and published data but not against real robots of its own.
- **MuJoCo on its own** if you need a physics library for control or learning and no robot
  simulator around it. 4.0.1 is the fastest MuJoCo measured here.
- **PyBullet** for a quick Python experiment. It is exact on these closed forms, but its last
  release was in January 2025.
- **Isaac Sim** (not measured) for RTX-rendered sensors and GPU-parallel learning, if you have
  the GPU it requires. **CARLA** (not measured) for autonomous-driving research on Unreal
  Engine. **Webots** (not measured) for cross-platform teaching and quick starts.

The key numbers, all measured here: real-time factor (higher is faster), median of five runs.

| | Lagrange 1.3.0 | Gazebo Jetty 10.5.0 |
|---|---|---|
| AMR in a warehouse, physics only | 96.6 (MuJoCo), 69.0 (Jolt) | 14.8 (DART) |
| Same, with 2D and 3D lidars and an IMU | 54.1 (CPU lidar), 18.9 (GPU lidar) | 12.7, and only 330–349 of 600 scans per lidar arrived |
| One 640 × 480 camera at 30 Hz | 10.7 | 12.3 |
| 200 boxes falling into a pile | 0.17 (MuJoCo), 2.38 (Jolt) | did not finish 5 s in 30 minutes |
| A quadcopter hovering (different airframes) | 38.0 | 18.3 |
| Start-up to the first step | 0.07 s | 2.4 to 2.7 s |
| Peak memory, AMR with sensors | 78 MB (CPU lidar) | 961 MB |
| Two runs bit-identical | yes (both backends) | not known: the second run was stopped |
| Worst closed-form error in the six physics cases | Jolt backend loses 16 % of a pendulum's energy in 60 s; MuJoCo backend within 0.15 % | within 0.15 % |

## Method

- **Machine.** A laptop: Intel Core i9-13900H (20 threads), 30 GB RAM, NVIDIA GeForce RTX 4060
  Laptop GPU (8 GB, driver 595.99.02) beside an Intel Iris Xe. Ubuntu 26.04.1, Linux 7.0,
  Python 3.14. Both simulators rendered on the NVIDIA GPU: Lagrange picks it itself, and Gazebo
  was given NVIDIA's EGL library (`__EGL_VENDOR_LIBRARY_FILENAMES`). `nvidia-smi` showed both
  processes on it.
- **Quiet machine.** The timed runs started after the machine's other heavy job (a release build)
  had ended and the 1-minute load average was under 3. Each run's record holds the load average
  and the GPU's clocks, temperature and power state at its start (`performance.jsonl`).
- **Versions.**
  - Lagrange 1.3.0 code, RelWithDebInfo. It was built just before the release commit, which
    only bumps version numbers, so `lagrange info --version` printed 1.2.1 (`machine.txt`). Its
    MuJoCo is 3.12.0 and its Jolt 5.6.0.
  - Gazebo Jetty: gz-sim 10.5.0 from `ros-lyrical-ros-gz`. Its default physics is DART; the run
    log shows `gz::physics::dartsim::Plugin` loaded.
  - MuJoCo 3.12.0 and 4.0.1, and PyBullet 3.2.7, from pip in virtual environments.
- **Same scenes.** Every scene is generated from one description, so both simulators get the
  same bodies, masses, inertias, friction, 1 ms step and gravity.
  - The sample AMR in the small warehouse: Gazebo reads the AMR as `gz sdf -p` converts its URDF,
    and the warehouse as one static model (`make_gazebo_world.py`).
  - The accuracy cases: MJCF for Lagrange (both backends) and standalone MuJoCo, SDFormat for
    Gazebo, PyBullet's API for PyBullet.
- **As fast as possible, headless.**
  - Lagrange runs through its Python bindings: `lagrange.Simulation(...).step_until(T)`, no ROS,
    no viewer.
  - Gazebo runs as `gz sim -s -r --headless-rendering --iterations N`, under GNU `time`.
  - The robots stand still: there is no controller in Gazebo, and Lagrange's demo drive is off
    (`joint_control=True` with no commands).
- **What is measured.**
  - **Start-up**: process start to the end of the first step. For Gazebo this is the wall time
    of a one-iteration run, which also includes its shutdown.
  - **Stepping time**: the rest of the run. For Gazebo it is the run's wall time minus the
    median one-iteration run.
  - **Real-time factor**: simulated time over stepping time.
  - **CPU**: CPU seconds over wall seconds while stepping, so 1.0 is one busy core.
  - **Peak RSS**: of the whole process. Lagrange's includes the Python interpreter. Gazebo's
    includes the `gz` Ruby front end, which runs the server in its own process.
- **Repeats.** Five runs of each, interleaved, as separate processes. Tables give the median and
  the range (min–max).

## Performance

Measured on 2026-11-14 from 21:45 to 22:35 JST, 111 runs. At the start of each run, the
1-minute load average was 0.30 to 2.70. The GPU's graphics clock was 210 to 2010 MHz (it idles
between runs and boosts under load), at 44 to 52 °C. Medians of five runs, with the range in
brackets.

| Scenario (simulated time) | Simulator | Real-time factor | Start-up to first step | Peak RSS | CPU while stepping | Sensor output |
|---|---|---|---|---|---|---|
| **AMR in the warehouse, physics only** (60 s) | Lagrange, MuJoCo | **96.6** (96.1–97.0) | 0.07 s | 76 MB | 1.0 core | |
| | Lagrange, Jolt | 69.0 (68.6–69.2) | 0.07 s | 77 MB | 1.0 core | |
| | Gazebo (DART) | 14.8 (14.6–14.9) | 2.44 s | 137 MB | 1.3 cores | |
| **AMR with its sensors**: 2D lidar 720 beams, 3D lidar 16 × 900, both 10 Hz; IMU 200 Hz (60 s) | Lagrange, CPU lidar | **54.1** (53.7–56.3) | 0.07 s | 78 MB | 7.8 cores | 8.2 M rays/s |
| | Lagrange, GPU lidar | 18.9 (17.8–19.0) | 0.24 s | 279 MB | 1.8 cores | 2.9 M rays/s |
| | Gazebo, a subscriber on each lidar | 12.7 (12.3–13.2) | 2.73 s | 961 MB | 1.5 cores | 1.1 M rays/s received; **330–349 of 600 scans per lidar** |
| | Gazebo, no subscriber | 13.0 (12.8–13.3) | 2.73 s | 883 MB | 1.2 cores | nothing (not rendered) |
| **One 640 × 480 camera at 30 Hz** on the AMR (20 s) | Lagrange | 10.7 (10.5–11.2) | 0.32 s | 396 MB | 0.6 cores | 322 frames/s (colour, depth and instance) |
| | Gazebo, a subscriber | **12.3** (11.7–12.7) | 2.72 s | 961 MB | 1.9 cores | 372 frames/s; all 607 frames received |
| **200 boxes falling into a pile** (5 s) | Lagrange, MuJoCo 3.12 | 0.166 (0.165–0.166) | 0.13 s | 84 MB | 1.0 core | |
| | Lagrange, Jolt | **2.38** (2.38–2.38) | 0.11 s | 79 MB | 1.0 core | |
| | Gazebo (DART) | **did not finish in 30 minutes** (one run) | 2.52 s | | | |
| | MuJoCo 3.12 standalone, its defaults | 0.289 (0.289–0.290) | 0.16 s | 60 MB | 1.0 core | |
| | MuJoCo 3.12 standalone, Lagrange's options | 0.184 (0.184–0.184) | 0.15 s | 61 MB | 1.0 core | |
| | MuJoCo 4.0.1 standalone, its defaults | 0.458 (0.458–0.460) | 0.16 s | 62 MB | 1.0 core | |
| | PyBullet 3.2.7 | 0.239 (0.238–0.241) | 0.06 s | 56 MB | 1.0 core | |
| **Quadcopter hovering** (60 s) | Lagrange X500: battery, motor drives at 50 µs, rotors, IMU, GNSS, magnetometer, barometer | **38.0** (37.3–38.2) | 0.09 s | 89 MB | 1.0 core | holds 2.13 m |
| | Lagrange X500, sensors off | 38.4 (38.2–38.6) | 0.09 s | 89 MB | 1.0 core | holds 2.13 m |
| | Gazebo X3 (4 MulticopterMotorModel, MulticopterVelocityControl) | 18.3 (18.2–18.6) | 2.21 s | 135 MB | 1.3 cores | holds 1.88 m |

Notes on the table:

- **Start-up.**
  - Lagrange's figure runs from the first line of the benchmark script to the end of the first
    step. With the interpreter's own start and exit, the whole process outside stepping took
    0.09 to 0.46 s.
  - Gazebo's is a one-iteration `gz sim` run, start to exit.
  - The Gazebo quadcopter's figure is in-process, from loading the world to its first step.
- **Rays per second.** Rays simulated (Lagrange) or received (Gazebo) per second of wall time.
- **Gazebo's lidar scans.** Over 60 simulated seconds each lidar should give 600 scans.
  - The subscriber received 330 to 349 from each.
  - In the same set-up the camera's frames all arrived (607), so the subscriber kept up with far
    larger messages.
  - Why Gazebo's lidars published fewer scans than their rate was not investigated here.
- **The 200-box pile on Gazebo.**
  - The one 5 s run was stopped after 30 minutes of wall time at one CPU core.
  - The determinism check below ran the first 1 s of the same pile.
  - Gazebo's DART needs far more time per step once the boxes touch. The pile itself has no
    initial overlaps: the script checks the boxes' footprints.
- **Gazebo's quadcopter** ran in-process (gz.sim's Python `TestFixture`), because its controller
  flies only after an `/X3/enable` message. Odometry was read from a transport topic, so no
  Python ran inside the simulation loop.

What the numbers say:

- **Physics-only robot runs: Lagrange is faster here.** About 6.5 times Gazebo's real-time factor
  on the AMR, and twice on the quadcopter. Lagrange is also ready in a tenth of a second instead
  of 2.4 s, with about half the memory.
- **Lidars on the CPU: Lagrange is faster.** It uses all cores: 7.8 of them while stepping.
- **Lidars on the GPU: Lagrange's is slower than its own CPU path in this scene.** GPU lidar
  stepped at RTF 18.9 against 54.1 on the CPU, and used 279 MB against 78 MB. Use `--lidar cpu`
  (the default) for small scenes like this one.
- **Camera: Gazebo is faster, by about 15 %.** That is with Lagrange drawing colour, depth and
  an instance image at 8 samples per pixel against Gazebo's colour only. The image quality was
  not compared.
- **Many contacts.**
  - Lagrange's default MuJoCo backend is slow on the pile: RTF 0.17.
  - Against standalone MuJoCo 3.12 with its defaults (0.289), Lagrange's choice of the elliptic
    friction cone and the implicit-fast integrator costs about a third (0.184).
  - About 10 % comes from Lagrange's own overhead (0.166 against 0.184).
  - MuJoCo 4.0.1 runs the same scene 2.8 times faster than Lagrange's MuJoCo.
  - Lagrange's Jolt backend is the fastest measured here: 2.38, 14 times its MuJoCo backend. It
    loses energy in the pendulum test, though (below).
  - PyBullet is in between. Gazebo's DART did not finish.
- **Memory.** Gazebo's rendering adds 750 to 820 MB (883 to 961 MB with sensors against 137 MB
  without). Lagrange's camera adds about 320 MB, and its GPU lidar 200 MB.


## Determinism

Each engine ran the first 1 s of the 200-box pile twice, in two separate processes. The table
compares a SHA-256 of every box's final position and orientation, as exact bits.

| Engine | Run 1 | Run 2 | Bit-identical |
|---|---|---|---|
| Lagrange, MuJoCo backend | `bbc85ed6` | `bbc85ed6` | yes |
| Lagrange, Jolt backend | `4c510ca2` | `4c510ca2` | yes |
| Gazebo (DART) | `6c9fbab9` | not run | not known |
| MuJoCo, PyBullet | not run | not run | not known |

Missing runs:

- **Gazebo's second run** was stopped: DART needed over ten minutes for that 1 s of the pile.
- **The MuJoCo and PyBullet runs** never started, because measuring ended at that point.
- **Lagrange's own check** (`lagrange determinism profile_sensors.yaml --seconds 60`) was run
  only during the dry run. It printed "deterministic: 601 state hashes over 60.0 s identical in
  both runs" (`results/determinism_cli_dryrun.jsonl`).
- **Earlier 0.3 s dry runs** gave identical digests for all five engines. Their output was not
  kept as a result.

## Physics accuracy against closed forms

Six cases, each with a closed-form answer. The same bodies run in every engine with g = 9.81 m/s²
and a 1 ms step:

- **Free fall:** a 0.1 m cube from rest, its height after 1 s.
- **Pendulum period:** a 1 kg sphere (r = 0.02 m) on a massless 1 m rod at 0.05 rad. Its
  period is measured over 20 s and compared with the exact physical-pendulum period, by the
  complete elliptic integral. No small-angle approximation: 2.006541 s.
- **Pendulum energy:** the same pendulum from 1 rad, its total energy after 60 s.
- **Slide:** a 0.2 m cube on a 30° slab with Coulomb friction μ = 0.3 on both surfaces. Its
  acceleration is fitted from 0.5 s to 2 s and compared with g(sin θ − μ cos θ) = 2.3563 m/s².
- **Hold:** the same on 10°, below the friction angle: it should not move.
- **Rest:** a 2 kg cube resting on a slab. The contact force should be m·g = 19.62 N, averaged
  over 1.5–2 s. The table also gives how far the cube sinks into the slab.

Each engine reports the contact force its own way:

- Lagrange: `body/<b>/contact_force`.
- Gazebo: a contact sensor's wrenches.
- MuJoCo: `mj_contactForce`.
- PyBullet: `getContactPoints`' normal forces.

The cases are deterministic, so one run each.

| Engine | Free fall, z after 1 s: error | Pendulum period (0.05 rad): error | Pendulum energy after 60 s (1 rad) | Slide on 30°, μ 0.3: acceleration error | Hold on 10°: creep in 2 s | Resting box: contact force error | Resting box: sinks |
|---|---|---|---|---|---|---|---|
| Lagrange, MuJoCo backend | −4.905 mm | −0.41 ppm | −0.07 % (worst 0.15 %) | −136 ppm | 0.916 mm | 0 | 0.108 mm |
| Lagrange, Jolt backend | −4.898 mm | −0.02 ppm | **−16.4 %** (worst 16.4 %) | −4.0 ppm | 0.001 mm | 0 (by construction, see below) | 0.0002 mm |
| Gazebo Jetty (DART) | −4.905 mm | −0.41 ppm | −0.07 % (worst 0.15 %) | −0.0 ppm | 0.000 mm | +0.1 ppm | 0.0000 mm |
| MuJoCo 3.12.0 standalone | −4.905 mm | −0.41 ppm | −0.07 % (worst 0.15 %) | +124 ppm | 1.282 mm | −0.000007 ppm | 0.108 mm |
| MuJoCo 4.0.1 standalone | −4.905 mm | −0.41 ppm | −0.07 % (worst 0.15 %) | +124 ppm | 1.282 mm | −0.000007 ppm | 0.108 mm |
| PyBullet 3.2.7 | −4.905 mm | −0.41 ppm | −0.07 % (worst 0.15 %) | −0.1 ppm | 0.036 mm | −0.000006 ppm | 0.010 mm |

What this says:

- **Free fall.** Every engine is off by the same −4.905 mm. That is not an engine error: it is
  semi-implicit Euler at a 1 ms step, which falls exactly g·Δt·t/2 further than the closed form.
  Jolt's −4.898 mm is the same integrator in single precision: Lagrange builds Jolt with
  `DOUBLE_PRECISION=OFF`.
- **Pendulum period.** Every engine is within half a part per million of the exact period.
- **Pendulum energy.** This is the clearest difference found. MuJoCo, DART, Bullet and Lagrange
  on MuJoCo keep the energy within 0.15 % over 60 s. **Lagrange on Jolt loses 16 % of it.**
  Lagrange zeroes Jolt's linear and angular body damping, so the loss comes from the hinge
  constraint's solver; the exact mechanism was not investigated here. Lagrange's own energy test
  (`Pendulum.EnergyDriftAndLargeAnglePeriod`, under 0.1 % over 10 s) covers the MuJoCo backend
  only. Use MuJoCo, the default and the reference, when energy over long runs matters.
- **Slide.** Every engine is within 0.014 % of g(sin θ − μ cos θ).
  - Lagrange's MuJoCo backend uses MuJoCo's elliptic friction cone, and is 136 ppm low.
  - Standalone MuJoCo, with its default pyramidal cone, is 124 ppm high.
  - DART, Bullet and Jolt are within 4 ppm.
- **Hold.** MuJoCo's contacts are soft, so a box that should stay put creeps about 1 mm in 2 s:
  0.9 mm in Lagrange and 1.3 mm standalone. DART, Bullet and Jolt hold it to a few
  micrometres or less. MuJoCo's resting box also sinks 0.1 mm into the slab, Bullet's 0.01 mm.
- **Resting force.** All engines give m·g to well under a part per million. Lagrange's Jolt
  backend computes contact force from Newton's second law on each body
  ([rigid body physics](../physics/rigid-body.md)), so a body at rest gives m·g by construction.
  That row shows consistency, not an independent check.
- **Lagrange adds no error of its own.** On MuJoCo it matches standalone MuJoCo 3.12, the same
  library, in every case except the two friction cases. The difference there is Lagrange's
  choice of the elliptic cone.

## Features (cited, not measured)

Each cell comes from the project's own documentation, repository or package index. The sources
are listed under the table. GitHub stars are the repository's count on 2026-11-14 (GitHub API).
*From their documentation, not measured here*, except the Lagrange column, which is this
repository.

| | Lagrange 1.3 | Gazebo Jetty | Isaac Sim 6.x | Webots | CARLA | MuJoCo | PyBullet |
|---|---|---|---|---|---|---|---|
| Licence | Apache-2.0 | Apache-2.0 [g1] | Apache-2.0 for the repository; it needs the Omniverse Kit SDK and assets under NVIDIA's own licence [i1] | Apache-2.0 [w1] | MIT (code), CC-BY (assets); Unreal Engine under its own terms [c1] | Apache-2.0 [m1] | zlib [p1] |
| Platforms | Ubuntu 26.04 only (amd64; arm64 builds, CPU tests) | Ubuntu 26.04 and 24.04, macOS (arm64); Windows experimental [g2] | Ubuntu 22.04 (deprecated) and 24.04, Windows 11; RTX GPU required, minimum RTX 4080 [i2] | Linux (Ubuntu 22.04 to 26.04), Windows 10 and 11, macOS 12 to 14 [w2] | Ubuntu 20.04/22.04 and Windows 10/11 (0.9.x); Ubuntu 22.04/24.04 and Windows 11 (Unreal Engine 5 branch) [c2] | Linux, Windows, macOS [m1] | Linux, Windows, macOS (PyPI classifiers) [p2] |
| Physics engines | MuJoCo 3.12 (default) and Jolt 5.6 (single precision); others as C-ABI plugins | DART by default; Bullet, Bullet-featherstone and trivial-physics-engine plugins [g3] | PhysX by default; Newton experimental [i3] | ODE (a fork) [w3] | Unreal Engine (Chaos for vehicles in 0.10) [c3] | its own [m2] | Bullet [p3] |
| Sensor types | 8 built-in types and 36 built-in kinds ([sensors guide](../guide/sensors.md)) | 17 in gz-sensors, plus contact and others as systems [g4] | about 12 [i4] | 14 generic sensor nodes [w4] | 18 [c4] | 49 sensor elements, mostly state readouts; images from the renderer [m3] | camera images, ray tests, joint force-torque; no sensor framework [p3] |
| Radar | yes, and a 4D imaging radar kind | not found [g4] | yes (RTX Radar) [i4] | yes [w4] | yes [c4] | not found | not found |
| Thermal camera | yes, temperatures from the thermal domain | yes; temperatures are set per entity, no heat model [g5] | not found | not found | not found | not found | not found |
| Lidar intensity | yes, from surface material, incidence and range | an intensity field; whether it models reflectivity: not found [g6] | yes, from material [i5] | not found | from distance only [c4] | not found | not found |
| Weather on sensors | yes: fog, rain, snow and dust on lidars and cameras | particle emitters scatter lidar ranges [g7] | not found | a Fog node in rendering; effect on sensors not stated [w5] | fog and wetness on the RGB camera only [c5] | not found | not found |
| Electrical, thermal, EMI | battery, bus, motor drives (100 µs), thermal networks and conduction, EMI fields and coupling | linear battery plugin; fixed temperature per entity; EMI not found [g8] | not found | battery as an energy counter [w6] | not found | not found | not found |
| ROS 2 | built-in bridge (ROS 2 Lyrical), `simulation_interfaces`, ros2_control | `ros_gz`; Lyrical with Jetty recommended [g9] | ROS 2 bridge extension; Humble and Jazzy recommended [i6] | `webots_ros2` [w7] | native ROS 2 interface and a ROS bridge [c6] | not found | not found |
| Determinism claim | a CPU reference mode, bit-identical on one platform | not found | stepping "deterministically" in scripted workflows; no bit-reproducibility claim found [i7] | reproducible with a fixed seed and synchronised supervisor calls [w8] | "physics and collision determinism under specific circumstances" [c7] | "entirely deterministic and reproducible", with caveats [m4] | after restoring saved state, if contact points are saved too [p3] |
| Ecosystem | 11 sample robot folders, 12 sample worlds and 18 example packages in this repository; 0 GitHub stars | 3,455 models and 277 worlds on Fuel [g11]; gz-sim 1,578 stars | "more than 240 robot configurations" [i9]; 4,477 stars | 4,769 stars | 14,603 stars | 17,926 stars | 14,859 stars (bullet3) |
| First release | 2026-10-14 (v0.1.0) | Gazebo 2002 (first stable 2012); new Gazebo 2019 [g12] | Omniverse-based preview 2020 [i10] | 1998 [w3] | 2017 [c8] | open-sourced 2022 (earlier as a commercial product) [m2] | first PyPI upload 2017 [p2] |

Sources, all read on 2026-11-14:

- **Gazebo:**
  - [g1] [gz-sim repository](https://github.com/gazebosim/gz-sim) (licence field).
  - [g2] [Getting started, Jetty](https://gazebosim.org/docs/jetty/getstarted/).
  - [g3] [gz-physics 9 plugins](https://gazebosim.org/api/physics/9/physicsplugin.html) and the
    [physics tutorial](https://github.com/gazebosim/gz-sim/blob/gz-sim10/tutorials/physics.md): "Gazebo
    uses the DART physics engine by default". Bullet and Bullet-featherstone have "preliminary
    support". The run logs here show `gz::physics::dartsim::Plugin` loaded.
  - [g4] [gz-sensors 10 README](https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/README.md)
    and the [gz-sim 10 systems](https://github.com/gazebosim/gz-sim/tree/gz-sim10_10.6.0/src/systems).
  - [g5] [Thermal system](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/thermal/Thermal.hh).
  - [g6] [GpuLidarSensor.cc](https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/src/GpuLidarSensor.cc).
  - [g7] [Particle emitter](https://gazebosim.org/api/sim/10/particle_emitter.html).
  - [g8] [Battery](https://gazebosim.org/api/sim/10/battery.html).
  - [g9] [ROS installation, Jetty](https://gazebosim.org/docs/jetty/ros_installation/).
  - [g10] the systems directory in [g4].
  - [g11] the [Fuel models](https://fuel.gazebosim.org/1.0/models?per_page=1) and
    [worlds](https://fuel.gazebosim.org/1.0/worlds?per_page=1) API (`X-Total-Count`).
  - [g12] [Gazebo classic](https://classic.gazebosim.org/) and the
    [releases page](https://gazebosim.org/docs/jetty/releases/).
- **Isaac Sim:**
  - [i1] [LICENSE](https://github.com/isaac-sim/IsaacSim/blob/main/LICENSE).
  - [i2] [requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
  - [i3] [Newton physics](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/physics/newton_physics.html).
  - [i4] [sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/index.html).
  - [i5] [RTX sensor materials](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_materials.html).
  - [i6] [ROS 2 tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html).
  - [i7] [simulation tips](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_simulation/robot_simulation_tips.html).
  - [i8] [performance handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/reference_material/sim_performance_optimization_handbook.html).
  - [i9] [assets overview](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_overview.html).
  - [i10] [2020.1 preview](https://developer.nvidia.com/blog/isaac-sim-2020-1-preview/).
  - [i11] [releases](https://github.com/isaac-sim/IsaacSim/releases).
- **Webots:**
  - [w1] [foreword](https://github.com/cyberbotics/webots/blob/master/docs/guide/foreword.md).
  - [w2] [system requirements](https://github.com/cyberbotics/webots/blob/master/docs/guide/system-requirements.md).
  - [w3] [cyberbotics.com](https://cyberbotics.com/).
  - [w4] [sensors](https://github.com/cyberbotics/webots/blob/master/docs/guide/sensors.md).
  - [w5] [Fog](https://github.com/cyberbotics/webots/blob/master/docs/reference/fog.md).
  - [w6] [Robot](https://github.com/cyberbotics/webots/blob/master/docs/reference/robot.md).
  - [w7] [webots_ros2](https://github.com/cyberbotics/webots_ros2).
  - [w8] [WorldInfo](https://github.com/cyberbotics/webots/blob/master/docs/reference/worldinfo.md)
    and [Supervisor](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md).
  - [w9] [introduction](https://github.com/cyberbotics/webots/blob/master/docs/guide/introduction-to-webots.md).
  - [w10] [releases](https://github.com/cyberbotics/webots/releases).
- **CARLA:**
  - [c1] [repository](https://github.com/carla-simulator/carla).
  - [c2] [quick start](https://carla.readthedocs.io/en/latest/start_quickstart/).
  - [c3] [0.10.0 release](https://carla.org/2024/12/19/release-0.10.0/).
  - [c4] [sensors reference](https://carla.readthedocs.io/en/latest/ref_sensors/).
  - [c5] [weather parameters](https://github.com/carla-simulator/carla/blob/ue4-dev/PythonAPI/docs/weather.yml).
  - [c6] [ROS](https://carla.readthedocs.io/en/latest/ecosys_ros/).
  - [c7] [synchrony and time-step](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/).
  - [c8] [releases](https://github.com/carla-simulator/carla/releases).
- **MuJoCo:**
  - [m1] [README](https://github.com/google-deepmind/mujoco).
  - [m2] [overview](https://github.com/google-deepmind/mujoco/blob/main/doc/overview.rst).
  - [m3] [XML reference](https://github.com/google-deepmind/mujoco/blob/main/doc/XMLreference.rst).
  - [m4] [computation](https://github.com/google-deepmind/mujoco/blob/main/doc/computation/index.rst).
  - [m5] [MuJoCo Warp](https://github.com/google-deepmind/mujoco/blob/main/doc/mjwarp/index.rst).
  - [m6] [releases](https://github.com/google-deepmind/mujoco/releases).
- **PyBullet:**
  - [p1] [LICENSE.txt](https://github.com/bulletphysics/bullet3/blob/master/LICENSE.txt).
  - [p2] [PyPI](https://pypi.org/pypi/pybullet/json).
  - [p3] [quick start guide](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstartguide.pdf).

GitHub stars come from `gh api repos/OWNER/REPO` for gazebosim/gz-sim, isaac-sim/IsaacSim,
cyberbotics/webots, carla-simulator/carla, google-deepmind/mujoco, bulletphysics/bullet3 and
Open-Physical-Systems/Lagrange. Stars measure attention, not quality. The fair point is that
Lagrange has no community yet.

## Pros and cons

### Lagrange

Better at:

- **Fast headless runs.** In these scenes it reached 2 to 6.5 times Gazebo's real-time factor
  on robot physics and lidars. It starts in a tenth of a second and uses a fraction of the
  memory.
- **Coupled domains.** Battery, motor drives, thermal networks, heat conduction, EMI, radio
  links, weather and air flow run beside the rigid bodies, and any of them can be swapped for a
  stand-in. None of the other simulators here has this; Gazebo's battery is a linear
  open-circuit voltage and its thermal camera reads fixed temperatures.
- **Sensors that react to the environment.** Weather on lidars and cameras, lidar intensity from
  materials, thermal cameras that read the thermal domain, radar and an event camera.
- **Determinism you can check.** `lagrange determinism` compares state hashes of two runs, and
  both of its backends repeated the box pile bit for bit (below).
- **Accuracy matching its engines.** On MuJoCo it adds no error of its own.

Worse at, or missing:

- **Young.** The first release was on 2026-10-14, a month before this page. It has a small user base,
  no community yet and no third-party plugins. Gazebo has been developed since
  2002 and has thousands of models on Fuel.
- **One platform.** Ubuntu 26.04 with ROS 2 Lyrical only. No macOS, no Windows, no older
  Ubuntu or ROS distributions. Gazebo Jetty supports Ubuntu 24.04 and macOS as well, and its
  older releases cover older ROS distributions.
- **No real-robot validation from its own robots.** Its owner has no hardware to record.
  Models are checked against closed forms, published datasets and OpenFOAM
  ([validation report](report.md)), not against a robot it drove.
- **Some sensor defaults are illustrative.** Where no datasheet was found, the default
  parameters are marked *illustrative* in [sensors.md](../design/domains/sensors.md). Give your
  part's figures.
- **Rendering.** Its own Vulkan renderer is not Isaac Sim's RTX path tracing or CARLA's Unreal
  Engine for photorealism. Training perception on its images is not a validated use.
- **No PhysX**, and so no GPU-parallel rigid bodies for reinforcement learning at scale (Isaac
  Sim, MuJoCo's MJX and Warp).
- **The Jolt backend loses energy** in the pendulum test: 16 % over 60 s. Stay on MuJoCo, the
  default, when that matters.
- **The quadcopter needed its electrical domain.** With `electrical` switched to its ideal
  substitute, the sample quadcopter's rotors did not turn and it never took off (found while
  preparing this page; fixed on `dev` after v1.3.0, test
  `Multicopter.HoversWithTheElectricalDomainOff`). Its sample profile runs the motors at 50 µs,
  which is most of a hover's cost. Its [release notes](../release/v1.3.0.md) also note the
  quadcopter run is 3 to 5 % slower than before its last performance round.
- **Uses an older MuJoCo.** It ships MuJoCo 3.12. MuJoCo 4.0.1 steps the 200-box pile much
  faster (below).
- **Slower on the measured camera** than Gazebo, by about 15 %, though it draws more per frame.
- **Its GPU lidar is slower than its CPU lidar** in a small scene: RTF 18.9 against 54.1.
- **Its default MuJoCo backend is slow with many contacts.** The 200-box pile ran at RTF 0.17,
  14 times slower than its Jolt backend and 2.8 times slower than MuJoCo 4.0.1. The cost is
  MuJoCo 3.12 with Lagrange's elliptic friction cone, not the framework.

### Gazebo Jetty

- **Better:**
  - Maturity: over twenty years of Gazebo, the de-facto ROS simulator.
  - Thousands of Fuel models and worlds, many tutorials, a large community.
  - Many robot descriptions already ship Gazebo plugins.
  - More platforms (Ubuntu 24.04 and 26.04, macOS).
  - A GUI editor.
  - DART's hard contacts hold a box on a slope with no creep at all (accuracy table).
- **Worse:**
  - Slower in every physics and lidar scene measured here.
  - A 2.4 to 2.7 s start-up.
  - Around 900 MB once its sensors render.
  - It did not finish 5 s of the 200-box pile in 30 minutes.
  - It delivered only 330 to 349 of 600 scans per lidar over 60 s when run as fast as possible.
  - Sensors render only for a subscriber.
  - No electrical, thermal or EMI domains beyond a linear battery and fixed temperatures.
  - No bit-for-bit determinism claim was found in its documentation. Its second determinism run
    here was stopped, so this page cannot say.

### MuJoCo (standalone)

- **Better:**
  - The fastest and most widely used physics library for control and learning.
  - Determinism stated in its documentation.
  - GPU versions (MJX, Warp) for many parallel worlds.
  - Version 4 is much faster than 3.12 on contact-heavy scenes.
- **Worse:**
  - A library, not a robot simulator: no ROS 2, no lidars, cameras only through its renderer,
    no worlds.
  - Soft contacts creep: about 1 mm in 2 s on a slope that should hold.

### PyBullet

- **Better:**
  - Simple Python API, quick to set up.
  - Exact on these closed forms.
- **Worse:**
  - The last PyPI release was January 2025 and the last bullet3 GitHub release 2022.
  - No sensor framework and no ROS 2 bridge.
  - Physics on the CPU only.
  - Defaults to watch: 0.04 linear and angular damping, a velocity motor on every joint, and
    friction coefficients multiplied between the two surfaces.

### Isaac Sim, Webots, CARLA (not measured here)

*From their documentation, not measured here:*

- **Isaac Sim:** RTX-rendered sensors and GPU physics. It needs an RTX GPU (minimum RTX 4080 in
  its requirements) and parts under NVIDIA's licence.
- **Webots:** the broadest platform support (Linux, Windows, macOS), long history (1998), and
  an ODE fork for physics.
- **CARLA:** the reference for driving research. Unreal Engine, an event camera, radar,
  weather on its RGB camera. Cars only.

## Limitations of this comparison

- **One machine, one day.** A laptop with a mobile GPU. A desktop or server GPU, or another CPU,
  will move the ratios. Laptop clocks also vary with temperature, so each run records them.
- **Small scenes.** One robot, a seven-box warehouse, one camera, 200 boxes and one quadcopter.
  Large worlds, many robots, articulated contact-rich robots (hands, legs) and long runs are not
  covered.
- **Different engines under the same name.** Gazebo stepped DART, and Lagrange stepped MuJoCo or
  Jolt. Most of a physics-time difference is the engine, not the framework around it. Gazebo
  with MuJoCo is not an option in Jetty, so the like-for-like comparison does not exist.
- **Different sensors under the same name.**
  - Gazebo's lidars render depth on the GPU and see visual geometry. Lagrange's CPU lidar casts
    exact rays against collision geometry; its GPU lidar runs a compute kernel. In this
    warehouse the visual and collision geometry are the same boxes.
  - Lagrange's camera writes colour, depth and an instance image for every frame, with its
    "realistic" look (8 samples per pixel). Gazebo's camera writes colour only. Image quality
    was not compared.
- **The quadcopters differ.** Gazebo's X3 is the model from its multicopter example: four
  first-order motor models and a velocity controller. Lagrange's is its sample X500 with a
  battery and four motor drives on an electrical network at 50 µs. Only the order of magnitude
  of that row is comparable.
- **Gazebo needs a subscriber to render.** Its sensor rows had a subscriber process on each
  lidar or camera topic, counting messages. The first messages arrive late, while the
  subscription connects, so "received" can be a little under what was simulated.
- **Lagrange ran without ROS, Gazebo without `ros_gz`.** Neither bridge cost is in the numbers.
  Gazebo's runs did include its transport and scene broadcaster, as a user's would.
- **Start-up is not the same thing in both.**
  - Lagrange's includes starting Python and loading the bindings.
  - Gazebo's includes its Ruby front end and its shutdown.
  - Neither includes the first launch on a cold disk.
- **Accuracy cases are simple on purpose.** They check the integrator and the contact and
  friction models against closed forms. They say nothing about articulated chains, stacking
  stability, grasping, or how realistic a robot's sensors are. Lagrange's own validation against
  published and measured data is in the [validation report](report.md).
- **Not compared at all:** rendering realism (Isaac Sim and CARLA are built on RTX and Unreal
  Engine; Lagrange's renderer is its own), ease of use, documentation quality, editor and GUI,
  community support.

## Reproduce

All scripts are in
[`docs/release/comparison/`](https://github.com/Open-Physical-Systems/Lagrange/tree/main/docs/release/comparison),
with the raw results of this page in its `results/` folder. On Ubuntu 26.04 with ROS 2 Lyrical,
Lagrange built and sourced, and `ros-lyrical-ros-gz` installed:

```bash
mkdir -p ~/compare && cd ~/compare
python3 -m venv venv && venv/bin/pip install mujoco pybullet numpy
python3 -m venv venv312 && venv312/bin/pip install mujoco==3.12.0 numpy
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4"
source /opt/ros/lyrical/setup.bash && source ~/lagrange_ws/install/setup.bash
~/lagrange_ws/src/lagrange/docs/release/comparison/run.sh ~/compare/results 5
```

`run.sh` writes:

- `machine.txt`;
- the accuracy results (`accuracy_*.json`);
- one line per timed run (`performance.jsonl`: the command, the load average and the GPU state
  at its start, and what the run printed);
- the determinism digests (`determinism.jsonl`);
- the tables of this page (`tables.txt`, from `summarize.py`).

`QUICK=1` makes a dry run of a few minutes. A full run took about an hour here, most of it the Gazebo 200-box run, which
`run_all.py` stops after 30 minutes and runs once. `run_all.py --resume` continues an
interrupted set of runs.
