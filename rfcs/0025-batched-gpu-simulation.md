# RFC 0025: Batched simulation on the GPU for reinforcement learning

| | |
|---|---|
| Status | Draft (for the owner) |
| Created | 2026-11-22 |
| Owner | Lagrange core maintainers; the project owner accepts or refuses |
| Affects | `python/` (a new `lagrange.batch` module and `lagrange.envs.BatchedVectorEnv`), `modules/rigid_body/mujoco_backend/` (writes its compiled scene as MJCF text), `core/signals/` (device-resident batch signals, RFC 0001 open question 6), `core/scheduler/` (a batched schedule), `gpu/kernels/batch/` (new Slang kernels for the reduced domains), `profiles/schema.json` (one new optional key, `batching`), `docs/` |
| Not affected | `schemas/` (nothing new on the stage), `proto/` (the viewer does not watch a batch, open question 7), `core/plugin_abi/` (plugins do not run in a batch, §6), the C++ server's single-environment path, the CPU reference |
| Builds on | RFC 0001 (modules, `batchCount`, open question 6), RFC 0004 (tiers), RFC 0005 (profiles: `batch`), RFC 0007 (the 1.x freeze), RFC 0017 (MuJoCo 4.x), v1.6.0 plan item A4 |
| Review | Touches `profiles/schema.json` (an addition) and makes an exception to invariant 8 inside a third-party engine (§5.3): core-maintainer review |

## Summary

Run thousands of copies of one robot task on one RTX-class GPU (Tier 2) for reinforcement
learning, with the observations, rewards and actions staying on the GPU as tensors that torch
reads without a copy. Rigid-body physics is MuJoCo Warp (Apache-2.0, maintained by Google
DeepMind and NVIDIA), integrated rather than built (invariant 12). Lagrange keeps what it is for:
every importer and the stage loader build the model, the same MuJoCo options and units apply, and
the other domains run beside physics in **reduced, batched forms** written as Slang kernels
(averaged electrical as closed-form formulas, coefficient rotors, an IMU, ray-cast lidar). A
domain without a batched form must be off with a substitute, or the batch refuses to start, as a
unit mismatch does today. The batched path is a GPU path: it may be non-deterministic
(invariant 7), and it is checked against the CPU reference statistically and on short
trajectories, not bit for bit. Everything is an addition under RFC 0007: the existing `batch`
key keeps its meaning, a new optional `batching` key chooses the device, and the Python API
gains a class.

A spike on this machine (RTX 4060 Laptop, physics only) measured MuJoCo Warp 4.0.1 at 5.5
million environment steps per second for the sample arm and 6.9 million for the sample quad
with 4096 environments, against 0.1 to 0.3 million on one (shared) CPU core of MuJoCo 4.0.1, and
against about 11 000 `LagrangeEnv` steps per second today for the AMR with all its domains
(§Spike: these are not like for like).

## Motivation

- **Scale.** On-policy RL for locomotion and manipulation is trained with thousands of
  environments in parallel. Lagrange gives one `Simulation` per process: `LagrangeEnv` reaches
  about 1 400 steps per second at 20 ms and 11 100 at 2 ms for the sample AMR, and
  `ParallelEnvs` with four workers 1.5 to 1.8 times one worker, because each worker's CPU lidar
  already uses every core (docs/design/benchmarks.md, "Python throughput"). That is two to three
  orders of magnitude below what a GPU-batched simulator gives for the same robot.
- **The wedge.** Batched physics alone exists elsewhere (survey below). What no batched
  simulator has is the battery that sags, the motor that heats, the rotor that loses thrust in
  thin air, the magnetometer that sees the motor leads. Training with those in the loop, even
  in reduced form, is the reason to do this in Lagrange rather than send users to MuJoCo
  Playground or Isaac Lab.
- **Promised.** RFC 0001 reserves `batchCount` on every signal and leaves "GPU modules" (open
  question 6) for "a later RFC before Tier 2 batched physics"; RFC 0005 has a `batch` key that
  does nothing today (docs/guide/profiles.md: "not yet: a warning"); the v1.6.0 plan's item A4
  asks for this RFC.

### What exists

| Piece | Where | What batching gets from it |
|---|---|---|
| `batchCount` and whole-batch access (`readBatch`, `writeBatch`, `[env][component]`) | `core/signals/SignalBus.hpp` | the layout of batch signals, today in host memory |
| `BufferHandle` signals with lazy buffers | `core/signals/Signal.hpp` | the pattern for device-resident batch signals |
| The MuJoCo backend builds an `mjSpec` from the world tables (elliptic cone, `implicitfast`) | `modules/rigid_body/mujoco_backend/src/MujocoBackend.cpp` | the batched model, written out as MJCF |
| `LagrangeEnv`, `ParallelEnvs`, `VectorEnv` (Gymnasium interface, `randomize=`) | `python/lagrange/envs.py` | the observation, action and reward conventions to keep |
| `save_state`, `restore_state`, `state_hash`, `lagrange determinism` | Python API, `lagrange_server` CLI | the CPU reference the batch is checked against |
| Slang kernels compiled for Vulkan (lidar BVH, cameras, LBM) | `gpu/kernels/`, `gpu/hal/` (Vulkan only today) | the language of the reduced domains |

## Goal and scale

| | Target |
|---|---|
| Hardware | one NVIDIA RTX-class GPU with 8 GB or more (Tier 2, RFC 0004); compute capability 7.5 or newer and driver R580 or newer (Warp's wheels since 1.18, per Newton's README) |
| Environments | 1 to 4096 of one robot and one world per batch (the same model; per-environment parameters, §4.4) |
| Throughput (physics only, 1 ms step) | at least 10⁶ environment steps per second at N = 4096 for the sample arm and quad on an RTX 4060 Laptop; the spike gives 5.5 to 6.9 × 10⁶ |
| Throughput (with the reduced domains) | at least half of physics only (to be measured in step 4) |
| Observations | on the device, `[N, obs]` `float32`, zero-copy to torch through DLPack |
| CPU fallback | the same API on the CPU (MuJoCo Warp's CPU device, or the existing `ParallelEnvs`) for development, at whatever speed it gives |

Non-goals: heterogeneous batches (different robots in one batch), streamed huge worlds, cameras
in the first phases, viewers watching a batch, ROS 2 in a batch, plugins (C ABI) in a batch,
differentiable simulation (MuJoCo Warp does not offer it yet, its README says).

## Engine survey

Checked on 2026-11-22 from PyPI's JSON API (version, date, Python and dependency pins), the
projects' GitHub repositories (licence file, releases) and their own documentation
(`doc/mjwarp/index.rst` and `doc/mjx.rst` in google-deepmind/mujoco at `main`, the READMEs).
"TODO(verify)" marks what these sources did not settle.

| | MuJoCo Warp (MJWarp) | MJX (JAX) | Newton | Genesis World | Isaac Lab / PhysX GPU | Brax |
|---|---|---|---|---|---|---|
| Licence | Apache-2.0 (LICENSE) | Apache-2.0 | Apache-2.0 (docs CC-BY-4.0); Linux Foundation project | Apache-2.0 (LICENSE) | Isaac Lab BSD-3-Clause; PhysX SDK BSD-3-Clause; Isaac Sim proprietary ("NVIDIA Proprietary Software" on PyPI), not needed for kit-less Newton workflows | Apache-2.0 |
| Latest release | 4.0.1, 2026-11-11 | `mujoco-mjx` 4.0.1, 2026-11-11 | 1.8.0, 2026-11-11 | 1.4.7, 2026-11-17 | Isaac Lab 3.0.0, 2026-10-20 (PyPI 2026-10-19); Isaac Sim 6.2.0.0, 2026-10-13; `ovphysx-0.8.0` tag, 2026-11-13 | 0.14.2, 2026-03-15 |
| Maturity | PyPI classifier "Development Status :: 3 - Alpha"; maintained by Google DeepMind and NVIDIA as part of Newton | MJX-JAX stable for years (TODO(verify) classifier); the docs point NVIDIA users to MJX-Warp | PyPI "5 - Production/Stable" | academic since Dec 2024, now backed by Genesis AI; no status classifier | 3.0 "generally available" (release notes) | Only `brax/training` maintained since 0.13.0; its README sends physics users to MJX or MuJoCo Warp |
| Contacts | all MuJoCo geoms and colliders, BVH for meshes, hfield, flex; fixed-size contact and constraint buffers per world (`nconmax`, `njmax`), overflow flagged per world | MJX-JAX: plane, hfield, sphere, capsule, box, mesh; ellipsoid and cylinder only against primitives; no SDF | through its solvers, MuJoCo Warp the primary one | rigid solver with contacts, plus FEM, MPM, PBD/SPH, IPC (README); details TODO(verify) | PhysX 5 GPU rigid bodies and articulations; Newton backend (MuJoCo Warp, VBD, MPM) | its own pipelines (generalized, positional, spring) or MJX |
| Tendons | all, all wrapping | fixed and spatial; joint, site, pulley, sphere, cylinder wrapping | via MuJoCo Warp (TODO(verify) in its other solvers) | TODO(verify) | PhysX fixed and spatial tendons (TODO(verify) on GPU in 3.0) | not in its own pipelines (TODO(verify)) |
| Actuators | all dynamics, gains, biases and transmissions except plugin actuators | dynamics none, integrator, filter, filterexact, muscle; gain fixed, affine, muscle | via MuJoCo Warp | PD and force control (TODO(verify)) | Isaac Lab actuator models (implicit PD, DC motor, networks; TODO(verify) list in 3.0) | — |
| Sensors | all MuJoCo sensors except plugin sensors; batched ray casting; a batch ray-tracing renderer (RGB, depth; optional OptiX, Python 3.12, not in CUDA graphs) | a fixed list (IMU, frame, joint, touch, contact, force/torque, rangefinder, …) | TODO(verify) | cameras (Nyx, Luisa, Pyrender), others TODO(verify) | Isaac Lab sensors; RTX cameras with Isaac Sim or OVRTX | — |
| Integrators, solvers | all integrators (`implicitfast` without its midpoint feature); CG and Newton (no PGS, no noslip) | Euler, RK4, implicitfast; CG, Newton | several (XPBD, VBD, MPM, MuJoCo Warp; list TODO(verify)) | TODO(verify) | PhysX TGS/PGS (TODO(verify)) | — |
| MuJoCo version needed | `mujoco-core >= 4.0.0` (and versioned with MuJoCo: 3.15.0 on 2026-10-05, 4.0.0 on 2026-11-06) | `mujoco-core[rendering] >= 4.0.1.dev0`; `mujoco-mjx[warp]` pins `warp-lang == 1.18.0` | the `sim` extra pins `mujoco ~= 3.17.0` and `mujoco-warp ~= 3.17.0` | `mujoco >= 3.2.5` (for its MJCF reader) | none | `mujoco` (unpinned) |
| Python | ≥ 3.11 (ran on 3.13.15 here) | ≥ 3.11; `jax` 0.12.1 needs ≥ 3.12 | ≥ 3.11 (README; PyPI says ≥ 3.10) | ≥ 3.12, < 3.15 | Isaac Lab ≥ 3.12, < 3.13; Isaac Sim 3.12 only | ≥ 3.11 |
| CUDA | Warp (`warp-lang >= 1.15`); no CUDA toolkit; Warp ≥ 1.18 wheels need compute capability 7.5 and driver R580+ | JAX: `jax[cuda12]` or `jax[cuda13]` plugins | Warp ≥ 1.17.0; same driver rule; `torch-cu12`/`torch-cu13` extras | Quadrants compiler: CUDA, ROCm, Metal, Vulkan, x86, ARM64 (README); torch installed by the user (TODO(verify) its CUDA) | Isaac Lab 3.0: PyTorch 2.11 with CUDA 13.0, Warp 1.17 | as MJX |
| C++ usable from Lagrange | no: Python and Warp kernels JIT-compiled at run time | no: JAX/XLA | no: Python/Warp | no: Python | PhysX SDK yes (C++, BSD-3); Isaac Lab no | no |
| Per-world model parameters (domain randomisation) | yes: model fields can take an `nworld` dimension (documented with `body_mass`, `geom_size`, `dof_damping`) | yes (vmap over model) | yes (TODO(verify) API) | yes (TODO(verify) API) | yes | yes |
| Determinism | an option `deterministic` (bitmask `DeterminismType`; what it guarantees TODO(verify)) | XLA on GPU: TODO(verify) | TODO(verify) | TODO(verify) | PhysX GPU: TODO(verify) | — |

What the table says:

- **MuJoCo Warp is the one to integrate.** It is MuJoCo, which Lagrange already uses and
  validates (spike 0.2-C1, `lagrange_physics_conformance`), with nearly every feature the backend
  uses (elliptic cones, `implicitfast`, joint equalities for mimic joints, damping, friction loss,
  mesh hulls, height fields). Its gaps (plugins, PGS, noslip, implicitfast's midpoint) are not
  used by the backend (TODO(verify) against the backend's options once more in step 1). Its
  licence is Apache-2.0 with no proprietary piece.
- **MJX-JAX** has the narrowest features (no cylinder–box contacts, which the sample robots
  use) and pulls in JAX/XLA; MJX-Warp is MuJoCo Warp behind a JAX interface. Not chosen; users
  who want JAX can take MJX on the same exported MJCF.
- **Newton** is the framework MuJoCo Warp belongs to; it adds other solvers and USD support. It
  pins an older MuJoCo (3.17 while MuJoCo Warp itself is at 4.0.1). Integrating MuJoCo Warp
  directly keeps one dependency instead of two; Newton's other solvers (cloth, cables, MPM) are
  a later question (open question 3).
- **Genesis World** brings its own compiler (Quadrants) and its own rigid solver; adopting it
  would mean a second physics engine whose results Lagrange has not validated. Isaac Lab's PhysX
  GPU path needs either Isaac Sim (proprietary terms, not redistributable here, dependencies.md
  §8) or the new standalone `OVPhysX` (licence of its binaries TODO(verify)); Lagrange's own
  PhysX backend is still not built. Brax as a simulator is retired by its authors.

## Design

### 1. Where it runs: a Python-hosted batch beside the C++ server

MuJoCo Warp is Python with kernels JIT-compiled by Warp. It cannot be linked into the C++ core
(dependencies.md §8 already says so for Warp, Newton, MuJoCo Warp and Genesis). The batch
therefore runs in the Python process, as `lagrange.batch`, and the C++ side only builds:

1. **Build once, in C++.** `lagrange.batch.BatchSimulation(robot, world=..., profile=..., n=4096)`
   asks the nanobind module to load the robot and world exactly as `Simulation` does (importers,
   overlays, the stage, world tables, the profile) and to return:
   - the MuJoCo backend's `mjSpec` written as MJCF text (`mj_saveXMLString`; TODO(verify) the
     exact function in MuJoCo 3.12 and that it keeps every option the backend sets). Text, not
     an `mjModel`, so Lagrange's MuJoCo (3.12.0 from `mujoco_vendor`) and MuJoCo Warp's
     (`mujoco-core >= 4.0.0` in the user's Python) need not be the same version (open
     question 1, RFC 0017);
   - the batched form of every enabled domain (§3): its parameters as flat arrays (the motor
     constants, battery curve, rotor coefficients, sensor noise), its `SignalSpec`s with units,
     and its step;
   - the wiring the scheduler built, unit-checked as for any run (invariant 2: a mismatch fails
     here, before a GPU is touched).
2. **Step on the GPU, in Python.** `mujoco_warp.put_model` and `make_data(nworld=n, nconmax,
   njmax)`; the reduced domains' Slang kernels and MuJoCo Warp's `step` are launched in the
   batched schedule (§2) on one CUDA stream and captured into one CUDA graph; each `step()` of
   the environment replays the graph a fixed number of times.
3. **No server, no ROS, no viewer** in a batch (non-goals). `lagrange_server` is unchanged.

The other choice, a C++ batched backend over the PhysX SDK's GPU API, is under Alternatives.

### 2. The multirate scheduler in a batch (invariant 4)

A batch runs **one schedule for all N environments, in lockstep**. Every environment has the
same step size per domain; the scheduler's existing dependency graph, rates and coupling order
are computed once on the host, as for a single run, and turned into a list of kernel launches
per macro step (the longest rate in the run that is a multiple of every shorter one, normally
the physics step). That list is what the CUDA graph captures. Each module's batched step is one
whole-batch launch over `[N, …]` arrays: the per-environment loop is the GPU's.

Consequences, per domain rate:

| Rate | In a batch |
|---|---|
| Electrical, 100 µs | ten launches of the batched electrical kernel per 1 ms physics step inside the graph, the same substeps as the CPU run; or, with `batching.electrical: zoh` (§3.1), one launch per physics step with the exact zero-order-hold solution over 1 ms. The first keeps the CPU rates; the second is faster and still exact for the linear motor model while the joint speed is held (it is in the CPU run too, between physics steps) |
| Physics, 1 ms | one `mujoco_warp.step` launch sequence |
| Rotor, 1 ms | one launch before physics (thrust and torque into the applied forces, `xfrc_applied` or actuator inputs) |
| Sensors, IMU 1 ms, lidar 10 to 100 ms | the IMU's launch every step; the lidar's on the steps where the CPU run would scan (a launch predicated on the step index, so the graph stays fixed) |
| Thermal, 100 ms | one launch every hundredth step (predicated, as the lidar) |

Disabled modules cost zero, as invariant 4 asks: they are not in the launch list. A module whose
domain has no batched form cannot be in the list: the batch refuses to start and names it,
unless the profile disables the domain with a substitute that has one (every `zero`, `constant`
and `ideal` substitute has a trivial batched form: a fill or a copy; `replay` has none in phase
1, open question 5).

Signals in a batch live on the device as `[N, components]` arrays, `float32` by default
(open question 2), in the layout `SignalBus::readBatch` already uses, so a batched module reads
and writes them as the CPU module reads `readBatch`/`writeBatch`. This answers RFC 0001's open
question 6 for the batch only: device-resident batch signals, read back to the host only when
Python asks (`sim.read_batch(name)` as a host copy) or never (`sim.tensor(name)`, §4).

### 3. Which domains run batched

A **reduced form** is the same physics as the CPU module, with the same parameters from the
same stage, simplified only where the table says so, and checked against the CPU module (§5.2).

| Domain | Batched form | Reduced how | Phase |
|---|---|---|---|
| `rigid_body` | MuJoCo Warp | none: the same MJCF; Jolt has no batched form (a profile with `backend: jolt` refuses) | 1 |
| `electrical` | §3.1 averaged drive and battery as vectorised formulas | `averaged` fidelity only; no FMUs (`fmu_host`), no RFC 0010 branch networks with diodes or switches in phase 2 (one bus per robot, open question 4) | 2 |
| `rotor_aero` | coefficient rotors: thrust `T = C_T ρ n² D⁴`, torque `Q = C_Q ρ n² D⁵` (the `coefficients` fidelity), ground effect as on the CPU | no BEM (`fidelity: bem` refuses), no downwash between robots (one robot per environment) | 2 |
| `sensors` | IMU (from MuJoCo Warp's accelerometer and gyro values, with the CPU model's bias, noise and quantisation from the counter-based RNG keyed by environment), joint encoders, `ground_truth` | lidar in phase 3 (§3.2); cameras unsupported (MuJoCo Warp's batch renderer is a separate look, open question 6); GNSS, magnetometer, barometer phase 3 | 2, 3 |
| `thermal` | the lumped network, explicit at 100 ms or backward Euler as on the CPU | no `conduction` fidelity (RFC 0012 cells) | 3 |
| `aerodynamics` | per-link drag in a uniform wind per environment | no NanoVDB wind fields, no LBM, no downwash | 3 |
| `weather` | site values per environment as constants (air density, friction scale) | no fields, no rain or fog clutter | 3 |
| `emi`, `comms`, `hydrodynamics`, `tether`, `vehicle_dynamics`, `traffic`, `humans`, `fluid_power`, `radiation` | none | must be off with a substitute | — |

#### 3.1 Electrical as formulas

The averaged drive is, per motor, `L di/dt = v − R i − K_e ω` with the PI current loop and the
torque `τ = K_t i` (docs/design/domains/electrical.md); the battery an equivalent circuit with an
OCV curve of state of charge. In a batch each motor of each environment is one thread. Two
integrations are proposed:

- **substeps** (default): the CPU module's own discretisation at 100 µs, ten launches per
  physics step: equal results to the CPU module's arithmetic up to float32 and order;
- **zoh**: the closed form of the first-order circuit over the physics step with `v` and `ω`
  held, `i(t+h) = i_∞ + (i(t) − i_∞) e^{−hR/L}`, `i_∞ = (v − K_e ω)/R`, and the charge drawn as
  its exact integral. The sample motor's `L/R` = 0.00031 H / 0.12 Ω ≈ 2.6 ms is longer than the
  1 ms step, so the current cannot be taken as algebraic; the closed form is exact for the
  linear part and its PI loop needs the loop discretised at the same step (TODO(verify) the
  stability margin of the sampled PI loop at 1 ms against the CPU's 100 µs on the sample arm and
  quad before offering it).

#### 3.2 Lidar in a batch

The batched lidar casts the CPU lidar's ray pattern per environment against the environment's
MuJoCo Warp geometry. Two ways, chosen in step 5 by measurement:

- MuJoCo Warp's own batched ray casting (`ray` with BVH over meshes, height fields and flex), in
  Warp kernels: no new code beyond the pattern and the noise; an invariant 8 exception (§5.3);
- a Slang kernel of Lagrange's existing lidar traversal (`gpu/kernels/lidar/bvh_traverse.slang`)
  compiled to CUDA (§5.3) over a BVH per environment built from the geom poses each scan.

OptiX hardware ray tracing (MuJoCo Warp's optional render backend) cannot be recorded into CUDA
graphs (its README), so it is not used inside the batched step.

### 4. Python API and the profile

#### 4.1 `BatchedVectorEnv`

```python
import lagrange, lagrange.envs

env = lagrange.envs.BatchedVectorEnv(
    lagrange.sample_robot(),
    world=lagrange.sample_world(),
    profile="profiles/examples/amr_batch.yaml",  # batch: 4096, batching: {device: cuda}
    observe=["body/base_link/pose", "joint/left_wheel/velocity", "battery/main/soc"],
    act=[("joint/left_wheel", "velocity"), ("joint/right_wheel", "velocity")],
    reward=my_reward,  # a function of device tensors, or None
    control_dt=0.02,
    seed=0,
    randomize={"site:air_density": (1.0, 1.3)},
)
obs, info = env.reset()  # obs: a DLPack-capable device array, shape [4096, obs]
obs_t = torch.from_dlpack(obs)  # zero-copy, same device, same stream (TODO(verify) stream sync)
obs, rew, term, trunc, info = env.step(torch.as_tensor(actions, device="cuda"))
```

- It follows Gymnasium's `VectorEnv` interface (`num_envs`, batched `observation_space` and
  `action_space`, `autoreset_mode` NEXT_STEP as Gymnasium's default) when Gymnasium is
  installed, as `VectorEnv` does today, and works without it.
- Observations, rewards, terminations and actions are device arrays exposing `__dlpack__` and
  `__dlpack_device__` (Warp arrays do; TODO(verify) Warp's DLPack stream semantics with torch's
  current stream). `torch.from_dlpack` shares the memory. Nothing is copied to the host unless
  `info["host"]` is asked for.
- Reset is per environment by a done mask: a masked copy from the environment's saved initial
  state on the device, so a batch never stops for one environment.
- `randomize=` keeps its meaning from `LagrangeEnv`: site parameters drawn per environment and
  per reset; model parameters (mass, friction, damping) drawn per environment through MuJoCo
  Warp's per-world model fields.
- `lagrange.batch.BatchSimulation` is the lower layer (`step(n)`, `tensor(name)`,
  `read_batch(name)`, `write_batch(name, tensor)`, `reset(mask)`, `state_hash()` per environment
  for the CPU mode, §5.1).

The extras it needs (`mujoco-warp`, and `torch` only for the user's own code) are optional: the
`lagrange` package imports them lazily and its tests skip when they are missing, as for
Gymnasium today. They are not system packages on Ubuntu 26.04 and not rosdep keys (TODO(verify)),
so they are pip-installed into a venv by the user, documented, never into the debs.

#### 4.2 The profile

The existing key keeps its schema and gains its meaning:

```yaml
batch: 4096                 # RFC 0005, integer ≥ 1, default 1: the number of environments
batching:                   # new, optional; absent = today's behaviour
  device: cuda              # cuda | cpu (cpu: ParallelEnvs workers, the CPU reference)
  engine: mujoco_warp       # the only value in phase 1
  precision: float32        # float32 | float64 (open question 2)
  nconmax: 32               # MuJoCo Warp's buffers per environment; overflow is an error
  njmax: 128
  electrical: substeps      # substeps | zoh (§3.1)
```

- `batch: N` with no `batching` in a server run stays a warning, as now: `lagrange_server` runs
  one environment. `BatchedVectorEnv` reads `batch` as its default `num_envs`.
- A domain enabled in the profile with no batched form (§3) makes `BatchedVectorEnv` refuse to
  start, naming the domain and its substitutes, as the profile validator does for substitutes.
- `profiles/examples/amr_batch.yaml`, `arm6_batch.yaml` and `quad_x500_batch.yaml` (new) validate
  in CI with the others.

### 5. Fit with the other invariants

#### 5.1 The CPU reference (invariant 7)

The batched GPU path may be non-deterministic: contact ordering, atomics and float32 reductions
differ run to run. The CPU reference stays the single-environment CPU run, which is unchanged and
keeps `SameRunTwiceIsBitForBit` and `lagrange determinism` green. The batch is checked against it
three ways:

1. **Short trajectories, one environment at a time.** For each sample (arm, quad, AMR) and a
   fixed action sequence, environment `k` of a batch of 64 is compared with a CPU run started
   from the same state: joint positions, body poses and every reduced domain's outputs within a
   stated tolerance over the first 0.5 s, before contacts make them diverge (tolerances set in
   step 2 and written in the test, TODO(verify) the values).
2. **Statistics over many environments.** With noise and randomisation, the distributions of
   episode quantities (settling time, energy used, final pose, battery charge drawn) over 1024
   batched environments and 1024 CPU runs agree (two-sample Kolmogorov–Smirnov at a stated
   level).
3. **Closed forms.** Each reduced domain has its own unit test against the formula it
   implements (the RL motor's step response, rotor thrust, the IMU at rest reads `g`), as the
   CPU modules do.

`batching.device: cpu` runs the same API on `ParallelEnvs` over the CPU reference, so a policy
can be evaluated in the deterministic mode. MuJoCo Warp's `deterministic` option is measured in
step 1 (TODO(verify) what it guarantees and what it costs) and reported, not relied on.

#### 5.2 Substitutes (invariant 3)

Every domain off in a batch still names a substitute with a batched form (§2). A reduced form is
not a substitute: it reads the same stage parameters and writes the same signal names and units
as the CPU module, documented per domain as "batched form" in docs/physics/.

#### 5.3 Slang kernels (invariant 8)

Invariant 8 says Lagrange's GPU kernels are written once in Slang. This RFC keeps that for every
kernel Lagrange writes (the reduced domains, the reward helpers, reset and randomisation) and
asks for **one exception: the third-party engine's own kernels are Warp**, as PhysX's own CUDA
kernels would be if the PhysX backend were built. The justification is invariant 12: MuJoCo
Warp is the integrated engine and its kernels are not Lagrange's code. The exception does not
extend to new Lagrange kernels in Warp.

How Lagrange's Slang kernels run beside Warp's:

- Slang compiles to CUDA source (`slangc -target cuda`, which the apt `slang-compiler` here
  does) and to PTX through NVRTC (`-target ptx`, which fails here: "failed to load downstream
  compiler 'nvrtc'"; NVRTC comes with the CUDA toolkit, an optional proprietary dependency,
  dependencies.md). The PTX is built at package build time on a machine with the toolkit, or at
  first use with NVRTC if present (open question 8).
- The PTX modules are loaded into Warp's CUDA context and launched on Warp's stream on the raw
  device pointers of the batch arrays, so they are captured in the same CUDA graph (TODO(verify)
  the launch path: Warp's own API for external CUDA modules, or `cuda-python`'s driver API on
  `wp.get_stream().cuda_stream`).
- `gpu/hal/` gains no CUDA backend for this; the Vulkan HAL is untouched. If the PTX route does
  not work in step 3, the fallback is to write the reduced domains as Warp kernels, which would
  widen the exception and needs the owner's decision (open question 8).

Kernels stay free of constructs that preclude other backends: plain arrays, no Warp-specific or
CUDA-specific intrinsics in the Slang sources.

#### 5.4 Integrate before building (invariant 12)

Physics, collision, contact solving and ray casting are MuJoCo Warp's. Lagrange builds only the
reduced domains (its wedge), the wiring and the environment API.

#### 5.5 Units, double precision, tiers

- Units (invariant 2): the batch is wired from the same `SignalSpec`s; a mismatch fails at
  start.
- Double world positions (invariant 6): a batch is one robot in a bounded world near the origin
  (no streaming, no floating origin); `float32` positions are local-frame values within ±1 km
  (TODO(verify) MuJoCo Warp's precision setting; Warp supports `float64` arrays, MuJoCo Warp's
  kernels are `float32` by default). Absolute geodetic positions (GNSS) are computed on the host
  side from the double origin, as now.
- Tiers (invariant 9): Tier 2 runs on the GPU. Tier 1 (no CUDA, or not NVIDIA): `device: cuda`
  refuses and names `device: cpu`. Tier 0: `device: cpu` only.

### 6. What the 1.x freeze allows (RFC 0007)

| Surface | Change | Allowed because |
|---|---|---|
| Profiles | new optional key `batching`; `batch` keeps type, default and meaning (number of environments) | a new optional key with a default that keeps the old behaviour; no `schema_version` change |
| Python API | new class `BatchedVectorEnv`, new module `lagrange.batch` | not frozen; semver minor |
| Schemas, plugin ABI, protocol, recordings | unchanged | — |
| C++ headers (`SignalBus`, scheduler) | device-resident batch arrays as an addition beside `readBatch` | not frozen; additions |

Plugins (the C ABI) do not run in a batch in this RFC: a profile with `plugins:` refuses in a
batch. A batched entry point in the ABI would be an appended function table (MINOR + 1) in a
later RFC.

## Spike

**A spike, not a benchmark.** MuJoCo Warp 4.0.1, Warp 1.19.0, MuJoCo 4.0.1 (Python bindings),
Python 3.13.15 in a venv under `scratch/` (nothing added to the repository's dependencies), on an
NVIDIA GeForce RTX 4060 Laptop GPU (8 GB, driver 595.99.02) with an i9-13900H. The machine was
shared with other builds (load average about 18 on 20 threads), which lowers the CPU numbers and
may add launch latency. Script: `docs/plan/spikes/batched-gpu/mjw_spike.py`; raw numbers:
`docs/plan/spikes/batched-gpu/results-2026-11-22.json`.

Method: each sample URDF loaded by MuJoCo's own URDF reader into an `MjSpec`, with the backend's
options (1 ms, elliptic cones, `implicitfast`) and a floor plane. The arm (`arm6.urdf`: 8 DoF, 11
geoms) gets a position servo on each joint; the quad (`quad_x500.urdf`: a free base and 4 rotor
joints, 10 DoF, 3 collision geoms) a free joint and a hover thrust on a site in each rotor. One
`mujoco_warp.step` captured in a CUDA graph, 20 warm-up launches, 500 timed launches; the CPU
figure is `mj_step` on one core. `nconmax` 16, `njmax` 64; no world overflowed or went NaN.
**Physics only**: no electrical, thermal, rotor or sensor domain, no observation or reward
computation, no resets.

| N | arm6: batched steps/s | arm6: env steps/s | quad: batched steps/s | quad: env steps/s |
|---|---|---|---|---|
| 1 | 5 481 | 5 481 | 5 715 | 5 715 |
| 64 | 3 969 | 254 013 | 6 015 | 384 981 |
| 1024 | 2 817 | 2 884 221 | 4 122 | 4 221 100 |
| 4096 | 1 354 | 5 544 760 | 1 685 | 6 900 043 |
| CPU MuJoCo, 1 core | — | 224 708 | — | 120 641 |

`docs/plan/spikes/batched-gpu/plot.py` draws them as a PNG from the JSON (the image is not
committed). A second run of the same script six minutes later gave the GPU numbers again within
1 % at N ≥ 1024 (arm 5 541 119, quad 6 943 371 at N = 4096) but the CPU ones moved by a factor of
2 to 2.5 under the shared load (arm 111 070, quad 305 339): the CPU column is only an order of
magnitude.

Read with care:

- At N = 1 the GPU is about 20 to 40 times slower than one CPU core: launch latency dominates.
  The GPU passes one core somewhere between N = 16 and 64, and at N = 4096 gives roughly 20 to
  60 times one core. Batched steps per second fall from N = 1024 to 4096, so each environment
  runs at 1.35 to 1.7 times real time: the GPU is saturated there.
- Against Lagrange today (about 11 000 `LagrangeEnv` steps per second at 2 ms for the AMR with
  two lidars, MuJoCo and the electrical network) the gap is about 500 times, but that number
  includes Lagrange's other domains and the spike does not. Step 4 measures the reduced domains'
  share; the target in "Goal and scale" asks they cost at most half.
- The sample robots are small. Contact-rich scenes (a legged robot on terrain, the arm grasping)
  need larger `nconmax`/`njmax` and run slower; step 1 adds the quadruped.
- The compile on first use took tens of seconds (Warp module loads of 0.1 to 6.4 s each, cached
  afterwards in Warp's kernel cache).

## Phased steps (each a PR, after acceptance)

1. **Spike, extended** (`docs/plan/spikes/batched-gpu.md`): the arm, the quad and the quadruped
   with N = 1, 4, 16, 64, 256, 1024 and 4096, through the MJCF the backend writes (not MuJoCo's
   URDF reader), on this machine and the CI GPU runner; MuJoCo Warp's `deterministic` option and
   its cost; contact overflow behaviour; the comparison of one batched environment with the CPU
   backend over 0.5 s.
2. **Export and the API, physics only.** `mj_saveXMLString` export from the backend
   (`Simulation.mujoco_xml()`), `lagrange.batch.BatchSimulation`, `BatchedVectorEnv` with
   observations of joint and body signals, DLPack, masked reset, per-environment randomisation;
   `batching` in `profiles/schema.json`; the three sample profiles; tests skipped without
   `mujoco_warp` or a CUDA device; `docs/guide/rl.md` and the trajectory check of §5.1 (1).
3. **Slang on CUDA.** The PTX build and the launch on Warp's stream inside the graph, with one
   kernel (the ideal torque source as a copy) and its closed-form test; or the owner's decision
   on open question 8 if it does not work.
4. **Electrical and rotors batched.** §3.1 substeps (and zoh if its check passes), the
   coefficient rotors and the multicopter's hover controller as a reference policy; tests
   against closed forms and against the CPU modules (§5.1 (1) and (2)); throughput with them.
5. **Sensors batched.** IMU and encoders with noise from Philox keyed by environment; then the
   lidar (§3.2, chosen by measurement).
6. **Thermal, aerodynamics and weather constants batched**; `docs/design/benchmarks.md` gains a
   batched section; an `examples/lagrange_example_rl_batched` package (skipped without the
   extras).

## Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| MuJoCo Warp is "Alpha" on PyPI; APIs move (it went from 3.15.0 to 4.0.0 in a month) | High | Medium | pin an exact version in the docs and the examples; the integration surface is small (`put_model`, `make_data`, `step`, model fields) |
| Lagrange's MuJoCo (3.12) and MuJoCo Warp's (≥ 4.0) differ: the MJCF of one may not mean the same in the other | Medium | Medium | MJCF text, not `mjModel`; step 1's trajectory check; RFC 0017 removes the gap |
| The reduced domains drift from the CPU modules as those change | Medium | High | the same parameters from the same loader; the trajectory and statistics checks run in CI on the GPU runner |
| Slang's CUDA/PTX path or launching it in Warp's graph does not work | Medium | Medium | step 3 is early and small; fallback is open question 8 |
| float32 is not enough for some quantities (battery charge over long episodes, small currents) | Medium | Low | keep integrators in float64 where needed (Warp supports it), or `precision: float64` |
| GPU-only CI: the GPU runner is self-hosted and nightly; PRs cannot check the batch | High | Medium | CPU mode (`device: cpu`, MuJoCo Warp's CPU device for the API) in every PR; GPU checks in the merge queue and nightly, as CLAUDE.md §12.6 has for GPU tests |
| Users expect every Lagrange feature in a batch | High | Low | the refusal names the domain and its substitutes; docs/guide/rl.md lists what is batched |

## Alternatives considered

- **Do nothing; scale `ParallelEnvs`.** Process-per-environment is bounded by cores (1.5 to 1.8×
  with four workers today). It stays the CPU mode.
- **PhysX 5 GPU in C++ as a batched backend.** Keeps everything in C++ and the server, and
  PhysX is BSD-3. But the PhysX backend is not built yet (its vendor package is "High" risk in
  dependencies.md), Lagrange has validated MuJoCo, not PhysX, and its GPU part needs the CUDA
  toolkit at build time, which the ROS buildfarm's debs cannot depend on by default. A later RFC
  if MuJoCo Warp falls short.
- **Write a batched rigid-body solver in Slang.** Against invariant 12 and years of work.
- **MJX-JAX.** Narrower features (cylinder contacts) and JAX/XLA in the user's stack.
- **Newton as the engine.** Brings MuJoCo Warp plus other solvers, but a second pinned MuJoCo
  (3.17); reconsider for cloth or cables (open question 3).
- **Genesis World.** A second, unvalidated rigid-body engine with its own compiler.
- **Isaac Lab as the training framework, Lagrange as a backend for it.** Isaac Lab 3.0 is
  multi-backend; a Lagrange backend for Isaac Lab is possible on top of this RFC, not instead of
  it.
- **Batch the C++ scheduler on the CPU** (vectorised modules over `readBatch`, one process). It
  removes the pipe cost but stays bounded by cores; physics would still be N MuJoCo instances.
  Useful for the reduced domains' CPU mode, not for scale.

## Open questions for the owner (with a proposed answer)

1. **MuJoCo version.** MuJoCo Warp needs `mujoco-core >= 4.0.0`; Lagrange ships 3.12.0. Proposed:
   go through MJCF text now; when RFC 0017 lands, compare the two on the same version.
2. **Precision.** Proposed: `float32` for physics and observations (MuJoCo Warp's default),
   `float64` for the battery's charge and the thermal states.
3. **Newton.** Proposed: not now; revisit for deformables and cables (tethers in a batch).
4. **Electrical scope.** Proposed: one bus per robot with its motors and battery in phase 4;
   RFC 0010 branch networks (diodes, switches, several buses) later.
5. **Replay substitutes in a batch.** Proposed: not in this RFC.
6. **Cameras.** MuJoCo Warp's batch renderer is fast but is not Lagrange's camera model (lens,
   noise, realistic look), and its OptiX path is not in CUDA graphs. Proposed: unsupported here;
   a separate RFC decides whether a batched camera may be another look than the viewer's
   (invariant 13 binds the viewer, not training).
7. **Watching a batch in the viewer.** Proposed: no; a user replays one environment's actions
   in a normal run (`batching.device: cpu`) to watch it.
8. **Invariant 8 fallback.** If Slang kernels cannot be launched in Warp's CUDA graph, may the
   reduced domains be Warp kernels (a wider exception)? Proposed: yes, only for code under
   `python/lagrange/batch/`, each kernel with a Slang twin for the CPU-mode tests, and revisit
   when the HAL gains CUDA.
9. **Where the Python extras come from.** Proposed: documented `pip install mujoco-warp==<pin>`
   into a venv (as Gymnasium today); no rosdep key, no debs dependency; licences recorded in
   docs/design/dependencies.md when step 2 adds the optional import.
10. **CI GPU runner.** Proposed: the batch's GPU tests run nightly on the self-hosted NVIDIA
    runner; PRs run the CPU mode.

## References checked for this draft (2026-11-22)

- PyPI JSON API: `mujoco-warp`, `mujoco-mjx`, `mujoco`, `warp-lang`, `newton`, `genesis-world`,
  `brax`, `isaaclab`, `isaacsim`, `jax` (version, upload date, `requires_python`,
  `requires_dist`, classifiers).
- GitHub releases and licence files: google-deepmind/mujoco_warp, newton-physics/newton,
  Genesis-Embodied-AI/genesis, isaac-sim/IsaacLab (v3.0.0 release notes), google/brax,
  NVIDIA-Omniverse/PhysX, NVIDIA/warp.
- MuJoCo documentation sources: `doc/mjwarp/index.rst` (feature parity, `nconmax`/`njmax`,
  per-world model fields, graph capture, differences from MuJoCo, `deterministic` option) and
  `doc/mjx.rst` (MJX-Warp and MJX-JAX feature table).
- READMEs: mujoco_warp (API compatibility, batch rendering, OptiX), newton (requirements,
  driver R580+), Genesis World (layers, Quadrants targets), IsaacLab (Isaac Sim compatibility,
  licences), brax (maintenance note).
- Lagrange: docs/design/benchmarks.md (Python throughput), docs/design/dependencies.md (§8,
  Slang, CUDA toolkit, PhysX), rfcs/0001, 0005, 0007, 0017, `core/signals/SignalBus.hpp`,
  `modules/rigid_body/mujoco_backend/src/MujocoBackend.cpp`, `python/lagrange/envs.py`.
- Local check: `slangc -target cuda` works with the apt Slang here; `-target ptx` needs NVRTC.
