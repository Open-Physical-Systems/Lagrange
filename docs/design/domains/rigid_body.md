# Domain: rigid body and articulations

## 1. Scope and release

Rigid bodies, articulated joints, contacts and friction. Implemented by wrapping existing engines
(invariant 12). First release: v0.2 with one backend; a second backend before v0.3 to prove the
abstraction.

Backend order proposed (open question Q9):

| Backend | Role | License | Source on Resolute | GPU |
|---|---|---|---|---|
| MuJoCo | v0.2 default and deterministic CPU reference | Apache-2.0 | `ros-lyrical-mujoco-vendor` or `libmujoco-dev` 3.5.0 | CPU (MuJoCo Warp is Python/Warp, not linked) |
| Jolt | second CPU backend; large scenes with many free bodies | MIT | `jolt_vendor` (v5.6.0 upstream) | CPU |
| PhysX 5 | vehicles (v0.6), GPU batches (Tier 2) | BSD-3-Clause | `physx_vendor` | CUDA only |

MuJoCo is first because it is packaged for Lyrical, has no GPU or CUDA build requirement, and its
MJCF parser is also needed by the importer.

## 2. Models per fidelity level

| Level | Content | Backend support |
|---|---|---|
| `kinematic` | Joint positions follow commands; no dynamics. Used by the replay and fixed substitutes | built-in |
| `articulated` (default) | Reduced-coordinate articulations, convex contacts, Coulomb friction, joint limits, armature inertia | all three |
| `articulated_gpu` | Same model, batched over environments | PhysX GPU (Tier 2) |

Soft bodies, cloth and fluids are out of scope before 1.0.

## 3. Signals

```cpp
// modules/rigid_body/<backend>/ — declare()
for (JointId j : world.joints().actuated()) {
    d.input ({.name = fmt("joint/{}/torque", j),            .type = SignalType::Scalar, .unit = units::N_m,
              .hold = Hold::WindowAverage});               // from electrical or its substitute
    d.input ({.name = fmt("joint/{}/load_torque", j),       .type = SignalType::Scalar, .unit = units::N_m,
              .hold = Hold::WindowAverage, .optional = true});   // rotor reaction etc., via wrench_sum
    d.output({.name = fmt("joint/{}/position", j),          .type = SignalType::Scalar, .unit = units::rad});
    d.output({.name = fmt("joint/{}/velocity", j),          .type = SignalType::Scalar, .unit = units::rad_per_s});
    d.output({.name = fmt("joint/{}/acceleration", j),      .type = SignalType::Scalar, .unit = units::rad_per_s2});
    d.output({.name = fmt("joint/{}/effective_inertia", j), .type = SignalType::Scalar, .unit = units::kg_m2});
    d.output({.name = fmt("joint/{}/constraint_torque", j), .type = SignalType::Scalar, .unit = units::N_m});
}
// Prismatic joints use the same names with units N, m, m/s, m/s², kg.

for (BodyId b : world.bodies().dynamic()) {
    d.input ({.name = fmt("body/{}/external_wrench", b), .type = SignalType::Wrench, .unit = units::N,
              .frame = "world_local", .hold = Hold::WindowAverage, .optional = true});   // aero, rotor, hydro
    d.output({.name = fmt("body/{}/pose", b),            .type = SignalType::Pose,   .unit = units::m,
              .frame = "world_local"});
    d.output({.name = fmt("body/{}/twist", b),           .type = SignalType::Wrench, .unit = units::m_per_s,
              .frame = "world_local"});                   // linear (m/s) then angular (rad/s)
    d.output({.name = fmt("body/{}/acceleration", b),    .type = SignalType::Wrench, .unit = units::m_per_s2,
              .frame = "body"});                          // proper acceleration, for the IMU
}

d.input ({.name = "env/friction_scale", .type = SignalType::Scalar, .unit = units::one,
          .optional = true, .defaultValue = 1.0});        // from weather (wet surfaces)
d.output({.name = "contact/points", .type = SignalType::Bytes, .unit = units::one});     // packed list for the viewer
```

A `Wrench`- or `Pose`-typed signal holds two physical quantities; its `unit` names the first
(force, linear velocity, position) and the type fixes the second (torque in N·m, angular velocity
in rad/s, unit quaternion). This rule is stated in RFC 0001.

## 4. Step size

Default 1 ms. Allowed 0.1 ms to 10 ms. Vehicles at 1–2 ms; large free-body scenes at 2–4 ms with
more solver iterations. The backend's own substeps are internal and do not appear to the
scheduler.

## 5. Substitutes and contract

Contract: `joint/{j}/position`, `joint/{j}/velocity`, `joint/{j}/effective_inertia` for every
actuated joint; `body/{b}/pose`, `body/{b}/twist`, `body/{b}/acceleration` for every dynamic body.

| Profile token | Kind | Behaviour |
|---|---|---|
| `fixed` | constant | All joints at their initial position, zero velocity; `effective_inertia` = a large constant (1e6 kg·m²), so motors see a locked rotor |
| `load_profile` | ideal | Per actuated joint, a one-degree-of-freedom rotor: `J·ω̇ = τ − τ_load(ω)`, with `τ_load` from a CSV torque-speed table and `J` from `elec:rotorInertia` plus a table value. This is the virtual-load bench in CLAUDE.md §11.2 |
| `replay` | replay | Joint and body states from an MCAP file; `effective_inertia` from the recording if present, else computed once from the model at the recorded pose |

## 6. Placement

| Tier | Placement |
|---|---|
| 0 | CPU, single thread in deterministic mode |
| 1 | CPU; broad-phase and island solving on the worker pool |
| 2 | CPU for one environment; PhysX GPU when `batch > 1` and `gpu: true` |

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Free fall, pendulum, double pendulum energy drift | closed-form or high-order integration | energy drift < 0.1 % over 10 s at 1 ms |
| Sliding block on an incline | closed-form (Coulomb) | stick/slip angle within 1° |
| Backend agreement | same scene in MuJoCo, Jolt, PhysX | joint trajectories within a stated envelope; differences reported, not hidden |
| `effective_inertia` | analytic two-link arm inertia matrix diagonal | 1e-9 relative |
| Determinism | two runs, same seed | bit-identical state hash (MuJoCo backend) |
| AMR drive test (v0.2) | sim-vs-real: recorded wheel odometry of a real differential-drive robot driving a square. Dataset to be recorded or sourced (open question Q16) | position error after 10 m within the spread of repeated real runs |
