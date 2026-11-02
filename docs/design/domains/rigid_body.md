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
| PhysX 5 | not built, by the owner's v0.6 decision; planned for vehicles and GPU batches (Tier 2) | BSD-3-Clause | none | CUDA only |

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
| Backend agreement | same scene in MuJoCo, Jolt, PhysX | joint trajectories within a stated envelope; differences reported, not hidden. MuJoCo and Jolt (v0.3, `server/test/test_backends.cpp`): arm6 under tilted gravity, joint-space inertia within 2 %, its fall over 0.3 s within 2 % (+5 mrad), with armature within 5 % (+15 mrad); the sample AMR at speed within 0.02 m/s and 0.02 rad/s. Reported: on Jolt's rigid contacts the AMR's drive wheels slip while the drive ramps up (wheels and casters share the load indeterminately), about 0.4 m behind after 6 s |
| `effective_inertia` | analytic two-link arm inertia matrix diagonal | 1e-9 relative |
| Determinism | two runs, same seed | bit-identical state hash (MuJoCo backend) |
| AMR drive test (v0.2) | sim-vs-real: recorded wheel odometry of a real differential-drive robot driving a square. Dataset to be recorded or sourced (open question Q16) | position error after 10 m within the spread of repeated real runs |

## 8. Backends as plugins: the contract and the conformance suite (RFC 0008 §5)

A plugin module with `domain = "rigid_body"` is a backend, picked by `--physics NAME` or a
profile's `rigid_body: {backend: NAME}`. Its contract is §3 and §5, binding:

- **Outputs**, per hinge and slide joint: `joint/<j>/position`, `velocity`; for actuated joints
  also `acceleration`, `effective_inertia`, `constraint_torque`. Per moving body (its joint to the
  world, or one of its ancestors', is not fixed): `body/<b>/pose`, `twist` (world_local),
  `acceleration` (proper acceleration, body frame: what an IMU feels).
- **Inputs**: `joint/<j>/torque` and `load_torque` (window average, optional);
  `body/<b>/external_wrench` (world_local, at the body origin, optional);
  `body/<b>/pose_command` for kinematic bodies (optional: without one the body stays);
  `env/friction_scale` (optional, default 1).
- **Behaviour**: free root bodies; every `ShapeType`, or a refusal in `declare` that names the
  shape; no collisions between bodies of one kinematic tree, and collisions between separate
  trees (a world file's loose objects share a robot index but not a tree); joint limits;
  armature; `World::Joints::initialPosition` (or a warning that it starts at zero);
  `shift_origin`; and, for a backend that says it is deterministic, `save_state` and
  `restore_state` that restore a run bit for bit.

The conformance suite, `ros2 run lagrange_server lagrange_physics_conformance`
(`server/test/physics_conformance.cpp`), checks a backend against it. It runs the backends in
`LAGRANGE_CONFORMANCE` (`NAME` or `NAME@PLUGIN`, comma-separated; default
`mujoco,jolt,jolt_abi@lagrange_jolt`); a third-party engine runs it as
`LAGRANGE_CONFORMANCE=my_engine@libmy_engine.so`:

| Test | Requirement |
|---|---|
| Free fall | speed after 2 s within 0.1 % of 2 g: nothing damps it |
| A falling box | dropped from 1 m, rests at its half height within 5 mm after 3 s |
| A pendulum | period of the physical pendulum, `2π √(I / (m g d))·(1 + θ₀²/16)`, within 0.5 % |
| A box on a slope | friction 1: holds at 30° (moves < 1 cm in 1.4 s), slides at 60° (> 0.3 m) |
| A stack of three boxes | stands for 10 s: the top box within 1 cm of its place |
| The sample AMR's loop | over 10 s, the distance driven within 5 % of MuJoCo's, the end within 10 % of it |
| Determinism and state | two runs bit-identical; a saved state restored runs on bit-identically |

MuJoCo, Jolt and `jolt_abi` (the same Jolt engine as a plugin through the C ABI,
`modules/rigid_body/jolt_backend/src/jolt_plugin.cpp`, `--plugin lagrange_jolt`) pass it; Jolt and
`jolt_abi` give identical numbers. On Jolt the AMR's loop ends 5.3 % of its 5.4 m from MuJoCo's
end (heading drift from its per-body friction). Writing the suite found two Jolt bugs, both
fixed (a third, Jolt's default body damping, came up later and is fixed too): loose objects of one world file passed through each other (collisions were filtered by
robot index instead of by tree), and a restored state lost its contacts with the static scenery.
Restored contacts now include the scenery whenever it is unchanged; it changes between a save
and a restore only when streamed tiles come or go.

## Contact force

`body/<b>/contact_force` (N, in the world frame) is the sum of what touches a moving body pushes it with, read from each step's contacts, for force sensing and foot contact. On a robot at rest the bodies' forces add up to its weight (`test_contact_forces_on_a_robot_at_rest_carry_its_weight`). Jolt's contact listener gives no impulses per step, so on Jolt it is what is left of a group's momentum change: `m·Δv/Δt − m·g` less its external forces and its joints' forces (below), by Newton's second law; a group of bodies fixed together (one Jolt body) gets it on its root body. `contact/points` above is still MuJoCo only.

`joint/<j>/reaction` (Wrench, world frame, linear then angular) is what the parent does to the child at the joint, including what its motor applies, as a force-torque sensor there reads it. Checked on a hinge holding a known weight (`test_a_joints_reaction_is_what_it_carries`). On Jolt it is the joint constraint's impulses over the step (`GetTotalLambda*`: the point, rotation, limit and motor parts, + on the child, − on the parent, the hinge's rotation part in the basis Jolt builds, `b2 = a2⊥`, `c2 = a2 × b2`) plus the effort applied through the joint. The conformance suite checks both engines: a box at rest is held up by its weight, a still pendulum's pivot carries it with no moment, and a swinging one's carries `m·g + m·d·ω²` at the bottom within 1% (11.349 N on MuJoCo, 11.348 N on Jolt).
