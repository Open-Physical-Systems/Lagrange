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

### Pushing against measurement (2026-11-03)

`APushedBlockMovesAsTheMcubeMeasurements` repeats the repeated-push experiment of Yu, Bauza, Fazeli
and Rodriguez ("More than a Million Ways to Be Pushed", IROS 2016, §VII, Table IV): their
stainless-steel square rect1 (0.837 kg, 90 mm, 13 mm thick) pushed 15 cm at 20 mm/s by a 9.5 mm
rod, normal to a face, halfway between its centre and edge, on four surfaces whose friction they
measured (abs 0.13, delrin 0.12, plywood 0.24, polyurethane 0.28; rod 0.25). Final (Δx, Δy, Δθ):

| | abs | delrin | plywood | pu |
|---|---|---|---|---|
| Measured, mean of 2000 | (40.1, −67.6, 74.7) | (38.8, −50.7, 78.5) | (36.4, −93.6, 70.2) | (40.2, −85.0, 69.3) |
| Quasi-static model (Lynch et al. 1992, run by Yu et al.) | (41.0, −98.1, 66.3) | | | |
| MuJoCo | (38.7, −91.1, 61.8) | (38.6, −91.3, 61.7) | (39.4, −92.3, 59.9) | (39.6, −93.2, 59.0) |
| Jolt | (6.0, −148.8, 1.9) | (8.5, −148.4, 2.6) | (0.2, −149.6, 0.3) | (0.6, −149.4, 0.7) |

- **MuJoCo is the quasi-static theory's answer**: within 6 mm of Lynch et al.'s model, turning 4
  to 7° less (a box on a plane rests on its four corners there, where the model spreads the
  pressure evenly). Uniform Coulomb friction makes the path independent of the support's friction,
  so it gives one answer for all four surfaces. Against the measurements it is 3 to 8 mm off on
  plywood and polyurethane but 24 and 41 mm off on abs and delrin, and 10 to 17° short on all: as
  the theory itself is (8 to 12° short, 6 to 48 mm off). What the surfaces do differently
  (uneven pressure, anisotropy, friction that changes with speed and place, their §V) is not in
  the model.
- **Jolt holds the sliding block straight.** Jolt 5.6 keeps linear friction, at the centre of
  the contact points, and friction about the contact normal on separate budgets (μ·Σλ and
  μ·Σ dᵢ·λᵢ, `ContactConstraintManager`), so a block already sliding at full friction still
  resists turning at full strength: it is carried 149 mm in front of the rod, turning 2°. Coulomb
  friction couples the two (the limit surface, which Yu et al. measured). A known limitation of
  the engine, reported, not asserted; use MuJoCo for pushing and manipulation.
- **A bug found on the way.** Jolt gave a body whose colliders have no friction its default of
  0.5 (`settings.mFriction = friction > 0 ? friction : 0.5`); a frictionless body now stays
  frictionless (`AFrictionlessBoxSlidesAtGSinTheta`, all backends).

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

MuJoCo and kinematic bodies (since 1.2.x): MuJoCo makes no contact between two bodies without
degrees of freedom (a kinematic body and the scenery, or two kinematic bodies), but its broad
phase still sorts and pairs them, which for the town's 650 traffic and people bodies was about
half of a step. Before each collision pass the backend clears the collision masks of the kinematic
bodies that no moving body can reach (a sphere about each, against a box about each moving
tree's colliders at their last place, widened by 0.5 m and 200 m/s for a step) and puts them back
as one comes near (`MujocoBackend::cullKinematicContacts`). The contacts are the same: the test
`CullingFarKinematicBodiesLeavesTheRunUnchanged` runs a pusher into a box with and without it
and compares the state hash at every step. With a moving plane in the world it stays off.
A world with no degree of freedom at all (scenery and kinematic bodies only, as the town
without a robot) skips MuJoCo's step: it advances the time as `mj_step2` does and places the
kinematic bodies (`mj_kinematics`); the state is the same bit for bit (checked on the town's
state hash over 20 s). The backend writes a kinematic body's pose, twist and contact force only
when they change (a body's pose changes when its command does, every 10 ms for the traffic and
the people), and its acceleration, always zero, once: the bus keeps a value until its writer
writes another. Everything is written again after a restore and an origin shift.

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
