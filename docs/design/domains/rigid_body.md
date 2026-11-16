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
  `joint/<j>/friction_limit` (N or N·m, the latest value, optional, since RFC 0020): a Coulomb
  friction limit for the step added to the joint's own (`JointDynamicsAPI_1`) friction, which a
  screw's or worm's gear sets from its load (MuJoCo: the dof's `frictionloss`, its friction row
  stiffened to `solimp` 0.9999 and `solref` 2·dt while the input has a writer, rebuilt with
  `mj_step1` when the value changes; Jolt 5.6: `SliderConstraint::SetMaxFrictionForce`,
  `HingeConstraint::SetMaxFrictionTorque`). `constraint_torque` includes the friction the joint
  applied (MuJoCo's `qfrc_constraint` row; on Jolt the limit's and the motor part's impulses, the
  motor part being the joint's friction);
  `body/<b>/external_wrench` (world_local, at the body origin, optional);
  `body/<b>/pose_command` for kinematic bodies (optional: without one the body stays);
  `env/friction_scale` (optional, default 1).
- **Behaviour**: free root bodies; every `ShapeType`, or a refusal in `declare` that names the
  shape; no collisions between bodies of one kinematic tree, and collisions between separate
  trees (a world file's loose objects share a robot index but not a tree); joint limits;
  armature; `World::Joints::initialPosition` (or a warning that it starts at zero);
  `shift_origin`; and, for a backend that says it is deterministic, `save_state` and
  `restore_state` that restore a run bit for bit.
- **Friction** is per collider: a contact takes the larger of its two colliders' coefficients
  (MuJoCo's rule for geoms of equal priority), times `env/friction_scale`. Jolt keeps each
  collider's coefficient on its sub-shape of the body's compound shape (the shape's user data)
  and combines per contact manifold (`SetCombineFriction` with the sub-shape IDs); it turns off
  manifold reduction on a body whose colliders differ, which would otherwise merge the contacts
  of e.g. feet and body under one sub-shape. Through v1.3.0 Jolt took the largest coefficient of a
  body's colliders for all of them (the legged spike, `docs/plan/spikes/legged-contact.md` §5).

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
| A pendulum's energy | from 1 rad, 60 s at 1 ms: period of the first five swings within 0.25 % of the exact `4 √(I / (m g d))·K(sin(θ₀/2))`; energy never above its start by 0.5 %, at least 80 % of it kept (§8.1) |
| A box on a slope | friction 1: holds at 30° (moves < 1 cm in 1.4 s), slides at 60° (> 0.3 m) |
| Friction per collider | a body box of friction 1 on four feet of 0.3 (the ramp 0.3): holds at 10°, slides at 25° at `g (sin θ − 0.3 cos θ)` within 10 % |
| A stack of three boxes | stands for 10 s: the top box within 1 cm of its place |
| The sample AMR's loop | over 10 s, the distance driven within 5 % of MuJoCo's, the end within 10 % of it |
| Determinism and state | two runs bit-identical; a saved state restored runs on bit-identically |
| A self-locking screw | the sample screw lift's Tr12x2 (`friction_limit` from the electrical domain) holds 100 kg for 60 s with its motor shorted: settles within 0.1 mm, then moves < 0.5 mm |
| A back-drivable screw | its 16x5 ball screw under 20 kg, motor shorted, sinks at `η_back·m·g·R/((2π/lead)²·Kt²)` within 2 %, and on bit for bit from a restored checkpoint |

MuJoCo, Jolt and `jolt_abi` (the same Jolt engine as a plugin through the C ABI,
`modules/rigid_body/jolt_backend/src/jolt_plugin.cpp`, `--plugin lagrange_jolt`) pass it; Jolt and
`jolt_abi` give identical numbers. On Jolt the AMR's loop ends 5.3 % of its 5.4 m from MuJoCo's
end (heading drift; not from friction, since each of the AMR's bodies has one friction on all
its colliders: the run is bit for bit the same with Jolt's friction per body or per collider).
Writing the suite found two Jolt bugs, both
fixed (a third, Jolt's default body damping, came up later and is fixed too): loose objects of one world file passed through each other (collisions were filtered by
robot index instead of by tree), and a restored state lost its contacts with the static scenery.
Restored contacts now include the scenery whenever it is unchanged; it changes between a save
and a restore only when streamed tiles come or go.

### 8.1 Energy on Jolt: joints as constraints, and sub-steps

A free pendulum (1 kg, 1 m, from 1 rad, 1 ms) keeps its energy within 0.07 % over 60 s on
MuJoCo. On Jolt it lost 16.4 % up to v1.4.0 (`docs/validation/comparison.md`). The loss
was Jolt's method, not a setting of Lagrange's:

- Lagrange gives Jolt no damping: body linear and angular damping are zero
  (`mLinearDamping`, `mAngularDamping`, whose defaults are 0.05), the hinge has no friction
  (`mMaxFrictionTorque` is the joint's, 0 here) and no motor, the angular velocity is not
  clamped (`mMaxAngularVelocity`), and the body never sleeps. Jolt's solver settings change
  nothing measurable: no position steps, Baumgarte 0 or 1, 30 velocity steps, warm start off
  all give 16.3–16.4 %.
- MuJoCo, DART and Bullet's multibody solve a robot in joint coordinates. Jolt keeps every body
  free and holds a joint by projecting the velocities at each step (sequential impulses). Over a
  step the body turns by ω Δt, so that share of its velocity is radial at the next step, and the
  projection takes it off as an inelastic impulse: (ω Δt)² of the kinetic energy per step,
  O(Δt) per second. The same scheme in double precision on a point mass (semi-implicit Euler,
  velocity projection, position projection) loses 16.35 % of this pendulum's energy in 60 s;
  Jolt lost 16.38 %.
- **The backend gives half of that impulse back** (`measureJointTurn`, `kHingeTurnShare`).
  Before each Jolt step it takes each hinge anchor's velocity error made by the turn alone:
  the velocities as they are, on the anchor's arms now less on its arms of the last step, where
  the solve made them agree (ω₂ × Δr₂ − ω₁ × Δr₁; no gravity, effort, or error left over, so
  nothing is given back twice). After the step it applies the impulses that change those
  errors by −½ of themselves, equal and opposite on the two bodies, solved together for a
  chain (Gauss-Seidel, 10 passes), and counts them in the joint's reaction. With a share e
  given back, the next projection takes ½ ((1 − e) ω Δt)² of the energy and the share puts
  back ½ (e ω Δt)², so ½ is the one share that keeps it to leading order (the velocities then
  run along the step's middle, as in the midpoint rule). Measured (`APendulumKeepsItsEnergy`,
  `ADoublePendulumKeepsItsEnergy` in test_jolt): the pendulum keeps 99.87 % from 1 rad,
  99.96 % from 2 rad and 100.00 % from 3 rad (before: 83.6 %, 64.9 %, 60.4 %); a double
  pendulum, 30 s, ends at 100.02 % from 1 rad and from 2 rad (chaotic; 99.1–100.8 % on the
  way), where it lost 7.1 % and 40.7 %. A share of 0.4 loses 3.9 % and 0.6 gains 4.0 %; each
  anchor's share given back on its own, without the chain solve, made the double pendulum blow
  up. The arms are each body's attachment point turned by its rotation alone: taken from world
  positions, float rounding far from the origin read as a turn (a floating-origin rebase moved
  the AMR's trajectory by 0.2 mm). Sliders are left as they were (a slide does not turn its
  body).
- The period is not hurt: within 0.02 ppm of the exact one at 0.05 rad. A pendulum swinging
  freely for a minute is the worst case; a driven joint gets its energy back from the motor.
- Jolt runs have new state hashes since this change: a Jolt run is still bit-for-bit
  repeatable, and a saved state carries each hinge's last arms so a restored run goes on
  bit-identically.

`domains.rigid_body.substeps: N` (profile, 1 to 64, default 1; Jolt only, MuJoCo has nothing to
divide) runs N Jolt steps of Δt/N per scheduler step, collision detection included, so it costs
about N times as much. Joint reactions, limit torques and the contact force left over from them
are the constraint impulses averaged over the N sub-steps, over Δt/N (Jolt's total lambdas
start afresh each sub-step, so a step listener adds each one up before the next solve; the last
sub-step alone left part of a swinging pendulum's pull to "contact", 4 % of m·g at 4 sub-steps;
`SubstepsKeepTheReactionAndInventNoContact` and `SubstepsInventNoContactOnASwingingPendulum` in
test_jolt). Measured on the pendulum and the comparison's scenes (one core; energy before the
half given back, then with it):

| substeps | energy after 60 s, before | energy after 60 s, now | 200 boxes, real-time factor | AMR in the warehouse, real-time factor |
|---|---|---|---|---|
| 1 | −16.4 % | −0.13 % | 2.35 | 68.6 |
| 2 | −9.0 % | −0.14 % | 1.29 | 41.6 |
| 4 | −5.3 % | −0.65 % | 0.66 | 23.8 |
| 8 | −3.2 % | −1.0 % | | |
| 16 | −2.3 % | −1.3 % | | |

Sub-steps no longer help the energy: a loss that grows with their count, about 0.6 % at 4 and
1.3 % at 16, was there before too (the double-precision model of the projection gives −4.65 %
at 4, Jolt −5.3 %) and is not the projection's; TODO(verify) its cause. They still shorten the
step for contacts and stiff joints. `Jolt.APendulumKeepsItsEnergy` holds the loss under 1 % from
1, 2 and 3 rad and any gain under 0.5 %; `Conformance.APendulumKeepsItsPeriodAndEnergy` holds
every backend to the bound in the table above.

Jolt is built with `DOUBLE_PRECISION=OFF` (`vendor/jolt_vendor`). Its free fall after 1 s is
−4.898 mm against the closed form instead of semi-implicit Euler's −4.905 mm. Float32 velocity
and position reproduce it to the last digit (−4.898281 mm). Float32 velocity with double
positions gives −4.900 mm. Jolt's double precision covers positions only, so turning it on would
remove 2 of the 7 µm.

### 8.2 MuJoCo's creep on slopes: noslip and armature

MuJoCo's soft contacts let a body that friction should hold creep down a slope: the comparison's
box on 10° at 0.44 mm/s, a quadruped standing on impedance-held legs at 5.7 mm/s on 10° and
11.6 mm/s on 20° (docs/plan/spikes/legged-contact.md §5, §6). Two opt-in levers:

- `domains.rigid_body.noslip_iterations: N` (profile, 0 to 100, default 0 = off): MuJoCo's noslip
  post-solver (`mjOption.noslip_iterations`). 5 stops the steady creep on 10° (to under 1 µm/s on
  the rigid robot), halves it near the friction limit, costs 30–60 % more per step, and changes
  transients where feet must slip (the legged robot standing up on 20° falls over with it). Jolt
  has none and needs none (its contacts do not creep); other backends ignore the key, with a
  profile warning. `MujocoBackend.NoslipIterationsStopTheCreepOnASlope`.
- Joint armature (`World::Joints::armature`): a motor's `elec:rotorInertia · elec:gearRatio²`
  (`electrical::applyRotorInertia`, always) and an MJCF joint's `armature` plus its actuators'
  `armature · gear²`. MuJoCo takes it as `dof_armature`, Jolt as armature torques (above). Creep
  falls in proportion to the foot's inverse weight. URDF and SDFormat have no such element.

## Contact force

`body/<b>/contact_force` (N, in the world frame) is the sum of what touches a moving body pushes it with, read from each step's contacts, for force sensing and foot contact. On a robot at rest the bodies' forces add up to its weight (`test_contact_forces_on_a_robot_at_rest_carry_its_weight`). Jolt's contact listener gives no impulses per step, so on Jolt it is what is left of a group's momentum change: `m·Δv/Δt − m·g` less its external forces and its joints' forces (below), by Newton's second law; a group of bodies fixed together (one Jolt body) gets it on its root body. `contact/points` above is still MuJoCo only.

`joint/<j>/reaction` (Wrench, world frame, linear then angular) is what the parent does to the child at the joint, including what its motor applies, as a force-torque sensor there reads it. Checked on a hinge holding a known weight (`test_a_joints_reaction_is_what_it_carries`). On Jolt it is the joint constraint's impulses over the step (`GetTotalLambda*`: the point, rotation, limit and motor parts, + on the child, − on the parent, the hinge's rotation part in the basis Jolt builds, `b2 = a2⊥`, `c2 = a2 × b2`) plus the effort applied through the joint. The conformance suite checks both engines: a box at rest is held up by its weight, a still pendulum's pivot carries it with no moment, and a swinging one's carries `m·g + m·d·ω²` at the bottom within 1% (11.349 N on MuJoCo, 11.348 N on Jolt).
