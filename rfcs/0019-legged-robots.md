# RFC 0019: Legged robots as a supported robot class

| | |
|---|---|
| Status | Draft (for the owner to accept or reject) |
| Created | 2026-11-15 |
| Affects | `schemas/joint` (one new applied schema, an addition), `modules/sensors/` (a `foot_contact` kind), `modules/rigid_body/` (contact points on the bus, per-collider contact settings, a backlash and series-elastic transmission), `server/` (an impedance joint mode, contact and path overlays), `bridges/ros2_control/` (impedance command interfaces), `io/importers/` (MJCF contact settings), `python/lagrange/envs.py` (additions), `profiles/schema.json` (additions), `tools/` (a terrain maker), `assets/` (a small hexapod, a terrain course), `examples/` (a balancing quadruped, a biped), `viewer/` (contact arrows, foot paths, support polygon), `docs/` |
| Not affected | `proto/` (`Overlay.KIND_CONTACT_POINT`, `KIND_FORCE_ARROW`, `KIND_PATH` and `SetOverlays.contacts`/`forces` exist since 1.0 and are unused), `core/plugin_abi/` (open question 11) |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0004 (tiers), RFC 0005 (profiles), RFC 0007 (the 1.x freeze), RFC 0008 (plugins, backends), RFC 0009 (walking humans: the closest code), RFC 0010 (electrical nodes), RFC 0017 (MuJoCo 4, Draft) |

## Summary

Make legged robots (quadrupeds, bipeds and humanoids, hexapods) a robot class Lagrange supports
the way it supports AMRs, arms, drones and cars: sample robots that load and stand on first run,
the sensors a legged controller reads (foot contact, joint torque, IMU, encoders), terrain to
walk on (stairs, slopes, rough height fields, friction patches), actuators with the limits that
matter on legs (current limits and heating through the electrical and thermal domains, gearbox
friction, backlash, series elasticity), a joint mode that matches what legged motor drivers take
(position and velocity targets with per-joint stiffness and damping plus a feed-forward torque),
a reference balancing controller as an example plus a tested path to an existing ROS 2 legged
stack, the Gymnasium environment extended for locomotion, viewer overlays for contacts, foot
paths and the support polygon, and validation against closed forms and one published dataset.

Most of the physics already exists. What this RFC mostly adds is measurement (are MuJoCo's soft
contacts and Jolt's rigid ones good enough for legs, at which step), missing plumbing (contact
points never reach the bus or the viewer), and two model additions (transmission, impedance
mode). Every change is an addition under RFC 0007.

## Motivation

- CLAUDE.md §1 lists legged robots as a later robot class; §2 names "other robot classes" as
  post-1.0 work; `docs/design/robot-classes.md` §1 has the row "Legged, marine, climbing |
  post-1.0" with nothing planned.
- `docs/plan/evaluation-2026-11.md` item 6 (Medium): "Legged robots have no controller. A class
  users ask for first after arms". `lagrange_example_quadruped` "trots open loop; no balance
  control, no terrain or contact sensing".
- `docs/validation/comparison.md`, "Limitations of this comparison": "articulated contact-rich
  robots (hands, legs) ... are not covered". Nothing measured says how Lagrange does on legs.
- Lagrange's distinguishing feature (comparison.md, "Coupled domains") matters on legs more than
  on most classes: a legged robot's motors run near their current and thermal limits, its
  battery sags under the stance load, and a policy trained on ideal torque sources meets all of
  that on the real robot. Testing a locomotion controller or a learned policy against the
  electrical and thermal domains is a use no other simulator in the comparison offers.

## What exists today (read for this draft)

| Area | State | Where |
|---|---|---|
| Free-floating branched robots | A URDF whose driven joints branch keeps a free base (legs, as wheels and rotors) | `server/src/Robot.cpp` (`drivenChainBelow`) |
| Example | `lagrange_example_quadruped`: a 12-joint primitive quadruped (138-line URDF, sphere feet), a Python node that puts feet on an ellipse by inverse kinematics and sends positions on `/joint_commands`; heading feedback only, no balance. Logs "trotted 1.00 m forward in 6.7 s" on the machine it was checked on | `examples/lagrange_example_quadruped/` |
| Joint control | `--joint-control`: position, velocity, effort or voltage per joint; position and velocity loops are a critically damped second-order system of a fixed bandwidth (`--joint-loop-hz`, default 8) on each joint's effective inertia, torque clamped to the effort limit. No user stiffness and damping, no feed-forward torque with a position target | `server/include/lagrange/server/JointControl.hpp` |
| ros2_control | `LagrangeSystem` with position, velocity and effort command interfaces; `encoder_resolution` quantises states | `bridges/ros2_control/src/LagrangeSystem.cpp` |
| Contact force | `body/<b>/contact_force` (N, world frame) on both engines (Jolt: from Newton's law per body group); `joint/<j>/reaction` (wrench); bodies' forces on a robot at rest add up to its weight (a test) | `docs/design/domains/rigid_body.md` "Contact force" |
| Contact points | `contact/points` is in the design (rigid_body.md §3) but no backend writes it; the viewer's "Contacts and forces" overlay button says "no data from this server" | `viewer/panels/src/ViewportPanel.cpp` |
| Sensors | IMU (`ImuAPI_1`, noise and bias instability), `bumper` (with `vertical = 1` the whole contact force counts), `force_torque`, `joint_torque`, sensor faults, sensor latency and jitter | `docs/guide/sensors.md`, `docs/design/domains/sensors.md` §9 |
| Joint dynamics | `JointDynamicsAPI_1`: viscous damping, Coulomb friction (MuJoCo `frictionloss`), mimic | `schemas/joint/schema.usda` |
| Motors | `ElectricalMotorAPI_1`: `elec:maxCurrent` (required), `elec:gearRatio`, `elec:gearEfficiency`, `elec:rotorInertia`, velocity and voltage modes, winding temperature through the thermal domain | `schemas/electrical/schema.usda` |
| Terrain | Height fields (USD grid mesh with `lagrange:heightField:columns`/`rows`; MJCF height fields), boxes for stairs, per-link friction (`<mu1>`), `env/friction_scale` from the weather (wet, snow) | `docs/design/formats.md` §2 |
| MuJoCo settings | Elliptic friction cone (`mjCONE_ELLIPTIC`), `implicitfast`; MuJoCo's default contact `solref`/`solimp` and `condim` (no per-collider contact settings) | `modules/rigid_body/mujoco_backend/src/MujocoBackend.cpp` |
| MJCF import | Joint-driving actuators become effort limits; joint equalities become mimics; other equalities (closed loops), tendon actuators and the file's contact settings are noted and dropped | `io/importers/src/MjcfImporter.cpp` |
| Contact accuracy | MuJoCo: a box that should hold on 10° creeps 0.916 mm in 2 s, a resting 2 kg box sinks 0.108 mm. Jolt: creep 0.001 mm, but loses 16.4 % of a free pendulum's energy in 60 s (5.3 % at `substeps: 4`), and keeps sliding and twisting friction on separate budgets | `docs/validation/comparison.md`, rigid_body.md §7, §8.1 |
| RL | `lagrange.envs.LagrangeEnv` (Gymnasium interface; effort, position or velocity actions; `randomize=`), `ParallelEnvs` one simulation per process; no GPU batches. Measured on a sensorless arm: 14,000 steps of 20 ms per second on one process, 45,000 on four | `python/lagrange/envs.py`, `examples/lagrange_example_rl/README.md` |
| Gait code | `modules/humans` walks kinematic capsules (`Gait.cpp`: stance foot planted, leg solved analytically, walk ratio from Sekiya et al. 1996). Kinematic, not dynamic: useful for its foot-placement and phase bookkeeping, not as a controller | `modules/humans/src/Gait.cpp` |

## Use cases

1. **Quadrupeds (ANYmal-, Spot-, Go2-class).** Test a locomotion controller or a learned policy
   on stairs, slopes and rough ground, with the robot's motors, battery and heat in the loop.
   Navigate with Nav2 or a custom stack using a legged odometry estimate.
2. **Bipeds and humanoids.** Standing balance, push recovery, walking on flat ground first. Hard
   contact-rich cases (hands, closed-loop legs like Cassie's) are later (non-goals).
3. **Hexapods.** Statically stable gaits on rough ground: the easiest class to make walk, and the
   one where foot contact sensing decides most.
4. **Learning.** Train small policies with `ParallelEnvs`, or, more realistically, train
   elsewhere (GPU-batched simulators) and **evaluate** the policy in Lagrange against current
   limits, battery sag, motor heating, sensor noise and latency: sim-to-sim transfer as a check
   before hardware.
5. **Hardware sizing.** `lagrange check` already lists what each joint must hold at rest against
   its effort limit and motor; for legs that becomes the stance load and the duty cycle's heat.

## Non-goals

- GPU-batched rigid bodies for RL at scale (thousands of environments). No PhysX (the owner's
  v0.6 decision) and no MJX/MuJoCo Warp (Python/JAX/Warp, not linked: rigid_body.md §1). The
  docs say plainly that Isaac Lab, MJX and similar are the tools for large-scale training.
- A production locomotion controller. Lagrange ships a reference controller as an example and a
  tested integration with an existing stack, not its own MPC.
- Closed kinematic loops (Cassie's leg linkages, parallel ankles), tendons and soft feet in this
  RFC. They need loop constraints in `World` and both backends: a later RFC (open question 9).
- Deformable terrain (sand, mud, snow that compacts). Rigid ground with friction variety only.
- Hydraulic actuators (HyQ-class): the transmission here is electric motor plus gearbox.
- Climbing and marine robots (the same roadmap line): separate RFCs.

## Design

### 1. Sample robots (fetched, pinned, licensed)

Large meshes stay out of the repository (CLAUDE.md §9); `fetch_assets` already fetches pinned,
checksummed models with their licence (`assets/external_assets.yaml`). Candidates whose licences
were read for this draft:

| Robot | Source | Licence as stated | Format | Use |
|---|---|---|---|---|
| ANYmal C | MuJoCo Menagerie `anybotics_anymal_c` | BSD-3-Clause | MJCF | quadruped reference (phase 2) |
| ANYmal C | `ANYbotics/anymal_c_simple_description` ("a simplified robot description (URDF)") | BSD 3-Clause | URDF | the same robot through the URDF path; ros2_control |
| Unitree Go2 | Menagerie `unitree_go2` | BSD-3-Clause | MJCF | small quadruped |
| Unitree Go1, A1, Go2, H1, G1 and others | `unitreerobotics/unitree_ros` (ROS 1 repository; URDF and xacro per robot) | BSD-3-Clause | URDF | URDF path; the repository being ROS 1 does not matter to the importer |
| Boston Dynamics Spot | Menagerie `boston_dynamics_spot` | BSD-3-Clause | MJCF | TODO(verify) the model's provenance and any trademark terms before naming it in docs |
| Unitree H1 / G1 | Menagerie `unitree_h1`, `unitree_g1` | BSD-3-Clause | MJCF | humanoid balance (phase 4) |
| Robotis OP3 | Menagerie `robotis_op3` | Apache-2.0 | MJCF | small humanoid |
| Agility Cassie | Menagerie `agility_cassie` | MIT | MJCF | **not loadable** until closed loops exist (open question 9) |
| MIT Mini Cheetah | `mit-biomimetics/Cheetah-Software` | MIT | TODO(verify): whether the repository holds a usable robot description | — |
| Hexapod | none verified | — | — | TODO(verify) an openly licensed hexapod description; meanwhile a primitive one of Lagrange's own (below) |

Menagerie lists no hexapod. In the repository: the existing primitive quadruped stays, and a
primitive hexapod (boxes, capsules, spheres; written for Lagrange, Apache-2.0, a few kB) is
added, so every test and tutorial runs without a download. Each fetched robot gets an overlay
(`*.overlay.yaml`, formats.md §9) adding motors and a battery: the motor constants are the
user's to give. Where a maker publishes actuator figures they are cited; otherwise the overlay
says *illustrative* (sensors.md's rule), and no number is invented here.

The MJCF importer drops the file's contact settings (`solref`, `solimp`, `condim`, `priority`,
`margin`), which menagerie models are tuned with. §4 carries them (open question 4).

### 2. Sensors

- **`foot_contact`** (new built-in sensor kind: no schema change; URDF `<sensor
  type="foot_contact">`, USD `CustomSensorAPI_1` with `sensor:custom:kind = "foot_contact"`, as
  every built-in kind). It reads the foot body's `body/<b>/contact_force`, or, when `point` and
  `radius` are given (a foot sphere at the end of a shin that may also touch with its knee), only
  the contacts within `radius` of `point` from `contact/points` (§4). Parameters: `threshold`
  (N), `hysteresis` (N), `noise` (1 sigma, N), `range` (N), `resolution` (N), `cutoff_frequency`
  (Hz), `mode` (`switch` | `force` | `force3d`), all ideal by default, and no default a datasheet
  should give (the `bumper`'s rule: a default marked *illustrative* is a placeholder). Signals:
  `sensor/<n>/contact` (Bool), `force` (N, normal), `force3d` (Vec3, sensor frame), `saturated`.
  ROS: `/<n>/contact` as `std_msgs/Bool` and `/<n>/force` as `geometry_msgs/WrenchStamped`
  (TODO(verify) whether a common legged message exists in Lyrical that controllers expect;
  quadruped_ros2_control reads foot force through ros2_control state interfaces, §5).
- **Joint encoders and torque.** The existing `joint_torque` kind and ros2_control's
  `encoder_resolution`. Added: `encoder_resolution` and a velocity-from-position filter on the
  plain `/joint_states` path as well (`--joint-control` users), and an "estimated torque from
  current" quantity (`joint/<j>/estimated_torque`: the drive's `i·kt·N·η`, what most legged
  robots without torque sensors report), written by the electrical domain when it runs.
- **IMU** as it is. A legged guide section on placing it on the base and giving its noise
  figures.
- **State estimation inputs.** Lagrange does not estimate state: it supplies the inputs at a
  common stamp (IMU, joint states, foot contacts, with their sensor latency) and ground truth on
  `/ground_truth/odom`. For a legged robot `/odom` stays ground truth unless the user maps
  another source (open question 6). A test checks that the inputs of one control period carry
  one stamp.

### 3. Terrain

- `ros2 run lagrange_terrain lagrange_make_terrain KIND OUT.usda [options]` (beside
  `lagrange_make_fields`), deterministic from a seed (`core/random`, Philox): `stairs` (rise,
  run, count, width), `slope` (angle), `rough` (a height field with a given RMS height and
  correlation length), `stepping_stones`, `gaps`, `friction_patches` (tiles with friction values
  given by the user), `curriculum` (a grid of the above at increasing difficulty, as RL terrain
  curricula do). Output is a USD world with height fields and boxes, each tile's friction on its
  colliders, under 1 MB.
- Sample world `assets/worlds/legged_course/` made by it, and the docs show the weather's
  `env/friction_scale` (rain, snow) on the same course.
- No new schema: height fields, boxes and per-collider friction exist.

### 4. Physics

Requirements, each with the test that checks it (§Validation):

- **Contact points on the bus.** Implement `contact/points` (designed, unwritten): per contact,
  the two bodies, the point (world-local, double), the normal, the normal force and the friction
  force. MuJoCo: `mj_contactForce` per contact. Jolt: points and normals from the contact
  listener; per-contact forces are not given per step (rigid_body.md "Contact force"), so on
  Jolt only the per-body sum is a force, and a point's force is that sum shared by penetration
  depth, documented as an approximation (open question 3). Feeds `foot_contact` with `point` and
  the viewer.
- **Per-collider contact settings** (`ContactAPI_1`, §6) read by both backends where they have
  an equivalent: MuJoCo `solref`/`solimp`-style stiffness and damping time constant, `condim`
  (3, 4 with torsional, 6 with rolling friction), torsional and rolling friction coefficients;
  Jolt: friction and restitution only, the rest logged as not supported. The MJCF importer
  carries menagerie's values through.
- **Friction cone.** The elliptic cone stays (comparison.md: within 136 ppm of Coulomb on the
  slide). Torsional friction matters for point feet turning in place: `condim` 4 on feet.
- **Step size.** 1 ms stays the default. A legged profile is measured at 1 ms, 0.5 ms and 2 ms
  (foot penetration, contact-force ripple, energy in a drop, real-time factor) and the guide
  gives the numbers; no step is recommended before that measurement. TODO(verify) what the
  usual RL frameworks use (a coarser physics step with the policy decimated) before comparing.
- **Creep on slopes.** MuJoCo's soft contacts creep (0.9 mm in 2 s on 10° for a box). A standing
  legged robot on a slope creeps too. Measured on the stand-on-slope test; MuJoCo's `noslip`
  iterations are offered as a profile key if they remove it (TODO(verify) the effect on 3.12
  and 4.x).
- **Backends.** MuJoCo (joint coordinates, energy-preserving, the deterministic reference) is the
  legged reference. Jolt is run through every legged test and its results reported, not hidden:
  its joint energy loss and its separate twisting-friction budget (rigid_body.md §7) may matter
  for swinging legs and pivoting feet; that is measured, not assumed. RFC 0017 (MuJoCo 4) helps
  contact-heavy scenes; this RFC does not depend on it.
- **Statically indeterminate stance.** Four feet on a plane share the weight indeterminately;
  Jolt's rigid contacts already showed this on the AMR's wheels and casters (rigid_body.md §7).
  The sum must equal the weight on both engines; the split is reported per engine.
- **Determinism.** `lagrange determinism` on a legged profile (MuJoCo, CPU lidar off) joins the
  determinism job: bit-identical state hashes every 100 ms (invariant 7).

### 5. Actuators and joint modes

- **Impedance mode** (the low-level command most legged robots' motor drivers take: a position
  and velocity target, stiffness `kp`, damping `kd` and a feed-forward torque, `τ = kp·(q* − q) +
  kd·(q̇* − q̇) + τ_ff`, clamped to the effort limit). Added to `server::JointControl` as mode
  `Impedance`; ros2_control: `LagrangeSystem` accepts the command interfaces `position`,
  `velocity`, `effort`, `stiffness`, `damping` claimed together (custom interface names are
  allowed by ros2_control; TODO(verify) the names quadruped_ros2_control uses, e.g. `kp`/`kd`,
  and accept both); Python: `sim.command_joint(j, "impedance", q, dq=..., kp=..., kd=...,
  tau=...)`; `/joint_commands` is unchanged (JointState has no field for gains; open question 5).
  With the electrical domain on, the torque goes through the motor drive like any torque command,
  so its current limit and the bus voltage bound it.
- **Transmission** (`JointTransmissionAPI_1`, §6): backlash (a dead band in rad between motor
  and link, modelled as a spring-damper with a gap on the link side of the joint), a series
  elastic element (spring stiffness and damping between motor and link: a second, internal
  degree of freedom per joint), and a load-dependent gearbox friction beyond `joint:friction`
  (open question 7). The backlash and spring need a motor-side inertia: `elec:rotorInertia ·
  N²`, which MuJoCo already adds as armature. Implemented in the backends as an extra hidden
  joint (MuJoCo: a body of the motor's reflected inertia between parent and link) so contacts
  and the integrator see it; Jolt the same through a constraint (measured).
- **Current limits and heat.** Existing: a legged overlay with 12 (or 18) motors, a battery and a
  bus. The guide shows `elec:maxCurrent` capping torque, back-EMF capping speed, and windings
  heating through a long stance. `lagrange check` extends to a legged robot standing: each
  joint's stance torque against its continuous rating (open question 8 on where that rating
  lives).

### 6. Schema additions (RFC 0007: additions only, core-maintainer review, 2 approvals)

Both in `schemas/joint` (new classes, nothing changed in `JointDynamicsAPI_1`), with URDF
`<gazebo reference=...>` and overlay forms, and `test_usd_assets`/`frozen_1_0.json` untouched:

**`ContactAPI_1`** (single apply, on a collider prim or a bound material; resolved like
`SensorMaterialAPI_1`):

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `contact:timeConstant` | double | s | none (engine default) | contact stiffness as a time constant (MuJoCo `solref[0]`) |
| `contact:dampingRatio` | double | 1 | none | (MuJoCo `solref[1]`) |
| `contact:impedance` | double[] | 1 | `[]` | MuJoCo `solimp`, passed through; other engines ignore it with a note |
| `contact:dimension` | int | 1 | `3` | 1 frictionless, 3 sliding, 4 + torsional, 6 + rolling |
| `contact:torsionalFriction` | double | m | `0` | torsional coefficient (MuJoCo `friction[1]`) |
| `contact:rollingFriction` | double | m | `0` | rolling coefficient (MuJoCo `friction[2]`) |
| `contact:restitution` | double | 1 | `0` | Jolt; MuJoCo by a negative `solref` (TODO(verify) the mapping) |

"none" means the engine's own default, kept as today. Sliding friction stays where it is
(`Colliders::friction`, `<mu1>`).

**`JointTransmissionAPI_1`** (single apply, on a joint):

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `joint:transmission:backlash` | double | rad (m on a slide) | `0` | total dead band, link side |
| `joint:transmission:stiffness` | double | N·m/rad (N/m) | none: rigid | series elastic spring |
| `joint:transmission:damping` | double | N·m·s/rad (N·s/m) | `0` | across the spring |
| `joint:transmission:loadFriction` | double | 1 | `0` | Coulomb friction growing with transmitted torque (share of it) |

No numeric fallback is a device value: `0` means "off", "none" means "rigid".

The impedance mode, the `foot_contact` kind, `estimated_torque` and `contact/points` are not
schema changes. Profile additions (`profiles/schema.json`, additions): `rigid_body.noslip_iterations`,
`rigid_body.contact_defaults` (the `ContactAPI_1` keys for every collider without its own).

### 7. Controllers

- **Reference controller, as an example** (`lagrange_example_quadruped` grows, or a new
  `lagrange_example_legged`): a closed-loop trot in Python with Raibert-style foot placement
  (Raibert, *Legged Robots That Balance*, MIT Press, 1986) and body attitude and height held by
  stance-leg forces mapped through the leg Jacobians (`τ = Jᵀ F`), in impedance mode. Its job is
  to show the sensors and modes and to be read, not to compete with MPC; its README says what it
  cannot do. No controller goes into the server (unlike the multicopter's built-in hover, which
  exists so a drone sample does something on first run; open question 1).
- **Integration with an existing ROS 2 stack.** Candidates read for this draft:

  | Stack | Licence | ROS | Fit |
  |---|---|---|---|
  | `legubiao/quadruped_ros2_control` | Apache-2.0 | ROS 2; default branch Jazzy, `humble` branch | Best fit: ros2_control-based, OCS2 MPC, RL and "unitree guide" controllers, runs on MuJoCo and Gazebo Harmonic. TODO(verify) it builds on Lyrical |
  | `leggedrobotics/ocs2` | BSD 3-Clause | ROS 1 on `main`, ROS 2 on branch `ros2` | MPC library underneath the above; pinocchio and hpp-fcl through rosdep on Lyrical: TODO(verify) |
  | `chvmp/champ` | BSD-3-Clause | ROS 1 (tested Kinetic, Melodic); ROS 2 not mentioned on its page | TODO(verify) a maintained ROS 2 port before relying on it |
  | `qiayuanl/legged_control` | BSD-3-Clause | ROS 1 (Noetic) | Not usable on Lyrical |

  Proposed: an example package `lagrange_example_legged_ros2_control` that builds only when
  quadruped_ros2_control is found (like `lagrange_autoware`), running its controller on
  `LagrangeSystem` with a fetched Go2 or ANYmal C. Its hardware interface names (joint
  `kp`/`kd`, foot force, IMU) decide §5's interface names.

### 8. Learning

- `LagrangeEnv(mode="impedance", ...)`: actions as position offsets around a default pose with
  fixed `kp`/`kd`, the common locomotion action space.
- Observation helpers named in `signals=`: base angular velocity and projected gravity from the
  IMU, foot contacts, commanded velocity; a `command=` sampler.
- `randomize=` extends to friction, payload mass, motor strength (`elec:maxCurrent` scale) and
  terrain tiles from `lagrange_make_terrain curriculum`; `reset` with a random initial base pose.
- **A policy runner without new dependencies**: an example that loads an MLP's weights from
  `.npz` and runs it in numpy in impedance mode, so a policy trained elsewhere can be evaluated
  in Lagrange with its electrical and thermal domains. ONNX Runtime would be a new dependency
  (TODO(verify) licence and Ubuntu 26.04 packaging) and is not proposed.
- **Speed, measured not promised.** The arm's 14,000 steps of 20 ms per second (one process) is
  not a legged number. Phase 1 measures a 12-joint quadruped on flat ground and on the rough
  course, one and N processes, with and without the electrical domain, and publishes it in
  comparison.md. The guide states the scale limit (non-goals).

### 9. Viewer

The protocol already has what is needed (no `proto/` change):

- **Contact points and force arrows**: the server fills `Overlay.KIND_CONTACT_POINT` (point,
  normal, magnitude) and `KIND_FORCE_ARROW` from `contact/points` when the viewer's
  `SetOverlays.contacts`/`forces` asks; the Viewport's "Contacts and forces" button, today
  disabled with "no data from this server", turns on. Arrow length scale in the overlay's
  settings; friction part drawn in the cone colour of the design tokens.
- **Foot trajectories**: `KIND_PATH`, each foot's last N seconds (default 2) as a polyline,
  coloured by stance and swing.
- **Support polygon and centre of mass**: the convex hull of the stance feet as a closed
  `KIND_PATH` on the ground and the centre of mass projected down (open question 10 on a
  dedicated kind). The centre of mass is already computable from `World` and body poses.
- **Scopes**: foot forces, joint torques against limits, winding temperatures, as signals.
- Per `docs/design/ui/`: the overlay specs are added to the Viewport's SPEC before code, with
  screenshots (the binding UI design).

## Validation

Each model and claim ships with a test (CLAUDE.md §10). Closed forms first; no tolerance below is
promised before it is measured, except where marked as a bound the test enforces.

1. **Weight at rest.** A standing quadruped (primitive, then fetched): the sum of feet
   `contact/points` normal forces and the sum of `foot_contact` forces equal m·g (the existing
   rule for `body/<b>/contact_force`). The split per foot is reported per engine, not asserted
   (indeterminate).
2. **Symmetric stance.** A symmetric robot with its centre of mass over the centre of its four
   feet: on MuJoCo each foot carries m·g/4 by symmetry; reported for Jolt.
3. **Static leg torques.** A two-link leg holding a known load at a known pose: each joint's
   torque equals `Jᵀ F` computed in closed form (as `lagrange check`'s `jointLoads`).
4. **Drop.** A body dropped onto the ground: the contact impulse over the impact equals the
   momentum change; peak penetration and settling time per engine and step reported.
5. **Friction on a slope.** A standing robot on slopes below and above `atan μ`: holds below,
   slides above; creep below reported (MuJoCo's soft contact).
6. **Rimless wheel.** A passive rimless wheel down a slope: with mass at the hub and a no-slip,
   no-bounce impact, the angular velocity after a spoke's impact is cos 2α times that before it
   (α half the inter-spoke angle; McGeer, "Passive Dynamic Walking", IJRR 9(2), 1990; Coleman,
   Chatterjee and Ruina, *Dynamics and Stability of Systems* 12(3), 1997, TODO(verify) volume and
   pages), and its steady rolling speed follows. A direct test of the contact impulse model on
   both engines; soft contacts are expected to deviate, and by how much is reported.
7. **Simplest walker** (stretch): the passive compass-gait model of Garcia, Chatterjee, Ruina and
   Coleman ("The Simplest Walking Model", J. Biomech. Eng. 120(2), 1998) on a shallow slope;
   does Lagrange keep its period-one gait from the published initial conditions (TODO(verify)
   the slope range and conditions from the paper). Foot scuffing at mid-swing is ignored in the
   model and must be handled (e.g. a retractable foot), documented.
8. **Transmission.** Backlash: a joint reversing under no load moves the motor through the dead
   band with the link still. Series elastic: the spring–inertia natural frequency `√(k/J)`
   against a closed form.
9. **Same model, same answer.** A menagerie quadruped imported from MJCF and standing, in
   Lagrange-on-MuJoCo and standalone MuJoCo at the same version and settings: joint trajectories
   match to the comparison's "Lagrange adds no error" standard. Needs §4's carried contact
   settings.
10. **Determinism.** Two runs of a legged profile, bit-identical hashes (MuJoCo).
11. **A published dataset.** The HyQ proprioceptive dataset (G. Fink, Istituto Italiano di Tecnologia, *Proprioceptive
    Sensor Dataset for Quadruped Robots*, IEEE DataPort, doi:10.21227/4vxz-xw05, CC BY 4.0:
    encoders, force and torque sensors, MEMS and fibre-optic IMUs, motion-capture ground truth).
    Used in bench mode (`rigid_body` replay substitute): the recorded motion drives the robot and
    Lagrange's IMU and encoder models are compared with the recorded ones. HyQ is hydraulic, so
    its actuators are out of scope (non-goals), and a HyQ description with a usable licence is
    TODO(verify). The Mini Cheetah contact data of "Proprioceptive Invariant Robot State
    Estimation" (arXiv 2311.04320) is a second candidate for contact labels; TODO(verify) its
    availability and licence. No dataset of a legged robot walking with measured foot forces and
    a public model was found for this draft; open question 12.

Each passes on CPU (Tier 0) in CI; nothing here needs a GPU.

## Tiers

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Physics, sensors, actuators, terrain | CPU, deterministic | same | same (no GPU physics) |
| Viewer overlays | software Vulkan (llvmpipe) | yes | yes |
| Learning | one simulation per process | same; cameras on the GPU | same |

## Steps (each a PR)

1. **Measure first.** A legged test profile on the primitive quadruped and hexapod; tests 1–5
   and 10 on both engines at 0.5, 1 and 2 ms; the RL speed; results into comparison.md and
   rigid_body.md. No model changes. Decides §4's open points.
2. **Contact points.** `contact/points` in both backends; server overlays; Viewport button live;
   the `foot_contact` kind; screenshots.
3. **Impedance mode.** `JointControl`, Python, ros2_control interfaces; the reference controller
   example closed-loop on flat ground and the primitive hexapod's tripod gait.
4. **Schemas** (`ContactAPI_1`, `JointTransmissionAPI_1`; RFC 0007 review, 2 approvals); MJCF
   importer carries contact settings; test 9 with a fetched menagerie quadruped.
5. **Transmission** in both backends; test 8; `estimated_torque`.
6. **Terrain maker** and `legged_course`; the controller on stairs and slopes as far as it goes,
   honestly reported.
7. **Fetched samples and overlays** (ANYmal C, Go2, a humanoid standing); `lagrange check` for
   stance.
8. **quadruped_ros2_control example** (when it builds on Lyrical).
9. **Learning additions** and the numpy policy runner; tests 6, 7, 11.
10. **Docs**: a legged tutorial with its `tools/ci/tutorial_legged.sh`, docs/physics pages,
    robot-classes.md row filled in, a feature video.

## Open questions (with a proposed answer)

1. *A controller in the server, like the multicopter's hover?* No. Examples only; the first-run
   sample stands in impedance mode at its default pose, which already shows something useful
   (stance loads, motor heat). Revisit if users ask.
2. *Which backend for legs?* MuJoCo as the reference and default; Jolt supported and measured.
   Step 1's numbers may change this for Jolt in either direction.
3. *Per-contact forces on Jolt?* Proposed: the per-body sum is exact (Newton), the per-point
   share is by penetration depth and documented as an approximation; `foot_contact` with
   `point`/`radius` on Jolt warns once. Alternative: Jolt's `ContactListener` with manifold
   impulses after the solve, if a later Jolt exposes them (TODO(verify)).
4. *Carry MJCF contact settings into USD?* Yes, through `ContactAPI_1`; anything without an
   equivalent (MuJoCo `priority`, `margin`, `gap`) is noted, as other dropped elements are.
5. *Impedance commands over plain ROS?* Not on `/joint_commands` (JointState has no gains).
   Proposed: ros2_control is the ROS path; Python and the plugin's bus signals are the others.
   A Lagrange message package for one topic is not worth it.
6. *What is `/odom` for a legged robot?* Ground truth, as today, named so in the guide; a user's
   estimator publishes its own. A built-in leg odometry is out of scope.
7. *Load-dependent gearbox friction: in the transmission or in the electrical drive?* In the
   transmission (mechanical); `elec:gearEfficiency` stays the drive's power loss. The two must
   not double-count: the docs say to use one.
8. *Continuous torque ratings for `lagrange check`?* From the motor's thermal model (the winding
   temperature reaching its limit at a constant current) rather than a new attribute; no new
   schema.
9. *Closed loops (Cassie, parallel ankles)?* A later RFC: loop constraints in `World::Joints`,
   both backends, importers. Cassie stays listed as not loadable.
10. *A support-polygon overlay kind?* Use a closed `KIND_PATH` now; append `KIND_POLYGON` (a
    filled shape) only if the viewer's spec needs a fill. Appending an enum value is allowed
    under RFC 0007.
11. *Plugin ABI?* Nothing in this RFC; a plugin reads `contact/points` as a bus signal. A typed
    contact getter would be an ABI MINOR append in its own RFC.
12. *A walking dataset with foot forces?* Ask the owner whether to request one from a lab
    (ANYbotics, the Istituto Italiano di Tecnologia, MIT); until then validation is the closed forms, the standalone-MuJoCo
    match, and the HyQ bench comparison.

## References checked for this draft (2026-11-15)

- MuJoCo Menagerie README (github.com/google-deepmind/mujoco_menagerie): the legged models and
  licences in §1's table.
- `ANYbotics/anymal_c_simple_description` README: "released under a BSD 3-Clause license".
- `unitreerobotics/unitree_ros`: BSD-3-Clause; robot list; ROS 1 (catkin).
- `mit-biomimetics/Cheetah-Software`: MIT licence; robot description not confirmed.
- `legubiao/quadruped_ros2_control`: Apache-2.0; Jazzy default, `humble` branch; OCS2, RL and
  unitree-guide controllers; MuJoCo and Gazebo Harmonic.
- `leggedrobotics/ocs2`: BSD 3-Clause; `ros2` branch.
- `chvmp/champ`: BSD-3-Clause; tested on Kinetic and Melodic.
- `qiayuanl/legged_control`: BSD-3-Clause; ROS 1 Noetic, built on OCS2.
- IEEE DataPort, *Proprioceptive Sensor Dataset for Quadruped Robots* (G. Fink, Istituto Italiano di Tecnologia),
  doi:10.21227/4vxz-xw05, CC BY 4.0.
- arXiv 2311.04320 (*Proprioceptive Invariant Robot State Estimation*): abstract only.
- McGeer 1990, Coleman et al. 1997, Garcia et al. 1998, Raibert 1986: cited from the literature,
  not re-read for this draft; the TODO(verify) marks above stand until they are.

Everything else marked TODO(verify) is to be read from its source before the step that uses it.
