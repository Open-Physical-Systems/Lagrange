# RFC 0023: Joint springs

| | |
|---|---|
| Status | Draft (for the owner) |
| Created | 2026-11-19 |
| Affects | `schemas/joint/` (two appended attributes), `core/world/` (two `World::Joints` columns), `io/importers/` (SDFormat, MJCF), `io/stage/` (read and write), `server/` (URDF `<gazebo reference>` form, inspector, `lagrange check`), `modules/rigid_body/` (MuJoCo and Jolt backends), `core/plugin_abi/` (two fields appended to `lagrange_joint_info`, the C++ SDK), `viewer/` (builder rows), `server/test/physics_conformance.cpp`, `docs/` |
| Not affected | `proto/` (the inspector reads schema attributes through the existing `GetAttributes`), `profiles/schema.json`, recordings (no new signal unless open question 4 says so) |
| Builds on | RFC 0002 (schemas), RFC 0007 (the 1.x freeze), RFC 0008 (plugins, the rigid-body contract), #199 (joint damping, friction and mimic from every format) |
| Review | Touches `schemas/` and `core/plugin_abi/`: core-maintainer review and **2 approvals** (CLAUDE.md §9, CODEOWNERS) |

## Summary

A joint today can be damped, have dry friction and copy another joint (`JointDynamicsAPI_1`),
but it cannot be springy. This RFC adds a linear spring to every hinge and slide:
`joint:stiffness` (N·m/rad on a hinge, N/m on a slide) and `joint:springReference` (rad or m,
where the spring pushes nothing), so that

$$
\tau_{joint} = \tau - k\,(q - q_0) - d\,\dot q - f\,\mathrm{sign}(\dot q).
$$

SDFormat (`<axis><dynamics><spring_stiffness>`, `<spring_reference>`) and MJCF (`stiffness`,
`springref`) already carry it, and Lagrange drops it on import. URDF has no spring, so it takes
Gazebo's URDF extension (`<gazebo reference="JOINT"><springStiffness>`, `<springReference>`).
MuJoCo applies the spring itself. Jolt applies it as a joint torque, like the damping it already
applies. Both fallbacks are 0, so every existing robot runs as before. Everything is an addition
under RFC 0007.

## Motivation

- **Imports lose data.** An SDFormat or MJCF robot with springs (passive gripper fingers, a
  leaf-spring suspension, a compliant foot, a door on a closer, the MuJoCo Menagerie models that
  use `stiffness` for passive joints) runs with them silently missing. The importer report does
  not list them either, because nobody reads the two values.
- **Passive mechanisms are common.** Spring-return valves, sprung caster forks, return springs on
  levers, torsion-spring hinges and flexures modelled as a hinge are all one stiffness and one
  rest angle. Today a user needs a plugin that writes `joint/<j>/load_torque`, and only actuated
  joints have that input.
- **The comparison baseline has it.** Gazebo Jetty reads the same SDFormat elements.
  TODO(verify): which gz-physics engines (DART, Bullet-Featherstone) apply `spring_stiffness`
  in gz-sim 10.

## Non-goals

- **Nonlinear springs.** MuJoCo 3.12 has `stiffness[mjNPOLY+1]` with polynomial terms
  (`jnt_stiffnesspoly`). Only the linear term is in this RFC. The importer reports the others as
  dropped.
- **Series elastic actuators.** A spring between a motor and its link is
  `joint:transmission:stiffness` in RFC 0019 (Draft): a second state per joint. This RFC is the
  spring between parent and child, in parallel with whatever drives the joint.
- **Springs on ball and free joints.** `JointDynamicsAPI_1` is for revolute and prismatic joints
  only (its doc string). MJCF ball-joint stiffness is reported as dropped.
- **Contact and limit softness.** RFC 0019 `contact:timeConstant` and Jolt's
  `mLimitsSpringSettings` are separate.
- **Tendon springs** (MJCF `<tendon stiffness springlength>`): tendons are not imported.

## Design

### 1. Schema: two attributes appended to `JointDynamicsAPI_1`

Appended in `schemas/joint/schema.usda` after `joint:mimicOffset`, so the `_1` version stays.
Precedent: owner decisions D55 and D56 (2026-11-15) added `joint:armature` and joint limits to
`_1` the same way.

| Attribute | Type | Unit (`lagrangeUnit`) | Fallback | Meaning |
|---|---|---|---|---|
| `joint:stiffness` | double | `N*m/rad or N/m` | 0 | Linear spring between parent and child, the first on a hinge, the second on a slide. 0: no spring. Negative values are refused at load. |
| `joint:springReference` | double | `rad or m` | 0 | The joint position where the spring pushes nothing, in the joint's own coordinate (the same zero as `lowerLimit`, `upperLimit` and `joint/<j>/position`). Counts turns on a continuous hinge. |

`test_every_numeric_attribute_has_a_unit` covers both. A new test in `schemas/test/test_schemas.py`,
`test_joint_springs_are_additions`, checks the two names and fallbacks, as
`test_gear_back_efficiency_is_an_addition` does for RFC 0020. `JointDynamicsAPI_1` is not in
`frozen_1_0.json` (it came in 1.1, #199), but RFC 0007 holds it anyway: only appending.

`springReference` outside the joint limits is allowed (a preloaded spring, such as a closer that
pushes a door against its stop) and noted in the import report.

### 2. World tables

`World::Joints` (`core/world/include/lagrange/world/World.hpp`) gets two columns next to
`damping` and `friction`:

```cpp
/// A linear spring at the joint: τ = −stiffness · (q − springReference).
/// N·m/rad or N/m; rad or m (counting turns). 0 and 0 for none.
std::vector<double> stiffness, springReference;
```

`World::addJoint` (`core/world/src/World.cpp`) pushes 0 and 0. Every place that copies a joint
row (spawning and deleting robots, `--add-robot` prefixing, region splitting, the floating-origin
rebuild) already goes through the whole `Joints` struct. TODO(verify) in step 2 that none of them
copies columns one by one.

A spring is a property of the model, not state: `Server::saveState`/`restoreState` and the
checkpoints need nothing.

### 3. Importers

| Format | Source | Into Lagrange | Notes |
|---|---|---|---|
| SDFormat | `<axis><dynamics><spring_stiffness>`, `<spring_reference>` (spec 1.5 and later; `sdf/1.12/joint.sdf` lines 84–89 and 144–149 in the installed `sdformat_vendor`) | `sdf::JointAxis::SpringStiffness()`, `SpringReference()` (`gz/sdformat16/sdf/JointAxis.hh`) in `SdfImporter.cpp` next to `axis->Damping()` | `axis2` of a universal joint has them too. Universal joints are not imported, so they are not read. |
| MJCF | `<joint stiffness springref springdamper>` | from the compiled model, as damping is read today (`MjcfImporter.cpp` reads `dof_damping`): `jnt_stiffness[j]`, and `qpos_spring[jnt_qposadr[j]] − qpos0[jnt_qposadr[j]]` | Lagrange's joint zero is MJCF's `ref` (the importer already notes "its rest pose is at ref"), so the reference is shifted by `ref`. `springdamper` is compiled into `jnt_stiffness` and `dof_damping`, so it is covered. Non-zero `jnt_stiffnesspoly` entries and a ball joint's stiffness are reported as dropped. TODO(verify) the `ref` shift against a model with `ref ≠ 0` in step 3. |
| URDF | none in the URDF spec. Gazebo's URDF extension: `<gazebo reference="JOINT"><springStiffness>K</springStiffness><springReference>Q0</springReference></gazebo>`, the names sdformat's `parser_urdf` reads (both strings are in the installed `libsdformat.so`) | `server/src/Robot.cpp`, in the existing `<gazebo reference>` loop (`mu1`, `visual_material`, `sensor_material`), matched against joint names as well as link names | A reference that names no link and no joint is noted, as today. TODO(verify): read `parser_urdf.cc` to confirm the element names and that they sit directly under `<gazebo reference>`. |
| USD | `joint:stiffness`, `joint:springReference` | `io/stage/src/StageReader.cpp`, next to `joint:damping` | |

`Description.hpp`'s `Joint` gets `stiffness` and `springReference` (doubles, 0 by default), and
`Description.cpp` copies them where it copies `damping`. `io/stage/src/StageWriter.cpp` applies
`JointDynamicsAPI_1` when `stiffness > 0` too, and writes both. That means `lagrange_import`
keeps springs, and `test_usd_assets` keeps the samples in step.

### 4. MuJoCo backend

In `MujocoBackend.cpp`, where `joint->damping[0]` and `joint->frictionloss` are set for a hinge
or slide:

```cpp
joint->stiffness[0] = joints.stiffness[j];  // N·m/rad or N/m, the linear term
joint->springref = joints.springReference[j];
```

`mjsJoint::stiffness[mjNPOLY+1]` and `springref` are in the installed MuJoCo 3.12 `mjspec.h`
(lines 310–313). They compile to `jnt_stiffness` and `qpos_spring`. MuJoCo then adds the spring
to its passive forces. The backend builds joints with `ref = 0` (Lagrange's zero), so
`springref` needs no shift. TODO(verify) the last point in `MujocoBackend.cpp` during step 4.

MuJoCo couples a mimic follower both ways, so a spring on a follower also loads the leader, as a
geared spring would.

Integration: the backend runs `mjINT_IMPLICITFAST`. TODO(verify) in MuJoCo's Computation chapter
whether joint stiffness is integrated implicitly or only damping is. If only damping is, the
explicit stability bound of §5 also applies to MuJoCo, and the loader checks it for both engines.

RFC 0017 (MuJoCo 4, Draft): `stiffness` and `springref` exist in 4.x as well. TODO(verify)
against the 4.0.1 headers when that RFC moves.

### 5. Jolt backend

Jolt 5.6.0 (`vendor/jolt_vendor`) has two ways to do it:

1. **A joint torque**, added where the explicit damper is added today (`JoltBackend::step`,
   `damper` next to `rotor`):
   `spring = −k · (q − q₀)`, with `q` the hinge's turn-counting `j.angle` or the slider's
   `GetCurrentPosition()`. It is applied equal and opposite to child and parent about the joint
   axis, the same code path as `torque`, `rotor` and `damper`.
2. **The constraint's motor as a spring.** `HingeConstraint`/`SliderConstraint` in
   `EMotorState::Position`, with `MotorSettings::mSpringSettings` in
   `ESpringMode::StiffnessAndDamping` (`SpringSettings.h`: "the damping (c) in the spring
   equation … T = -k * theta - c * w for an angular spring"), target `q₀` by `SetTargetAngle` /
   `SetTargetPosition`. This is implicit (soft constraint), so it is stable at any stiffness.

**Proposed: (1), the explicit torque.** Reasons:

- The motor is taken. Mimic joints use it (`kMimicMotorHz`), and a later position mode (RFC 0020)
  may need it too.
- `SetTargetAngle` wraps to ±π and clamps to the limits (`HingeConstraint.h` line 123). A
  continuous hinge wound three turns, or a preloaded spring with `q₀` past a stop, would get the
  wrong torque. `j.angle` counts turns, as MuJoCo's position does.
- It is the same path as the existing damping, so `joint/<j>/reaction`, `applied` and the
  conformance numbers come out consistently.

Stability: an explicit spring under Jolt's symplectic Euler is stable for `ω·dt < 2`, with
`ω = √(k / I)`. Following `kMaxDampingShare`, the backend caps the stiffness it applies at
`kMaxSpringShare · I_rest / dt²` (proposed `kMaxSpringShare = 1`, so `ω·dt ≤ 1`). It logs a
warning once per joint when it caps one, naming the joint, the asked and the applied value, and
`domains.rigid_body.substeps` as the cure. At 1 ms this caps nothing below `I · 10⁶` N·m/rad,
which covers every sample and imported spring met so far. TODO(verify) the claim over the
Menagerie models in step 4.

On a mimic follower, Jolt only moves the copy (its motor overrides the spring's effect on it),
and nothing reaches the leader. This is the same documented difference that mimic already has
(docs/physics/rigid-body.md). The loader notes it.

The plugin form of Jolt (`jolt_plugin.cpp`, `lagrange_jolt`) reads the joint from
`lagrange_joint_info` (§6), so it gets the spring for free through `JoltBackend::SignalIo`.

### 6. Plugin ABI: appended to `lagrange_joint_info`

```c
    double mimic_multiplier, mimic_offset; /* position = multiplier × mimic's + offset */
    /* Appended (RFC 0023): check struct_size. A joint spring: τ = −stiffness·(q − spring_reference). */
    double stiffness;        /* N·m/rad or N/m; 0: none */
    double spring_reference; /* rad or m, counting turns */
} lagrange_joint_info;
```

`test_abi_layout.c`: `PIN(lagrange_joint_info, 168)`, `AT(lagrange_joint_info, stiffness, 152)`,
`AT(lagrange_joint_info, spring_reference, 160)`. Every earlier offset is unchanged.
`tools/ci/abi_check.sh` (abidiff) should report a size growth only.

Compatibility both ways works because the fallback is zero. The host's `fill()` copies
`min(out->struct_size, sizeof)` bytes. A new plugin on an older host keeps its own zero-initialised
tail, which reads as "no spring". An old plugin on a new host never sees the new fields.

**The C++ SDK needs one fix in the same step.** `plugin_sdk/World.hpp` guards `mimic` with
`in.struct_size >= sizeof in`. Because `fill()` keeps the *plugin's* `struct_size`, that check is
always true for the plugin's own build. That is harmless today and stays harmless (an older host
leaves `mimic` at the SDK's zero-init, not `LAGRANGE_NO_MIMIC`, so TODO(verify) whether the SDK
should pre-set `in.mimic = LAGRANGE_NO_MIMIC` before the call). The SDK's `Joint` gets
`stiffness` and `springReference`, read without a guard because zero is the right default.

ABI MINOR: #199 appended `damping … mimic_offset` without raising `LAGRANGE_PLUGIN_ABI_MINOR`.
This RFC proposes the same (open question 2).

### 7. Protocol and viewer

- **Inspector (E6).** `Server.cpp`'s `schemaAttributes(prim, item)` lists a prim's authored
  schema attributes with their `lagrangeUnit`. So `joint:stiffness` and `joint:springReference`
  appear and are editable on Apply through `SetAttribute`, with no proto change. The server
  checks the value as the loader would (≥ 0), rebuilds the run and carries its state over.
  TODO(verify) in step 5 that `SetAttribute`'s loader check accepts `joint:` attributes on a
  joint prim, as it does `elec:`.
- **Builder.** The joint's Dynamics section (`viewer/panels/src/BuilderProperties.cpp`,
  "damping · friction") gets `stiffness` (N·m/rad or N/m by joint type) and `spring reference`
  (rad or m) rows. `viewer/builder/src/Urdf.cpp` writes and reads the `<gazebo reference>` form of
  §3, and `test_builder_roundtrip.py` covers it. The builder's `Validation.cpp` warns when the
  explicit cap of §5 would apply at the run's step. The design of the rows follows
  `docs/design/ui/` (binding).
- **No new overlay.** A later option could draw a spring glyph on sprung joints, in its own UI
  spec.

### 8. ROS 2, ros2_control and joint control

- The spring is passive and lives inside physics, like damping. `/joint_states` effort and
  `LagrangeSystem`'s `effort` state interface stay what they are today (the commanded effort,
  `slot.effort`), so they do not include the spring, just as they do not include damping. This
  is documented.
- `server::JointControl` (`--joint-control`): its position loop
  `τ = I·(ω²(q* − q) − 2ω q̇)` would settle off target by `k (q* − q₀) / (I ω² + k)`. Proposed: the
  loop adds the spring's torque `k (q − q₀)` as feed-forward, so a commanded position is reached
  as on a joint without a spring. The ideal servo stays ideal (open question 3). Effort and
  voltage modes get no compensation: the spring acts on them as on a real joint.
- `lagrange check` (`server/src/RobotCheck.cpp`, `jointLoads`) adds the spring torque at the
  starting pose to what each joint must hold against gravity. A spring can make a pose easier or
  harder to hold.
- `simulation_interfaces` `get_entity_state` and the rest: no change.

### 9. Determinism and tiers

Both backends compute the spring on the CPU from doubles, in a fixed joint order. The CPU
reference stays bit-reproducible (invariant 7), and `lagrange determinism` covers it once a
sample has a spring. The behaviour is the same on Tier 0, 1 and 2: the MuJoCo and Jolt backends
run on the CPU on every tier. A future batched GPU backend must pass the same conformance cases.

## Validation

New `TEST_P(Conformance, …)` cases in `server/test/physics_conformance.cpp`, run for every
backend by `lagrange_physics_conformance` (`mujoco`, `jolt`, `jolt_abi@lagrange_jolt`). They are
added to the contract table of `docs/design/domains/rigid_body.md` §8, so a third-party backend
must pass them too.

| Case | Setup | Expected (closed form) | Tolerance |
|---|---|---|---|
| Torsional pendulum | Hinge about the vertical axis (gravity along it, so no gravity torque), bob of known `I` about the axis, `k` chosen for a 0.5 s period, released from 0.2 rad, `q₀ = 0`, 10 s | `T = 2π √(I / k)`, armature included in `I` | Period within 0.25 % (MuJoCo, Jolt); amplitude kept ≥ 99 % (MuJoCo), ≥ 80 % (Jolt, as the pendulum's energy contract of §8.1) |
| Damped torsional pendulum | Same, `d` for `ζ = 0.1` | `ω_d = ω₀ √(1 − ζ²)`, logarithmic decrement `δ = 2πζ / √(1 − ζ²)` | Period 0.5 %, decrement 2 % |
| Spring reference | Same, `q₀ = 0.7` rad, released from rest at 0, `d` for `ζ = 1` | Settles at 0.7 rad | Within 1e-4 rad after 5 s |
| Continuous hinge wound up | Continuous hinge, `q₀ = 6π`, released at 0 | Swings about 6π, not about 0 (turns are counted) | Mean of the swing within 1 % of 6π |
| Slide spring-mass, vertical | Prismatic along gravity, mass `m`, `k`, `q₀ = 0`, from rest at 0 | Oscillates about `q₀ − m g / k` with `T = 2π √(m / k)` | Equilibrium 0.5 %, period 0.25 % |
| Preload at a stop | Hinge with `q₀` 0.5 rad past its upper limit | Rests at the limit, `joint/<j>/reaction` torque about the axis = `k (q₀ − q_upper)` | 1 % |
| Zero is the old behaviour | Every existing conformance case | Unchanged | The existing ones, run as they are |
| Determinism | The torsional pendulum twice, and saved and restored mid-way | Identical state hash | Bit-exact (CPU) |

Unit tests in step 2 and 3: the SDFormat and MJCF importers read the values (an MJCF joint with
`ref ≠ 0` and `springref`), the URDF `<gazebo reference>` form, the USD round trip through
`StageWriter`/`StageReader`, a negative stiffness refused, polynomial stiffness reported as
dropped. `test_abi_layout.c` pins the layout. A C plugin in `server/test/plugins/` reads the
values back through `world_joint`.

## Compatibility under RFC 0007

| Surface | Change | Allowed because |
|---|---|---|
| Schemas | two attributes appended to `JointDynamicsAPI_1`, fallback 0 | addition; fallback keeps today's behaviour |
| Migrations | none: an older file without the attributes reads as stiffness 0, and `migrate_schema --check` passes unchanged files. No `tools/migrate/lagrange_migrate/` function: those exist per version step (`_1 → _2`), and this stays `_1`. An entry in `schemas/CHANGELOG.md` ("Additions, nothing to migrate", as RFC 0020's `NozzleAPI_1`) | addition |
| Plugin ABI | two `double`s appended to `lagrange_joint_info` (152 → 168 bytes), pinned in `test_abi_layout.c` | appending; zero is "no spring", so both mixes of old and new run |
| Protocol | none | |
| Profiles | none | |
| Recordings | none (or one signal per sprung joint, open question 4) | addition |
| Behaviour of existing runs | none: no existing sample, URDF or USD asset has a spring. Imported SDFormat/MJCF robots that had springs now get them | this is a fix of dropped data, listed in the release notes |

## Steps (each a PR)

1. This RFC.
2. Schema attributes, `World::Joints` columns, USD read and write, the schema test, the
   `schemas/CHANGELOG.md` entry. *Gated:
   `schemas/`.*
3. Importers: SDFormat, MJCF (with the `ref` shift and drop notes), URDF `<gazebo reference>`;
   `test_importer_audit` entries.
4. MuJoCo and Jolt backends, the Jolt cap, the conformance cases, `rigid_body.md` §8 table.
5. Plugin ABI append, SDK fields, `test_abi_layout.c`, a C test plugin. *Gated:
   `core/plugin_abi/`.*
6. JointControl feed-forward, `lagrange check`, the inspector check, the builder rows (with
   screenshots), docs.

## Docs to update

- `docs/physics/rigid-body.md`: "Joint damping and dry friction" becomes "Joint springs, damping
  and dry friction", with the equation of the Summary and the natural frequency `√(k/I)`. It
  states that MuJoCo couples a mimic follower's spring to the leader and Jolt does not, and gives
  the Jolt stiffness cap and `substeps`.
- `docs/design/formats.md`: a `schemas/joint/schema.usda` entry (there is none yet; §3.17), and
  §9 importer rules for the three formats.
- `docs/design/domains/rigid_body.md` §8: the new conformance rows.
- `docs/guide/cli.md` "Importing a robot": the URDF `<gazebo reference>` form.
- CLAUDE.md §7 "Joint dynamics" line: `stiffness`, `springReference`.
- The release notes of the version that ships it: "SDFormat and MJCF springs are now imported".

## Alternatives considered

- **A `JointSpringAPI_1` schema of its own.** It is cleaner to toggle, but the spring is one more
  term of the same passive joint dynamics, and every format puts it next to damping. D55 and D56
  set the precedent of appending to `JointDynamicsAPI_1`.
- **A plugin that writes `load_torque`.** It works only on actuated joints, the importers would
  still drop the data, and every user would rewrite the same three lines.
- **Jolt's motor spring (§5 option 2).** It is implicit and stable, but it competes with mimic
  and position modes for the motor, wraps at ±π and clamps to the limits. It stays the fallback
  if the explicit cap turns out to bite real models (open question 1).
- **Polynomial stiffness now.** No format but MJCF has it. It can be appended later as
  `joint:stiffnessPoly` if a user needs it.

## Open questions (with a proposed answer)

1. *Jolt: an explicit torque with a stiffness cap, or the motor's implicit spring?* Explicit, with
   `kMaxSpringShare = 1` and a warning when capped (§5). Revisit if an imported model is capped
   at 1 ms.
2. *Raise `LAGRANGE_PLUGIN_ABI_MINOR` to 2 for the append?* No: follow #199, which appended to
   the same struct within 1.1. Zero means "no spring", so a plugin need not know whether the host
   fills it. The header comment says "Appended (RFC 0023)".
3. *Does `--joint-control`'s position loop compensate the spring?* Yes, by feed-forward `k (q − q₀)`,
   so it stays an ideal servo. Effort and voltage modes do not.
4. *Publish the spring torque as a signal (`joint/<j>/spring_torque`, N·m or N) for scopes and
   recordings?* Not in this RFC. `joint/<j>/position` and the two constants give it, and a
   signal per sprung joint can be appended later.
5. *Allow a spring on a mimic follower?* Yes, with a note in the load report and the documented
   engine difference (§4, §5).
6. *Refuse or clamp a negative stiffness?* Refuse at load with the joint's name (negative
   stiffness is an unstable model, not a typo to fix silently).
7. *Pre-approve the gated merges of steps 2 and 5 once each passes its tests?* For the owner. Both
   need 2 approvals under CLAUDE.md §9.

## References checked for this draft

- The repository at origin/dev `021b0c20`: `schemas/joint/schema.usda`,
  `core/world/include/lagrange/world/World.hpp` (`World::Joints`), `core/world/src/World.cpp`,
  `io/importers/src/SdfImporter.cpp`, `MjcfImporter.cpp`, `Description.{hpp,cpp}`,
  `io/stage/src/StageReader.cpp`, `StageWriter.cpp`, `server/src/Robot.cpp` (`<gazebo reference>`),
  `server/src/Server.cpp` (`schemaAttributes`, joint attributes), `server/src/JointControl.cpp`,
  `modules/rigid_body/mujoco_backend/src/MujocoBackend.cpp`,
  `modules/rigid_body/jolt_backend/src/JoltBackend.cpp` (`kMaxDampingShare`, `kMimicMotorHz`,
  the torque path), `core/plugin_abi/include/lagrange_plugin.h`, `core/plugin_abi/src/PluginLibrary.cpp`
  (`fill`, `worldJoint`), `core/plugin_abi/include/lagrange/plugin_sdk/World.hpp`,
  `core/plugin_abi/test/test_abi_layout.c`, `server/test/physics_conformance.cpp`,
  `schemas/test/test_schemas.py`, `docs/physics/rigid-body.md`, `docs/plan/owner-decisions-2026-11-15.md`
  (D55, D56), RFC 0019 (series elastic, contact time constant), RFC 0020.
- Installed headers and specs on Ubuntu 26.04 / Lyrical:
  `/opt/ros/lyrical/opt/mujoco_vendor/include/mujoco/mjspec.h` (MuJoCo 3.12.0,
  `mjVERSION_HEADER 3012000`; `mjsJoint::stiffness[mjNPOLY+1]`, `springref`, `springdamper`),
  `mjmodel.h` (`jnt_stiffness`, `jnt_stiffnesspoly`, `qpos_spring`, `mjNPOLY 2`);
  `/usr/include/gz/sdformat16/sdf/JointAxis.hh` (`SpringStiffness`, `SpringReference`);
  `/opt/ros/lyrical/opt/sdformat_vendor/share/sdformat/1.12/joint.sdf`
  (`spring_reference`, `spring_stiffness`, default 0); `libsdformat.so` strings
  (`springStiffness`, `springReference`, from `parser_urdf`); Jolt 5.6.0
  `Jolt/Physics/Constraints/{HingeConstraint,SliderConstraint,MotorSettings,SpringSettings}.h`.
- Still unread, marked TODO(verify): sdformat `parser_urdf.cc` source, MuJoCo's Computation
  chapter on which passive forces the implicit integrators treat implicitly, gz-physics spring
  support per engine.
