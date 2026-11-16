# Owner decisions, 2026-11-15

Everything that waits on the owner after the 2026-11-14/15 merges, in one place. Each row is one
decision, with a proposed answer, so a reply can be "as proposed except D12, D39". Line numbers
are as of `origin/dev` at `bc081b4` (#399).

## What merged

64 PRs went into `dev` after the 1.3.0 release (`155bc43`), numbered #332 to #399. They cover:

- **The robot builder.** RFC 0016 phase 1, Addendum A (meshes and collision) and phase 3 (power
  system and overlay sensors). This includes the proto additions `ImportRobot.content`,
  `overlay_content`, `RemoveRobot` (33) and `CommandJoint` (34), already accepted with RFC 0016.
- **Viewer workspaces.** ROS 2, Performance, Replay compare, Domains, Signals, Scene, World,
  Electrical and Environment tabs. Each was built on what protocol v1 carries today and lists
  what it lacks.
- **Draft RFCs and plans.** Three draft RFCs: 0017 (MuJoCo 4), 0018 (radiation) and 0019
  (legged). The legged-contact spike (RFC 0019 step 1). The builder phase 4 plan with a draft
  Addendum B.
- **Fixes and performance.** Two QA rounds, rigid-body performance and fixes (MuJoCo static
  scenery, Jolt substeps, `noslip_iterations`, MJCF armature), and Python `ParallelEnvs` over
  shared memory.

The full list is in [CHANGELOG.md, 1.4.0](../../CHANGELOG.md#140-2026-11-15), which covers #332
to #423. **Update 2026-11-17:** v1.4.0 was released on 2026-11-15 (#425), so section F is done
except where marked; sections A to E still wait. RFCs 0020 to 0022 came after: section G.

Count: **77 decisions**. 63 are decisions (D1–D63), 9 are actions only the owner can take (E1–E9), and 5 are
about the release (F1–F5). Rows that tie to another row say so; the shared item is counted once.

## A. RFCs to accept or reject

### A1. RFC 0017, MuJoCo 4.x ([rfcs/0017-mujoco-4.md](../../rfcs/0017-mujoco-4.md), spike [mujoco-4.md](spikes/mujoco-4.md))

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D1 | Accept RFC 0017 | accept / reject / stay on 3.12 | Accept. The switch merges only after the four gates (water heave test, full check, comparison rerun, kinematic IMU) | Builds with no source change. Conformance 39/39, determinism 101/101, box pile 1.31× faster inside Lagrange | Faster contact scenes; same MuJoCo as mujoco_ros2_control | L5, L34–41 |
| D2 | Q1: a date after which Lagrange ships its own MuJoCo vendor package | no date / a date / own vendor now / Jolt as default | No date. Wait for `mujoco_vendor` 4.0.1+ and comment on its #18 (E1) | #18 pins 4.0.0, which has the sparse Cholesky bug. An own vendor package is a maintenance cost | Shipping 4.x at all | L64–65 |
| D3 | Q2: pin the MuJoCo version in conformance | pin / log + header check | Log `mj_versionString()` at start; a test checks `mjVERSION_HEADER == mj_version()` | Catches a mismatch between header and library without freezing the version | Nothing urgent | L66–68 |

### A2. RFC 0018, the radiation domain ([rfcs/0018-radiation.md](../../rfcs/0018-radiation.md))

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D4 | Accept RFC 0018 | accept / reject / defer | Accept. Steps 2–8 follow in order | Radiation is the only CLAUDE.md §1 domain with no module. Everything is added under RFC 0007 | Schemas, data tables, `lagrange_radiation`, effects, dose maps, viewer, tutorial | L5, L345–362 |
| D5 | Q1: one `RadiationAPI_1` with a role, or three schemas | one / three | Three: `RadiationSourceAPI_1` (multi-apply), `RadiationMaterialAPI_1`, `RadiationSusceptibilityAPI_1`. They add the unit Sv and two QuantityKinds (2 approvals) | The formats.md §3.6 sketch never shipped, so nothing migrates | Step 2 schemas | L366–368 |
| D6 | Q2: the medium a victim's dose is in | silicon / air / water / tissue | Silicon by default. Air for air kerma; water or tissue on request; a dosimeter uses its own quantity | Electronics are what fails | Victim signals | L369–371 |
| D7 | Q3: dose to people | effective dose / H*(10); ICRP 74 or 116 | H*(10) from ICRP 74. No table ships until its reuse terms are checked (E4); until then the user supplies `radiation.h10_per_air_kerma` | Both ICRP sets are copyrighted | `rad/human/*` | L372–378 |
| D8 | Q4: build-up through several materials | most mean free paths / Broder multilayer | Use the material with the most mean free paths, documented as approximate. Broder later if the Monte Carlo check shows the need | Simple, and the error can be bounded later | Point kernel | L379–382 |
| D9 | Q5: source of the G-P build-up coefficients | ANSI/ANS-6.4.3 / RSICC DLC-129 / user file | Default `buildup: none`; `radiation.buildup_table` takes the user's own copy; the owner asks RSICC and ANS (E2) | The standard is sold and its reproduction is restricted | Build-up by default | L383–387 |
| D10 | Q6: interpolating μ/ρ across absorption edges | log-log linear keeping edge rows / other | Log-log linear, keeping the duplicate edge rows | No interpolation then crosses an edge (lead's K edge at 88 keV) | Attenuation tables | L388–390 |
| D11 | Q7: prims with no radiation material | shield as air / refuse | They do not shield. The start logs how many colliders lack a material, and `lagrange check` warns | Avoids silent wrong defaults without refusing scenes | Loader | L391–393 |
| D12 | Q8: hollow objects (a lead pot, a cask) | convex parts / thin-shell attribute | Convex parts, with a docs recipe | No new attribute | Docs | L394–397 |
| D13 | Q9: radioactive decay during a run | off / `halfLife` | Off. An optional `rad:source:<n>:halfLife` may come later | Runs are short next to half-lives | — | L398–399 |
| D14 | Q10: step size | 100 ms / other | 100 ms (`radiation.dt`), with a warning when a victim's dose rate changes by more than 20 % in one step | Matches thermal-scale dynamics | Scheduler entry | L400–402 |
| D15 | Q11: spectrometers, directional detectors, gamma cameras | now / later RFC | A later RFC | Keeps this RFC bounded | — | L403–405 |
| D16 | Q12: a published benchmark | SINBAD / closed forms + Geant4 | Request SINBAD (E3). Until then, closed forms plus the Geant4 comparison (tests 1–9) | A realism claim needs a reference (CLAUDE.md §10) | Validation | L406–408 |
| D17 | Q13: a segment query in the plugin ABI | in this RFC / own RFC | Not in this RFC. A `scene_segments` call would be a 1.x MINOR in its own RFC | The ABI is frozen by RFC 0007 | — | L409–411 |
| D18 | Q14: neutron point kernels | yes / no | No. Neutrons come only as maps and as particle flux | Point kernels are poor for neutrons | — | L412–414 |

### A3. RFC 0019, legged robots ([rfcs/0019-legged-robots.md](../../rfcs/0019-legged-robots.md), spike [legged-contact.md](spikes/legged-contact.md))

Spike, in brief:

- **Weight at rest.** Contact forces carry the weight to within 0.016 % on both engines.
- **Static leg torques.** MuJoCo within 0.006 %, Jolt within 0.96 %.
- **Slopes.** MuJoCo creeps 5.7 mm/s at 10° on a legged robot. `noslip` 20 stops it at 10° and
  costs 30–60 % more per step.
- **Rimless wheel.** MuJoCo within +1 % of the closed form, Jolt within +3.4 % and chatters.
- **Gains.** kp 100 stands at every step size; 250 diverges at 2 ms.
- **RL speed.** 2,659 policy steps/s in one process, 24,379 on 12.
- **Determinism.** 20/20 hash checks equal.

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D19 | Accept RFC 0019 | accept / reject / defer | Accept. Step 1 (the spike) is done | Legged robots have no controller (evaluation item 6); post-1.0 "other robot classes" | Steps 2–10, the live Contacts overlay | L5, L379–398 |
| D20 | Q1: a legged controller in the server | yes / examples only | Examples only. The first-run sample stands in impedance mode | Controllers are users' work. Revisit if users ask | — | L402–404 |
| D21 | Q2: backend for legs | MuJoCo / Jolt | MuJoCo is the reference and default; Jolt is supported and measured | The spike agrees: MuJoCo is 1.5× faster, and Jolt chatters on impacts | Defaults and tests | L405–406 |
| D22 | Q3: per-contact forces on Jolt | depth-weighted share / ContactListener | The per-body sum is exact; the per-point share goes by penetration depth, documented as approximate; warn once | Jolt does not expose manifold impulses | `foot_contact` on Jolt | L407–410 |
| D23 | Q4: MJCF contact settings into USD | yes via `ContactAPI_1` / no | Yes (`solref`/`solimp`). `priority`, `margin` and `gap` are noted and dropped | The spike says they must reach MuJoCo | `ContactAPI_1` | L411–412 |
| D24 | Q5: impedance commands over plain ROS | `/joint_commands` / ros2_control only | Not on `/joint_commands`. ROS goes through ros2_control; Python and bus signals carry the rest | No Lagrange message package | ROS path | L413–415 |
| D25 | Q6: what `/odom` means for a legged robot | ground truth / leg odometry | Ground truth, named so in the guide | No built-in estimator | Docs | L416–417 |
| D26 | Q7: load-dependent gearbox friction | transmission / electrical drive | In the transmission. `elec:gearEfficiency` stays the drive's loss; use one or the other | No double counting | `JointTransmissionAPI_1` | L418–420 |
| D27 | Q8: continuous torque ratings for `lagrange check` | new attribute / thermal model | Derived from the motor's thermal model | No new attribute | `lagrange check` | L421–423 |
| D28 | Q9: closed loops (Cassie, parallel ankles) | now / later RFC | A later RFC. Cassie stays "not loadable" | Joint loops are refused today (#366) | — | L424–425 |
| D29 | Q10: a support-polygon overlay kind | `KIND_PATH` / new `KIND_POLYGON` | A closed `KIND_PATH` now. Append `KIND_POLYGON` only if the viewer spec needs a fill | No proto change | Overlays | L426–428 |
| D30 | Q11: plugin ABI | getter now / own RFC | Nothing in this RFC. `contact/points` is a bus signal | The ABI is frozen | — | L429–430 |
| D31 | Q12: a walking dataset with foot forces | ask a lab / closed forms + HyQ | Owner decides whether to ask a lab (E6). Until then: closed forms, the standalone MuJoCo match and HyQ | Realism claim needs a reference | Validation depth | L431–433 |

### A4. RFC 0016 Addendum B and builder phase 4 ([builder-phase4-plan.md](builder-phase4-plan.md) §11–§12)

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D32 | Accept Addendum B (open USD, SDFormat and MJCF as URDF + overlay through a converter; save USD through `lagrange_import` with the source embedded) | accept / reject | Accept, with Q2–Q11 as proposed (Q1, joint limits, is D56 in C), then paste it after Addendum A in RFC 0016 | It keeps URDF + overlay as the builder's one format | All of phase 4 | §12 L552–595 |
| D33 | Q2: embed the builder's URDF and overlay in exported USD `customLayerData` | yes / no | Yes for builder exports; `lagrange_import --embed-source` for anyone | A file then returns to the builder exactly as it was saved | WP4-D | L529 |
| D34 | Q3: save over a `.usda` only when the builder or `lagrange_import` wrote it and it has no composition arcs | yes / always / never | Yes; anything else is Save as… | Never clobber hand-authored USD | WP4-F | L531 |
| D35 | Q4: composed stages | flatten into a new document / edit the base layer and keep the `over`s | Flatten into a new document | Editing the base layer needs a check that each `over` still finds its prim | WP4-F | L533 |
| D36 | Q5: where the converter lives | new `lagrange_builder_convert` / `lagrange export urdf` | `lagrange_builder_convert`, linking the builder library | One URDF writer | WP4-0 (lands first) | L535 |
| D37 | Q6: SDFormat meshes through `lagrange_meshes` in `importSdf` | yes (own PR) / no | Yes, as its own PR. **Changes what `lagrange_server` loads**: meshes collide and draw instead of being skipped or drawn magenta | Fixes a real gap | WP4-E | L538 |
| D38 | Q7: SDFormat `navsat`, `magnetometer` and `air_pressure` read as GNSS, magnetometer and barometer | yes (own PR) / no | Yes, as its own PR. **Changes runs** of SDFormat robots that have them (today they are custom kinds that do nothing) | Users expect them to work | WP4-E | L541 |
| D39 | Q8: save as SDFormat | now / later / never | Later, on request | No demand yet | Optional WP4-H | L544 |
| D40 | Q9: inline meshes | write OBJ beside the document / refuse to save | Write OBJ files beside the document | Saving never fails | Phase 4 | L545 |
| D41 | Q10: radar in an overlay | `applyOverlay` makes radars (own PR) / USD only | `applyOverlay` makes radars, own PR | #372 already made an overlay's radar a sensor of the run | Phase 4 | L547 |
| D42 | Q11: worlds with several robots | refuse / open the first and note the rest | Open the first and note the rest | Partial use beats refusal | Phase 4 | L549 |

### A5. Earlier acceptance to confirm

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D43 | Review RFC 0007 (v1.0 freeze), accepted under the 2026-10-29 pre-approval | confirm / amend | Confirm | Every row in B and C relies on its rule that changes come only by addition | B, C | [progress.md L603](progress.md), [rfcs/0007](../../rfcs/0007-v1-freeze.md) L5 |

## B. Protocol additions (1.x by addition, RFC 0007; CLAUDE.md §9: 2 approvals)

Every number below was checked against `proto/lagrange/viewer/v1/*.proto` at `bc081b4`. No
message uses `reserved`. "Free" means no field uses that number today.

Where the PR named the number, it is "proposed". Where only the field was proposed, the number
is the next free one and is marked "(next free)".

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D44 | ROS 2 graph: services, parameters, TF frames and actions in `RosGraph` (`session.proto`) | add / keep terminal hint | Add as `RosGraph` 10–13. **Free**: `RosGraph` ends at `note = 9` | The Services and Params tabs show an empty state; Actions and TF are inferred from topics | ROS 2 workspace tabs (#377) | `session.proto` L115–154; `Ros2Panels.cpp` L383 |
| D45 | Performance: `ServerStats.gpu_passes`, a determinism check and the server's threads | add / leave "not reported" | Add as `ServerStats` 11–13. **Free**: it ends at `estimated = 10` | Two KPI cards read "not reported" | Performance KPIs and Threads (#378) | `session.proto` L195–204; `PerformancePanels.cpp` L191, L226, L747 |
| D46 | `ServerStats.Module.gpu_ms_per_step` | add / no | Add as 8 (next free). **Free**: `Module` ends at `on_gpu = 7`. Needs HAL timestamp queries on the server first | `cpu_ms_per_step` of a GPU module includes its wait | Per-module GPU time | [protocol.md](../design/protocol.md) L665 |
| D47 | Replay compare of a remote replay: `OpenRecording.compare_path = 2`, `RecordingInfo.compare = 11`, `SignalInfo.compare = 11` (bool, `compare/` prefix) | add / local files only | Add. **All free**: `OpenRecording` has only `path = 1`; `RecordingInfo` ends at 10; `SignalInfo` ends at 10 | Comparing a replay from another machine needs the server to read the second file | Remote Compare (#380) | [viewer.md](../design/viewer.md) L439–444 |
| D48 | `Command.resimulate = 35`, `Resimulate { SimTime from = 1; string profile_yaml = 2; }` | add / no | Add. **Free**: the `Command` oneof ends at `command_joint = 34`. The state comes from recorded root poses and joints, as spawning does | The "Re-simulate from here…" button is drawn disabled | Replay Source strip | viewer.md L445–450; `ReplayCompare.cpp` L1433 |
| D49 | Scene layers: `LayerInfo` + `SceneSnapshot.layers`; commands `SetLayerMuted`, `SetEditTarget` | add / read-only tab | Add. `SceneSnapshot.layers = 12` (next free; it ends at `colliders = 11`); the commands at `Command` 25 and 26 (next free; 25–29 are unused in the scene-edit block) | The Layers tab can't mute layers or set the edit target | Scene Layers tab (#386) | `SceneTabsPanels.cpp` L163 |
| D50 | Undo / Redo: commands `Undo`, `Redo`, the undo and redo depth in `ServerStatus`, `CommandResult.can_undo` | add / no undo | Add. `Command` 27 and 28 (next free); `ServerStatus` 11 and 12 (next free; it ends at `recording_path = 10`); `CommandResult.can_undo = 4` (next free; it has 1–3 and the oneof 10–12) | The History tab lists edits but cannot undo them | Scene History | #386 |
| D51 | World: `GeoOrigin.source`, `Agent.z`, `Agent.region`, `Region` origin, server and recenters, `WorldState.geo_sources` | add / no | Add. All next free: `GeoOrigin.source = 5`; `Agent.z = 10` and `region = 11`; `Region` `origin_x = 6`, `origin_y = 7`, `server = 8`, `recenters = 9`; `WorldState.geo_sources = 13`. (`FloatingOrigin.recenters = 7` already exists for the run's one origin; this is per region) | Agents are 2D today; regions carry no origin, so RegionHost runs can't be drawn | World Regions and Geodesy (#388) | `session.proto` L317–372 |
| D52 | Environment: `Weather.drop_size_model`, `Wind.gust_model`, `Field.source`, `gpu_bytes`, `voxel_m`, a `FieldProbe` message | add / no | Add. All next free: `Weather` 5; `Wind` 5; `Field` 8, 9 and 10; `FieldProbe` as `Envelope` 48 (ends at `field_volume = 47`) with a `Command` 56 request (ends at `set_camera_interest = 55`) | Field info shows what the server does not say; the probe tooltip samples slices only | Environment Field info, probe (#390) | `session.proto` L228–269 |
| D53 | An Export Modelica command | add / CLI only | Defer. `lagrange export modelica` covers it | Low value next to D44–D52 | Electrical "Export Modelica" (disabled) | `ElectricalPanels.cpp` L677 |
| D54 | How to land D44–D52 | one proto PR assigning every number / one per feature | **One PR** with every number above, `buf breaking` against `dev`, the §9 note and 2 approvals. Server fills in later PRs | D48–D52 all reach for the next free `Command` numbers; parallel PRs would collide | All of B | `proto/` |

## C. Schema additions (`schemas/`, RFC 0007 by addition; 2 approvals)

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D55 | `double joint:armature` in `JointDynamicsAPI_1` | add to `_1` (additive) / `JointDynamicsAPI_2` / none | Add to `_1`, default 0 | MJCF armature is imported now (#394) but is lost through `lagrange_import`. A USD legged robot gets armature only from `elec:rotorInertia·N²` | Legged USD robots, RFC 0019 | [legged-contact.md](spikes/legged-contact.md) L475–480; `schemas/joint/schema.usda` |
| D56 | `joint:velocityLimit`, `joint:effortLimit` (0 = not given) in `JointDynamicsAPI_1` (phase 4 Q1) | (a) add / (b) no limit + Warning / (c) user retypes | (a), in the **same PR as D55**; no migration | USD carries no limits today, so a round trip loses them | Builder WP4-A, Addendum B | builder-phase4-plan.md L522 |
| D57 | RFC 0019 schemas `ContactAPI_1`, `JointTransmissionAPI_1` | with D19 / later | Review with D19, merge at its step | Backlash, series elastic, load friction; `solref`/`solimp` | RFC 0019 steps | rfcs/0019 L228, L388 |
| D58 | Ratify the `profiles/schema.json` keys merged today: `rigid_body.substeps` (1–64) and `noslip_iterations` (0–100) | ratify / revert | Ratify. Both are additive, and the defaults are bit-identical | Profiles change only by addition; these are additions | — | #353, #394 |

The radiation schemas are decided under D5.

## D. UI and design reviews ([docs/design/ui/](../design/ui/) is binding)

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| D59 | Where the builder's Power and Motor sections sit | as built (Power under the Robot inspector after Parts; Motor under Joint) / elsewhere | As built | Power belongs to the robot; a motor belongs to its joint | 12-builder SPEC sign-off | [SPEC.md](../design/ui/12-builder/SPEC.md) L149–166 |
| D60 | `JointSlider` and `ShapeTile` into `00-design-system` | accept / change | Accept as specified (slider: 4 px `bg4` track, `#3A4F6E` fill, 14 px `t1` knob, dashed when continuous; tile: 56×54 `bg2`, 1 px `ln2`) | Already built and used | Design-system completeness | SPEC.md L211–219 |
| D61 | Launcher action tiles: 2×2 with "Build a robot" (`kBuildTile = true`) | 2×2 / the design's row of three (`false`; Build stays in File and Ctrl B) | 2×2, and update `01-launcher` SPEC to four tiles | Four in one row cut their titles at the design's width. RFC 0016 Q6 accepted the tile, but the layout was never reviewed | 01-launcher SPEC | `Launcher.cpp` L651–692; [01-launcher/SPEC.md](../design/ui/01-launcher/SPEC.md) L13 |
| D62 | Radar radial-velocity colour | keep the `t3` (`#8190A6`) middle / the error map's `#16243A` middle / a new radar token | A new `colormap.velocity` token, blue to `t3` to orange | `#16243A` is `bg3`, so a target at rest vanishes on the viewport. The error map is not a radar token | Radar overlay and view (#360) | `RadarScale.hpp` L19–24; [design-tokens.json](../design/ui/shared/design-tokens.json) L54 |
| D63 | The 12-builder SPEC and README still say "RFC 0016 Draft, waits for review" | mark accepted after D59–D60 / leave | Mark accepted once D59–D60 are answered | Stale since 2026-11-14 | — | SPEC.md L3; README.md L3, L6 |

## E. External actions only the owner can take

| # | Action | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| E1 | Comment on `pal-robotics/mujoco_vendor#18` asking for 4.0.1+ instead of 4.0.0 | do / skip | Do | 4.0.0 has the sparse Cholesky bug | D1–D2 | rfcs/0017 L64 |
| E2 | Ask RSICC (DLC-129) and the ANS about redistributing the ANSI/ANS-6.4.3 G-P coefficients | ask / don't | Ask | Build-up stays off by default without them | D9 | rfcs/0018 L174–176, L383–387 |
| E3 | Request SINBAD access through the NEA Data Bank | request / don't | Request | Its distribution is restricted and given on request | D16 | rfcs/0018 L336–339 |
| E4 | Check reuse terms of ICRP Publications 74 and 116 | check / don't | Check | No conversion table ships until then | D7 | rfcs/0018 L372–378 |
| E5 | Check NIST SRD 126 terms for redistributing its tables in an Apache-2.0 repo; check Geant4 licence compatibility | check / don't | Check | Attenuation data in the repo; Geant4 is offline only | D4 | rfcs/0018 L165–166, L210 |
| E6 | Ask a lab (ANYbotics, Istituto Italiano di Tecnologia, MIT) for a walking dataset with foot forces | ask / don't | Owner's call. HyQ (CC BY 4.0) suffices for now | Validation depth | D31 | rfcs/0019 L431–433 |
| E7 | Verify the licences of legged sample robots: Spot (provenance and trademark), Mini Cheetah description, an open hexapod, HyQ description, arXiv 2311.04320 data | verify / pick others | Verify before step 2 | Samples ship or are fetched | D19 samples | rfcs/0019 L108–113, L364–366 |
| E8 | Run Geant4 offline on the `hot_cell` sample for RFC 0018 test 9 | run / delegate | A maintainer runs it when step 4 lands | Needs Geant4, which does not ship | D16 | rfcs/0018 L329–335 |
| E9 | The second approval on the B and C PRs (CLAUDE.md §9: 2 approvals) | owner + one / owner-delegated as before | Owner's call | Governance | B, C | `GOVERNANCE.md` |

## F. Release: should `dev` become v1.4.0? (done: v1.4.0 released 2026-11-15, #425)

| # | Decision | Options | Proposed | Why | Blocks what | Where |
|---|---|---|---|---|---|---|
| F1 | **Done** (release 1.4.0, #425, 2026-11-15). Release `dev` as v1.4.0 | now / after B and C land / wait | **Yes, now**, as a 1.x minor without the drafts. B and C go into 1.5 | 64 PRs, a whole new workspace (the builder), and behaviour changes users should get labelled | — | [CHANGELOG](../../CHANGELOG.md#unreleased) |
| F2 | **Done**: the changelog (1.4.0 lists #332–#423), a full local check at #423 (2506 tests, 0 failures after #424), and the debs archive (51 debs, clean-machine install test passed, uploaded to the v1.4.0 release). Before tagging: the changelog for #382–#399, a full local check on the `dev` head (per-PR checks passed, e.g. #394: 0 failures, but not on `bc081b4` itself), the debs archive | do / skip | Do all three | Unreleased stops at #381; each release needs its debs uploaded | F1 | — |
| F3 | **Done** (CLAUDE.md §2 now says v1.4.0 is released). CLAUDE.md §2 still said v1.2.0 is the current release; 1.3.0 shipped 2026-11-14 | update with F1 / now | Update in the release PR | Stale phase text | — | CLAUDE.md §2 |
| F4 | **Done** (1.4.0 ships a "Breaking or behaviour changes" section as a minor). Is "Breaking or behaviour changes" acceptable in a minor under RFC 0007? These are: `valid` topics now `Bool`, out-of-range parameters refuse, faults on Bool/Int refuse, joint loops refused, one ground, new state hashes | minor / hold for a deprecation | Minor. These are fixes of wrong behaviour, and schemas, ABI and protocol change only by addition | — | F1 | CHANGELOG Unreleased |
| F5 | **Done** (1.4.0 has only the opt-in `LAGRANGE_MUJOCO_PACKAGE` option, #364). MuJoCo 4 in 1.4.0 | in / out | Out. The vendor package isn't ready (D2) | — | — | rfcs/0017 L45–50 |

### What v1.4.0 contained

- **Robot builder.** RFC 0016 phase 1, Addendum A and phase 3:
  - shapes, meshes, joints and catalog sensors
  - gizmos
  - per-link collision choice with CoACD
  - power system and overlay sensors
  - loads, converters, fuses, switches and diodes
  - Test in its own server or in the connected run
  - URDF + overlay files
  - (#340–#368)
- **Protocol (by addition).** `ImportRobot.content` and `overlay_content`, `RemoveRobot`,
  `CommandJoint` (#354, #357).
- **Viewer.**
  - ROS 2 Services, Actions, Params and TF tabs
  - Performance KPIs, Threads, GPU passes and Memory
  - Replay Compare, Sim vs real and Validation (`--compare`, `--suite`)
  - Domains Modules, Coupling, Diff and Schema
  - Signals Saved sets, Measure and FFT
  - Scene Layers and History, sensor views beside the viewport
  - World Regions and Geodesy
  - Electrical Probes and History; Environment fields in the viewport and Field info
  - the launcher as designed, the command palette, radar detections in 3D, text at the design's
    size
  - (#360, #374–#390, #399)
- **Server.**
  - `ServerStats` GPU modules and memory
  - `/spawn_entity` names
  - a zero aerodynamics substitute
  - EMI sources carry their prim
  - catalog sensor needs checked before Test
  - joint loops refused
  - one ground per run
- **Physics.**
  - Jolt `substeps` and friction per collider
  - MuJoCo `noslip_iterations`, MJCF armature, static scenery on the world body, faster input
    reads
  - `convexHullOf` on nearly flat sets
- **Python.** `ParallelEnvs` over shared memory with a Gymnasium `VectorEnv`; environments take
  a profile, an overlay and a world; a legged sample profile (#392, #397).
- **Documentation.**
  - the comparison with Gazebo Jetty, MuJoCo and PyBullet
  - a builder guide and clips, six sensor clips, a viewer guide
  - draft RFCs 0017, 0018 and 0019 (drafts, not features)
  - the MuJoCo 4 and legged-contact spikes
- **Behaviour changes to label.** See F4. State hashes change for MuJoCo worlds with static
  bodies.

## G. RFCs 0020 to 0022 (Draft, added 2026-11-17)

Reply "accept all" to take every proposed answer, or override by number, e.g. "accept all except
RFC 0021 Q5". Each RFC's own acceptance is the first line of its group. Each ends with a pre-approval
question for its gated schema merges; "Yes" there means the owner pre-approves them.

### G1. RFC 0020, actuators ([rfcs/0020-actuators.md](../../rfcs/0020-actuators.md))

Adds the actuators Lagrange lacks (suction and magnetic grippers, hobby and smart servos, lead
screws and linear actuators, steppers, solenoids; later pneumatic and hydraulic cylinders), each with
equations, a reference, signals, authoring, ROS 2 interface, tier behaviour and a substitute.

- Accept RFC 0020 → accept (not stated in the RFC; Status is Draft).
- Q1 — grasped objects held by force pairs or temporary welds? → Force pairs (pull-off, slip, peel; no engine change; works on plugin backends).
- Q2 — one `actuators` domain or one per family? → One domain, a module per family running only when its prims exist; `--domain actuators=off` turns all off.
- Q3 — porosity of target surfaces? → Add optional `sensmat:airPermeability` (fallback 0, airtight) to `SensorMaterialAPI_1`, plus a `cardboard` preset value (TODO(verify) a published value).
- Q4 — `elec:gearBackEfficiency` fallback? → −1, meaning "use `gearEfficiency`", so every 1.x file behaves as before.
- Q5 — servo pulse input or angle input as primary command? → Both: `pulse_width` for flight controllers and hobby boards, `goal_position` for ROS and ros2_control; the pulse goes through the servo's own map.
- Q6 — smart-servo overload shutdown? → Model the overload and overheating flags only (current over limit too long; winding temperature from the thermal node); the error-status bit layout is out of scope.
- Q7 — the Dynamixel serial protocol (Protocol 2.0 over a virtual port)? → Not in this RFC; ros2_control and topics cover it. An emulator only if users ask.
- Q8 — stepper `lost_steps`, which no real driver reports? → Keep it as a documented ground-truth signal; back-EMF stall detection is a later addition (TODO(verify) which drivers publish their method).
- Q9 — pneumatics and hydraulics here or in their own RFC? → Their own RFC before phase 3; this RFC only reserves `fluid_power`, `FluidSupplyAPI_1`, `ValveAPI_1`, `CylinderAPI_1`.
- Q10 — does the `ideal` servo substitute replace `JointControl`? → No; `JointControl` stays the motor-less tool, and a joint with `ServoAPI_1` follows its servo instead and logs it once.
- Q11 — pre-approve gated schema merges of phase 1 (as for RFC 0011)? → Yes, phase 1 only; phases 2 and 3 return to the owner.
- Q12 — plugin ABI? → No change; plugins already write `body/<b>/wrench/<source>`, `joint/<j>/torque_command`, `load/<n>/power_demand`, and substitute through RFC 0008.

### G2. RFC 0021, lights ([rfcs/0021-lights.md](../../rfcs/0021-lights.md))

Lamps on robots and in scenes (headlights, indicators, brake lights, LEDs, beacons, street and traffic
lights): candela over a beam, colour, rise time, blink patterns, power from the bus, seen by cameras and
light sensors.

- Accept RFC 0021 → accept (not stated in the RFC; Status is Draft).
- Q1 — built in (`lights` domain plus renderer) or a plugin as asked? → Built in; a plugin remains the way to give a lamp custom behaviour.
- Q2 — which axis is the beam? → +X of the lamp's prim, stated in the schema doc; the SDFormat importer rotates its −Z `<light>`.
- Q3 — Lagrange's own `lamp:` attributes or UsdLux's? → Own, for photometric units from datasheets; write matching UsdLux for other tools, read bare UsdLux lights with a warning.
- Q4 — one schema for every lamp kind or one per kind? → One, with `lamp:kind`.
- Q5 — how many lamps per frame? → 64 in a storage buffer, nearest and brightest first by `I/d²`, one log line when more exist.
- Q6 — add luminous intensity as a unit dimension? → Yes (cd, lx); the unit strings `cd` and `lx` are new text.
- Q7 — filament inrush current? → Not modelled; a docs note, a later optional attribute.
- Q8 — lamp shadows in phase 1? → No, phase 2; the limit is stated.
- Q9 — lamp light scattered by fog (veiling luminance)? → Phase 2, with the per-pixel fog veil and single-scattering closed form (TODO(verify) reference).
- Q10 — plugin ABI additions (`world_lamp` getter, tokens)? → None now; plugins control lamps through signals; a getter would be an ABI MINOR in its own RFC.
- Q11 — pre-approve gated schema merges of steps 2–7? → Yes.
- Q12 — model the moon's phase in `env/sky_illuminance`? → Not in phase 1 (moonless 0.8 mlx at night, documented); needs the moon's position, phase and a cited illuminance.

### G3. RFC 0022, fire ([rfcs/0022-fire.md](../../rfcs/0022-fire.md))

Fires that start, grow, spread, heat parts, make smoke that blinds cameras and lidars, and are put out
by water, wind or smothering, using fire-protection engineering correlations; depends on RFC 0020 (the
nozzle) and RFC 0021 (flames light the scene).

- Accept RFC 0022 → accept (not stated in the RFC; Status is Draft). Its nozzle needs RFC 0020 accepted.
- Q1 — built-in domain or plugin? → Built in, as tethers are (fire feeds thermal, sensors, renderers and the viewer).
- Q2 — FDS as a separate program, files only? → Yes; one rule for offline solvers, nothing linked.
- Q3 — ship fuel presets with SFPE Handbook and Babrauskas numbers? → Yes, as individual cited values, once the owner confirms that is acceptable; until then presets ship empty and samples author their own values with sources.
- Q4 — χ_r fallback 0.3 or required? → Required; presets give it (no device values in fallbacks).
- Q5 — burn-out decay over the last 20 % of fuel? → Yes, both: `fire:decayFraction` (fallback 0.2) and an `fire:hrrCurve` table that replaces §2 entirely.
- Q6 — smoke by puffs or by the live LBM air box? → Puffs first (any size, any tier, deterministic); advecting smoke in `LbmBox` is a later step.
- Q7 — blow-off with no general correlation? → A user parameter, off by default, plus optional `fire:blowOffMaxHrr`; search the literature before step 3 (TODO(verify)).
- Q8 — wind raising a pool fire's burning rate? → Not in phase 1; documented.
- Q9 — local CO2 dilution in open air? → Behind `fire.co2_open_air: true` (default false), documented as unvalidated.
- Q10 — dry powder and clean agents? → Not in this RFC.
- Q11 — lidar returns from flames? → None; revisit if a published measurement is found.
- Q12 — tank sloshing and inertia? → Left out; the tank is a weight only, and the docs say a large tank's inertia is understated.
- Q13 — pre-approve gated schema merges of steps 2–7? → Yes.
