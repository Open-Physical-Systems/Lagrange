# RFC 0009: Walking humans

| | |
|---|---|
| Status | Accepted (2026-10-31) by the owner, with the proposed answer to each question below; gated merges of its steps pre-approved |
| Created | 2026-10-31 |
| Affects | `schemas/` (one new library), `modules/humans/` (new), `io/stage/`, `io/importers/`, `server/`, `profiles/schema.json` (additions), `assets/humans/`, `docs/` |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0005 (profiles), RFC 0007 (the 1.x freeze), #177 (spawning) |

## Summary

People who walk around robots, as Gazebo's actors do: an articulated body of rigid segments, moved
by a walk cycle whose stride follows the walking speed, following waypoints or wandering, and
steering around robots, walls and each other. Lidars, radars and cameras see the limbs; robots
bump into them. They are moved, not simulated: nothing balances and no muscle pulls. Every change
is an addition under RFC 0007.

## Motivation

The traffic module's pedestrians are capsules on sidewalks. They exist only on worlds with road
lanes, have no limbs, cannot be spawned, and a lidar sees a pole. Testing an AMR or a car near
people needs legs that scissor (leg-tracking detectors depend on it), people in a warehouse, and
a way to tell how close a robot came to one.

## Non-goals

- A humanoid that balances on simulated joint torques: that is the legged robot class, later.
- Skinned meshes: the renderer draws rigid parts (formats.md §9 does not import skins).
- Global path planning: a person heads straight for the next waypoint and steers locally, so a
  route round a wall must be given as waypoints.
- Stairs and foot placement on rough ground: the body follows the ground under it; feet do not
  adapt to slopes or steps.

## Design

### 1. A built-in domain, not a plugin

I said earlier that humans would be a plugin on RFC 0008. This RFC recommends against it. The
bodies must be created by the loader (RFC 0008 ruled out plugins creating bodies), the waypoints
are arrays and tokens the ABI's `world_schema_real` cannot read, and a plugin would have to be
named with `--plugin` before a file with people in it could run. Humans therefore follow the
traffic module: `modules/humans`, domain `humans`, C++, on the same signals a plugin would use.
The cost is that they do not serve as a worked example of the plugin interface.

### 2. The data: `HumanAPI_1`

A new schema library, `schemas/human/schema.usda`, on an Xform prim:

| Attribute | Unit | Fallback | Meaning |
|---|---|---|---|
| `human:height` | m | 1.75 | Scales every segment |
| `human:walkSpeed` | m/s | 1.34 | Preferred speed (Weidmann 1993, already used by traffic) |
| `human:behavior` | token | `stand` | `stand`, `waypoints` or `wander` |
| `human:waypoints` | m, double3[] | [] | World ENU; a `dwell` of 0 walks straight on |
| `human:loop` | bool | true | Back to the first waypoint at the end |
| `human:dwell` | s | 0 | Pause at each waypoint |
| `human:wanderRadius` | m | 5 | Around the start, for `wander` |
| `human:seed` | int | 0 | With the run's seed, for `wander` |

The loader expands the prim into segments (§3). Three ways to get a person:

- `assets/humans/adult.usda` and variants, spawned like a robot: `lagrange spawn
  assets/humans/adult.usda --at 3,0`, or `/spawn_entity`.
- SDFormat `<actor>`, so Gazebo worlds keep their people: name, pose and `<script>` trajectory
  become waypoints with the timing dropped to a speed. Its skin and animation files are ignored.
- `--humans N[,seed=S][,radius=M]`: N wandering people, placed clear of each other and the robots.

### 3. The body

14 segments: pelvis, torso, head, and upper arm, forearm, thigh, shin and foot per side, as capsule
colliders with matching meshes. Lengths are fractions of height from published anthropometry
(TODO(verify) at implementation: Drillis and Contini, as tabulated in Winter's *Biomechanics and
Motor Control of Human Movement*).

Each segment is a kinematic body hanging from the world (`World::Bodies::kinematic`, MuJoCo mocap,
Jolt kinematic), moved by `body/<b>/pose_command`, because both backends already support that and
the conformance suite's contract covers it. So a person pushes a robot and a robot cannot push a
person: contact is one-way, as with traffic. A person weighs nothing in physics.

`human/<name>` is one entity: `/get_entities` lists it with category `CATEGORY_HUMAN`, and
`/delete_entity` removes all its segments.

### 4. The walk cycle

Each stance foot is planted on the ground and the leg solved to reach it (two-link inverse
kinematics); the foot rolls from heel to flat to toe; the swing foot follows an arc to the next
footfall; arms swing with the opposite leg; the pelvis bobs. Cadence and step length follow from
speed by the walk ratio (0.006 to 0.007 m per step/min when walking freely; Sekiya et al. 1996).
Starting and stopping blend to and from a standing pose. As built, this replaces the Fourier fit
to published angle tables this RFC first proposed: the tables could not be sourced, and planting
the foot rules out skating by construction. The resulting joint ranges are compared to the
widely reported ones in the tests (`TODO(verify)` against Winter's tables).

### 5. Steering

At 20 Hz (the limbs at 100 Hz): the social-force model traffic uses (Helbing and Molnár 1995), with
a goal force, repulsion from other people, from robots (their `body/<b>/pose`, every moving body
that is not a person) and from walls, found by a fan of rays over the fixed scenery (the
`SceneQuery` comms and vehicles use). The ground height under a person comes from a ray too. A
cornered person stops and waits instead of walking through; one blocked past a timeout skips to
the next waypoint and logs it. Traffic agents and humans do not see each other in this RFC.

### 6. Signals

Per person: `human/<n>/pose` (pelvis), `speed` (m/s), `gait_phase` (1), `nearest_robot` (m: lets a
test or a scope say how close a robot came), and the optional input `human/<n>/goal` (x, y in m,
overriding the behaviour until the person arrives). ROS reaches them through `--ros-topics`, with
no bridge change. Stepped on the CPU with a seed from the scheduler, so runs stay deterministic;
`saveState` and `shiftOrigin` as the contract asks. Runs on every tier; the viewer needs no change
(the segments are prims) and neither does the protocol or the recording format.

### 7. Substitute

`--domain humans=off` or `humans: {enabled: false, substitute: zero}` hides them, as traffic's
does, for a baseline run. A new optional `humans` key under the profile's `domains`.

### 8. Spawning: a person is not the robot

#177 makes the first thing spawned into an empty run the robot ROS drives, and much of the server
treats the first body as the robot (`baseLink`, `focusWorld`, `recentre`, the viewer's follow
camera). A file with `HumanAPI_1` prims is therefore never the robot: it is added after the robots,
without a prefix on its entity name, and the next real robot still becomes the driven one.

### 9. Limits, stated

- Contact is one-way, and a person never falls or reacts to being hit.
- 14 bodies per person cost physics and viewer updates. Target: 50 people in real time with the
  sample AMR on the development machine, measured in step 2; below it, the crowd option's limit
  and the docs say so.
- Lidar reflectivity and radar cross-section are the defaults until someone gives a measured value
  (`SensorMaterialAPI_1` applies to segments like any prim); a radar's detections of a person are
  not validated.
- A spawn or delete rebuilds the run (#177): people keep their place, and their gait restarts.

## Validation

| Test | Check |
|---|---|
| Speed | a straight walk averages `walkSpeed` within 2 % |
| Gait | cadence and step length within the published relation; joint ranges within the source tables |
| Foot slip | the stance foot moves less than 2 cm during stance (no skating) |
| Sensors | a lidar scan at knee height shows two legs that swap places over a step |
| Avoidance | a person walking at a stopped AMR passes it without `nearest_robot` under a set clearance; on a corridor blocked by it, stops |
| Contact | an AMR driven into a standing person is stopped or pushed; the person does not move |
| Behaviour | waypoints in order, loop, dwell, `goal` override; `wander` stays in its radius |
| Determinism | same seed, same state hashes, MuJoCo and Jolt |
| Spawning | a person spawned into an empty run is not the robot; the robot spawned next is |

## Compatibility under RFC 0007

| Surface | Change | Allowed because |
|---|---|---|
| Schemas | new `HumanAPI_1` library | a new schema; `frozen_1_0.json` unchanged |
| Profiles | optional `domains.humans` | a new optional key |
| Plugin ABI, protocol, recordings | none | a recording carries the new signals as new channels |

## Implementation plan (one PR each)

1. The schema, the loader's body and its standing pose, the entity category, the spawn rule.
2. The walk cycle along a straight line, its tests, and the cost measurement.
3. Steering: behaviours, walls, robots, the `goal` input, the substitute.
4. `--humans`, SDFormat `<actor>`, a `lagrange` sample, a tutorial with a README clip.

## Alternatives considered

| Alternative | Why not |
|---|---|
| A plugin | §1 |
| One kinematic body per person, limbs only drawn | Cheaper, but sensors would see a capsule: the point of this work is lost |
| Jointed segments in physics, driven by position servos | Feet would need balance or a hidden root force; two backends to tune; a person could be knocked into a heap by a solver spike |
| Skinned mesh | Needs skinning in the renderer and the camera raster, and colliders separately anyway |
| Extend traffic's pedestrians | They depend on road lanes; a separate domain can be toggled alone |

## Resolutions on acceptance (2026-10-31)

The owner accepted this RFC with the proposed answer to each question below.

1. **Traffic's pedestrians.** Should they get these bodies too? Proposed: not now; capsules stay
   the default and a later option can swap them, since 14 bodies times a crowd costs real time.
2. **A realistic mesh.** Proposed: start with the capsule mannequin in the repository, and add a
   fetched model later through `fetch_assets` once a suitably licensed one is chosen
   (TODO(verify): no source is chosen; it needs splitting into segments).
3. **Do people react to a robot beyond steering** (stop and look, yield)? Proposed: no.
4. **The substitute**: hide (proposed, as traffic) or stand still?
