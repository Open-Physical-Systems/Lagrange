# RFC 0008: Plugins in the server: loading, scene access, custom sensors and physics backends

| | |
|---|---|
| Status | Accepted (2026-10-30) by the owner, with the proposed answer to each unresolved question; gated merges of its steps pre-approved |
| Created | 2026-10-30 |
| Affects | `core/plugin_abi/` (ABI 1.1, additions only), `server/`, `profiles/schema.json` (additions), `schemas/sensor/` (one new schema), `io/importers/`, `bridges/ros2/`, `docs/guide/` |
| Builds on | RFC 0001 (module interface, C ABI), RFC 0005 (profiles), RFC 0007 (the 1.x freeze) |

## Summary

Make the plugin C ABI usable end to end, so that anyone can extend Lagrange without changing
it: `lagrange_server` loads plugins, a plugin can see the scene (bodies, joints, colliders,
sensors, rays), a robot file can carry custom sensors that a plugin implements, a plugin's
outputs reach ROS 2, and a whole rigid-body engine can be a plugin, checked by a conformance
suite. Every change is an addition allowed by RFC 0007: ABI 1.0 plugins keep loading, and every
1.0 profile, robot and recording keeps working.

## Motivation

The ABI exists (RFC 0001; frozen at 1.0 by RFC 0007) and `PluginLibrary` loads and adapts a
library in tests, but:

1. **Nothing loads a plugin.** `lagrange_server` has no option, profile key or search path for
   one; the module guide still says plugins "come with a later release".
2. **Plugins are blind.** The host API gives joints, schema values, the profile and the bus.
   A sensor plugin (sonar, time of flight, bumper, gas, people counter) needs ray casts, bodies
   and its mounting pose; a physics plugin needs every body, joint and collider.
3. **No custom sensor can be put on a robot.** Unknown `<sensor>` types in URDF and SDFormat are
   skipped; there is no USD schema for one.
4. **Plugin outputs stop at the bus.** The viewer's scopes and recordings see them; ROS does not.
5. **The physics engine is closed to others.** MuJoCo and Jolt are compiled in. Their signal
   contract (`rigid_body.md` §3, §5) is already engine-neutral, but nothing lets another engine
   (Bullet, ODE, Drake, PhysX, a research solver) implement it without forking.

## Design

### 1. Loading

- `lagrange_server --plugin LIB` (repeatable; `lagrange` passes it on). `LIB` is a path, or a
  bare name looked up as `lib<name>.so` in `$LAGRANGE_PLUGIN_PATH` (colon-separated), then in the
  `lagrange_plugins` ament resource index, so `colcon build` of a plugin package and
  `--plugin acme_tof` is enough.
- A profile may list them too: a new optional top-level key
  `plugins: [{library: acme_tof, config: {...}}]`. `config` is free-form and is what the plugin's
  `config_get_real`/`config_get_string` read for a domain the plugin introduces.
- Each module a library returns (`lagrange_plugin_get_module(…, index)`) is registered by its
  `domain/name`:
  - a **new domain** (`acme_sonar`) runs whenever it is loaded, like a built-in module whose
    prims exist; `--domain acme_sonar=off` turns it off (its contract is then its outputs at
    their `default_value`, the zero substitute);
  - a **built-in domain** offers a backend or a substitute: `--physics bullet`,
    `domains: {rigid_body: {backend: bullet}}`, `domains: {electrical: {enabled: false,
    substitute: acme_bench}}`. The profile schema's `backend` enum becomes a string with the
    built-in names as examples (accepts everything it accepted before); the server checks the
    name against what it loaded and refuses an unknown one, as it refuses an unknown key now.
- Loading fails at startup, naming the library, for a missing file, another ABI major, no
  modules, two modules with one `domain/name`, or a module whose declaration fails.
- **Rebuilds**: a plugin module is created again on every build, like built-in modules (profile
  apply, spawn, floating origin); its state is carried by `save_state`/`restore_state` where the
  host carries state (RFC 0001), and a plugin without them restarts.
- **Trust**: a plugin is native code in the server process, with the server's rights. Only the
  person who starts the server loads plugins: `--plugin` and profiles read from disk at start
  (`lagrange run`). A profile applied from a viewer (`ApplyProfile`) that names a library not
  already loaded is refused, so a remote viewer can never load code (security.md gets a section).
- **Determinism**: a module with `deterministic = 0` makes the run non-deterministic;
  `lagrange determinism` says which module is the reason instead of failing late.

### 2. ABI 1.1: what a plugin can see

Appended at the end of `lagrange_host_api` (MINOR 0 → 1; a 1.0 plugin never reads them, a 1.1
plugin checks `struct_size` before calling them, so it also runs, degraded or refusing, on a 1.0
host). All return values are in SI units and the region-local frame; strings stay valid until
the call returns to the host.

**World, in `declare` and `init`** (the flat tables of `core/world`, read-only):

| Group | Functions |
|---|---|
| bodies | `world_body_count`, `world_body_name`, `_parent`, `_mass`, `_com`, `_inertia` (xx, yy, zz, xy, xz, yz), `_pose_in_parent`, `_rest_pose`, `_kinematic`, `_robot` |
| joints | `world_joint_type`, `_parent`, `_child`, `_axis`, `_limits`, `_effort_limit`, `_velocity_limit`, `_armature`, `_initial_position`, `_actuated` (the existing count and name stay) |
| colliders | `world_collider_count`, `_body` (or `LAGRANGE_WORLD_BODY`), `_shape`, `_size`, `_pose`, `_friction`, `_hull` (points, triangles), `_heightfield` (grid, cell size, heights) |
| sensors | `world_sensor_count`, `_name`, `_kind` ("lidar", "imu", … or a custom kind, §3), `_body`, `_pose_in_body`, `_rate`, `_param_real`, `_param_string` |
| site | `world_gravity`, `world_region_origin` (world ENU of the region-local frame) |

**Scene, in `step`** (the shapes at the body poses committed when the module's step begins;
exact, deterministic, safe from several threads during one step):

```c
typedef struct lagrange_ray_hit {
    uint32_t struct_size;
    double distance;    /* m along the unit direction; +inf when nothing is hit */
    uint32_t collider;  /* index as world_collider_*; LAGRANGE_INVALID_SIGNAL when nothing */
    uint32_t body;      /* the collider's body, or LAGRANGE_WORLD_BODY for fixed scenery */
} lagrange_ray_hit;

int (*scene_ray)(const lagrange_bus*, const double origin[3], const double dir[3],
                 double max_distance, lagrange_ray_hit* out);                 /* 1 if hit */
uint32_t (*scene_rays)(const lagrange_bus*, uint32_t count, const double* origins,
                       const double* dirs, double max_distance, lagrange_ray_hit* out);
uint32_t (*scene_overlap_sphere)(const lagrange_bus*, const double center[3], double radius,
                                 uint32_t* colliders, uint32_t capacity);   /* total found */
```

The host builds the query from the same `ShapeCaster` the CPU lidar uses (Embree culling,
closed-form tests in double, invariant 7), only when a loaded module calls it, and refits it at
most once per base tick. A module that needs the poses of moving bodies also declares their
`body/<b>/pose` inputs, so the scheduler orders it after physics.

A header-only C++ helper, `lagrange/plugin_sdk/World.hpp`, turns these functions into a
`lagrange::World`-shaped struct, so a C++ plugin (an engine wrapper) can reuse code written
against `core/world` without linking Lagrange's C++ libraries; the ABI stays C.

### 3. Custom sensors on robots

- **World**: `SensorType::Custom`, with the sensor's kind (a string such as `"sonar"`) and its
  parameters (`settings` for numbers, a new `stringSettings` for text).
- **URDF / SDFormat**: a `<sensor>` whose `type` is not one Lagrange simulates becomes a custom
  sensor of that kind instead of being skipped: name, link, `<pose>`, `<update_rate>`, and every
  leaf element under it as a parameter by its path (`<range><max>4</max></range>` →
  `range/max = 4`). Gazebo files that use `sonar`, `contact` or `force_torque` therefore load with
  those sensors present, ready for a plugin that implements them.
- **USD**: a new schema `CustomSensorAPI_1` (with `SensorAPI_1`): `string sensor:custom:kind`
  and parameters as `sensor:custom:param:<name>` attributes of type double or string. A new
  schema is an allowed 1.x addition; `frozen_1_0.json` is unchanged.
- A sensor module finds its sensors with `world_sensor_kind`, reads their mounts and
  parameters, and writes `sensor/<name>/<quantity>` signals. Nothing else in Lagrange touches a
  custom sensor; with no plugin for its kind the server logs it once at load.

### 4. Plugin outputs on ROS 2

`lagrange_server --ros-topics FILE.yaml` maps signals to topics and back, in the style of
`--link-topics`:

```yaml
publish:
  - {signal: sensor/front_tof/range, topic: /front_tof/range, type: sensor_msgs/msg/Range,
     frame: front_tof, params: {min_range: 0.05, max_range: 4.0, field_of_view: 0.44}}
  - {signal: acme/gas/ppm, topic: /gas, type: std_msgs/msg/Float64}
subscribe:
  - {topic: /acme/valve, type: std_msgs/msg/Float64, signal: acme/valve/command}
```

- Published when the signal is committed (at the writer's rate), stamped with simulation time.
- Message types, each with a fixed mapping from the signal type: `std_msgs` Float64, Float64MultiArray,
  Bool, Int64; `sensor_msgs` Range, Temperature, FluidPressure, Illuminance, RelativeHumidity,
  LaserScan (a ScalarArray of ranges with the sensor's angles), PointCloud2 (a PointCloud or x, y, z
  triples); `geometry_msgs` PoseStamped, TwistStamped, WrenchStamped, Vector3Stamped. A pair that
  does not fit fails at startup, naming the signal, its type and the message.
- `subscribe` writes an input-only signal (the bridge is its writer), held between messages, so a
  plugin can take commands without linking ROS.
- A plugin may still create its own rclcpp node; that is outside the ABI and its guarantees.
- The same outputs are already visible in the viewer's scopes and in recordings; the viewer
  needs no protocol change (domains, signals and prims are listed by name).

### 5. Physics engines as plugins

A plugin module with `domain = "rigid_body"` is a backend. The contract is the one MuJoCo and
Jolt implement, written down in `rigid_body.md` as normative:

- Outputs per hinge and slide joint: `joint/<j>/position`, `velocity`; for actuated joints also
  `acceleration`, `effective_inertia`, `constraint_torque`. Per moving body: `body/<b>/pose`,
  `twist` (world_local), `acceleration` (proper acceleration, body frame).
- Inputs: `joint/<j>/torque`, `load_torque` (window average, optional), `body/<b>/external_wrench`
  (world_local, at the body origin, optional), `body/<b>/pose_command` for kinematic bodies,
  `env/friction_scale`.
- Behaviour: free root bodies, every `ShapeType` or a refusal in `declare` naming the shape, no
  collisions within one robot's tree, joint limits, armature, `initial_position`, `shift_origin`
  and `save_state`/`restore_state` for a deterministic backend.

**Conformance suite** (`lagrange_physics_conformance`, an installed test runner that takes a
backend name or a library): a box falling onto the plane comes to rest at its half height; a
pendulum's period matches `2π√(L/g)` within 0.5 % at small amplitude; a block on a slope slides
exactly when `tan θ > μ`; a stack of three boxes stays standing 10 s; the sample AMR drives its
demo loop and ends within 2 % of the MuJoCo path; two runs give the same state hashes when the
backend says it is deterministic; save, step, restore, step again is bit-identical; a rebase
leaves the motion unchanged. MuJoCo and Jolt pass it in CI; a third-party engine that passes it
can call itself a Lagrange backend.

**Worked example**: the Jolt backend also built as a plugin library through the C ABI and the SDK
helper (`libLagrangeJolt.so`), passing the suite. This proves a full engine fits through the
ABI, with no new dependency.

### 6. Developer kit

- `docs/guide/writing-a-plugin.md`: the ABI, a sensor plugin step by step (a time-of-flight
  sensor in C: custom sensor in a URDF, rays, noise from the step seed, `/front_tof/range` on
  ROS), and a physics plugin (the contract and the conformance suite).
- `tools/templates/lagrange_plugin_template/`: an `ament_cmake` package with the header from the
  installed `lagrange_core`, a module skeleton, a test that loads it into a headless server, and
  the `lagrange_plugins` resource so `--plugin <name>` finds it.
- The module guide's "v0.2 limitation" note is replaced by a pointer to the plugin guide.

## Compatibility under RFC 0007

| Surface | Change | Allowed because |
|---|---|---|
| Plugin ABI | 1.0 → 1.1: functions appended to `lagrange_host_api`, one new struct | appending to a table, MINOR + 1; `test_abi_layout.c` gains 1.1 assertions, the 1.0 ones stay |
| Schemas | new `CustomSensorAPI_1` | new schema; no existing attribute changes |
| Profiles | new optional `plugins`; `backend` accepts any string | new optional key; the relaxed `backend` accepts every profile it accepted |
| Protocol, recordings | none | — |

## Implementation plan (one PR each, tested on its own)

1. Loading, the profile key, security, `lagrange --plugin`, a C sample in a headless server test.
2. ABI 1.1 world and scene queries, the SDK helper, the time-of-flight sample plugin.
3. Custom sensors: World, URDF/SDFormat importers, `CustomSensorAPI_1`, `lagrange_import`.
4. `--ros-topics` publish and subscribe in the bridge.
5. The rigid-body contract in `rigid_body.md`, the conformance suite, Jolt as a plugin.
6. The plugin guide, the template package, CLAUDE.md §7.

## Alternatives considered

| Alternative | Why not |
|---|---|
| C++ plugin API (classes across the boundary) | Breaks with every compiler or standard library change; invariant 10 chose a C ABI for this |
| Python plugins (nanobind) | Wanted later for prototyping; the GIL and allocation in `step` conflict with real-time and determinism. Can be one C plugin that hosts Python, outside this RFC |
| WebAssembly plugins | A sandbox would be welcome for untrusted code, but no Lyrical-packaged runtime is chosen and the per-call cost for physics is high; revisit with a security need |
| Out-of-process plugins over IPC | Latency per step at 1–10 kHz; FMUs (FMI 3.0) already cover co-simulated components |
| Write a Lagrange physics engine | Years of work against mature open engines (invariant 12); the plugin path lets whoever needs a special solver bring it |
| Expose USD to plugins | Invariant 5: USD is for authoring; plugins read the flat tables |

## Resolutions on acceptance (2026-10-30)

The owner accepted this RFC with the proposed answer to each question below.

1. **Hit details.** Should `lagrange_ray_hit` also carry the surface normal and the material
   (`SensorMaterialAPI_1` reflectivity)? Proposed: yes, appended in this RFC if the caster can give
   normals exactly; otherwise a later 1.x minor.
2. **Contacts for plugins.** Physics plugins could publish contacts (`contact/points`), and sensor
   plugins (bumpers) read them. Proposed: define the packed format in step 5, optional for backends.
3. **GPU access.** A plugin sensor that renders (a thermal camera) needs the Vulkan device.
   Proposed: a later RFC with the batch `BufferHandle` work (RFC 0001 question 6).
4. **Adding bodies from a plugin.** The walking humans could be a plugin that moves kinematic
   bodies through `pose_command`; the bodies come from a spawned file. Proposed: no body creation
   from plugins in 1.x.
5. **Search path name.** `LAGRANGE_PLUGIN_PATH` and the `lagrange_plugins` resource: confirm.
