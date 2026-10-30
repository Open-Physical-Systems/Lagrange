# Writing a plugin

A plugin extends Lagrange without changing it: a new sensor, a new domain (gas, radiation, a
crowd), a substitute for a built-in domain, or a whole physics engine. It is a shared library
built against one C header, `lagrange_plugin.h`, and loaded when the simulator starts:

```bash
lagrange --robot my_robot.urdf --plugin my_plugin
```

This chapter builds one from the template, then walks through the time-of-flight sensor that
comes with Lagrange, then a physics engine. The design is RFC 0008; the interface is RFC 0001.
Read [Concepts](concepts.md) first for signals, units and the scheduler.

## 1. Start from the template

```bash
cp -r tools/templates/lagrange_plugin_template ~/ws/src/my_lagrange_plugin
cd ~/ws && source ~/lagrange/install/setup.bash   # or /opt/ros/lyrical/setup.bash with the debs
colcon build && colcon test --packages-select my_lagrange_plugin
source install/setup.bash
lagrange --plugin my_lagrange_plugin
```

The package (`ament_cmake`) builds one `MODULE` library from `src/my_sensor.c`, installs it to
`lib/`, and registers it in the `lagrange_plugins` resource index, which is how `--plugin
my_lagrange_plugin` finds it by name (a path, or a directory in `$LAGRANGE_PLUGIN_PATH`, also
work). Its test runs the plugin in a headless simulator with the sample AMR. The library links
nothing of Lagrange's: the header is the whole interface.

## 2. The interface

A library exports one function, `lagrange_plugin_get_module(major, minor, index)`, which returns
its modules one index at a time (`NULL` after the last, or for an ABI major it was not built
for). A module is a table of functions (`lagrange_module_api`):

| Function | When | Does |
|---|---|---|
| `create(host)` | each run | allocates the module; keeps `host`, the table of host functions |
| `declare(self, world, config, declarer)` | before wiring | reads the world and its settings; declares every input and output with its unit |
| `init(self, world, config, bus)` | after wiring | writes the first value of every output |
| `step(self, context, bus)` | every step | reads inputs, computes, writes outputs. No allocation, no blocking, no I/O |
| `save_state` / `restore_state` | checkpoints, rebuilds | the whole dynamic state, for deterministic runs |
| `shift_origin(self, shift)` | the floating origin moved | positions the module keeps move by −shift |
| `shutdown`, `destroy`, `last_error` | | |

Its fields say what it is: `domain` and `name`, whether it is a substitute, whether it is
deterministic, the lowest capability tier it runs on, and its step (`preferred_step_ns`).

What a module does in the run depends on its domain:

- **its own domain** (`acme_gas`): it runs whenever it is loaded; `--domain acme_gas=off` turns it
  off. Its settings come from the profile: `plugins: [{library: my_plugin, config: {gain: 2}}]`,
  read with `config_get_real("gain", &out)` and `config_get_string`;
- **a built-in domain, as a substitute**: `domains: {electrical: {enabled: false, substitute:
  acme_bench}}`;
- **`rigid_body`, as a backend**: `--physics NAME` (section 5).

### Signals and units

A signal is a name (`sensor/front_tof/range`, `<class>/<instance>/<quantity>`), a type (scalar,
pose, wrench, array, ...) and a unit: SI dimensions, a scale (1 for SI) and a quantity kind that
tells torque from energy. Units are checked when the run is wired: a mismatch stops the start,
naming both modules and both units; nothing is ever converted. Inputs say how they see a writer
at another rate (`hold`: last value, extrapolated, or averaged over the window), whether they
may stay unconnected (`optional`, then `default_value`), and whether they read the writer's
previous step (`feedback`, which breaks a loop).

The signals other modules write are listed by `lagrange graph` and in the viewer's Signals
workspace: `body/<b>/pose`, `joint/<j>/position`, `env/air_density`, `sensor/<s>/ranges`, and so
on. Declaring a body's pose as an input also orders the module after physics.

### Seeing the scene (ABI 1.1)

In `declare` and `init` a module reads the world: `world_body_count` and `world_body` (name,
parent, mass, inertia, poses), `world_joint`, `world_collider` (shapes, hulls, height fields),
`world_sensor` and its parameters, `world_site` (gravity, the region origin). Each getter fills
a struct whose first field, `struct_size`, the plugin sets, so newer fields can be added later
without breaking it. In `step` it can cast rays and query overlaps against every collider where
physics put it: `scene_ray`, `scene_rays`, `scene_overlap_sphere`. They are exact and
deterministic (the CPU lidar's caster) and safe from several threads during one step. A 1.1
plugin checks the host's `struct_size` before calling them, so it can refuse politely on a 1.0
host. C++ plugins can read the whole world into containers with the header-only
`lagrange/plugin_sdk/World.hpp`.

### Rules

- `step` must not allocate, block, read files or talk to the network: it runs on the
  simulation's clock, possibly faster than real time.
- Determinism: noise comes from `context->seed` and `context->step_index` (a counter-based
  generator), never from the clock or `rand()`. A module that is deterministic sets the flag and
  saves its whole state; one that is not (it takes input from outside) clears it.
- A plugin is native code in the simulator's process: load only plugins you would run as
  programs. Only the person starting the simulator loads them (`--plugin`, or a profile read at
  start); a viewer never can.

## 3. A sensor: time of flight

`core/plugin_abi/samples/tof_sensor.c` (installed as `lagrange_tof`) implements every custom
sensor of kind `tof`. A robot file declares one like any Gazebo sensor; a `<sensor>` of a type
Lagrange does not simulate itself becomes a custom sensor with its parameters by path:

```xml
<gazebo reference="base_link">
  <sensor name="front_tof" type="tof">
    <pose>0.25 0 0.1 0 0.3 0</pose>
    <update_rate>50</update_rate>
    <tof><range><min>0.02</min><max>4</max></range><fov>0.44</fov>
         <noise><stddev>0.005</stddev></noise></tof>
  </sensor>
</gazebo>
```

In `declare` it walks the sensors, keeps those whose kind is `tof`, reads `range/max`, `fov` and
`noise/stddev` with `world_sensor_param_real`, finds its body with `world_body`, and declares the
body's pose as an input and `sensor/front_tof/range` (m) and `valid` as outputs. In `step`, on
the steps its rate asks for, it composes the body's pose with the mount, casts a ray (or a 5 × 5
grid over its field of view) along the sensor's +x with `scene_ray`, skips hits on its own body,
keeps the nearest, and adds Gaussian noise from the step's seed. Its test checks the range to the
ground against the exact beam–plane distance (`server/test/test_plugins.cpp`).

To see it on ROS 2, map the signal to a topic (`--ros-topics`, [The `lagrange`
command](cli.md#plugin-outputs-on-ros-2)):

```yaml
publish:
  - {signal: sensor/front_tof/range, topic: /front_tof/range, type: sensor_msgs/msg/Range,
     frame: front_tof, params: {min_range: 0.02, max_range: 4.0, field_of_view: 0.44}}
```

## 4. Commands from ROS 2

A topic can become a signal the plugin reads, again without the plugin linking ROS:

```yaml
subscribe:
  - {topic: /acme/valve, type: std_msgs/msg/Float64, signal: acme/valve/command, unit: "1"}
```

## 5. A physics engine

A module of domain `rigid_body` replaces MuJoCo: `lagrange --plugin my_engine --physics NAME`.
Its contract, what it outputs and accepts per joint and body, is
[rigid_body.md §8](../design/domains/rigid_body.md#8-backends-as-plugins-the-contract-and-the-conformance-suite-rfc-0008-5).
It reads the whole world in `declare` (bodies, joints, colliders) and steps it.

`modules/rigid_body/jolt_backend/src/jolt_plugin.cpp` is the worked example: the Jolt engine as
a plugin (`--plugin lagrange_jolt --physics jolt_abi`). It reads the world with the SDK, rebuilds
the world tables, and runs the same engine code as the built-in Jolt backend, whose signal
access goes through a small interface (`JoltBackend::SignalIo`) implemented over the ABI's
declare and bus functions.

The conformance suite checks an engine against the contract: a falling box, a pendulum's
period, friction on a slope, a stack of boxes, the sample AMR's loop against MuJoCo, determinism
and state restore:

```bash
LAGRANGE_CONFORMANCE=my_engine@libmy_engine.so \
    ros2 run lagrange_server lagrange_physics_conformance
```

## 6. When it does not load

The simulator logs every module it loaded and what it is for (`[plugins] acme_gas/gas from
.../libacme_gas.so (its own domain; ...)`), and stops the start with a reason when it cannot:

| Message | Means |
|---|---|
| `plugin X: no libX.so in LAGRANGE_PLUGIN_PATH and no lagrange_plugins resource` | not built, not installed, or the workspace not sourced |
| `... offers no module for plugin ABI 1.x` | built for another major version, or `get_module` returned `NULL` |
| `UnitMismatch <signal>: writer ...: <unit>; reader ...: <unit>` | the plugin's unit (or quantity kind) is not the other module's |
| `MissingInput <signal>` | nothing writes an input that is not optional |
| `<domain>/<name>: <reason>` | the module refused, with its `last_error` |
| `X is not loaded` (applying a profile) | a profile from the viewer named a library the simulator was not started with |
