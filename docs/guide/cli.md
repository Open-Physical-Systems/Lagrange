# The `lagrange` command

`lagrange` is on the PATH once ROS is sourced (package `lagrange`, `tools/cli/`). Without a
command it runs the simulator and its viewer together, like `gazebo`:

```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
lagrange [SAMPLE | ROBOT [WORLD]] [SERVER OPTIONS] [-- VIEWER OPTIONS]
lagrange --server-only ...         # the server alone, headless
lagrange viewer [VIEWER OPTIONS]   # the viewer alone
lagrange samples                   # amr (default), sedan, quad, lexus, x500, swerve, empty
lagrange empty                     # no robot, no world: the ground only
lagrange spawn FILE [--name NAME] [--at X,Y[,Z[,YAW]]]   # a robot into the running simulator
lagrange delete NAME               # and out again
```

The server listens on a socket of its own for the run (`$XDG_RUNTIME_DIR/lagrange/run-PID.sock`,
unless `--listen` is given), so it never meets another simulator on the default socket; closing
the viewer stops it, and Ctrl-C stops both. The commands below are the simulator's tools; they
are also `ros2 run lagrange_server lagrange <command>`:

```bash
lagrange <command> [arguments]
```

| Command | What it does |
|---|---|
| `lagrange run PROFILE [options]` | Runs the simulator with the profile applied and serves it to viewers and ROS 2, like `lagrange_server` |
| `lagrange validate PROFILE... [options]` | Checks each profile against `profiles/schema.json` and against what this simulator can run. Exits 1 if any profile has an error |
| `lagrange graph [PROFILE] [options]` | Prints the wiring of the run (modules and the signals between them) as a Graphviz digraph |
| `lagrange determinism [PROFILE] [options] [--seconds N]` | Runs the simulation twice, without ROS and with the CPU lidar, and compares the hashes of the whole state every 100 ms of simulated time. Exits 1 at the first difference (default 10 s) |
| `lagrange export spice\|modelica [PROFILE] [options] [--out FILE]` | Writes the robot's power system (from its electrical schemas or overlay) as an ngspice netlist or a Modelica model, for offline study: battery ECM, bus capacitors, loads, and the drives as averaged power sinks whose power you set |
| `lagrange info [--version]` | Prints the version, the installed sample robot, world and profile schema, the viewer socket, and the Vulkan devices with their capability tiers |

The options are the same as `lagrange_server`'s (`--robot`, `--world`, `--overlay`, `--spawn`,
`--lidar`, `--physics`, `--rtf`, `--no-ros`, `--record`, ...; `lagrange --help` lists them).
`--physics jolt` (or a profile's `rigid_body: {backend: jolt}`) runs the rigid bodies on Jolt
instead of MuJoCo; MuJoCo stays the default and the reference (rigid_body.md §7).

`--overlay FILE.yaml` applies the project's schemas to a robot from a format that cannot carry
them (motors, battery, bus on a URDF; [formats.md §9](../design/formats.md#9-importers)). The
sample AMR gets its power system from `amr_diffdrive.overlay.yaml` by default; another `--robot`
gets no overlay unless one is given, and `--overlay none` turns it off.

## Spawning robots

Like Gazebo, the simulator can start without a robot (`lagrange empty`, or `--robot none` with
any world) and take robots while it runs. Three ways do the same thing:

- `lagrange spawn robot.urdf --at 2,0` from a terminal (it prints the entity name, `/robot`);
  `lagrange delete /robot` takes it out;
- the viewer's **File → Add robot or open recording** (Ctrl+O): the robot goes to the origin of
  an empty run, else 3 m beside the robots there are;
- the standard ROS 2 services of `simulation_interfaces`: `/spawn_entity`, `/spawn_entities`,
  `/delete_entity`, and `/reset_simulation` with `SCOPE_SPAWNED`, from any language:

```bash
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity \
  "{name: mate, entity_resource: {uri: $PWD/assets/robots/amr_diffdrive/amr_diffdrive.urdf},
    initial_pose: {header: {frame_id: world}, pose: {position: {x: 0.0, y: 4.0}}}}"
ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{entity: /mate}"
```

The file is a URDF, SDFormat, MJCF or USD path (`file://` and `package://` URIs too), or its text
in `resource_string`; xacro is run first (`xacro robot.urdf.xacro > robot.urdf`). The pose is in
the frame `world`, the frame of `/odom`'s origin; the robot is placed as `--spawn` places it, its
lowest collision shape just above z.

- **The first robot in a run without one is the robot**: `/cmd_vel`, `/odom`, `/joint_states`,
  `/robot_description` and its sensors' topics are its, and the demo driver drives it. Robots
  after it are passive, with their names prefixed (`mate_base_link`), as `--add-robot` makes
  them; `entity_namespace` is not supported yet.
- **A spawn or a delete rebuilds the run** with the new set of robots. Time goes on (`/clock` never
  jumps back), every body keeps its pose and, with MuJoCo, every joint its position; velocities
  start from rest, and the modules' own state (controllers, batteries, filters) starts again.
  A recording stops at the change. `reset_simulation` goes back to t = 0 with the robots there
  are, where they were spawned.
- Any robot can be deleted, the one the simulator started with too; the world's scenery cannot.

## Plugins

A plugin is a shared library built against the C ABI (`lagrange_plugin.h`, RFC 0001 and 0008)
that holds one or more modules. Load it when the server starts:

```bash
lagrange --plugin libacme_sonar.so            # a path
lagrange --plugin acme_sonar                  # lib<name>.so in $LAGRANGE_PLUGIN_PATH, then the
                                              # lagrange_plugins resource of a colcon package
lagrange run profile.yaml                     # with plugins: [{library: acme_sonar, config: {...}}]
```

What a module does depends on its domain:

- **a domain of its own** (`acme_sonar`): it runs in every run; `--domain acme_sonar=off` turns it
  off. Its settings are the profile's `plugins[].config` (`config_get_real`, `config_get_string`);
- **a built-in domain, as a substitute**: the profile names it, `electrical: {enabled: false,
  substitute: acme_bench}`;
- **`rigid_body`, as a backend**: `--physics NAME` or `rigid_body: {backend: NAME}`.

The server logs every module it loaded and what it is for. A profile applied from the viewer can
configure loaded plugins but never load one.

## Profiles and stages

A profile (RFC 0005) chooses which domains run, which substitutes replace the others, and the
step sizes. Its `stage` names what to simulate. This version loads URDF robots, so `stage` is a
URDF path, looked up as written, next to the profile, and in `lagrange_assets`. With
`--robot FILE`, the profile is checked against that robot instead, whatever its stage says.

`profiles/examples/amr_sample_urdf.yaml` is the run the v0.2 simulator offers: the sample AMR
with MuJoCo physics and its IMU and lidars, every domain without a module yet substituted.

```console
$ lagrange validate profiles/examples/amr_sample_urdf.yaml
profiles/examples/amr_sample_urdf.yaml: valid (0 errors, 0 warnings)

$ lagrange validate profiles/examples/amr_warehouse_nav2.yaml --robot assets/robots/amr_diffdrive/amr_diffdrive.urdf
profiles/examples/amr_warehouse_nav2.yaml: invalid (2 errors, 4 warnings)
  error   /domains/electrical/enabled: electrical has no module in this server yet; disable it and name a substitute
  error   /domains/thermal/enabled: thermal has no module in this server yet; disable it and name a substitute
  warning /environment: not applied by this server yet
  ...
```

Errors use JSON pointers into the profile, as the viewer's Domains panel does. Settings the
server accepts but does not apply yet are warnings.

## The wiring graph

```bash
lagrange graph profiles/examples/amr_sample_urdf.yaml | dot -Tsvg -o amr-wiring.svg
```

Each box is a module with its step size; each arrow is one signal, from the module that writes
it to the one that reads it. Dashed arrows are feedback inputs, which read the writer's previous
step to break a loop. Dashed boxes are substitutes standing in for a disabled domain. For the
sample AMR:

![Wiring of the sample AMR: the diff-drive controller's torque commands go to the electrical
network at 100 µs, whose wheel torques MuJoCo reads at 1 ms; MuJoCo feeds two lidars at 100 ms
and the IMU at 5 ms](../imgs/amr-wiring.svg)

The diff-drive controller writes wheel torque *commands*. The electrical network
(`electrical/averaged_network`, every 100 µs) turns them into the torques MuJoCo reads, through
the sample's motors, drives and battery (its overlay file): a command beyond the drive's current
or the bus voltage is not delivered, and the battery's charge, the bus voltage and the motor
currents are signals of their own. With `--domain electrical=off`, or a robot without motors,
the ideal substitute takes its place and passes the commands through unchanged. Both read the
wheel velocities back as feedback inputs, so each loop has a one-step delay instead of a
cycle. `core/wrench_sum` adds up external forces on the
bodies (none in this world). The
sensors read body poses (and the IMU also twist and acceleration) at their own rates.
