# The `lagrange` command

`lagrange` runs, checks and inspects simulations from the command line. It is installed with
the simulator (package `lagrange_server`):

```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 run lagrange_server lagrange <command> [arguments]
```

| Command | What it does |
|---|---|
| `lagrange run PROFILE [options]` | Runs the simulator with the profile applied and serves it to viewers and ROS 2, like `lagrange_server` |
| `lagrange validate PROFILE... [options]` | Checks each profile against `profiles/schema.json` and against what this simulator can run. Exits 1 if any profile has an error |
| `lagrange graph [PROFILE] [options]` | Prints the wiring of the run (modules and the signals between them) as a Graphviz digraph |
| `lagrange determinism [PROFILE] [options] [--seconds N]` | Runs the simulation twice, without ROS and with the CPU lidar, and compares the hashes of the whole state every 100 ms of simulated time. Exits 1 at the first difference (default 10 s) |
| `lagrange info [--version]` | Prints the version, the installed sample robot, world and profile schema, the viewer socket, and the Vulkan devices with their capability tiers |

The options are the same as `lagrange_server`'s (`--robot`, `--world`, `--overlay`, `--spawn`,
`--lidar`, `--rtf`, `--no-ros`, `--record`, ...; `lagrange --help` lists them).

`--overlay FILE.yaml` applies the project's schemas to a robot from a format that cannot carry
them (motors, battery, bus on a URDF; [formats.md §9](../design/formats.md#9-importers)). The
sample AMR gets its power system from `amr_diffdrive.overlay.yaml` by default; another `--robot`
gets no overlay unless one is given, and `--overlay none` turns it off.

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
