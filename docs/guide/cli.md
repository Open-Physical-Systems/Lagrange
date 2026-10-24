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

The options are the same as `lagrange_server`'s (`--robot`, `--world`, `--spawn`, `--lidar`,
`--rtf`, `--no-ros`, `--record`, ...; `lagrange --help` lists them).

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

![Wiring of the sample AMR: MuJoCo at 1 ms feeds the diff-drive controller, two lidars at 100 ms
and the IMU at 5 ms](../imgs/amr-wiring.svg)

The diff-drive controller writes the wheel torques that MuJoCo reads. It reads the wheel
velocities back as feedback inputs, so the loop has a one-step delay instead of a cycle. The
sensors read body poses (and the IMU also twist and acceleration) at their own rates.
