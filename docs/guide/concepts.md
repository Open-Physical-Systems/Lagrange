# Concepts

Lagrange simulates a robot as a set of **modules** that exchange **signals** over a bus. Each
physical **domain** (rigid-body motion, electrical, thermal, sensors, ...) is one or more modules,
and any domain can be switched off and replaced by a **substitute**. A **profile** says which
domains run and how. This page explains these ideas. The design documents behind them are
[architecture.md](../design/architecture.md) and RFC 0001.

## Domains

A domain is one kind of physics, simulated by its own modules:

| Domain | In v0.2 | Substitute when off |
|---|---|---|
| `rigid_body` | MuJoCo 3.12: bodies, joints, contacts, at 1 ms | `fixed`: the robot holds its pose |
| `sensors` | IMU and 2D/3D lidar, with noise models | `ground_truth`: exact values, no noise |
| `electrical` | v0.3: battery, bus, motor drives and loads, averaged, at 100 µs, for robots with motor prims (the sample AMR's come from its overlay file) | `ideal`: the commanded torque reaches the joint |
| `thermal`, `aerodynamics`, `weather`, `emi`, `radiation` | no module yet | `ambient`, `zero`, `clear`, `zero`, `zero` |

Domains without a module are always substituted. Robot-class domains (`comms`, `traffic`,
`vehicle_dynamics`, `rotor_aero`, `hydrodynamics`) follow in later releases.

### Substitutes

Turning a domain off must never break the domains that depend on it. So every domain has
substitutes, of three kinds:

- **ideal**: what the domain would deliver if it were perfect. With electrical off, the
  commanded torque reaches the joint exactly. With sensors off, sensors report the true state.
- **constant**: a fixed value. Thermal off means a constant ambient temperature.
- **replay**: values from a recording (MCAP) of an earlier run or of a real robot.

A substitute writes the same signals, with the same units, as the module it replaces, so the
rest of the run does not notice the swap. That is how you can study navigation with perfect
sensors, or motor currents on a robot that does not move.

## Signals and the bus

Modules never call each other. Each module declares the signals it reads (**inputs**) and writes
(**outputs**), and the scheduler connects writers to readers by name.

- **Names** follow `<class>/<instance>/<quantity>`: `joint/left_wheel_joint/velocity`,
  `body/base_link/pose`, `base/command/linear_velocity`.
- **Types**: `Scalar`, `Vec3`, `Quat`, `Pose` (position + quaternion), `Wrench`,
  `ScalarArray` (for example a lidar's ranges), `Bool`, `Int`, and handles to images, point
  clouds and fields.
- **Units** are part of every declaration: `rad/s`, `N·m`, `m`. Each output has exactly one
  writer; any number of modules may read it.

A module reads and writes through `SignalId`s it got at declaration time. In the step loop,
a read or write is an array access (about 4 ns; see [benchmarks](../design/benchmarks.md)).

### Units are checked, never converted

When the scheduler connects a reader to a writer, both declarations must agree on the unit:
the same dimensions, the same scale, and the same kind of quantity. A mismatch stops the run
before it starts, naming the signal, both modules and both units:

```text
UnitMismatch joint/left_wheel_joint/velocity: writer rigid_body/mujoco: rad/s (AngularVelocity);
reader sensors/wrong_unit: m/s (LinearVelocity)
```

The kind matters too. Torque (N·m) and energy (J) have the same SI dimensions, and they are
still refused as a pair. A reader that wants millimetres when the writer gives metres is also
refused: Lagrange never converts silently, so a wrong assumption in one module cannot hide.

## Time and the scheduler

Every module has its own step size: physics 1 ms, the IMU 5 ms, a lidar 100 ms, later an
electrical model 100 µs. The scheduler is a **multirate** co-simulation master:

- The **base tick** is the greatest common divisor of all step sizes. At each tick, the modules
  that are due step, in a fixed order: writers before their readers, ties broken by step size,
  domain and name. The order is the same on every run.
- A reader that steps more slowly than its writer chooses how to see the values in between with
  the input's **hold**: `ZeroOrder` (the latest value, the default), `WindowAverage` (the mean
  since the reader's previous step), or `LinearExtrapolate`.
- A loop (a controller reads joint velocities, and physics reads the controller's torques) needs
  one input marked **feedback**: it reads the writer's previous step. Without one, wiring
  reports a `Cycle`.
- Disabled modules are not in the run at all, so they cost nothing.

`lagrange graph` draws the wiring; the [CLI page](cli.md) shows the sample AMR's.

## Capability tiers

Every feature defines what it does on each tier of hardware:

| Tier | Hardware | Lidar, for example |
|---|---|---|
| 0 | CPU only (servers, CI runners) | exact casts in double precision; Embree skips the shapes a beam cannot reach; all cores |
| 1 | a GPU without hardware ray tracing (integrated, older) | a compute shader over a bounding-volume hierarchy |
| 2 | an RTX-class GPU with hardware ray tracing | (later) hardware ray tracing |

`lagrange info` prints the tier of each GPU on the machine.

## Determinism

A run can be **deterministic**: the same profile, robot and seed give bit-identical results
on the same machine. This makes failures reproducible and lets tests compare whole runs by a hash
of their state. In a deterministic run:

- the scheduler refuses modules that say they are not deterministic (GPU paths, live ROS input);
- random noise comes from counter-based generators (Philox) seeded per run and module, so
  adding a module does not change another's noise;
- every module can save and restore its complete state.

`lagrange_server` runs deterministically when nothing outside the simulation steers it: no
ROS 2 bridge (`--no-ros`) and the lidar on the CPU (the default; `--lidar gpu` moves it to the
GPU, and the run is then not deterministic).

## Coordinates

World positions are double precision, in metres, east-north-up. Physics and rendering work in a
**region-local** frame near the robot (a floating origin), so a world 500 km from its origin
simulates exactly like one at the origin. `/odom`, `/tf` and the viewer use the region-local
frame. Details: [huge-worlds.md](../design/huge-worlds.md).
