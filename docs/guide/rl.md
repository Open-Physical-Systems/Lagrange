# Learning: environments and domain randomisation

`lagrange.envs` runs Lagrange as a reinforcement-learning environment from Python, headless and
without ROS. `LagrangeEnv` has Gymnasium's `Env` interface (and is one when `gymnasium` is
installed); `ParallelEnvs` and `VectorEnv` run many of them in worker processes over shared
memory. One `Simulation` runs in each process: every domain the robot's files ask for runs in
each environment.

```python
from lagrange.envs import LagrangeEnv

env = LagrangeEnv("arm.urdf", mode="effort", action_scale=50.0, reward=my_reward)
observation, info = env.reset(seed=1)
observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

A reset goes back to the state saved when the environment was made, so episodes are cheap and
repeat exactly for a seed.

## Domain randomisation

`randomize=` draws the run afresh at each reset, from the generator that `reset(seed=)` seeds:
the same seed gives the same draws, and `info["randomization"]` says what was drawn (every
parameter's value, and the pushes).

```python
env = LagrangeEnv(
    lagrange.sample_quadruped(),
    overlay=lagrange.sample_quadruped_overlay(),
    randomize={
        "mass": {"bodies": "*", "scale": (0.8, 1.2)},  # each body, inertia with it
        "friction": (0.5, 1.2),  # every collider, scenery too
        "joint_damping": {"joints": "*_knee", "value": (0.0, 0.5)},
        "motor_resistance": (0.9, 1.1),
        "sensor_noise": (1.0, 3.0),
        "start": {"x": (-1, 1), "y": (-1, 1), "yaw": (-3.14, 3.14)},
        "push": {"force": (20, 60), "duration": 0.1, "count": (0, 2)},
        "wind_speed": (0, 8),
    },
)
```

A key's value is either a range `(low, high)`, drawn uniformly, or a dict that says what it acts
on and how:

- a target list: `"bodies"`, `"joints"`, `"motors"`, `"batteries"`, `"rotors"` or `"sensors"`,
  a glob or a list of globs (default `"*"`; motors, batteries and rotors match their prim's path
  or name);
- `"scale"` (a factor on the file's value) or `"value"` (the value itself), a range;
- `"per"`: `"each"` (default, one draw per target) or `"all"` (one draw for all of them).

| Key | A bare range is | Sets |
|---|---|---|
| `mass` | a scale | each body's mass, and its inertia by the same factor (the same shape, denser); `"inertia": False` keeps the inertia |
| `com` | an offset, m | the centre of mass on each axis of the body frame |
| `inertia` | a scale | all six inertia components |
| `friction` | a value | the Coulomb coefficient of every collider of the bodies, and of the scenery (`"world"` in `bodies`). MuJoCo takes the larger of two touching colliders' coefficients, so set both |
| `joint_damping` | a value, N·m·s/rad or N·s/m | the joints' viscous damping |
| `joint_friction` | a value, N·m or N | the joints' dry friction |
| `motor_resistance` | a scale | `elec:windingResistance` |
| `motor_torque_constant` | a scale | `elec:torqueConstant` |
| `motor_current_limit` | a scale | `elec:maxCurrent` |
| `battery_capacity` | a scale | `elec:battery:capacity` |
| `battery_resistance` | a scale | `elec:battery:r0`, the series resistance per cell |
| `rotor_thrust` | a scale | `aero:rotor:thrustCoeff` |
| `rotor_torque` | a scale | `aero:rotor:powerCoeff` |
| `sensor_noise` | a scale | every setting of the sensors with `noise` or `random_walk` in its name |
| `sensor_bias` | (a dict) | IMUs: `{"accel": (lo, hi), "gyro": (lo, hi)}`, the turn-on bias mean (m/s², rad/s), of either sign per axis |
| `sensor_latency` | a value, s | the sensors' latency (`sensor/<n>/delayed/...`; a latency of 0 has no delayed signals) |
| `start` | (a dict) | `{"x", "y", "z": m, "yaw": rad}`: the robot's start moved in the world frame and turned about the vertical through its first body |
| `push` | (a dict) | push disturbances: `force` (N) and/or `torque` (N·m) magnitude ranges, `duration` (s, default 0.1), `count` (a number or a range, default 1), `window` (s after the episode starts, default the whole episode), `bodies` (default the robot's first body), `vertical` (default false: forces in the horizontal plane). Each push is a constant wrench at the body's centre of mass, in a direction drawn uniformly |
| `gravity`, `wind_speed`, `wind_from`, `temperature`, `air_density`, `pressure`, `relative_humidity`, `rain_rate`, `visibility`, `snow_rate`, `snow_cover` | a value | the site, as `Simulation.set_site` |

An unknown key, an unknown entry in a dict, a range with `low > high`, or a key with nothing in
the run to act on (`motor_resistance` on a robot without motors) is refused with a `ValueError`
that names it, when the environment is made. `ParallelEnvs` and `VectorEnv` take the same
`randomize=` and check it before any worker starts; environment *i* is seeded `seed + i`.

Every draw applies on the files' values, so an episode never keeps the last one's:
`reset(options={"randomize": False})` gives the files' values exactly, bit for bit the run of an
environment made without `randomize`.

### Underneath: `Simulation.parameters` and `reset_to`

The environment drives two calls of `lagrange.Simulation`, which can be used on their own:

```python
sim = lagrange.Simulation(robot, pushes=True)
state = sim.save_state()
sim.parameters()  # {"body/base_link/mass": 1.9, "joint/j1/damping": 0.0, ...}
sim.reset_to(state, {"body/base_link/mass": 2.3, "world/friction": 0.6, "site/gravity": 9.6})
sim.set_pushes([("", 1.0, 0.1, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0)])  # body, start, s, F, tau
```

`parameters()` names every parameter with its file's value: `body/<b>/mass`,
`body/<b>/com_x|y|z`, `body/<b>/inertia_xx|yy|zz|xy|xz|yz`, `body/<b>/friction`,
`world/friction`, `joint/<j>/damping|friction`, `sensor/<n>/<setting>`, the numbers of
Lagrange's schemas as `<prim path>.<attribute>` (as the inspector edits them), and
`start/x|y|z|yaw`. `reset_to(state, parameters)` goes back to `state` with those parameters
(and `site/<name>` values) set on the files' values and every other one at its file's.
`set_pushes` needs the run made with `pushes=True` (the robot's first body) or a list of bodies;
it writes `body/<b>/wrench/push`, which is summed with the other wrenches on the body.

### What a reset costs

Each parameter takes the cheapest path that is still exact:

| Path | When | What it does |
|---|---|---|
| Restore | nothing differs from the run as it is | the saved state is copied back (`restore_state`) |
| Rebuild and restore | only the site differs | the modules are made again from the world tables and the state restored into them, as `set_site` does |
| Rebuild afresh | a table parameter differs (mass, friction, motors, sensors, start, ...) | the world tables are set, and the modules made again from them at t = 0, as the run started. No file is read again and no model is composed |
| No reset at all | pushes | the push module reads its schedule as an input, like a joint command |

Mass, inertia and friction live in the physics engine's model, motor and battery parameters in
the electrical network's, and so on: no module can change them while it runs without its own
hook, so the rebuild from the tables is the cheapest correct path, and it costs about as much as
the rebuild a site value already paid. Measured on one core (2026-11-22, the CPU lidar, the mean
of 40 resets, each alternating between a changed and the files' value):

| Robot | Restore, ms | Site value (rebuild and restore), ms | Body mass (rebuild afresh), ms |
|---|---|---|---|
| arm6 (MuJoCo) | 0.01 | 3.6 | 5.4 |
| Sample AMR, two lidars | 0.14 | 1.7 | 1.2 |
| Quadcopter (rotors, motors, battery) | 0.01 | 4.4 | 4.5 |
| Quadruped | 0.03 | 1.2 | 1.1 |

A randomised reset costs a few milliseconds; making the run (reading its files and composing the
model, tens to hundreds of ms) is never paid again.

A run started from the tables is the run as it was made: an episode with the files' values after
randomised ones repeats the plain run exactly (`test_randomization.py`).

### What is checked

`python/test/test_randomization.py` checks each parameter against a closed form: a heavier body
falls the same (z0 − g t²/2) but the quadcopter's rotors then carry its weight (ΣT = M g); a box
pushed on the ground slides a T²/2 + (a T)²/(2 μ g) with a = F/m − μ g; a push gives F T/m and
a torque τ T/I; a centre of mass past the base's edge tips the box; a damped slide reaches
F/c (1 − e^(−c t/m)) and dry friction f takes (F − f)/m or holds it; a motor's torque is Kt i_q
and its loss grows with R; a current limit holds; the state of charge falls by the charge over the
capacity and the voltage by R0 I; thrust and torque over ω² scale with their coefficients; an
IMU's spread scales with its noise and its mean with its bias; latency delays the samples; then
that draws repeat per seed and that no episode leaks into the next.
