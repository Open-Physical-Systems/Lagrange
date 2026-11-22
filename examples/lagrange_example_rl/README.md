# Learning to control an arm

Environments in parallel, one simulation per process, with gravity drawn afresh each episode. `lagrange.envs.LagrangeEnv` has Gymnasium's interface (`check_env` of Gymnasium 1.4.0 passes), so learning libraries take it as it is; this example uses numpy alone so that it shows the environment rather than a library.

```bash
ros2 launch lagrange_example_rl train.launch.py workers:=4
```

The result it logs, on the machine it was checked on:

```
RESULT: summed joint error 1.42 rad down to 0.23 rad over 8 generations; learned stiffness 102 N m/rad, damping 2.5 N m s/rad
```

Where to look: `scripts/train.py`, `python/lagrange/envs.py`.

What it shows: `LagrangeEnv`, `ParallelEnvs`, `randomize=`.

## Environments in parallel

`lagrange.envs` has two front ends over the same worker processes:

- `ParallelEnvs(make, count)`: `reset(seed)` gives the observations, `step(actions)` the
  observations, rewards and which episodes ended (`terminated`, `truncated` and
  `final_observations` hold the details). `step_async`/`step_wait` split a step in two.
  `send(actions, env_ids)` and `recv(min_envs=1)` step environments at their own pace: `recv`
  gives back those finished first, with their indices, so a slow one does not hold the rest.
- `VectorEnv(make, num_envs)`: Gymnasium's VectorEnv interface (it is one when Gymnasium 1.1 or
  later is installed; checked with 1.4.0), `reset(seed=, options=)` and the five-part `step`, with
  the same-step autoreset: an ended episode's last observation is in `infos["final_obs"]`.

Actions, observations, rewards and episode ends go through shared memory and each step wakes a
worker by a semaphore, so nothing is pickled per step; a worker restarts an ended episode itself.
Environment `i` gets seed `seed + i` and steps exactly as one `LagrangeEnv` with that seed would in
one process (`test_envs.py` compares them value for value). `envs_per_worker` puts several
environments in one process; `spin_seconds` (default 1 ms) is how long a waiting process watches
shared memory before it sleeps. A worker whose environment raises, or that dies, raises
`WorkerError` with its traceback, and `close()` (or a `with` block) leaves no process behind.

Policy steps of 20 ms per second, summed over the environments, for the legged spike's quadruped
(12 joints) and hexapod (18 joints) with random position actions, on a 20-thread laptop CPU shared
with other jobs (load 15 to 29 during the run, so the numbers move by 10–20 %). "Independent" is
each process stepping its own environment with no coordination, the machine's limit; the earlier
`ParallelEnvs` sent pickled actions and results through a pipe per worker.

| Robot | Workers | Independent | Earlier `ParallelEnvs` | `step()` | `send`/`recv` |
|---|---|---|---|---|---|
| Quadruped | 1 | 2,950 | 1,860 | 2,850 | 2,780 |
| | 4 | 11,800 | 4,400 | 8,730 | 10,990 |
| | 12 | 25,600 | 7,710 | 12,850 | 23,200 |
| Hexapod | 1 | 1,540 | 1,410 | 1,630 | 1,610 |
| | 4 | 6,710 | 2,900 | 5,170 | 6,390 |
| | 12 | 16,200 | 4,250 | 7,750 | 14,120 |

`step()` waits for the slowest environment each step, which on a loaded machine is a process the
scheduler set aside; `send`/`recv` reaches about 90 % of the independent rate on 12 workers. There are no batches on the GPU: on this machine the sensorless arm gave 14,000 steps of 20 ms per second on one process and 45,000 on four; a robot with a camera is limited by rendering, about 2,400 on any number.

## Many environments in one process

A worker process costs 35 to 40 MB and hundreds of milliseconds to start (the interpreter,
numpy, the Lagrange libraries); a simulation in it costs 1 to 3 MB and 5 to 15 ms. So for many
environments, put several in each worker and step them on threads:

```python
envs = ParallelEnvs(make_env, 64, envs_per_worker=16, worker_threads=16)  # 4 processes
envs = VectorEnv(robot, 64, envs_per_worker=16, worker_threads=16, env_options={...})
```

Each simulation lets go of Python's lock while it steps, so a worker's threads keep several cores
busy; each thread steps the same environments every time. The defaults (`envs_per_worker=1`,
`worker_threads=1`) are the earlier behaviour: one process each, or several stepped in turn.

- Every environment stays its own run, bit for bit: environment k of a threaded worker gives
  exactly what the same seed gives alone in one process (`test_envs.py` compares observations
  and the whole-state hash at every step, for the AMR with lidars and the quadcopter).
- Pass `cameras=False` to `LagrangeEnv` (or `Simulation`) when the policy does not observe images:
  cameras then publish their ground truth and no GPU device is made, one per simulation. Cameras
  rendering from threads have not been checked.
- Environments built by your own `make` must be safe to step beside each other on threads; a
  plugin's module must keep no global state between instances.
- What one process gives is bounded by its Python share (observations, rewards, actions hold the
  lock): with cheap steps, use more workers with fewer threads each.

On the 20-thread laptop, shared and heavily loaded, at 64 environments four threaded workers used
4 to 7 MB per environment instead of 35 to 39 MB, started in 0.5 to 1 s instead of 2 to 4 s, and
stepped faster than the same workers stepping in turn for each sample robot
(`docs/design/benchmarks.md` §6; `tests/benchmarks/scripts/rl_throughput.py` measures it on yours).

## Profile, overlay and world

`LagrangeEnv(robot, world, profile=, overlay=)` hands all three to `lagrange.Simulation`: the
scenery, a simulation profile (a path, or an example's name as `lagrange.sample_profile` takes
it: which domains run and which are substituted, the physics backend and its step), and a power
system for the robot (an overlay YAML or a SPICE netlist, as `lagrange_server --overlay`). Its
signals can then be observed like any other, through `signals=`. `ParallelEnvs` and `VectorEnv`
take a robot file in place of `make`, with the same three and `env_options=` for the rest of
`LagrangeEnv`'s options:

```python
import lagrange
from lagrange.envs import LagrangeEnv, VectorEnv

env = LagrangeEnv(
    lagrange.sample_quadruped(),
    profile="quadruped_trot_powered",  # the electrical network at 100 µs
    overlay=lagrange.sample_quadruped_overlay(),  # battery, 48 V bus, a drive on each joint
    mode="position",
    action_scale=0.5,
    signals=["bus/dc_48v/voltage", "battery/battery/soc", "battery/battery/current"],
)
envs = VectorEnv(
    lagrange.sample_quadruped(),
    8,
    profile="quadruped_trot",  # the same robot, electrical substituted
    overlay=lagrange.sample_quadruped_overlay(),
    env_options={"mode": "position", "action_scale": 0.5},
)
```

The sample quadruped (`assets/robots/quadruped`, the trotting example's robot) comes with two
profiles: `quadruped_trot` (MuJoCo at 1 ms, every other domain substituted; `lagrange
determinism profiles/examples/quadruped_trot.yaml` checks it) and `quadruped_trot_powered` (the
same with the electrical network on, for the overlay). The overlay is the sample AMR's battery,
bus, computer and drive on each of the twelve joints, illustrative and not sized for a legged
robot: each drive's 8 A limit gives 12.5 N·m at the joint, below the URDF's 40 N·m.

What the electrical domain costs (`docs/plan/spikes/legged/rl_speed_electrical.py`, results in
`docs/plan/spikes/legged/results/`): policy steps of 20 ms per second with random position
actions, one process with the three cases stepped in turn (median of 15 rounds of 300 steps), and
`ParallelEnvs` with `send`/`recv`, on the same shared 20-thread laptop (load 13 to 14). Two runs:

| Case | 1 process | 4 processes | 8 processes |
|---|---|---|---|
| Bare: no overlay | 2,640 / 1,380 | 5,170 / 4,870 | 13,270 / 9,470 |
| Overlay, electrical substituted (`quadruped_trot`) | 2,640 / 1,390 | 6,700 / 5,320 | 14,400 / 10,840 |
| Overlay, electrical on (`quadruped_trot_powered`) | 990 / 420 | 3,380 / 1,600 | 4,040 / 3,820 |

The absolute rates moved by a factor of two between the runs (the machine was shared), but in one
process, where the cases share the load, the electrical network at 100 µs costs the same each
time: a step takes 2.7 to 3.3 times as long as without it (twelve drives and the battery 200
times per policy step), and in parallel the ratio is about the same (0.30 to 0.40 of the rate,
once 0.65). The overlay alone, with the domain substituted, costs nothing. So run the electrical
network when the policy should learn what the battery and drives do (current limits, sag), and
train on `quadruped_trot` otherwise.
