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
