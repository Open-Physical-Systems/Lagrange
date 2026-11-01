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

What it shows: `LagrangeEnv`, `ParallelEnvs`, `randomize=`. There are no batches on the GPU: on this machine the sensorless arm gave 14,000 steps of 20 ms per second on one process and 45,000 on four; a robot with a camera is limited by rendering, about 2,400 on any number.
