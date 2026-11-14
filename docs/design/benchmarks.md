# Performance benchmarks

Roadmap item 0.2-H6. The benchmarks measure the costs that bound how fast a run can go: the
scheduler's overhead per module step, signal bus access, and lidar rays per second on each
capability tier. They live in `tests/benchmarks/` (package `lagrange_benchmarks`, Google Benchmark).

## 1. What is measured

| Benchmark | What one item is | Notes |
|---|---|---|
| `SchedulerTick/modules:N/multirate:M` | one module step | N chained modules, each reading the previous one's output and writing its own (one read, one write per step). With `multirate:1`, every other module steps at 100 µs and the rest at 1 ms. Measures input preparation, commit and ordering |
| `BusAccess` | one `SignalBus::read` or `write` | 1000 of each per step inside a module, so the scheduler's share is negligible |
| `LidarReference/shapes:N` | one ray | Every beam against every shape, exact, in double, one thread: the definition Tier 0 must match bit for bit. A ground plane plus N random shapes within 20 m; 32 768 beams in all directions (a 32-channel, 1024-column scan) |
| `LidarTier0/shapes:N` | one ray | What the lidar module runs on the CPU: the same exact tests, culled by Embree (`ShapeCaster`), beams in parallel with TBB. Wall time of a scan, including the hierarchy refit |
| `LidarTier1/shapes:N` | one ray | The compute kernel over a bounding-volume hierarchy, same scene and beams. Wall time of a whole scan: shape conversion, hierarchy build on the CPU, upload, dispatch and read-back, as the module does at each update |

`LAGRANGE_TEST_GPU=<name>` picks the Vulkan device for Tier 1, as in the tests.

## 2. Running

```bash
colcon build --packages-up-to lagrange_benchmarks --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
tools/ci/benchmarks.sh MACHINE [OUT_DIR]      # three repetitions, medians against baseline.json
```

`MACHINE` is a key of `tests/benchmarks/baseline.json`. A result fails when it is slower than
its baseline by more than `MAX_SLOWDOWN` (default 1.25). Skipped benchmarks (no Vulkan device)
do not fail; a baseline with no result does. For a machine with no baseline, the script prints
the results. To record them:

```bash
install/lagrange_benchmarks/lib/lagrange_benchmarks/check_benchmarks.py \
    --baseline tests/benchmarks/baseline.json --machine MACHINE --record "description" \
    OUT_DIR/bench_core.json OUT_DIR/bench_lidar.json
```

`colcon test` runs each benchmark once as a smoke test only, because timings on a shared
machine are not pass/fail.

The nightly workflow has a `benchmarks` job for the hosted runner (machine key
`ci-ubuntu-latest`, Tier 1 on llvmpipe, `MAX_SLOWDOWN` 2.0 because shared runners vary).
Its baseline is recorded from the job's first log. GitHub Actions workflows are disabled for now
(2026-10-24), so it has not run yet. The roadmap's self-hosted NVIDIA runner gets its own key.

## 3. Baselines (2026-10-24, with the Embree Tier 0)

Intel Core i9-13900H laptop on Ubuntu 26.04, RelWithDebInfo, median of three:

| Benchmark | Result | Per item |
|---|---|---|
| Scheduler, 1 module | 25.3 M steps/s | 40 ns |
| Scheduler, 10 modules | 22.5 M steps/s | 44 ns |
| Scheduler, 100 modules | 21.8 M steps/s | 46 ns |
| Scheduler, 100 modules, multirate | 21.4 M steps/s | 47 ns |
| Bus read or write | 249 M/s | 4 ns |
| Lidar reference, 50 shapes | 1.69 M rays/s | 19.4 ms per scan |
| Lidar reference, 400 shapes | 0.145 M rays/s | 225 ms per scan |
| **Lidar Tier 0, 50 shapes** | **178 M rays/s** | **0.19 ms per scan** |
| **Lidar Tier 0, 400 shapes** | **61 M rays/s** | **0.54 ms per scan** |
| Lidar Tier 1, Iris Xe, 50 shapes | 34.3 M rays/s | 0.96 ms per scan |
| Lidar Tier 1, Iris Xe, 400 shapes | 36.5 M rays/s | 0.90 ms per scan |
| Lidar Tier 1, llvmpipe, 50 shapes | 40.3 M rays/s | 0.81 ms per scan |
| Lidar Tier 1, llvmpipe, 400 shapes | 10.4 M rays/s | 3.14 ms per scan |

The Iris Xe is Intel's integrated GPU (Mesa 26.0.8). llvmpipe is Mesa's software Vulkan device
(LLVM 21.1.8), which runs the same kernel on the CPU's threads. The machine keys are
`i9-13900H-iris-xe` and `i9-13900H-llvmpipe`. Two runs agreed within 3 %.

What the numbers say:

- The scheduler costs about 45 ns per module step, nearly flat from 1 to 100 modules. A
  1 kHz physics module with ten 100 Hz sensors spends under 0.1 % of real time in it.
- The reference scales with the shape count, because every beam tests every shape: 400 shapes
  are 12 times slower than 50. Tier 0 gives the same results bit for bit, about 100 times
  faster at 50 shapes and 400 times at 400: Embree leaves each beam only the shapes near its path,
  and the beams spread over the 20 hardware threads.
- A Tier 1 scan has a fixed cost of about 0.8 ms (submit, wait, read-back), so with few shapes
  a hardware GPU is no faster than llvmpipe. The hierarchy keeps it flat as shapes grow: 400
  shapes cost the Iris Xe nothing more, while llvmpipe slows four times.
- On this machine, Tier 0 beats the integrated GPU by 1.7 to 5 times for these scans and is
  deterministic, so `--lidar auto` stays on the CPU. Tier 1 (`--lidar gpu`) is for lidars with
  far more beams, or a discrete GPU; it needs re-measuring there.

## 4. Whole runs: where the server's time goes (2026-11-12)

`LAGRANGE_SERVER_STATS=1` makes `lagrange_server` print, once a second on stderr, the real-time
factor, the scheduler's module time and what it spends besides (capture, record, world update,
the ROS bridge, emit), the ten costliest modules (ms per wall second, steps, ms per step), and with
viewers connected the loop rate and the scene deltas sent (per second, size, build time, skipped
for congestion). Timing is never part of the state.

Measured headless with `--no-ros --rtf 0` (as fast as it goes) for 25 s, the median of the 1 s
windows after the first four, each scenario run before and after back to back. Machine: the
i9-13900H laptop with an RTX 4060 Laptop GPU, RelWithDebInfo; **shared with about eight other
builds and test runs (load average 15 to 30 on 20 threads)**, so the absolute numbers are low and
noisy (±30 % between repeats); the ratios held across two full repeats.

| Run | Before (RTF) | After (RTF) | What changed it |
|---|---|---|---|
| Sample AMR, warehouse (`--lidar cpu`) | 7.3 | 23.2 | no sleep while time is owed |
| Quadcopter, open field | 14.5 | 52.9 | no sleep while time is owed |
| Thermal camera AMR, 6 people | 3.6 | 4.6 | colour not drawn, palette in parallel |
| Sedan, city block | 1.1 | 4.7 | camera model on the GPU |
| Sedan, city block, with ROS | 1.0 | 5.8 | as above, and no images built for nobody |
| Sedan in the town, 24 cars, 36 people | 0.8 | 2.8 | camera model on the GPU, radar in parallel |
| Town, no robot, 24 cars, 36 people | 3.7 | 4.8 | no sleep while time is owed |

Per step (ms, same runs): the sedan's front camera 76.6 → 11.6 (city) and 73.8 → 11.7 (town);
its radar 2.3 → 1.05 in the town; the thermal camera 11.1 → 8.9; the ROS bridge with the sedan
34 → 3 ms per simulated second when nothing subscribes to the images.

What each change does:

- **The run never sleeps while it owes time.** `runOnce()` polled its sockets with a 5 ms
  timeout every iteration, also when the run was behind real time or ran with `--rtf 0`; a run
  that cannot keep up lost those 5 ms each time (the quadcopter spent 0.66 s of every second
  asleep). Now the poll waits only when the run is on time.
- **The camera model runs on the GPU** (`gpu/kernels/camera/camera_model.slang`) right after the
  render, in the same submission; only the output's size comes back. The CPU `CameraModel::apply`
  (18 to 50 ms for the sedan's 960 × 540 camera: six Box-Muller normals per pixel) stays the
  reference (`LAGRANGE_CAMERA_MODEL=cpu`). sensors.md §2 has the agreement test.
- **The radar casts and works out its echoes in parallel**, summed in ray order (bit-identical,
  checked on a recording of the sedan), and on the GPU with `--lidar gpu`.
- **The thermal camera draws no colour** (`RasterCamera::setColour(false)`) and paints its palette
  in parallel.
- **The ROS bridge builds camera images and lidar clouds only when someone subscribes** (or the
  topic rides on a radio link or sensor delay, whose random draws must not change).
- **Viewers get scene deltas every 8 ms while a long batch of ticks runs**, not once per batch: in
  the town at real time, 105 to 135 deltas a second instead of 40 to 50 (42 kB each, 0.1 ms to
  build; writing them to the socket is most of the cost).

What is left, largest first:

- The camera render itself: 4 to 10 ms per frame for the sedan (8 samples, the realistic look's
  shadow cascades), waited for on the simulation thread. Overlapping it with the next ticks
  would cost a frame of latency.
- MuJoCo with many kinematic bodies: the town's 655 bodies cost 0.16 to 0.25 ms per 1 ms step,
  of which `mj_step1` (positions, broad phase) about 150 µs, the bus reads and writes of every
  body's pose, twist and contact force about 80 µs, `mj_rnePostConstraint` 30 µs.
- The CPU lidar, 1.3 to 4 ms per scan; Tier 1 (`--lidar gpu`) is still not faster for these scans.

### 4.1 Second round (2026-11-12, later)

Same scenarios and method (`--no-ros --rtf 0`, 25 s, the median of the 1 s windows after the
first four), the build before (the first round's state) and after run back to back per scenario,
three times. The table is the third round (load average 1 to 11 on 20 threads, the quietest);
the ratio column gives the range over all three rounds (load 2 to 21).

| Run | Before (RTF) | After (RTF) | Ratio (3 rounds) | What changed it |
|---|---|---|---|---|
| Sample AMR, warehouse (`--lidar cpu`) | 31.1 | 31.8 | 0.99 to 1.02 | nothing it uses |
| Quadcopter, open field | 61.9 | 66.8 | 1.08 to 1.19 | bus commits (most likely: nothing else it uses changed) |
| Thermal camera AMR, 6 people | 11.4 | 13.2 | 1.16 to 1.8 | render overlapped |
| Sedan, city block | 10.0 | 12.9 | 1.29 to 1.45 | render overlapped |
| Sedan in the town, 24 cars, 36 people | 2.5 | 3.7 | 1.11 to 1.45 | render overlapped, MuJoCo |
| Town, no robot, 24 cars, 36 people | 8.3 | 18.6 | 1.87 to 2.39 | MuJoCo, bus commits |

Per step (third round): MuJoCo in the town 0.091 → 0.035 ms, in the town with the sedan 0.095
→ 0.075 ms; the sedan's front camera 6.4 → 4.1 ms (city) and 22.4 → 12.7 ms (town), the
thermal camera 4.2 → 3.3 ms (a split step's cost is its two halves, without what runs between).
Every signal's value is the same bit for bit before and after (all signals hashed every 100 ms
over 20 s of each scenario, cameras off as in `lagrange determinism`); `lagrange determinism`
passes for `amr_sample_urdf`, `sedan_city` and the sedan in the town.

What each change does:

- **The camera renders while the tick's other modules run.** The scheduler has split steps
  (`SplitStep`, architecture.md §6): the camera submits its render in `step()`, the lidar, the
  radar and the rest of the tick run while the GPU draws, and the camera waits for it right
  before the first module that reads its images (none, in these runs: the end of the tick).
- **The camera model reads the render from device memory**: the images no longer go to host
  memory and back (labels and copies 1.4 → 0.45 ms on the GPU for the sedan's camera).
- **MuJoCo leaves far kinematic bodies out of its broad phase** (rigid_body.md): it makes no
  contact between two bodies without degrees of freedom but sorted and paired them anyway, about
  half of the town's step. **A world without degrees of freedom skips MuJoCo's step** (the town
  without a robot): only the time and the kinematic bodies' poses change.
- **Bus commits touch less memory**: a module's outputs commit together, in runs; commit times in
  their own array; previous values kept only where a reader uses them. In the town 2700 body
  signals commit every tick.
- **The thermal camera remembers each surface's apparent temperature** by its temperature and
  emissivity (0.15 → 0.01 ms a frame for the sample AMR's 138 surfaces).
- **The thermal camera's image is drawn on the GPU** after the render (`thermal_camera.slang`,
  sensors.md §2), measured after the table above: the sample AMR's camera 4.0 → 3.5 ms a frame,
  RTF 12.1 → 13.1 (the same build with `LAGRANGE_THERMAL_CAMERA=cpu` and without, twice each).

Measured and left alone: allocation per step is small (an `LD_PRELOAD` malloc counter: under one
per tick in the town, about four per tick with the sedan's camera, radar and lidar); the humans
(0.04 ms per 10 ms step for 36 people) and the traffic (0.015 ms) are a few per cent of the town.

What is left, largest first:

- The camera's GPU work, 3 to 12 ms a frame in the town, set mostly by the laptop GPU's clocks
  (a camera at 10 Hz of simulated time leaves it idle in between, and it runs slow): the same
  frame took 2.9 ms of GPU time in one run and 10.3 ms in the next. The CPU lidar after it in the
  tick hides 2 to 4 ms of it.
- MuJoCo with a robot among many kinematic bodies: each step still runs its kinematics, velocities
  and `mj_rnePostConstraint` over all 664 bodies (0.075 ms in the town with the sedan).
- The scheduler's own work in the town, about 15 µs a tick: filling 1300 input slots and
  committing 2700 outputs, mostly memory traffic.
- The CPU lidar, 1.7 to 4 ms per scan.

### 4.2 Third round (2026-11-12, day)

Same scenarios and method (`--no-ros --rtf 0`, 20 s, the median of the 1 s windows after the
first four), the build before (71880eb, the second round's state) and after run in the order
before, after, after, before per scenario; load average 1.4 to 8 on 20 threads. Camera runs are
set by the laptop GPU's clocks (below), so for them the table also gives the real-time factor
without the camera's time (each window's factor divided by one less the camera's share of it).

| Run | Before (RTF) | After (RTF) | Without the camera, before → after | What changed it |
|---|---|---|---|---|
| Sample AMR, warehouse (`--lidar cpu`) | 27.3, 29.2 | 31.3, 31.5 | | caster hierarchy, lidar noise |
| Quadcopter, open field | 61.9 to 65.9 | 59.3 to 63.9 | | 3 to 5 % slower: bus bookkeeping (below) |
| Thermal camera AMR, 6 people | 12.3, 17.4 | 12.7, 12.8 | 21.8, 21.3 → 25.8, 25.8 | caster hierarchy, lidar noise |
| Sedan, city block | 10.4, 12.5 | 11.1, 11.7 | 19.6, 25.7 → 28.6, 32.2 | as above |
| Sedan in the town | 3.0, 3.0 | 3.6, 3.0 | 5.3, 5.7 → 7.0, 5.8 | as above, MuJoCo writes |
| Sedan in the town, `--lidar gpu` | 2.6, 2.5 | 3.7, 3.4 | 2.9, 2.9 → 4.5, 4.0 | as above, BVH refit |
| Town, no robot, 24 cars, 36 people | 16.4, 16.6, 16.9 | 21.7, 22.0, 22.9 | | bus, MuJoCo writes |

Per step (medians): the AMR's top lidar 1.03 → 0.56 ms; the sedan's roof lidar 2.1 → 0.97 ms
(city) and 3.1 → 2.0 ms (town), its radar in the town 1.0 → 0.8 ms; MuJoCo in the town 0.040 →
0.028 ms, with the sedan 0.096 → 0.087 ms. Every signal is the same bit for bit before and after:
`LAGRANGE_DETERMINISM_HASHES=1 lagrange determinism` prints every state hash, and the before and
after builds give the same 201 hashes over 20 s for `amr_sample_urdf`, `sedan_city`,
`quad_x500_hover`, the sedan in the town and the town without a robot (and each passes
`lagrange determinism`).

What each change does:

- **The ray caster builds the hierarchy of shapes that never moved once.** `ShapeCaster` rebuilt
  Embree's hierarchy over every shape whenever one moved: each lidar scan of a robot in a world
  rebuilt the world's (about 10 % of the main thread with the sedan in the town). Shapes that have
  never moved now sit in a scene built once, the moved ones in a second scene rebuilt when they
  move; a top scene instances both.
- **The lidar draws its range noise in parallel**: a Gaussian per beam on one thread was about a
  millisecond of a 32 768-beam scan. Each beam has its own Philox stream, so threads change
  nothing.
- **Held inputs are copied only after a new commit**, and **commits copy only written signals**
  (architecture.md §6): in the town MuJoCo's 655 pose-command inputs, written every 10 ms, were
  copied every 1 ms (input preparation 9.5 → 3.5 % of the main thread).
- **MuJoCo writes a kinematic body's unchanged values once** (rigid_body.md): its pose when its
  command changes, its twist and contact force when they change, its zero acceleration once.
- **The GPU lidar refits its hierarchy** instead of rebuilding it every scan (rebuilt when the
  inner boxes' area has grown by a quarter, or after 30 scans).

Measured and not fixed:

- **The camera, and the GPU lidar, are set by the GPU's clocks.** With `nvidia-smi` sampled once a
  second during the sedan in the town: for the first 10 s the GPU stays at 2010 MHz (P0) and the
  front camera takes 3.0 ms a frame; then the driver drops it to 600 to 990 MHz (P5) at 30 to 40 %
  utilisation and the same frame takes 12 to 13.5 ms, for the rest of the run. The driver keeps the
  utilisation in a band by lowering the clock, so batching the same work differently does not
  raise it; only overlapping the render with the next ticks (a frame of latency) or a GPU held at
  its clocks would. Pinning clocks needs root (`nvidia-smi -lgc`); whether the PowerMizer
  "prefer maximum performance" setting holds them for a Vulkan compute client on Wayland is
  TODO(verify). Not changed here.
- **Why Tier 1 (`--lidar gpu`) is not faster.** Timed per phase: building the shapes and the tree
  is 0.02 to 0.05 ms; the dispatch with its wait is the rest. The sample AMR's lidar (14 400 beams,
  13 shapes, trivial work) takes 1.1 ms per dispatch, and a second dispatch straight after the
  first as long (spinning on the fence instead of sleeping: 0.95 ms), at 400 to 870 MHz: the GPU is
  latency-bound at low clocks, where the CPU caster over 20 threads takes 0.56 ms. In the town the
  lidar's and the radar's dispatches also wait behind the camera's render in flight (6.4 ms for
  1 350 radar beams as for 32 768 lidar beams). `--lidar auto` keeps the CPU caster.
- **The bus bookkeeping costs the quadcopter 3 to 5 %** (62.6 → 59.9 and 65.7 → 63.1 RTF in two
  sets of four runs): its 100 µs ticks commit small signals that are all written every step, so
  the dirty flags and the per-slot checks are pure overhead there (merging nearby written signals
  into one copy took back part of it). Left as it is; worth a look.
- MuJoCo with a robot among the kinematic bodies: the broad-phase culling already applies with a
  robot present (it is per moving tree), so what is left is MuJoCo's per-body passes
  (kinematics, velocities, `mj_rnePostConstraint`) over all 664 bodies, which a backend cannot
  restrict to a subset of an `mjModel`.

## 5. Start-up and Python throughput (2026-11-12)

The target (§13.4 of the project rules) is under 5 s from a cold start to the viewer's first frame
with a sample robot on a warm disk. Every sample is far below it; what is left is mostly the GPU.

`LAGRANGE_TIMING=1` makes the server (and the Python `Simulation`) print on stderr how long each
load stage took (`compose`: the robot and world files read into world tables; `ros`; `streaming`;
`lidar`; `cameras`; `build`: the modules made and wired), each module's `declare` and `init`, and
each step of making the camera scene (`gpu device`, `pipelines`, `meshes`, `textures`). Timing is
never part of the state.

Measured on the same laptop as §4 (i9-13900H, RTX 4060 Laptop GPU, RelWithDebInfo), **shared with
other builds and test runs (load average 8 to 23)**. Before is `night/integration` as built, after is
this work; each is run in a clean environment, alternating, five times for the load and four
for the first frame. The tables show medians. The GPU was awake: see the cold GPU below.

| Run | Server load, before → after (s) | First frame with `lagrange`, before → after (s) |
|---|---|---|
| Sample AMR, warehouse | 0.12 → 0.07 | 0.81 → 0.42 |
| Sedan, city block (camera, radar, lidar) | 0.80 → 0.47 | 1.65 → 0.93 |
| Sedan in the town | 0.81 → 0.52 | 1.73 → 1.30 |
| Quadcopter, open field | 0.13 → 0.15 | 1.02 → 0.67 |
| Tiled town, `--spawn 1036,1036` (16 tiles at start) | 0.15 → 0.16 | 1.00 → 0.58 |

The load is `lagrange_server --no-ros` from exec until its socket listens. The first frame is
measured as the `lagrange` command starts things, with ROS: before, the server until it listened
and then the viewer (`--headless --frames 1`); after, both at once.

| Rebuild | Before (s) | After (s) |
|---|---|---|
| Spawn an AMR into the sedan's city run (`Simulation.spawn`), three spawns and a delete | 0.70 to 0.80 each | 0.09 to 0.15 each |
| Apply a profile (quadcopter hover), a spawn without cameras | 0.02 to 0.05 | unchanged |

What changed:

- **The viewer starts beside the server.** `lagrange` waited for the server's socket and only
  then started the viewer, so the first frame came after both start-ups, one after the other. Now
  both start at once. Until its first connection the viewer's client tries every 50 ms (for 20 s),
  not after an exponential backoff that has grown to seconds by the time the server listens. This
  saves 0.3 to 0.7 s on every run.
- **A rebuilt run keeps its camera device.** Spawning or deleting a robot, or a profile that
  rebuilds the run, made a new Vulkan device and decoded the realistic textures again for the
  cameras. `gpu::RasterScene::create(..., reuse)` now shares the previous scene's device, queue
  lock and textures. The meshes are uploaded again because they changed.
- **The camera model's ray table is computed in parallel.** It holds the undistorted ray and the
  vignetting of every output pixel: 0.26 to 0.36 s on one core for the sedan's 960 × 540 camera,
  now 20 to 50 ms. Each row's reach is kept separately and combined in order, so the table is
  the same on any number of cores.
- **The sixteen surface photographs are decoded at once** (one thread per JPEG): 100 to 175 ms
  became 50 to 90 ms, for camera runs and for the viewer's realistic look.
- Python: `Server::read` finds signals in a name index instead of walking every signal, and
  `commandJoint` keeps the run's drive modes instead of working them out on every call.
  `LagrangeEnv` builds its observed signal names once, and `image()` no longer lists every signal
  before each frame. The API is unchanged.

Results are unchanged. `lagrange determinism` passes (AMR, quadcopter). Python `state_hash()`
after the same steps is equal before and after for the AMR, the quadcopter, the sedan and a
spawn sequence. The `LagrangeEnv` episodes give identical observations and hashes.

**The cold GPU.** On this laptop the NVIDIA GPU powers down when idle. The first Vulkan device
after that takes 2.1 to 2.3 s instead of 0.15 to 0.28 s; it was seen in about one run in five
here, after a pause. The server's cameras and the viewer each pay it. Starting the two together
lets them pay it at the same time instead of one after the other. Nothing else in a start-up
comes close. A `VkPipelineCache` on disk was considered and not added: the camera pipelines
take 0.3 to 1.3 ms to build, because the NVIDIA and Mesa drivers keep their own shader caches.

**Python throughput** (sample AMR with `joint_control`, CPU lidar, medians of 2000 calls):

| Call | Cost |
|---|---|
| `read(name)` | 0.1 to 0.25 µs |
| `command_joint(...)` | 0.1 to 0.35 µs |
| `signals()` (95 signals) | 18 to 38 µs |
| One 1 ms tick through `step_until` | 16 to 46 µs, the same whether called per tick or once for many |
| `LagrangeEnv.step`, 20 ms steps | 1240 → 1420 per second (before → after, same run) |
| `LagrangeEnv.step`, 2 ms steps | 10 900 → 11 100 per second |
| `ParallelEnvs`, 1 env, 2 ms steps | ~9 900 per second: the pipe to the worker costs ~45 µs a step |

The Python layer adds about 1 % to an RL step. The rest is the simulation itself (§4): the AMR's
two lidars, MuJoCo and the electrical network. Under the shared load, `ParallelEnvs` with four
workers gave 1.5 to 1.8× one worker at 20 ms steps. Each worker's CPU lidar already uses every
core, and pinning workers to separate cores was slower here.
