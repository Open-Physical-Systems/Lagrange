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
