# Spike: MuJoCo 3.12 → 4.x

Date: 2026-11-15. Question: should Lagrange's MuJoCo backend (`modules/rigid_body/mujoco_backend`,
and the MJCF importer in `io/importers`) move from MuJoCo 3.12.0
(`ros-lyrical-mujoco-vendor`, spike [0.2-C1](0.2-C1-mujoco.md)) to MuJoCo 4.x? The reason to ask:
[comparison.md](../../validation/comparison.md) measured the 200-box pile at RTF 0.166 in
Lagrange on MuJoCo 3.12 and 0.458 in standalone MuJoCo 4.0.1 (with MuJoCo's defaults).

**Recommendation: yes, move to MuJoCo ≥ 4.0.1, through `mujoco_vendor` once it ships 4.0.1 or
later, as a 1.x minor release under an RFC ([0017](../../../rfcs/0017-mujoco-4.md), Draft).**
Lagrange builds against 4.0.1 with no source change, the conformance suite and the backend's
validation tests pass, runs stay bit-identical run to run, and the contact-heavy case steps
about twice as fast in MuJoCo itself. One server test fails (a floating box heaves 11 % too
far), and Lagrange gains much less than MuJoCo on the box pile (1.3× against 2.0×), so the move
needs that test understood and a profile of the backend's per-step work. Not 4.0.0: its sparse
Cholesky bug (fixed in 4.0.1) gives the Newton solver wrong Hessian factors.

## 1. What changed between 3.12.0 and 4.0.1

Versions and dates are from MuJoCo's GitHub releases (`gh api repos/google-deepmind/mujoco/releases`)
and `doc/changelog.rst` at tag `4.0.1` (commit `4ed093a9f06b`). Six releases came after 3.12.0
(2026-08-20): 3.13.0 (2026-09-08), 3.14.0 (09-22), 3.15.0 (10-05), 3.16.0 (10-14), 3.17.0
(10-22), 4.0.0 (11-06), **4.0.1 (2026-11-11), the latest** on the day of this spike. Licence:
still Apache-2.0 (GitHub's licence API for the repository).

### API and ABI breaks that touch Lagrange

There are no ABI guarantees between MuJoCo releases: every release has its own soname
(`libmujoco.so.3.12.0`, `libmujoco.so.4.0.1`), so everything that links it must be rebuilt with
it. `mjVERSION_HEADER` is `4000001`.

What Lagrange uses (`mj_step1/2`, `mj_rnePostConstraint`, `mj_contactForce`, `mj_get/setState`
with `mjSTATE_INTEGRATION`, `mj_fullM`, `mj_objectVelocity/Acceleration`, `mjs_add*` for bodies,
joints, geoms, meshes, height fields, equalities and excludes, `mj_loadXML`, `mj_id2name`, …)
compiled unchanged against 4.0.1. Breaks read from the changelog and checked against our code:

| Change | Version | Effect on Lagrange |
|---|---|---|
| An asset added through the `mjSpec` API is no longer named after its file; an unnamed mesh, height field or texture is a compile error | 3.16 | None: the backend names every `hull_<c>` and `hfield_<c>` |
| `mjs_setName` on a duplicate name keeps the old name; `mjsElement.signature` is 0 until compiled | 3.14 | None found |
| `mjSTATE_PLASTIC` added to `mjSTATE_PHYSICS`, so to `mjSTATE_INTEGRATION` | 4.0 | Zero-sized for rigid models: `Server::saveState` blobs keep their size. The bits inside differ between versions (below) |
| `mjNEQDATA` 11 → 12 (weld relative pose) | 3.17 | ABI only; we read no `eq_data` |
| `mj_rnePostConstraint`: weld torque fixed; spatial tendon forces now included | 3.14 | Changes `cfrc_int`/`cfrc_ext` for welds and tendons. We make no welds or tendons; `joint/<j>/reaction` unchanged in the conformance cases |
| `mj_objectAcceleration` on static and mocap bodies reports proper acceleration (gravity), not zero | 4.0 | A body we read through it while kinematic (traffic agents, humans) would now read +g. Not seen in tests; TODO(verify) the IMU on a kinematic body |
| `mj_readCtrl` signature; resource-provider and decoder renames; `mjs_addTexture(spec, def)`; `mjrf_*` renames | 3.14–3.16 | Not used |
| MJCF: `composite` and the `cable` plugin removed; `flexvert` removed; files included twice rejected; height fields with < 2 rows or columns rejected; `sidesite` semantics changed; flex elastic constants fixed | 4.0 | User MJCF files using these stop loading or move differently through `lagrange_import`. The sample MJCF (`amr_diffdrive.xml`) uses none |
| Solver termination scaled by the trace of the current inertia instead of `stat.meaninertia` (`meaninertia` deprecated) | 3.17 | Different iteration counts, so different bits, in every contact scene |
| Newton with elliptic cones refactorises the Hessian in one go when cheaper: sliding contacts 1.4–2×, slowest steps 3–4× | 3.13 | The speed-up measured below (Lagrange uses elliptic cones and Newton) |
| `mju_boxQP` incremental Cholesky (1.4–2.9×); Plane–Mesh collider rewritten; nativeccd/multiccd stability | 3.13–4.0 | Changes contacts on mesh and convex colliders (all our hulls) |

### Determinism

MuJoCo still documents run-to-run determinism on one build; nothing in the changelog weakens
it for rigid bodies. Two 4.x items do not keep full state: the experimental `ipc` flag (3.14)
and plastic state outside `mjSTATE_PLASTIC`. Lagrange uses neither.

## 2. Build and measurements

Build: MuJoCo 4.0.1 from source (tag `4.0.1`, GCC 15.2, Release, LTO and AVX on, samples,
simulate and tests off) into `scratch/mujoco-install` of the spike worktree: 53 s at `-j6`,
22 MB installed. The spike patch (separate commit) adds a cache variable
`LAGRANGE_MUJOCO_PACKAGE` (default `mujoco_vendor`) to `lagrange_rigid_body_mujoco` and
`lagrange_importers`; `-DLAGRANGE_MUJOCO_PACKAGE=mujoco` takes MuJoCo's own CMake package. Both
give the target `mujoco::mujoco`. The two packages must agree: the server loads both, and two
libmujocos in one process would interpose each other's symbols. Two overlays were built from
the same source (`origin/dev` 24d8c07) on the shared 1.3.0 install: `old` (vendor 3.12.0,
prebuilt by Google, GCC 14.2 per its `.comment`) and `new` (4.0.1 as above); `ldd` checked
which libmujoco each binary loads.

The machine: i9-13900H, 20 threads, other agents building at times. Load averages are given
per measurement.

| Check | MuJoCo 3.12.0 | MuJoCo 4.0.1 |
|---|---|---|
| `test_mujoco_validation`, `test_mujoco_backend` | pass (CI) | 9/9, 14/14 pass |
| `lagrange_physics_conformance` (mujoco, jolt, jolt_abi) | 39/39 | 39/39 |
| `test_server` | not rerun here: the build dir was deleted for disk space before it ran. `Water.ASmallFloatRidesLongWaves` passes on the 1.3.0 build (3.12) | **213/214**: `Water.ASmallFloatRidesLongWaves` heaves 0.1107 m for 0.1 m waves (tolerance 0.01) |
| `lagrange determinism` amr_sample_urdf, sedan_city, quad_x500_hover (10 s each) | 101/101 hashes equal, each | 101/101 hashes equal, each |
| State hash, sample AMR in the warehouse, demo drive, 10 s | 14074422418178036292 | 13035836972387799478 |
| AMR base pose at 1, 2, … 10 s | — | identical to 1e-9 in every component (position and quaternion rounded to 9 decimals) |

So the hashes change, as expected, but in the AMR run not in anything a user sees: the pose
over 10 s is equal to the ninth decimal. The difference is in state the hash covers beyond the
pose, probably the solver's warm start (not checked). The pile of boxes is chaotic and diverges.

### The 200-box pile (5 s at 1 ms, `make_scenarios.py`'s `boxes.xml`)

Standalone: a C program (`scratch/bench/boxbench.c`) loads `boxes.xml`, optionally sets
Lagrange's options (elliptic cone, `implicitfast`) and calls `mj_step`, linked against each
library. Lagrange: `bench_lagrange.py` with the comparison's `profile_boxes.yaml` (MuJoCo
backend, other domains off). Old and new back to back; RTF = simulated / wall time.

| Run | Load (1 min) | 3.12.0 | 4.0.1 | 4.0.1 / 3.12 |
|---|---|---|---|---|
| Standalone, Lagrange's options (round 1) | 1.5–3.4 | 0.183 | 0.362 | 1.98× |
| Standalone, MuJoCo's defaults (round 1) | 3.3 | 0.287 | 0.413 | 1.44× |
| Standalone, Lagrange's options (round 4) | 5.2–6.7 | 0.183 | 0.365 | 1.99× |
| **Lagrange, MuJoCo backend** | 5.1–6.9 | 0.156 | 0.204 | **1.31×** |

Rounds 2 and 3 ran while the overlays compiled (load 6–10) and are left out (0.159 and 0.218
for Lagrange's options). Both versions find 851 contacts at 5 s with Lagrange's options.

What this says:

- MuJoCo 4.0.1 steps the pile with Lagrange's options at twice 3.12's speed. The likeliest
  cause is the 3.13 elliptic-cone Newton refactorisation (the changelog's 1.4–2× for sliding
  contacts); bisecting the six releases was not done.
- With 4.0.1 the elliptic cone costs little: 0.365 against 0.413 with the pyramidal default
  (3.12: 0.183 against 0.287).
- **Lagrange keeps only part of it** (0.156 → 0.204). Its MuJoCo step costs more than `mj_step`
  on its own: split `mj_step1`/`mj_step2`, `mj_rnePostConstraint` every step for the IMU and
  `joint/<j>/reaction`, a `mj_contactForce` loop over every contact for `body/<b>/contact_force`,
  and writing every body's signals. On 3.12 that overhead was 15 % of the step; on 4.0.1 it is
  44 %. A profile of the backend with 851 contacts is the next step (and is worth doing on
  3.12 too); computing contact forces and `rnePostConstraint` only when something reads them is
  the obvious candidate. TODO(verify) by profiling. *Profiled on 3.12 afterwards
  ([benchmarks.md §4.4](../../design/benchmarks.md)): Lagrange's own work is about 2 % of the
  pile's step; the gap to `boxes.xml` comes from the model Lagrange builds (an extra ground plane,
  the ground box as a body of its own), in which the pile falls differently.*

Not measured, and why:

- **The AMR scenes of `run_all.py`** (physics only, sensors, camera): no time left in the time
  box after the disk ran short. The AMR's pose is identical for 10 s and its step has a handful
  of contacts, so no large change is expected; the AMR determinism run took 1.2 s wall time for
  2 × 10 s on both versions. TODO(verify) with `run_all.py` before any release note.
- **The prebuilt 4.0.1 binary** (the one `mujoco_vendor` would ship): two downloads of the
  release archive broke off. The standalone numbers above use a GCC 15 source build, while the
  3.12 vendor library is Google's prebuilt; part of the difference may be the compiler.
  TODO(verify) with the release tarball (`mujoco-4.0.1-linux-x86_64.tar.gz`, SHA-256
  `a511a1cc9122bf1b9cf404d16d3ff410c94b544932d40a29085142a9438a9f7e` per its `.sha256` file).
- **The water test**: why the float heaves 11 % more. Candidates: the 3.17 solver termination
  change, or the inertia-trace scaling with the hydrodynamics' added-mass forces. It decides
  whether the test's tolerance or the model is wrong.
- Sanitizer builds (MuJoCo 4.0.1's `mjsan.h` under GCC with ASan; the existing include-guard
  workaround should still apply), arm64, the viewer and `lagrange_python`'s tests, the
  examples and realism suite.

## 3. How it could ship

- **Lyrical's vendor package is still 3.12.0.** rosdistro `lyrical/distribution.yaml` releases
  `mujoco_vendor` 0.1.2-1; its `CMakeLists.txt` pins `MUJOCO_VERSION "3.12.0"` (tags 0.1.0 to
  0.1.2). The apt package on this machine is 0.1.0. Upstream (pal-robotics/mujoco_vendor) has an
  open PR #18 (2026-11-09) "Bump MuJoCo to 4.0.0", which downloads Google's prebuilt release
  archives like the current version; it notes 4.0.0 no longer ships `libelasticity.so`. It pins
  4.0.0, which has the sparse Cholesky bug; Lagrange needs ≥ 4.0.1. When it would reach a Lyrical
  sync is not known (TODO(verify)).
- 0.1.2 renamed the vendor's lodepng and simulate targets, with deprecated aliases;
  `mujoco::mujoco`, the only one Lagrange uses, is unchanged.
- **Options:**
  1. *Wait for `mujoco_vendor` ≥ 4.0.1 in Lyrical* (preferred). No packaging work for Lagrange:
     the debs depend on `ros-lyrical-mujoco-vendor` as today and are rebuilt against it (they
     must be: the soname changes). `mujoco_ros2_control` in Lyrical moves at the same time, so
     the comparison baseline stays shared. A comment on PR #18 asking for 4.0.1 costs nothing.
  2. *Lagrange's own vendor package* (`vendor/lagrange_mujoco_vendor`, like `jolt_vendor`):
     ExternalProject from the `4.0.1` tag or the release tarball with its SHA-256, installing
     under its own prefix. It must not be named `mujoco_vendor`, and a process must never load
     both libmujocos: anything in the same process that links Lyrical's `mujoco_vendor` (e.g. a
     user's `mujoco_ros2_control` plugin next to Lagrange's ros2_control host) would then be a
     conflict. More debs to build (about a minute), and Lagrange owns security updates of MuJoCo.
  3. Stay on 3.12 and work on the backend's overhead (§2) first: it is the part of the box-pile
     gap Lagrange controls, and helps on either version.
- The spike patch's `LAGRANGE_MUJOCO_PACKAGE` switch lets developers and CI build against a
  local 4.x now, without changing what the debs depend on.

## 4. Risks

- **Bits change in every contact scene.** Recorded state hashes, `--replay` checkpoints and
  `Server::saveState` blobs from 3.12 runs load (same size) but continue differently. RFC 0007
  freezes formats, not trajectories; the release notes must say so. Determinism within one
  build holds (above).
- **One server test fails** (water heave) until understood; there may be more in suites not
  run here (python, examples, realism).
- **MJCF users** with `composite`, the `cable` plugin, `flexvert`, duplicated includes or
  degenerate height fields: `lagrange_import` refuses those files after the move. The importer
  should report it in words.
- **Being early:** 4.0.0 shipped a solver bug that 4.0.1 fixed five days later. Waiting for the
  vendor (option 1) also waits out the first point releases.
- **Disk and memory:** none of note: 22 MB installed, under a minute to build.
