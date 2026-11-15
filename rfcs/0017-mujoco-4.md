# RFC 0017: MuJoCo 4.x for the MuJoCo backend

| | |
|---|---|
| Status | Draft (for the owner) |
| Created | 2026-11-15 |
| Affects | `modules/rigid_body/mujoco_backend/`, `io/importers/` (CMake and package.xml only), `packaging/`, `docs/` (release notes, comparison) |
| Builds on | RFC 0007 (the 1.x freeze), spike [0.2-C1](../docs/plan/spikes/0.2-C1-mujoco.md), spike [mujoco-4](../docs/plan/spikes/mujoco-4.md) |

## Summary

Move Lagrange's MuJoCo from 3.12.0 to 4.0.1 or a later 4.x, preferably by taking
`ros-lyrical-mujoco-vendor` once it ships ≥ 4.0.1. No source changes are needed in the backend
or the MJCF importer. The motivation is speed with many contacts: MuJoCo 4.0.1 steps the 200-box
pile with Lagrange's options (elliptic cone, `implicitfast`) at RTF 0.36 against 0.18 on 3.12.

## Motivation

- The comparison page lists "uses an older MuJoCo" and "slow with many contacts" as weaknesses.
- 3.13 to 4.0.1 bring solver speed-ups for elliptic cones, a rewritten plane–mesh collider and
  more stable convex collisions, all used by the backend.
- Lyrical's `mujoco_ros2_control` moves with `mujoco_vendor`; staying on the same MuJoCo keeps
  side-by-side comparisons honest.

## Proposal

1. **Source of MuJoCo.** Keep the rosdep key `mujoco_vendor`; require a version that ships
   MuJoCo ≥ 4.0.1 (not 4.0.0: its sparse Cholesky bug). If that has not reached Lyrical by the
   owner's chosen date, fall back to a Lagrange vendor package (`vendor/lagrange_mujoco_vendor`,
   pinned tag and SHA-256, its own prefix) — see open question 1.
2. **Build switch.** Keep the cache variable `LAGRANGE_MUJOCO_PACKAGE` (default
   `mujoco_vendor`) from the spike so developers and CI can build against any MuJoCo 4.x install
   ahead of the packages. The backend and the importer must always use the same package.
3. **Before the switch merges:**
   - explain or fix `Water.ASmallFloatRidesLongWaves` (heaves 0.1107 m for 0.1 m waves on 4.0.1);
   - run the full local check (all packages, sanitizers, python, examples, `tools/ci/realism.sh`)
     on 4.x and list every changed number;
   - rerun `docs/release/comparison/perf/run_all.py` with the prebuilt 4.x binary the vendor
     ships, and update `docs/validation/comparison.md`;
   - check the IMU of a kinematic body (`mj_objectAcceleration` now reports gravity for static
     and mocap bodies, 4.0 changelog item 46).
4. **Backend overhead (independent, can land first).** Profile the backend on the box pile and
   compute `mj_rnePostConstraint` and per-body contact forces only when their signals are read.
   On 4.0.1 the backend's own work is 44 % of the step on that scene (15 % on 3.12).
5. **Release.** A 1.x minor release. RFC 0007 freezes formats, not trajectories: schemas,
   plugin ABI, protocol, profiles and recordings do not change (the MuJoCo state blob in
   `saveState` keeps its size for rigid models). The release notes say that runs with contacts
   take different bits than on 3.12, that state hashes recorded with 1.x ≤ this release do not
   carry over, and which MJCF constructs MuJoCo 4 refuses (`composite`, the `cable` plugin,
   `flexvert`, duplicated includes, height fields under 2 × 2).

## Alternatives

- **Stay on 3.12** until Lyrical's vendor moves on its own, doing only step 4. Cheapest; keeps
  the slower solver.
- **Own vendor package now.** Fastest route to 4.0.1, but two libmujocos must never meet in one
  process (a user's `mujoco_ros2_control` beside Lagrange's ros2_control host would), and
  Lagrange then owns MuJoCo updates.
- **Jolt as the default backend.** Already 2.38 on the box pile, but loses pendulum energy
  (comparison.md) and is not the reference engine.

## Open questions

1. A date after which Lagrange ships its own vendor package if `mujoco_vendor` has not moved?
   Proposed: none; wait, and comment on pal-robotics/mujoco_vendor#18 asking for 4.0.1.
2. Should the conformance suite pin a MuJoCo version, so a vendor update cannot change physics
   unnoticed? Proposed: the backend logs `mj_versionString()` at start, and a test checks
   `mjVERSION_HEADER == mj_version()` (header and library from the same release).
