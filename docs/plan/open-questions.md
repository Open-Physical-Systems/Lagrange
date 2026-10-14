# Open questions and challenges to the plan

Status: for decision. Part 1 lists decisions needed, each with a recommendation. Part 2 lists
flaws found in the reference examples of CLAUDE.md §11–§12 and in CLAUDE.md's own statements,
each with a proposed fix. Nothing here has been changed silently: the design documents use the
recommended option and say so, and CLAUDE.md is untouched.

Evidence references (V…) point to `docs/plan/verification.md`.

## Decisions recorded on 2026-10-13

| Question | Decision |
|---|---|
| Q17 project name | **Lagrange**, organisation Open Physical Systems (`Open-Physical-Systems/Lagrange` on GitHub). Logos are in `docs/imgs/` |
| Q19 hosting | The repository lives in a GitHub organisation. Team size and a self-hosted GPU runner are still open |
| Q4 v0.1 without OpenUSD | Accepted: the mock server converts URDF directly to protocol scene messages; USD arrives in v0.2 |
| Q15 mock server language | Python for v0.1 only. The shipped product is C++: replay moves into the C++ server in v0.2 and the Python server stays as a test tool |
| Q20 protocol (RFC 0003) | Accepted: Protobuf over a Unix socket or TCP, `memfd` shared memory, Zenoh deferred |
| Q21 window library (RFC 0006) | Accepted: SDL3 |
| Q9 first physics backend | Accepted: MuJoCo in v0.2, Jolt next, PhysX in v0.6. No in-house rigid-body solver (invariant 12) |
| Q3 thin electrical slice in v0.2 | Accepted as a stretch item |
| Q2 radiation | Accepted: contract and `zero` substitute from v0.2; full module as v0.8 or post-1.0 |
| Q8 FMUs | Accepted: built-in C++ models are the default; FMUs are an extension path. Accepting FMI 2.0 is still open |
| Q13 Node-free tooling | Accepted: commitizen and mdformat |
| Q16 sim-vs-real data | Acknowledged; hardware and publication terms still to be named |
| Commit attribution | Commits carry only the author's `Signed-off-by`; no tool co-author trailers |

## Part 1 — Decisions needed

### Highest priority (block v0.1 or change its shape)

**Q4. How does the v0.1 mock server show a URDF without a simulator or C++ OpenUSD?**
OpenUSD is not packaged for Ubuntu 26.04 (V2) and is the largest build risk. The v0.1 viewer
never needs USD because it only speaks the protocol.
Options: (a) the Python mock server converts URDF directly to protocol scene messages, and USD
arrives in v0.2; (b) the mock server uses the `usd-core` pip wheel (Python 3.14 support reported
from v26.05, secondary source); (c) build `openusd_vendor` already in v0.1.
**Recommendation: (a).** It keeps v0.1 free of the USD build and of pip-installed binaries next
to rosdep. Cost: a small URDF-to-scene path in the mock server that the real importer replaces
in v0.2. The scene tree then shows the URDF link hierarchy presented as prim paths.

**Q15. Mock server language and its future as "replay mode".**
CLAUDE.md says the mock server is Python at first, is production code, and becomes replay mode.
A Python replay server and a C++ simulation server would be two implementations of the protocol
server side.
**Recommendation:** keep Python for v0.1 (fast to write, and a second-language implementation is
a good protocol conformance check). In v0.2, move replay into the C++ server as a mode that uses
replay substitutes for every domain, and keep the Python server as a test tool. Decide finally in
issue 0.2-E8.

**Q20. Protocol encoding and transport (RFC 0003).**
**Recommendation:** Protobuf (proto3) with raw little-endian arrays in `bytes` fields for bulk
data; Unix domain socket locally and TCP remotely; shared memory through `memfd` passed over the
Unix socket; Zenoh deferred. Reasons are in RFC 0003.

**Q21. Window library (RFC 0006).**
**Recommendation: SDL3** (3.4.2 in apt). Reasons are in RFC 0006.

**Q13. Node-free developer tooling.**
commitlint and markdownlint-cli2 need Node. CLAUDE.md §12.3 already prefers avoiding Node.
**Recommendation:** commitizen for commit messages and mdformat for Markdown (both Python, both
verified). release-please stays, because it runs only as a GitHub Action.

**Q17. Project name.** — **Decided: Lagrange.**
Identifiers use `lagrange` (packages `lagrange_core`, `lagrange_viewer`, …), the C ABI and CMake
prefix `LAGRANGE_`, and the C++ namespace `lagrange`. The repository is
`Open-Physical-Systems/Lagrange`. Still to check before the first release: existing ROS, PyPI and
apt package names that contain `lagrange`, and trademarks (TODO(verify)).

**Q19. Team size, repository owner and GitHub plan.**
Estimates in the roadmap are relative because the team size is unknown. Merge queue and
GitHub-hosted GPU runners depend on the owner type and plan (unverified).
**Needed from you:** who will work on this (and how many), whether the repository lives in an
organisation, and whether a self-hosted RTX runner is available. **Recommendation:** a GitHub
organisation (teams are needed for CODEOWNERS anyway) and one self-hosted RTX machine.

### Architecture and scope

**Q1. C++ standard.**
CLAUDE.md says "C++20, matching Lyrical's language baseline". On the installed distro, 48 CMake
exports say `cxx_std_17` and 2 say `cxx_std_20` (V3); the written policy could not be fetched.
GCC 15.2 supports C++20 fully. **Recommendation:** keep C++20 for this project (it is needed for
designated initialisers, `std::span`, concepts and `<=>` used in the interfaces), and change the
wording in CLAUDE.md to "C++20 (supported by Ubuntu 26.04's GCC 15; Lyrical's own packages mostly
build as C++17)". Public headers that ROS users include (the ros2_control plugin does not expose
any) should not force C++20 on downstream packages.

**Q2. Where does radiation go?**
The roadmap in CLAUDE.md has no release for the radiation domain, although it is a listed domain.
**Recommendation:** contract and `zero` substitute from v0.2 (so profiles are complete), full
module as v0.8 between huge worlds and 1.0, or post-1.0 if capacity is short. It has the weakest
link to the five robot classes.

**Q3. A thin electrical slice in v0.2?**
The wedge is multi-physics, but v0.2 as specified (AMR with Nav2) competes with Gazebo where this
project has no advantage, and the electrical domain only arrives in v0.3. The coupling risk (R3)
would also stay hidden until then.
**Recommendation:** add a small slice to v0.2 (issue 0.2-H7): battery ECM + averaged DC motor +
the ideal substitute, enough for a "brownout on the ramp" demo and the first coupling tests. Make
it a stretch item that does not block the v0.2 release.

**Q5. Scope of invariant 7 (deterministic CPU reference).**
Tier 0 cameras render through a software Vulkan device. Bit-identical images across Mesa
versions cannot be promised. **Recommendation:** state that invariant 7 covers module state and
numeric signals on a given machine and binary. Rendered sensor buffers are deterministic only
with the CPU ray-casting paths (Embree lidar, radar); raster camera images are excluded unless a
test proves them stable for the pinned Mesa. Update CLAUDE.md §3.7 accordingly.

**Q6. Unit system implementation.**
Units must be checked at wiring time across a C ABI, so they must exist at run time. A
compile-time library (mp-units 2.5.0 is packaged for Lyrical) cannot do that alone.
**Recommendation:** a small runtime `Unit` (dimension exponents + scale + quantity kind) as in
`architecture.md` §3. Modules may use mp-units internally if they wish; it is not part of any
interface.

**Q7. Codeless USD schemas.**
CLAUDE.md §6 has `schemas/generated/` with C++ and Python from `usdGenSchema`. Because
`core/stage` reads attributes generically into SoA pools, generated classes are not needed.
**Recommendation:** codeless schemas (`skipCodeGeneration = true`), keeping only
`generatedSchema.usda` and `plugInfo.json` as generated files. This removes a build step that
depends on USD's Python tooling. Revisit if external tools need typed C++ access.

**Q8. Dependence on FMUs.**
OpenModelica's FMI 3.0 export is experimental and FMU redistribution under OSMC-PL is unverified
(V16). **Recommendation:** built-in C++ models for the battery ECM and motors are the default and
the validated path in v0.3; the FMU host is an extension mechanism delivered in v0.3 with one
reference FMU whose license is clear. Consider accepting FMI 2.0 co-simulation FMUs too, since
most existing vendor FMUs are 2.0; this would be a change to the format stack and needs your
decision.

**Q9. First rigid-body backend and MuJoCo source.**
The brief lists PhysX first. PhysX is not packaged, has a custom build system, and its GPU part
needs CUDA. MuJoCo is packaged for Lyrical, uses `double`, and its parser is needed for MJCF
import. **Recommendation:** MuJoCo as the v0.2 backend and deterministic reference; Jolt second
(v0.3); PhysX in v0.6 for vehicles and GPU batches. Use `ros-lyrical-mujoco-vendor` if it exports
usable CMake targets, so that our package and `mujoco_ros2_control` share one MuJoCo; otherwise
Ubuntu's `libmujoco-dev`. Never both in one process.
Consequence: example profiles in CLAUDE.md §11.1 say `backend: physx`; they stay valid, but the
default in new examples is `mujoco`.

**Q10. Vehicle dynamics: own tyre and drivetrain models or the PhysX vehicle SDK.**
**Recommendation:** own models in `modules/vehicle_dynamics/` on top of scene queries, so they
work with every backend and respect the module boundary. Reconsider at v0.6 planning if effort
is too high.

**Q11. Schematic canvas library.**
imgui-node-editor's last release is from 2023 (last push March 2026); imnodes' last release is
from 2022. Both are MIT. **Recommendation:** imgui-node-editor behind a `Canvas` interface, with a
go/no-go check when `imgui_vendor` is built. The schematic needs only pan/zoom, nodes, pins and
links, so an in-house canvas on `ImDrawList` is a realistic fallback (estimated L).

**Q12. Dear ImGui Test Engine.**
Its license is custom (v1.04): free for software released publicly under an OSI license, among
other cases, but it is not itself an OSI license (V13). **Recommendation:** use it in CI only,
fetched at configure time behind a CMake option that is off by default, never committed to the
repository and never shipped in a package; record this in `dependencies.md`. If that is not
acceptable, panel tests fall back to driving ImGui through synthetic input events in our own
harness (more work, less coverage).

**Q14. Additions to the repository layout.**
The design needs two things not in CLAUDE.md §6: `gpu/scene/` (the GPU scene shared by the viewer
and the sensor renderer, fed from two sources) and a small always-on core module `wrench_sum`
(combines wrenches from several domains, required by the single-writer rule). Also
`schemas/world/` (tile schema) and `schemas/sensor/`. **Recommendation:** accept and update
CLAUDE.md §6.

**Q22. ROS topic browser without ROS in the viewer.**
**Recommendation:** the viewer does not link `rclcpp`. The server's ROS bridge sends the graph
through the protocol (`RosGraph`). The viewer then starts without a ROS environment and can
connect to a remote server. If you want the viewer to browse a ROS graph without our server,
that would need an optional ROS-linked helper process; not planned.

**Q23. Autonomous-car demo without Autoware on Lyrical.**
**Recommendation:** v0.6 acceptance does not depend on Autoware. Decide at v0.5 whether to
demonstrate with Autoware in a container on its own distro, or with another stack.

### Process and distribution

**Q16. Sim-vs-real data.**
Several validations need recordings from real hardware (AMR odometry, a BLDC bench, a
magnetometer near a known source, Wi-Fi RSSI in a building). **Needed from you:** which hardware
is available, and whether recordings can be published under an open license.
**Recommendation:** start with one dataset per release (v0.2: AMR square drive; v0.3: motor
bench), stored in the assets repository with a data sheet.

**Q18. Binary distribution.**
The ROS buildfarm may not accept long vendor builds or CUDA dependencies (unverified).
**Recommendation:** aim for the buildfarm for core packages; plan a project apt repository for
vendor-heavy and CUDA packages; decide after the v0.2 packaging trial (issue 0.2-H8).

**Q24. Jetson.**
JetPack is on Ubuntu 24.04 (V9). **Recommendation:** no Jetson support statement; revisit when a
JetPack for 26.04 exists. CLAUDE.md's TODO can be replaced by this finding.

**Q25. Documentation license.** Apache-2.0 for everything (simple) or CC-BY-4.0 for docs.
**Recommendation:** Apache-2.0 everywhere.

**Q26. Reference machine.** Budgets need a fixed machine. **Recommendation:** define it in v0.1
(proposal in `00-overview.md` §5) and use the self-hosted runner as that machine.

## Part 2 — Flaws found in the reference examples and in CLAUDE.md

Each has a fix that the design documents already use. Please confirm or reject.

**F1. `SignalSpec::name` is a `std::string_view` (§11.4).**
Signal names are built at run time from prim names (`joint/left_wheel/torque`), so a view has no
stable owner; returning `std::span<const SignalSpec>` with views into temporary strings is a
use-after-free waiting to happen, and the C ABI needs copied strings anyway.
**Fix:** `std::string name` in `SignalSpec`, plus `SignalId` handles for the hot path.

**F2. `IModule` cannot know its signals before it sees the world, has no error path, and reads
signals by name in `step()` (§11.4, §11.5).**
`inputs()`/`outputs()` depend on the joints in the stage, but `init()` is the first call that
receives the world. `void` returns leave only exceptions, which may not cross the C ABI.
`bus.read(j.torqueCommand)` is fine if those members are ids, but the example does not say so.
There is also no way to save and restore state, which determinism tests and rollback need.
**Fix:** add `declare(WorldView, ModuleConfig, SignalDeclarer&)`, `Status` returns,
`saveState`/`restoreState`, `StepContext`, and resolved `SignalId`s. `IdealTorqueSource::step`
keeps its shape. Details in RFC 0001.

**F3. Schema version suffix `_v1` (§3 invariant 11, §11.3).**
OpenUSD's schema versioning convention is `<Family>_<N>` (for example `ElectricalMotorAPI_1`),
and USD's registry parses family and version from that form (verified with v26.08 in the
OpenUSD spike, `docs/plan/spikes/0.2-B1-openusd.md`). `_v1` would not be recognised as a version by USD tooling. The usage example
in §11.3 also applies unversioned names.
**Fix:** use `<Family>_<N>`; say "version 1" in prose.

**F4. Schema attributes are `float` and carry units only in comments (§11.3).**
`float` loses precision for values such as capacities in coulombs or geodetic quantities and
forces conversions at load. Units in comments cannot be checked.
**Fix:** `double` for physical parameters; a registered `lagrangeUnit` metadata field on every
attribute definition, parsed by the loader. (Not customData: `usdGenSchema` and the schema
registry drop it; see the OpenUSD spike.)

**F5. Profile format gaps (§11.1).**
(a) No `schema_version`, so migration is impossible. (b) No `seed`, `deterministic`, `tier`.
(c) `dt` as a binary float cannot represent 0.0001 exactly; the scheduler needs exact rates.
(d) Substitute tokens (`zero`, `clear`, `ground_truth`) do not map visibly to the three kinds
(ideal, constant, replay) in §11.2. (e) "Every disabled domain names a substitute" needs domains
that are absent for a robot (vehicle dynamics for an arm) to be optional.
**Fix:** add the fields; parse `dt` as decimal text into integer nanoseconds; keep the short
tokens and define their kind/preset mapping (table in `architecture.md` §7); core domains are
required keys, class-specific domains optional. The two example profiles stay valid after adding
`schema_version: 1`.

**F6. `.clang-tidy` `HeaderFilterRegex: '^(core|gpu|…)/'` (§12.2).**
The regex is matched against header paths as the compiler resolves them, normally absolute, so
the `^` anchor matches nothing and header diagnostics are silently dropped [High].
**Fix:** `'.*/(core|gpu|modules|bridges|io|viewer|server|mock_server)/.*'`.

**F7. `.pre-commit-config.yaml` details (§12.3).**
(a) The ruff hook id is `ruff-check` in newer revisions of `ruff-pre-commit`, with `ruff` as the
older name [Medium; verify at pin time]. (b) No commit-message hook is listed although
Conventional Commits are required. (c) `clang-format` with `types_or: [c++, c]` does not cover
`.slang` files, which §12.4 says are formatted with clang-format. (d) No hook validates example
profiles against the JSON Schema although §11.1 requires it in CI.
**Fix:** as drafted in `repo-tooling.md` §5.

**F8. `.clang-format` include categories (§12.1).**
No category for quoted includes, ROS headers, or the viewer's third-party headers (Vulkan, SDL,
ImGui), so they all fall into the catch-all standard-library group.
**Fix:** added categories in `repo-tooling.md` §4.

**F9. Torque and energy have the same SI dimensions.**
§11.5 says a N·m output wired to a N input is refused, which a dimension check does. But a
dimension-only check accepts energy (J) wired into torque (N·m).
**Fix:** `Unit` carries a `QuantityKind` in addition to dimensions and scale.

**F10. Multi-quantity signal types.**
`Pose`, `Wrench` and a geodetic triple hold more than one physical quantity, but `SignalSpec` has
one `Unit`. **Fix:** the type defines the secondary quantities (`Wrench` = force in N then torque
in N·m); mixed arrays such as latitude/longitude/height are split into scalar signals (RFC 0001).

**F11. Single-writer rule versus several force sources.**
Aerodynamics, rotor aero and hydrodynamics all apply wrenches to the same body. With one writer
per signal they cannot all write `body/*/external_wrench`. **Fix:** per-domain wrench signals and
an always-on `wrench_sum` core module (Q14).

**F12. Statements in CLAUDE.md that verification contradicts or refines.**

| Section | Statement | Finding | Proposed wording |
|---|---|---|---|
| §1a, §5 | "C++20, matching Lyrical's language baseline" | Lyrical packages mostly export C++17 (V3) | See Q1 |
| §1a | "Dependencies not packaged … (likely OpenUSD, PhysX 5, Slang; TODO(verify))" | OpenUSD and PhysX: not packaged. **Slang is packaged** (2026.1.1). Also not packaged: Jolt, ImPlot, MCAP as a system library (ROS vendor exists), FMI libraries. Dear ImGui is packaged without docking (V1, V12) | Update the list |
| §1a | "Jetson … TODO(verify)" | JetPack 7.2.1 is on Ubuntu 24.04 (V9) | "Not supported: no JetPack on 26.04 as of 2026-10" |
| §7 | `--mixin asan` | Mixins need per-machine setup | Explicit `-DLAGRANGE_SANITIZER=…` |
| §7 | Viewer smoke test with `VK_ICD_FILENAMES=<lavapipe icd>` | Path is `/usr/share/vulkan/icd.d/lvp_icd.json` on Resolute | Fill in the path |
| §10, §13.5 | ImGui multi-viewport on Wayland "[Medium confidence]" | Confirmed: issue #9370 open, positioning still not possible; SDL's own notes say the same (verification §9) | Raise to [High] |
| §12.3 | "markdownlint-cli2 needs Node; prefer mdformat" | mdformat 1.0.0 is in apt | Remove the TODO; use mdformat |
| §12.6 | "`ros:lyrical` containers (TODO(verify) image tag)" | Tags `ros:lyrical`, `ros:lyrical-ros-base-resolute` exist for amd64 and arm64 | Name the tag |
| §12.6 | "ImGui Test Engine panel tests (TODO(verify) license)" | Custom license, free for OSS, not OSI (V13) | See Q12 |
| §4 | "Protobuf (or FlatBuffers …), Zenoh or TCP/Unix socket" | Recommendation in RFC 0003 | Protobuf + Unix socket/TCP + shm |
| §13.1 | "imgui-node-editor … (TODO(verify) license + maintenance)" | MIT; last release 2023-10, last push 2026-03 | See Q11 |
| §13.1 | "ImGuizmo or own implementation (TODO(verify))" | ImGuizmo 1.10 (2026-05), MIT | Use ImGuizmo |
| §1 | Roadmap omits radiation and comms | — | See Q2; comms with v0.5 |
| — | No mention of `simulation_interfaces` or `rosidl::Buffer` | Both exist on Lyrical (V5, V6) | Add to §6 bridges and §11.6 |

**F13. "Default RMW `rmw_fastrtps_cpp`".** Consistent with what `ros-lyrical-desktop` installs,
but the statement that it is the default came from a secondary source only. Keep, with
TODO(verify) against the official release page when it can be fetched.

**F14. Gazebo Jetty as the comparison baseline.** Reasonable: `gz-jetty` is what Lyrical pairs
with. Note that Jetty was released in October 2025 and its support ends before Lyrical's (one
source says September 2030), and that Gazebo's next release is planned for March 2027. The
baseline should move to that release when it ships.

## Part 3 — Suggested CLAUDE.md edits after your decisions

Not applied. Once the questions above are answered, one `docs:` PR should update: §1a (C++
wording, dependency list, Jetson), §2 (phase unchanged), §3.7 (determinism scope), §3.11 (schema
suffix), §4 (protocol row), §6 (`gpu/scene/`, `schemas/sensor/`, `schemas/world/`,
`simulation_interfaces`), §7 (commands), §11.3–§11.5 (examples with the fixes), §12.2–§12.3
(tooling fixes), §13.1 (resolved TODOs).
