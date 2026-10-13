# Dependencies

Status: draft for review. This file is the record required by CLAUDE.md §10: every dependency,
its license, how it is obtained on Ubuntu 26.04 with ROS 2 Lyrical, and its build risk. Versions
and licenses come from `docs/plan/verification.md` (checked 2026-10-13). A cell marked
`TODO(verify)` was not confirmed and must be checked before the dependency is added.

## 1. Policy

1. **Order of preference**: Ubuntu apt package through a rosdep key → ROS Lyrical package
   (including existing `*_vendor` packages) → our own `*_vendor` package in `vendor/` → never
   vcpkg, Conan or pip-installed C++ libraries next to rosdep (CLAUDE.md §1a).
2. **Licenses**: Apache-2.0-compatible permissive licenses in anything linked or shipped
   (Apache-2.0, MIT, BSD, Zlib, BSL-1.0, MPL-2.0 and EPL-2.0 as unmodified separate libraries or
   processes). No GPL or LGPL-only code linked into `lagrange_*` libraries. GPL tools may be used
   as developer tools or offline programs that exchange files.
3. **Pinning**: vendor packages pin an exact tag and a SHA-256 of the source archive. apt
   dependencies float with the distribution (that is what being a `ros-lyrical-*` deb means), and
   CI catches breakage from archive updates with a weekly scheduled build.
4. **Missing rosdep keys**: for apt packages without a key (SDL3, Slang, zenoh-c, SUMO, …) we
   contribute keys to `ros/rosdistro`. Until merged, a local rosdep source file
   (`packaging/rosdep/lagrange.yaml`) provides them, and the README tells users to add it.
5. **Recording**: adding a dependency means adding a row here in the same PR, with license and
   source. CI's REUSE check and an SBOM generation step cross-check the list.
6. **Optional dependencies** are compile-time options with a clear message when absent. A user
   without CUDA or PhysX can still build and run Tier 0 and Tier 1.

## 2. Core, simulation and I/O

| Dependency | Used by | Obtained from | Version seen | License | Build risk | Notes |
|---|---|---|---|---|---|---|
| OpenUSD | `core/stage`, `schemas`, importers, tools | `openusd_vendor` | upstream v26.08 | TOST-1.0 (Apache-2.0 with modified trademark clause) | **High** | Section 4 |
| MuJoCo | rigid body, MJCF import | `ros-lyrical-mujoco-vendor` (preferred) or `libmujoco-dev` | 3.12.0 (vendor), 3.5.0 (apt), 3.15.0 (upstream) | Apache-2.0 | Low | One source per build (Q9) |
| Jolt Physics | rigid body | `jolt_vendor` | v5.6.0 | MIT | Low | Plain CMake, no dependencies |
| PhysX 5 | rigid body (vehicles, GPU) | `physx_vendor` | SDK version TODO(verify) | BSD-3-Clause | **High** | Custom build system (`generate_projects`), GPU part needs the CUDA toolkit; GCC 15 compatibility TODO(verify). Optional |
| Embree | CPU ray casting (lidar, radar, dose, comms) | apt `libembree-dev` | 4.4.0 | Apache-2.0 | Low | rosdep key `embree` |
| TBB | Embree, OpenVDB, worker pool | apt `libtbb-dev` | 2022.3.0 | Apache-2.0 (TODO(verify)) | Low | |
| OpenVDB / NanoVDB | fields | apt `libopenvdb-dev`, `libnanovdb-dev` | 10.0.1 | Apache-2.0 (MPL-2.0 for older releases; TODO(verify) which applies to 10.0.1) | Medium | Three majors behind upstream 13.1.0 (R10). NanoVDB is header-only, so a newer copy can be vendored alone if needed |
| FMI import library | `modules/electrical/fmu_host` | `fmilib_vendor` (Modelon FMI Library) | 3.0.4 | BSD-3-Clause-style | Medium | FMI 3.0 co-simulation coverage TODO(verify). Alternative: a thin in-house loader over the FMI 3.0 C headers (the headers are BSD-2-Clause; TODO(verify)) |
| MCAP C++ | recording, replay, mock server | `ros-lyrical-mcap-vendor` | 0.33.3 | MIT | Low | Header-only library plus zstd/lz4 |
| MCAP Python (`mcap`) | mock server | PyPI, through the rosdep key `python3-mcap-pip` in `packaging/rosdep/lagrange.yaml`; version pinned in `packaging/pip-constraints.txt` | 1.5.0 | MIT (repository license; PyPI metadata has no license field) | Low | Not packaged for Ubuntu. Pulls `lz4` and `zstandard` from PyPI. Only the mock server needs it, which becomes a test tool in v0.2 |
| zstd, lz4 | MCAP, protocol compression | apt | 1.5.7, 1.10.0 | BSD / BSD-2-Clause (TODO(verify)) | Low | |
| Protobuf | protocol, recordings | apt `libprotobuf-dev`, `protobuf-compiler` | 3.21.12 | BSD-3-Clause (TODO(verify)) | Low | Old: proto3 only, no editions. rosdep key `protobuf-dev` |
| gRPC | control API (v0.2+) | apt `libgrpc++-dev` | 1.51.1 | Apache-2.0 (TODO(verify)) | Low | rosdep key TODO(verify) |
| Zenoh (zenoh-c, zenoh-cpp) | optional transport, bridge | apt `libzenohc-dev`, `libzenohcpp-dev` | 1.8.0 | Apache-2.0 OR EPL-2.0 | Low | `ros-lyrical-zenoh-cpp-vendor` 0.10.6 also exists; avoid loading two copies in one process (TODO(verify) symbol conflicts) |
| yaml-cpp | profiles | apt | 0.8.0 | MIT (TODO(verify)) | Low | |
| JSON Schema validator (C++) | profile validation in server and viewer | `TODO(verify)`: candidate libraries to evaluate; none found in apt in this pass | — | — | Medium | The mock server uses `python3-jsonschema` 4.19.2 (apt). For C++ the options are a small vendored validator or validating in-process through generated code |
| nlohmann-json | JSON handling | apt `nlohmann-json3-dev` | 3.11.3-based | MIT (TODO(verify)) | Low | |
| Eigen | maths | apt `libeigen3-dev` | 3.4.0 | MPL-2.0 | Low | Header-only |
| fmt, spdlog | logging | apt | 10.1.1, 1.15.3 | MIT | Low | |
| xxHash | state hashes, content keys | apt `libxxhash-dev` | 0.8.3 | BSD-2-Clause | Low | |
| CLI11 | command-line parsing | apt `libcli11-dev` | 2.6.1 | BSD-3-Clause | Low | |
| urdfdom / `urdf`, xacro | URDF import | ROS Lyrical | urdfdom 6.0.0, xacro 2.1.1 | BSD-3-Clause (TODO(verify)) | Low | |
| SDFormat | SDF import | `ros-lyrical-sdformat-vendor` | 0.3.4 (vendor package) | Apache-2.0 (TODO(verify)) | Low | |
| Assimp | mesh import | apt `libassimp-dev` | 6.0.4 | BSD-3-Clause (TODO(verify)) | Low | Import-time only |
| CoACD | collision bake tool | `coacd_vendor` | TODO(verify) | MIT | Medium | Offline tool; V-HACD (BSD-3-Clause) is the alternative |
| GDAL, PROJ, GeographicLib | geospatial | apt | 3.12.2, 9.7.1, 2.7 | MIT-style (TODO(verify) each) | Low | Tools only, plus GeographicLib in tests |
| nanobind | Python bindings | apt `nanobind-dev` | 2.11.0 | BSD-3-Clause | Low | rosdep key `nanobind-dev` |
| mp-units | optional compile-time units inside modules | `ros-lyrical-mp-units-vendor` | 2.5.0 | MIT (TODO(verify)) | Low | Not used for the runtime `Unit` check (Q6) |

Separate processes and offline tools (not linked, not required to build):

| Tool | Role | Source | License | Notes |
|---|---|---|---|---|
| SUMO | traffic co-simulation over TraCI | apt `sumo` 1.25.0 | EPL-2.0 | Runtime-optional |
| OpenModelica | author Modelica models and export FMUs | upstream v1.27.1, not in apt | OSMC-PL | FMI 3.0 export is experimental; redistribution of generated FMUs needs a license review (R5) |
| ngspice | run exported netlists | apt `ngspice` 45.2 | BSD-style (TODO(verify)) | Offline |
| OpenFOAM | produce wind fields | not checked | GPL-3.0 (TODO(verify)) | Offline; we consume files only |
| Geant4 | produce dose fields | not checked | Geant4 Software License (TODO(verify)) | Offline |
| PX4, ArduPilot SITL | autopilots in the loop | upstream | BSD-3-Clause, GPL-3.0 (TODO(verify)) | Separate processes over MAVLink/UDP/TCP |
| esmini | OpenSCENARIO reference / optional library | upstream | MPL-2.0 | Decision in v0.6 |

## 3. GPU and viewer

| Dependency | Used by | Obtained from | Version seen | License | Build risk | Notes |
|---|---|---|---|---|---|---|
| Vulkan headers and loader | `gpu/hal` | apt `libvulkan-dev` | 1.4.341.0 | Apache-2.0 / MIT (TODO(verify)) | Low | rosdep key `libvulkan-dev` |
| volk (Vulkan loader helper) | not used | not packaged | — | — | — | Correction: Ubuntu's `libvolk-dev` is GNU Radio's "Vector-Optimized Library of Kernels", an unrelated library with the same name. The GPU layer links the standard Vulkan loader (`libvulkan`) directly |
| Vulkan Memory Allocator | `gpu/hal` | apt `libvulkan-memory-allocator-dev` | 3.3.0 | MIT (TODO(verify)) | Low | Header-only |
| Slang | shader and kernel compilation | apt `slang-compiler`, `libslang-compiler-dev`; fallback `slang_vendor` | 2026.1.1 (apt), v2026.19 (upstream) | Apache-2.0 WITH LLVM-exception | Medium | CUDA target in the Ubuntu build TODO(verify). Shaders are compiled at build time, so Slang is a build dependency, not a runtime one, unless runtime kernel specialisation is added |
| SPIRV-Tools, validation layers | CI validation | apt | 2026.1, 1.4.341.0 | Apache-2.0 (TODO(verify)) | Low | Test-only |
| Mesa lavapipe | software Vulkan for CI | apt `mesa-vulkan-drivers` | 26.0.8 | MIT (TODO(verify)) | Low | Test-only |
| CUDA toolkit | optional fast paths, PhysX GPU, `cuda_buffer` | apt `nvidia-cuda-toolkit` (version TODO(verify)) | — | NVIDIA EULA (proprietary) | Medium | Optional. Debs built on the ROS buildfarm cannot depend on it by default; CUDA paths live in a separate optional package |
| SDL3 | viewer window and input | apt `libsdl3-dev` | 3.4.2 | Zlib | Low | No rosdep key yet |
| Dear ImGui (docking branch) | viewer UI | `imgui_vendor` | tag `v1.92.9b-docking`, pinned by commit and archive SHA-256 | MIT | Low | apt package is the master branch without docking (V1). The vendor package uses CMake `FetchContent` with a URL hash; `ament_cmake_vendor_package` is not needed for a library this small |
| ImPlot | scopes | `imgui_vendor` | v1.0 | MIT | Low | |
| imgui-node-editor | schematic | `imgui_vendor` | v0.9.3 + later commits | MIT | Medium | Compatibility with the pinned ImGui TODO(verify); behind an interface |
| ImGuizmo | gizmo | `imgui_vendor` | 1.10 | MIT | Low | |
| FreeType | font rasterisation | apt `libfreetype-dev` | 2.14.2 | FTL or GPL-2.0 (use under FTL) | Low | |
| stb (image) | texture decode | apt `libstb-dev` | 2025-09 snapshot | MIT or public domain | Low | |
| Fonts: IBM Plex Sans (400/500/600) and IBM Plex Mono (400/500); IBM Plex Sans JP and an icon font still to choose | viewer | TTFs in `viewer/assets/fonts/`, installed to `share/lagrange_viewer/fonts` | TODO(verify) upstream release of the bundled builds | SIL OFL 1.1 (`LICENSES/OFL-1.1.txt`); JP and icon fonts TODO(verify) | Low | Chosen by the UI design (`docs/design/ui/`). A subsetted Japanese font must stay under 1 MB per file or move to the assets bundle |
| Dear ImGui Test Engine | UI tests | fetched in CI only | — | Dear ImGui Test Engine License v1.04 (not OSI) | Low | Never vendored, never shipped (Q12) |

`imgui_vendor` builds one static library containing ImGui core, the SDL3 platform backend, the
Vulkan renderer backend, `imgui_freetype`, ImPlot, imgui-node-editor and ImGuizmo, all from
pinned commits, because they must agree on one ImGui version and one `imconfig.h`.

## 4. OpenUSD build plan (highest build risk)

Facts: not packaged for Ubuntu 26.04; upstream v26.08; needs TBB and (for imaging) many optional
libraries; long build. Building with GCC 15.2, CMake 4.2 and Python 3.14 is unverified.

Plan:

1. **Minimal configuration.** `openusd_vendor` builds only what the simulator core needs:
   `usd`, `usdGeom`, `usdShade`, `usdPhysics`, `usdLux`, `sdf`, `ar`, `tf`, `vt`, `gf`, `plug`,
   `kind`, `pcp`, `work`, `trace`. Off: imaging (Hydra, `usdImaging`, `usdview`), OpenGL,
   OpenImageIO, OpenColorIO, Alembic, Draco, MaterialX (until needed), tests, tutorials,
   examples. Options in upstream CMake terms: `PXR_BUILD_IMAGING=OFF`,
   `PXR_BUILD_USD_IMAGING=OFF`, `PXR_BUILD_USDVIEW=OFF`, `PXR_BUILD_TESTS=OFF`,
   `PXR_BUILD_EXAMPLES=OFF`, `PXR_BUILD_TUTORIALS=OFF`, `PXR_ENABLE_PYTHON_SUPPORT` per step 3
   (option names TODO(verify) against v26.08).
2. **System dependencies only**: TBB from apt. No bundled downloads; the vendor package does not
   use upstream's `build_usd.py`, which fetches its own dependencies.
3. **Python**: build the USD Python bindings against system Python 3.14 only if step 5's spike
   shows it works. Otherwise C++ only; Python tools that need USD use a subprocess to our C++
   tools.
4. **Monolithic shared library** (`PXR_BUILD_MONOLITHIC=ON`) installed under the package prefix
   with a namespaced library name, so that it cannot collide with another USD on the machine
   (NVIDIA tools ship their own).
5. **Spike first (v0.2 issue).** Build v26.08 in a `ros:lyrical` container on amd64 and arm64;
   record build time, binary size, and patches needed. Fallback versions: the newest release
   that builds. The spike result decides the pin.
6. **Caching**: ccache for developers; in CI, the built `openusd_vendor` install tree is cached
   keyed on (USD tag, compiler version, CMake options hash) and restored as a colcon underlay.
   A prebuilt underlay tarball per architecture is also published as a release asset so that
   contributors can skip the build.
7. **Binary distribution**: whether the ROS buildfarm can build `openusd_vendor` (time limits,
   memory, network access during the build) is unverified (R1). Fallback: publish debs from our
   own apt repository built in our CI, as Gazebo does with `packages.osrfoundation.org`.

v0.1 does not need C++ OpenUSD at all: the viewer speaks only the protocol, and the mock server
reads recordings. See open question Q4 for how the mock server gets scene content.

## 5. ROS 2 dependencies

| Package | Used by | Version on Lyrical | Notes |
|---|---|---|---|
| `ament_cmake`, `ament_cmake_vendor_package`, `ament_index_cpp` | build, plugin discovery | 2.8.8 | `ament_target_dependencies` is removed; use `target_link_libraries` |
| `rclcpp`, `rclcpp_components` | ROS bridge | 32.0.3 | |
| `rosgraph_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `visualization_msgs` | ROS bridge | Lyrical | |
| `rosidl_buffer`, `cuda_buffer` | zero-copy sensor output | 5.2.1, 0.1.2 | `cuda_buffer` is optional (pulls NVIDIA libraries) |
| `simulation_interfaces` | standard simulator services | in rosdistro; version TODO(verify) | |
| `hardware_interface`, `controller_manager`, `pluginlib` | `lagrange_ros2_control` | ros2_control 6.10.1 | |
| `rosbag2_storage_mcap`, `mcap_vendor` | recording interoperability | 0.33.3 | |
| `rmw_fastrtps_cpp`, `rmw_zenoh_cpp` | tested RMWs | 9.4.10, 0.10.6 | |
| Test-only: `nav2_bringup`, `moveit`, `ros2_controllers`, `mavros` | integration tests and demos | 1.5.1, 2.15.2, 6.9.0, 2.15.1 | `<test_depend>` or demo packages only |

The viewer package (`lagrange_viewer`) depends on `ament_cmake` and `ament_index_cpp` for build
and resource lookup only. It does not depend on `rclcpp`, so it starts without a ROS graph.

## 6. Developer tools

| Tool | Source | Version seen | License | Notes |
|---|---|---|---|---|
| clang-format, clang-tidy | apt | 21.1.6 | Apache-2.0 WITH LLVM-exception | Pin the major version (21) in CI and pre-commit |
| gersemi | PyPI via pre-commit | 0.29.2 | MPL-2.0 | Not in apt |
| ruff | PyPI via pre-commit | 0.17.0 | MIT | Not in apt |
| mypy | apt or PyPI | 1.19.1 / 2.4.0 | MIT | |
| buf | GitHub release binary | v1.73.0 | Apache-2.0 | Not in apt; installed in CI by checksum |
| pre-commit | apt | 4.5.1 | MIT | |
| yamllint | apt | 1.37.1 | GPL-3.0-or-later | Tool only |
| yamlfmt | Go binary | TODO(verify) | Apache-2.0 | Optional; yamllint alone may be enough |
| mdformat | apt | 1.0.0 | MIT | Replaces markdownlint (Node) |
| shfmt, shellcheck | apt | 3.12.0, 0.11.0 | BSD-3-Clause, GPL-3.0 (TODO(verify)) | Tools only |
| typos | pre-commit | TODO(verify) | MIT or Apache-2.0 (TODO(verify)) | |
| reuse | apt | 5.0.2 | mixed, includes GPL-3.0-or-later | Tool only |
| commitizen | PyPI via pre-commit | 4.19.2 | MIT | Replaces commitlint (Node) (Q13) |
| release-please | GitHub Action | v17.11.2 | Apache-2.0 | Runs on GitHub's runners, not on developer machines |
| ccache, sccache | apt | 4.12.3, 0.13.0 | GPL-3.0, Apache-2.0 (TODO(verify)) | Tools only |
| libabigail (`abidiff`) | apt (TODO(verify)) | — | — | ABI checks from v0.9 |
| GoogleTest, Google Benchmark | apt or ROS vendor packages | 1.17.0, 1.9.1 | BSD-3-Clause, Apache-2.0 | |
| pytest, check-jsonschema | apt / PyPI | 9.0.2, TODO(verify) | MIT, Apache-2.0 (TODO(verify)) | |

## 7. Vendor packages to create

| Package | Upstream | Needed from | Build time class | Cache strategy |
|---|---|---|---|---|
| `imgui_vendor` | ocornut/imgui (docking), epezent/implot, thedmd/imgui-node-editor, CedricGuillemet/ImGuizmo | v0.1 | seconds | ccache |
| `openusd_vendor` | PixarAnimationStudios/OpenUSD | v0.2 | tens of minutes (TODO(verify) by the spike) | install-tree cache + prebuilt underlay |
| `jolt_vendor` | jrouwe/JoltPhysics | v0.2 or v0.3 | minutes | ccache |
| `fmilib_vendor` | modelon-community/fmi-library | v0.3 | minutes | ccache |
| `physx_vendor` | NVIDIA-Omniverse/PhysX | v0.6 (or earlier for GPU batches) | tens of minutes | install-tree cache; CUDA part optional |
| `coacd_vendor` | SarahWeiii/CoACD | v0.2 tools | minutes | ccache |
| `slang_vendor` | shader-slang/slang | only if the apt build lacks a needed target | tens of minutes | install-tree cache |

Each vendor package uses `ament_cmake_vendor_package` (`ament_vendor(...)` with a pinned
`VCS_VERSION` and patches in `vendor/<name>/patches/`), installs under its own prefix, exports
CMake targets, and has a `package.xml` whose `<license>` is the upstream license.

## 8. Things deliberately not depended on

| Not used | Reason |
|---|---|
| Qt, OGRE, gz-rendering | The viewer has its own Vulkan renderer shared with sensors (invariant 13) |
| Any browser engine, Node, Electron, Tauri | Invariant 13 |
| vcpkg, Conan | CLAUDE.md §1a |
| Omniverse Kit, RTX renderer | Proprietary terms; not redistributable in an Apache-2.0 project (verification §5) |
| Warp, Newton, MuJoCo Warp, Genesis as linked backends | Python/Warp runtimes; cannot be linked into the C++ core. They remain comparison baselines and possible out-of-process backends through an RFC |
| Boost (direct use) | Not needed with C++20; OpenVDB pulls parts of it transitively |
