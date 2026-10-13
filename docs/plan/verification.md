# Verification record

Date of checks: **2026-10-13**. Everything here was checked on that day; re-run before pinning.

Three kinds of evidence are used and labelled on every row:

- **LOCAL** — read directly from an Ubuntu 26.04.1 (Resolute) amd64 machine with
  `ros-lyrical-desktop` installed, the `ros2` and `gazebo-stable` apt sources enabled, and an
  NVIDIA RTX 4060 Laptop GPU on driver 595.91.07 in a Wayland session. Commands: `apt-cache policy`,
  `apt-cache show`, `dpkg-deb`, and inspection of `/opt/ros/lyrical`.
- **API** — GitHub REST API (`/repos/{r}` and `/repos/{r}/releases/latest`), PyPI JSON API,
  Docker Hub tags API, or a raw file from the upstream repository.
- **WEB** — a web search result or fetched page. Secondary sources are marked as such.

Anything not confirmed by one of these is marked **UNVERIFIED**. Nothing in this file is a guess.

## 1. Findings that change the plan

| # | Finding | Evidence | Consequence |
|---|---|---|---|
| V1 | `libimgui-dev` 1.92.2b in Ubuntu 26.04 is the **master** branch: `imgui.h` has no `IMGUI_HAS_DOCK` / `IMGUI_HAS_VIEWPORT`. | LOCAL (`dpkg-deb` extract, grep) | Dear ImGui docking must be vendored (`imgui_vendor`). The apt package cannot be used. |
| V2 | OpenUSD is **not** in Ubuntu 26.04 or in rosdep. Upstream latest is v26.08 (2026-07-20). | LOCAL (`apt-cache search`), API | `openusd_vendor` is required for any C++ USD use. v0.1 can avoid it (see open question Q4). |
| V3 | Lyrical core packages mostly export `cxx_std_17` (48 CMake exports) and only 2 export `cxx_std_20`. | LOCAL (grep of `/opt/ros/lyrical/share/*/cmake`) | "C++20 matching Lyrical's baseline" in CLAUDE.md §1a is not what the installed distro shows. C++20 is still usable (GCC 15.2). See Q1. |
| V4 | `ament_target_dependencies` is gone: `/opt/ros/lyrical/share/ament_cmake_target_dependencies/cmake/` contains only `ament_get_recursive_properties.cmake` and config files. | LOCAL | Confirms CLAUDE.md §1a. Use `target_link_libraries` with exported targets. |
| V5 | Lyrical ships a GPU/zero-copy transport: `rosidl::Buffer`. All `uint8[]` fields are `rosidl::Buffer<uint8_t>` in C++. Backends are pluggable; `ros-lyrical-cuda-buffer` 0.1.2 exists and depends on `libcudart12` and `libnvidia-compute-595`. | WEB (release announcement, release-notes source), LOCAL (apt) | The ROS 2 bridge can publish `sensor_msgs/Image` and `PointCloud2` from GPU memory without a CPU copy. Planned for v0.2 sensors. |
| V6 | `simulation_interfaces` is in the Lyrical rosdistro. | API (`ros/rosdistro` `lyrical/distribution.yaml`) | The ROS 2 bridge should implement this standard simulator API. It was missing from the plan. |
| V7 | Autoware is not released into Lyrical: only `autoware_*_msgs`, `autoware_utils_*`, `autoware_cmake` and `autoware_lint_common` are in apt. `autoware_core` and `autoware_universe` are absent from the Lyrical rosdistro. | LOCAL, API | v0.6 (autonomous cars) depends on an upstream that does not yet target the platform. Risk R7. |
| V8 | PX4's ROS 2 guide recommends Jazzy on Ubuntu 24.04 (Humble on 22.04). ArduPilot's `Tools/ros2/README.md` lists Humble and Jazzy only. Neither names Lyrical. | API (raw files on `main`/`master`) | v0.4 should use the MAVLink/SITL path, which does not depend on the ROS distro. |
| V9 | JetPack is at 7.2.1 on Ubuntu 24.04 (kernel 6.8). No JetPack on Ubuntu 26.04 was found. | WEB (NVIDIA downloads page via search) | No Jetson support can be promised. arm64 stays "build + CPU tests". |
| V10 | Three MuJoCo versions coexist: Ubuntu `libmujoco-dev` 3.5.0, `ros-lyrical-mujoco-vendor` "version 3.12.0", upstream 3.15.0. | LOCAL, API | Pick one source of MuJoCo per build (Q9). |
| V11 | Ubuntu's OpenVDB/NanoVDB is 10.0.1; upstream is 13.1.0. | LOCAL, API | Three major versions behind. GPU NanoVDB features may be missing (R10). |
| V12 | Slang **is** packaged in Ubuntu 26.04 universe: `slang-compiler` / `libslang-compiler-dev` 2026.1.1. Upstream latest is v2026.19. | LOCAL, API | A `slang_vendor` package may not be needed. Whether the Ubuntu build enables the CUDA target is UNVERIFIED. |
| V13 | Dear ImGui Test Engine is free for open-source derivative software, but its license is a custom one (v1.04), not OSI-approved. | API (raw `LICENSE.txt`) | It may be used in CI, but must not be vendored into the Apache-2.0 source tree or shipped in debs. See Q12. |
| V14 | `commitlint` and `markdownlint-cli2` need Node. Python alternatives exist: `commitizen` 4.19.2 (MIT), `mdformat` 1.0.0 (MIT, also in apt). | API (PyPI), LOCAL | To keep Node out of the dev toolchain, use the Python tools (Q13). |
| V15 | RViz2 on Lyrical renders with OGRE **1.12.10** (OpenGL), built by `rviz_ogre_vendor`. | LOCAL (`OgrePrerequisites.h`, `libOgreMain.so.1.12.10`) | No renderer can be shared with RViz2. Interoperate through standard ROS messages only. |
| V16 | OpenModelica is under the OSMC Public License and lists FMI 3.0.2 export as **Experimental** (FMI 2.0.5 is supported). | API (raw license file), WEB (User's Guide via search) | FMI 3.0 FMUs from OpenModelica are a schedule and licensing risk (R5, Q8). |

## 2. Ubuntu 26.04 toolchain

| Item | Version | Evidence |
|---|---|---|
| Ubuntu | 26.04.1 LTS, codename `resolute` | LOCAL (`lsb_release`) |
| GCC / G++ | 15.2.0 (`gcc` meta 4:15.2.0-5ubuntu1) | LOCAL |
| Clang, clang-format, clang-tidy | 21.1.6 (1:21.1.6-71) | LOCAL (apt candidate) |
| CMake | 4.2.3 | LOCAL |
| Python | 3.14 (`python3` 3.14.3 candidate; 3.14.4 installed) | LOCAL |
| colcon | `colcon` 0.20.0, `python3-colcon-common-extensions` 0.3.0 | LOCAL |
| rosdep | 0.27.0 | LOCAL |
| bloom | `python3-bloom` 0.14.4 | LOCAL |
| ccache / sccache | 4.12.3 / 0.13.0 | LOCAL |
| Mesa Vulkan drivers | 26.0.8; installs `lvp_icd.json` (lavapipe) | LOCAL (`/usr/share/vulkan/icd.d/`) |
| Vulkan headers/loader | 1.4.341.0 | LOCAL |
| NVIDIA driver | `nvidia-driver-595` 595.91.07; `nvidia-driver-580` 580.178.04 | LOCAL |

CMake 4.x removed compatibility with `cmake_minimum_required` below 3.5 [Medium, from general
CMake 4.0 release knowledge; not re-checked]. Vendored third-party projects with old minimums may
need `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`. Verify per vendor package.

## 3. Dependencies: availability on Resolute

"rosdep key" is the key in `ros/rosdistro` `rosdep/base.yaml` on 2026-10-13 (API). "none" means a
key must be contributed upstream or the dependency handled by a vendor package.

| Dependency | apt package and version | rosdep key | Upstream latest | License (SPDX) | Decision |
|---|---|---|---|---|---|
| OpenUSD | not packaged | none | v26.08 | Tomorrow Open Source Technology License 1.0 (Apache-2.0 with a changed trademark section) | `openusd_vendor` |
| PhysX 5 | not packaged | none | repo active 2026-10-09; latest tag is `ovphysx-0.6.4`, the SDK version is UNVERIFIED | BSD-3-Clause | `physx_vendor` |
| MuJoCo | `libmujoco-dev` 3.5.0; `ros-lyrical-mujoco-vendor` (3.12.0) | none | 3.15.0 | Apache-2.0 | see Q9 |
| Jolt Physics | not packaged | none | v5.6.0 | MIT | `jolt_vendor` |
| Embree | `libembree-dev` 4.4.0 | `embree` | v4.4.1 | Apache-2.0 | apt |
| OpenVDB / NanoVDB | `libopenvdb-dev`, `libnanovdb-dev` 10.0.1; `ros-lyrical-openvdb-vendor` 2.7.0 | `libopenvdb-dev`, `libnanovdb-dev` | v13.1.0 | Apache-2.0 (GitHub reports Apache-2.0) | apt first, vendor if needed |
| Slang | `slang-compiler`, `libslang-compiler-dev` 2026.1.1 | none | v2026.19 | Apache-2.0 WITH LLVM-exception | apt, add rosdep key |
| FMI import library | not packaged | none | Modelon FMI Library 3.0.4 (2025-06-25) | BSD-3-Clause-style (file `LICENSE.md`) | `fmilib_vendor`; FMI 3.0 coverage UNVERIFIED |
| fmi4c | — | — | UNVERIFIED (the repository path tried returned 404) | UNVERIFIED | not planned until verified |
| OpenModelica | not packaged | none | v1.27.1 | OSMC-PL | offline tool only, not a build dependency |
| nanobind | `nanobind-dev`, `python3-nanobind` 2.11.0 | `nanobind-dev` | PyPI 3.1.0 | BSD-3-Clause | apt |
| MCAP (C++) | no standalone package; `ros-lyrical-mcap-vendor` 0.33.3 | via ROS package dependency | repo active; latest release tag is the Python package v1.5.0 | MIT | depend on `mcap_vendor` |
| Zenoh | `libzenohc-dev`, `libzenohcpp-dev` 1.8.0; `ros-lyrical-zenoh-cpp-vendor` 0.10.6 | none for the system libs | 1.10.1 | Apache-2.0 OR EPL-2.0 | optional transport, later |
| SDL3 | `libsdl3-dev` 3.4.2 | none | release-3.4.18 | Zlib | apt, add rosdep key |
| GLFW | `libglfw3-dev` 3.4 | `libglfw3-dev` | 3.5.1 | Zlib | not chosen (RFC 0006) |
| Dear ImGui | `libimgui-dev` 1.92.2b (master branch, **no docking**) | none | v1.92.9b; docking branch is at "1.93.0 WIP" | MIT | `imgui_vendor` (docking branch commit) |
| ImPlot | not packaged | none | v1.0 (2026-04-05) | MIT | inside `imgui_vendor` |
| imgui-node-editor | not packaged | none | v0.9.3 (2023-10-14); last push 2026-03-29 | MIT | inside `imgui_vendor`, behind an interface |
| imnodes | not packaged | none | v0.5 (2022-03-09); last push 2026-05-13 | MIT | fallback candidate |
| ImGuizmo | not packaged | none | 1.10 (2026-05-11) | MIT | inside `imgui_vendor` |
| Vulkan Memory Allocator | `libvulkan-memory-allocator-dev` 3.3.0 | none | not checked | MIT (UNVERIFIED here) | apt |
| volk | `libvolk-dev` 3.3.0 | none | not checked | MIT (UNVERIFIED here) | apt |
| Protobuf | `libprotobuf-dev`, `protobuf-compiler` 3.21.12 | `protobuf-dev` | not checked | BSD-3-Clause (UNVERIFIED here) | apt; proto3 only, no editions |
| FlatBuffers | `libflatbuffers-dev` 23.5.26 | `libflatbuffers-dev` | not checked | Apache-2.0 (UNVERIFIED here) | not chosen (RFC 0003) |
| gRPC | `libgrpc++-dev` 1.51.1 | none found under `grpc` | not checked | Apache-2.0 (UNVERIFIED here) | v0.2+, key TODO(verify) |
| ngspice | `ngspice` 45.2 | none | not checked | UNVERIFIED | offline tool |
| SUMO | `sumo` 1.25.0 | none | not checked | EPL-2.0 | separate process (TraCI) |
| GDAL / PROJ / GeographicLib | 3.12.2 / 9.7.1 / 2.7 | `libgdal-dev`, `proj`, `geographiclib` | not checked | UNVERIFIED here | apt |
| Assimp | `libassimp-dev` 6.0.4 | `assimp` | not checked | BSD-3-Clause (UNVERIFIED here) | apt (DAE/OBJ/STL import) |
| CoACD | not packaged | none | not checked | MIT | `coacd_vendor`, offline tool |
| V-HACD | not packaged | none | not checked | BSD-3-Clause | alternative to CoACD |
| esmini (OpenSCENARIO) | not packaged | none | not checked | MPL-2.0 | candidate for v0.6 |
| yaml-cpp, nlohmann-json, Eigen, TBB, spdlog, fmt, CLI11, GTest, benchmark, xxhash, zstd, lz4, tinyxml2, FreeType | all in apt (0.8.0, 3.11.3-based, 3.4.0, 2022.3.0, 1.15.3, 10.1.1, 2.6.1, 1.17.0, 1.9.1, 0.8.3, 1.5.7, 1.10.0, 11.0.0, 2.14.2) | all have keys | not checked | permissive (UNVERIFIED here) | apt |
| mp-units | `ros-lyrical-mp-units-vendor` 2.5.0 | via ROS package | not checked | UNVERIFIED | optional compile-time units inside modules (Q6) |

License notes that affect design:

- **OpenModelica (OSMC-PL).** FMUs exported by OpenModelica embed OpenModelica runtime sources.
  Whether prebuilt FMUs containing that runtime can be redistributed inside an Apache-2.0 repo and
  in debs is UNVERIFIED and needs a license review before any FMU binary is committed (R5).
- **SUMO (EPL-2.0)** and **esmini (MPL-2.0)** are weak-copyleft. Using them as separate processes
  or unmodified shared libraries is compatible with shipping Apache-2.0 code [Medium, standard
  reading of those licenses; confirm in the dependency review].
- **yamllint** (GPL-3.0-or-later) and **reuse** (includes GPL-3.0-or-later) are developer tools
  only. They are never linked or shipped.

## 4. ROS 2 Lyrical Luth

| Item | Finding | Evidence |
|---|---|---|
| Release | 22 May 2026, LTS, supported until May 2031 | WEB (release announcement on ROS Discourse) |
| Tier 1 platforms | Ubuntu 26.04 amd64 and arm64; Windows 11 amd64 | WEB (same) |
| Default RMW | `rmw_fastrtps_cpp` 9.4.10 is installed by `ros-lyrical-desktop`; `rmw_zenoh_cpp` 0.10.6 is available | LOCAL. That Fast DDS is *the default* is from a secondary source only [Medium] |
| C++ standard policy | UNVERIFIED as a written policy (REP 2000 on ros.org was blocked; the GitHub copy fetched had no Lyrical section). Observed: 48 `cxx_std_17` exports vs 2 `cxx_std_20` | LOCAL |
| `rosidl::Buffer` | Zero-copy buffers; release-notes source says publishers and subscribers on `rmw_fastrtps_cpp` **or** `rmw_zenoh_cpp` may use it. The Discourse announcement said Zenoh support was "coming soon", so Zenoh support landed after release | API (raw `Release-Lyrical-Luth.rst` on `rolling`), WEB |
| Other changes | `EventsCBGExecutor`; asyncio `AsyncNode` in rclpy; URDF 1.2 acceleration/jerk limits; RViz can fetch meshes over a service | WEB, API |
| Docker images | `ros:lyrical`, `ros:lyrical-ros-core[-resolute]`, `ros:lyrical-ros-base[-resolute]`, `ros:lyrical-perception[-resolute]` — amd64 + arm64, updated 2026-10-02. `osrf/ros:lyrical-desktop[-full]`, `osrf/ros:lyrical-simulation` updated 2026-09-10 | API (Docker Hub) |
| lavapipe in images | `mesa-vulkan-drivers` is in the Resolute archive and provides lavapipe. Whether it is preinstalled in `ros:lyrical` is UNVERIFIED; CI installs it explicitly | LOCAL |

Integration packages on Lyrical (LOCAL apt candidates unless noted):

| Stack | Package and version | Status |
|---|---|---|
| ros2_control | `ros-lyrical-ros2-control` 6.10.1, `ros2-controllers` 6.9.0 | available |
| gz_ros2_control | `ros-lyrical-gz-ros2-control` 3.0.9 | available |
| mujoco_ros2_control | `ros-lyrical-mujoco-ros2-control` 0.1.2 | available; a direct point of comparison for our `SystemInterface` |
| Nav2 | `ros-lyrical-navigation2`, `nav2-bringup` 1.5.1 | available |
| MoveIt 2 | `ros-lyrical-moveit` 2.15.2 | available |
| Open-RMF | `rmf-traffic` 3.8.0, `rmf-task`, `rmf-traffic-ros2`, messages, demos assets, `rmf-building-sim-gz-plugins` | available. `ros-lyrical-rmf-fleet-adapter` has **no apt candidate** |
| MAVROS | `ros-lyrical-mavros` 2.15.1 | available |
| px4_msgs | in the Lyrical rosdistro file, but `ros-lyrical-px4-msgs` has no apt candidate | source-only or not yet synced |
| Autoware | messages and utils only | **not available** as a stack (V7) |
| Gazebo | `gz-jetty` 1.0.0, `ros-lyrical-ros-gz` 3.0.10, `gz-sim-vendor` 0.4.6 | available |
| rosbag2 MCAP | `ros-lyrical-rosbag2-storage-mcap` 0.33.3 | available |
| Foxglove bridge | `ros-lyrical-foxglove-bridge` 3.5.0 | available |
| simulation_interfaces | in the Lyrical rosdistro | available (version not checked) |

## 5. Competitors and adjacent projects

| Project | Status on 2026-10-13 | License | Evidence |
|---|---|---|---|
| Gazebo Jetty (gz-sim 10) | 10th major release, announced 2025-10-01, LTS. End of support reported as September 2030 by the announcement; another page says May 2031 (conflict, UNVERIFIED). Official platform at release was Ubuntu 24.04 amd64; arm, Windows and macOS best-effort. Packaged for Resolute as `gz-jetty` | Apache-2.0 (UNVERIFIED in this pass; API rate limit) | WEB, LOCAL |
| Jetty features | Zenoh transport option (`GZ_TRANSPORT_IMPLEMENTATION=zenoh`), standalone `gz` executables, parallel Fuel downloads, versionless package names, server-only package, warehouse/Open-RMF demo, RL tutorial with Stable Baselines3, mass-based auto inertia, wheel-slip plugin | — | WEB |
| Gazebo release cadence | From 2027, Gazebo releases in March, two months before the paired ROS 2 release | — | WEB (Lyrical announcement) |
| Isaac Sim | v6.1.0 (2026-09-10). Source is Apache-2.0, but building or running needs Omniverse Kit and assets under separate NVIDIA terms; redistribution as part of a product or service needs an NVIDIA AI Enterprise license | Apache-2.0 + additional NVIDIA terms | API (raw `LICENSE`), WEB (license FAQ 6.0) |
| Isaac Lab | v3.0.0-EA (2026-09-16) | BSD-3-Clause | API |
| Newton | v1.6.1 (2026-10-05). Linux Foundation project. Extends Warp; uses MuJoCo Warp as primary backend. 1.0 reported GA in March 2026 (secondary source) | Apache-2.0 | API, WEB |
| MuJoCo | 3.15.0 (2026-10-05) | Apache-2.0 | API |
| MuJoCo Warp | v3.15.0 (2026-10-05) | Apache-2.0 | API |
| MJX | UNVERIFIED in this pass | — | — |
| Genesis | `LICENSE` on `main` is Apache-2.0. Release cadence and maturity UNVERIFIED (API rate limit; only a January 2025 v0.2.1 seen in a secondary source) | Apache-2.0 | API (raw file) |
| PhysX GPU source | GPU simulation kernels released as source under BSD-3 with PhysX 5.6 (April 2025). CUDA only | BSD-3-Clause | WEB (secondary: CGChannel, others), API for repo license |
| Warp | `LICENSE.md` is Apache-2.0 | Apache-2.0 | API (raw file) |

What Gazebo Jetty does not offer, as far as these sources show, and what this project targets: an
electrical domain (battery, motor drive, bus), thermal networks, EMI and radiation fields, a
per-domain toggle with substitutes, a unit-checked signal bus, a native Vulkan viewer with scopes
and a schematic, and a batched GPU physics path. This list is drawn from the Jetty announcement
and feature list only; a line-by-line check of the gz-sensors and gz-rendering changelogs was not
done [Medium].

## 6. OpenUSD and standards

| Item | Finding | Evidence |
|---|---|---|
| OpenUSD license | "Tomorrow Open Source Technology License 1.0"; differs from Apache-2.0 only in section 6 (trademarks). Renamed in 24.08 | API (raw `LICENSE.txt`), WEB (AOUSD forum) |
| AOUSD Core Specification 1.0 | Announced 2025-12-17; covers data model, composition and value resolution | WEB (Linux Foundation press release) |
| UsdPhysics standardization | Not found in the Core Spec 1.0 coverage. An AOUSD physics working group exists. Status of a normative physics spec is UNVERIFIED | WEB |
| OpenUSD and Python 3.14 | v26.05 added Python 3.14 for the `usd-core` wheel (secondary summary of the AOUSD release post) | WEB [Medium] |
| OpenUSD with GCC 15 / CMake 4 | UNVERIFIED. Must be tested by building `openusd_vendor` | — |

## 7. Slang

| Item | Finding | Evidence |
|---|---|---|
| Governance | Khronos-hosted project | WEB |
| Targets | SPIR-V and CUDA are listed targets in the documentation | WEB (secondary mirrors) [Medium] |
| Ubuntu package | `slang-compiler` 2026.1.1 in universe with `libslang-compiler-dev`, `libslang-rt-dev`, `libgfx-dev` | LOCAL |
| CUDA target in the Ubuntu build | UNVERIFIED | — |
| Upstream | v2026.19 (2026-09-29) | API |

## 8. FMI tooling

| Item | Finding | Evidence |
|---|---|---|
| OpenModelica FMI 3.0 | FMI 3.0.2 export is "Experimental"; FMI 2.0.5 is supported | WEB (User's Guide) |
| Modelon FMI Library | 3.0.4, BSD-style license | API |
| FMPy | 0.3.32 on PyPI (useful for FMU validation in tests) | API |
| fmi4c | UNVERIFIED | — |
| Reference FMUs (modelica/Reference-FMUs) | not checked | UNVERIFIED |

## 9. Viewer stack

| Item | Finding | Evidence |
|---|---|---|
| SDL3 | 3.4.2 in apt. Upstream README: Wayland is preferred over X11 by default; toplevel windows cannot be positioned programmatically on Wayland; decorations on GNOME need libdecor; window state restore needs `xdg-session-management-v1` | LOCAL, API (raw `docs/README-wayland.md`) |
| GLFW | 3.4 in apt, 3.5.1 upstream | LOCAL, API |
| ImGui SDL3 backend (docking branch) | Multi-viewport supported; IME supported with a known position issue; uses SDL 3.4 display events | API (raw `backends/imgui_impl_sdl3.cpp` header) |
| ImGui GLFW backend (docking branch) | Multi-viewport supported, but "Missing ImGuiBackendFlags_HasParentViewport support" | API (raw `backends/imgui_impl_glfw.cpp` header) |
| Multi-viewport on Wayland | Issue #9370 (opened 2026-04-16) is open with no maintainer reply; window positioning remains the limit | WEB |
| Docking branch version | `IMGUI_VERSION "1.93.0 WIP"` | API |
| ImPlot | v1.0 released 2026-04-05, MIT | API |
| imgui-node-editor | MIT; last release 2023-10; last push 2026-03. Compatibility with ImGui 1.92+/1.93 is UNVERIFIED | API |
| ImGuizmo | 1.10 (2026-05-11), MIT | API |
| ImGui Test Engine | Custom license v1.04. Free if the derivative software is public and under an OSI license, or for individuals, education, non-profits, or entities under 2 M USD turnover; otherwise paid after a 45-day trial | API (raw license) |
| lavapipe | `lvp_icd.json` installed by `mesa-vulkan-drivers` 26.0.8 | LOCAL |
| RViz2 | 15.2.6, OGRE 1.12.10 | LOCAL |
| Fonts | `fonts-noto-cjk` in apt (SIL OFL per upstream; license UNVERIFIED here). A Japanese-capable font must be bundled or found at runtime | LOCAL |

## 10. GPU drivers, runners, storage

| Item | Finding | Evidence |
|---|---|---|
| NVIDIA driver on 26.04 | 595.91.07 and 580.178.04 in the archive | LOCAL |
| Vulkan hardware ray tracing on 26.04 | Not tested in this pass (`vulkaninfo` not installed on the check machine). NVIDIA's proprietary driver has supported `VK_KHR_ray_tracing_pipeline` for years [High as general knowledge, UNVERIFIED on this exact driver] | — |
| GitHub-hosted GPU runner | `linux_4_core_gpu` at USD 0.052 per minute; needs a Team or Enterprise Cloud plan. GPU model was Tesla T4 in a 2024 listing; current hardware UNVERIFIED | WEB (GitHub docs via search) |
| Git LFS | 10 GiB storage and 10 GiB bandwidth per month on Free; 250 GiB on Team/Enterprise; overage USD 0.07/GiB storage and 0.0875/GiB bandwidth | WEB (GitHub docs via search) |
| Jetson | JetPack 7.2.1, Ubuntu 24.04 | WEB |

## 11. Tool versions (for pinning later)

| Tool | Latest | Where | Evidence |
|---|---|---|---|
| clang-format / clang-tidy | 21.1.6 | apt on Resolute | LOCAL |
| gersemi | 0.29.2 (MPL-2.0) | PyPI / GitHub | API |
| ruff | 0.17.0 (MIT) | PyPI / GitHub; not in apt | API |
| mypy | PyPI 2.4.0; apt 1.19.1 | — | API, LOCAL |
| buf | v1.73.0 (Apache-2.0) | GitHub; not in apt | API |
| release-please | v17.11.2 (Apache-2.0) | GitHub | API |
| pre-commit | PyPI 4.7.0; apt 4.5.1 | — | API, LOCAL |
| reuse | PyPI 6.2.0; apt 5.0.2 | — | API, LOCAL |
| yamllint | PyPI 1.38.0; apt 1.37.1 | — | API, LOCAL |
| mdformat | 1.0.0 | PyPI and apt | API, LOCAL |
| commitizen | 4.19.2 (MIT) | PyPI | API |
| shfmt / shellcheck | 3.12.0 / 0.11.0 | apt | LOCAL |
| yamlfmt | Apache-2.0; version not checked | GitHub (Go tool) | API (license only) |

Pre-commit `rev:` values and GitHub Action commit SHAs were **not** resolved. They stay
`TODO(verify)` in `repo-tooling.md` and must be resolved when the files are created.

## 12. Still unverified (carry into issues)

1. Lyrical's written C++ standard policy (REP 2000 text for Lyrical).
2. OpenUSD v26.08 building with GCC 15.2, CMake 4.2 and Python 3.14.
3. PhysX SDK version on `main`, and its build with GCC 15 and the CUDA toolkit in Resolute.
4. Whether Ubuntu's Slang 2026.1.1 has the CUDA target enabled.
5. Modelon FMI Library 3.0.4 coverage of FMI 3.0 co-simulation; fmi4c status.
6. Redistribution terms for OpenModelica-generated FMUs.
7. imgui-node-editor against the current docking branch.
8. Gazebo Jetty license line and exact end-of-support date; MJX status; Genesis release cadence.
9. Vulkan ray-tracing extensions on driver 595 (run `vulkaninfo` on the GPU runner).
10. Whether `ros:lyrical` images already contain `mesa-vulkan-drivers`.
11. Whether the ROS buildfarm allows the network access and build time that `openusd_vendor`
    and `physx_vendor` need.
12. GitHub merge-queue availability for the chosen repository owner type and plan.

## Sources

- ROS 2 Lyrical Luth release announcement: <https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021>
- Lyrical release notes source: <https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst>
- Lyrical release page (blocked to automated fetch): <https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html>
- rosdistro: <https://github.com/ros/rosdistro> (`lyrical/distribution.yaml`, `rosdep/base.yaml`)
- Lyrical package sync 2026-08-07: <https://discourse.openrobotics.org/t/new-packages-and-patch-release-for-lyrical-2026-08-07/57230>
- Gazebo Jetty announcement: <https://discourse.openrobotics.org/t/gazebo-jetty-released/50349>
- Gazebo release notes and features: <https://gazebosim.org/docs/all/release_notes/>, <https://gazebosim.org/docs/all/release-features>
- Isaac Sim license FAQ: <https://docs.isaacsim.omniverse.nvidia.com/6.0.0/common/license-faq.html>
- Isaac Lab license: <https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/refs/license.html>
- Newton: <https://github.com/newton-physics/newton>, <https://pypi.org/project/newton/>
- PhysX GPU source release (secondary): <https://www.cgchannel.com/2025/04/nvidia-open-sources-physxs-gpu-simulation-code>
- PhysX repository: <https://github.com/NVIDIA-Omniverse/PhysX>
- OpenUSD: <https://github.com/PixarAnimationStudios/OpenUSD>; license rename: <https://forum.aousd.org/t/upcoming-openusd-license-update/1561>
- AOUSD Core Specification 1.0: <https://www.linuxfoundation.org/press/alliance-for-openusd-announces-core-specification-1.0-the-universal-language-for-building-3d-worlds>
- OpenUSD v26.08 release post: <https://forum.aousd.org/t/openusd-v26-08-has-been-released/3004>
- Slang: <https://github.com/shader-slang/slang>; Khronos announcement: <https://www.khronos.org/news/press/khronos-group-launches-slang-initiative-hosting-open-source-compiler-contributed-by-nvidia>
- OpenModelica FMI guide: <https://openmodelica.org/doc/OpenModelicaUsersGuide/latest/fmitlm.html>
- Modelon FMI Library: <https://github.com/modelon-community/fmi-library>
- Dear ImGui: <https://github.com/ocornut/imgui> (docking branch backends); Wayland multi-viewport issue #9370 (mirror): <https://upd.dev/ocornut/imgui/issues/9370>
- ImGui Test Engine license: <https://github.com/ocornut/imgui_test_engine/blob/main/imgui_test_engine/LICENSE.txt>
- ImPlot: <https://github.com/epezent/implot>; imgui-node-editor: <https://github.com/thedmd/imgui-node-editor>; imnodes: <https://github.com/Nelarius/imnodes>; ImGuizmo: <https://github.com/CedricGuillemet/ImGuizmo>
- SDL Wayland notes: <https://github.com/libsdl-org/SDL/blob/main/docs/README-wayland.md>
- PX4 ROS 2 guide: <https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros2/user_guide.md>
- ArduPilot ROS 2 README: <https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/README.md>
- JetPack downloads: <https://developer.nvidia.com/embedded/jetpack/downloads>
- GitHub Actions runner pricing: <https://docs.github.com/en/billing/reference/actions-runner-pricing>
- Git LFS billing: <https://docs.github.com/en/billing/concepts/product-billing/git-lfs>
- Docker Hub tags: <https://hub.docker.com/_/ros/tags>, <https://hub.docker.com/r/osrf/ros/tags>
- Tools: <https://github.com/BlankSpruce/gersemi>, <https://github.com/astral-sh/ruff>, <https://github.com/bufbuild/buf>, <https://github.com/googleapis/release-please>, <https://pypi.org/project/commitizen/>, <https://pypi.org/project/mdformat/>
