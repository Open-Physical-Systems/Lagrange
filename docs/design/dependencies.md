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
| MuJoCo | rigid body, MJCF import (`lagrange_importers` reads the compiled `mjModel`) | `ros-lyrical-mujoco-vendor`, rosdep `mujoco_vendor` (decided in spike 0.2-C1) | 3.12.0 (vendor), 3.5.0 (apt, not used), 3.15.0 (upstream) | Apache-2.0; bundled third-party code MIT, BSD, Apache-2.0 with LLVM exception, OFL (verified from the package's `LICENSE` and `THIRD_PARTY_NOTICES`, 2026-10-14) | Low | One source per build (Q9). Validation: `docs/plan/spikes/0.2-C1-mujoco.md` |
| Jolt Physics | rigid body (second backend; spike 0.2-C6) | `jolt_vendor` (release tarball, SHA-256 pinned) | v5.6.0 (latest release, 2026-07-11) | MIT (checked on GitHub, 2026-10-25) | Low | Plain CMake, no dependencies; GPU compute (hair) disabled in the vendor build |
| PhysX 5 | rigid body (vehicles, GPU) | `physx_vendor` | SDK version TODO(verify) | BSD-3-Clause | **High** | Custom build system (`generate_projects`), GPU part needs the CUDA toolkit; GCC 15 compatibility TODO(verify). Optional |
| Embree | CPU ray casting: Tier 0 lidar culling (`ShapeCaster`, `lagrange_sensors`); later radar, dose, comms | apt `libembree-dev` | 4.4.0 | Apache-2.0 (checked in `/usr/share/doc/libembree-dev/copyright` of 4.4.0+dfsg-1, 2026-10-24) | Low | rosdep key `embree` |
| TBB | Embree, OpenVDB, worker pool; parallel lidar beams (`lagrange_sensors`) | apt `libtbb-dev` (rosdep `tbb`) | 2022.3.0 | Apache-2.0 for the library; GPL only for one example's resource file and Debian's packaging (checked in `/usr/share/doc/libtbb-dev/copyright` of 2022.3.0-2, 2026-10-24) | Low | |
| OpenVDB / NanoVDB | fields (`lagrange_fields`: NanoVDB only so far) | apt `libnanovdb-dev` (rosdep `libnanovdb-dev`); `libopenvdb-dev` not used yet | 10.0.1 | MPL-2.0 (checked: `/usr/share/doc/libnanovdb-dev/copyright` of 10.0.1-3 and the headers' SPDX lines, 2026-10-25; Apache-2.0 came with later upstream releases). Used unmodified as headers, which MPL-2.0 allows in an Apache-2.0 project | Medium | Three majors behind upstream (R10). Header-only, so a newer copy can be vendored alone if needed. The build also finds a local copy in `~/.local/nanovdb/include` until the package is installed |
| FMI import library | `modules/electrical/fmu_host` | `fmilib_vendor` (Modelon FMI Library) | 3.0.4 | BSD-3-Clause-style | Medium | FMI 3.0 co-simulation coverage TODO(verify). Alternative: a thin in-house loader over the FMI 3.0 C headers (the headers are BSD-2-Clause; TODO(verify)) |
| MCAP C++ | recording, replay, mock server | `ros-lyrical-mcap-vendor` | 0.33.3 | MIT | Low | Header-only library plus zstd/lz4 |
| Gymnasium | optional base of `lagrange.envs.LagrangeEnv`; never required, checked against 1.4.0 | PyPI, not a package dependency | 1.4.0 | MIT | Low | `check_env` passes; numpy stands in when it is not installed |
| MCAP Python (`mcap`) | mock server | PyPI, through the rosdep key `python3-mcap-pip` in `packaging/rosdep/lagrange.yaml`; version pinned in `packaging/pip-constraints.txt` | 1.5.0 | MIT (repository license; PyPI metadata has no license field) | Low | Not packaged for Ubuntu. Pulls `lz4` and `zstandard` from PyPI. Only the mock server needs it, which becomes a test tool in v0.2 |
| zstd, lz4 | MCAP, protocol compression | apt | 1.5.7, 1.10.0 | BSD / BSD-2-Clause (TODO(verify)) | Low | |
| Protobuf | protocol, recordings | apt `libprotobuf-dev`, `protobuf-compiler` | 3.21.12 | BSD-3-Clause (TODO(verify)) | Low | Old: proto3 only, no editions. rosdep key `protobuf-dev` |
| gRPC | control API (v0.2+) | apt `libgrpc++-dev` | 1.51.1 | Apache-2.0 (TODO(verify)) | Low | rosdep key TODO(verify) |
| Zenoh (zenoh-c, zenoh-cpp) | optional transport, bridge | apt `libzenohc-dev`, `libzenohcpp-dev` | 1.8.0 | Apache-2.0 OR EPL-2.0 | Low | `ros-lyrical-zenoh-cpp-vendor` 0.10.6 also exists; avoid loading two copies in one process (TODO(verify) symbol conflicts) |
| FMI Library | FMU host (`lagrange_electrical`, `FmuHost.cpp`); FMI 3 headers for the reference FMU (`lagrange_models`) | `fmilibrary3_vendor` (release tarball, SHA-256 pinned; static, with its bundled Expat, zlib and minizip) | 3.0.4 (latest release, 2025-06-25; checked on GitHub 2026-10-25) | BSD-3-Clause (Modelon, `LICENSE.md`); bundled Expat MIT, zlib and minizip zlib, FMI 3.0 headers BSD-2-Clause; Catch2 (BSL-1.0) only in its tests, not built | Low | Lyrical's `fmilibrary_vendor` (Bosch Research; source entry in rosdistro, no Lyrical release) pins FMI Library 2.2.3, which reads FMI 1.0 and 2.0 only, so it does not serve the FMI 3.0 format stack |
| yaml-cpp | profiles (`lagrange_server`), overlay files (`lagrange_stage`), FMU maps (`lagrange_electrical`) | apt `libyaml-cpp-dev`, rosdep `yaml-cpp` | 0.8.0 | X11, an MIT-style licence (Ubuntu copyright file; only Debian's packaging files are GPL-2.0+) | Low | |
| jsoncons (JSON Schema validator, C++) | profile validation in `lagrange_server` (RFC 0005 resolution 7) | apt `libjsoncons-dev`, rosdep key `libjsoncons-dev` in `packaging/rosdep/lagrange.yaml` | 1.5.0 | BSL-1.0 (Ubuntu copyright file) | Low | Header-only; supports JSON Schema 2020-12 including `unevaluatedProperties`, which `profiles/schema.json` uses. valijson was rejected: no 2020-12 support, so unknown keys would pass. The mock server keeps `python3-jsonschema` |
| nlohmann-json | JSON handling | apt `nlohmann-json3-dev` | 3.11.3-based | MIT (TODO(verify)) | Low | |
| Eigen | maths | apt `libeigen3-dev` | 3.4.0 | MPL-2.0 | Low | Header-only |
| fmt, spdlog | logging | apt | 10.1.1, 1.15.3 | MIT | Low | |
| xxHash | state hashes, content keys | apt `libxxhash-dev` | 0.8.3 | BSD-2-Clause | Low | |
| CLI11 | command-line parsing | apt `libcli11-dev` | 2.6.1 | BSD-3-Clause | Low | |
| urdfdom / `urdf`, xacro | URDF import; `lagrange_server` loads URDF with urdfdom until the USD importer exists | ROS Lyrical (rosdep `urdfdom`) | urdfdom 6.0.0, xacro 2.1.1 | urdfdom BSD (package `copyright`, verified 2026-10-14); xacro TODO(verify) | Low | |
| tinyxml2 | `lagrange_server`: reads the `<ros2_control>` block that urdfdom ignores; `lagrange_importers`: tells URDF, SDFormat and MJCF apart by their root element | apt `libtinyxml2-dev` (rosdep `tinyxml2`) | 11.0.0 | zlib/libpng for the library; Debian packaging files GPL-2+, not linked (verified 2026-10-14) | Low | |
| rclcpp, rmw and the message packages (rosgraph_msgs, sensor_msgs, nav_msgs, geometry_msgs, tf2_msgs, std_msgs) | `lagrange_ros2` bridge | ROS Lyrical | as installed | Apache-2.0 (ROS 2 core) | Low | The bridge links them; the core does not |
| ament_index_cpp | `lagrange_server`: finds the installed sample robot and profile schema | ROS Lyrical | as installed | Apache-2.0 (package `copyright`, verified 2026-10-14) | Low | |
| SDFormat (libsdformat) | SDF import (`lagrange_importers`) | `ros-lyrical-sdformat-vendor`, rosdep `sdformat_vendor` | 0.3.4 (vendor package) of sdformat 16.1.0 | Apache-2.0; `src/urdf/*` (the URDF parser inside) BSD-3-Clause (verified from the Debian `copyright` of libsdformat16 and the vendor's `package.xml`, 2026-10-17) | Low | Links gz-math and gz-utils (Apache-2.0, through the same vendor chain) |
| Assimp | mesh import (`lagrange_meshes`) | apt `libassimp-dev` (rosdep `assimp`) | 6.0.4 | BSD-3-Clause; bundled parts MIT, ISC, BSL-1.0, Apache-2.0, public domain; GPL-3+ only for Debian packaging scripts (checked in `/usr/share/doc/libassimp-dev/copyright` of 6.0.4+ds-1build1, 2026-10-18) | Low | Import-time only |
| Google Benchmark | performance benchmarks (`lagrange_benchmarks`) | apt `libbenchmark-dev` (rosdep `benchmark`) | 1.9.1 | Apache-2.0 (checked in `/usr/share/doc/libbenchmark-dev/copyright` of 1.9.1-1build1, 2026-10-24) | Low | Benchmarks only, not in the product |
| nanobind | Python bindings (`lagrange_python`) | apt `nanobind-dev` (rosdep `nanobind-dev`) | 2.11.0 | BSD-3-Clause (checked in `/usr/share/doc/nanobind-dev/copyright`, 2026-10-25) | Low | Build time only |
| CoACD | collision bake tool | `coacd_vendor` | TODO(verify) | MIT | Medium | Offline tool; V-HACD (BSD-3-Clause) is the alternative |
| libpng | texture decoding in mesh import (`lagrange_meshes`: textures baked into colour parts) | apt `libpng-dev` (rosdep `libpng-dev`) | 1.6.57 | libpng licence (permissive; `/usr/share/doc/libpng-dev/copyright`, verified 2026-10-29) | Low | Import-time only, linked privately |
| ambientCG textures | `gpu/assets_textures`: photographs of surfaces for the realistic renderer | committed, 1024 px JPG (`SOURCES.md`) | sets named there, fetched 2026-11-01 | CC0 1.0 (docs.ambientcg.com/license, verified 2026-11-01) | Low | Files, not code; the view draws procedural surfaces without them |
| libjpeg-turbo | the same, for JPEG textures | apt `libjpeg-turbo8-dev` (rosdep `libjpeg`) | 2.1.5 | IJG (BSD-like), BSD-3-Clause and Zlib (`/usr/share/doc/libjpeg-turbo8-dev/copyright`, verified 2026-10-29) | Low | Import-time only, linked privately |
| OpenSSL | TLS for the viewer protocol (`lagrange_proto` transport, `tls://` endpoints) | apt `libssl-dev` (rosdep `libssl-dev`) | 3.5.5 | Apache-2.0 (`/usr/share/doc/libssl-dev/copyright`, verified 2026-10-29) | Low | Linked privately by the transport |
| GDAL, PROJ | elevation models and coordinate reference systems (`lagrange_geospatial`: `lagrange_build_world`) | apt `libgdal-dev` (rosdep `libgdal-dev`), PROJ through it | 3.12.2, 9.7.1 | Expat (MIT) for both, from `/usr/share/doc/libgdal-dev/copyright` and `/usr/share/doc/libproj-dev/copyright` (verified 2026-10-29) | Low | Offline world building only; the simulator does not link them |
| proj-data (EGM96 geoid grid `egm96_15.gtx`) | orthometric to ellipsoidal heights for elevation models | apt `proj-data` (own rosdep key, `packaging/rosdep/lagrange.yaml`) | 9.7.1 | Expat for the package (`/usr/share/doc/proj-data/copyright`); the EGM96 grid is NGA's (US Government work) | Low | Read at world-building time |
| GeographicLib | `lagrange_core` tests (coordinate conversions) | apt `libgeographiclib-dev`, rosdep `geographiclib` | 2.7 | MIT (Expat) for the library; `kissfft.hh` BSD-3-Clause; Debian packaging files GPL-3+, not linked (verified in the package's `copyright` file, 2026-10-14) | Low | Test dependency only; later geospatial tools |
| nanobind | Python bindings | apt `nanobind-dev` | 2.11.0 | BSD-3-Clause | Low | rosdep key `nanobind-dev` |
| mp-units | optional compile-time units inside modules | `ros-lyrical-mp-units-vendor` | 2.5.0 | MIT (TODO(verify)) | Low | Not used for the runtime `Unit` check (Q6) |

Open data the sample worlds are built from (downloaded at build time, never committed):

| Data | Used by | Licence | Notice |
|---|---|---|---|
| Copernicus DEM GLO-30 (AWS open data, `copernicus-dem-30m`) | `tools/ci/build_sample_world.sh` | Licence for Copernicus DEM instance COP-DEM-GLO-30-F (free; reproduction, distribution and adaptation allowed; read 2026-10-29) | "produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved" and the no-liability sentence of its article 6(c), both written to the world's ATTRIBUTION.md |
| OpenStreetMap (Overpass API) | the same | ODbL 1.0 | "© OpenStreetMap contributors", in ATTRIBUTION.md; the world is a produced work |
| Lexus RX 450h body mesh (Autoware `sample_vehicle_launch`, commit 93fb260) | `assets/robots/lexus_rx450h`, fetched by `fetch_assets` (not in this repository) | Apache-2.0 (the repository's LICENSE, fetched with it) | LICENSE and NOTICE next to the fetched files |
| Holybro X500 V2 meshes (PX4 `PX4-gazebo-models`, `x500_base`, commit e5997f4) | `assets/robots/holybro_x500_v2`, fetched by `fetch_assets` | BSD-3-Clause, © 2022 Rudis Laboratories (the model's LICENSE, fetched with it) | the same |
| AMR_FMS swerve AMR and warehouse (the owner's AMR_FMS project) | `assets/robots/amr_fms`, `assets/worlds/amr_fms_warehouse` (copied, 2.3 MB in all, no file over 1 MB) | Apache-2.0 (its `package.xml`), same author | — |

Separate processes and offline tools (not linked, not required to build):

| Tool | Role | Source | License | Notes |
|---|---|---|---|---|
| SUMO | traffic co-simulation over TraCI | apt `sumo` 1.25.0 | EPL-2.0 | Runtime-optional |
| OpenModelica | author Modelica models and export FMUs | upstream v1.27.1, not in apt | OSMC-PL | FMI 3.0 export is experimental; redistribution of generated FMUs needs a license review (R5) |
| ngspice | run exported netlists | apt `ngspice` 45.2 | BSD-style (TODO(verify)) | Offline |
| OpenFOAM (openfoam.com) | CFD, `lagrange foam`; produce wind fields | container `opencfd/openfoam-default:2512`, run v2512; v2606 is out, its image not yet seen | GPL-3.0 (openfoam.com/documentation/licencing) | A separate program, never linked or shipped; we write its input files and read its output |
| Geant4 | produce dose fields | not checked | Geant4 Software License (TODO(verify)) | Offline |
| PX4, ArduPilot SITL | autopilots in the loop | upstream PX4 v1.17.0, ArduPilot Copter-4.7.2, built on the host | BSD-3-Clause (PX4 `LICENSE`), GPL-3.0 (ArduPilot `COPYING.txt`) | Separate processes over TCP 4560 (PX4) and UDP 9002 (ArduPilot JSON); nothing of either is linked or copied; airframe and parameter files in `bridges/mavlink/` are our own |
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
libraries. v26.08 builds unpatched with GCC 15.2, CMake 4.2, system TBB 2022.3 and Python 3.14
on amd64: about 5 min without Python and 12 min with it on 16 cores (spike, 2026-10-14).

Plan:

1. **Minimal configuration.** `openusd_vendor` builds only what the simulator core needs:
   `usd`, `usdGeom`, `usdShade`, `usdPhysics`, `usdLux`, `sdf`, `ar`, `tf`, `vt`, `gf`, `plug`,
   `kind`, `pcp`, `work`, `trace`. Off: imaging (Hydra, `usdImaging`, `usdview`), OpenGL,
   OpenImageIO, OpenColorIO, Alembic, Draco, MaterialX (until needed), tests, tutorials,
   examples. Options in upstream CMake terms: `PXR_BUILD_IMAGING=OFF`,
   `PXR_BUILD_USD_IMAGING=OFF`, `PXR_BUILD_USDVIEW=OFF`, `PXR_BUILD_TESTS=OFF`,
   `PXR_BUILD_EXAMPLES=OFF`, `PXR_BUILD_TUTORIALS=OFF`, `PXR_ENABLE_PYTHON_SUPPORT` per step 3,
   plus `PXR_BUILD_EXEC=OFF` and `PXR_ENABLE_GL_SUPPORT=OFF` (option names verified in v26.08's
   `cmake/defaults/Options.cmake`; spike result in `docs/plan/spikes/0.2-B1-openusd.md`).
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
| `simulation_interfaces` | standard simulator services (`lagrange_ros2`) | 2.1.0 | Apache-2.0 |
| `hardware_interface`, `controller_manager`, `pluginlib` | `lagrange_ros2_control` | ros2_control 6.10.1, pluginlib 5.8.5 | Apache-2.0; pluginlib BSD |
| `diff_drive_controller`, `joint_state_broadcaster` | `lagrange_demos` (exec only); `forward_command_controller` in `lagrange_ros2_control`'s test | 6.9.0 | Apache-2.0 |
| `nav2_amcl` | `lagrange_demos` (exec only: AMCL localizes when ros2_control drives) | 1.5.1 | **LGPL-2.1-or-later**. Started as its own process by the demo launch file; nothing in the project links it, so no LGPL code is in core or any library we ship |
| `rosbag2_storage_mcap`, `mcap_vendor` | recording interoperability; `lagrange_server --record` writes MCAP with `mcap_vendor` | 0.33.3 | Apache-2.0 (`mcap_vendor` package); zstd BSD-3-Clause or GPL-2 (used under BSD), lz4 library BSD-2-Clause (Ubuntu copyright files); the MCAP C++ library MIT (foxglove/mcap LICENSE) |
| `rmw_fastrtps_cpp`, `rmw_zenoh_cpp` | tested RMWs | 9.4.10, 0.10.6 | |
| `nav2_map_server`, `nav2_planner`, `nav2_navfn_planner`, `nav2_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_waypoint_follower`, `nav2_lifecycle_manager`, `nav2_simple_commander` | `lagrange_demos` (exec only) | 1.5.1 | Apache-2.0 (map_server, navfn also BSD-3-Clause). Not `nav2_bringup`: not installed by default, and the demo starts the nodes itself |
| `robot_state_publisher`, `launch_ros` | `lagrange_demos` (exec only) | 3.5.6, 0.29.9 | BSD, Apache-2.0 |
| `mavlink` | `lagrange_mavlink` (PX4 SITL bridge): the generated C headers of `common.xml` | 2026.9.9 | The generator is (L)GPL-3; generated code embedded in a binary is MIT under the package's generator-output exception (`share/mavlink/COPYING.txt`), so no LGPL code is in what we ship. Header-only |
| PX4 Autopilot (not a build dependency) | the v0.4 mission: SITL built from source, v1.17.0 (the latest stable release, 13 May 2026) | v1.17.0 | BSD-3-Clause. Built on the host under `~/PX4-Autopilot` (`make px4_sitl_default`, Python deps from its `Tools/setup/requirements.txt`); run as its own process, never linked |
| `ackermann_msgs` | `lagrange_ros2` (`/ackermann_cmd` for road vehicles), `lagrange_autoware` | 2.0.2 | BSD (package.xml) |
| `autoware_control_msgs`, `autoware_vehicle_msgs` | `lagrange_autoware` (optional: built only when both are found, asked for with `LAGRANGE_AUTOWARE=on`) | `autoware_control_msgs` 1.11.0 as `ros-lyrical-autoware-control-msgs`; `autoware_vehicle_msgs` has no Lyrical binary (apt, 2026-10-25), so both are built from the `autoware_msgs` 1.11.0 tag in an underlay | Apache-2.0 (the repository's `LICENSE` at 1.11.0, checked 2026-10-25). Messages only; Autoware itself runs as its own processes (docs/guide/autoware.md) |
| Test-only: `moveit`, `ros2_controllers`, `mavros` | integration tests and demos | 2.15.2, 6.9.0, 2.15.1 | `<test_depend>` or demo packages only |

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
| ngspice | apt `ngspice` | 45.2 (Ubuntu 26.04) | BSD-3-clause (its sources; Debian copyright file) | Offline tool only, run as a separate program: the netlist round-trip test (`lagrange_electrical`'s `test_export`, skipped without it) and users' switching-level studies of exported netlists |
| CoACD | PyPI `coacd` (rosdep `python3-coacd-pip`) | 1.0.14 | MIT (the wheel's LICENSE). Its native library may bundle third-party code: TODO(verify) their licences | Offline tool only: `collision_bake` (`lagrange_collision_bake`), run by people preparing assets, never by the simulator |
| OpenCV | apt `libopencv-dev` | 4.10.0 (Ubuntu 26.04) | Apache-2.0 AND BSD-3-Clause (Debian copyright file) | Test only: the camera projection test (`lagrange_gpu`'s `test_raster`) finds checkerboard corners and calibrates with it |
| yamlfmt | Go binary | TODO(verify) | Apache-2.0 | Optional; yamllint alone may be enough |
| mdformat | apt | 1.0.0 | MIT | Replaces markdownlint (Node) |
| shfmt, shellcheck | apt | 3.12.0, 0.11.0 | BSD-3-Clause, GPL-3.0 (TODO(verify)) | Tools only |
| typos | pre-commit | TODO(verify) | MIT or Apache-2.0 (TODO(verify)) | |
| reuse | apt | 5.0.2 | mixed, includes GPL-3.0-or-later | Tool only |
| commitizen | PyPI via pre-commit | 4.19.2 | MIT | Replaces commitlint (Node) (Q13) |
| Sphinx | PyPI (`tools/docs/requirements.txt`) | 9.2.0 (2026-10-21) | BSD-2-Clause (PyPI, checked 2026-11-03) | Builds the documentation site (`docs/conf.py`, `tools/docs/build.sh`); not shipped. Chosen over Material for MkDocs, whose fixes end on 2026-11-05, and its successor Zensical, still alpha (0.0.74) |
| MyST-Parser | PyPI | 5.1.0 | MIT | The site's Markdown, math and heading anchors |
| Furo | PyPI | 2025.12.19 | MIT | The site's theme (pulls sphinx-basic-ng, beautifulsoup4, accessible-pygments) |
| Mermaid | jsDelivr, in the reader's browser | 11.12.1 | MIT | Draws the site's ```mermaid blocks, loaded by `docs/_static/mermaid-init.js` (one diagram at a time; sphinxcontrib-mermaid 2.1.1's own loader mixed diagrams up on pages with several). The math is MathJax (Apache-2.0), loaded by Sphinx the same way. Neither is in the product |
| release-please | GitHub Action | v17.11.2 | Apache-2.0 | Runs on GitHub's runners, not on developer machines |
| ccache, sccache | apt | 4.12.3, 0.13.0 | GPL-3.0, Apache-2.0 (TODO(verify)) | Tools only |
| libabigail (`abidiff`) | apt (TODO(verify)) | — | — | ABI checks from v0.9 |
| GoogleTest, Google Benchmark | apt or ROS vendor packages | 1.17.0, 1.9.1 | BSD-3-Clause, Apache-2.0 | |
| pytest, check-jsonschema | apt / PyPI | 9.0.2, TODO(verify) | MIT, Apache-2.0 (TODO(verify)) | |

## 7. Vendor packages to create

| Package | Upstream | Needed from | Build time class | Cache strategy |
|---|---|---|---|---|
| `imgui_vendor` | ocornut/imgui (docking), epezent/implot, thedmd/imgui-node-editor, CedricGuillemet/ImGuizmo | v0.1 | seconds | ccache |
| `openusd_vendor` | PixarAnimationStudios/OpenUSD | v0.2 | about 12 min with Python on 16 cores (spike), longer on CI runners | install-tree cache (`actions/cache`, keyed on the vendor package's CMake files) restored as a colcon underlay; prebuilt underlay asset still to do |
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
