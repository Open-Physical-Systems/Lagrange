cmake_minimum_required(VERSION 3.22)
project(openusd_vendor VERSION 0.0.0 LANGUAGES NONE)

# OpenUSD from its pinned release tarball, built with the system TBB and Python only (no
# build_usd.py, no downloaded dependencies), in the configuration of the OpenUSD spike
# (docs/plan/spikes/0.2-B1-openusd.md, docs/design/dependencies.md §4).
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
include(ExternalProject)
# Where OpenUSD installs its Python package (the env hook puts it on PYTHONPATH).
set(OPENUSD_PYTHON_DIR
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

set(OPENUSD_VERSION 26.08)
set(OPENUSD_SHA256
    4bccbb95cddda1dbeef2f74a08b9456352f2aa91bfd4578c0c613009c7950149
)
set(OPENUSD_STAGE "${CMAKE_CURRENT_BINARY_DIR}/openusd_install")

# Release regardless of the workspace build type: a Debug or sanitizer build of the project
# still links a Release USD (it is a dependency, not code under test).
ExternalProject_Add(
    openusd
    URL
        "https://github.com/PixarAnimationStudios/OpenUSD/archive/refs/tags/v${OPENUSD_VERSION}.tar.gz"
    URL_HASH SHA256=${OPENUSD_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${OPENUSD_STAGE}
        -DPXR_BUILD_MONOLITHIC=ON -DPXR_BUILD_USD_TOOLS=ON
        -DPXR_BUILD_IMAGING=OFF -DPXR_BUILD_USD_IMAGING=OFF
        -DPXR_BUILD_USDVIEW=OFF -DPXR_BUILD_EXEC=OFF -DPXR_ENABLE_GL_SUPPORT=OFF
        -DPXR_BUILD_TESTS=OFF -DPXR_BUILD_EXAMPLES=OFF -DPXR_BUILD_TUTORIALS=OFF
        -DPXR_ENABLE_PYTHON_SUPPORT=ON
        -DPython3_EXECUTABLE=${Python3_EXECUTABLE}
)

install(
    DIRECTORY ${OPENUSD_STAGE}/
    DESTINATION opt/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_environment_hooks(env-hooks/openusd_vendor.dsv.in)
ament_package(CONFIG_EXTRAS cmake/openusd_vendor-extras.cmake)
