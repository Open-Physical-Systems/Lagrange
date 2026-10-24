cmake_minimum_required(VERSION 3.22)
project(jolt_vendor VERSION 0.0.0 LANGUAGES NONE)

# Jolt Physics from its pinned release tarball (MIT; docs/design/dependencies.md).
find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(JOLT_VERSION 5.6.0)
set(JOLT_SHA256 6e069ee0172478cc78182047aac87e5310ba14a67a53348ae14cc37801fd3f8e)
set(JOLT_STAGE "${CMAKE_CURRENT_BINARY_DIR}/jolt_install")

# Release regardless of the workspace build type. Cross-platform deterministic (invariant 7).
# The profiler, debug renderer, asserts and floating-point exceptions are off: Jolt enables them
# per build configuration through public definitions that change its struct layouts, and a
# consumer in another configuration would then disagree with the library.
ExternalProject_Add(
    jolt
    URL "https://github.com/jrouwe/JoltPhysics/archive/refs/tags/v${JOLT_VERSION}.tar.gz"
    URL_HASH SHA256=${JOLT_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR Build
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${JOLT_STAGE}
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCROSS_PLATFORM_DETERMINISTIC=ON
        -DDOUBLE_PRECISION=OFF -DUSE_ASSERTS=OFF -DFLOATING_POINT_EXCEPTIONS_ENABLED=OFF
        -DDEBUG_RENDERER_IN_DEBUG_AND_RELEASE=OFF -DPROFILER_IN_DEBUG_AND_RELEASE=OFF
        -DINTERPROCEDURAL_OPTIMIZATION=OFF -DENABLE_ALL_WARNINGS=OFF -DENABLE_INSTALL=ON
        -DTARGET_UNIT_TESTS=OFF -DTARGET_HELLO_WORLD=OFF -DTARGET_PERFORMANCE_TEST=OFF
        -DTARGET_SAMPLES=OFF -DTARGET_VIEWER=OFF -DJPH_BUILD_SHARED_LIBS=OFF
        # No GPU compute (hair simulation): the rigid-body backend does not use it.
        -DJPH_USE_VK=OFF -DJPH_USE_DX12=OFF -DJPH_USE_MTL=OFF -DJPH_USE_CPU_COMPUTE=OFF
)

install(
    DIRECTORY ${JOLT_STAGE}/
    DESTINATION opt/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_package(CONFIG_EXTRAS cmake/jolt_vendor-extras.cmake)
