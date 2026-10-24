cmake_minimum_required(VERSION 3.22)
project(fmilibrary3_vendor VERSION 0.0.0 LANGUAGES NONE)

# FMI Library 3.0.4 (Modelon, BSD-3-Clause; docs/design/dependencies.md) from its pinned release
# tarball: FMI 3.0 import for the FMU host. Lyrical's fmilibrary_vendor pins 2.2.3, which reads
# FMI 1.0 and 2.0 only, and the format stack's FMUs are FMI 3.0 (CLAUDE.md §4).
find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(FMIL_VERSION 3.0.4)
set(FMIL_SHA256
    99bb1381f67fbc521faf8229ddeb9ae47aaa9d764989dcbc9af296ec565cd34f
)
set(FMIL_STAGE "${CMAKE_CURRENT_BINARY_DIR}/fmil_install")

# A static library with its bundled Expat, zlib and minizip, position independent so that it
# links into our shared libraries. No tests, no documentation.
ExternalProject_Add(
    fmil
    URL
        "https://github.com/modelon-community/fmi-library/archive/refs/tags/${FMIL_VERSION}.tar.gz"
    URL_HASH SHA256=${FMIL_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    # Its test data (unbuilt) includes deliberately malformed XML, which colcon test-result
    # would otherwise report from this package's build directory.
    PATCH_COMMAND ${CMAKE_COMMAND} -E rm -rf <SOURCE_DIR>/Test
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${FMIL_STAGE}
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DFMILIB_BUILD_SHARED_LIB=OFF
        -DFMILIB_BUILD_TESTS=OFF -DFMILIB_GENERATE_DOXYGEN_DOC=OFF
)

install(
    DIRECTORY ${FMIL_STAGE}/
    DESTINATION opt/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_package(CONFIG_EXTRAS cmake/fmilibrary3_vendor-extras.cmake)
