cmake_minimum_required(VERSION 3.22)
project(lagrange_rigid_body_mujoco VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
# MuJoCo 3.12.0 from ros-lyrical-mujoco-vendor (docs/plan/spikes/0.2-C1-mujoco.md). Spike
# (docs/plan/spikes/mujoco-4.md): -DLAGRANGE_MUJOCO_PACKAGE=mujoco takes MuJoCo's own CMake package
# instead (a MuJoCo 4.x install on CMAKE_PREFIX_PATH); both give the target mujoco::mujoco.
set(LAGRANGE_MUJOCO_PACKAGE
    mujoco_vendor
    CACHE STRING
    "CMake package that provides mujoco::mujoco"
)
find_package(${LAGRANGE_MUJOCO_PACKAGE} REQUIRED)

# MuJoCo 3.12's mjsan.h switches to ASan stack instrumentation when it detects AddressSanitizer
# (__SANITIZE_ADDRESS__ or __has_feature). Under GCC its inline definitions do not compile, and the
# prebuilt library has no instrumented symbols anyway. In sanitizer builds the header is skipped
# through its include guard; nothing here uses its contents.
function(lagrange_skip_mujoco_asan_header target)
    if(LAGRANGE_SANITIZER MATCHES "address")
        target_compile_definitions(${target} PRIVATE MUJOCO_INCLUDE_MJSAN_H_)
    endif()
endfunction()

add_library(lagrange_rigid_body_mujoco src/MujocoBackend.cpp)
add_library(lagrange::rigid_body_mujoco ALIAS lagrange_rigid_body_mujoco)
lagrange_target_defaults(lagrange_rigid_body_mujoco)
lagrange_skip_mujoco_asan_header(lagrange_rigid_body_mujoco)
target_include_directories(
    lagrange_rigid_body_mujoco
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_rigid_body_mujoco
    PUBLIC lagrange_core::lagrange_core
    PRIVATE mujoco::mujoco
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_rigid_body_mujoco
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_mujoco_validation test/test_mujoco_validation.cpp)
    target_link_libraries(test_mujoco_validation mujoco::mujoco)
    lagrange_target_defaults(test_mujoco_validation)
    lagrange_skip_mujoco_asan_header(test_mujoco_validation)

    ament_add_gtest(test_mujoco_backend test/test_mujoco_backend.cpp)
    target_link_libraries(test_mujoco_backend lagrange_rigid_body_mujoco)
    lagrange_target_defaults(test_mujoco_backend)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core ${LAGRANGE_MUJOCO_PACKAGE})
ament_package()
