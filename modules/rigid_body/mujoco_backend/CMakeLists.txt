cmake_minimum_required(VERSION 3.22)
project(lagrange_rigid_body_mujoco VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
# MuJoCo 3.12.0 from ros-lyrical-mujoco-vendor (docs/plan/spikes/0.2-C1-mujoco.md).
find_package(mujoco_vendor REQUIRED)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_mujoco_validation test/test_mujoco_validation.cpp)
    target_link_libraries(test_mujoco_validation mujoco::mujoco)
    lagrange_target_defaults(test_mujoco_validation)
    # MuJoCo 3.12's mjsan.h switches to ASan stack instrumentation when it detects AddressSanitizer
    # (__SANITIZE_ADDRESS__ or __has_feature). Under GCC its inline definitions do not compile, and
    # the prebuilt library has no instrumented symbols anyway. In sanitizer builds the header is
    # skipped through its include guard; nothing here uses its contents.
    if(LAGRANGE_SANITIZER MATCHES "address")
        target_compile_definitions(
            test_mujoco_validation
            PRIVATE MUJOCO_INCLUDE_MJSAN_H_
        )
    endif()
endif()

ament_package()
