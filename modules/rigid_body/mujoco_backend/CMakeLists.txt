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
endif()

ament_package()
