cmake_minimum_required(VERSION 3.22)
project(lagrange_tutorial VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_rigid_body_mujoco REQUIRED)
find_package(lagrange_server REQUIRED)
find_package(lagrange_electrical REQUIRED)
find_package(ament_index_cpp REQUIRED)

# The module of docs/guide/writing-a-module.md.
add_library(lagrange_tutorial_odometer STATIC src/Odometer.cpp)
lagrange_target_defaults(lagrange_tutorial_odometer)
target_include_directories(
    lagrange_tutorial_odometer
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
    lagrange_tutorial_odometer
    PUBLIC lagrange_core::lagrange_core
)

add_executable(odometer_demo src/odometer_demo.cpp)
lagrange_target_defaults(odometer_demo)
target_link_libraries(
    odometer_demo
    PRIVATE
        lagrange_tutorial_odometer
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_server::lagrange_server
        lagrange_electrical::lagrange_electrical
        ament_index_cpp::ament_index_cpp
)
install(TARGETS odometer_demo DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_odometer test/test_odometer.cpp TIMEOUT 120)
    target_link_libraries(
        test_odometer
        lagrange_tutorial_odometer
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_server::lagrange_server
        lagrange_electrical::lagrange_electrical
    )
    target_compile_definitions(
        test_odometer
        PRIVATE
            LAGRANGE_TEST_AMR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../../assets/robots/amr_diffdrive/amr_diffdrive.urdf"
    )
    lagrange_target_defaults(test_odometer)
endif()

ament_package()
