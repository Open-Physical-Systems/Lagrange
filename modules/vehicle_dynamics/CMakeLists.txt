cmake_minimum_required(VERSION 3.22)
project(lagrange_vehicle_dynamics VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(lagrange_vehicle_dynamics src/Tyre.cpp src/VehicleDynamics.cpp)
lagrange_target_defaults(lagrange_vehicle_dynamics)
target_include_directories(
    lagrange_vehicle_dynamics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_vehicle_dynamics
    PUBLIC lagrange_core::lagrange_core
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_vehicle_dynamics
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_tyre test/test_tyre.cpp)
    target_link_libraries(test_tyre lagrange_vehicle_dynamics)
    target_compile_definitions(
        test_tyre
        PRIVATE LAGRANGE_TEST_ASSETS="${CMAKE_CURRENT_SOURCE_DIR}/../../assets"
    )
    lagrange_target_defaults(test_tyre)
    find_package(lagrange_rigid_body_mujoco REQUIRED)
    find_package(lagrange_sensors REQUIRED)
    find_package(lagrange_electrical REQUIRED)
    ament_add_gtest(test_vehicle_dynamics test/test_vehicle_dynamics.cpp TIMEOUT 300)
    target_link_libraries(
        test_vehicle_dynamics
        lagrange_vehicle_dynamics
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_sensors::lagrange_sensors
        lagrange_electrical::lagrange_electrical
    )
    lagrange_target_defaults(test_vehicle_dynamics)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
