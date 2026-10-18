cmake_minimum_required(VERSION 3.22)
project(lagrange_sensors VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_gpu REQUIRED)

add_library(lagrange_sensors src/Sensors.cpp)
lagrange_target_defaults(lagrange_sensors)
target_include_directories(
    lagrange_sensors
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_sensors
    PUBLIC lagrange_core::lagrange_core lagrange_gpu::lagrange_gpu_raycast
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_sensors
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_rigid_body_mujoco REQUIRED)
    ament_add_gtest(test_sensors test/test_sensors.cpp)
    target_link_libraries(
        test_sensors
        lagrange_sensors
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    lagrange_target_defaults(test_sensors)
    # Runs on the software Vulkan device (llvmpipe) where there is no GPU; skipped without any.
    ament_add_gtest(
        test_lidar_tiers
        test/test_lidar_tiers.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_lidar_tiers lagrange_sensors)
    lagrange_target_defaults(test_lidar_tiers)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_gpu)
ament_package()
