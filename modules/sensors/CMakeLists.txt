cmake_minimum_required(VERSION 3.22)
project(lagrange_sensors VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)
find_package(lagrange_gpu REQUIRED)
# Tier 0 ray casting: Embree's hierarchy culls, the exact tests stay ours (ShapeCaster).
find_package(embree 4 REQUIRED)
find_package(TBB REQUIRED)

add_library(
    lagrange_sensors
    src/Sensors.cpp
    src/ImuNoise.cpp
    src/ShapeCaster.cpp
    src/Wmm.cpp
    src/GeoSensors.cpp
    src/WeatherEffects.cpp
)
lagrange_target_defaults(lagrange_sensors)
target_include_directories(
    lagrange_sensors
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_sensors
    PUBLIC
        lagrange_core::lagrange_core
        lagrange_fields::lagrange_fields
        lagrange_gpu::lagrange_gpu_raycast
        lagrange_gpu::lagrange_gpu_raster
    PRIVATE embree TBB::tbb
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
    ament_add_gtest(test_shape_caster test/test_shape_caster.cpp TIMEOUT 120)
    target_link_libraries(test_shape_caster lagrange_sensors)
    lagrange_target_defaults(test_shape_caster)
    ament_add_gtest(test_wmm test/test_wmm.cpp)
    target_link_libraries(test_wmm lagrange_sensors)
    target_compile_definitions(
        test_wmm
        PRIVATE LAGRANGE_TEST_WMM_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/wmm2025"
    )
    lagrange_target_defaults(test_wmm)
    ament_add_gtest(test_geo_sensors test/test_geo_sensors.cpp)
    target_link_libraries(test_geo_sensors lagrange_sensors)
    lagrange_target_defaults(test_geo_sensors)
    ament_add_gtest(test_weather_effects test/test_weather_effects.cpp)
    target_link_libraries(test_weather_effects lagrange_sensors)
    lagrange_target_defaults(test_weather_effects)
    ament_add_gtest(test_imu_noise test/test_imu_noise.cpp TIMEOUT 120)
    target_link_libraries(test_imu_noise lagrange_sensors)
    lagrange_target_defaults(test_imu_noise)
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
    ament_add_gtest(
        test_camera
        test/test_camera.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_camera lagrange_sensors)
    lagrange_target_defaults(test_camera)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_gpu embree TBB)
ament_package()
