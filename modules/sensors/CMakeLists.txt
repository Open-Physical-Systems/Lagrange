cmake_minimum_required(VERSION 3.22)
project(lagrange_sensors VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)
find_package(lagrange_gpu REQUIRED)
find_package(lagrange_thermal REQUIRED)
# Tier 0 ray casting: Embree's hierarchy culls, the exact tests stay ours (ShapeCaster).
find_package(embree 4 REQUIRED)
find_package(TBB REQUIRED)

add_library(
    lagrange_sensors
    src/Sensors.cpp
    src/DelayLine.cpp
    src/Faults.cpp
    src/ImuNoise.cpp
    src/ShapeCaster.cpp
    src/BodyScene.cpp
    src/Wmm.cpp
    src/GeoSensors.cpp
    src/Radar.cpp
    src/CameraModel.cpp
    src/Fisheye.cpp
    src/WeatherEffects.cpp
    src/ThermalCamera.cpp
    src/kinds/Kinds.cpp
    src/kinds/ImagingRadar.cpp
    src/kinds/CameraKinds.cpp
    src/kinds/EventCamera.cpp
    src/kinds/OpticalFlow.cpp
    src/kinds/StereoCamera.cpp
    src/kinds/BatteryMonitor.cpp
    src/kinds/EscTelemetry.cpp
    src/kinds/PowerMonitor.cpp
    src/kinds/TensionSensor.cpp
    src/kinds/EnvironmentSensors.cpp
    src/kinds/Airspeed.cpp
    src/kinds/Anemometer.cpp
    src/kinds/HumiditySensor.cpp
    src/kinds/LightSensor.cpp
    src/kinds/RainSensor.cpp
    src/kinds/Rangefinder.cpp
    src/kinds/SunSensor.cpp
    src/kinds/VisibilitySensor.cpp
    src/kinds/Bumper.cpp
    src/kinds/Cliff.cpp
    src/kinds/ForceTorque.cpp
    src/kinds/SafetyScanner.cpp
    src/kinds/WheelOdometry.cpp
    src/kinds/Detections.cpp
    src/kinds/LandingTarget.cpp
    src/kinds/LineSensor.cpp
    src/kinds/TagReader.cpp
    src/kinds/GripperSensor.cpp
    src/kinds/ParticleSensor.cpp
    src/kinds/Proximity.cpp
    src/kinds/Tpms.cpp
    src/kinds/Uwb.cpp
    src/kinds/WifiScanner.cpp
    src/kinds/AdsbReceiver.cpp
    src/kinds/Sonar.cpp
    src/kinds/V2x.cpp
    src/kinds/VehicleCan.cpp
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
    PRIVATE embree TBB::tbb lagrange_thermal::lagrange_thermal
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# The sensor catalog (RFC 0016 §5): data only, read by the robot builder.
install(
    FILES catalog/sensor_catalog.yaml
    DESTINATION share/${PROJECT_NAME}/catalog
)
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
    ament_add_gtest(test_faults test/test_faults.cpp)
    target_link_libraries(
        test_faults
        lagrange_sensors
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    lagrange_target_defaults(test_faults)
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
    ament_add_gtest(test_radar test/test_radar.cpp)
    target_link_libraries(
        test_radar
        lagrange_sensors
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    lagrange_target_defaults(test_radar)
    # Kind imaging_radar (sensors.md §2 "4D imaging radar"); its GPU case runs on the software
    # Vulkan device (llvmpipe) where there is no GPU, skipped without any device.
    ament_add_gtest(
        test_kinds_imaging_radar
        test/test_kinds_imaging_radar.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_kinds_imaging_radar lagrange_sensors)
    lagrange_target_defaults(test_kinds_imaging_radar)
    ament_add_gtest(test_camera_model test/test_camera_model.cpp)
    target_link_libraries(test_camera_model lagrange_sensors)
    lagrange_target_defaults(test_camera_model)
    ament_add_gtest(test_imu_noise test/test_imu_noise.cpp TIMEOUT 120)
    target_link_libraries(test_imu_noise lagrange_sensors)
    lagrange_target_defaults(test_imu_noise)
    # The built-in custom sensor kinds (sensors.md §9), each against closed forms.
    foreach(
        kind
        bumper
        cliff
        force_torque
        safety_scanner
        wheel_odometry
    )
        ament_add_gtest(test_kinds_${kind} test/test_kinds_${kind}.cpp TIMEOUT 120)
        target_link_libraries(
            test_kinds_${kind}
            lagrange_sensors
            lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        )
        lagrange_target_defaults(test_kinds_${kind})
    endforeach()
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
        test_lidar_patterns
        test/test_lidar_patterns.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_lidar_patterns lagrange_sensors)
    lagrange_target_defaults(test_lidar_patterns)
    ament_add_gtest(
        test_fisheye
        test/test_fisheye.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_fisheye lagrange_sensors)
    lagrange_target_defaults(test_fisheye)
    ament_add_gtest(
        test_camera
        test/test_camera.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_camera lagrange_sensors)
    lagrange_target_defaults(test_camera)
    ament_add_gtest(
        test_thermal_camera
        test/test_thermal_camera.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_thermal_camera lagrange_sensors)
    lagrange_target_defaults(test_thermal_camera)
    # The camera-like built-in kinds: closed forms without a GPU; the rendered runs on the
    # software Vulkan device (llvmpipe) where there is no GPU, skipped without any device.
    foreach(kind optical_flow event_camera stereo_camera)
        ament_add_gtest(
            test_kinds_${kind}
            test/test_kinds_${kind}.cpp
            TIMEOUT 120
            ENV
            ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
        )
        target_link_libraries(test_kinds_${kind} lagrange_sensors TBB::tbb)
        target_include_directories(test_kinds_${kind} PRIVATE src/kinds)
        lagrange_target_defaults(test_kinds_${kind})
    endforeach()
    # Built-in custom sensor kinds (sensors.md §9).
    # Built-in kinds (sensors.md §9) tested on a scheduler without physics or rendering. One
    # kind per line, sorted, so branches that add kinds merge cleanly.
    foreach(
        kind
        battery_monitor
        current_sensor
        esc_telemetry
        gripper_sensor
        landing_target
        line_sensor
        particle_sensor
        proximity
        tag_reader
        tension_sensor
        tpms
        uwb
        voltage_sensor
        wifi_scanner
    )
        ament_add_gtest(test_kinds_${kind} test/test_kinds_${kind}.cpp)
        target_link_libraries(test_kinds_${kind} lagrange_sensors)
        lagrange_target_defaults(test_kinds_${kind})
    endforeach()
    foreach(kind adsb_receiver sonar v2x vehicle_can)
        ament_add_gtest(test_kinds_${kind} test/test_kinds_${kind}.cpp)
        target_link_libraries(test_kinds_${kind} lagrange_sensors)
        lagrange_target_defaults(test_kinds_${kind})
    endforeach()
    ament_add_gtest(test_kinds_airspeed test/test_kinds_airspeed.cpp)
    target_link_libraries(test_kinds_airspeed lagrange_sensors)
    lagrange_target_defaults(test_kinds_airspeed)
    ament_add_gtest(test_kinds_anemometer test/test_kinds_anemometer.cpp)
    target_link_libraries(test_kinds_anemometer lagrange_sensors)
    lagrange_target_defaults(test_kinds_anemometer)
    ament_add_gtest(test_kinds_humidity_sensor test/test_kinds_humidity_sensor.cpp)
    target_link_libraries(test_kinds_humidity_sensor lagrange_sensors)
    lagrange_target_defaults(test_kinds_humidity_sensor)
    ament_add_gtest(test_kinds_light_sensor test/test_kinds_light_sensor.cpp)
    target_link_libraries(test_kinds_light_sensor lagrange_sensors)
    lagrange_target_defaults(test_kinds_light_sensor)
    ament_add_gtest(test_kinds_rain_sensor test/test_kinds_rain_sensor.cpp)
    target_link_libraries(test_kinds_rain_sensor lagrange_sensors)
    lagrange_target_defaults(test_kinds_rain_sensor)
    ament_add_gtest(test_kinds_rangefinder test/test_kinds_rangefinder.cpp)
    target_link_libraries(test_kinds_rangefinder lagrange_sensors)
    lagrange_target_defaults(test_kinds_rangefinder)
    ament_add_gtest(test_kinds_sun_sensor test/test_kinds_sun_sensor.cpp)
    target_link_libraries(test_kinds_sun_sensor lagrange_sensors)
    lagrange_target_defaults(test_kinds_sun_sensor)
    ament_add_gtest(test_kinds_visibility_sensor test/test_kinds_visibility_sensor.cpp)
    target_link_libraries(test_kinds_visibility_sensor lagrange_sensors)
    lagrange_target_defaults(test_kinds_visibility_sensor)
    # Renders on the software Vulkan device (llvmpipe) where there is no GPU; skipped without any.
    ament_add_gtest(
        test_kinds_detections
        test/test_kinds_detections.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_kinds_detections lagrange_sensors)
    lagrange_target_defaults(test_kinds_detections)
    # The sensor catalog against the code (RFC 0016 §5): the kinds are builtinKinds(), and every
    # default written explicitly gives the same signals as none.
    find_package(yaml-cpp REQUIRED)
    ament_add_gtest(
        test_sensor_catalog
        test/test_sensor_catalog.cpp
        test/catalog_cases_power_camera.cpp
        test/catalog_cases_environment.cpp
        test/catalog_cases_contact.cpp
        test/catalog_cases_scene_radio.cpp
        TIMEOUT 300
    )
    target_link_libraries(
        test_sensor_catalog
        lagrange_sensors
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        yaml-cpp::yaml-cpp
        TBB::tbb
    )
    target_include_directories(test_sensor_catalog PRIVATE src/kinds)
    target_compile_definitions(
        test_sensor_catalog
        PRIVATE
            LAGRANGE_SENSOR_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/catalog/sensor_catalog.yaml"
    )
    lagrange_target_defaults(test_sensor_catalog)
    # The catalog against sensors.md §9, both ways (tools/ci/check_sensor_catalog.py).
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_check_sensor_catalog test/test_check_sensor_catalog.py TIMEOUT 60)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_fields lagrange_gpu lagrange_thermal embree TBB)
ament_package()
