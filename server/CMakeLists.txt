cmake_minimum_required(VERSION 3.22)
project(lagrange_server VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(lagrange_rigid_body_jolt REQUIRED)
find_package(lagrange_rigid_body_mujoco REQUIRED)
find_package(lagrange_rigid_body_substitutes REQUIRED)
find_package(lagrange_sensors REQUIRED)
find_package(lagrange_electrical REQUIRED)
find_package(lagrange_thermal REQUIRED)
find_package(lagrange_rotor_aero REQUIRED)
find_package(lagrange_aerodynamics REQUIRED)
find_package(lagrange_weather REQUIRED)
find_package(lagrange_fields REQUIRED)
find_package(lagrange_emi REQUIRED)
find_package(lagrange_mavlink REQUIRED)
find_package(lagrange_recording REQUIRED)
find_package(lagrange_meshes REQUIRED)
find_package(lagrange_ros2 REQUIRED)
find_package(lagrange_stage REQUIRED)
find_package(lagrange_importers REQUIRED)
find_package(lagrange_ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lagrange_gpu REQUIRED)
# jsoncons (Boost Software License) validates profiles against profiles/schema.json, which is
# JSON Schema 2020-12; header-only, from Ubuntu's libjsoncons-dev.
find_path(
    LAGRANGE_JSONCONS_INCLUDE_DIR
    jsoncons_ext/jsonschema/jsonschema.hpp
    REQUIRED
)
find_package(mcap_vendor REQUIRED)

add_library(
    lagrange_server
    src/Server.cpp
    src/Robot.cpp
    src/DiffDrive.cpp
    src/Multicopter.cpp
    src/MapExport.cpp
    src/Profile.cpp
    src/Recorder.cpp
)
lagrange_target_defaults(lagrange_server)
target_include_directories(
    lagrange_server
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(
    lagrange_server
    SYSTEM
    PRIVATE ${LAGRANGE_JSONCONS_INCLUDE_DIR}
)
# GCC's flow analysis reports maybe-uninitialized inside jsoncons' bigint once its templates are
# inlined here; -isystem does not silence warnings at inlined locations. Only this file uses it.
set_source_files_properties(
    src/Profile.cpp
    PROPERTIES COMPILE_OPTIONS -Wno-maybe-uninitialized
)
target_link_libraries(
    lagrange_server
    PUBLIC lagrange_core::lagrange_core lagrange_proto::lagrange_proto_transport
    PRIVATE
        lagrange_rigid_body_jolt::lagrange_rigid_body_jolt
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_rigid_body_substitutes::lagrange_rigid_body_substitutes
        lagrange_sensors::lagrange_sensors
        lagrange_electrical::lagrange_electrical
        lagrange_thermal::lagrange_thermal
        lagrange_rotor_aero::lagrange_rotor_aero
        lagrange_aerodynamics::lagrange_aerodynamics
        lagrange_weather::lagrange_weather
        lagrange_fields::lagrange_fields
        lagrange_emi::lagrange_emi
        lagrange_mavlink::lagrange_mavlink
        lagrange_recording::lagrange_recording
        lagrange_meshes::lagrange_meshes
        lagrange_ros2::lagrange_ros2
        lagrange_stage::lagrange_stage
        lagrange_importers::lagrange_importers
        lagrange_ros2_control::lagrange_ros2_control
        urdfdom::urdfdom_model
        tinyxml2::tinyxml2
        yaml-cpp::yaml-cpp
        mcap_vendor::mcap
)

# The command line that lagrange_server and lagrange share.
add_library(lagrange_server_cli STATIC src/CommandLine.cpp)
lagrange_target_defaults(lagrange_server_cli)
target_link_libraries(
    lagrange_server_cli
    PUBLIC lagrange_server ament_index_cpp::ament_index_cpp rclcpp::rclcpp
)

add_executable(lagrange_server_main src/main.cpp)
set_target_properties(
    lagrange_server_main
    PROPERTIES OUTPUT_NAME lagrange_server
)
lagrange_target_defaults(lagrange_server_main)
target_link_libraries(lagrange_server_main PRIVATE lagrange_server_cli)

# `lagrange run|validate|graph|info` (roadmap 0.2-E7). The version is the package's.
file(
    STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    _lagrange_version_line
    REGEX "<version>"
)
string(
    REGEX REPLACE ".*<version>([^<]+)</version>.*"
    "\\1"
    LAGRANGE_PACKAGE_VERSION
    "${_lagrange_version_line}"
)
add_executable(lagrange_cli src/cli.cpp)
set_target_properties(lagrange_cli PROPERTIES OUTPUT_NAME lagrange)
lagrange_target_defaults(lagrange_cli)
target_compile_definitions(
    lagrange_cli
    PRIVATE
        LAGRANGE_VERSION="${LAGRANGE_PACKAGE_VERSION}"
        LAGRANGE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
target_link_libraries(
    lagrange_cli
    PRIVATE
        lagrange_server_cli
        lagrange_gpu::lagrange_gpu
        yaml-cpp::yaml-cpp
        lagrange_electrical::lagrange_electrical
)

add_executable(lagrange_import src/import_main.cpp)
lagrange_target_defaults(lagrange_import)
target_link_libraries(
    lagrange_import
    PRIVATE lagrange_server lagrange_stage::lagrange_stage
)

install(
    TARGETS lagrange_server_main lagrange_cli lagrange_import
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# The library is exported so that programs can build a run with their own modules until
# plugins load through the C ABI (docs/guide/writing-a-module.md).
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_server
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_schemas REQUIRED)
    find_package(lagrange_models REQUIRED)
    get_filename_component(models_share "${lagrange_models_DIR}/.." ABSOLUTE)
    get_filename_component(schemas_share "${lagrange_schemas_DIR}/.." ABSOLUTE)
    ament_add_gtest(
        test_server
        test/test_map_export.cpp
        test/test_profile.cpp
        test/test_backends.cpp
        test/test_recorder.cpp
        test/test_robot.cpp
        test/test_server.cpp
        test/test_multicopter.cpp
        test/test_toggles.cpp
        test/test_usd_assets.cpp
        ENV "PXR_PLUGINPATH_NAME=${schemas_share}/usd/metadata:${schemas_share}/usd/sensor:${schemas_share}/usd/electrical:${schemas_share}/usd/thermal:${schemas_share}/usd/aero:${schemas_share}/usd/weather:${schemas_share}/usd/emi:${schemas_share}/usd/sensor_material:${schemas_share}/usd/comms"
        TIMEOUT 120
    )
    target_link_libraries(
        test_server
        lagrange_server
        lagrange_electrical::lagrange_electrical
        lagrange_rotor_aero::lagrange_rotor_aero
        lagrange_rigid_body_jolt::lagrange_rigid_body_jolt
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_sensors::lagrange_sensors
        lagrange_stage::lagrange_stage
        mcap_vendor::mcap
        tinyxml2::tinyxml2
    )
    target_compile_definitions(
        test_server
        PRIVATE
            LAGRANGE_TEST_AMR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.urdf"
            LAGRANGE_TEST_SCHEMA="${CMAKE_CURRENT_SOURCE_DIR}/../profiles/schema.json"
            LAGRANGE_TEST_PROFILES="${CMAKE_CURRENT_SOURCE_DIR}/../profiles/examples"
            LAGRANGE_TEST_WAREHOUSE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/warehouse_small/warehouse_small.urdf"
            LAGRANGE_TEST_AMR_SDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.sdf"
            LAGRANGE_TEST_AMR_MJCF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.xml"
            LAGRANGE_TEST_ARM_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/primitives_arm/primitives_arm.urdf"
            LAGRANGE_TEST_AMR_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.usda"
            LAGRANGE_TEST_DC_DRIVE_FMU="${models_share}/fmu/dc_drive.fmu"
            LAGRANGE_TEST_AMR_OVERLAY="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.overlay.yaml"
            LAGRANGE_TEST_AMR_POWERED_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive_powered.usda"
            LAGRANGE_TEST_WAREHOUSE_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/warehouse_small/warehouse_small.usda"
            LAGRANGE_TEST_ARM6_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/arm6/arm6.urdf"
            LAGRANGE_TEST_ARM6_OVERLAY="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/arm6/arm6.overlay.yaml"
            LAGRANGE_TEST_RACK_BAKED_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/rack_sample/rack_baked.usda"
            LAGRANGE_TEST_WORKCELL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/workcell_arm6/workcell_arm6.urdf"
            LAGRANGE_TEST_QUAD_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/quad_x500/quad_x500.usda"
            LAGRANGE_TEST_QUAD_EMI_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/quad_x500/quad_x500_emi.usda"
            LAGRANGE_TEST_POWER_LINE_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/power_line/power_line.usda"
            LAGRANGE_TEST_QUAD_INVERTED_USDA="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/quad_x500_inverted/quad_x500_inverted.usda"
    )
    # The arm with ros2_control in the loop: the standard spawner and joint_trajectory_controller.
    find_package(trajectory_msgs REQUIRED)
    ament_add_gtest(
        test_arm
        test/test_arm.cpp
        ENV "PXR_PLUGINPATH_NAME=${schemas_share}/usd/metadata:${schemas_share}/usd/sensor:${schemas_share}/usd/electrical:${schemas_share}/usd/thermal:${schemas_share}/usd/aero:${schemas_share}/usd/weather:${schemas_share}/usd/emi:${schemas_share}/usd/sensor_material:${schemas_share}/usd/comms"
        TIMEOUT 180
    )
    target_link_libraries(
        test_arm
        lagrange_server
        rclcpp::rclcpp
        ${trajectory_msgs_TARGETS}
    )
    target_compile_definitions(
        test_arm
        PRIVATE
            LAGRANGE_TEST_ARM6_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/arm6/arm6.urdf"
            LAGRANGE_TEST_ARM6_OVERLAY="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/arm6/arm6.overlay.yaml"
            LAGRANGE_TEST_ARM6_CONTROLLERS="${CMAKE_CURRENT_SOURCE_DIR}/../demos/config/arm6_controllers.yaml"
            LAGRANGE_TEST_WORKCELL_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/workcell_arm6/workcell_arm6.urdf"
    )
    lagrange_target_defaults(test_arm)
    lagrange_target_defaults(test_server)
    # The installed tool end to end: exit 0 for the valid example, 1 for an invalid one.
    set(_lagrange_cli_robot
        --robot
        ${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.urdf
        --world
        ${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/warehouse_small/warehouse_small.urdf
    )
    add_test(
        NAME lagrange_cli_validate
        COMMAND
            $<TARGET_FILE:lagrange_cli> validate
            ${CMAKE_CURRENT_SOURCE_DIR}/../profiles/examples/amr_sample_urdf.yaml
            ${_lagrange_cli_robot}
    )
    add_test(
        NAME lagrange_cli_validate_rejects
        COMMAND
            $<TARGET_FILE:lagrange_cli> validate
            ${CMAKE_CURRENT_SOURCE_DIR}/../profiles/examples/amr_warehouse_nav2.yaml
            ${_lagrange_cli_robot}
    )
    set_tests_properties(
        lagrange_cli_validate_rejects
        PROPERTIES WILL_FAIL TRUE
    )
    add_test(
        NAME lagrange_cli_determinism
        COMMAND
            $<TARGET_FILE:lagrange_cli> determinism
            ${CMAKE_CURRENT_SOURCE_DIR}/../profiles/examples/amr_sample_urdf.yaml
            ${_lagrange_cli_robot} --seconds 3
    )
    set_tests_properties(
        lagrange_cli_determinism
        PROPERTIES PASS_REGULAR_EXPRESSION "deterministic: 31 state hashes"
    )
    add_test(
        NAME lagrange_cli_version
        COMMAND $<TARGET_FILE:lagrange_cli> info --version
    )
    set_tests_properties(
        lagrange_cli_version
        PROPERTIES PASS_REGULAR_EXPRESSION "^lagrange [0-9]+\\.[0-9]+\\.[0-9]+"
    )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_proto)
ament_package()
