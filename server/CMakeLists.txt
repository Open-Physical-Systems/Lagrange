cmake_minimum_required(VERSION 3.22)
project(lagrange_server VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(lagrange_rigid_body_mujoco REQUIRED)
find_package(lagrange_rigid_body_substitutes REQUIRED)
find_package(lagrange_sensors REQUIRED)
find_package(lagrange_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
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
    src/Meshes.cpp
    src/DiffDrive.cpp
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
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_rigid_body_substitutes::lagrange_rigid_body_substitutes
        lagrange_sensors::lagrange_sensors
        lagrange_ros2::lagrange_ros2
        urdfdom::urdfdom_model
        tinyxml2::tinyxml2
        yaml-cpp::yaml-cpp
        mcap_vendor::mcap
)

add_executable(lagrange_server_main src/main.cpp)
set_target_properties(
    lagrange_server_main
    PROPERTIES OUTPUT_NAME lagrange_server
)
lagrange_target_defaults(lagrange_server_main)
target_link_libraries(
    lagrange_server_main
    PRIVATE lagrange_server ament_index_cpp::ament_index_cpp rclcpp::rclcpp
)

install(TARGETS lagrange_server_main RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
        test_server
        test/test_map_export.cpp
        test/test_profile.cpp
        test/test_recorder.cpp
        test/test_robot.cpp
        test/test_server.cpp
        test/test_toggles.cpp
        TIMEOUT 120
    )
    target_link_libraries(
        test_server
        lagrange_server
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        lagrange_sensors::lagrange_sensors
        mcap_vendor::mcap
        tinyxml2::tinyxml2
    )
    target_compile_definitions(
        test_server
        PRIVATE
            LAGRANGE_TEST_AMR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.urdf"
            LAGRANGE_TEST_SCHEMA="${CMAKE_CURRENT_SOURCE_DIR}/../profiles/schema.json"
            LAGRANGE_TEST_WAREHOUSE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/worlds/warehouse_small/warehouse_small.urdf"
            LAGRANGE_TEST_ARM_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/primitives_arm/primitives_arm.urdf"
    )
    lagrange_target_defaults(test_server)
endif()

ament_package()
