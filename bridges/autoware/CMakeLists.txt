cmake_minimum_required(VERSION 3.22)
project(lagrange_autoware VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)

# Autoware's messages are optional (package.xml): without them there is nothing to build.
find_package(autoware_control_msgs QUIET)
find_package(autoware_vehicle_msgs QUIET)
if(NOT autoware_control_msgs_FOUND OR NOT autoware_vehicle_msgs_FOUND)
    message(
        STATUS
        "lagrange_autoware: autoware_control_msgs and autoware_vehicle_msgs not found; "
        "the Autoware interface is not built (docs/guide/autoware.md)"
    )
    ament_package()
    return()
endif()

find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(lagrange_autoware src/AutowareInterface.cpp)
lagrange_target_defaults(lagrange_autoware)
target_include_directories(
    lagrange_autoware
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
    lagrange_autoware
    PUBLIC
        rclcpp::rclcpp
        ${ackermann_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${autoware_control_msgs_TARGETS}
        ${autoware_vehicle_msgs_TARGETS}
)

add_executable(lagrange_autoware_interface src/main.cpp)
lagrange_target_defaults(lagrange_autoware_interface)
target_link_libraries(lagrange_autoware_interface lagrange_autoware)

install(TARGETS lagrange_autoware_interface DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # A domain of its own, on this machine only, so parallel tests do not hear each other.
    ament_add_gtest(
        test_autoware_interface
        test/test_autoware_interface.cpp
        ENV ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        TIMEOUT 60
    )
    target_link_libraries(test_autoware_interface lagrange_autoware)
    lagrange_target_defaults(test_autoware_interface)
endif()

ament_package()
