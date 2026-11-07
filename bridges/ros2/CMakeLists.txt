cmake_minimum_required(VERSION 3.22)
project(lagrange_ros2 VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(lagrange_ros2 src/Bridge.cpp src/SignalTopics.cpp)
lagrange_target_defaults(lagrange_ros2)
target_include_directories(
    lagrange_ros2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_ros2
    PUBLIC lagrange_core::lagrange_core
    PRIVATE
        rclcpp::rclcpp
        rmw::rmw
        ${rosgraph_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${simulation_interfaces_TARGETS}
        ${nav_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${tf2_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${ackermann_msgs_TARGETS}
        yaml-cpp::yaml-cpp
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_ros2
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # A domain of its own, on this machine only, so parallel tests do not hear each other.
    ament_add_gtest(
        test_bridge
        test/test_bridge.cpp
        ENV ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        TIMEOUT 60
    )
    target_link_libraries(
        test_bridge
        lagrange_ros2
        rclcpp::rclcpp
        ${rosgraph_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${simulation_interfaces_TARGETS}
        ${nav_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${ackermann_msgs_TARGETS}
    )
    lagrange_target_defaults(test_bridge)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    lagrange_core
    rclcpp
    rmw
    rosgraph_msgs
    sensor_msgs
    simulation_interfaces
    nav_msgs
    geometry_msgs
    tf2_msgs
    std_msgs
    ackermann_msgs
)
ament_package()
