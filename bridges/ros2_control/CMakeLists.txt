cmake_minimum_required(VERSION 3.22)
project(lagrange_ros2_control VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tinyxml2 REQUIRED)

# Shared: the controller manager dlopens this library for the LagrangeSystem plugin, and the
# scheduler module in the same library must share its JointTable.
add_library(
    lagrange_ros2_control
    SHARED
    src/ControllerHost.cpp
    src/ControlModule.cpp
    src/LagrangeSystem.cpp
)
add_library(lagrange::ros2_control ALIAS lagrange_ros2_control)
lagrange_target_defaults(lagrange_ros2_control)
target_include_directories(
    lagrange_ros2_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_ros2_control
    PUBLIC lagrange_core::lagrange_core
    PRIVATE
        rclcpp::rclcpp
        controller_manager::controller_manager
        hardware_interface::hardware_interface
        pluginlib::pluginlib
        tinyxml2::tinyxml2
)
pluginlib_export_plugin_description_file(hardware_interface lagrange_ros2_control_plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_ros2_control
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(lagrange_electrical REQUIRED)
    ament_add_gtest(test_encoder test/test_encoder.cpp)
    target_include_directories(
        test_encoder
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    lagrange_target_defaults(test_encoder)
    ament_add_gtest(
        test_ros2_control
        test/test_ros2_control.cpp
        ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        TIMEOUT 120
    )
    target_link_libraries(
        test_ros2_control
        lagrange_ros2_control
        lagrange_electrical::lagrange_electrical
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
    )
    target_compile_definitions(
        test_ros2_control
        PRIVATE
            LAGRANGE_TEST_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/test/wheel_controllers.yaml"
    )
    lagrange_target_defaults(test_ros2_control)
    lagrange_test_ros_domain(test_ros2_control 74)
    # GPIO interfaces (RFC 0020 §5) driven by gpio_controllers' GpioCommandController.
    find_package(control_msgs REQUIRED)
    ament_add_gtest(
        test_gpio
        test/test_gpio.cpp
        ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        TIMEOUT 120
    )
    target_link_libraries(
        test_gpio
        lagrange_ros2_control
        rclcpp::rclcpp
        ${control_msgs_TARGETS}
    )
    target_compile_definitions(
        test_gpio
        PRIVATE
            LAGRANGE_TEST_GPIO_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/test/gpio_controllers.yaml"
    )
    lagrange_target_defaults(test_gpio)
    lagrange_test_ros_domain(test_gpio 75)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
