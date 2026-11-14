cmake_minimum_required(VERSION 3.22)
project(lagrange_tx16s VERSION 1.3.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(imgui_vendor REQUIRED)
find_package(lagrange_viewer REQUIRED) # the design's theme header and fonts
find_package(SDL3 REQUIRED)

# The radio's model: sticks, switches, keyboard, joystick map, the command. No SDL, no ROS.
add_library(lagrange_tx16s_radio STATIC src/Radio.cpp)
lagrange_target_defaults(lagrange_tx16s_radio)
target_include_directories(
    lagrange_tx16s_radio
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(lagrange_tx16s src/main.cpp src/RemoteUi.cpp)
lagrange_target_defaults(lagrange_tx16s)
target_include_directories(
    lagrange_tx16s
    PRIVATE ${lagrange_viewer_INCLUDE_DIRS}
)
target_link_libraries(
    lagrange_tx16s
    PRIVATE
        lagrange_tx16s_radio
        imgui_vendor::imgui
        SDL3::SDL3
        ament_index_cpp::ament_index_cpp
        rclcpp::rclcpp
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
)

install(TARGETS lagrange_tx16s RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_radio test/test_radio.cpp TIMEOUT 60)
    target_link_libraries(test_radio lagrange_tx16s_radio)
    lagrange_target_defaults(test_radio)
    # A USB radio as SDL sees it: a virtual joystick's axes through the map.
    ament_add_gtest(test_joystick test/test_joystick.cpp TIMEOUT 60)
    target_link_libraries(test_joystick lagrange_tx16s_radio SDL3::SDL3)
    lagrange_target_defaults(test_joystick)
endif()

ament_package()
