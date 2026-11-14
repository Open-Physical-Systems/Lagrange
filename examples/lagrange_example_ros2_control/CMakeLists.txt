cmake_minimum_required(VERSION 3.22)
project(lagrange_example_ros2_control VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/send_trajectory.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME send_trajectory
)

ament_package()
