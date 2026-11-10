cmake_minimum_required(VERSION 3.22)
project(lagrange_example_spawn VERSION 1.2.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/spawn_robot.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME spawn_robot
)

ament_package()
