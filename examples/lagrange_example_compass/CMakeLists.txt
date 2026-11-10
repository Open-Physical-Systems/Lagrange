cmake_minimum_required(VERSION 3.22)
project(lagrange_example_compass VERSION 1.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_compass.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_compass
)

ament_package()
