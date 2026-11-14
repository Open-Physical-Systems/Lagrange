cmake_minimum_required(VERSION 3.22)
project(lagrange_example_sensors VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/check_sensors.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME check_sensors
)

ament_package()
