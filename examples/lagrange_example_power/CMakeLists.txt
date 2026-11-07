cmake_minimum_required(VERSION 3.22)
project(lagrange_example_power VERSION 1.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/check_power.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME check_power
)

ament_package()
