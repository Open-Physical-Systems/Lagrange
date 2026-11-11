cmake_minimum_required(VERSION 3.22)
project(lagrange_example_heat VERSION 1.2.1 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_heat.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_heat
)

ament_package()
