cmake_minimum_required(VERSION 3.22)
project(lagrange_example_weather VERSION 1.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_range.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_range
)

ament_package()
