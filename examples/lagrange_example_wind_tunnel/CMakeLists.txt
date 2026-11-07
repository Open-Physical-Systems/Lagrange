cmake_minimum_required(VERSION 3.22)
project(lagrange_example_wind_tunnel VERSION 1.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_report.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_report
)
ament_package()
