cmake_minimum_required(VERSION 3.22)
project(lagrange_demos VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/patrol.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME patrol
)

ament_package()
