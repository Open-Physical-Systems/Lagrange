cmake_minimum_required(VERSION 3.22)
project(lagrange_example_car VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/drive_car.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME drive_car
)

ament_package()
