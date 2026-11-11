cmake_minimum_required(VERSION 3.22)
project(lagrange_example_camera VERSION 1.2.1 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_camera.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_camera
)

ament_package()
