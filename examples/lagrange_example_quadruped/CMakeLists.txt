cmake_minimum_required(VERSION 3.22)
project(lagrange_example_quadruped VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trot.py DESTINATION lib/${PROJECT_NAME} RENAME trot)

ament_package()
