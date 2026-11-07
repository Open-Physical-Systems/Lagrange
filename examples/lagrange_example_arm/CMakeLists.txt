cmake_minimum_required(VERSION 3.22)
project(lagrange_example_arm VERSION 1.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wave.py DESTINATION lib/${PROJECT_NAME} RENAME wave)

ament_package()
