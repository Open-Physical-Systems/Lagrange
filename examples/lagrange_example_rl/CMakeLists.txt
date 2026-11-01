cmake_minimum_required(VERSION 3.22)
project(lagrange_example_rl VERSION 1.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/train.py DESTINATION lib/${PROJECT_NAME} RENAME train)
ament_package()
