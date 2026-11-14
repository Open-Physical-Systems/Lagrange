cmake_minimum_required(VERSION 3.22)
project(lagrange_example_holybro VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/watch.py DESTINATION lib/${PROJECT_NAME} RENAME watch)

ament_package()
