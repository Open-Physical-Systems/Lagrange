cmake_minimum_required(VERSION 3.22)
project(lagrange_profiles VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(FILES schema.json DESTINATION share/${PROJECT_NAME})
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

ament_package()
