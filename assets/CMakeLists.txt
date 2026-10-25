cmake_minimum_required(VERSION 3.22)
project(lagrange_assets VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY fields propellers robots tyres worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
