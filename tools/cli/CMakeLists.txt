cmake_minimum_required(VERSION 3.22)
project(lagrange VERSION 1.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# On the PATH once ROS is sourced: `lagrange` runs the server and the viewer together.
install(PROGRAMS lagrange DESTINATION bin)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_cli test/test_cli.py TIMEOUT 180)
endif()

ament_package()
