cmake_minimum_required(VERSION 3.22)
project(lagrange_examples_check VERSION 1.3.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_examples test/test_examples.py TIMEOUT 900)
endif()

ament_package()
