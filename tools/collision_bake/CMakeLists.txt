cmake_minimum_required(VERSION 3.22)
project(lagrange_collision_bake VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Only so that workspace-wide -DLAGRANGE_...=... options are declared for this package too.
find_package(lagrange_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/collision_bake DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(
        test_bake
        test/test_bake.py
        TIMEOUT 300
        APPEND_ENV
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

ament_package()
