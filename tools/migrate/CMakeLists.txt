cmake_minimum_required(VERSION 3.22)
project(lagrange_migrate VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Only so that workspace-wide -DLAGRANGE_...=... options are declared for this package too.
find_package(lagrange_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(
    PROGRAMS scripts/migrate_schema scripts/migrate_profile
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    foreach(_test schema profile)
        ament_add_pytest_test(
            test_${_test}
            test/test_${_test}.py
            TIMEOUT 120
            ENV
            LAGRANGE_REPO=${CMAKE_CURRENT_SOURCE_DIR}/../..
            APPEND_ENV
            PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
        )
    endforeach()
endif()

ament_package()
