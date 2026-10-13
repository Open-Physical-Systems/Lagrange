cmake_minimum_required(VERSION 3.22)
project(lagrange_mock_server VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Only so that workspace-wide -DLAGRANGE_...=... options are declared for this package too.
find_package(lagrange_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/lagrange_mock_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    foreach(
        _test
        framing
        recording
        replay
        server
        urdf
    )
        ament_add_pytest_test(
            test_${_test}
            test/test_${_test}.py
            TIMEOUT 120
            APPEND_ENV
            PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
        )
    endforeach()
endif()

ament_package()
