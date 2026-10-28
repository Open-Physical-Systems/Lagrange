cmake_minimum_required(VERSION 3.22)
project(lagrange_assets VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY fields propellers robots tyres worlds
    DESTINATION share/${PROJECT_NAME}
)

# The fetch tool for the large assets (external_assets.yaml): ros2 run lagrange_assets fetch_assets
install(FILES external_assets.yaml DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/fetch_assets.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME fetch_assets
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_external_assets test/test_external_assets.py TIMEOUT 60)
endif()

ament_package()
