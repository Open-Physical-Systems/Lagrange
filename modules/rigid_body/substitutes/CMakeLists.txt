cmake_minimum_required(VERSION 3.22)
project(lagrange_rigid_body_substitutes VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(lagrange_rigid_body_substitutes src/FixedRobot.cpp)
add_library(
    lagrange::rigid_body_substitutes
    ALIAS lagrange_rigid_body_substitutes
)
lagrange_target_defaults(lagrange_rigid_body_substitutes)
target_include_directories(
    lagrange_rigid_body_substitutes
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_rigid_body_substitutes
    PUBLIC lagrange_core::lagrange_core
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_rigid_body_substitutes
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_fixed_robot test/test_fixed_robot.cpp)
    target_link_libraries(test_fixed_robot lagrange_rigid_body_substitutes)
    lagrange_target_defaults(test_fixed_robot)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
