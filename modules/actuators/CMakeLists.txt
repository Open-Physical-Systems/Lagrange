cmake_minimum_required(VERSION 3.22)
project(lagrange_actuators VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(
    lagrange_actuators
    src/Gripper.cpp
    src/Grippers.cpp
    src/Nozzle.cpp
    src/Nozzles.cpp
)
lagrange_target_defaults(lagrange_actuators)
target_include_directories(
    lagrange_actuators
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_actuators PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_actuators
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_gripper test/test_gripper.cpp TIMEOUT 120)
    target_link_libraries(test_gripper lagrange_actuators)
    lagrange_target_defaults(test_gripper)
    ament_add_gtest(test_nozzle test/test_nozzle.cpp TIMEOUT 120)
    target_link_libraries(test_nozzle lagrange_actuators)
    lagrange_target_defaults(test_nozzle)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
