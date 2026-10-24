cmake_minimum_required(VERSION 3.22)
project(lagrange_electrical VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(
    lagrange_electrical
    substitutes/src/IdealTorqueSource.cpp
    network/src/Models.cpp
    network/src/Network.cpp
)
lagrange_target_defaults(lagrange_electrical)
target_include_directories(
    lagrange_electrical
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_electrical PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_electrical
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_ideal_torque_source test/test_ideal_torque_source.cpp)
    target_link_libraries(test_ideal_torque_source lagrange_electrical)
    lagrange_target_defaults(test_ideal_torque_source)
    ament_add_gtest(test_models test/test_models.cpp)
    target_link_libraries(test_models lagrange_electrical)
    lagrange_target_defaults(test_models)
    ament_add_gtest(test_coupling test/test_coupling.cpp TIMEOUT 300)
    target_link_libraries(test_coupling lagrange_electrical)
    lagrange_target_defaults(test_coupling)
    ament_add_gtest(test_network test/test_network.cpp TIMEOUT 120)
    target_link_libraries(test_network lagrange_electrical)
    lagrange_target_defaults(test_network)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
