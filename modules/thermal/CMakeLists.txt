cmake_minimum_required(VERSION 3.22)
project(lagrange_thermal VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(
    lagrange_thermal
    lumped/src/Lumped.cpp
    lumped/src/ThermalNetwork.cpp
    substitutes/src/AmbientTemperature.cpp
)
lagrange_target_defaults(lagrange_thermal)
target_include_directories(
    lagrange_thermal
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_thermal PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_thermal
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_lumped test/test_lumped.cpp)
    target_link_libraries(test_lumped lagrange_thermal)
    lagrange_target_defaults(test_lumped)
    find_package(lagrange_electrical REQUIRED)
    ament_add_gtest(test_thermal_coupling test/test_thermal_coupling.cpp TIMEOUT 120)
    target_link_libraries(
        test_thermal_coupling
        lagrange_thermal
        lagrange_electrical::lagrange_electrical
    )
    lagrange_target_defaults(test_thermal_coupling)
    ament_add_gtest(test_thermal_network test/test_thermal_network.cpp)
    target_link_libraries(test_thermal_network lagrange_thermal)
    lagrange_target_defaults(test_thermal_network)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
