cmake_minimum_required(VERSION 3.22)
project(lagrange_aerodynamics VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)

add_library(lagrange_aerodynamics drag/src/Aerodynamics.cpp)
lagrange_target_defaults(lagrange_aerodynamics)
target_include_directories(
    lagrange_aerodynamics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_aerodynamics
    PUBLIC lagrange_core::lagrange_core lagrange_fields::lagrange_fields
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_aerodynamics
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_aerodynamics test/test_aerodynamics.cpp)
    target_link_libraries(test_aerodynamics lagrange_aerodynamics)
    lagrange_target_defaults(test_aerodynamics)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_fields)
ament_package()
