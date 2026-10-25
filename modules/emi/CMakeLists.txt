cmake_minimum_required(VERSION 3.22)
project(lagrange_emi VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)

add_library(lagrange_emi src/Emi.cpp)
lagrange_target_defaults(lagrange_emi)
target_include_directories(
    lagrange_emi
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_emi
    PUBLIC lagrange_core::lagrange_core lagrange_fields::lagrange_fields
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_emi
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_emi test/test_emi.cpp)
    target_link_libraries(test_emi lagrange_emi)
    lagrange_target_defaults(test_emi)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_fields)
ament_package()
