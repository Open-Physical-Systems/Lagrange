cmake_minimum_required(VERSION 3.22)
project(lagrange_fire VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)

add_library(lagrange_fire src/Correlations.cpp src/Fires.cpp src/Smoke.cpp)
lagrange_target_defaults(lagrange_fire)
target_include_directories(
    lagrange_fire
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_fire
    PUBLIC lagrange_core::lagrange_core lagrange_fields::lagrange_fields
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_fire
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_thermal REQUIRED)
    ament_add_gtest(test_fire_correlations test/test_correlations.cpp TIMEOUT 120)
    target_link_libraries(test_fire_correlations lagrange_fire)
    lagrange_target_defaults(test_fire_correlations)
    ament_add_gtest(test_fires test/test_fires.cpp TIMEOUT 300)
    target_link_libraries(
        test_fires
        lagrange_fire
        lagrange_thermal::lagrange_thermal
    )
    lagrange_target_defaults(test_fires)
    ament_add_gtest(test_smoke test/test_smoke.cpp TIMEOUT 300)
    target_link_libraries(test_smoke lagrange_fire)
    lagrange_target_defaults(test_smoke)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_fields)
ament_package()
