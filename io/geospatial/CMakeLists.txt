cmake_minimum_required(VERSION 3.22)
project(lagrange_geospatial VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_stage REQUIRED)
find_package(lagrange_meshes REQUIRED)
find_package(GDAL CONFIG REQUIRED)
find_package(tinyxml2 REQUIRED)
find_path(LAGRANGE_JSONCONS_INCLUDE_DIR jsoncons/json.hpp REQUIRED)

add_library(
    lagrange_geospatial
    src/Dem.cpp
    src/Osm.cpp
    src/Tiles3d.cpp
    src/WorldBuilder.cpp
)
lagrange_target_defaults(lagrange_geospatial)
target_include_directories(
    lagrange_geospatial
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(
    lagrange_geospatial
    SYSTEM
    PRIVATE ${LAGRANGE_JSONCONS_INCLUDE_DIR}
)
target_link_libraries(
    lagrange_geospatial
    PUBLIC lagrange_core::lagrange_core
    PRIVATE
        lagrange_stage::lagrange_stage
        lagrange_meshes::lagrange_meshes
        GDAL::GDAL
        tinyxml2::tinyxml2
)

add_executable(lagrange_build_world tools/build_world.cpp)
target_link_libraries(lagrange_build_world lagrange_geospatial)
lagrange_target_defaults(lagrange_build_world)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS lagrange_build_world DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_geospatial
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_geospatial test/test_geospatial.cpp TIMEOUT 300)
    target_link_libraries(
        test_geospatial
        lagrange_geospatial
        lagrange_stage::lagrange_stage
        GDAL::GDAL
    )
    lagrange_target_defaults(test_geospatial)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
