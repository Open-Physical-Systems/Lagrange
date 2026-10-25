cmake_minimum_required(VERSION 3.22)
project(lagrange_stage VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(openusd_vendor REQUIRED)
find_package(lagrange_schemas REQUIRED)

add_library(lagrange_stage src/StageReader.cpp src/StageWriter.cpp)
add_library(lagrange::stage ALIAS lagrange_stage)
target_include_directories(
    lagrange_stage
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_stage
    PUBLIC lagrange_core::lagrange_core
    PRIVATE openusd_vendor::usd
)
lagrange_target_defaults(lagrange_stage)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_stage
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # The schema plugins from lagrange_schemas' install tree, as at run time.
    get_filename_component(schemas_share "${lagrange_schemas_DIR}/.." ABSOLUTE)
    ament_add_gtest(
        test_stage
        test/test_stage.cpp
        ENV
            "PXR_PLUGINPATH_NAME=${schemas_share}/usd/metadata:${schemas_share}/usd/sensor"
        TIMEOUT 120
    )
    target_link_libraries(test_stage lagrange_stage openusd_vendor::usd)
    lagrange_target_defaults(test_stage)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core openusd_vendor lagrange_schemas)
ament_package()
