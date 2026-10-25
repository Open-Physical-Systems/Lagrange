cmake_minimum_required(VERSION 3.22)
project(lagrange_opendrive VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_stage REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(lagrange_opendrive src/OpenDrive.cpp)
lagrange_target_defaults(lagrange_opendrive)
target_include_directories(
    lagrange_opendrive
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_opendrive
    PUBLIC lagrange_core::lagrange_core
    PRIVATE tinyxml2::tinyxml2
)

add_executable(lagrange_opendrive_import tools/opendrive_import.cpp)
target_link_libraries(
    lagrange_opendrive_import
    lagrange_opendrive
    lagrange_stage::lagrange_stage
)
lagrange_target_defaults(lagrange_opendrive_import)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS lagrange_opendrive_import DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_opendrive
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_opendrive test/test_opendrive.cpp)
    target_link_libraries(test_opendrive lagrange_opendrive)
    target_compile_definitions(
        test_opendrive
        PRIVATE
            LAGRANGE_TEST_CITY_XODR="${CMAKE_CURRENT_SOURCE_DIR}/../../assets/worlds/city_block/city_block.xodr"
    )
    lagrange_target_defaults(test_opendrive)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
