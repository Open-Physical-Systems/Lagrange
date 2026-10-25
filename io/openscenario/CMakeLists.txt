cmake_minimum_required(VERSION 3.22)
project(lagrange_openscenario VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(lagrange_openscenario src/Scenario.cpp)
lagrange_target_defaults(lagrange_openscenario)
target_include_directories(
    lagrange_openscenario
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_openscenario
    PUBLIC lagrange_core::lagrange_core
    PRIVATE tinyxml2::tinyxml2
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_openscenario
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_opendrive REQUIRED)
    ament_add_gtest(test_openscenario test/test_openscenario.cpp)
    target_link_libraries(
        test_openscenario
        lagrange_openscenario
        lagrange_opendrive::lagrange_opendrive
    )
    target_compile_definitions(
        test_openscenario
        PRIVATE
            LAGRANGE_TEST_CUT_IN="${CMAKE_CURRENT_SOURCE_DIR}/../../assets/scenarios/cut_in.xosc"
    )
    lagrange_target_defaults(test_openscenario)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core tinyxml2)
ament_package()
