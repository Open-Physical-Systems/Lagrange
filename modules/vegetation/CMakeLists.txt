cmake_minimum_required(VERSION 3.22)
project(lagrange_vegetation VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(lagrange_vegetation src/Vegetation.cpp src/Leaves.cpp src/Branches.cpp)
lagrange_target_defaults(lagrange_vegetation)
target_include_directories(
    lagrange_vegetation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_vegetation PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_vegetation
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_vegetation test/test_vegetation.cpp TIMEOUT 120)
    target_link_libraries(test_vegetation lagrange_vegetation)
    lagrange_target_defaults(test_vegetation)
    ament_add_gtest(test_leaves test/test_leaves.cpp TIMEOUT 120)
    target_link_libraries(test_leaves lagrange_vegetation)
    lagrange_target_defaults(test_leaves)
    ament_add_gtest(test_branches test/test_branches.cpp TIMEOUT 120)
    target_link_libraries(test_branches lagrange_vegetation)
    lagrange_target_defaults(test_branches)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
