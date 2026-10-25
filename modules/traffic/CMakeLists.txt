cmake_minimum_required(VERSION 3.22)
project(lagrange_traffic VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(lagrange_traffic src/Traffic.cpp)
lagrange_target_defaults(lagrange_traffic)
target_include_directories(
    lagrange_traffic
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_traffic PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_traffic
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_opendrive REQUIRED)
    find_package(lagrange_rigid_body_mujoco REQUIRED)
    ament_add_gtest(test_traffic test/test_traffic.cpp TIMEOUT 300)
    target_link_libraries(
        test_traffic
        lagrange_traffic
        lagrange_opendrive::lagrange_opendrive
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    target_compile_definitions(
        test_traffic
        PRIVATE
            LAGRANGE_TEST_CITY_XODR="${CMAKE_CURRENT_SOURCE_DIR}/../../assets/worlds/city_block/city_block.xodr"
    )
    lagrange_target_defaults(test_traffic)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
