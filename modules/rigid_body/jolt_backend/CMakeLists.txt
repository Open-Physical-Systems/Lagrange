cmake_minimum_required(VERSION 3.22)
project(lagrange_rigid_body_jolt VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(jolt_vendor REQUIRED)

add_library(lagrange_rigid_body_jolt src/JoltBackend.cpp)
lagrange_target_defaults(lagrange_rigid_body_jolt)
target_include_directories(
    lagrange_rigid_body_jolt
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_rigid_body_jolt
    PUBLIC lagrange_core::lagrange_core
    PRIVATE Jolt::Jolt
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_rigid_body_jolt
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_server REQUIRED)
    find_package(lagrange_electrical REQUIRED)
    ament_add_gtest(test_jolt test/test_jolt.cpp TIMEOUT 120)
    target_link_libraries(
        test_jolt
        lagrange_rigid_body_jolt
        lagrange_server::lagrange_server
        lagrange_electrical::lagrange_electrical
    )
    target_compile_definitions(
        test_jolt
        PRIVATE
            LAGRANGE_TEST_AMR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../../../assets/robots/amr_diffdrive/amr_diffdrive.urdf"
    )
    lagrange_target_defaults(test_jolt)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core jolt_vendor)
ament_package()
