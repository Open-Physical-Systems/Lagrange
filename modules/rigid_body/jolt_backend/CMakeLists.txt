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

# lagrange_jolt: the same engine as a plugin through the C ABI (RFC 0008 §5), backend jolt_abi,
# found by `--plugin lagrange_jolt` through the lagrange_plugins resource index.
add_library(lagrange_jolt MODULE src/jolt_plugin.cpp)
lagrange_target_defaults(lagrange_jolt)
target_link_libraries(
    lagrange_jolt
    PRIVATE lagrange_rigid_body_jolt lagrange_core::lagrange_core
)
install(TARGETS lagrange_jolt LIBRARY DESTINATION lib)
ament_index_register_resource(
    lagrange_plugins
    PACKAGE_NAME lagrange_jolt
    CONTENT "lib/liblagrange_jolt.so"
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
    ament_add_gtest(test_jolt test/test_jolt.cpp TIMEOUT 120)
    target_link_libraries(test_jolt lagrange_rigid_body_jolt)
    lagrange_target_defaults(test_jolt)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core jolt_vendor)
ament_package()
