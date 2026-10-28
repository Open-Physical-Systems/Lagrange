cmake_minimum_required(VERSION 3.22)
project(lagrange_meshes VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Assimp (BSD-3-Clause) reads glTF, OBJ, STL and Collada; Ubuntu's libassimp-dev.
find_package(assimp REQUIRED)
# libpng and libjpeg(-turbo) decode textures, which the importer bakes into colours.
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)

add_library(lagrange_meshes src/MeshImport.cpp src/Image.cpp)
lagrange_target_defaults(lagrange_meshes)
target_include_directories(
    lagrange_meshes
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_meshes
    PUBLIC lagrange_core::lagrange_core
    PRIVATE assimp::assimp ament_index_cpp::ament_index_cpp PNG::PNG JPEG::JPEG
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_meshes
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_meshes test/test_meshes.cpp)
    target_link_libraries(test_meshes lagrange_meshes)
    target_compile_definitions(
        test_meshes
        PRIVATE
            LAGRANGE_TEST_MESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    )
    lagrange_target_defaults(test_meshes)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core assimp ament_index_cpp PNG JPEG)
ament_package()
