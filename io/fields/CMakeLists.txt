cmake_minimum_required(VERSION 3.22)
project(lagrange_fields VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

# NanoVDB is header-only (libnanovdb-dev, rosdep key libnanovdb-dev). A local copy of the
# headers serves until the package is installed (NANOVDB_INCLUDE_DIR or ~/.local/nanovdb).
find_path(
    NANOVDB_INCLUDE_DIR
    nanovdb/NanoVDB.h
    HINTS $ENV{HOME}/.local/nanovdb/include
    REQUIRED
)

add_library(lagrange_fields src/Field.cpp)
lagrange_target_defaults(lagrange_fields)
target_include_directories(
    lagrange_fields
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(
    lagrange_fields
    SYSTEM
    PRIVATE ${NANOVDB_INCLUDE_DIR}
)
target_link_libraries(
    lagrange_fields
    PUBLIC lagrange_core::lagrange_core
    PRIVATE Threads::Threads ZLIB::ZLIB
)
# NanoVDB's zlib codec: files are written compressed (the grid's internal tables are mostly
# zeros) and either kind is read.
target_compile_definitions(lagrange_fields PRIVATE NANOVDB_USE_ZIP)

add_executable(lagrange_make_fields tools/make_sample_fields.cpp)
target_link_libraries(lagrange_make_fields lagrange_fields)
lagrange_target_defaults(lagrange_make_fields)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS lagrange_make_fields DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_fields
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_fields test/test_fields.cpp)
    target_link_libraries(test_fields lagrange_fields)
    lagrange_target_defaults(test_fields)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core Threads ZLIB)
ament_package()
