cmake_minimum_required(VERSION 3.22)
project(lagrange_recording VERSION 0.0.0 LANGUAGES CXX)

# Recordings (MCAP, docs/design/formats.md §7): reading them, and the replay substitute.
find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(mcap_vendor REQUIRED)

add_library(lagrange_recording src/Recording.cpp)
lagrange_target_defaults(lagrange_recording)
target_include_directories(
    lagrange_recording
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_recording
    PUBLIC lagrange_core::lagrange_core
    PRIVATE lagrange_proto::lagrange_proto mcap_vendor::mcap
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_recording
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_recording test/test_recording.cpp)
    target_link_libraries(
        test_recording
        lagrange_recording
        lagrange_proto::lagrange_proto
        mcap_vendor::mcap
    )
    lagrange_target_defaults(test_recording)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_proto mcap_vendor)
ament_package()
