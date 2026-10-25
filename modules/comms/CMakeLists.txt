cmake_minimum_required(VERSION 3.22)
project(lagrange_comms VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)

add_library(lagrange_comms src/Comms.cpp)
lagrange_target_defaults(lagrange_comms)
target_include_directories(
    lagrange_comms
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_comms PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_comms
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_comms test/test_comms.cpp)
    target_link_libraries(test_comms lagrange_comms)
    lagrange_target_defaults(test_comms)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
