cmake_minimum_required(VERSION 3.22)
project(lagrange_core VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)

add_library(lagrange_core time/src/SimTime.cpp)
add_library(lagrange::core ALIAS lagrange_core)
lagrange_target_defaults(lagrange_core)
target_include_directories(
    lagrange_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/time/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(DIRECTORY time/include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_sim_time time/test/test_sim_time.cpp)
    target_link_libraries(test_sim_time lagrange_core)
    lagrange_target_defaults(test_sim_time)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
