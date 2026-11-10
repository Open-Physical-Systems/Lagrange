cmake_minimum_required(VERSION 3.22)
project(lagrange_mavlink VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(mavlink REQUIRED)

add_library(
    lagrange_mavlink
    src/Px4Lockstep.cpp
    src/ArduPilotJson.cpp
    src/RotorTopSpeed.cpp
)
lagrange_target_defaults(lagrange_mavlink)
target_include_directories(
    lagrange_mavlink
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# The generated headers are C with casts and unused parameters that our warnings reject.
target_include_directories(
    lagrange_mavlink
    SYSTEM
    PRIVATE ${mavlink_INCLUDE_DIRS}/mavlink/v2.0
)
target_link_libraries(lagrange_mavlink PUBLIC lagrange_core::lagrange_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY px4 ardupilot DESTINATION share/${PROJECT_NAME})
install(
    TARGETS lagrange_mavlink
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_px4_lockstep test/test_px4_lockstep.cpp TIMEOUT 120)
    target_link_libraries(test_px4_lockstep lagrange_mavlink)
    target_include_directories(
        test_px4_lockstep
        SYSTEM
        PRIVATE ${mavlink_INCLUDE_DIRS}/mavlink/v2.0
    )
    lagrange_target_defaults(test_px4_lockstep)
    ament_add_gtest(test_ardupilot_json test/test_ardupilot_json.cpp TIMEOUT 120)
    target_link_libraries(test_ardupilot_json lagrange_mavlink)
    lagrange_target_defaults(test_ardupilot_json)
    ament_add_gtest(test_rotor_top_speed test/test_rotor_top_speed.cpp)
    target_link_libraries(test_rotor_top_speed lagrange_mavlink)
    lagrange_target_defaults(test_rotor_top_speed)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core)
ament_package()
