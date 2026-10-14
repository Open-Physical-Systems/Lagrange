cmake_minimum_required(VERSION 3.22)
project(lagrange_core VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)

# One library; each subdirectory is one part of the framework with its own include tree.
set(LAGRANGE_CORE_PARTS
    time
    signals
    random
    world
    world/coords
    module
    scheduler
)

add_library(
    lagrange_core
    time/src/SimTime.cpp
    signals/src/Unit.cpp
    signals/src/SignalBus.cpp
    random/src/Philox.cpp
    world/src/World.cpp
    world/src/Meshes.cpp
    world/coords/src/Frames.cpp
    module/src/Module.cpp
    scheduler/src/Scheduler.cpp
)
# xxHash is used header-only (XXH_INLINE_ALL) for state and topology hashes.
find_path(LAGRANGE_XXHASH_INCLUDE_DIR xxhash.h REQUIRED)
target_include_directories(
    lagrange_core
    SYSTEM
    PRIVATE ${LAGRANGE_XXHASH_INCLUDE_DIR}
)
add_library(lagrange::core ALIAS lagrange_core)
lagrange_target_defaults(lagrange_core)
foreach(part IN LISTS LAGRANGE_CORE_PARTS)
    target_include_directories(
        lagrange_core
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${part}/include>
    )
    install(DIRECTORY ${part}/include/ DESTINATION include/${PROJECT_NAME})
endforeach()
target_include_directories(
    lagrange_core
    PUBLIC $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(
    TARGETS lagrange_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(
        test
        IN
        ITEMS
            time/test/test_sim_time.cpp
            signals/test/test_unit.cpp
            random/test/test_philox.cpp
            scheduler/test/test_scheduler.cpp
    )
        get_filename_component(name ${test} NAME_WE)
        ament_add_gtest(${name} ${test})
        target_link_libraries(${name} lagrange_core)
        lagrange_target_defaults(${name})
    endforeach()

    # The coordinate conversions are checked against GeographicLib (tests only).
    list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
    find_package(GeographicLib REQUIRED)
    ament_add_gtest(test_frames world/coords/test/test_frames.cpp)
    target_link_libraries(test_frames lagrange_core ${GeographicLib_LIBRARIES})
    target_include_directories(
        test_frames
        SYSTEM
        PRIVATE ${GeographicLib_INCLUDE_DIRS}
    )
    lagrange_target_defaults(test_frames)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
