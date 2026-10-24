cmake_minimum_required(VERSION 3.22)
project(lagrange_core VERSION 0.0.0 LANGUAGES C CXX)

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
    plugin_abi
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
    plugin_abi/src/PluginLibrary.cpp
)
# xxHash is used header-only (XXH_INLINE_ALL) for state and topology hashes.
find_path(LAGRANGE_XXHASH_INCLUDE_DIR xxhash.h REQUIRED)
target_include_directories(
    lagrange_core
    SYSTEM
    PRIVATE ${LAGRANGE_XXHASH_INCLUDE_DIR}
)
# dlopen for plugins built against the C ABI (core/plugin_abi).
target_link_libraries(lagrange_core PRIVATE ${CMAKE_DL_LIBS})
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

    # The plugin C ABI: a C99 sample (strict, so the header stays plain C), built twice: once
    # for this ABI major and once for another, which the host must refuse.
    foreach(variant IN ITEMS current other_major)
        add_library(
            sample_plugin_${variant}
            MODULE
            plugin_abi/samples/ideal_torque_source.c
        )
        target_include_directories(
            sample_plugin_${variant}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/plugin_abi/include
        )
        set_target_properties(
            sample_plugin_${variant}
            PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
        )
        target_compile_options(
            sample_plugin_${variant}
            PRIVATE -pedantic -Wall -Wextra -Werror
        )
    endforeach()
    target_compile_definitions(
        sample_plugin_other_major
        PRIVATE SAMPLE_ABI_MAJOR=99u
    )
    ament_add_gtest(test_plugin_abi plugin_abi/test/test_plugin_abi.cpp)
    target_link_libraries(test_plugin_abi lagrange_core)
    target_compile_definitions(
        test_plugin_abi
        PRIVATE
            LAGRANGE_SAMPLE_PLUGIN="$<TARGET_FILE:sample_plugin_current>"
            LAGRANGE_SAMPLE_PLUGIN_OTHER_MAJOR="$<TARGET_FILE:sample_plugin_other_major>"
    )
    add_dependencies(
        test_plugin_abi
        sample_plugin_current
        sample_plugin_other_major
    )
    lagrange_target_defaults(test_plugin_abi)

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
