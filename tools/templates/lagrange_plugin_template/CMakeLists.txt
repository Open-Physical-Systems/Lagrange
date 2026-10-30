cmake_minimum_required(VERSION 3.22)
project(my_lagrange_plugin LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_core REQUIRED) # for lagrange_plugin.h, the C ABI (header only)

# The plugin: a shared object that exports lagrange_plugin_get_module. It links nothing of
# Lagrange's; the header is all it needs.
add_library(${PROJECT_NAME} MODULE src/my_sensor.c)
get_target_property(
    _lagrange_includes
    lagrange_core::lagrange_core
    INTERFACE_INCLUDE_DIRECTORIES
)
target_include_directories(${PROJECT_NAME} PRIVATE ${_lagrange_includes})
set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# `lagrange_server --plugin my_lagrange_plugin` finds it through this entry.
ament_index_register_resource(
    lagrange_plugins
    PACKAGE_NAME ${PROJECT_NAME}
    CONTENT "lib/lib${PROJECT_NAME}.so"
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_server REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    ament_add_gtest(test_my_sensor test/test_my_sensor.cpp)
    target_link_libraries(
        test_my_sensor
        lagrange_server::lagrange_server
        ament_index_cpp::ament_index_cpp
    )
    target_compile_features(test_my_sensor PRIVATE cxx_std_20)
    target_compile_definitions(
        test_my_sensor
        PRIVATE MY_PLUGIN="$<TARGET_FILE:${PROJECT_NAME}>"
    )
    add_dependencies(test_my_sensor ${PROJECT_NAME})
endif()

ament_package()
