cmake_minimum_required(VERSION 3.22)
project(lagrange_example_plugin VERSION 1.2.1 LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(lagrange_core REQUIRED) # lagrange_plugin.h, the C ABI (header only)

# A shared object that exports lagrange_plugin_get_module; it links nothing of Lagrange's.
add_library(${PROJECT_NAME} MODULE src/distance_sensor.c)
get_target_property(
    _includes
    lagrange_core::lagrange_core
    INTERFACE_INCLUDE_DIRECTORIES
)
target_include_directories(${PROJECT_NAME} PRIVATE ${_includes})
target_link_libraries(${PROJECT_NAME} PRIVATE m)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# `lagrange_server --plugin lagrange_example_plugin` finds it through this entry.
ament_index_register_resource(
    lagrange_plugins
    PACKAGE_NAME ${PROJECT_NAME}
    CONTENT "lib/lib${PROJECT_NAME}.so"
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS scripts/read_plugin.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME read_plugin
)

ament_package()
