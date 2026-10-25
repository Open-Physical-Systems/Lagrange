cmake_minimum_required(VERSION 3.22)
project(lagrange_electrical VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(fmilibrary3_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
    lagrange_electrical
    substitutes/src/IdealTorqueSource.cpp
    network/src/Models.cpp
    network/src/Network.cpp
    fmu_host/src/FmuHost.cpp
    export/src/Export.cpp
)
lagrange_target_defaults(lagrange_electrical)
target_include_directories(
    lagrange_electrical
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_electrical
    PUBLIC lagrange_core::lagrange_core
    PRIVATE fmilibrary3_vendor::fmilib yaml-cpp::yaml-cpp
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_electrical
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_ideal_torque_source test/test_ideal_torque_source.cpp)
    target_link_libraries(test_ideal_torque_source lagrange_electrical)
    lagrange_target_defaults(test_ideal_torque_source)
    ament_add_gtest(test_models test/test_models.cpp)
    target_link_libraries(test_models lagrange_electrical)
    lagrange_target_defaults(test_models)
    find_package(lagrange_models REQUIRED)
    get_filename_component(models_share "${lagrange_models_DIR}/.." ABSOLUTE)
    ament_add_gtest(test_fmu_host test/test_fmu_host.cpp TIMEOUT 120)
    target_link_libraries(test_fmu_host lagrange_electrical)
    target_compile_definitions(
        test_fmu_host
        PRIVATE LAGRANGE_TEST_DC_DRIVE_FMU="${models_share}/fmu/dc_drive.fmu"
    )
    lagrange_target_defaults(test_fmu_host)
    # The netlist round trip runs ngspice when it is installed (apt ngspice; offline tool).
    find_program(NGSPICE ngspice)
    ament_add_gtest(test_export test/test_export.cpp TIMEOUT 120)
    target_link_libraries(test_export lagrange_electrical)
    target_compile_definitions(
        test_export
        PRIVATE LAGRANGE_TEST_NGSPICE="$<$<BOOL:${NGSPICE}>:${NGSPICE}>"
    )
    lagrange_target_defaults(test_export)
    ament_add_gtest(test_coupling test/test_coupling.cpp TIMEOUT 300)
    target_link_libraries(test_coupling lagrange_electrical)
    lagrange_target_defaults(test_coupling)
    # The battery model against published cell data (data/lg_hg2, CC BY 4.0).
    ament_add_gtest(test_battery_dataset test/test_battery_dataset.cpp)
    target_link_libraries(test_battery_dataset lagrange_electrical)
    target_compile_definitions(
        test_battery_dataset
        PRIVATE
            LAGRANGE_TEST_HG2_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/lg_hg2"
    )
    lagrange_target_defaults(test_battery_dataset)
    ament_add_gtest(test_network test/test_network.cpp TIMEOUT 120)
    target_link_libraries(test_network lagrange_electrical)
    lagrange_target_defaults(test_network)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core fmilibrary3_vendor yaml-cpp)
ament_package()
