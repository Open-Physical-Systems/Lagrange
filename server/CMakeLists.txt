cmake_minimum_required(VERSION 3.22)
project(lagrange_server VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(lagrange_rigid_body_mujoco REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(
    lagrange_server
    src/Server.cpp
    src/Robot.cpp
    src/Meshes.cpp
    src/FigureEightDriver.cpp
)
lagrange_target_defaults(lagrange_server)
target_include_directories(
    lagrange_server
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_server
    PUBLIC lagrange_core::lagrange_core lagrange_proto::lagrange_proto_transport
    PRIVATE
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
        urdfdom::urdfdom_model
        tinyxml2::tinyxml2
)

add_executable(lagrange_server_main src/main.cpp)
set_target_properties(
    lagrange_server_main
    PROPERTIES OUTPUT_NAME lagrange_server
)
lagrange_target_defaults(lagrange_server_main)
target_link_libraries(
    lagrange_server_main
    PRIVATE lagrange_server ament_index_cpp::ament_index_cpp
)

install(TARGETS lagrange_server_main RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
        test_server
        test/test_robot.cpp
        test/test_server.cpp
        TIMEOUT 120
    )
    target_link_libraries(
        test_server
        lagrange_server
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    target_compile_definitions(
        test_server
        PRIVATE
            LAGRANGE_TEST_AMR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/amr_diffdrive/amr_diffdrive.urdf"
            LAGRANGE_TEST_ARM_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots/primitives_arm/primitives_arm.urdf"
    )
    lagrange_target_defaults(test_server)
endif()

ament_package()
