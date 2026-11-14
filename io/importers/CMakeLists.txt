cmake_minimum_required(VERSION 3.22)
project(lagrange_importers VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
# libsdformat 16.1.0 from ros-lyrical-sdformat-vendor; MuJoCo 3.12.0 from ros-lyrical-mujoco-vendor.
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
# See lagrange_rigid_body_mujoco: LAGRANGE_MUJOCO_PACKAGE picks the MuJoCo; both must agree.
set(LAGRANGE_MUJOCO_PACKAGE
    mujoco_vendor
    CACHE STRING
    "CMake package that provides mujoco::mujoco"
)
find_package(${LAGRANGE_MUJOCO_PACKAGE} REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(
    lagrange_importers
    src/Description.cpp
    src/SdfImporter.cpp
    src/MjcfImporter.cpp
    src/Format.cpp
)
add_library(lagrange::importers ALIAS lagrange_importers)
target_include_directories(
    lagrange_importers
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_importers
    PUBLIC lagrange_core::lagrange_core
    PRIVATE sdformat::sdformat mujoco::mujoco tinyxml2::tinyxml2
)
lagrange_target_defaults(lagrange_importers)
# See lagrange_rigid_body_mujoco: MuJoCo's mjsan.h does not compile under GCC with ASan.
if(LAGRANGE_SANITIZER MATCHES "address")
    target_compile_definitions(
        lagrange_importers
        PRIVATE MUJOCO_INCLUDE_MJSAN_H_
    )
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_importers
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lagrange_rigid_body_mujoco REQUIRED)
    set(assets "${CMAKE_CURRENT_SOURCE_DIR}/../../assets")
    ament_add_gtest(test_importers test/test_importers.cpp TIMEOUT 120)
    target_link_libraries(
        test_importers
        lagrange_importers
        lagrange_rigid_body_mujoco::lagrange_rigid_body_mujoco
    )
    target_compile_definitions(
        test_importers
        PRIVATE
            LAGRANGE_TEST_AMR_SDF="${assets}/robots/amr_diffdrive/amr_diffdrive.sdf"
            LAGRANGE_TEST_AMR_MJCF="${assets}/robots/amr_diffdrive/amr_diffdrive.xml"
            LAGRANGE_TEST_AMR_URDF="${assets}/robots/amr_diffdrive/amr_diffdrive.urdf"
    )
    lagrange_target_defaults(test_importers)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core sdformat_vendor sdformat ${LAGRANGE_MUJOCO_PACKAGE} tinyxml2)
ament_package()
