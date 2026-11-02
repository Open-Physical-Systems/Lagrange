cmake_minimum_required(VERSION 3.22)
project(lagrange_aerodynamics VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_fields REQUIRED)
find_package(TBB REQUIRED)
find_package(lagrange_gpu REQUIRED)

add_library(
    lagrange_aerodynamics
    drag/src/Aerodynamics.cpp
    drag/src/Downwash.cpp
    lbm/src/Lbm.cpp
    lbm/src/LbmBox.cpp
    lbm/src/LiveAir.cpp
    lbm/src/WindTunnel.cpp
)
lagrange_target_defaults(lagrange_aerodynamics)
target_include_directories(
    lagrange_aerodynamics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_aerodynamics
    PUBLIC lagrange_core::lagrange_core lagrange_fields::lagrange_fields
    PRIVATE
        TBB::tbb
        lagrange_gpu::lagrange_gpu
        lagrange_gpu::lagrange_gpu_fluids
)

# The air box round a hovering multicopter, for measuring its speed and flow
# (rotor_aero.md, the live lattice Boltzmann box).
add_executable(lagrange_lbm_hover lbm/src/hover_box_main.cpp)
lagrange_target_defaults(lagrange_lbm_hover)
target_link_libraries(
    lagrange_lbm_hover
    PRIVATE lagrange_aerodynamics lagrange_gpu::lagrange_gpu
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS lagrange_lbm_hover DESTINATION lib/${PROJECT_NAME})
install(
    TARGETS lagrange_aerodynamics
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_aerodynamics test/test_aerodynamics.cpp)
    target_link_libraries(test_aerodynamics lagrange_aerodynamics)
    lagrange_target_defaults(test_aerodynamics)
    ament_add_gtest(test_downwash test/test_downwash.cpp)
    target_link_libraries(test_downwash lagrange_aerodynamics)
    lagrange_target_defaults(test_downwash)
    ament_add_gtest(test_lbm test/test_lbm.cpp TIMEOUT 300)
    target_link_libraries(
        test_lbm
        lagrange_aerodynamics
        lagrange_gpu::lagrange_gpu
    )
    lagrange_target_defaults(test_lbm)
    ament_add_gtest(test_lbm_box test/test_lbm_box.cpp TIMEOUT 300)
    target_link_libraries(
        test_lbm_box
        lagrange_aerodynamics
        lagrange_gpu::lagrange_gpu
    )
    lagrange_target_defaults(test_lbm_box)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core lagrange_fields TBB lagrange_gpu)
ament_package()
