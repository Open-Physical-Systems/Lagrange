cmake_minimum_required(VERSION 3.22)
project(lagrange_rotor_aero VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(
    lagrange_rotor_aero
    coefficients/src/RotorAero.cpp
    bem/src/Propeller.cpp
)
lagrange_target_defaults(lagrange_rotor_aero)
target_include_directories(
    lagrange_rotor_aero
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_rotor_aero
    PUBLIC lagrange_core::lagrange_core
    PRIVATE yaml-cpp::yaml-cpp Threads::Threads
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
    TARGETS lagrange_rotor_aero
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_rotor_aero test/test_rotor_aero.cpp)
    target_link_libraries(test_rotor_aero lagrange_rotor_aero)
    lagrange_target_defaults(test_rotor_aero)
    ament_add_gtest(test_bem test/test_bem.cpp)
    target_link_libraries(test_bem lagrange_rotor_aero)
    target_compile_definitions(
        test_bem
        PRIVATE
            LAGRANGE_PROPELLERS="${CMAKE_CURRENT_SOURCE_DIR}/../../assets/propellers"
    )
    lagrange_target_defaults(test_bem)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lagrange_core yaml-cpp)
ament_package()
