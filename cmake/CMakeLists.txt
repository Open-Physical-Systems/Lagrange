cmake_minimum_required(VERSION 3.22)
project(lagrange_cmake VERSION 0.0.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# Declares the LAGRANGE_* options here too, so a workspace-wide -DLAGRANGE_...=... does not
# produce an "unused variable" warning for this package.
include(LagrangeCompileOptions.cmake)

install(
    FILES LagrangeCompileOptions.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

# find_package(lagrange_cmake) makes lagrange_target_defaults() available.
ament_package(CONFIG_EXTRAS lagrange_cmake-extras.cmake)
