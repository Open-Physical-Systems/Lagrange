cmake_minimum_required(VERSION 3.22)
project(lagrange_python VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_server REQUIRED)
find_package(Python 3 REQUIRED COMPONENTS Interpreter Development.Module)
# nanobind from Ubuntu's nanobind-dev (BSD-3-Clause).
find_package(nanobind CONFIG REQUIRED PATHS /usr/share/nanobind/cmake)

# The native part, imported by the `lagrange` package as lagrange._lagrange.
nanobind_add_module(_lagrange src/bindings.cpp)
target_link_libraries(_lagrange PRIVATE lagrange_server::lagrange_server)
target_compile_features(_lagrange PRIVATE cxx_std_20)

ament_python_install_package(lagrange)
ament_get_python_install_dir(python_install_dir)
install(TARGETS _lagrange DESTINATION "${python_install_dir}/lagrange")

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_lagrange test/test_lagrange.py TIMEOUT 120)
    ament_add_pytest_test(test_town test/test_town.py TIMEOUT 600)
endif()

ament_package()
