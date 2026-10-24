cmake_minimum_required(VERSION 3.22)
project(lagrange_benchmarks VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_core REQUIRED)
find_package(lagrange_sensors REQUIRED)
find_package(benchmark REQUIRED)

# Run by hand or by the nightly job (tools/ci/benchmarks.sh), not by colcon test: timings on a
# shared machine are not pass/fail. The smoke test below only checks that every benchmark runs.
add_executable(bench_core src/bench_core.cpp)
target_link_libraries(
    bench_core
    lagrange_core::lagrange_core
    benchmark::benchmark_main
)
lagrange_target_defaults(bench_core)

add_executable(bench_lidar src/bench_lidar.cpp)
target_link_libraries(
    bench_lidar
    lagrange_sensors::lagrange_sensors
    benchmark::benchmark_main
)
lagrange_target_defaults(bench_lidar)

install(TARGETS bench_core bench_lidar DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/check_benchmarks.py DESTINATION lib/${PROJECT_NAME})
install(FILES baseline.json DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    foreach(bench bench_core bench_lidar)
        add_test(
            NAME ${bench}_smoke
            COMMAND $<TARGET_FILE:${bench}> --benchmark_min_time=1x
        )
        set_tests_properties(
            ${bench}_smoke
            PROPERTIES
                TIMEOUT 120
                ENVIRONMENT "ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}"
        )
    endforeach()
endif()

ament_package()
