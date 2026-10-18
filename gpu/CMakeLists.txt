cmake_minimum_required(VERSION 3.22)
project(lagrange_gpu VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(Vulkan REQUIRED)
find_package(VulkanMemoryAllocator REQUIRED)
find_package(ZLIB REQUIRED)

include(cmake/LagrangeSlang.cmake)

# The allocator's implementation is third-party code: compiled without our warning level.
add_library(lagrange_gpu_vma OBJECT hal/src/Vma.cpp)
target_compile_features(lagrange_gpu_vma PRIVATE cxx_std_20)
set_target_properties(lagrange_gpu_vma PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(lagrange_gpu_vma PRIVATE -w)
# The allocator is header-only. Using its include directory instead of linking its target
# keeps it out of this package's exported link interface.
get_target_property(
    _vma_include
    GPUOpen::VulkanMemoryAllocator
    INTERFACE_INCLUDE_DIRECTORIES
)
target_include_directories(lagrange_gpu_vma SYSTEM PRIVATE ${_vma_include})
target_link_libraries(lagrange_gpu_vma PRIVATE Vulkan::Vulkan)

add_library(
    lagrange_gpu
    hal/src/Compute.cpp
    hal/src/Device.cpp
    hal/src/Resources.cpp
    $<TARGET_OBJECTS:lagrange_gpu_vma>
)
add_library(lagrange::gpu ALIAS lagrange_gpu)
lagrange_target_defaults(lagrange_gpu)
lagrange_target_vulkan_warnings(lagrange_gpu)
target_include_directories(
    lagrange_gpu
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hal/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(lagrange_gpu SYSTEM PRIVATE ${_vma_include})
target_link_libraries(lagrange_gpu PUBLIC Vulkan::Vulkan PRIVATE ZLIB::ZLIB)

# The Tier 1 lidar's ray caster: kernels/lidar/bvh_traverse.slang and its host side.
add_library(lagrange_gpu_raycast raycast/src/RayCaster.cpp)
add_library(lagrange::gpu_raycast ALIAS lagrange_gpu_raycast)
lagrange_target_defaults(lagrange_gpu_raycast)
lagrange_target_vulkan_warnings(lagrange_gpu_raycast)
target_include_directories(
    lagrange_gpu_raycast
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/raycast/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_gpu_raycast PRIVATE lagrange_gpu)
lagrange_add_slang_shader(
    lagrange_gpu_raycast
    SOURCE kernels/lidar/bvh_traverse.slang
    ENTRY castRays
    STAGE compute
    SYMBOL kBvhTraverse
)

add_executable(lagrange_gpu_info tools/gpu_info.cpp)
lagrange_target_defaults(lagrange_gpu_info)
target_link_libraries(lagrange_gpu_info PRIVATE lagrange_gpu)

install(DIRECTORY hal/include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY raycast/include/ DESTINATION include/${PROJECT_NAME})
# Shader sources are installed so that other packages compile the same kernels (invariant 13).
install(DIRECTORY kernels/ DESTINATION share/${PROJECT_NAME}/kernels)
install(
    FILES cmake/LagrangeSlang.cmake cmake/EmbedSpirv.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)
install(
    TARGETS lagrange_gpu lagrange_gpu_raycast
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS lagrange_gpu_info RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
        test_gpu
        hal/test/test_gpu.cpp
        TIMEOUT 120
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(test_gpu lagrange_gpu)
    lagrange_target_defaults(test_gpu)
    lagrange_target_vulkan_warnings(test_gpu)
    lagrange_add_slang_shader(
        test_gpu
        SOURCE hal/test/triangle.slang
        ENTRY vsMain
        STAGE vertex
        SYMBOL kTriangleVertex
    )
    lagrange_add_slang_shader(
        test_gpu
        SOURCE hal/test/triangle.slang
        ENTRY fsMain
        STAGE fragment
        SYMBOL kTriangleFragment
    )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Vulkan ZLIB)
ament_package(CONFIG_EXTRAS lagrange_gpu-extras.cmake)
