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

# The wind tunnel's step: kernels/fields/lbm.slang and its host side.
add_library(lagrange_gpu_fluids fluids/src/LbmStepper.cpp)
add_library(lagrange::gpu_fluids ALIAS lagrange_gpu_fluids)
lagrange_target_defaults(lagrange_gpu_fluids)
lagrange_target_vulkan_warnings(lagrange_gpu_fluids)
target_include_directories(
    lagrange_gpu_fluids
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/fluids/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_gpu_fluids PRIVATE lagrange_gpu)
lagrange_add_slang_shader(
    lagrange_gpu_fluids
    SOURCE kernels/fields/lbm.slang
    ENTRY stepCells
    STAGE compute
    SYMBOL kLbm
)

# The mesh pipeline shared by the viewer and the camera sensor, and the sensor's renderer:
# kernels/camera/mesh.slang and its host side.
find_package(JPEG REQUIRED)
add_library(
    lagrange_gpu_raster
    raster/src/Raster.cpp
    raster/src/Realistic.cpp
    raster/src/SurfaceTextures.cpp
)
add_library(lagrange::gpu_raster ALIAS lagrange_gpu_raster)
lagrange_target_defaults(lagrange_gpu_raster)
lagrange_target_vulkan_warnings(lagrange_gpu_raster)
target_include_directories(
    lagrange_gpu_raster
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/raster/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    lagrange_gpu_raster
    PUBLIC lagrange_gpu
    PRIVATE JPEG::JPEG
)
# Photographs of surfaces: installed, else from the source tree when run from a build.
target_compile_definitions(
    lagrange_gpu_raster
    PRIVATE
        LAGRANGE_GPU_TEXTURES="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/textures"
        LAGRANGE_GPU_SOURCE_TEXTURES="${CMAKE_CURRENT_SOURCE_DIR}/assets_textures"
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/mesh.slang
    ENTRY vsMain
    STAGE vertex
    SYMBOL kMeshVertex
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/mesh.slang
    ENTRY fsSensorColor
    STAGE fragment
    SYMBOL kSensorColor
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/mesh.slang
    ENTRY fsSensorLabels
    STAGE fragment
    SYMBOL kSensorLabels
)

lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY vsMesh
    STAGE vertex
    SYMBOL kRealMeshVertex
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY fsMesh
    STAGE fragment
    SYMBOL kRealMeshFragment
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY vsShadow
    STAGE vertex
    SYMBOL kRealShadowVertex
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY fsShadow
    STAGE fragment
    SYMBOL kRealShadowFragment
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY vsScreen
    STAGE vertex
    SYMBOL kRealScreenVertex
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY fsSky
    STAGE fragment
    SYMBOL kRealSkyFragment
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY fsComposite
    STAGE fragment
    SYMBOL kRealCompositeFragment
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY vsWater
    STAGE vertex
    SYMBOL kRealWaterVertex
)
lagrange_add_slang_shader(
    lagrange_gpu_raster
    SOURCE kernels/camera/realistic.slang
    ENTRY fsWater
    STAGE fragment
    SYMBOL kRealWaterFragment
)

install(
    DIRECTORY assets_textures/
    DESTINATION share/${PROJECT_NAME}/textures
    FILES_MATCHING
    PATTERN "*.jpg"
    PATTERN "SOURCES.md"
)

add_executable(lagrange_gpu_info tools/gpu_info.cpp)
lagrange_target_defaults(lagrange_gpu_info)
target_link_libraries(lagrange_gpu_info PRIVATE lagrange_gpu)

install(DIRECTORY hal/include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY raycast/include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY fluids/include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY raster/include/ DESTINATION include/${PROJECT_NAME})
# Shader sources are installed so that other packages compile the same kernels (invariant 13).
install(DIRECTORY kernels/ DESTINATION share/${PROJECT_NAME}/kernels)
install(
    FILES cmake/LagrangeSlang.cmake cmake/EmbedSpirv.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)
install(
    TARGETS
        lagrange_gpu
        lagrange_gpu_raycast
        lagrange_gpu_raster
        lagrange_gpu_fluids
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

    # The camera's projection test calibrates with OpenCV (test only).
    find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
    ament_add_gtest(
        test_raster
        raster/test/test_raster.cpp
        TIMEOUT 300
        ENV
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
    target_link_libraries(
        test_raster
        lagrange_gpu_raster
        opencv_core
        opencv_imgproc
        opencv_calib3d
    )
    target_include_directories(
        test_raster
        SYSTEM
        PRIVATE ${OpenCV_INCLUDE_DIRS}
    )
    lagrange_target_defaults(test_raster)
endif()

ament_export_dependencies(JPEG)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Vulkan ZLIB)
ament_package(CONFIG_EXTRAS lagrange_gpu-extras.cmake)
