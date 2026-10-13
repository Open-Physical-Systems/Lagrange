cmake_minimum_required(VERSION 3.22)
project(lagrange_viewer VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(lagrange_gpu REQUIRED)
find_package(imgui_vendor REQUIRED)
find_package(SDL3 REQUIRED)

# Protocol client and scene store. No Vulkan, no ImGui: testable without a display.
add_library(
    lagrange_viewer_client
    STATIC
    client/src/SceneStore.cpp
    client/src/ScopeBuffer.cpp
    client/src/ViewerClient.cpp
)
lagrange_target_defaults(lagrange_viewer_client)
target_include_directories(lagrange_viewer_client PUBLIC client/include)
target_link_libraries(
    lagrange_viewer_client
    PUBLIC lagrange_proto::lagrange_proto_transport
)

# Scene renderer on the GPU layer, with the shared Slang shaders.
add_library(
    lagrange_viewer_render
    STATIC
    render/src/Camera.cpp
    render/src/SceneRenderer.cpp
)
lagrange_target_defaults(lagrange_viewer_render)
lagrange_target_vulkan_warnings(lagrange_viewer_render)
target_include_directories(lagrange_viewer_render PUBLIC render/include)
target_link_libraries(
    lagrange_viewer_render
    PUBLIC lagrange_viewer_client lagrange_gpu::lagrange_gpu
)
set(_kernels "${LAGRANGE_GPU_KERNELS_DIR}")
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/mesh.slang
    ENTRY vsMain
    STAGE vertex
    SYMBOL kMeshVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/mesh.slang
    ENTRY fsMain
    STAGE fragment
    SYMBOL kMeshFragment
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/common/line.slang
    ENTRY vsMain
    STAGE vertex
    SYMBOL kLineVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/common/line.slang
    ENTRY fsMain
    STAGE fragment
    SYMBOL kLineFragment
)

add_library(
    lagrange_viewer_panels
    STATIC
    panels/src/Panels.cpp
    panels/src/ScopePanel.cpp
)
lagrange_target_defaults(lagrange_viewer_panels)
target_include_directories(lagrange_viewer_panels PUBLIC panels/include)
target_link_libraries(
    lagrange_viewer_panels
    PUBLIC lagrange_viewer_client lagrange_viewer_render imgui_vendor::imgui
)

add_executable(lagrange_viewer app/src/main.cpp app/src/App.cpp)
lagrange_target_defaults(lagrange_viewer)
lagrange_target_vulkan_warnings(lagrange_viewer)
target_link_libraries(
    lagrange_viewer
    PRIVATE lagrange_viewer_panels lagrange_viewer_render SDL3::SDL3
)

install(TARGETS lagrange_viewer RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Threads REQUIRED)
    ament_add_gtest(
        test_viewer_client
        client/test/test_scene_store.cpp
        client/test/test_scope_buffer.cpp
        client/test/test_viewer_client.cpp
        render/test/test_camera.cpp
        TIMEOUT 120
    )
    target_link_libraries(
        test_viewer_client
        lagrange_viewer_client
        lagrange_viewer_render
        Threads::Threads
    )
    lagrange_target_defaults(test_viewer_client)

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(
        test_headless
        test/test_headless.py
        TIMEOUT 180
        ENV
        LAGRANGE_VIEWER=$<TARGET_FILE:lagrange_viewer>
        ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
    )
endif()

ament_package()
