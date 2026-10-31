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
    client/src/ProfileDraft.cpp
    client/src/ProfileSession.cpp
    client/src/RecentList.cpp
    client/src/SceneStore.cpp
    client/src/ScopeBuffer.cpp
    client/src/ScopeSession.cpp
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
    render/src/Picking.cpp
    render/src/RealisticPass.cpp
    render/src/SceneRenderer.cpp
)
lagrange_target_defaults(lagrange_viewer_render)
lagrange_target_vulkan_warnings(lagrange_viewer_render)
target_include_directories(lagrange_viewer_render PUBLIC render/include)
target_link_libraries(
    lagrange_viewer_render
    PUBLIC
        lagrange_viewer_client
        lagrange_gpu::lagrange_gpu
        lagrange_gpu::lagrange_gpu_raster
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
    SOURCE ${_kernels}/camera/mesh.slang
    ENTRY fsUnlit
    STAGE fragment
    SYMBOL kMeshUnlitFragment
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

lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY vsMesh
    STAGE vertex
    SYMBOL kRealMeshVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY fsMesh
    STAGE fragment
    SYMBOL kRealMeshFragment
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY vsShadow
    STAGE vertex
    SYMBOL kRealShadowVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY fsShadow
    STAGE fragment
    SYMBOL kRealShadowFragment
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY vsScreen
    STAGE vertex
    SYMBOL kRealScreenVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY fsSky
    STAGE fragment
    SYMBOL kRealSkyFragment
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY fsComposite
    STAGE fragment
    SYMBOL kRealCompositeFragment
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY vsWater
    STAGE vertex
    SYMBOL kRealWaterVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/realistic.slang
    ENTRY fsWater
    STAGE fragment
    SYMBOL kRealWaterFragment
)

# The design system: theme, fonts and the widgets every panel is built from
# (docs/design/ui/00-design-system).
add_library(
    lagrange_viewer_widgets
    STATIC
    app/theme/src/Fonts.cpp
    app/widgets/src/Icons.cpp
    app/widgets/src/Widgets.cpp
)
lagrange_target_defaults(lagrange_viewer_widgets)
target_include_directories(
    lagrange_viewer_widgets
    PUBLIC app/theme/include app/widgets/include
)
target_link_libraries(lagrange_viewer_widgets PUBLIC imgui_vendor::imgui)

add_library(
    lagrange_viewer_panels
    STATIC
    panels/src/Chrome.cpp
    panels/src/ConsolePanel.cpp
    panels/src/DomainsPanel.cpp
    panels/src/ElectricalPanels.cpp
    panels/src/EnvironmentPanels.cpp
    panels/src/InspectorPanel.cpp
    panels/src/Launcher.cpp
    panels/src/PanelFormat.cpp
    panels/src/PerformancePanels.cpp
    panels/src/ProfilePanel.cpp
    panels/src/ReplayPanels.cpp
    panels/src/Ros2Panels.cpp
    panels/src/ScenePanel.cpp
    panels/src/ScheduleChart.cpp
    panels/src/ScopePanel.cpp
    panels/src/ScopePlots.cpp
    panels/src/SensorsPanels.cpp
    panels/src/SignalsPanel.cpp
    panels/src/TimelinePanel.cpp
    panels/src/ViewportPanel.cpp
    panels/src/WiringPanels.cpp
    panels/src/WorldPanels.cpp
)
lagrange_target_defaults(lagrange_viewer_panels)
target_include_directories(lagrange_viewer_panels PUBLIC panels/include)
target_link_libraries(
    lagrange_viewer_panels
    PUBLIC
        lagrange_viewer_client
        lagrange_viewer_render
        lagrange_viewer_widgets
        imgui_vendor::imgui
)

add_executable(lagrange_viewer app/src/main.cpp app/src/App.cpp)
lagrange_target_defaults(lagrange_viewer)
lagrange_target_vulkan_warnings(lagrange_viewer)
target_link_libraries(
    lagrange_viewer
    PRIVATE lagrange_viewer_panels lagrange_viewer_render SDL3::SDL3
)

# Fonts are looked for next to the installed program first, then in the source tree, so the
# viewer also finds them when it is run from the build directory.
target_compile_definitions(
    lagrange_viewer
    PRIVATE
        LAGRANGE_VIEWER_SOURCE_FONTS="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts"
)

install(TARGETS lagrange_viewer RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY assets/fonts/
    DESTINATION share/${PROJECT_NAME}/fonts
    FILES_MATCHING
    PATTERN "*.ttf"
    PATTERN "OFL.txt"
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Threads REQUIRED)
    ament_add_gtest(
        test_viewer_client
        client/test/test_profile_draft.cpp
        client/test/test_profile_session.cpp
        client/test/test_recent_list.cpp
        client/test/test_scene_store.cpp
        client/test/test_scene_fuzz.cpp
        client/test/test_scope_buffer.cpp
        client/test/test_scope_session.cpp
        client/test/test_viewer_client.cpp
        render/test/test_camera.cpp
        render/test/test_picking.cpp
        render/test/test_thermal.cpp
        TIMEOUT 120
    )
    target_link_libraries(
        test_viewer_client
        lagrange_viewer_client
        lagrange_viewer_render
        Threads::Threads
    )
    target_compile_definitions(
        test_viewer_client
        PRIVATE
            LAGRANGE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../proto/test/golden"
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

option(LAGRANGE_BUILD_FUZZERS "Build libFuzzer targets (needs Clang)" OFF)
if(LAGRANGE_BUILD_FUZZERS)
    add_executable(fuzz_scene_builder client/fuzz/fuzz_scene_builder.cpp)
    target_link_libraries(fuzz_scene_builder PRIVATE lagrange_viewer_client)
    target_compile_options(fuzz_scene_builder PRIVATE -fsanitize=fuzzer)
    target_link_options(fuzz_scene_builder PRIVATE -fsanitize=fuzzer)
    lagrange_target_defaults(fuzz_scene_builder)
endif()

ament_package()
