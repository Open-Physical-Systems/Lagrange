cmake_minimum_required(VERSION 3.22)
project(lagrange_viewer VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(lagrange_proto REQUIRED)
find_package(lagrange_gpu REQUIRED)
find_package(imgui_vendor REQUIRED)
find_package(SDL3 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(yaml-cpp REQUIRED)
# Mesh files for the builder (RFC 0016 open question 8, Addendum A): the server's own reader.
find_package(lagrange_meshes REQUIRED)

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
    client/src/Circuit.cpp
    client/src/FlowLegend.cpp
    client/src/RadarScale.cpp
    client/src/SchematicLayout.cpp
)
lagrange_target_defaults(lagrange_viewer_client)
target_include_directories(lagrange_viewer_client PUBLIC client/include)
target_link_libraries(
    lagrange_viewer_client
    PUBLIC lagrange_proto::lagrange_proto_transport
)

# The robot builder's model (RFC 0016): document, mass, kinematics, validation, undo, URDF,
# sensor catalog, primitives, scene adapter, test server, meshes and collision. No ImGui, no
# Vulkan, no simulation module (invariant 14); lagrange_viewer_client only for the SceneFrame
# types. lagrange_meshes (and through it lagrange_core, a library) is linked here only, privately,
# so the builder reads a mesh and makes its hull as the server does (open question 8, accepted);
# no builder header includes it, so the panels stay free of core.
add_library(
    lagrange_viewer_builder
    STATIC
    builder/src/Collision.cpp
    builder/src/CollisionSplit.cpp
    builder/src/Document.cpp
    builder/src/Edits.cpp
    builder/src/Gizmo.cpp
    builder/src/History.cpp
    builder/src/Kinematics.cpp
    builder/src/MassProperties.cpp
    builder/src/Materials.cpp
    builder/src/MeshShapes.cpp
    builder/src/Overlay.cpp
    builder/src/Primitives.cpp
    builder/src/SceneAdapter.cpp
    builder/src/SensorCatalog.cpp
    builder/src/SensorNeeds.cpp
    builder/src/TestServer.cpp
    builder/src/Urdf.cpp
    builder/src/Validation.cpp
)
lagrange_target_defaults(lagrange_viewer_builder)
target_include_directories(lagrange_viewer_builder PUBLIC builder/include)
target_link_libraries(
    lagrange_viewer_builder
    PUBLIC lagrange_viewer_client
    PRIVATE
        tinyxml2::tinyxml2
        yaml-cpp::yaml-cpp
        lagrange_meshes::lagrange_meshes
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
    SOURCE ${_kernels}/camera/mesh.slang
    ENTRY vsHeat
    STAGE vertex
    SYMBOL kHeatVertex
)
lagrange_add_slang_shader(
    lagrange_viewer_render
    SOURCE ${_kernels}/camera/mesh.slang
    ENTRY fsHeat
    STAGE fragment
    SYMBOL kHeatFragment
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
    panels/src/BuilderAdd.cpp
    panels/src/BuilderGizmo.cpp
    panels/src/BuilderJoints.cpp
    panels/src/BuilderPanels.cpp
    panels/src/BuilderView.cpp
    panels/src/BuilderPower.cpp
    panels/src/BuilderProblems.cpp
    panels/src/BuilderProperties.cpp
    panels/src/BuilderTree.cpp
    panels/src/BuilderTest.cpp
    panels/src/Chrome.cpp
    panels/src/ConsolePanel.cpp
    panels/src/DomainsPanel.cpp
    panels/src/ElectricalPanels.cpp
    panels/src/CircuitDraw.cpp
    panels/src/NetlistPanel.cpp
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
    panels/src/SensorCatalogPanel.cpp
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
        lagrange_viewer_builder
        lagrange_viewer_client
        lagrange_viewer_render
        lagrange_viewer_widgets
        imgui_vendor::imgui
)

add_executable(
    lagrange_viewer
    app/src/main.cpp
    app/src/App.cpp
    app/src/BuilderHost.cpp
)
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
# The design's theme (header only, Dear ImGui and ImPlot), for the tools in utils/ that draw
# with it: #include <lagrange/viewer/lagrange_theme.hpp>, fonts in share/lagrange_viewer/fonts.
install(
    FILES app/theme/include/lagrange/viewer/lagrange_theme.hpp
    DESTINATION include/${PROJECT_NAME}/lagrange/viewer
)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(imgui_vendor)
install(
    DIRECTORY assets/fonts/
    DESTINATION share/${PROJECT_NAME}/fonts
    FILES_MATCHING
    PATTERN "*.ttf"
    PATTERN "OFL.txt"
)

# The sensor catalog (RFC 0016 §5, R11): lagrange_sensors owns it; the viewer build-depends on
# that package for this data file only (no find_package, no library) and installs a copy, so a
# viewer-only install still has it.
set(_lagrange_sensor_catalog "")
string(REPLACE ":" ";" _lagrange_prefixes "$ENV{AMENT_PREFIX_PATH}")
foreach(_prefix IN LISTS _lagrange_prefixes)
    set(_candidate
        "${_prefix}/share/lagrange_sensors/catalog/sensor_catalog.yaml"
    )
    if(NOT _lagrange_sensor_catalog AND EXISTS "${_candidate}")
        set(_lagrange_sensor_catalog "${_candidate}")
    endif()
endforeach()
if(_lagrange_sensor_catalog)
    message(STATUS "Sensor catalog: ${_lagrange_sensor_catalog}")
    install(
        FILES "${_lagrange_sensor_catalog}"
        DESTINATION share/${PROJECT_NAME}/catalog
    )
else()
    message(
        STATUS
        "Sensor catalog: not installed by lagrange_sensors yet; the builder's is empty"
    )
endif()

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Threads REQUIRED)
    ament_add_gtest(
        test_viewer_client
        client/test/test_circuit.cpp
        client/test/test_flow_legend.cpp
        client/test/test_schematic_layout.cpp
        client/test/test_profile_draft.cpp
        client/test/test_profile_session.cpp
        client/test/test_radar_scale.cpp
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

    # The robot builder's model (RFC 0016): each work package adds its test files here.
    ament_add_gtest(
        test_viewer_builder
        builder/test/test_builder_smoke.cpp
        builder/test/test_history.cpp
        builder/test/test_kinematics.cpp
        builder/test/test_mass_properties.cpp
        builder/test/test_validation.cpp
        builder/test/test_scene_adapter.cpp
        builder/test/test_urdf.cpp
        builder/test/test_edits.cpp
        builder/test/test_sensor_catalog_loader.cpp
        builder/test/test_gizmo.cpp
        builder/test/test_test_server.cpp
        builder/test/test_scene_renderer_statics.cpp
        builder/test/test_mesh_shapes.cpp
        builder/test/test_overlay.cpp
        TIMEOUT 600
    )
    target_link_libraries(
        test_viewer_builder
        lagrange_viewer_builder
        lagrange_viewer_render
        lagrange_meshes::lagrange_meshes
    )
    target_compile_definitions(
        test_viewer_builder
        PRIVATE
            LAGRANGE_BUILDER_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/builder"
            LAGRANGE_THERMAL_MATERIALS_CPP="${CMAKE_CURRENT_SOURCE_DIR}/../modules/thermal/conduction/src/Materials.cpp"
            LAGRANGE_BUILDER_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../assets/robots"
            LAGRANGE_SCHEMAS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../schemas"
            LAGRANGE_SENSOR_CATALOG_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../modules/sensors/catalog/sensor_catalog.yaml"
            LAGRANGE_PLUGIN_CATALOG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../core/plugin_abi/samples/catalog"
    )
    lagrange_target_defaults(test_viewer_builder)
    # Read and write a URDF from the command line, and make the templates (WP-B); not installed.
    add_executable(builder_urdf_tool builder/test/urdf_tool.cpp)
    target_link_libraries(builder_urdf_tool PRIVATE lagrange_viewer_builder)
    lagrange_target_defaults(builder_urdf_tool)

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_circuit_roundtrip test/test_circuit_roundtrip.py TIMEOUT 120)
    ament_add_pytest_test(test_viewer_input test/test_viewer_input.py TIMEOUT 180)
    ament_add_pytest_test(test_radar_view test/test_radar_view.py TIMEOUT 600)
    ament_add_pytest_test(test_builder_input test/test_builder_input.py TIMEOUT 180)
    ament_add_pytest_test(test_builder_gizmo test/test_builder_gizmo.py TIMEOUT 300)
    ament_add_pytest_test(test_builder_view test/test_builder_view.py TIMEOUT 300)
    ament_add_pytest_test(test_builder_test test/test_builder_test.py TIMEOUT 600)
    ament_add_pytest_test(test_builder_into_run test/test_builder_into_run.py TIMEOUT 300)
    ament_add_pytest_test(test_builder_acceptance test/test_builder_acceptance.py TIMEOUT 600)
    ament_add_pytest_test(test_builder_joint_drive test/test_builder_joint_drive.py TIMEOUT 300)
    ament_add_pytest_test(test_builder_power test/test_builder_power.py TIMEOUT 600)
    ament_add_pytest_test(test_builder_files test/test_builder_files.py TIMEOUT 300)
    ament_add_pytest_test(
        test_builder_roundtrip
        test/test_builder_roundtrip.py
        TIMEOUT 300
        ENV
        LAGRANGE_BUILDER_URDF_TOOL=$<TARGET_FILE:builder_urdf_tool>
    )
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
