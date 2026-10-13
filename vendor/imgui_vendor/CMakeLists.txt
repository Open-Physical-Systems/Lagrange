cmake_minimum_required(VERSION 3.24)
project(imgui_vendor VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(SDL3 REQUIRED)
find_package(Vulkan REQUIRED)

# Dear ImGui, docking branch. Pinned to the commit of the upstream tag, with the archive hash.
# renovate: datasource=github-tags depName=ocornut/imgui
set(IMGUI_VERSION "v1.92.9b-docking")
set(IMGUI_COMMIT "b48d1afbe8ee8b238e2961dc363a949dd7304e23")
set(IMGUI_SHA256
    "747c5a465a05c4ef5b4a09e2e0e3d0ce2bc76b31420f0f5af33df470907cf893"
)

include(FetchContent)
FetchContent_Declare(
    imgui_src
    URL "https://github.com/ocornut/imgui/archive/${IMGUI_COMMIT}.tar.gz"
    URL_HASH SHA256=${IMGUI_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
FetchContent_MakeAvailable(imgui_src)

add_library(
    imgui
    STATIC
    ${imgui_src_SOURCE_DIR}/imgui.cpp
    ${imgui_src_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_src_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_src_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_src_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_src_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_src_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)
add_library(imgui_vendor::imgui ALIAS imgui)
target_compile_features(imgui PUBLIC cxx_std_17)
set_target_properties(imgui PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
    imgui
    SYSTEM
    PUBLIC
        $<BUILD_INTERFACE:${imgui_src_SOURCE_DIR}>
        $<BUILD_INTERFACE:${imgui_src_SOURCE_DIR}/backends>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(imgui PUBLIC SDL3::SDL3 Vulkan::Vulkan)

install(
    FILES
        ${imgui_src_SOURCE_DIR}/imgui.h
        ${imgui_src_SOURCE_DIR}/imconfig.h
        ${imgui_src_SOURCE_DIR}/imgui_internal.h
        ${imgui_src_SOURCE_DIR}/imstb_rectpack.h
        ${imgui_src_SOURCE_DIR}/imstb_textedit.h
        ${imgui_src_SOURCE_DIR}/imstb_truetype.h
        ${imgui_src_SOURCE_DIR}/backends/imgui_impl_sdl3.h
        ${imgui_src_SOURCE_DIR}/backends/imgui_impl_vulkan.h
    DESTINATION include/${PROJECT_NAME}
)
install(
    FILES ${imgui_src_SOURCE_DIR}/LICENSE.txt
    DESTINATION share/${PROJECT_NAME}
    RENAME LICENSE.imgui.txt
)
install(
    TARGETS imgui
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(SDL3 Vulkan)
ament_package()
