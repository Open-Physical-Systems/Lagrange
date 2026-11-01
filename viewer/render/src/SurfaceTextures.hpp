#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace lagrange::viewer {

/// Photographs of surfaces for the realistic view, one layer per kind, in the order of the layers
/// that realistic.slang's imageSurface takes. Each layer is tightly packed RGBA8.
struct SurfaceTextures {
    static constexpr std::uint32_t kSize = 1024;  // pixels, every file's width and height
    std::vector<std::vector<std::uint8_t>> albedo, normal;
    /// False when any file is missing or unreadable: the view then draws its procedural surfaces.
    bool complete{false};
};

/// Reads <directory>/<kind>_albedo.jpg and <kind>_normal.jpg for every kind.
[[nodiscard]] SurfaceTextures loadSurfaceTextures(const std::filesystem::path& directory);

/// Where the viewer's textures are: installed next to the program, else in the source tree.
[[nodiscard]] std::filesystem::path surfaceTextureDirectory();

}  // namespace lagrange::viewer
