#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::geospatial {

/// One mesh of a 3D Tiles tileset, in the world's ENU frame (m).
struct Tiles3dMesh {
    std::vector<std::array<double, 3>> points;
    std::vector<std::uint32_t> indices;
    std::array<double, 3> centre{};
};

struct Tiles3dContent {
    std::vector<Tiles3dMesh> meshes;
    std::vector<std::string> notes;
};

/// Reads an OGC 3D Tiles tileset (huge-worlds.md §4): tileset.json, its leaf tiles' content
/// (glTF .glb or .gltf, or Batched 3D Model .b3dm with RTC_CENTER) through Assimp, each
/// tile's transform applied (column-major, accumulated from the root) into ECEF, and glTF's
/// y-up turned z-up as the specification says, then into the ENU frame of `origin`. Visual
/// only: photogrammetry meshes get no colliders. External tilesets and implicit tiling are
/// not followed (noted).
[[nodiscard]] Expected<Tiles3dContent, std::string> readTiles3d(const std::string& tileset,
                                                                const coords::GeoOrigin& origin);

}  // namespace lagrange::geospatial
