#pragma once

#include "lagrange/module/Expected.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::meshes {

/// One material's worth of triangles, in the file's units and axes after the importer's own
/// conversions (Collada <unit>), float32 positions and normals, uint32 indices.
struct MeshPart {
    std::string material;  // the file's material name; may be empty
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<std::uint32_t> indices;
    std::array<float, 4> rgba{0.70F, 0.70F, 0.72F, 1.0F};
    bool hasColor{false};  // the file gave a diffuse or base colour
};

struct ImportedMesh {
    std::vector<MeshPart> parts;
    std::vector<std::string> notes;  // what was left out (textures, lines, points)
};

/// Reads a glTF 2.0, OBJ, STL or Collada file (any format Assimp reads) for display: the node
/// hierarchy is baked in, faces are triangulated, missing normals are generated (smooth below
/// 30°), and the triangles are grouped by material. Collada's <up_axis> is ignored, so the
/// geometry keeps the axes it was authored in, as ROS tools expect of meshes in URDF files.
/// Textures, animations and skins are not imported (formats.md §9). Import-time only, never in
/// step() (CLAUDE.md §4).
[[nodiscard]] Expected<ImportedMesh, std::string> importMesh(const std::string& path);

/// The file a URDF/SDFormat mesh reference names: "package://pkg/rel" through the ament index,
/// "file:///abs", or a path relative to `baseDirectory` (the directory of the robot file).
[[nodiscard]] Expected<std::string, std::string> resolveMeshUri(const std::string& uri,
                                                                const std::string& baseDirectory);

}  // namespace lagrange::meshes
