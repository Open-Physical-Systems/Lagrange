#pragma once

// Mesh shapes of the builder (RFC 0016 Addendum A): mesh files read by lagrange_meshes, the
// reader the server uses (open question 8), their mass from volume integrals (Mirtich 1996),
// the convex hull the server collides them as (core/world's convexHullOf, the same call), and
// "Fit to mesh" for the template shapes.
//
// The builder library links lagrange_meshes and so lagrange_core for this file only (open
// question 8, accepted); nothing of lagrange_core appears in this header, so the panels that
// include it stay free of core.

#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

/// A closed or open triangle surface: points (m) and triangles by index.
struct Surface {
    std::vector<std::array<double, 3>> points;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

/// What the solid of a mesh shape is, at one scale.
struct MeshSolid {
    /// At density 1 kg/m³ (scale by the density), in the mesh's frame (the shape's origin not
    /// applied): mass is then the volume in m³.
    MassProperties unit;
    bool closed{false};           ///< the mass is the closed mesh's own
    bool fromHull{false};         ///< the mesh is not closed: the mass is its convex hull's
    std::optional<Surface> hull;  ///< the convex hull the server collides; nullopt: flat
    /// How much of the hull the closed mesh fills (core's hullFill); 1 for an open mesh.
    double fill{1.0};
    /// fill below core's kConcaveFill: the server's hull fills in what is hollow.
    bool concave{false};
};

/// A mesh file as the builder sees it, read once (until the file changes).
class MeshFile {
public:
    std::string path;   ///< the resolved file
    std::string error;  ///< why it did not load; empty when it did
    /// For drawing: one mesh per material of the file, unscaled, in the file's frame.
    std::vector<MeshData> parts;
    /// Per part: the file's colour (linear 0..1), or all zero when the file gives none.
    std::vector<std::array<float, 4>> colors;
    /// Every part's triangles together, vertices welded where they are equal (unscaled).
    Surface welded;
    /// Every edge of `welded` is shared by exactly two triangles that run it both ways.
    bool closed{false};
    std::vector<std::string> notes;  ///< what the reader left out (textures, lines)

    /// The solid at `scale` (cached per scale; thread-safe).
    [[nodiscard]] const MeshSolid& solid(const std::array<double, 3>& scale) const;
    /// The vertices of every part as the server merges them (float, unscaled), for convexHullOf.
    std::vector<float> positions;
    std::vector<std::uint32_t> indices;

    MeshFile();
    ~MeshFile();
    MeshFile(const MeshFile&) = delete;
    MeshFile& operator=(const MeshFile&) = delete;

private:
    struct Cache;
    std::unique_ptr<Cache> cache_;
};

/// The folder relative mesh paths are read against (the document's file's folder; empty: the
/// current directory). Set by BuilderHost from the session's path; read by mass and drawing.
void setMeshBase(const std::filesystem::path& base);
[[nodiscard]] std::filesystem::path meshBase();

/// Reads the mesh `uri` (as a URDF names it: package://, lagrange-asset://, file://, absolute
/// or relative to `base`) through lagrange_meshes. Cached by resolved path and modification
/// time; a file that cannot be found or read gives a MeshFile with `error` set. Never null.
[[nodiscard]] std::shared_ptr<const MeshFile> loadMesh(const std::string& uri,
                                                       const std::filesystem::path& base);
/// loadMesh(uri, meshBase()).
[[nodiscard]] std::shared_ptr<const MeshFile> loadMesh(const std::string& uri);

/// Mass properties of the solid a closed, consistently wound triangle surface bounds, of
/// uniform density `density` (kg/m³), about its centre of mass, axes of the surface's frame:
/// Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties", J. Graphics Tools
/// 1(2), 1996 (projection, face and volume integrals). Either winding: an inward-wound
/// surface gives the same result. Mass 0 for an empty or flat surface.
[[nodiscard]] MassProperties volumeIntegrals(const Surface& surface, double density);

/// The convex hull core/world's convexHullOf makes of these vertices at `scale` (what the server
/// collides a mesh as), wound outwards; nullopt when it is flat.
[[nodiscard]] std::optional<Surface> serverHull(const MeshFile& mesh,
                                                const std::array<double, 3>& scale);

/// A surface as a flat-shaded mesh to draw (one normal per triangle).
[[nodiscard]] MeshData surfaceMesh(const Surface& surface);

/// "Fit to mesh" (Addendum A): a shape of `kind` around the mesh shape `mesh`, in the link frame
/// (the mesh's origin and scale applied). Box: the tightest box about the mesh's principal axes
/// of inertia or about its own axes, whichever is smaller (a part modelled square to its frame
/// fits best there). Cylinder and Capsule: about one of those six axes, the one of least
/// volume; the cross-section is the smallest enclosing circle, and a capsule's straight part
/// the shortest that holds every point. Sphere: the smallest enclosing sphere (Welzl 1991).
/// Fitted to every vertex of the file. The shape is collision-only and not solid. nullopt when
/// the mesh did not load or is flat, or `kind` is Mesh.
[[nodiscard]] std::optional<Shape> fitToMesh(const Shape& mesh, Shape::Kind kind);

}  // namespace lagrange::viewer::builder
