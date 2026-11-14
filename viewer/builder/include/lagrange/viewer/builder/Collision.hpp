#pragma once

// How a link collides, the user's choice (RFC 0016 Addendum A): the edits behind the Properties
// "Collision" section. Pure document edits; the split itself (CoACD) is CollisionSplit.hpp's.
//
// What each choice does to the link's shapes (the mesh: its first visual mesh shape):
//   Mesh    the mesh collides (visual, collision, solid); collision-only shapes are removed.
//   Split   the mesh is visual and solid only; the parts (setSplitParts) are collision-only
//           mesh shapes, not solid. Choosing it removes the other collision-only shapes.
//   Shapes, Drawn  the mesh is visual and solid only; collision-only primitives (templates,
//           Fit to mesh, or placed by hand in the view) collide, not solid. Choosing one
//           removes collision-only mesh shapes (split parts) and keeps the primitives.
// So the mass always comes from the mesh, and the collision is exactly what was chosen.

#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Edits.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

/// The mesh shape the choice is about: the link's first visual mesh shape; nullopt: none.
[[nodiscard]] std::optional<std::size_t> collisionMeshOf(const Link& link);

/// True for a shape that only collides (no visual): a template, a fitted or drawn shape, a part.
[[nodiscard]] bool collisionOnly(const Shape& shape);

/// Sets `link.collision` and arranges the shapes as above. Mesh and Split need a mesh shape:
/// without one only the choice is set.
void setCollisionChoice(Link& link, CollisionChoice choice);

/// The split's parts (mesh files, already scaled: their scale is 1) become the link's collision:
/// collision-only mesh shapes at the mesh's origin, not solid, in place of every other
/// collision-only shape. Sets the choice to Split.
void setSplitParts(Link& link, const std::vector<std::string>& files);

/// Adds `shape` as collision-only and not solid (a template, a fitted or a drawn shape) and
/// returns its index in link.shapes.
std::size_t addCollisionShape(Link& link, Shape shape);

/// Add → Mesh… (Addendum A): a link on `parent` by a new joint of `type` (as addLink) with one
/// mesh shape naming `file` (visual, collision, solid; scale `scale`), its collision choice
/// Mesh, density `density` (kg/m³; 0: none). The link is named after the file's stem when that
/// is a valid name ("wheel.stl" → wheel, made unique), its joint `<link>_joint`. Empty names
/// when `parent` is not a link of the document.
[[nodiscard]] AddedLink addMeshLink(Document& document, std::string_view parent,
                                    const std::string& file, Joint::Type type, double density,
                                    const std::array<double, 3>& scale = {1, 1, 1});

}  // namespace lagrange::viewer::builder
