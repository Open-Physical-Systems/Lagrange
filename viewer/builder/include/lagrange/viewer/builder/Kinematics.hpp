#pragma once

// The link tree and forward kinematics of a document (RFC 0016 §3, SPEC "Joints"). Owner: WP-A.
// Forward kinematics only: the joint preview poses the robot; nothing is simulated.

#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"

#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

/// A rigid transform: position in m, rotation as a unit quaternion {w, x, y, z}.
struct Frame {
    std::array<double, 3> position{};
    std::array<double, 4> rotation{1, 0, 0, 0};  // w x y z
};

/// The tree derived from the joints: one parent per link, one root, no loops.
struct Tree {
    /// Per link, in document order: the index into Document::joints of the joint whose child
    /// it is; -1 for the root (and for any link no joint reaches when !ok).
    std::vector<int> parentJoint;
    int root{-1};       ///< index into Document::links; -1 when there is no single root
    bool ok{false};     ///< false: loops, two roots, a joint naming a missing link, ...
    std::string error;  ///< why not ok, for Problems (one line, names the item)
};

[[nodiscard]] Tree buildTree(const Document& document);

/// Joint preview positions by joint name: rad for revolute and continuous, m for prismatic.
/// Missing joints are at 0. Mimic joints follow their leader (multiplier · leader + offset)
/// whatever the map says.
using Preview = std::map<std::string, double, std::less<>>;

/// Each link's frame in the root link's frame (the root at the identity), in document order
/// (same size as document.links). A link the tree does not reach is at the identity. Only
/// SceneFrame::transforms change with the preview (plan decision 2).
[[nodiscard]] std::vector<Frame> linkFrames(const Document& document, const Tree& tree,
                                            const Preview& preview);

/// The whole robot's mass properties at zero preview, in the root link's frame (the Robot
/// strip's mass and COM).
[[nodiscard]] MassProperties robotMass(const Document& document);

/// Composition helpers, exact as written (no normalisation of `b`'s rotation).
[[nodiscard]] Frame compose(const Frame& a, const Frame& b);  // a then b: a ∘ b
[[nodiscard]] Frame toFrame(const Pose& pose);                // URDF rpy (fixed x, y, z)

}  // namespace lagrange::viewer::builder
