#pragma once

// What a robot's joints must hold against gravity, from its file alone: `lagrange check`. Finds
// the effort limit that cannot hold the links it carries, or masses that are not what was meant,
// before a run shows them as an arm lying on the ground.

#include "lagrange/world/World.hpp"

#include <optional>
#include <string>
#include <vector>

namespace lagrange::server {

struct JointLoad {
    std::string name;
    double effortLimit{0.0};  ///< N·m or N, from the file; 0: none given
    double massCarried{0.0};  ///< kg, of everything beyond the joint
    double atRest{0.0};       ///< N·m or N of gravity at the pose the file starts in
    /// N·m or N in the worst pose its other joints can reach, taking every link straight out
    /// sideways: an upper bound. 0 for a vertical axis nothing above it can tilt.
    double worst{0.0};
    std::optional<double> motorMost;  ///< N·m or N a motor can give through its gear
};

/// Each joint of a robot fixed to the ground, base first. Empty for one that moves about freely:
/// its joints do not carry its weight in the same way.
[[nodiscard]] std::vector<JointLoad> jointLoads(const World& world);

/// A link's mass against the space its collision shapes take. Hulls fill in what is hollow, so the
/// density is a lower bound for a mesh; it tells a CAD export's wrong density from a heavy part.
struct LinkDensity {
    std::string name;
    double mass{0.0};    ///< kg
    double volume{0.0};  ///< m³ of its collision shapes; 0 when it has none
    /// mass / volume in kg/m³, when it has a volume; 0 otherwise.
    [[nodiscard]] double density() const { return volume > 0.0 ? mass / volume : 0.0; }
};

/// Above this no solid material is (osmium is 22,590 kg/m³): the mass or the shape is wrong.
inline constexpr double kDensestSolid = 22600.0;

[[nodiscard]] std::vector<LinkDensity> linkDensities(const World& world);

}  // namespace lagrange::server
