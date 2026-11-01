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

}  // namespace lagrange::server
