#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/World.hpp"

#include <string>

namespace lagrange::server {

struct MapSettings {
    double heightM{0.23};      // slice height: where a 2D lidar sees
    double resolutionM{0.05};  // cell size
    double marginM{1.0};       // free border around the scenery
};

/// Writes an occupancy map of the world's fixed geometry for nav2_map_server: `yamlPath` and,
/// next to it, a PGM image (black = occupied, white = free, trinary mode).
///
/// A cell is occupied when its centre, at `heightM`, lies inside a collision shape that never
/// moves (static scenery and bodies welded to the world). Moving bodies, the robot among them,
/// are left out. Exact for every ShapeType; the ground plane is ignored.
[[nodiscard]] Expected<void, std::string> exportOccupancyMap(const World& world,
                                                             const std::string& yamlPath,
                                                             const MapSettings& settings = {});

}  // namespace lagrange::server
