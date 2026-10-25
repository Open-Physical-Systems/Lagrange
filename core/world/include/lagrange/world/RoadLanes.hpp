#pragma once

// A world's road lanes (RoadLaneAPI_1, written by the OpenDRIVE importer) as a graph, for
// traffic agents and scenario runners. Read once, in declare or init (invariant 5).

#include "lagrange/world/World.hpp"

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lagrange {

struct RoadLane {
    std::string name;
    std::vector<std::array<double, 3>> points;  // centre line, in the direction of travel
    std::vector<double> distance;               // along it, from its start (m)
    double width{3.5};
    double speedLimit{0.0};  // m/s; 0: none
    bool sidewalk{false};
    bool junction{false};
    int road{0};
    int lane{0};                    // the OpenDRIVE lane id: negative runs along the road's s
    double sStart{0.0}, sEnd{0.0};  // the section's range along the road
    std::vector<std::size_t> successors, predecessors;  // indices in RoadLanes::lanes

    [[nodiscard]] double length() const { return distance.empty() ? 0.0 : distance.back(); }
    /// Position and heading (rad, about z) at `s` metres along the lane.
    [[nodiscard]] std::pair<std::array<double, 3>, double> at(double s) const;
    /// The nearest point of the lane to `p`: its distance along the lane, and the lateral
    /// distance, positive to the left.
    [[nodiscard]] std::pair<double, double> project(const std::array<double, 3>& p) const;
};

struct RoadLanes {
    std::vector<RoadLane> lanes;
    /// The lane of OpenDRIVE road `road`, lane `lane` at road position `s`, and the distance
    /// along it (its direction of travel) that `s` is.
    [[nodiscard]] std::optional<std::pair<std::size_t, double>> find(int road, int lane,
                                                                     double s) const;
    /// The lane next to lane `index` at its distance `along`, `step` lanes to the left (+1) or
    /// right (−1) of its direction of travel, going the same way; and the distance along it.
    [[nodiscard]] std::optional<std::pair<std::size_t, double>> neighbour(std::size_t index,
                                                                          double along,
                                                                          int step) const;
};

/// The driving and sidewalk lanes of `world`, linked.
[[nodiscard]] RoadLanes roadLanesOf(const WorldView& world);

}  // namespace lagrange
