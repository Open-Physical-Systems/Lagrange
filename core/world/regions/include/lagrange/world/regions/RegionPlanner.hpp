#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace lagrange::regions {

/// Active regions (huge-worlds.md §3): a region is a bubble of full simulation around one or
/// more robots, with its own floating origin and its own physics scene. Two robots whose bubbles
/// overlap share a region; robots far apart get separate ones.
struct RegionSettings {
    double radius{0.0};  ///< m, world.streaming.active_region_radius. Required.
    /// Two regions merge when robots of each are closer than 2 radius; a region splits when
    /// its robots fall into groups farther apart than 2 radius times this.
    double splitFactor{1.25};
};

/// Which robots share a region: robot indices per region, each list ascending, the regions in
/// the order of their first robot. Robots and regions are numbered as given.
using RegionAssignment = std::vector<std::vector<std::uint32_t>>;

/// Groups robots into regions with hysteresis: robots within merge distance (2 radius) of each
/// other, directly or through others, share a region; robots already sharing one stay together
/// until they are farther apart than split distance (2 radius x splitFactor). A pure function
/// of the positions and the previous assignment, so deterministic.
class RegionPlanner {
public:
    explicit RegionPlanner(RegionSettings settings);

    /// The assignment for robots at `positions` (world ENU, m), given the previous one (empty
    /// at the start). Robots not in `previous` are placed by the merge rule alone.
    [[nodiscard]] RegionAssignment plan(std::span<const std::array<double, 3>> positions,
                                        const RegionAssignment& previous = {}) const;

    [[nodiscard]] const RegionSettings& settings() const { return settings_; }

private:
    RegionSettings settings_;
};

/// What moves with a robot from one region to another (huge-worlds.md §3: the hand-off is a
/// serialisable value): its base pose and velocity in world ENU and its joints' positions and
/// velocities. A robot's other dynamic state (electrical, thermal) is not carried.
struct AgentRecord {
    std::uint32_t robot{0};
    std::array<double, 3> position{};                       ///< world ENU, m
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};  ///< w, x, y, z
    std::array<double, 6> twist{};       ///< linear (m/s) then angular (rad/s), world frame
    std::vector<double> jointPositions;  ///< in the robot's joint order
    std::vector<double> jointVelocities;
};

}  // namespace lagrange::regions
