#pragma once

#include "lagrange/module/Status.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/regions/RegionPlanner.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::server {

/// Several robots in one large world, in active regions (huge-worlds.md §3): robots whose
/// bubbles overlap share a run; robots far apart run in separate ones, each with its own
/// floating origin, physics scene and streamed tiles, stepped in lockstep. Headless: no ROS
/// and no viewers (a region is a Server that could be served on its own later).
struct RegionHostOptions {
    /// What every region's run shares: the world, the profile's settings, the real-time
    /// factor (ros and the robot are set per region).
    ServerOptions base;
    /// Every robot: its name, file and start. In each region the first robot listed is the
    /// region's robot (driven by its demo or controller), the others are passive extras.
    std::vector<ServerOptions::ExtraRobot> robots;
    regions::RegionSettings regions;
};

class RegionHost {
public:
    explicit RegionHost(RegionHostOptions options);
    ~RegionHost();
    RegionHost(const RegionHost&) = delete;
    RegionHost& operator=(const RegionHost&) = delete;
    RegionHost(RegionHost&&) = delete;
    RegionHost& operator=(RegionHost&&) = delete;

    /// Groups the robots by their starts and loads one run per region.
    Status load();
    /// Steps every region to `t`, 100 ms at a time. Robots that would now group differently
    /// are reported (Status Unsupported): moving a robot between regions is not built yet.
    Status stepUntil(SimTime t);

    [[nodiscard]] const regions::RegionAssignment& assignment() const { return assignment_; }
    [[nodiscard]] Server& region(std::size_t index) { return *servers_.at(index); }
    [[nodiscard]] std::size_t regionCount() const { return servers_.size(); }
    /// A robot's first body in world ENU (its region's origin added).
    [[nodiscard]] std::array<double, 3> robotPosition(std::uint32_t robot) const;

private:
    RegionHostOptions options_;
    regions::RegionPlanner planner_;
    regions::RegionAssignment assignment_;
    std::vector<std::unique_ptr<Server>> servers_;
};

}  // namespace lagrange::server
