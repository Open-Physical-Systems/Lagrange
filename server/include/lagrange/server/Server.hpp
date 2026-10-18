#pragma once

#include "lagrange/module/Status.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <cstdint>
#include <memory>
#include <set>
#include <string>

namespace lagrange::server {

struct ServerOptions {
    proto::Endpoint endpoint;
    std::string robotPath;          // URDF of the robot to simulate
    std::string worldPath;          // URDF of static scenery (root link "world"); empty = none
    Pose spawn;                     // where the robot starts, world ENU (LoadOptions::spawn)
    std::string profileSchemaPath;  // profiles/schema.json, sent to viewers; empty = none
    double realTimeFactor{1.0};     // 0 = as fast as possible
    bool ros{false};                // run the ROS 2 bridge (rclcpp::init must have been called)
    bool startPaused{false};
    std::string recordPath;                   // MCAP file to record the run to; empty: no recording
    std::uint32_t recordMaxComponents{1024};  // larger signals (3D lidar ranges) are left out
    /// Domains replaced by their substitute (CLAUDE.md §11.2): "rigid_body" (fixed robot) and
    /// "sensors" (ground truth); the others have no module yet and are always substituted.
    std::set<std::string> substituted;
    bool stampedCmdVel{true};  // /cmd_vel as TwistStamped; false: Twist
    /// Where lidars cast: Auto puts them on a hardware GPU (Tier 1 or 2) when the run is not
    /// deterministic (it takes ROS input) and on the CPU otherwise; Gpu also on a deterministic
    /// run, which then is not one; Cpu always on the CPU reference.
    enum class Lidar : std::uint8_t { Auto, Cpu, Gpu } lidar{Lidar::Auto};
    std::string gpu;  // Vulkan device for the lidar (index or part of its name); empty: automatic
};

/// The simulation server (roadmap 0.2-E1, E2): it loads a robot, wires the MuJoCo backend and
/// the demo driver on the scheduler, runs them against the wall clock, and serves the viewer
/// protocol (lagrange.viewer.v1) to any number of viewers, exactly as the mock server does. The
/// viewer needs no change to connect to it.
///
/// Single-threaded: run() alternates between socket I/O and simulation steps, never blocking
/// for longer than one viewer frame.
class Server {
public:
    explicit Server(ServerOptions options);
    ~Server();
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;
    Server(Server&&) = delete;
    Server& operator=(Server&&) = delete;

    /// Loads the robot, builds the simulation and starts listening.
    Status start();
    /// One iteration: waits up to `timeoutMs` for viewers, advances simulation time by the wall
    /// time that passed (times the real-time factor), and sends what changed.
    void runOnce(int timeoutMs);

    /// The endpoint actually bound (with the real port when port 0 was asked for).
    [[nodiscard]] const proto::Endpoint& endpoint() const;
    [[nodiscard]] SimTime simTime() const;
    [[nodiscard]] bool running() const;
    /// A client asked the simulator to quit (simulation_interfaces STATE_QUITTING).
    [[nodiscard]] bool quitRequested() const;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::server
