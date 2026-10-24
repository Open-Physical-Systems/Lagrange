#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/server/Profile.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace lagrange::server {

struct ServerOptions {
    proto::Endpoint endpoint;
    std::string robotPath;  // URDF of the robot to simulate
    std::string worldPath;  // URDF of static scenery (root link "world"); empty = none
    /// Overlay file applied to the robot (docs/design/formats.md §9: motors, battery, bus);
    /// empty = none
    std::string overlayPath;
    Pose spawn;                     // where the robot starts, world ENU (LoadOptions::spawn)
    std::string profileSchemaPath;  // profiles/schema.json, sent to viewers; empty = none
    /// What a profile's `stage` must say to apply to this run; empty: robotPath. `lagrange run`
    /// sets it to the stage as the profile writes it.
    std::string stageName;
    double realTimeFactor{1.0};  // 0 = as fast as possible
    bool ros{false};             // run the ROS 2 bridge (rclcpp::init must have been called)
    bool startPaused{false};
    std::string recordPath;                   // MCAP file to record the run to; empty: no recording
    std::uint32_t recordMaxComponents{1024};  // larger signals (3D lidar ranges) are left out
    /// Domains replaced by their substitute (CLAUDE.md §11.2): "rigid_body" (fixed robot) and
    /// "sensors" (ground truth); the others have no module yet and are always substituted.
    std::set<std::string> substituted;
    /// Parameters of an in-process ros2_control controller manager (update_rate, controller
    /// types); empty: the built-in drive controller instead. Needs the ROS bridge and a URDF
    /// robot whose <ros2_control> block uses lagrange_ros2_control/LagrangeSystem.
    std::string ros2ControlParams;
    bool stampedCmdVel{true};  // /cmd_vel as TwistStamped; false: Twist
    /// Drive the demo figure-eight until the first /cmd_vel; false: stand still until then
    /// (a navigation stack that is still starting up should find the robot where it began).
    bool demoDrive{true};
    /// Where lidars cast: Auto and Cpu on the CPU (Tier 0: exact casts culled by Embree,
    /// deterministic, the fastest for scans of tens of thousands of beams); Gpu on a hardware
    /// GPU's compute kernel (Tier 1), which makes the run non-deterministic.
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

    /// Loads the robot and the scenery and wires the simulation, without listening. start()
    /// does this first when it has not been done.
    Status load();
    /// Loads (if needed), then starts listening.
    Status start();

    /// After load(): the wiring as a Graphviz digraph (Scheduler::dumpGraphDot).
    [[nodiscard]] std::string wiringDot() const;
    /// After load(): checks a profile against the schema and this run, and that its settings
    /// wire. Needs ServerOptions::profileSchemaPath.
    [[nodiscard]] ProfileCheck validateProfile(const std::string& yaml);
    /// After load(): applies a valid profile, rebuilding the run at t = 0.
    Status applyProfile(const std::string& yaml);
    /// After load(), without start(): advances the simulation to `t` as fast as it can (no
    /// viewers, no wall clock), for headless checks such as `lagrange determinism`.
    Status stepUntil(SimTime t);
    /// After load(): the hash of the run's whole state (Scheduler::stateHash).
    [[nodiscard]] Expected<std::uint64_t, Status> stateHash() const;

    /// After load(): every signal on the bus, with its unit ("N·m") and components.
    struct SignalEntry {
        std::string name;
        std::string unit;
        std::uint32_t components{1};
    };
    [[nodiscard]] std::vector<SignalEntry> signals() const;
    /// After load(): the latest value of a signal; empty for an unknown name.
    [[nodiscard]] std::vector<double> read(const std::string& name) const;
    /// After load(): the simulated time.
    [[nodiscard]] SimTime now() const;
    /// After load(): the world (bodies, joints, colliders, sensors) as loaded.
    [[nodiscard]] const World& world() const;
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
