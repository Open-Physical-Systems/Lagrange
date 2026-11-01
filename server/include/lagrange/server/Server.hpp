#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/server/Profile.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <vector>

namespace lagrange::server {

struct ServerOptions {
    proto::Endpoint endpoint;
    /// The robot to simulate (URDF, SDFormat, MJCF or USD); empty: none, an empty run that
    /// robots are spawned into (Server::spawnRobot, simulation_interfaces' spawn_entity).
    std::string robotPath;
    std::string worldPath;  // URDF of static scenery (root link "world"); empty = none
    /// Overlay file applied to the robot (docs/design/formats.md §9: motors, battery, bus);
    /// empty = none
    std::string overlayPath;
    /// The site's ambient air temperature, K (--ambient-temperature; a profile's
    /// thermal.ambient_temperature overrides it). A site parameter with no default: without it
    /// the thermal domain does not run (CLAUDE.md §10).
    std::optional<double> ambientTemperatureK;
    /// The site's air density, kg/m³ (--air-density; a profile's rotor_aero.air_density
    /// overrides it). A site parameter with no default: rotors do not run without it.
    std::optional<double> airDensityKgM3;
    /// The weather (--weather KEY=VALUE,...: temperature K, pressure Pa, relative_humidity,
    /// visibility m, rain_rate m/s, dust_concentration kg/m³, dust_median_diameter m,
    /// solar_irradiance W/m², sky_temperature K; a profile's weather keys override them). The
    /// first three are site parameters with no default: the weather does not run without them
    /// (the temperature falls back to --ambient-temperature).
    std::map<std::string, double, std::less<>> weather;
    /// --weather's fields: fog_field and wind_field, NanoVDB files (.nvdb).
    std::map<std::string, std::string, std::less<>> weatherFiles;
    /// --link-topics: ROS topics that ride on radio links, topic → link (domains/comms.md).
    std::map<std::string, std::string> linkTopics;
    /// --traffic: the builtin traffic's agents when the world has road lanes (RoadLaneAPI_1);
    /// set when the server starts.
    /// --scenario: an OpenSCENARIO file to run (its roads become the world when --world is not
    /// given; its entity `scenarioEgo` is the robot, placed at its Init position).
    std::string scenarioPath;
    std::string scenarioEgo{"Ego"};
    int trafficVehicles{8};
    int trafficPedestrians{6};
    std::uint64_t trafficSeed{1};
    /// --humans N[,seed=S][,radius=M]: N people wandering within `radius` of the middle, placed
    /// clear of each other, the robots and the scenery (RFC 0009).
    int humanCount{0};
    std::uint64_t humanSeed{1};
    double humanRadius{10.0};  // m
    /// Where world ENU (0, 0, 0) is on the Earth (--geo-origin; a profile's world.geo_origin
    /// overrides it). A site parameter with no default: GNSS, magnetometers and barometers do
    /// not run without it.
    std::optional<coords::Geodetic> geoOrigin;
    /// PX4 SITL through the MAVLink bridge (--mavlink px4[:PORT]): the TCP port it connects to;
    /// none: the built-in multicopter controller flies a multicopter.
    std::optional<std::uint16_t> px4Port;
    /// ArduPilot SITL through its JSON interface (--mavlink ardupilot[:PORT]): the UDP port its
    /// servo frames come to.
    std::optional<std::uint16_t> ardupilotPort;
    /// Bench mode (--bench): physics replaced by this recording's rigid-body signals, replayed
    /// open loop (the replay substitute); empty = none. A profile's rigid_body replay source
    /// overrides it.
    std::string benchReplayPath;
    Pose spawn;  // where the robot starts, world ENU (LoadOptions::spawn)
    /// Security of TCP endpoints (v1.0, docs/design/security.md): a tls:// endpoint needs a
    /// certificate chain and key (PEM); a shared token, when given, must arrive in every TCP
    /// client's ClientHello. Listening beyond loopback without a token is refused.
    std::string tlsCertificate;
    std::string tlsKey;
    std::string authToken;
    /// More robots in the same run (--add-robot), after the robot. Their names are prefixed
    /// with `name` (prefixModel) and their joints are passive: nothing drives them yet.
    struct ExtraRobot {
        std::string name;
        std::string path;
        Pose spawn;  // world ENU
    };
    std::vector<ExtraRobot> extraRobots;
    /// Plugin libraries to load (--plugin; a profile's plugins for `lagrange run`): paths, or
    /// names found as lib<name>.so in $LAGRANGE_PLUGIN_PATH or the lagrange_plugins resource
    /// index (RFC 0008). Only these can be loaded: a profile from a viewer never adds one.
    std::vector<std::string> plugins;
    /// --ros-topics FILE.yaml: signals published on ROS topics and topics written into signals
    /// (RFC 0008 §4; ros2::readTopicMaps); empty = none.
    std::string rosTopicsPath;
    /// --joint-control: every moving joint of the robot is driven on its own, from
    /// /joint_commands (sensor_msgs/JointState) or commandJoint, with no robot controller.
    bool jointControl{false};
    /// Bandwidth of its position and velocity loops, Hz.
    double jointLoopHz{8.0};
    /// --water X,Y,LEVEL,SIZE_X,SIZE_Y,DEPTH: a body of still water (m); the surface at LEVEL.
    struct Water {
        std::array<double, 3> surface{};
        double sizeX{10.0}, sizeY{10.0}, depth{2.0};
    };
    std::vector<Water> water;
    /// --sensor-timing NAME=LATENCY[,JITTER]: s, of a sensor's messages on ROS (sensors.md).
    struct SensorTiming {
        std::string sensor;
        double latency{0.0}, jitter{0.0};
    };
    std::vector<SensorTiming> sensorTiming;
    /// --site NAME=VALUE: the site at the start, as Server::setSite takes them.
    std::map<std::string, double> site;
    std::string profileSchemaPath;  // profiles/schema.json, sent to viewers; empty = none
    /// What a profile's `stage` must say to apply to this run; empty: robotPath. `lagrange run`
    /// sets it to the stage as the profile writes it.
    std::string stageName;
    double realTimeFactor{1.0};  // 0 = as fast as possible
    bool ros{false};             // run the ROS 2 bridge (rclcpp::init must have been called)
    bool startPaused{false};
    std::string recordPath;                   // MCAP file to record the run to; empty: no recording
    std::uint32_t recordMaxComponents{1024};  // larger signals (3D lidar ranges) are left out
    /// Domains replaced by their substitute (CLAUDE.md §11.2): "rigid_body" (fixed robot),
    /// "electrical" (ideal torque source; also used when the robot has no motors) and "sensors"
    /// (ground truth); the others have no module yet and are always substituted.
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
    // How cameras draw colour: the viewport's realistic renderer on a GPU, else the simple shading,
    // which a software device always uses. Depth and instance id do not change.
    bool cameraRealistic{true};
    int cameraSeason{0};  // 0 summer, 1 fall, 2 winter, 3 rainy
    std::string gpu;      // Vulkan device for the lidar and cameras (index or part of its name)
    /// Render cameras (on `gpu`, or a software device without one), which makes the run
    /// non-deterministic; false: cameras publish their ground truth (empty images), as for
    /// `lagrange determinism`.
    bool cameras{true};
    std::string physics{"mujoco"};  // rigid-body backend: "mujoco" or "jolt" (--physics)
    /// Replay mode (roadmap 0.2-E8, Q15): serve this recording (`--record`'s MCAP) instead of a
    /// simulation. Every recorded domain is a replay substitute; the scene is the recorded one;
    /// the viewer can seek. No robot is loaded and the ROS bridge is off.
    std::string replayPath;
    /// Where edits from the viewer are saved when it asks (SetAttribute save_to_session_layer):
    /// the edited model as a USD layer. Empty: $XDG_STATE_HOME/lagrange/<robot>.session.usda.
    std::string sessionPath;
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
    /// After load(): the run's whole state (time, every module, the bus), to restore later in
    /// the same run (Scheduler::saveState); the physics conformance suite checks backends with it.
    [[nodiscard]] Expected<std::vector<std::byte>, Status> saveState() const;
    Status restoreState(std::span<const std::byte> state);
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
    /// After load(): adds a robot to the run (URDF, SDFormat, MJCF or USD) at `spawn` (world
    /// ENU, as --spawn) and returns its entity name ("/name"). In a run without a robot it
    /// becomes the robot (ROS drives it); otherwise it is passive, its names prefixed with
    /// `name` (as --add-robot). The run is rebuilt: time goes on, every body keeps its pose and
    /// (MuJoCo) every joint its position, velocities start from rest. `name` empty: from the
    /// file name, made unique.
    Expected<std::string, Status> spawnRobot(const std::string& path, const std::string& name,
                                             const Pose& spawn);
    /// After load(): changes the site while it runs and the run goes on from where it is:
    /// "gravity" (m/s², downward), "wind_speed" (m/s, at the reference height), "wind_from"
    /// (degrees, compass: where it blows from), "temperature" (K), "air_density" (kg/m³),
    /// "pressure" (Pa), "relative_humidity" (0..1), "rain_rate" (m/s), "visibility" (m).
    /// Wind acts on links with AeroAPI_1 and on rotors; a value that brings a domain in or
    /// takes one out (the first air density) rebuilds the run keeping time and poses.
    Status setSite(const std::string& name, double value);
    /// With joint control: drive a joint by "position" (rad, m), "velocity" (rad/s, m/s) or
    /// "effort" (N·m, N). False for another mode.
    bool commandJoint(const std::string& joint, const std::string& mode, double value);
    /// After load(): takes a robot out of the run by its entity name, rebuilt likewise.
    Status removeRobot(const std::string& name);
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
