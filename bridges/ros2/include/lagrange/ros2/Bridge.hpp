#pragma once

#include "lagrange/ros2/SignalTopics.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::ros2 {

/// One joint's command from /joint_commands.
struct JointCommand {
    enum class Mode : std::uint8_t { Position, Velocity, Effort };
    std::string joint;
    Mode mode{Mode::Position};
    double value{0.0};
};

struct BridgeSettings {
    std::string nodeName{"lagrange_ros2_bridge"};
    std::string odomFrame{"odom"};
    std::string baseLink;          // the robot's root body; empty: no /odom and no odom TF
    std::string robotDescription;  // URDF text for /robot_description; empty: not published
    double stateRateHz{50.0};      // /joint_states, /odom and the odom TF
    bool stampedCmdVel{true};      // /cmd_vel as TwistStamped (Nav2's default) or Twist
    /// A road vehicle: also /ackermann_cmd (ackermann_msgs/AckermannDriveStamped), steering
    /// angle and speed, as Autoware's interface (lagrange_autoware) and Ackermann stacks send.
    bool ackermannCmd{false};
    /// ros2_control drives the robot: its controllers publish /joint_states, /odom and the
    /// odom TF and take the velocity commands, so the bridge does none of these.
    bool controllersOwnState{false};
    /// Topics that ride on radio links (domains/comms.md): topic → link name. A message on a
    /// mapped topic is lost with the link's link/<l>/packet_error_rate (always while the link is
    /// down) and the rest arrive link/<l>/latency later; incoming /cmd_vel likewise.
    std::map<std::string, std::string> topicLinks;
    /// Signals published on topics and topics written into signals (--ros-topics, RFC 0008 §4).
    TopicMaps signalTopics;
    /// Set: /joint_commands (sensor_msgs/JointState) drives joints one by one: for each name,
    /// its position if given and finite, else its velocity, else its effort. On ROS's thread.
    std::function<void(const JointCommand&)> jointCommands;
};

/// A /cmd_vel message: forward speed and turn rate.
struct CmdVel {
    double linearMps{0.0};     // forward (x)
    double angularRadps{0.0};  // yaw rate (about z)
    double lateralMps{0.0};    // left (y), for robots that move sideways (multicopters)
    double verticalMps{0.0};   // up (z), likewise
    /// From /ackermann_cmd: the bicycle model's road-wheel angle (rad, positive left), which
    /// then stands for the turn rate.
    std::optional<double> steeringAngle;
};

/// What the viewer's ROS 2 workspace shows (lagrange.viewer.v1.RosGraph).
struct GraphSnapshot {
    struct Topic {
        std::string name;
        std::string type;
        std::uint32_t publishers{0};
        std::uint32_t subscribers{0};
        double rateHz{0.0};        // measured for the bridge's own topics, 0 for others
        double bandwidthBps{0.0};  // bits per second, the bridge's own topics
        bool fromBridge{false};
    };
    struct Node {
        std::string name;  // "/robot_state_publisher"
        std::vector<std::string> publishes;
        std::vector<std::string> subscribes;
        bool external{true};
    };
    std::vector<Topic> topics;
    std::vector<Node> nodes;
    std::string rmw;
    std::string distro;
    std::uint32_t domainId{0};
};

/// Outcome of a simulation_interfaces request; values are those of simulation_interfaces/Result
/// plus the extra codes of SetSimulationState.
enum class SimResult : std::uint8_t {
    Unsupported = 0,
    Ok = 1,
    NotFound = 2,
    IncorrectState = 3,
    Failed = 4,
    AlreadyInTargetState = 101,  // set_simulation_state
    IncorrectTransition = 103,
    NameNotUnique = 101,  // spawn_entity
    NameInvalid = 102,
    UnsupportedFormat = 103,
    NoResource = 104,
    NamespaceInvalid = 105,
    ResourceParseError = 106,
    InvalidPose = 109,
};

struct SimOutcome {
    SimResult result{SimResult::Ok};
    std::string message;
};

/// A simulation_interfaces SpawnEntity request.
struct SpawnRequest {
    std::string name;  // empty: the simulator picks one (with allowRenaming)
    bool allowRenaming{false};
    std::string uri;       // a file: a path, file://PATH or package://PACKAGE/PATH
    std::string resource;  // or the file's text, when uri is empty
    std::string entityNamespace;
    Pose pose;          // in `frame`
    std::string frame;  // "" or "world"
};

struct SpawnOutcome {
    SimOutcome outcome;
    std::string entityName;  // as get_entities lists it
};

/// simulation_interfaces/SimulationState values this server can be in or be asked for.
enum class SimState : std::uint8_t { Stopped = 0, Playing = 1, Paused = 2, Quitting = 3 };

/// simulation_interfaces/EntityCategory values.
enum class EntityCategory : std::uint8_t {
    Object = 0,
    Robot = 1,
    Human = 2,
    DynamicObject = 4,
    StaticObject = 5,
};

struct EntityDescription {
    std::string name;
    EntityCategory category{EntityCategory::Object};
    std::string description;
};

/// Ground-truth motion of an entity in the world frame: pose; twist and acceleration as linear
/// (m/s, m/s²) then angular (rad/s, rad/s²).
struct EntityMotion {
    Pose pose;
    std::array<double, 6> twist{};
    std::array<double, 6> acceleration{};
};

/// What the simulator does for the simulation_interfaces services. Called on the thread that
/// calls Bridge::handleRequests(), never on a ROS thread.
class SimulationHandler {
public:
    virtual ~SimulationHandler() = default;
    [[nodiscard]] virtual SimState simulationState() = 0;
    virtual SimOutcome setSimulationState(SimState target) = 0;
    /// Steps `ticks` base ticks; only while paused or stopped.
    virtual SimOutcome step(std::uint64_t ticks) = 0;
    /// Restarts the run at t = 0 with the initial state, keeping play or pause.
    virtual SimOutcome reset() = 0;
    [[nodiscard]] virtual std::vector<EntityDescription> entities() = 0;
    [[nodiscard]] virtual std::optional<EntityMotion> entityMotion(const std::string& name) = 0;
    /// Adds a robot to the running simulation (spawn_entity).
    virtual SpawnOutcome spawn(const SpawnRequest& request) {
        (void)request;
        return {{SimResult::Unsupported, "spawning is not supported"}, ""};
    }
    /// Takes a spawned or loaded robot out (delete_entity).
    virtual SimOutcome remove(const std::string& name) {
        (void)name;
        return {SimResult::Unsupported, "deleting is not supported"};
    }
    /// Takes every spawned robot out (reset_simulation with SCOPE_SPAWNED).
    virtual SimOutcome removeSpawned() { return {}; }
};

/// The ROS 2 bridge (roadmap 0.2-F1, F2): an rclcpp node in the server process.
///
/// It reads the signal bus after the scheduler's ticks and publishes, stamped with simulation
/// time: /clock; /joint_states for every hinge and slide joint; /odom and the odom → base TF
/// from the base body's pose and twist (ground truth); /tf_static for sensor frames; when
/// their sensors sample, /scan (sensor_msgs/LaserScan) for 2D lidars, /points
/// (sensor_msgs/PointCloud2: x, y, z, intensity, ring; organised rings x columns) for 3D
/// lidars, and /imu (sensor_msgs/Imu), each named so when the robot has one of the kind and
/// /<sensor>/scan, /<sensor>/points, /<sensor>/imu otherwise; and
/// /robot_description, latched. It subscribes /cmd_vel: geometry_msgs/TwistStamped, which Nav2
/// and ros2_control publish by default, or geometry_msgs/Twist (BridgeSettings::stampedCmdVel).
///
/// ROS callbacks run on the bridge's own executor thread; publish() runs on the caller's
/// thread between ticks and never blocks on the network (rclcpp publishing is asynchronous).
/// The design's per-consumer tick ring (architecture.md §10) replaces the direct bus reads when
/// the server gets its I/O threads.
class Bridge {
public:
    /// rclcpp must be initialised (rclcpp::init) before a bridge is made.
    explicit Bridge(BridgeSettings settings);
    ~Bridge();
    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;
    Bridge(Bridge&&) = delete;
    Bridge& operator=(Bridge&&) = delete;

    /// Resolves the signals to publish for this world on this bus. Call again after the
    /// scheduler is rebuilt (reset).
    void attach(const World& world, const SignalBus& bus);
    /// Publishes what changed up to `now`.
    void publish(const SignalBus& bus, SimTime now);
    /// The newest /cmd_vel if it arrived less than `maxAge` ago (wall clock).
    [[nodiscard]] std::optional<CmdVel> latestCommand(std::chrono::milliseconds maxAge) const;
    /// The robot that /odom, /ground_truth/odom, the odom TF, /robot_description and
    /// /ackermann_cmd are about, when it changes during the run (a robot spawned into an empty
    /// run, or deleted). Call before attach().
    void setRobot(const std::string& baseLink, const std::string& robotDescription,
                  bool ackermannCmd);
    /// What does not fit in the --ros-topics map after the last attach(): an unknown signal, or
    /// one the message cannot carry. The server refuses to run with any.
    [[nodiscard]] const std::vector<std::string>& topicProblems() const { return topicProblems_; }
    /// The module that writes the subscribed topics into their signals (nullptr when none).
    [[nodiscard]] std::unique_ptr<IModule> topicInputs() const;
    /// Messages lost on a link so far, by topic (the topics of BridgeSettings::topicLinks).
    [[nodiscard]] std::map<std::string, std::uint64_t> droppedOnLinks() const;
    /// The ROS graph as this node sees it. Cold path: about once a second.
    [[nodiscard]] GraphSnapshot graph();
    [[nodiscard]] const std::string& nodeName() const { return settings_.nodeName; }

    /// Offers the simulation_interfaces services (roadmap 0.2-F4) under the root namespace:
    /// get_simulator_features, get_simulation_state, set_simulation_state, step_simulation,
    /// reset_simulation, get_entities, get_entities_states, get_entity_state, get_entity_info,
    /// spawn_entity, spawn_entities, delete_entity.
    /// The other services of the standard answer RESULT_FEATURE_UNSUPPORTED with a reason.
    /// Requests wait for handleRequests(); `handler` must outlive the bridge.
    void serveSimulation(SimulationHandler& handler);
    /// Runs the waiting simulation_interfaces requests on the caller's thread.
    void handleRequests();

private:
    struct Impl;
    BridgeSettings settings_;
    std::unique_ptr<SignalTopics> signalTopics_;
    std::vector<std::string> topicProblems_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::ros2
