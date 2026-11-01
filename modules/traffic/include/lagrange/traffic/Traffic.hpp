#pragma once

// The traffic domain, builtin backend (docs/design/domains/traffic.md): road vehicles that follow
// the stage's RoadLaneAPI_1 lanes, pedestrians on its sidewalks, and fixed-cycle traffic lights
// at its junctions. Agents are kinematic bodies (World::Bodies::kinematic) that the module moves
// by their pose commands, so sensors see them and the robot can touch them.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/RoadLanes.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::traffic {

using Vec3 = std::array<double, 3>;

using Lane = RoadLane;

struct LaneGraph {
    std::vector<Lane> lanes;  // roadLanesOf's
    /// Traffic lights: per lane into a junction (a non-junction lane whose successors are
    /// junction lanes), its signal group within its junction; −1 for other lanes.
    std::vector<int> signalGroup;
    std::vector<int> junctionOf;  // the junction (an index) a lane leads into, or −1
    int junctions{0};
    /// Per junction: whether it has lights (lanes that conflict, or a crossing): a plain bend or
    /// a road's joint has none.
    std::vector<bool> signalled;
    /// Pedestrian crossings: sidewalk lanes that cross driving lanes.
    struct Crossing {
        std::size_t lane{0};
        int junction{-1};
        /// The junction's driving lanes it crosses, and how far along each.
        std::vector<std::pair<std::size_t, double>> over;
    };
    std::vector<Crossing> crossings;
    /// Where walkers go: per sidewalk lane, the sidewalk lanes that touch its start and its end.
    std::vector<std::array<std::vector<std::size_t>, 2>> walkLinks;
};

/// The network of `world`'s RoadLaneAPI_1 lanes, with signal groups: per junction, approaches
/// facing each other share a group.
[[nodiscard]] LaneGraph lanesOf(const WorldView& world);

/// A lamp of a signal: a kinematic body without a collider that the module shows at `lit`, in
/// front of its dark lens, or keeps at `dark`, inside the housing. So every client and camera that
/// draws bodies sees the signal change, and recordings hold it.
enum class LampKind : std::uint8_t { Red, Yellow, Green, DontWalk, Walk, Wait };
struct SignalLamp {
    std::string name;
    LampKind kind{LampKind::Red};
    std::size_t lane{0};  // the approach lane; for walk lamps, the index in crossings
    Pose lit, dark;
};
/// The lamps of `graph`'s lights and crossings, the same on every call.
[[nodiscard]] std::vector<SignalLamp> signalLamps(const LaneGraph& graph);

/// A person the traffic module will walk (a HumanAPI_1 person named `name`), on a sidewalk.
struct WalkerStart {
    std::string name;
    Pose ground;
    double height{1.75};     // m
    double walkSpeed{1.34};  // m/s
    std::int64_t seed{0};
};
/// Where `count` walkers start, on a world whose sidewalks have crossings; none on one without:
/// those worlds keep addAgents' pedestrians. Away from `avoid`.
[[nodiscard]] std::vector<WalkerStart> walkersFor(const WorldView& world, int count,
                                                  std::uint64_t seed,
                                                  const std::vector<Vec3>& avoid);

/// How many agents, and their seed.
struct Population {
    int vehicles{8};
    int pedestrians{6};
    std::uint64_t seed{1};
};

/// Agent dimensions (m): a mid-size car and a pedestrian (illustrative).
inline constexpr std::array<double, 3> kCarSize{4.5, 1.8, 1.5};
/// The body kinds of the traffic, length, width and height (m, illustrative): sedan, hatchback,
/// SUV, van, bus. Which vehicle is which follows from its index.
inline constexpr std::array<std::array<double, 3>, 5> kVehicleSizes{
    {{4.5, 1.8, 1.5}, {4.0, 1.75, 1.5}, {4.7, 1.9, 1.75}, {5.4, 2.0, 2.3}, {9.0, 2.5, 3.1}}};
[[nodiscard]] std::size_t vehicleKind(int index);
inline constexpr double kPedestrianRadius = 0.3;
inline constexpr double kPedestrianHeight = 1.75;

/// Adds the population's agents to `model` as kinematic bodies with collision shapes and drawn
/// prims (a robot named "traffic"), placed where the module starts them: vehicles on driving
/// lanes, pedestrians on sidewalks, spread out and away from `avoid` (the robots' positions).
/// Returns how many it placed (fewer when the lanes are full). The signals of the world's junctions
/// are drawn too: poles, housings and lamps (signalLamps). On a world with crossings pedestrians
/// are walkers instead (walkersFor), which the caller adds as people.
std::array<int, 2> addAgents(Model& model, const Population& population,
                             const std::vector<Vec3>& avoid);

/// Fixed-cycle lights (illustrative timing): each junction's groups take turns, green, yellow,
/// then all red.
inline constexpr double kGreenSeconds = 20.0;
inline constexpr double kYellowSeconds = 3.0;
inline constexpr double kAllRedSeconds = 2.0;
enum class Light : std::uint8_t { Red = 0, Yellow = 1, Green = 2 };
/// A junction with crossings is actuated: a walker's button cuts the green to kMinGreenSeconds, and
/// after the all red everything stops for kWalkSeconds of walk and a flashing clearance as long as
/// the longest crossing takes at kClearanceSpeed. A lone mid-block crossing stays green until
/// asked.
inline constexpr double kMinGreenSeconds = 8.0;
inline constexpr double kWalkSeconds = 7.0;
inline constexpr double kClearanceSpeed = 1.2;     // m/s (MUTCD 4E.06 assumes 3.5 ft/s, about 1.07)
inline constexpr double kClearHoldSeconds = 10.0;  // s, the longest wait for a junction to empty
enum class Walk : std::uint8_t { DontWalk = 0, Walk = 1, Flashing = 2 };
/// The light of signal group `group` of a junction with `groups` groups at time t.
[[nodiscard]] Light lightAt(int group, int groups, double t);

/// The Intelligent Driver Model's acceleration (Treiber, Hennecke and Helbing 2000):
/// a·[1 − (v/v0)⁴ − (s*/gap)²], s* = s0 + v·T + v·Δv/(2√(a·b)), with Δv = v − v_leader; free
/// road when gap is +inf.
struct Idm {
    double a{1.5};   // m/s², maximum acceleration
    double b{2.0};   // m/s², comfortable deceleration
    double s0{2.0};  // m, jam distance
    double T{1.5};   // s, time headway
};
[[nodiscard]] double idmAcceleration(const Idm& p, double v, double v0, double gap,
                                     double leaderSpeed);

/// The traffic domain, builtin backend, every 10 ms. Agents are the kinematic bodies that
/// addAgents made (traffic_vehicle_<i>, traffic_pedestrian_<i>).
///
/// Vehicles follow their lanes by the IDM behind the agent or robot ahead (on their lane or the
/// next), at the lane's speed limit (else 50 km/h), stop at red and yellow lights they can stop
/// for, and take a random successor at a lane's end (seeded per agent and decision); at the
/// network's edge they come back in at the start of a free entry lane. Into a junction they
/// wait while a junction lane that crosses or merges with theirs is taken, or while a vehicle
/// nearer its own line is about to take one (first come, first served; no priority rules). No
/// lane changes.
/// Pedestrians walk their sidewalk at 1.34 m/s (Weidmann 1993), turn back at its ends, and keep
/// away from each other and from the robot sideways by a social force (Helbing and Molnár 1995:
/// A·exp(−d/B), A = 2 m/s², B = 0.3 m). On a world with crossings, walkers (HumanAPI_1 people named
/// walker_<i>, whose steering and gait lagrange_humans does) go along the sidewalks by
/// human/<n>/goal and take crossings at random: at one they press the button unless someone has, or
/// wait for someone to and press after kPatience, and cross on walk only. Vehicles also stop for
/// anyone still on a crossing.
///
/// Inputs: the robots' body poses and twists (config "ego_bodies"). Outputs: each agent's
/// body/<agent>/pose_command; traffic/agent_count; traffic/agents (8 per agent: class 0 car
/// 1 pedestrian, x, y, z, yaw, speed, length, width); traffic/lights (per lane into a junction:
/// the lane's road id and its light, 0 red 1 yellow 2 green); traffic/crossings (3 per crossing:
/// the lane's road id, 0 don't walk 1 walk 2 flashing, whether the button has been pressed);
/// each lamp's pose command; each walker's human/<n>/goal.
///
/// With `zero` it is the zero substitute: no agents (they wait 100 m under the ground) and
/// every light red.
class Traffic final : public IModule {
public:
    explicit Traffic(bool zero = false) : zero_(zero) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Agent {
        bool pedestrian{false};
        std::size_t lane{0};
        double s{0.0};                    // m along the lane
        double speed{0.0};                // m/s, along the lane (pedestrians: signed)
        double offset{0.0};               // m, to the left of the centre line (pedestrians)
        std::optional<std::size_t> next;  // the lane chosen after this one
        std::uint64_t decisions{0};
        double height{0.0};
        double length{0.0}, width{0.0};  // m
        double tyre{0.33};               // m, its wheels' radius
        double wheel{0.0};               // rad, how far they have turned
        bool braking{false};
        int turning{0};  // +1 left, -1 right, 0 straight on
        SignalId wheelOut, brakeOut, leftOut, rightOut;
        SignalId poseCommand;
        Vec3 position{};
        double yaw{0.0};
    };
    struct Ego {
        SignalId pose, twist;
    };
    /// An actuated junction: one with crossings.
    struct Controller {
        enum class Phase : std::uint8_t { Green, Yellow, AllRed, Clearing, Walk, Flash, Emptying };
        int junction{-1};
        int groups{1};
        double flash{0.0};  // s, its longest crossing's clearance
        Phase phase{Phase::Green};
        int group{0};
        double elapsed{0.0};  // s in this phase
        bool requested{false};
    };
    struct Walker {
        enum class State : std::uint8_t {
            Along,
            ToCrossing,
            ToButton,
            Pressing,
            Waiting,
            Crossing
        };
        SignalId goal, pose, gesture;
        State state{State::Along};
        std::size_t lane{0};      // the sidewalk lane it is on
        bool forward{true};       // along the lane's direction
        std::size_t crossing{0};  // index, from ToCrossing to the end of Crossing
        bool crossingForward{true};
        double offset{0.0};  // m to the left of the way it goes
        double timer{0.0};   // s in this state
        std::uint64_t decisions{0};
        double x{0.0}, y{0.0};
    };
    struct LampOut {
        SignalId command;
        std::array<double, 7> lit{}, dark{};
    };
    void updateControllers(SignalBus& bus, double dt);
    void updateWalkers(SignalBus& bus, double dt, std::size_t agentCount);
    [[nodiscard]] Light lightOf(std::size_t lane, double t) const;
    [[nodiscard]] Walk walkOf(int junction, double t) const;
    [[nodiscard]] const Controller* controllerOf(int junction) const;
    void writeLamps(SignalBus& bus, double t);
    void place(Agent& a);
    std::size_t choose(Agent& a, std::size_t agentIndex, const std::vector<std::size_t>& options);
    /// Brings a vehicle back in at the start of a free entry lane; false when none is free.
    bool respawn(Agent& a, std::size_t agentIndex);
    void write(SignalBus& bus, double t);

    bool zero_;
    std::uint64_t seed_{1};
    LaneGraph graph_;
    std::vector<Agent> agents_;
    std::vector<Agent> start_;
    std::vector<Ego> ego_;
    SignalId count_, table_, lights_;
    std::vector<std::size_t> lightLanes_;  // lanes into junctions
    std::vector<std::size_t> entries_;     // lanes nothing leads into: where vehicles come in
    std::vector<std::vector<std::size_t>> conflicts_;  // per junction lane: lanes crossing it
    std::vector<Vec3> egos_;                           // the robots' positions this step
    Idm idm_;
    std::vector<Controller> controllers_;
    std::vector<int> controllerOfJunction_;  // per junction: index in controllers_, or −1
    std::vector<Walker> walkers_;
    std::vector<Walker> walkersStart_;
    std::vector<SignalLamp> lamps_;
    std::vector<LampOut> lampOut_;
    std::vector<int> onCrossing_;  // per crossing: how many walkers are on it
    SignalId crossingsOut_;
};

}  // namespace lagrange::traffic
