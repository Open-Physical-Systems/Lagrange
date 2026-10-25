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
};

/// The network of `world`'s RoadLaneAPI_1 lanes, with signal groups: per junction, approaches
/// facing each other share a group.
[[nodiscard]] LaneGraph lanesOf(const WorldView& world);

/// How many agents, and their seed.
struct Population {
    int vehicles{8};
    int pedestrians{6};
    std::uint64_t seed{1};
};

/// Agent dimensions (m): a mid-size car and a pedestrian (illustrative).
inline constexpr std::array<double, 3> kCarSize{4.5, 1.8, 1.5};
inline constexpr double kPedestrianRadius = 0.3;
inline constexpr double kPedestrianHeight = 1.75;

/// Adds the population's agents to `model` as kinematic bodies with collision shapes and drawn
/// prims (a robot named "traffic"), placed where the module starts them: vehicles on driving
/// lanes, pedestrians on sidewalks, spread out and away from `avoid` (the robots' positions).
/// Returns how many it placed (fewer when the lanes are full).
std::array<int, 2> addAgents(Model& model, const Population& population,
                             const std::vector<Vec3>& avoid);

/// Fixed-cycle lights (illustrative timing): each junction's groups take turns, green, yellow,
/// then all red.
inline constexpr double kGreenSeconds = 20.0;
inline constexpr double kYellowSeconds = 3.0;
inline constexpr double kAllRedSeconds = 2.0;
enum class Light : std::uint8_t { Red = 0, Yellow = 1, Green = 2 };
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
/// A·exp(−d/B), A = 2 m/s², B = 0.3 m).
///
/// Inputs: the robots' body poses and twists (config "ego_bodies"). Outputs: each agent's
/// body/<agent>/pose_command; traffic/agent_count; traffic/agents (8 per agent: class 0 car
/// 1 pedestrian, x, y, z, yaw, speed, length, width); traffic/lights (per lane into a junction:
/// the lane's road id and its light, 0 red 1 yellow 2 green).
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
        SignalId poseCommand;
        Vec3 position{};
        double yaw{0.0};
    };
    struct Ego {
        SignalId pose, twist;
    };
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
};

}  // namespace lagrange::traffic
