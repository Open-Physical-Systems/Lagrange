#pragma once

// OpenSCENARIO (ASAM OpenSCENARIO 1.x, XML): an own reader and runner for a subset of it
// (docs/design/domains/traffic.md, formats.md): the entities, their Init, the storyboard's
// stories, acts, maneuver groups, maneuvers and events, speed and lane-change actions, and the
// conditions that start them and stop the scenario. Scenario entities other than the ego are
// kinematic bodies that the runner moves along the road lanes; the ego is the robot.

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/RoadLanes.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::openscenario {

using Vec3d = std::array<double, 3>;

struct Position {
    enum class Kind { World, Lane };
    Kind kind{Kind::World};
    double x{0.0}, y{0.0}, z{0.0}, h{0.0};  // World
    int road{0};
    int lane{0};
    double s{0.0}, offset{0.0};  // Lane
};

/// How an action reaches its target: shape over a time (s), a distance (m) or at a rate.
struct Dynamics {
    enum class Shape { Step, Linear, Sinusoidal, Cubic };
    enum class Dimension { Time, Distance, Rate };
    Shape shape{Shape::Step};
    Dimension dimension{Dimension::Time};
    double value{0.0};
};
/// The fraction of a transition done at progress x in [0, 1] for a shape.
[[nodiscard]] double shapeAt(Dynamics::Shape shape, double x);

struct Action {
    enum class Kind { Teleport, Speed, LaneChange };
    Kind kind{Kind::Speed};
    std::string entity;                     // the actor it applies to
    Position position;                      // Teleport
    double speed{0.0};                      // Speed: absolute target (m/s)
    std::optional<std::string> relativeTo;  // Speed: relative to that entity's speed (+speed)
    Dynamics dynamics;
    // LaneChange: relative to an entity's lane (by `lanes` lanes, + left) or an absolute id.
    std::optional<std::string> laneRelativeTo;
    int lanes{0};
    std::optional<int> absoluteLane;
};

struct Condition {
    enum class Kind { SimulationTime, ReachPosition, RelativeDistance, Speed, TraveledDistance };
    enum class Rule { Greater, Less, Equal, GreaterOrEqual, LessOrEqual };
    enum class Edge { None, Rising, Falling, RisingOrFalling };
    Kind kind{Kind::SimulationTime};
    Rule rule{Rule::Greater};
    Edge edge{Edge::None};
    double value{0.0};
    double delay{0.0};
    std::vector<std::string> triggering;  // entities (any of them)
    std::string entityRef;                // RelativeDistance
    enum class Distance { Longitudinal, Lateral, Euclidean };
    Distance distance{Distance::Euclidean};
    bool freespace{false};
    Position position;  // ReachPosition
    double tolerance{1.0};
};

/// Condition groups (OR) of conditions (AND); empty: always true.
struct Trigger {
    std::vector<std::vector<Condition>> groups;
};

struct Event {
    std::string name;
    std::vector<Action> actions;
    Trigger start;
};

struct Act {
    std::string name;
    std::vector<std::string> actors;  // all its maneuver groups' actors
    std::vector<Event> events;        // its maneuvers' events, in order
    Trigger start;
};

struct Entity {
    std::string name;
    bool pedestrian{false};
    std::array<double, 3> size{4.5, 1.8, 1.5};  // bounding box (m)
};

struct Scenario {
    std::string path;         // the .xosc file
    std::string roadNetwork;  // the LogicFile, resolved; empty: none
    std::vector<Entity> entities;
    std::vector<Action> init;
    std::vector<Act> acts;
    Trigger stop;
    [[nodiscard]] const Entity* entity(const std::string& name) const;
};

/// Reads an .xosc file; an error naming what is not supported or wrong.
[[nodiscard]] Expected<Scenario, std::string> read(const std::string& path);
[[nodiscard]] Expected<Scenario, std::string> parse(const std::string& xml,
                                                    const std::string& directory);

/// A Position as a world pose (x, y, z, heading), on `lanes` for lane positions.
[[nodiscard]] std::optional<std::array<double, 4>> poseOf(const Position& p,
                                                          const RoadLanes& lanes);

/// Adds the scenario's entities other than `ego` to `model` as kinematic bodies
/// (scenario_<name>), at their Init teleport positions; an error for an entity without one.
[[nodiscard]] Expected<void, std::string> addEntities(Model& model, const Scenario& scenario,
                                                      const std::string& ego);

/// Runs a scenario every 10 ms: moves its entities along the road lanes by their speed and
/// lane-change actions, starts acts and events when their triggers hold, and ends the scenario
/// when its stop trigger holds. Reads the ego's body pose and twist (config "ego_body");
/// writes body/scenario_<name>/pose_command, scenario/state (0 running, 1 ended),
/// scenario/time (s since the start, frozen at the end) and scenario/entities (8 per entity:
/// x, y, z, heading, speed, distance travelled, lane's road, lane's id).
class ScenarioRunner final : public IModule {
public:
    ScenarioRunner(Scenario scenario, std::string ego)
        : scenario_(std::move(scenario)), egoName_(std::move(ego)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// What happened, for the log: "t=3.20 event cut_in started", ...
    [[nodiscard]] const std::vector<std::string>& events() const { return log_; }

private:
    struct Mover {
        std::string name;
        std::size_t lane{0};
        double along{0.0};
        double speed{0.0};
        double travelled{0.0};
        Vec3d position{};
        double heading{0.0};
        // A speed transition in progress.
        bool speeding{false};
        double fromSpeed{0.0}, toSpeed{0.0}, progress{0.0};
        Dynamics speedDynamics;
        // A lane change in progress.
        bool changing{false};
        std::size_t targetLane{0};
        double changeProgress{0.0};
        Dynamics changeDynamics;
        SignalId poseCommand;
    };
    struct State {
        std::vector<bool> actStarted;
        std::vector<std::vector<bool>> eventStarted;
        std::map<std::string, bool> lastValue;    // per condition, for edges
        std::map<std::string, double> trueSince;  // per condition, for delays
        bool ended{false};
        double endTime{0.0};
    };
    [[nodiscard]] bool holds(const Trigger& trigger, const std::string& key, double t);
    [[nodiscard]] bool evaluate(const Condition& c, double t) const;
    void apply(const Action& action, double t);
    void move(Mover& m, double dt);
    void place(Mover& m);
    [[nodiscard]] std::optional<std::array<double, 3>> stateOf(const std::string& entity,
                                                               double& speed, double& heading,
                                                               double& travelled) const;

    Scenario scenario_;
    std::string egoName_;
    RoadLanes lanes_;
    std::vector<Mover> movers_;
    std::vector<Mover> start_;
    State state_;
    SignalId egoPose_, egoTwist_, stateOut_, timeOut_, table_;
    Vec3d egoPosition_{};
    double egoSpeed_{0.0}, egoHeading_{0.0}, egoTravelled_{0.0};
    std::optional<Vec3d> egoLast_;
    std::vector<std::string> log_;
};

}  // namespace lagrange::openscenario
