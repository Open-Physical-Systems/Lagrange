// The traffic domain, builtin backend (Traffic.hpp, domains/traffic.md).
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/traffic/Traffic.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <random>

namespace lagrange::traffic {

using namespace literals;

namespace {

constexpr double kDefaultSpeed = 50.0 / 3.6;  // m/s, a lane without a limit
constexpr double kWalkSpeed = 1.34;           // m/s (Weidmann 1993)
constexpr double kSocialA = 2.0;              // m/s² (Helbing and Molnár 1995)
constexpr double kSocialB = 0.3;              // m
constexpr double kRelaxation = 0.5;           // s, from the social force to a sideways speed
constexpr double kMaxSideStep = 0.8;          // m/s, the fastest sideways step
constexpr double kLookAhead = 80.0;           // m, how far a vehicle looks for a leader
constexpr double kEgoLateral = 2.5;           // m: a robot this close to a lane is on it
constexpr double kHidden = -100.0;            // m, where the zero substitute keeps agents
constexpr double kVehicleSpacing = 12.0;      // m between vehicles when placing them
constexpr double kPedestrianSpacing = 3.0;    // m between pedestrians
constexpr double kAvoidRadius = 15.0;         // m kept free around the robots
constexpr double kEntryClearance = 15.0;      // m: an entry is free when nothing is closer
constexpr double kEntrySpeed = 8.0;           // m/s, a vehicle coming in at the network's edge
constexpr double kYieldMargin = 5.0;          // m before the line where right of way is decided
constexpr std::uint32_t kAgentClassCar = 0;
constexpr std::uint32_t kAgentClassPedestrian = 1;

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

double distance2d(const Vec3& a, const Vec3& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}

std::array<double, 4> yawQuat(double yaw) {
    return {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
}

/// The nearest point of a lane to `p`: (s along it, lateral distance, signed to the left).
std::pair<double, double> project(const Lane& lane, const Vec3& p) {
    double bestS = 0.0;
    double best = std::numeric_limits<double>::infinity();
    double side = 0.0;
    for (std::size_t i = 0; i + 1 < lane.points.size(); ++i) {
        const Vec3& a = lane.points[i];
        const Vec3& b = lane.points[i + 1];
        const double dx = b[0] - a[0];
        const double dy = b[1] - a[1];
        const double len2 = dx * dx + dy * dy;
        const double t =
            len2 > 0.0 ? std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2, 0.0, 1.0)
                       : 0.0;
        const double x = a[0] + t * dx - p[0];
        const double y = a[1] + t * dy - p[1];
        const double d = std::hypot(x, y);
        if (d < best) {
            best = d;
            bestS = lane.distance[i] + t * (lane.distance[i + 1] - lane.distance[i]);
            side = dx * (p[1] - a[1]) - dy * (p[0] - a[0]) >= 0.0 ? 1.0 : -1.0;
        }
    }
    return {bestS, side * best};
}

}  // namespace

/// Whether two polylines cross (or touch), in the plane.
bool crosses(const std::vector<Vec3>& a, const std::vector<Vec3>& b) {
    const auto orient = [](const Vec3& p, const Vec3& q, const Vec3& r) {
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    };
    for (std::size_t i = 0; i + 1 < a.size(); ++i) {
        for (std::size_t k = 0; k + 1 < b.size(); ++k) {
            const double d1 = orient(b[k], b[k + 1], a[i]);
            const double d2 = orient(b[k], b[k + 1], a[i + 1]);
            const double d3 = orient(a[i], a[i + 1], b[k]);
            const double d4 = orient(a[i], a[i + 1], b[k + 1]);
            if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))) {
                return true;
            }
        }
    }
    return false;
}

std::pair<Vec3, double> Lane::at(double s) const {
    if (points.size() < 2) {
        return {points.empty() ? Vec3{} : points.front(), 0.0};
    }
    s = std::clamp(s, 0.0, length());
    const auto it = std::upper_bound(distance.begin(), distance.end(), s);
    const std::size_t i =
        std::min(static_cast<std::size_t>(std::max<std::ptrdiff_t>(it - distance.begin() - 1, 0)),
                 points.size() - 2);
    const double span = distance[i + 1] - distance[i];
    const double t = span > 0.0 ? (s - distance[i]) / span : 0.0;
    const Vec3& a = points[i];
    const Vec3& b = points[i + 1];
    return {{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])},
            std::atan2(b[1] - a[1], b[0] - a[0])};
}

LaneGraph lanesOf(const WorldView& world) {
    LaneGraph g;
    const std::vector<const SchemaInstance*> instances = world.instancesOf("RoadLaneAPI_1");
    std::map<std::uint32_t, std::size_t> byPrim;
    for (const SchemaInstance* s : instances) {
        Lane lane;
        lane.name = std::string(world.signalInstanceName(s->prim));
        const std::span<const double> c = s->realArray("lane:centreLine");
        for (std::size_t i = 0; i + 2 < c.size(); i += 3) {
            lane.points.push_back({c[i], c[i + 1], c[i + 2]});
        }
        if (lane.points.size() < 2) {
            continue;
        }
        lane.distance.push_back(0.0);
        for (std::size_t i = 1; i < lane.points.size(); ++i) {
            lane.distance.push_back(lane.distance.back() +
                                    distance2d(lane.points[i], lane.points[i - 1]));
        }
        const std::span<const double> w = s->realArray("lane:width");
        lane.width = w.empty() ? 3.5 : w[w.size() / 2];
        lane.speedLimit = s->real("lane:speedLimit").value_or(0.0);
        lane.sidewalk = s->token("lane:type") == "sidewalk";
        lane.junction = s->integer("lane:junction").value_or(0) != 0;
        lane.road = static_cast<int>(s->integer("lane:road").value_or(0));
        if (const std::string_view type = s->token("lane:type").value_or("driving");
            type != "driving" && type != "sidewalk") {
            continue;
        }
        byPrim[s->prim.index] = g.lanes.size();
        g.lanes.push_back(std::move(lane));
    }
    // Links, now that every lane has its index.
    for (const SchemaInstance* s : instances) {
        const auto self = byPrim.find(s->prim.index);
        if (self == byPrim.end()) {
            continue;
        }
        for (const PrimId next : s->rel("lane:successors")) {
            if (const auto it = byPrim.find(next.index); it != byPrim.end()) {
                g.lanes[self->second].successors.push_back(it->second);
                g.lanes[it->second].predecessors.push_back(self->second);
            }
        }
    }
    // Junctions: groups of junction lanes reachable from each other's entries share one; each
    // approach (a lane into a junction) gets a signal group by its heading at the stop line.
    g.signalGroup.assign(g.lanes.size(), -1);
    g.junctionOf.assign(g.lanes.size(), -1);
    std::vector<int> junctionOfLane(g.lanes.size(), -1);
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (!g.lanes[i].junction || junctionOfLane[i] >= 0) {
            continue;
        }
        // Flood over junction lanes sharing an approach road or an exit road.
        std::vector<std::size_t> stack{i};
        junctionOfLane[i] = g.junctions;
        const auto visit = [&](std::size_t sibling) {
            if (g.lanes[sibling].junction && junctionOfLane[sibling] < 0) {
                junctionOfLane[sibling] = g.junctions;
                stack.push_back(sibling);
            }
        };
        while (!stack.empty()) {
            const std::size_t k = stack.back();
            stack.pop_back();
            for (const bool before : {true, false}) {
                for (const std::size_t outer :
                     before ? g.lanes[k].predecessors : g.lanes[k].successors) {
                    // Every lane of that road, and where its lanes lead or come from.
                    for (const Lane& other : g.lanes) {
                        if (other.junction || other.road != g.lanes[outer].road) {
                            continue;
                        }
                        for (const std::size_t sibling :
                             before ? other.successors : other.predecessors) {
                            visit(sibling);
                        }
                    }
                }
            }
        }
        ++g.junctions;
    }
    for (int j = 0; j < g.junctions; ++j) {
        // Approaches into junction j and their headings at the stop line.
        std::vector<std::pair<std::size_t, double>> approaches;
        for (std::size_t i = 0; i < g.lanes.size(); ++i) {
            const Lane& lane = g.lanes[i];
            if (lane.junction || lane.sidewalk) {
                continue;
            }
            const bool into = std::ranges::any_of(
                lane.successors, [&](std::size_t s) { return junctionOfLane[s] == j; });
            if (into) {
                approaches.emplace_back(i, lane.at(lane.length()).second);
                g.junctionOf[i] = j;
            }
        }
        // Group 0: approaches heading like the first one or the opposite way; group 1: across.
        if (approaches.empty()) {
            continue;
        }
        const double reference = approaches.front().second;
        for (const auto& [lane, heading] : approaches) {
            const double d = std::abs(std::remainder(heading - reference, std::numbers::pi));
            g.signalGroup[lane] = d < std::numbers::pi / 4.0 ? 0 : 1;
        }
    }
    return g;
}

Light lightAt(int group, int groups, double t) {
    const double phase = kGreenSeconds + kYellowSeconds + kAllRedSeconds;
    const double cycle = phase * groups;
    const double within = std::fmod(t, cycle) - group * phase;
    if (within < 0.0 || within >= phase) {
        return Light::Red;
    }
    if (within < kGreenSeconds) {
        return Light::Green;
    }
    return within < kGreenSeconds + kYellowSeconds ? Light::Yellow : Light::Red;
}

double idmAcceleration(const Idm& p, double v, double v0, double gap, double leaderSpeed) {
    const double free = 1.0 - std::pow(v / std::max(v0, 0.1), 4);
    if (!std::isfinite(gap)) {
        return p.a * free;
    }
    const double desired =
        p.s0 + std::max(0.0, v * p.T + v * (v - leaderSpeed) / (2.0 * std::sqrt(p.a * p.b)));
    const double ratio = desired / std::max(gap, 0.1);
    return p.a * (free - ratio * ratio);
}

std::array<int, 2> addAgents(Model& model, const Population& population,
                             const std::vector<Vec3>& avoid) {
    const LaneGraph g = lanesOf(WorldView(model.world));
    std::mt19937_64 rng(population.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<std::size_t> roads, walks;
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.lanes[i].junction || g.lanes[i].length() < 12.0) {
            continue;
        }
        (g.lanes[i].sidewalk ? walks : roads).push_back(i);
    }
    World& w = model.world;
    const auto robot = static_cast<std::uint32_t>(w.robots.size());
    w.robots.emplace_back("traffic");
    ScenePrim top;
    top.name = "traffic";
    top.path = "/traffic";
    top.typeName = "Xform";
    model.scene.prims.push_back(top);
    const auto topPrim = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
    static constexpr std::array<std::array<float, 4>, 6> kPaint{{{0.85F, 0.85F, 0.87F, 1.0F},
                                                                 {0.12F, 0.13F, 0.15F, 1.0F},
                                                                 {0.2F, 0.32F, 0.58F, 1.0F},
                                                                 {0.62F, 0.62F, 0.64F, 1.0F},
                                                                 {0.55F, 0.12F, 0.1F, 1.0F},
                                                                 {0.25F, 0.4F, 0.3F, 1.0F}}};
    const auto materialFor = [&model](const std::string& name, std::array<float, 4> rgba) {
        model.scene.materials.push_back({name, rgba});
        return static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    };
    const std::uint32_t carMesh = static_cast<std::uint32_t>(model.scene.meshes.size());
    model.scene.meshes.push_back(boxMesh(kCarSize[0], kCarSize[1], kCarSize[2]));
    const std::uint32_t walkerMesh = static_cast<std::uint32_t>(model.scene.meshes.size());
    model.scene.meshes.push_back(cylinderMesh(kPedestrianRadius, kPedestrianHeight, 16));
    const std::uint32_t walkerMaterial = materialFor("pedestrian", {0.3F, 0.45F, 0.75F, 1.0F});

    std::vector<Vec3> placed;
    const auto place = [&](const std::vector<std::size_t>& from, double spacing, bool pedestrian,
                           int index) -> bool {
        if (from.empty()) {
            return false;
        }
        for (int attempt = 0; attempt < 400; ++attempt) {
            const Lane& lane =
                g.lanes[from[static_cast<std::size_t>(unit(rng) * from.size()) % from.size()]];
            const double s = 6.0 + unit(rng) * (lane.length() - 12.0);
            const auto [p, yaw] = lane.at(s);
            const bool clear =
                std::ranges::none_of(placed,
                                     [&](const Vec3& q) { return distance2d(p, q) < spacing; }) &&
                std::ranges::none_of(
                    avoid, [&](const Vec3& q) { return distance2d(p, q) < kAvoidRadius; });
            if (!clear) {
                continue;
            }
            placed.push_back(p);
            const double height = pedestrian ? kPedestrianHeight : kCarSize[2];
            Pose pose;
            pose.position = {p[0], p[1], p[2] + height / 2.0};
            pose.orientation = yawQuat(yaw);
            const std::string name =
                std::string(pedestrian ? "traffic_pedestrian_" : "traffic_vehicle_") +
                std::to_string(index);
            // Masses and inertias are nominal: kinematic bodies are not moved by forces.
            const double mass = pedestrian ? 75.0 : 1500.0;
            const BodyId body = w.addBody(name, kWorldBody, pose, mass, {}, {1, 1, 1, 0, 0, 0});
            w.bodies.robot.back() = robot;
            w.bodies.kinematic.back() = true;
            if (pedestrian) {
                w.addCollider(body, ShapeType::Capsule,
                              {kPedestrianRadius, height / 2.0 - kPedestrianRadius, 0.0}, Pose{},
                              0.8);
            } else {
                w.addCollider(body, ShapeType::Box,
                              {kCarSize[0] / 2.0, kCarSize[1] / 2.0, kCarSize[2] / 2.0}, Pose{},
                              0.8);
            }
            model.bodyRestPose.push_back(pose);
            ScenePrim prim;
            prim.name = name;
            prim.path = "/traffic/" + name;
            prim.parent = topPrim;
            prim.typeName = "Mesh";
            prim.mesh = pedestrian ? walkerMesh : carMesh;
            prim.material =
                pedestrian
                    ? walkerMaterial
                    : materialFor(name, kPaint[static_cast<std::size_t>(index) % kPaint.size()]);
            prim.body = body.index;
            model.scene.prims.push_back(prim);
            model.bodyPrim.push_back(static_cast<std::uint32_t>(model.scene.prims.size() - 1));
            return true;
        }
        return false;
    };
    std::array<int, 2> made{0, 0};
    for (int i = 0; i < population.vehicles; ++i) {
        made[0] += place(roads, kVehicleSpacing, false, made[0]) ? 1 : 0;
    }
    for (int i = 0; i < population.pedestrians; ++i) {
        made[1] += place(walks, kPedestrianSpacing, true, made[1]) ? 1 : 0;
    }
    return made;
}

ModuleInfo Traffic::info() const {
    return zero_ ? ModuleInfo{"traffic", "zero", 0, 1, true, true, CapabilityTier::Tier0}
                 : ModuleInfo{"traffic", "builtin", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Traffic::preferredStep() const {
    return 10_ms;
}

Status Traffic::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    graph_ = lanesOf(world);
    seed_ = static_cast<std::uint64_t>(config.get<std::int64_t>("seed").value_or(1));
    const World& raw = world.raw();
    start_.clear();
    ego_.clear();
    lightLanes_.clear();
    for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
        const std::string& name = raw.bodies.names[b];
        const bool vehicle = name.starts_with("traffic_vehicle_");
        const bool pedestrian = name.starts_with("traffic_pedestrian_");
        if ((!vehicle && !pedestrian) || !raw.bodies.kinematic[b]) {
            continue;
        }
        Agent a;
        a.pedestrian = pedestrian;
        a.height = pedestrian ? kPedestrianHeight : kCarSize[2];
        a.poseCommand = d.output(
            spec("body/" + name + "/pose_command", SignalType::Pose, units::m, "world_local"));
        // Its start: the nearest lane of its kind to where addAgents put it.
        const Vec3 at = raw.bodies.poseInParent[b].position;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
            const Lane& lane = graph_.lanes[i];
            if (lane.sidewalk != pedestrian) {
                continue;
            }
            const auto [s, lateral] = project(lane, at);
            if (std::abs(lateral) < best) {
                best = std::abs(lateral);
                a.lane = i;
                a.s = s;
            }
        }
        if (!std::isfinite(best)) {
            return Status::error(
                StatusCode::InvalidConfig,
                name + ": no " + std::string(pedestrian ? "sidewalk" : "driving") + " lane for it");
        }
        a.speed = pedestrian ? kWalkSpeed : 0.0;
        start_.push_back(a);
    }
    for (const std::string& name :
         config.get<std::vector<std::string>>("ego_bodies").value_or(std::vector<std::string>{})) {
        Ego e;
        SignalSpec pose = spec("body/" + name + "/pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        SignalSpec twist =
            spec("body/" + name + "/twist", SignalType::Wrench, units::m_per_s, "world_local");
        twist.feedback = true;
        e.pose = d.input(pose);
        e.twist = d.input(twist);
        ego_.push_back(e);
    }
    // Junction lanes in conflict: their paths cross, or they end on the same lane (a merge).
    // Lanes leaving the same lane do not conflict: their vehicles queue on it in order.
    conflicts_.assign(graph_.lanes.size(), {});
    for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
        const Lane& a = graph_.lanes[i];
        if (!a.junction) {
            continue;
        }
        for (std::size_t k = i + 1; k < graph_.lanes.size(); ++k) {
            const Lane& b = graph_.lanes[k];
            if (!b.junction || a.predecessors == b.predecessors) {
                continue;
            }
            const bool merge = std::ranges::any_of(a.successors, [&](std::size_t s) {
                return std::ranges::count(b.successors, s) > 0;
            });
            if (merge || crosses(a.points, b.points)) {
                conflicts_[i].push_back(k);
                conflicts_[k].push_back(i);
            }
        }
    }
    entries_.clear();
    for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
        if (graph_.signalGroup[i] >= 0) {
            lightLanes_.push_back(i);
        }
        const Lane& l = graph_.lanes[i];
        if (!l.junction && !l.sidewalk && l.predecessors.empty()) {
            entries_.push_back(i);
        }
    }
    count_ = d.output(spec("traffic/agent_count", SignalType::Int, units::one));
    SignalSpec table = spec("traffic/agents", SignalType::ScalarArray, units::one);
    table.arrayLength = static_cast<std::uint32_t>(std::max<std::size_t>(start_.size(), 1) * 8);
    table_ = d.output(table);
    SignalSpec lights = spec("traffic/lights", SignalType::ScalarArray, units::one);
    lights.arrayLength =
        static_cast<std::uint32_t>(std::max<std::size_t>(lightLanes_.size(), 1) * 2);
    lights_ = d.output(lights);
    return {};
}

Status Traffic::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    agents_ = start_;
    for (Agent& a : agents_) {
        place(a);
    }
    write(bus, 0.0);
    return {};
}

void Traffic::place(Agent& a) {
    const Lane& lane = graph_.lanes[a.lane];
    auto [p, yaw] = lane.at(a.s);
    if (a.pedestrian) {
        // The offset is to the left of the lane's direction, whichever way the walker goes.
        p[0] -= std::sin(yaw) * a.offset;
        p[1] += std::cos(yaw) * a.offset;
        if (a.speed < 0.0) {
            yaw += std::numbers::pi;
        }
    }
    a.position = {p[0], p[1], p[2] + a.height / 2.0};
    a.yaw = yaw;
}

bool Traffic::respawn(Agent& a, std::size_t agentIndex) {
    std::vector<std::size_t> entries;
    for (const std::size_t lane : entries_) {
        const Lane& l = graph_.lanes[lane];
        const bool clear = std::ranges::none_of(agents_, [&](const Agent& o) {
            return !o.pedestrian && o.lane == lane && o.s < kEntryClearance;
        });
        if (clear && std::ranges::none_of(egos_, [&](const Vec3& p) {
                return std::hypot(p[0] - l.points.front()[0], p[1] - l.points.front()[1]) <
                       kEntryClearance;
            })) {
            entries.push_back(lane);
        }
    }
    if (entries.empty()) {
        return false;
    }
    a.lane = choose(a, agentIndex, entries);
    a.s = 0.0;
    const Lane& l = graph_.lanes[a.lane];
    a.speed = std::min(kEntrySpeed, l.speedLimit > 0.0 ? l.speedLimit : kDefaultSpeed);
    a.next.reset();
    if (!l.successors.empty()) {
        a.next = choose(a, agentIndex, l.successors);
    }
    return true;
}

std::size_t Traffic::choose(Agent& a, std::size_t agentIndex,
                            const std::vector<std::size_t>& options) {
    random::Stream rng(seed_, a.decisions++, static_cast<std::uint32_t>(agentIndex));
    return options[static_cast<std::size_t>(rng.uniform() * static_cast<double>(options.size())) %
                   options.size()];
}

Status Traffic::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const double t = context.t.seconds() + dt;
    if (zero_) {
        write(bus, t);
        return {};
    }
    // The robots, as obstacles and leaders.
    std::vector<std::pair<Vec3, double>> egos;  // position, speed
    egos_.clear();
    for (const Ego& e : ego_) {
        const std::span<const double> p = bus.readN(e.pose);
        const std::span<const double> v = bus.readN(e.twist);
        egos.push_back({{p[0], p[1], p[2]}, std::hypot(v[0], v[1])});
        egos_.push_back({p[0], p[1], p[2]});
    }
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        const Lane& lane = graph_.lanes[a.lane];
        if (a.pedestrian) {
            // Social force across the sidewalk from the other walkers and the robots.
            double push = 0.0;
            const double sideX = -std::sin(a.yaw);
            const double sideY = std::cos(a.yaw);
            const auto repel = [&](const Vec3& q) {
                const double dx = a.position[0] - q[0];
                const double dy = a.position[1] - q[1];
                const double d = std::hypot(dx, dy);
                if (d < 1e-6 || d > 4.0) {
                    return;
                }
                push += kSocialA * std::exp(-(d - 2.0 * kPedestrianRadius) / kSocialB) *
                        (dx * sideX + dy * sideY) / d;
            };
            for (std::size_t k = 0; k < agents_.size(); ++k) {
                if (k != i && agents_[k].pedestrian) {
                    repel(agents_[k].position);
                }
            }
            for (const auto& [p, v] : egos) {
                repel(p);
            }
            // Sideways: the force's lateral part over a relaxation time, drifting back to the
            // lane's middle; the walker's left is the lane's left when walking along it.
            const double side = a.speed < 0.0 ? -1.0 : 1.0;
            const double limit = std::max(0.0, lane.width / 2.0 - kPedestrianRadius);
            const double lateral =
                std::clamp(side * push * kRelaxation - 0.2 * a.offset, -kMaxSideStep, kMaxSideStep);
            a.offset = std::clamp(a.offset + lateral * dt, -limit, limit);
            a.s += a.speed * dt;
            if (a.s > lane.length() || a.s < 0.0) {
                const bool forward = a.s > lane.length();
                const std::vector<std::size_t>& next =
                    forward ? lane.successors : lane.predecessors;
                if (!next.empty()) {
                    const std::size_t to = choose(a, i, next);
                    a.s = forward ? a.s - lane.length() : graph_.lanes[to].length() + a.s;
                    a.lane = to;
                } else {
                    a.s = std::clamp(a.s, 0.0, lane.length());
                    a.speed = -a.speed;  // turn back
                }
            }
            place(a);
            continue;
        }
        // Vehicles: choose the next lane early, so a leader there is seen.
        if (!a.next && !lane.successors.empty()) {
            a.next = choose(a, i, lane.successors);
        }
        const double v0 = lane.speedLimit > 0.0 ? lane.speedLimit : kDefaultSpeed;
        double gap = std::numeric_limits<double>::infinity();
        double leaderSpeed = 0.0;
        const auto consider = [&](double g, double v) {
            if (g < gap) {
                gap = g;
                leaderSpeed = v;
            }
        };
        const double half = kCarSize[0] / 2.0;
        for (std::size_t k = 0; k < agents_.size(); ++k) {
            const Agent& o = agents_[k];
            if (k == i || o.pedestrian) {
                continue;
            }
            if (o.lane == a.lane && o.s > a.s) {
                consider(o.s - a.s - 2.0 * half, o.speed);
            } else if (a.next && o.lane == *a.next) {
                consider(lane.length() - a.s + o.s - 2.0 * half, o.speed);
            }
        }
        for (const auto& [p, v] : egos) {
            for (const std::size_t l : {a.lane, a.next.value_or(a.lane)}) {
                const auto [s, lateral] = project(graph_.lanes[l], p);
                if (std::abs(lateral) > kEgoLateral) {
                    continue;
                }
                const double ahead = l == a.lane ? s - a.s : lane.length() - a.s + s;
                if (ahead > 0.0 && ahead < kLookAhead) {
                    consider(ahead - half - 2.5, v);  // the robot's half length, about a car's
                }
            }
        }
        // Right of way into the junction: wait at the line while a conflicting junction lane is
        // taken, or while a vehicle nearer its own line is about to take one.
        if (a.next && graph_.lanes[*a.next].junction && !lane.junction) {
            const double toLine = lane.length() - a.s;
            const double reach = a.speed * a.speed / (2.0 * idm_.b) + kYieldMargin;
            if (toLine < reach + kYieldMargin) {
                const std::vector<std::size_t>& against = conflicts_[*a.next];
                bool yield = false;
                for (std::size_t k = 0; k < agents_.size() && !yield; ++k) {
                    const Agent& o = agents_[k];
                    if (k == i || o.pedestrian) {
                        continue;
                    }
                    if (std::ranges::count(against, o.lane) > 0) {
                        yield = true;
                    } else if (o.next && std::ranges::count(against, *o.next) > 0 &&
                               !graph_.lanes[o.lane].junction) {
                        const double theirs = graph_.lanes[o.lane].length() - o.s;
                        yield =
                            theirs < toLine - 0.5 || (std::abs(theirs - toLine) <= 0.5 && k < i);
                        yield = yield && theirs < reach + kYieldMargin;
                    }
                }
                if (yield) {
                    consider(std::max(toLine - half, 0.0), 0.0);
                }
            }
        }
        // Lights: stop at the line on red or yellow when there is room to.
        if (graph_.signalGroup[a.lane] >= 0) {
            const Light light = lightAt(graph_.signalGroup[a.lane], 2, t);
            const double toLine = lane.length() - a.s - half;
            if (light != Light::Green && toLine > a.speed * a.speed / (2.0 * idm_.b) - 1.0) {
                consider(std::max(toLine, 0.0), 0.0);
            }
        }
        const double accel = std::max(idmAcceleration(idm_, a.speed, v0, gap, leaderSpeed), -8.0);
        a.speed = std::max(0.0, a.speed + accel * dt);
        a.s += a.speed * dt;
        while (a.s > graph_.lanes[a.lane].length()) {
            const Lane& done = graph_.lanes[a.lane];
            if (!a.next) {
                // A dead end (the network's edge): come back in at a free entry.
                if (!respawn(a, i)) {
                    a.s = done.length();  // none free: wait
                    a.speed = 0.0;
                }
                break;
            }
            a.s -= done.length();
            a.lane = *a.next;
            a.next.reset();
            const Lane& now = graph_.lanes[a.lane];
            if (!now.successors.empty()) {
                a.next = choose(a, i, now.successors);
            }
        }
        place(a);
    }
    write(bus, t);
    return {};
}

void Traffic::write(SignalBus& bus, double t) {
    std::vector<double> table;
    table.reserve(agents_.size() * 8);
    for (const Agent& a : agents_) {
        std::array<double, 7> pose{};
        if (zero_) {
            pose = {a.position[0], a.position[1], kHidden, 1.0, 0.0, 0.0, 0.0};
        } else {
            const auto q = yawQuat(a.yaw);
            pose = {a.position[0], a.position[1], a.position[2], q[0], q[1], q[2], q[3]};
        }
        bus.writeN(a.poseCommand, pose);
        table.insert(table.end(),
                     {static_cast<double>(a.pedestrian ? kAgentClassPedestrian : kAgentClassCar),
                      a.position[0], a.position[1], pose[2], a.yaw, std::abs(a.speed),
                      a.pedestrian ? 2.0 * kPedestrianRadius : kCarSize[0],
                      a.pedestrian ? 2.0 * kPedestrianRadius : kCarSize[1]});
    }
    table.resize(std::max<std::size_t>(agents_.size(), 1) * 8, 0.0);
    bus.writeN(table_, table);
    bus.write(count_, zero_ ? 0.0 : static_cast<double>(agents_.size()));
    std::vector<double> lights;
    for (const std::size_t lane : lightLanes_) {
        lights.push_back(graph_.lanes[lane].road);
        lights.push_back(zero_ ? 0.0
                               : static_cast<double>(lightAt(graph_.signalGroup[lane], 2, t)));
    }
    lights.resize(std::max<std::size_t>(lightLanes_.size(), 1) * 2, 0.0);
    bus.writeN(lights_, lights);
}

Status Traffic::saveState(StateWriter& writer) const {
    for (const Agent& a : agents_) {
        writer.put(a.lane);
        writer.put(a.s);
        writer.put(a.speed);
        writer.put(a.offset);
        writer.put(a.next.has_value());
        writer.put(a.next.value_or(0));
        writer.put(a.decisions);
    }
    return {};
}

Status Traffic::restoreState(StateReader& reader) {
    for (Agent& a : agents_) {
        bool hasNext = false;
        std::size_t next = 0;
        if (!reader.get(a.lane) || !reader.get(a.s) || !reader.get(a.speed) ||
            !reader.get(a.offset) || !reader.get(hasNext) || !reader.get(next) ||
            !reader.get(a.decisions)) {
            return Status::error(StatusCode::Internal, "traffic: truncated state");
        }
        a.next = hasNext ? std::optional(next) : std::nullopt;
        place(a);
    }
    return {};
}

}  // namespace lagrange::traffic
