// OpenSCENARIO: reader and runner (Scenario.hpp).
#include "lagrange/module/StateIO.hpp"
#include "lagrange/openscenario/Scenario.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <numbers>
#include <tinyxml2.h>

namespace lagrange::openscenario {

using namespace literals;

namespace {

using Element = tinyxml2::XMLElement;

/// Attribute reading with $parameter substitution.
class Reader {
public:
    explicit Reader(std::map<std::string, std::string> parameters)
        : parameters_(std::move(parameters)) {}

    [[nodiscard]] std::string text(const Element* e, const char* name,
                                   const std::string& fallback = "") const {
        const char* v = e != nullptr ? e->Attribute(name) : nullptr;
        if (v == nullptr) {
            return fallback;
        }
        std::string s = v;
        if (s.starts_with('$')) {
            const auto it = parameters_.find(s.substr(1));
            if (it == parameters_.end()) {
                throw std::runtime_error("parameter " + s + " is not declared");
            }
            return it->second;
        }
        return s;
    }
    [[nodiscard]] double number(const Element* e, const char* name, double fallback = 0.0) const {
        const std::string s = text(e, name);
        if (s.empty()) {
            return fallback;
        }
        std::size_t used = 0;
        const double v = std::stod(s, &used);
        if (used != s.size()) {
            throw std::runtime_error(std::string(name) + "=\"" + s + "\" is not a number");
        }
        return v;
    }

private:
    std::map<std::string, std::string> parameters_;
};

const Element* child(const Element* e, const char* name) {
    return e != nullptr ? e->FirstChildElement(name) : nullptr;
}

template <class F>
void each(const Element* e, const char* name, F f) {
    for (const Element* c = e != nullptr ? e->FirstChildElement(name) : nullptr; c != nullptr;
         c = c->NextSiblingElement(name)) {
        f(c);
    }
}

Position position(const Reader& r, const Element* e) {
    Position p;
    if (const Element* w = child(e, "WorldPosition")) {
        p.kind = Position::Kind::World;
        p.x = r.number(w, "x");
        p.y = r.number(w, "y");
        p.z = r.number(w, "z");
        p.h = r.number(w, "h");
        return p;
    }
    if (const Element* l = child(e, "LanePosition")) {
        p.kind = Position::Kind::Lane;
        p.road = static_cast<int>(r.number(l, "roadId"));
        p.lane = static_cast<int>(r.number(l, "laneId"));
        p.s = r.number(l, "s");
        p.offset = r.number(l, "offset");
        return p;
    }
    throw std::runtime_error("only WorldPosition and LanePosition are read");
}

Dynamics dynamics(const Reader& r, const Element* e) {
    Dynamics d;
    const std::string shape = r.text(e, "dynamicsShape", "step");
    d.shape = shape == "linear"       ? Dynamics::Shape::Linear
              : shape == "sinusoidal" ? Dynamics::Shape::Sinusoidal
              : shape == "cubic"      ? Dynamics::Shape::Cubic
                                      : Dynamics::Shape::Step;
    const std::string dimension = r.text(e, "dynamicsDimension", "time");
    d.dimension = dimension == "distance" ? Dynamics::Dimension::Distance
                  : dimension == "rate"   ? Dynamics::Dimension::Rate
                                          : Dynamics::Dimension::Time;
    d.value = r.number(e, "value");
    return d;
}

Condition::Rule rule(const std::string& text) {
    if (text == "lessThan") {
        return Condition::Rule::Less;
    }
    if (text == "equalTo") {
        return Condition::Rule::Equal;
    }
    if (text == "greaterOrEqual") {
        return Condition::Rule::GreaterOrEqual;
    }
    if (text == "lessOrEqual") {
        return Condition::Rule::LessOrEqual;
    }
    return Condition::Rule::Greater;
}

Action privateAction(const Reader& r, const Element* e, const std::string& entity) {
    Action a;
    a.entity = entity;
    if (const Element* t = child(e, "TeleportAction")) {
        a.kind = Action::Kind::Teleport;
        a.position = position(r, child(t, "Position"));
        return a;
    }
    if (const Element* longitudinal = child(e, "LongitudinalAction")) {
        const Element* speed = child(longitudinal, "SpeedAction");
        if (speed == nullptr) {
            throw std::runtime_error("of LongitudinalAction, only SpeedAction is read");
        }
        a.kind = Action::Kind::Speed;
        a.dynamics = dynamics(r, child(speed, "SpeedActionDynamics"));
        const Element* target = child(speed, "SpeedActionTarget");
        if (const Element* abs = child(target, "AbsoluteTargetSpeed")) {
            a.speed = r.number(abs, "value");
        } else if (const Element* rel = child(target, "RelativeTargetSpeed")) {
            a.relativeTo = r.text(rel, "entityRef");
            a.speed = r.number(rel, "value");
            if (r.text(rel, "speedTargetValueType", "delta") != "delta") {
                throw std::runtime_error("RelativeTargetSpeed is read as a delta only");
            }
        } else {
            throw std::runtime_error("SpeedAction without a target");
        }
        return a;
    }
    if (const Element* lateral = child(e, "LateralAction")) {
        const Element* change = child(lateral, "LaneChangeAction");
        if (change == nullptr) {
            throw std::runtime_error("of LateralAction, only LaneChangeAction is read");
        }
        a.kind = Action::Kind::LaneChange;
        a.dynamics = dynamics(r, child(change, "LaneChangeActionDynamics"));
        const Element* target = child(change, "LaneChangeTarget");
        if (const Element* rel = child(target, "RelativeTargetLane")) {
            a.laneRelativeTo = r.text(rel, "entityRef");
            a.lanes = static_cast<int>(r.number(rel, "value"));
        } else if (const Element* abs = child(target, "AbsoluteTargetLane")) {
            a.absoluteLane = static_cast<int>(r.number(abs, "value"));
        } else {
            throw std::runtime_error("LaneChangeAction without a target");
        }
        return a;
    }
    throw std::runtime_error("only Teleport, Speed and LaneChange actions are read");
}

Trigger trigger(const Reader& r, const Element* e) {
    Trigger t;
    each(e, "ConditionGroup", [&](const Element* group) {
        std::vector<Condition> conditions;
        each(group, "Condition", [&](const Element* c) {
            Condition out;
            out.delay = r.number(c, "delay");
            const std::string edge = r.text(c, "conditionEdge", "none");
            out.edge = edge == "rising"            ? Condition::Edge::Rising
                       : edge == "falling"         ? Condition::Edge::Falling
                       : edge == "risingOrFalling" ? Condition::Edge::RisingOrFalling
                                                   : Condition::Edge::None;
            if (const Element* byValue = child(c, "ByValueCondition")) {
                const Element* time = child(byValue, "SimulationTimeCondition");
                if (time == nullptr) {
                    throw std::runtime_error(
                        "of ByValueCondition, only SimulationTimeCondition "
                        "is read");
                }
                out.kind = Condition::Kind::SimulationTime;
                out.value = r.number(time, "value");
                out.rule = rule(r.text(time, "rule"));
            } else if (const Element* byEntity = child(c, "ByEntityCondition")) {
                each(child(byEntity, "TriggeringEntities"), "EntityRef", [&](const Element* ref) {
                    out.triggering.push_back(r.text(ref, "entityRef"));
                });
                const Element* ec = child(byEntity, "EntityCondition");
                if (const Element* reach = child(ec, "ReachPositionCondition")) {
                    out.kind = Condition::Kind::ReachPosition;
                    out.tolerance = r.number(reach, "tolerance", 1.0);
                    out.position = position(r, child(reach, "Position"));
                } else if (const Element* rd = child(ec, "RelativeDistanceCondition")) {
                    out.kind = Condition::Kind::RelativeDistance;
                    out.entityRef = r.text(rd, "entityRef");
                    out.value = r.number(rd, "value");
                    out.rule = rule(r.text(rd, "rule"));
                    out.freespace = r.text(rd, "freespace", "false") == "true";
                    const std::string type =
                        r.text(rd, "relativeDistanceType", "cartesianDistance");
                    out.distance = type == "longitudinal" ? Condition::Distance::Longitudinal
                                   : type == "lateral"    ? Condition::Distance::Lateral
                                                          : Condition::Distance::Euclidean;
                } else if (const Element* speed = child(ec, "SpeedCondition")) {
                    out.kind = Condition::Kind::Speed;
                    out.value = r.number(speed, "value");
                    out.rule = rule(r.text(speed, "rule"));
                } else if (const Element* travelled = child(ec, "TraveledDistanceCondition")) {
                    out.kind = Condition::Kind::TraveledDistance;
                    out.value = r.number(travelled, "value");
                    out.rule = Condition::Rule::GreaterOrEqual;
                } else {
                    throw std::runtime_error(
                        "of the entity conditions, only ReachPosition, "
                        "RelativeDistance, Speed and TraveledDistance are "
                        "read");
                }
            } else {
                throw std::runtime_error("a Condition without a ByValue or ByEntity condition");
            }
            conditions.push_back(std::move(out));
        });
        t.groups.push_back(std::move(conditions));
    });
    return t;
}

bool compare(Condition::Rule rule, double a, double b) {
    switch (rule) {
        case Condition::Rule::Greater:
            return a > b;
        case Condition::Rule::Less:
            return a < b;
        case Condition::Rule::Equal:
            return std::abs(a - b) < 1e-6;
        case Condition::Rule::GreaterOrEqual:
            return a >= b;
        case Condition::Rule::LessOrEqual:
            return a <= b;
    }
    return false;
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

}  // namespace

double shapeAt(Dynamics::Shape shape, double x) {
    x = std::clamp(x, 0.0, 1.0);
    switch (shape) {
        case Dynamics::Shape::Step:
            return 1.0;
        case Dynamics::Shape::Linear:
            return x;
        case Dynamics::Shape::Sinusoidal:
            return 0.5 * (1.0 - std::cos(std::numbers::pi * x));
        case Dynamics::Shape::Cubic:
            return x * x * (3.0 - 2.0 * x);
    }
    return x;
}

const Entity* Scenario::entity(const std::string& name) const {
    for (const Entity& e : entities) {
        if (e.name == name) {
            return &e;
        }
    }
    return nullptr;
}

Expected<Scenario, std::string> parse(const std::string& xml, const std::string& directory) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{std::string("not XML: ") + doc.ErrorStr()};
    }
    const Element* top = doc.FirstChildElement("OpenSCENARIO");
    if (top == nullptr) {
        return Unexpected{std::string("no <OpenSCENARIO> element")};
    }
    try {
        std::map<std::string, std::string> parameters;
        each(child(top, "ParameterDeclarations"), "ParameterDeclaration", [&](const Element* p) {
            const char* name = p->Attribute("name");
            const char* value = p->Attribute("value");
            if (name != nullptr && value != nullptr) {
                parameters[name] = value;
            }
        });
        const Reader r(parameters);
        Scenario s;
        if (const Element* logic = child(child(top, "RoadNetwork"), "LogicFile")) {
            const std::filesystem::path file = r.text(logic, "filepath");
            s.roadNetwork = file.is_absolute() ? file.string()
                                               : (std::filesystem::path(directory) / file).string();
        }
        each(child(top, "Entities"), "ScenarioObject", [&](const Element* o) {
            Entity e;
            e.name = r.text(o, "name");
            const Element* vehicle = child(o, "Vehicle");
            const Element* pedestrian = child(o, "Pedestrian");
            const Element* object = vehicle != nullptr ? vehicle : pedestrian;
            if (object == nullptr) {
                throw std::runtime_error(e.name +
                                         ": only Vehicle and Pedestrian entities are "
                                         "read (no catalogs yet)");
            }
            e.pedestrian = pedestrian != nullptr;
            if (const Element* d = child(child(object, "BoundingBox"), "Dimensions")) {
                e.size = {r.number(d, "length", e.size[0]), r.number(d, "width", e.size[1]),
                          r.number(d, "height", e.size[2])};
            } else if (e.pedestrian) {
                e.size = {0.6, 0.6, 1.75};
            }
            s.entities.push_back(std::move(e));
        });
        const Element* storyboard = child(top, "Storyboard");
        if (storyboard == nullptr) {
            throw std::runtime_error("no Storyboard");
        }
        each(child(child(storyboard, "Init"), "Actions"), "Private", [&](const Element* p) {
            const std::string entity = r.text(p, "entityRef");
            each(p, "PrivateAction",
                 [&](const Element* a) { s.init.push_back(privateAction(r, a, entity)); });
        });
        each(storyboard, "Story", [&](const Element* story) {
            each(story, "Act", [&](const Element* actXml) {
                Act act;
                act.name = r.text(actXml, "name");
                act.start = trigger(r, child(actXml, "StartTrigger"));
                each(actXml, "ManeuverGroup", [&](const Element* group) {
                    std::vector<std::string> actors;
                    each(child(group, "Actors"), "EntityRef",
                         [&](const Element* ref) { actors.push_back(r.text(ref, "entityRef")); });
                    act.actors.insert(act.actors.end(), actors.begin(), actors.end());
                    each(group, "Maneuver", [&](const Element* maneuver) {
                        each(maneuver, "Event", [&](const Element* eventXml) {
                            Event event;
                            event.name = r.text(eventXml, "name");
                            event.start = trigger(r, child(eventXml, "StartTrigger"));
                            each(eventXml, "Action", [&](const Element* action) {
                                const Element* p = child(action, "PrivateAction");
                                if (p == nullptr) {
                                    throw std::runtime_error(event.name +
                                                             ": only private actions are read");
                                }
                                for (const std::string& actor : actors) {
                                    event.actions.push_back(privateAction(r, p, actor));
                                }
                            });
                            act.events.push_back(std::move(event));
                        });
                    });
                });
                s.acts.push_back(std::move(act));
            });
        });
        s.stop = trigger(r, child(storyboard, "StopTrigger"));
        return s;
    } catch (const std::exception& e) {
        return Unexpected{std::string(e.what())};
    }
}

Expected<Scenario, std::string> read(const std::string& path) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{path + ": " + doc.ErrorStr()};
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    auto s = parse(printer.CStr(), std::filesystem::path(path).parent_path().string());
    if (!s) {
        return Unexpected{path + ": " + s.error()};
    }
    s->path = path;
    return s;
}

std::optional<std::array<double, 4>> poseOf(const Position& p, const RoadLanes& lanes) {
    if (p.kind == Position::Kind::World) {
        return std::array<double, 4>{p.x, p.y, p.z, p.h};
    }
    const auto found = lanes.find(p.road, p.lane, p.s);
    if (!found) {
        return std::nullopt;
    }
    const auto [point, heading] = lanes.lanes[found->first].at(found->second);
    // offset: to the left of the lane's centre (OpenDRIVE t), so right of travel for left lanes.
    const double t = p.lane < 0 ? p.offset : -p.offset;
    return std::array<double, 4>{point[0] - t * std::sin(heading), point[1] + t * std::cos(heading),
                                 point[2], heading};
}

Expected<void, std::string> addEntities(Model& model, const Scenario& scenario,
                                        const std::string& ego) {
    const RoadLanes lanes = roadLanesOf(WorldView(model.world));
    World& w = model.world;
    const auto robot = static_cast<std::uint32_t>(w.robots.size());
    w.robots.emplace_back("scenario");
    ScenePrim top;
    top.name = "scenario";
    top.path = "/scenario";
    top.typeName = "Xform";
    model.scene.prims.push_back(top);
    const auto topPrim = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
    model.scene.materials.push_back({"scenario_vehicle", {0.95F, 0.6F, 0.1F, 1.0F}});
    const auto material = static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    for (const Entity& e : scenario.entities) {
        if (e.name == ego) {
            continue;
        }
        const Action* teleport = nullptr;
        for (const Action& a : scenario.init) {
            if (a.entity == e.name && a.kind == Action::Kind::Teleport) {
                teleport = &a;
            }
        }
        if (teleport == nullptr) {
            return Unexpected{e.name + " has no TeleportAction in Init"};
        }
        const auto pose = poseOf(teleport->position, lanes);
        if (!pose) {
            return Unexpected{e.name + ": its start is on no road lane of the network"};
        }
        Pose at;
        at.position = {(*pose)[0], (*pose)[1], (*pose)[2] + e.size[2] / 2.0};
        at.orientation = {std::cos((*pose)[3] / 2.0), 0.0, 0.0, std::sin((*pose)[3] / 2.0)};
        const std::string name = "scenario_" + e.name;
        const BodyId body =
            w.addBody(name, kWorldBody, at, e.pedestrian ? 75.0 : 1500.0, {}, {1, 1, 1, 0, 0, 0});
        w.bodies.robot.back() = robot;
        w.bodies.kinematic.back() = true;
        w.addCollider(body, ShapeType::Box, {e.size[0] / 2.0, e.size[1] / 2.0, e.size[2] / 2.0},
                      Pose{}, 0.8);
        model.bodyRestPose.push_back(at);
        model.scene.meshes.push_back(boxMesh(e.size[0], e.size[1], e.size[2]));
        ScenePrim prim;
        prim.name = name;
        prim.path = "/scenario/" + name;
        prim.parent = topPrim;
        prim.typeName = "Mesh";
        prim.mesh = static_cast<std::uint32_t>(model.scene.meshes.size() - 1);
        prim.material = material;
        prim.body = body.index;
        model.scene.prims.push_back(prim);
        model.bodyPrim.push_back(static_cast<std::uint32_t>(model.scene.prims.size() - 1));
    }
    return {};
}

ModuleInfo ScenarioRunner::info() const {
    return {"traffic", "scenario", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration ScenarioRunner::preferredStep() const {
    return 10_ms;
}

Status ScenarioRunner::declare(const WorldView& world, const ModuleConfig& config,
                               SignalDeclarer& d) {
    lanes_ = roadLanesOf(world);
    start_.clear();
    for (const Entity& e : scenario_.entities) {
        if (e.name == egoName_) {
            continue;
        }
        Mover m;
        m.name = e.name;
        // Its start: the Init teleport, on its lane.
        for (const Action& a : scenario_.init) {
            if (a.entity != e.name) {
                continue;
            }
            if (a.kind == Action::Kind::Teleport) {
                if (a.position.kind == Position::Kind::Lane) {
                    const auto found = lanes_.find(a.position.road, a.position.lane, a.position.s);
                    if (!found) {
                        return Status::error(StatusCode::InvalidConfig,
                                             e.name + ": its start is on no road lane");
                    }
                    m.lane = found->first;
                    m.along = found->second;
                } else {
                    const auto pose = poseOf(a.position, lanes_);
                    double best = INFINITY;
                    for (std::size_t i = 0; i < lanes_.lanes.size(); ++i) {
                        const auto [s, lateral] =
                            lanes_.lanes[i].project({(*pose)[0], (*pose)[1], 0});
                        if (!lanes_.lanes[i].sidewalk && std::abs(lateral) < best) {
                            best = std::abs(lateral);
                            m.lane = i;
                            m.along = s;
                        }
                    }
                }
            } else if (a.kind == Action::Kind::Speed && !a.relativeTo) {
                m.speed = a.speed;  // an Init speed is reached at once
            }
        }
        m.poseCommand = d.output(spec("body/scenario_" + e.name + "/pose_command", SignalType::Pose,
                                      units::m, "world_local"));
        start_.push_back(m);
    }
    if (const auto ego = config.get<std::string>("ego_body")) {
        SignalSpec pose = spec("body/" + *ego + "/pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        SignalSpec twist =
            spec("body/" + *ego + "/twist", SignalType::Wrench, units::m_per_s, "world_local");
        twist.feedback = true;
        egoPose_ = d.input(pose);
        egoTwist_ = d.input(twist);
    }
    stateOut_ = d.output(spec("scenario/state", SignalType::Int, units::one));
    timeOut_ = d.output(spec("scenario/time", SignalType::Scalar, units::s));
    SignalSpec table = spec("scenario/entities", SignalType::ScalarArray, units::one);
    table.arrayLength = static_cast<std::uint32_t>(std::max<std::size_t>(start_.size(), 1) * 8);
    table_ = d.output(table);
    return {};
}

Status ScenarioRunner::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    movers_ = start_;
    for (Mover& m : movers_) {
        place(m);
        bus.writeN(m.poseCommand, std::array<double, 7>{m.position[0], m.position[1], m.position[2],
                                                        std::cos(m.heading / 2.0), 0.0, 0.0,
                                                        std::sin(m.heading / 2.0)});
    }
    state_ = State{};
    state_.actStarted.assign(scenario_.acts.size(), false);
    state_.eventStarted.clear();
    for (const Act& act : scenario_.acts) {
        state_.eventStarted.emplace_back(act.events.size(), false);
    }
    egoLast_.reset();
    egoTravelled_ = 0.0;
    log_.clear();
    bus.write(stateOut_, 0.0);
    bus.write(timeOut_, 0.0);
    bus.writeN(table_, std::vector<double>(std::max<std::size_t>(movers_.size(), 1) * 8, 0.0));
    return {};
}

void ScenarioRunner::place(Mover& m) {
    const RoadLane& lane = lanes_.lanes[m.lane];
    auto [p, heading] = lane.at(m.along);
    if (m.changing) {
        // Between the lanes: blend towards the target lane's point beside this one.
        const RoadLane& target = lanes_.lanes[m.targetLane];
        const auto [s, lateral] = target.project(p);
        const auto [q, h2] = target.at(s);
        const double f = shapeAt(m.changeDynamics.shape, m.changeProgress);
        p = {p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2])};
        heading = heading + f * std::remainder(h2 - heading, 2.0 * std::numbers::pi);
    }
    const Entity* e = scenario_.entity(m.name);
    m.position = {p[0], p[1], p[2] + (e != nullptr ? e->size[2] / 2.0 : 0.75)};
    m.heading = heading;
}

void ScenarioRunner::move(Mover& m, double dt) {
    if (m.speeding) {
        const Dynamics& d = m.speedDynamics;
        double span = d.value;
        if (d.dimension == Dynamics::Dimension::Rate) {
            span = std::abs(m.toSpeed - m.fromSpeed) / std::max(d.value, 1e-9);  // → a time
        }
        if (d.dimension == Dynamics::Dimension::Distance) {
            m.progress += m.speed * dt / std::max(span, 1e-9);
        } else {
            m.progress += dt / std::max(span, 1e-9);
        }
        m.speed = m.fromSpeed + shapeAt(d.shape, m.progress) * (m.toSpeed - m.fromSpeed);
        if (m.progress >= 1.0 || d.shape == Dynamics::Shape::Step) {
            m.speed = m.toSpeed;
            m.speeding = false;
        }
    }
    const double ds = m.speed * dt;
    m.along += ds;
    m.travelled += ds;
    if (m.changing) {
        const Dynamics& d = m.changeDynamics;
        m.changeProgress += d.dimension == Dynamics::Dimension::Distance
                                ? ds / std::max(d.value, 1e-9)
                                : dt / std::max(d.value, 1e-9);
        if (m.changeProgress >= 1.0 || d.shape == Dynamics::Shape::Step) {
            const auto [p, h] = lanes_.lanes[m.lane].at(m.along);
            m.along = lanes_.lanes[m.targetLane].project(p).first;
            m.lane = m.targetLane;
            m.changing = false;
        }
    }
    // On to the next lane: the one that goes straightest.
    while (m.along > lanes_.lanes[m.lane].length() && !lanes_.lanes[m.lane].successors.empty()) {
        const RoadLane& done = lanes_.lanes[m.lane];
        const double heading = done.at(done.length()).second;
        std::size_t best = done.successors.front();
        double turn = INFINITY;
        for (const std::size_t next : done.successors) {
            const RoadLane& n = lanes_.lanes[next];
            const double h = n.at(n.length()).second;
            const double t = std::abs(std::remainder(h - heading, 2.0 * std::numbers::pi));
            if (t < turn) {
                turn = t;
                best = next;
            }
        }
        m.along -= done.length();
        m.lane = best;
        if (m.changing) {
            m.changing = false;  // a lane change does not carry across roads
        }
    }
    place(m);
}

std::optional<std::array<double, 3>> ScenarioRunner::stateOf(const std::string& entity,
                                                             double& speed, double& heading,
                                                             double& travelled) const {
    if (entity == egoName_) {
        speed = egoSpeed_;
        heading = egoHeading_;
        travelled = egoTravelled_;
        return egoPosition_;
    }
    for (const Mover& m : movers_) {
        if (m.name == entity) {
            speed = m.speed;
            heading = m.heading;
            travelled = m.travelled;
            return m.position;
        }
    }
    return std::nullopt;
}

bool ScenarioRunner::evaluate(const Condition& c, double t) const {
    if (c.kind == Condition::Kind::SimulationTime) {
        return compare(c.rule, t, c.value);
    }
    for (const std::string& who : c.triggering) {
        double speed = 0.0, heading = 0.0, travelled = 0.0;
        const auto p = stateOf(who, speed, heading, travelled);
        if (!p) {
            continue;
        }
        bool ok = false;
        switch (c.kind) {
            case Condition::Kind::Speed:
                ok = compare(c.rule, speed, c.value);
                break;
            case Condition::Kind::TraveledDistance:
                ok = travelled >= c.value;
                break;
            case Condition::Kind::ReachPosition: {
                const auto target = poseOf(c.position, lanes_);
                ok = target &&
                     std::hypot((*p)[0] - (*target)[0], (*p)[1] - (*target)[1]) <= c.tolerance;
                break;
            }
            case Condition::Kind::RelativeDistance: {
                double s2 = 0.0, h2 = 0.0, d2 = 0.0;
                const auto q = stateOf(c.entityRef, s2, h2, d2);
                if (!q) {
                    break;
                }
                const double dx = (*q)[0] - (*p)[0];
                const double dy = (*q)[1] - (*p)[1];
                double d = std::hypot(dx, dy);
                if (c.distance == Condition::Distance::Longitudinal) {
                    d = std::abs(dx * std::cos(heading) + dy * std::sin(heading));
                } else if (c.distance == Condition::Distance::Lateral) {
                    d = std::abs(-dx * std::sin(heading) + dy * std::cos(heading));
                }
                if (c.freespace) {
                    const Entity* a = scenario_.entity(who);
                    const Entity* b = scenario_.entity(c.entityRef);
                    const double half =
                        c.distance == Condition::Distance::Lateral
                            ? ((a ? a->size[1] : 1.8) + (b ? b->size[1] : 1.8)) / 2.0
                            : ((a ? a->size[0] : 4.5) + (b ? b->size[0] : 4.5)) / 2.0;
                    d = std::max(0.0, d - half);
                }
                ok = compare(c.rule, d, c.value);
                break;
            }
            case Condition::Kind::SimulationTime:
                break;
        }
        if (ok) {
            return true;
        }
    }
    return false;
}

bool ScenarioRunner::holds(const Trigger& trigger, const std::string& key, double t) {
    if (trigger.groups.empty()) {
        return true;
    }
    bool any = false;
    for (std::size_t g = 0; g < trigger.groups.size(); ++g) {
        bool all = true;
        for (std::size_t k = 0; k < trigger.groups[g].size(); ++k) {
            const Condition& c = trigger.groups[g][k];
            const std::string id = key + "/" + std::to_string(g) + "/" + std::to_string(k);
            const bool now = evaluate(c, t);
            const bool before = state_.lastValue.contains(id) ? state_.lastValue[id] : now;
            state_.lastValue[id] = now;
            bool value = now;
            switch (c.edge) {
                case Condition::Edge::None:
                    break;
                case Condition::Edge::Rising:
                    value = now && !before;
                    break;
                case Condition::Edge::Falling:
                    value = !now && before;
                    break;
                case Condition::Edge::RisingOrFalling:
                    value = now != before;
                    break;
            }
            // A delay: the condition counts once it has held that long.
            if (c.delay > 0.0) {
                if (value && !state_.trueSince.contains(id)) {
                    state_.trueSince[id] = t;
                }
                value = state_.trueSince.contains(id) && t - state_.trueSince[id] >= c.delay;
            }
            all = all && value;
        }
        any = any || all;
    }
    return any;
}

void ScenarioRunner::apply(const Action& action, double t) {
    for (Mover& m : movers_) {
        if (m.name != action.entity) {
            continue;
        }
        if (action.kind == Action::Kind::Speed) {
            double target = action.speed;
            if (action.relativeTo) {
                double s = 0.0, h = 0.0, d = 0.0;
                if (stateOf(*action.relativeTo, s, h, d)) {
                    target += s;
                }
            }
            m.fromSpeed = m.speed;
            m.toSpeed = target;
            m.progress = 0.0;
            m.speedDynamics = action.dynamics;
            m.speeding = true;
            if (action.dynamics.shape == Dynamics::Shape::Step) {
                m.speed = target;
                m.speeding = false;
            }
        } else if (action.kind == Action::Kind::LaneChange) {
            std::optional<std::size_t> target;
            if (action.absoluteLane) {
                const RoadLane& here = lanes_.lanes[m.lane];
                const double span = std::max(here.sEnd - here.sStart, 1e-9);
                const double s = here.lane < 0 ? here.sStart + m.along / here.length() * span
                                               : here.sEnd - m.along / here.length() * span;
                if (const auto f = lanes_.find(here.road, *action.absoluteLane, s)) {
                    target = f->first;
                }
            } else if (action.laneRelativeTo) {
                // Relative to that entity's lane: the lane `lanes` to its left.
                double sp = 0.0, h = 0.0, d = 0.0;
                const auto p = stateOf(*action.laneRelativeTo, sp, h, d);
                std::size_t reference = m.lane;
                double along = m.along;
                if (p) {
                    double best = INFINITY;
                    for (std::size_t i = 0; i < lanes_.lanes.size(); ++i) {
                        const auto [s, lateral] = lanes_.lanes[i].project(*p);
                        const double turn = std::abs(std::remainder(
                            lanes_.lanes[i].at(s).second - h, 2.0 * std::numbers::pi));
                        if (!lanes_.lanes[i].sidewalk && turn < 1.0 && std::abs(lateral) < best) {
                            best = std::abs(lateral);
                            reference = i;
                            along = s;
                        }
                    }
                }
                std::optional<std::pair<std::size_t, double>> lane{{reference, along}};
                for (int k = 0; k < std::abs(action.lanes) && lane; ++k) {
                    lane = lanes_.neighbour(lane->first, lane->second, action.lanes > 0 ? 1 : -1);
                }
                if (lane) {
                    target = lane->first;
                }
            }
            if (!target) {
                log_.push_back("t=" + std::to_string(t) + " " + m.name +
                               ": the lane change's target lane does not exist; skipped");
                continue;
            }
            if (*target == m.lane) {
                continue;
            }
            m.targetLane = *target;
            m.changeProgress = 0.0;
            m.changeDynamics = action.dynamics;
            m.changing = true;
        } else if (action.kind == Action::Kind::Teleport) {
            if (const auto pose = poseOf(action.position, lanes_)) {
                double best = INFINITY;
                for (std::size_t i = 0; i < lanes_.lanes.size(); ++i) {
                    const auto [s, lateral] = lanes_.lanes[i].project({(*pose)[0], (*pose)[1], 0});
                    if (!lanes_.lanes[i].sidewalk && std::abs(lateral) < best) {
                        best = std::abs(lateral);
                        m.lane = i;
                        m.along = s;
                    }
                }
                m.changing = false;
            }
        }
    }
}

Status ScenarioRunner::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const double t = context.t.seconds() + dt;
    if (egoPose_.valid()) {
        const std::span<const double> p = bus.readN(egoPose_);
        const std::span<const double> v = bus.readN(egoTwist_);
        egoPosition_ = {p[0], p[1], p[2]};
        egoSpeed_ = std::hypot(v[0], v[1]);
        egoHeading_ =
            std::atan2(2.0 * (p[3] * p[6] + p[4] * p[5]), 1.0 - 2.0 * (p[5] * p[5] + p[6] * p[6]));
        if (egoLast_) {
            egoTravelled_ += std::hypot(p[0] - (*egoLast_)[0], p[1] - (*egoLast_)[1]);
        }
        egoLast_ = egoPosition_;
    }
    if (!state_.ended) {
        for (std::size_t a = 0; a < scenario_.acts.size(); ++a) {
            const Act& act = scenario_.acts[a];
            if (!state_.actStarted[a] && holds(act.start, "act" + std::to_string(a), t)) {
                state_.actStarted[a] = true;
                log_.push_back("t=" + std::to_string(t) + " act " + act.name + " started");
            }
            if (!state_.actStarted[a]) {
                continue;
            }
            for (std::size_t e = 0; e < act.events.size(); ++e) {
                const Event& event = act.events[e];
                if (state_.eventStarted[a][e] ||
                    !holds(event.start, "act" + std::to_string(a) + "/event" + std::to_string(e),
                           t)) {
                    continue;
                }
                state_.eventStarted[a][e] = true;
                log_.push_back("t=" + std::to_string(t) + " event " + event.name + " started");
                for (const Action& action : event.actions) {
                    apply(action, t);
                }
            }
        }
        if (!scenario_.stop.groups.empty() && holds(scenario_.stop, "stop", t)) {
            state_.ended = true;
            state_.endTime = t;
            log_.push_back("t=" + std::to_string(t) + " the scenario ended (stop trigger)");
        }
    }
    std::vector<double> table;
    for (Mover& m : movers_) {
        move(m, dt);
        const std::array<double, 7> pose{m.position[0],
                                         m.position[1],
                                         m.position[2],
                                         std::cos(m.heading / 2.0),
                                         0.0,
                                         0.0,
                                         std::sin(m.heading / 2.0)};
        bus.writeN(m.poseCommand, pose);
        const RoadLane& lane = lanes_.lanes[m.lane];
        table.insert(table.end(),
                     {m.position[0], m.position[1], m.position[2], m.heading, m.speed, m.travelled,
                      static_cast<double>(lane.road), static_cast<double>(lane.lane)});
    }
    table.resize(std::max<std::size_t>(movers_.size(), 1) * 8, 0.0);
    bus.writeN(table_, table);
    bus.write(stateOut_, state_.ended ? 1.0 : 0.0);
    bus.write(timeOut_, state_.ended ? state_.endTime : t);
    return {};
}

Status ScenarioRunner::saveState(StateWriter& writer) const {
    // Checkpoints restore the movers; storyboard progress restarts from the restored time's
    // triggers (they are mostly time and distance conditions).
    for (const Mover& m : movers_) {
        writer.put(m.lane);
        writer.put(m.along);
        writer.put(m.speed);
        writer.put(m.travelled);
    }
    return {};
}

Status ScenarioRunner::restoreState(StateReader& reader) {
    for (Mover& m : movers_) {
        if (!reader.get(m.lane) || !reader.get(m.along) || !reader.get(m.speed) ||
            !reader.get(m.travelled)) {
            return Status::error(StatusCode::Internal, "scenario: truncated state");
        }
        place(m);
    }
    return {};
}

}  // namespace lagrange::openscenario
