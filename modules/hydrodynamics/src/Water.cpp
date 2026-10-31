#include "lagrange/hydrodynamics/Water.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace lagrange::hydrodynamics {

namespace {

constexpr int kCells = 6;           // a side, per collider
constexpr double kCellReach = 2.0;  // a cell's corner is within this many of its sides, generously

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

/// A shape's bounds in its frame, and its volume.
struct ShapeBox {
    std::array<double, 3> lo{}, hi{};
    double volume{0.0};
};

ShapeBox boxOf(ShapeType type, const std::array<double, 3>& s, const ConvexHull* hull) {
    constexpr double kPi = std::numbers::pi;
    const auto centred = [](double x, double y, double z, double volume) {
        return ShapeBox{{-x, -y, -z}, {x, y, z}, volume};
    };
    switch (type) {
        case ShapeType::Sphere:
            return centred(s[0], s[0], s[0], 4.0 / 3.0 * kPi * s[0] * s[0] * s[0]);
        case ShapeType::Capsule:
            return centred(s[0], s[0], s[1] + s[0],
                           kPi * s[0] * s[0] * (2.0 * s[1] + 4.0 / 3.0 * s[0]));
        case ShapeType::Cylinder:
            return centred(s[0], s[0], s[1], kPi * s[0] * s[0] * 2.0 * s[1]);
        case ShapeType::Box:
            return centred(s[0], s[1], s[2], 8.0 * s[0] * s[1] * s[2]);
        case ShapeType::ConvexHull: {
            if (hull == nullptr || hull->points.empty()) {
                return {};
            }
            ShapeBox box{hull->points.front(), hull->points.front(), 0.0};
            for (const auto& p : hull->points) {
                for (std::size_t k = 0; k < 3; ++k) {
                    box.lo.at(k) = std::min(box.lo.at(k), p.at(k));
                    box.hi.at(k) = std::max(box.hi.at(k), p.at(k));
                }
            }
            for (const auto& t : hull->triangles) {  // the divergence theorem, outward triangles
                const auto& a = hull->points.at(t[0]);
                const auto& b = hull->points.at(t[1]);
                const auto& c = hull->points.at(t[2]);
                box.volume +=
                    (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                     a[2] * (b[0] * c[1] - b[1] * c[0])) /
                    6.0;
            }
            box.volume = std::abs(box.volume);
            return box;
        }
        case ShapeType::Plane:
        case ShapeType::HeightField:
            break;
    }
    return {};
}

bool inside(ShapeType type, const std::array<double, 3>& s, const std::array<double, 3>& q,
            const ConvexHull* hull) {
    switch (type) {
        case ShapeType::ConvexHull:
            for (const auto& face : hull->faces) {
                if (face[0] * q[0] + face[1] * q[1] + face[2] * q[2] > face[3]) {
                    return false;
                }
            }
            return true;
        case ShapeType::Sphere:
            return std::hypot(q[0], q[1], q[2]) <= s[0];
        case ShapeType::Capsule: {
            const double z = std::clamp(q[2], -s[1], s[1]);
            return std::hypot(q[0], q[1], q[2] - z) <= s[0];
        }
        case ShapeType::Cylinder:
            return std::hypot(q[0], q[1]) <= s[0];
        default:
            return true;
    }
}

/// A point of a body in the world: the rotation of `compose`, without building a quaternion.
std::array<double, 3> placed(const Pose& pose, const std::array<double, 3>& point) {
    const auto& q = pose.orientation;
    const std::array<double, 3> u{q[1], q[2], q[3]};
    const std::array<double, 3> t{2.0 * (u[1] * point[2] - u[2] * point[1]),
                                  2.0 * (u[2] * point[0] - u[0] * point[2]),
                                  2.0 * (u[0] * point[1] - u[1] * point[0])};
    return {pose.position[0] + point[0] + q[0] * t[0] + u[1] * t[2] - u[2] * t[1],
            pose.position[1] + point[1] + q[0] * t[1] + u[2] * t[0] - u[0] * t[2],
            pose.position[2] + point[2] + q[0] * t[2] + u[0] * t[1] - u[1] * t[0]};
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

}  // namespace

std::vector<WaterBody> waterOf(const WorldView& world) {
    std::vector<WaterBody> out;
    const World& raw = world.raw();
    for (const SchemaInstance* s : world.instancesOf("WaterAPI_1")) {
        WaterBody w;
        w.name = raw.prims.instanceNames.at(s->prim.index);
        w.surface = raw.prims.poseInBody.at(s->prim.index).position;
        w.sizeX = s->real("water:sizeX").value_or(w.sizeX);
        w.sizeY = s->real("water:sizeY").value_or(w.sizeY);
        w.depth = s->real("water:depth").value_or(w.depth);
        w.density = s->real("water:density").value_or(w.density);
        w.drag = s->real("water:dragCoefficient").value_or(w.drag);
        w.reflectivity = s->real("water:reflectivity").value_or(w.reflectivity);
        out.push_back(std::move(w));
    }
    return out;
}

std::vector<WaterBody> takeWater(Model& model) {
    std::vector<WaterBody> out = waterOf(WorldView(model.world));
    std::erase_if(model.world.schemaInstances,
                  [](const SchemaInstance& s) { return s.schema == "WaterAPI_1"; });
    return out;
}

void addWater(Model& model, const WaterBody& water) {
    constexpr float kSheet = 0.01F;  // m, the drawn surface's thickness
    model.scene.meshes.push_back(boxMesh(water.sizeX, water.sizeY, kSheet));
    model.scene.materials.push_back({water.name, {0.08F, 0.28F, 0.42F, 0.8F}});
    ScenePrim prim;
    prim.name = water.name;
    prim.path = "/" + water.name;
    prim.typeName = "Mesh";
    prim.mesh = static_cast<std::uint32_t>(model.scene.meshes.size() - 1);
    prim.material = static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    prim.offset.position = water.surface;
    prim.offset.position[2] -= kSheet / 2.0;
    // A stage's bare prim of this name becomes the drawn surface.
    for (ScenePrim& existing : model.scene.prims) {
        if (existing.parent == kNoIndex && existing.name == water.name &&
            existing.mesh == kNoIndex && existing.body == kNoIndex) {
            prim.path = existing.path;
            existing = prim;
            prim.name.clear();
            break;
        }
    }
    if (!prim.name.empty()) {
        model.scene.prims.push_back(std::move(prim));
    }
    SchemaInstance instance;
    instance.schema = "WaterAPI_1";
    instance.prim = model.world.addPrim("/" + water.name, water.name);
    model.world.prims.poseInBody.at(instance.prim.index).position = water.surface;
    instance.reals["water:sizeX"] = water.sizeX;
    instance.reals["water:sizeY"] = water.sizeY;
    instance.reals["water:depth"] = water.depth;
    instance.reals["water:density"] = water.density;
    instance.reals["water:dragCoefficient"] = water.drag;
    instance.reals["water:reflectivity"] = water.reflectivity;
    model.world.schemaInstances.push_back(std::move(instance));
}

ModuleInfo Water::info() const {
    return {"hydrodynamics", "water", 1, 0};
}

Duration Water::preferredStep() const {
    return Duration{1'000'000};
}

Status Water::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    waters_ = waterOf(world);
    bodies_.clear();
    const World& raw = world.raw();
    gravity_ = raw.gravityLocal;
    const std::vector<bool> moving = movingBodies(raw);
    for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
        if (!moving[b] || raw.bodies.kinematic[b]) {
            continue;
        }
        Body body;
        body.com = raw.bodies.comLocal[b];
        for (std::size_t c = 0; c < raw.colliders.body.size(); ++c) {
            if (raw.colliders.body[c] != BodyId{b}) {
                continue;
            }
            const ShapeType type = raw.colliders.shape[c];
            const std::array<double, 3>& size = raw.colliders.size[c];
            const ConvexHull* hull =
                type == ShapeType::ConvexHull ? &raw.hulls.at(raw.colliders.hull[c]) : nullptr;
            const ShapeBox box = boxOf(type, size, hull);
            if (box.volume <= 0.0) {
                continue;
            }
            std::vector<Cell> cells;
            for (int i = 0; i < kCells; ++i) {
                for (int j = 0; j < kCells; ++j) {
                    for (int k = 0; k < kCells; ++k) {
                        const std::array<double, 3> q{
                            box.lo[0] + (box.hi[0] - box.lo[0]) * (i + 0.5) / kCells,
                            box.lo[1] + (box.hi[1] - box.lo[1]) * (j + 0.5) / kCells,
                            box.lo[2] + (box.hi[2] - box.lo[2]) * (k + 0.5) / kCells};
                        if (inside(type, size, q, hull)) {
                            cells.push_back(
                                {compose(raw.colliders.poseInBody[c], Pose{q, {1.0, 0.0, 0.0, 0.0}})
                                     .position,
                                 0.0, (box.hi[2] - box.lo[2]) / kCells});
                        }
                    }
                }
            }
            for (Cell& cell : cells) {
                cell.volume = box.volume / static_cast<double>(cells.size());
                cell.area = std::pow(cell.volume, 2.0 / 3.0) / kCells;
                body.reach = std::max(body.reach, std::hypot(cell.at[0], cell.at[1], cell.at[2]) +
                                                      std::cbrt(cell.volume) * kCellReach);
            }
            body.volume += box.volume;
            body.cells.insert(body.cells.end(), cells.begin(), cells.end());
        }
        if (body.cells.empty()) {
            continue;
        }
        const std::string base = "body/" + raw.bodies.names[b] + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        SignalSpec twist = spec(base + "twist", SignalType::Wrench, units::m_per_s, "world_local");
        pose.feedback = twist.feedback = true;
        body.pose = d.input(pose);
        body.twist = d.input(twist);
        body.wrench =
            d.output(spec(base + "wrench/water", SignalType::Wrench, units::N, "world_local"));
        body.submerged = d.output(spec(base + "submerged", SignalType::Scalar, units::one));
        bodies_.push_back(std::move(body));
    }
    return {};
}

Status Water::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        bus.writeN(body.wrench, std::array<double, 6>{});
        bus.write(body.submerged, 0.0);
    }
    return {};
}

Status Water::step(const StepContext&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        const std::span<const double> p = bus.readN(body.pose);
        const std::span<const double> v = bus.readN(body.twist);
        std::array<double, 6> wrench{};
        double under = 0.0;
        // Most bodies are nowhere near water most of the time.
        const bool near =
            p.size() == 7 && std::any_of(waters_.begin(), waters_.end(), [&](const WaterBody& w) {
                return std::abs(p[0] - w.surface[0]) <= w.sizeX / 2.0 + body.reach &&
                       std::abs(p[1] - w.surface[1]) <= w.sizeY / 2.0 + body.reach &&
                       p[2] - body.reach <= w.surface[2] &&
                       p[2] + body.reach >= w.surface[2] - w.depth;
            });
        if (near && v.size() == 6) {
            const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            const std::array<double, 3> com = placed(pose, body.com);
            const std::array<double, 3> spin{v[3], v[4], v[5]};
            for (const Cell& cell : body.cells) {
                const std::array<double, 3> at = placed(pose, cell.at);
                for (const WaterBody& water : waters_) {
                    const double top = water.surface[2];
                    if (std::abs(at[0] - water.surface[0]) > water.sizeX / 2.0 ||
                        std::abs(at[1] - water.surface[1]) > water.sizeY / 2.0 ||
                        at[2] - cell.height / 2.0 > top ||
                        at[2] + cell.height / 2.0 < top - water.depth) {
                        continue;
                    }
                    // How much of the cell is under the surface.
                    const double fraction =
                        std::clamp((top - (at[2] - cell.height / 2.0)) / cell.height, 0.0, 1.0);
                    const double volume = cell.volume * fraction;
                    under += volume;
                    const std::array<double, 3> r{at[0] - p[0], at[1] - p[1], at[2] - p[2]};
                    const std::array<double, 3> around = cross(spin, r);
                    const std::array<double, 3> speed{v[0] + around[0], v[1] + around[1],
                                                      v[2] + around[2]};
                    const double magnitude = std::hypot(speed[0], speed[1], speed[2]);
                    // The cells' share of the frontal area: a stack of kCells makes one face.
                    const double area =
                        fraction >= 1.0 ? cell.area : std::pow(volume, 2.0 / 3.0) / kCells;
                    const double dragGain = 0.5 * water.density * water.drag * area * magnitude;
                    const std::array<double, 3> force{
                        -water.density * gravity_[0] * volume - dragGain * speed[0],
                        -water.density * gravity_[1] * volume - dragGain * speed[1],
                        -water.density * gravity_[2] * volume - dragGain * speed[2]};
                    const std::array<double, 3> arm{at[0] - com[0], at[1] - com[1], at[2] - com[2]};
                    const std::array<double, 3> moment = cross(arm, force);
                    for (std::size_t i = 0; i < 3; ++i) {
                        wrench.at(i) += force.at(i);
                        wrench.at(i + 3) += moment.at(i);
                    }
                    break;
                }
            }
        }
        bus.writeN(body.wrench, wrench);
        bus.write(body.submerged, under / body.volume);
    }
    return {};
}

// It keeps nothing between steps.
Status Water::saveState(StateWriter&) const {
    return {};
}

Status Water::restoreState(StateReader&) {
    return {};
}

}  // namespace lagrange::hydrodynamics
