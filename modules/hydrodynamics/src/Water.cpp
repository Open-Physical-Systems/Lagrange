#include "lagrange/hydrodynamics/Water.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>

namespace lagrange::hydrodynamics {

namespace {

constexpr int kCells = 6;  // a side, per collider
// The added mass of a sphere in potential flow, as a share of the displaced water's mass
// (Lamb, Hydrodynamics, 1932, §92); the same along every axis here.
constexpr double kAddedMassShare = 0.5;
// Beyond this k·d the water counts as deep: cosh and sinh of k(z + d) over sinh(k d) are e^(k z).
constexpr double kDeepWater = 20.0;
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
    // Its whole body, surface on top: the viewer draws the top as water and the rest as its sides.
    const double kSheet = std::max(water.depth, 0.01);
    model.scene.meshes.push_back(boxMesh(water.sizeX, water.sizeY, kSheet));
    model.scene.materials.push_back(
        {water.name, {0.08F, 0.28F, 0.42F, 0.8F}, SurfaceLook{0.0F, 0.04F, {0.0F, 0.0F, 0.0F}}});
    ScenePrim prim;
    prim.name = water.name;
    prim.path = "/" + water.name;
    prim.typeName = "Mesh";
    prim.apiSchemas.emplace_back("WaterAPI_1");  // the viewer draws it as water
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

bool fossenFromConfig(const ModuleConfig& config, const std::string& body, FossenCoefficients& out,
                      std::string& error) {
    const std::optional<std::string> text = config.get<std::string>("fossen:" + body);
    if (!text) {
        return false;
    }
    std::vector<double> v;
    std::string cell;
    for (std::istringstream in(*text); std::getline(in, cell, ',');) {
        char* end = nullptr;
        const double x = std::strtod(cell.c_str(), &end);
        if (end == cell.c_str() || !std::isfinite(x) || x < 0.0) {
            error = "fossen:" + body + ": '" + cell + "' is not a number >= 0";
            return false;
        }
        v.push_back(x);
    }
    if (v.size() != 18) {
        error = "fossen:" + body +
                " needs 18 numbers (added mass, linear and quadratic damping, "
                "each surge, sway, heave, roll, pitch, yaw), not " +
                std::to_string(v.size());
        return false;
    }
    for (std::size_t k = 0; k < 6; ++k) {
        out.addedMass.at(k) = v[k];
        out.linearDamping.at(k) = v[6 + k];
        out.quadraticDamping.at(k) = v[12 + k];
    }
    return true;
}

Status Water::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    waters_ = waterOf(world);
    bodies_.clear();
    const World& raw = world.raw();
    gravity_ = raw.gravityLocal;
    current_ = {config.get<double>("current_x").value_or(0.0),
                config.get<double>("current_y").value_or(0.0),
                config.get<double>("current_z").value_or(0.0)};
    waveAmplitude_ = std::max(0.0, config.get<double>("wave_amplitude").value_or(0.0));
    const double period = config.get<double>("wave_period").value_or(0.0);
    const double towards =
        config.get<double>("wave_towards").value_or(0.0) * std::numbers::pi / 180.0;
    waveHeading_ = {std::cos(towards), std::sin(towards)};
    waveOmega_ = waveAmplitude_ > 0.0 && period > 0.0 ? 2.0 * std::numbers::pi / period : 0.0;
    // The wavenumber of each body of water: ω² = g·k·tanh(k·d), by Newton from deep water's.
    const double g = std::hypot(gravity_[0], gravity_[1], gravity_[2]);
    wavenumbers_.assign(waters_.size(), 0.0);
    for (std::size_t w = 0; w < waters_.size() && waveOmega_ > 0.0 && g > 0.0; ++w) {
        const double d = waters_[w].depth;
        double k = waveOmega_ * waveOmega_ / g;
        for (int i = 0; i < 50; ++i) {
            const double t = std::tanh(k * d);
            const double f = g * k * t - waveOmega_ * waveOmega_;
            const double slope = g * t + g * k * d * (1.0 - t * t);
            const double step = f / slope;
            k -= step;
            if (std::abs(step) < 1e-12 * k) {
                break;
            }
        }
        wavenumbers_[w] = k;
    }
    const std::vector<bool> moving = movingBodies(raw);
    for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
        if (!moving[b] || raw.bodies.kinematic[b]) {
            continue;
        }
        Body body;
        body.com = raw.bodies.comLocal[b];
        body.mass = raw.bodies.mass[b];
        std::string problem;
        body.fossen = fossenFromConfig(config, raw.bodies.names[b], body.coefficients, problem);
        if (!problem.empty()) {
            return Status::error(StatusCode::InvalidConfig, problem);
        }
        const std::array<double, 6>& inertia = raw.bodies.inertiaLocal[b];
        body.rigid = {body.mass, body.mass, body.mass, inertia[0], inertia[1], inertia[2]};
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
    for (Body& body : bodies_) {
        body.lastVelocity = {};
        body.acceleration = {};
        body.started = false;
        body.lastRelative = {};
        body.relativeAcceleration = {};
        bus.writeN(body.wrench, std::array<double, 6>{});
        bus.write(body.submerged, 0.0);
    }
    return {};
}

Water::Flow Water::flowAt(const WaterBody& water, double wavenumber,
                          const std::array<double, 3>& at, double t) const {
    Flow flow;
    flow.velocity = current_;
    flow.surface = water.surface[2];
    if (wavenumber <= 0.0) {
        return flow;
    }
    // Linear (Airy) waves (Dean and Dalrymple 1991, ch. 3): phase θ = k·x − ω·t along the
    // heading, z from the still surface (taken at most 0: linear theory stops there).
    const double x =
        (at[0] - water.surface[0]) * waveHeading_[0] + (at[1] - water.surface[1]) * waveHeading_[1];
    const double phase = wavenumber * x - waveOmega_ * t;
    const double z = std::min(at[2] - water.surface[2], 0.0);
    const double kd = wavenumber * water.depth;
    double along = 0.0;  // cosh k(z + d) / sinh k d
    double up = 0.0;     // sinh k(z + d) / sinh k d
    if (kd > kDeepWater) {
        along = up = std::exp(wavenumber * z);
    } else {
        const double s = std::sinh(kd);
        along = std::cosh(wavenumber * (z + water.depth)) / s;
        up = std::sinh(wavenumber * (z + water.depth)) / s;
    }
    const double a = waveAmplitude_;
    const double w = waveOmega_;
    const double horizontal = a * w * along * std::cos(phase);
    const double vertical = a * w * up * std::sin(phase);
    const double horizontalRate = a * w * w * along * std::sin(phase);
    const double verticalRate = -a * w * w * up * std::cos(phase);
    flow.velocity[0] += horizontal * waveHeading_[0];
    flow.velocity[1] += horizontal * waveHeading_[1];
    flow.velocity[2] += vertical;
    flow.acceleration = {horizontalRate * waveHeading_[0], horizontalRate * waveHeading_[1],
                         verticalRate};
    flow.surface += a * std::cos(phase);
    return flow;
}

Status Water::step(const StepContext& context, SignalBus& bus) {
    const double t = context.t.seconds();
    const double dt = context.dt.seconds();
    for (Body& body : bodies_) {
        const std::span<const double> p = bus.readN(body.pose);
        const std::span<const double> v = bus.readN(body.twist);
        std::array<double, 6> wrench{};
        double under = 0.0;
        std::array<double, 3> waterAcceleration{};  // volume-weighted, over the submerged cells
        // Most bodies are nowhere near water most of the time.
        const bool near =
            p.size() == 7 && std::any_of(waters_.begin(), waters_.end(), [&](const WaterBody& w) {
                return std::abs(p[0] - w.surface[0]) <= w.sizeX / 2.0 + body.reach &&
                       std::abs(p[1] - w.surface[1]) <= w.sizeY / 2.0 + body.reach &&
                       p[2] - body.reach <= w.surface[2] + waveAmplitude_ &&
                       p[2] + body.reach >= w.surface[2] - w.depth;
            });
        if (near && v.size() == 6) {
            const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            const std::array<double, 3> com = placed(pose, body.com);
            const std::array<double, 3> spin{v[3], v[4], v[5]};
            for (const Cell& cell : body.cells) {
                const std::array<double, 3> at = placed(pose, cell.at);
                for (std::size_t w = 0; w < waters_.size(); ++w) {
                    const WaterBody& water = waters_[w];
                    if (std::abs(at[0] - water.surface[0]) > water.sizeX / 2.0 ||
                        std::abs(at[1] - water.surface[1]) > water.sizeY / 2.0 ||
                        at[2] + cell.height / 2.0 < water.surface[2] - water.depth) {
                        continue;
                    }
                    const Flow flow = flowAt(water, wavenumbers_[w], at, t);
                    const double top = flow.surface;
                    if (at[2] - cell.height / 2.0 > top) {
                        continue;
                    }
                    // How much of the cell is under the surface.
                    const double fraction =
                        std::clamp((top - (at[2] - cell.height / 2.0)) / cell.height, 0.0, 1.0);
                    const double volume = cell.volume * fraction;
                    under += volume;
                    for (std::size_t k = 0; k < 3; ++k) {
                        waterAcceleration.at(k) += volume * flow.acceleration.at(k);
                    }
                    const std::array<double, 3> r{at[0] - p[0], at[1] - p[1], at[2] - p[2]};
                    const std::array<double, 3> around = cross(spin, r);
                    // Through the water: the cell's velocity less the water's there.
                    const std::array<double, 3> speed{v[0] + around[0] - flow.velocity[0],
                                                      v[1] + around[1] - flow.velocity[1],
                                                      v[2] + around[2] - flow.velocity[2]};
                    const double magnitude = std::hypot(speed[0], speed[1], speed[2]);
                    // The cells' share of the frontal area: a stack of kCells makes one face.
                    const double area =
                        fraction >= 1.0 ? cell.area : std::pow(volume, 2.0 / 3.0) / kCells;
                    const double dragGain =
                        body.fossen ? 0.0 : 0.5 * water.density * water.drag * area * magnitude;
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
        // Morison's inertia terms at the centre of mass: the water's acceleration pushes the
        // displaced volume, and the added mass resists the body's acceleration through it.
        const double density = waters_.empty() ? 1000.0 : waters_.front().density;
        const double added = body.fossen ? 0.0 : kAddedMassShare * density * under;
        if (body.fossen && v.size() == 6 && p.size() == 7) {
            fossenWrench(body, p, v, t, dt, under, wrench);
        }
        if (v.size() == 6) {
            const std::array<double, 3> now{v[0], v[1], v[2]};
            const double gain = body.mass > 0.0 ? 1.0 / (1.0 + added / body.mass) : 1.0;
            if (body.started && dt > 0.0) {
                for (std::size_t k = 0; k < 3; ++k) {
                    const double measured = (now.at(k) - body.lastVelocity.at(k)) / dt;
                    body.acceleration.at(k) += gain * (measured - body.acceleration.at(k));
                }
            }
            body.lastVelocity = now;
            body.started = true;
        }
        if (under > 0.0) {
            for (std::size_t k = 0; k < 3; ++k) {
                const double aw = waterAcceleration.at(k) / under;
                wrench.at(k) += density * under * aw + added * (aw - body.acceleration.at(k));
            }
        }
        bus.writeN(body.wrench, wrench);
        bus.write(body.submerged, under / body.volume);
    }
    return {};
}

void Water::fossenWrench(Body& body, std::span<const double> p, std::span<const double> v, double t,
                         double dt, double under, std::array<double, 6>& wrench) {
    const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
    const std::array<double, 3> com = placed(pose, body.com);
    const std::array<double, 4> toBody = conjugate(pose.orientation);
    // The water's velocity at the centre of mass (the first water body that holds it).
    std::array<double, 3> water{};
    for (std::size_t w = 0; w < waters_.size(); ++w) {
        const WaterBody& wb = waters_[w];
        if (std::abs(com[0] - wb.surface[0]) <= wb.sizeX / 2.0 &&
            std::abs(com[1] - wb.surface[1]) <= wb.sizeY / 2.0) {
            water = flowAt(wb, wavenumbers_[w], com, t).velocity;
            break;
        }
    }
    const std::array<double, 3> omegaWorld{v[3], v[4], v[5]};
    const std::array<double, 3> arm = rotate(pose.orientation, body.com);
    const std::array<double, 3> spin = cross(omegaWorld, arm);
    const std::array<double, 3> through = rotate(
        toBody, {v[0] + spin[0] - water[0], v[1] + spin[1] - water[1], v[2] + spin[2] - water[2]});
    const std::array<double, 3> omega = rotate(toBody, omegaWorld);
    const std::array<double, 6> nu{through[0], through[1], through[2],
                                   omega[0],   omega[1],   omega[2]};
    const double share = body.volume > 0.0 ? std::clamp(under / body.volume, 0.0, 1.0) : 0.0;
    const FossenCoefficients& c = body.coefficients;
    std::array<double, 6> a{};
    for (std::size_t k = 0; k < 6; ++k) {
        a.at(k) = share * c.addedMass.at(k);
    }
    // ν̇_r, smoothed per axis so that a light body with much added mass stays stable.
    if (body.started && dt > 0.0) {
        for (std::size_t k = 0; k < 6; ++k) {
            const double gain =
                body.rigid.at(k) > 0.0 ? 1.0 / (1.0 + a.at(k) / body.rigid.at(k)) : 1.0;
            const double measured = (nu.at(k) - body.lastRelative.at(k)) / dt;
            body.relativeAcceleration.at(k) += gain * (measured - body.relativeAcceleration.at(k));
        }
    }
    body.lastRelative = nu;
    // −M_A·ν̇_r − C_A(ν_r)·ν_r − D(ν_r)·ν_r, body frame. With A_t, A_r the translational and
    // rotational added masses: −C_A·ν = (ω × A_t·u, u × A_t·u + ω × A_r·ω).
    const std::array<double, 3> u{nu[0], nu[1], nu[2]};
    const std::array<double, 3> w{nu[3], nu[4], nu[5]};
    const std::array<double, 3> au{a[0] * u[0], a[1] * u[1], a[2] * u[2]};
    const std::array<double, 3> aw{a[3] * w[0], a[4] * w[1], a[5] * w[2]};
    const std::array<double, 3> f1 = cross(w, au);
    const std::array<double, 3> m1 = cross(u, au);
    const std::array<double, 3> m2 = cross(w, aw);
    std::array<double, 6> tau{};
    for (std::size_t k = 0; k < 6; ++k) {
        const double damping =
            share * (c.linearDamping.at(k) + c.quadraticDamping.at(k) * std::abs(nu.at(k)));
        tau.at(k) = -a.at(k) * body.relativeAcceleration.at(k) - damping * nu.at(k);
    }
    for (std::size_t k = 0; k < 3; ++k) {
        tau.at(k) += f1.at(k);
        tau.at(k + 3) += m1.at(k) + m2.at(k);
    }
    // Back to the world, the force at the centre of mass (wrench/water is about it).
    const std::array<double, 3> force = rotate(pose.orientation, {tau[0], tau[1], tau[2]});
    const std::array<double, 3> moment = rotate(pose.orientation, {tau[3], tau[4], tau[5]});
    for (std::size_t k = 0; k < 3; ++k) {
        wrench.at(k) += force.at(k);
        wrench.at(k + 3) += moment.at(k);
    }
}

// It keeps each body's last velocity and smoothed acceleration, for the added mass.
Status Water::saveState(StateWriter& writer) const {
    for (const Body& body : bodies_) {
        for (const double x : body.lastVelocity) {
            writer.put(x);
        }
        for (const double x : body.acceleration) {
            writer.put(x);
        }
        writer.put(body.started ? 1.0 : 0.0);
    }
    // Fossen's bodies after all the others, so that states without them still restore.
    for (const Body& body : bodies_) {
        if (body.fossen) {
            for (const double x : body.lastRelative) {
                writer.put(x);
            }
            for (const double x : body.relativeAcceleration) {
                writer.put(x);
            }
        }
    }
    return {};
}

Status Water::restoreState(StateReader& reader) {
    for (Body& body : bodies_) {
        double started = 0.0;
        bool ok = true;
        // A state saved before the water kept anything (1.0 to 1.x): start afresh.
        if (!reader.get(body.lastVelocity[0])) {
            for (Body& fresh : bodies_) {
                fresh.lastVelocity = {};
                fresh.acceleration = {};
                fresh.started = false;
            }
            return {};
        }
        for (std::size_t k = 1; k < 3; ++k) {
            ok = ok && reader.get(body.lastVelocity.at(k));
        }
        for (double& x : body.acceleration) {
            ok = ok && reader.get(x);
        }
        ok = ok && reader.get(started);
        if (!ok) {
            return Status::error(StatusCode::InvalidConfig, "water state truncated");
        }
        body.started = started != 0.0;
    }
    for (Body& body : bodies_) {
        if (!body.fossen) {
            continue;
        }
        bool fossenOk = true;
        for (double& x : body.lastRelative) {
            fossenOk = fossenOk && reader.get(x);
        }
        for (double& x : body.relativeAcceleration) {
            fossenOk = fossenOk && reader.get(x);
        }
        if (!fossenOk) {  // saved before the body had its own coefficients
            body.lastRelative = {};
            body.relativeAcceleration = {};
        }
    }
    return {};
}

}  // namespace lagrange::hydrodynamics
