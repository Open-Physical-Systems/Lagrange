#include "lagrange/vegetation/Vegetation.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <string>

namespace lagrange::vegetation {

namespace {

constexpr int kCells = 4;  // a side, per collider

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

/// A shape's half extents in its frame and its volume (0 for planes and height fields).
std::pair<std::array<double, 3>, double> extentOf(ShapeType type, const std::array<double, 3>& s,
                                                  const ConvexHull* hull,
                                                  std::array<double, 3>& centre) {
    constexpr double kPi = std::numbers::pi;
    centre = {};
    switch (type) {
        case ShapeType::Sphere:
            return {{s[0], s[0], s[0]}, 4.0 / 3.0 * kPi * s[0] * s[0] * s[0]};
        case ShapeType::Capsule:
            return {{s[0], s[0], s[1] + s[0]}, kPi * s[0] * s[0] * (2.0 * s[1] + 4.0 / 3.0 * s[0])};
        case ShapeType::Cylinder:
            return {{s[0], s[0], s[1]}, kPi * s[0] * s[0] * 2.0 * s[1]};
        case ShapeType::Box:
            return {s, 8.0 * s[0] * s[1] * s[2]};
        case ShapeType::ConvexHull: {
            if (hull == nullptr || hull->points.empty()) {
                return {{}, 0.0};
            }
            std::array<double, 3> lo = hull->points.front();
            std::array<double, 3> hi = lo;
            for (const auto& p : hull->points) {
                for (std::size_t k = 0; k < 3; ++k) {
                    lo.at(k) = std::min(lo.at(k), p.at(k));
                    hi.at(k) = std::max(hi.at(k), p.at(k));
                }
            }
            double volume = 0.0;  // the divergence theorem over outward triangles
            for (const auto& t : hull->triangles) {
                const auto& a = hull->points.at(t[0]);
                const auto& b = hull->points.at(t[1]);
                const auto& c = hull->points.at(t[2]);
                volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                           a[2] * (b[0] * c[1] - b[1] * c[0])) /
                          6.0;
            }
            std::array<double, 3> half{};
            for (std::size_t k = 0; k < 3; ++k) {
                centre.at(k) = (lo.at(k) + hi.at(k)) / 2.0;
                half.at(k) = (hi.at(k) - lo.at(k)) / 2.0;
            }
            return {half, std::abs(volume)};
        }
        case ShapeType::Plane:
        case ShapeType::HeightField:
            break;
    }
    return {{}, 0.0};
}

bool insideShape(ShapeType type, const std::array<double, 3>& s, const std::array<double, 3>& q,
                 const ConvexHull* hull) {
    switch (type) {
        case ShapeType::ConvexHull:
            return std::ranges::all_of(hull->faces, [&](const std::array<double, 4>& f) {
                return f[0] * q[0] + f[1] * q[1] + f[2] * q[2] <= f[3];
            });
        case ShapeType::Sphere:
            return std::hypot(q[0], q[1], q[2]) <= s[0];
        case ShapeType::Capsule:
            return std::hypot(q[0], q[1], q[2] - std::clamp(q[2], -s[1], s[1])) <= s[0];
        case ShapeType::Cylinder:
            return std::hypot(q[0], q[1]) <= s[0];
        default:
            return true;
    }
}

}  // namespace

std::vector<Canopy> canopiesOf(const World& world) {
    std::vector<Canopy> out;
    const World::Colliders& c = world.colliders;
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        if (c.body[i] != kWorldBody || c.material[i] == kNoMaterial ||
            (c.shape[i] != ShapeType::Box && c.shape[i] != ShapeType::Plane)) {
            continue;
        }
        const SurfaceMaterial& m = world.surfaceMaterials.at(c.material[i]);
        if (m.canopyHeight > 0.0) {
            out.push_back(
                {c.shape[i], c.size[i], c.poseInBody[i], m.canopyHeight, m.leafAreaIndex});
        }
    }
    return out;
}

bool insideCanopy(const Canopy& canopy, const std::array<double, 3>& point) {
    const std::array<double, 3> q =
        rotate(conjugate(canopy.pose.orientation),
               {point[0] - canopy.pose.position[0], point[1] - canopy.pose.position[1],
                point[2] - canopy.pose.position[2]});
    if (canopy.type == ShapeType::Plane) {
        return q[2] >= 0.0 && q[2] <= canopy.height;
    }
    return std::abs(q[0]) <= canopy.size[0] && std::abs(q[1]) <= canopy.size[1] &&
           q[2] >= canopy.size[2] && q[2] <= canopy.size[2] + canopy.height;
}

double frontalAreaDensity(double height, double leafAreaIndex) {
    if (!(height > 0.0) || !(leafAreaIndex > 0.0)) {
        return 0.0;
    }
    return kLeafProjection * leafAreaIndex / height;
}

std::array<double, 3> stemDrag(double density, double dragCoefficient, double frontalArea,
                               double volume, const std::array<double, 3>& velocity) {
    const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
    const double gain = 0.5 * density * dragCoefficient * frontalArea * volume * speed;
    return {-gain * velocity[0], -gain * velocity[1], -gain * velocity[2]};
}

ModuleInfo Vegetation::info() const {
    return {"vegetation", "stem_drag", 1, 0};
}

Duration Vegetation::preferredStep() const {
    return Duration{1'000'000};
}

Status Vegetation::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    const World& raw = world.raw();
    canopies_ = canopiesOf(raw);
    bodies_.clear();
    density_ = std::max(0.0, config.get<double>("density").value_or(0.0));
    dragCoefficient_ =
        std::max(0.0, config.get<double>("drag_coefficient").value_or(kStemDragCoefficient));
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
            std::array<double, 3> centre{};
            const auto [half, volume] = extentOf(type, size, hull, centre);
            if (volume <= 0.0) {
                continue;
            }
            std::vector<Cell> cells;
            for (int i = 0; i < kCells; ++i) {
                for (int j = 0; j < kCells; ++j) {
                    for (int k = 0; k < kCells; ++k) {
                        const std::array<double, 3> q{
                            centre[0] + half[0] * (2.0 * (i + 0.5) / kCells - 1.0),
                            centre[1] + half[1] * (2.0 * (j + 0.5) / kCells - 1.0),
                            centre[2] + half[2] * (2.0 * (k + 0.5) / kCells - 1.0)};
                        if (insideShape(type, size, q, hull)) {
                            cells.push_back(
                                {compose(raw.colliders.poseInBody[c], Pose{q, {1.0, 0.0, 0.0, 0.0}})
                                     .position,
                                 0.0});
                        }
                    }
                }
            }
            for (Cell& cell : cells) {  // the shape's true volume, shared out
                cell.volume = volume / static_cast<double>(cells.size());
            }
            body.volume += volume;
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
            d.output(spec(base + "wrench/vegetation", SignalType::Wrench, units::N, "world_local"));
        body.inside = d.output(spec(base + "in_vegetation", SignalType::Scalar, units::one));
        bodies_.push_back(std::move(body));
    }
    return {};
}

Status Vegetation::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        bus.writeN(body.wrench, std::array<double, 6>{});
        bus.write(body.inside, 0.0);
    }
    return {};
}

Status Vegetation::step(const StepContext&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        const std::span<const double> p = bus.readN(body.pose);
        const std::span<const double> v = bus.readN(body.twist);
        std::array<double, 6> wrench{};
        double inside = 0.0;
        if (p.size() == 7 && v.size() == 6) {
            const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            const std::array<double, 3> com = compose(pose, Pose{body.com, {1, 0, 0, 0}}).position;
            const std::array<double, 3> spin{v[3], v[4], v[5]};
            for (const Cell& cell : body.cells) {
                const std::array<double, 3> at =
                    compose(pose, Pose{cell.at, {1, 0, 0, 0}}).position;
                for (const Canopy& canopy : canopies_) {
                    if (!insideCanopy(canopy, at)) {
                        continue;
                    }
                    inside += cell.volume;
                    const std::array<double, 3> around =
                        cross(spin, {at[0] - p[0], at[1] - p[1], at[2] - p[2]});
                    const std::array<double, 3> force = stemDrag(
                        density_, dragCoefficient_,
                        frontalAreaDensity(canopy.height, canopy.leafAreaIndex), cell.volume,
                        {v[0] + around[0], v[1] + around[1], v[2] + around[2]});
                    const std::array<double, 3> moment =
                        cross({at[0] - com[0], at[1] - com[1], at[2] - com[2]}, force);
                    for (std::size_t k = 0; k < 3; ++k) {
                        wrench.at(k) += force.at(k);
                        wrench.at(k + 3) += moment.at(k);
                    }
                    break;  // one canopy per cell: overlapping canopies do not add up
                }
            }
        }
        bus.writeN(body.wrench, wrench);
        bus.write(body.inside, body.volume > 0.0 ? inside / body.volume : 0.0);
    }
    return {};
}

}  // namespace lagrange::vegetation
