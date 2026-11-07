// Rotor downwash and ground outwash (Downwash.hpp; docs/design/domains/rotor_aero.md §2).
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>

namespace lagrange::aerodynamics {

namespace {

using Vec3 = Downwash::Vec3;

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/// Distance along `axis` (unit) and from it of `x` relative to `origin`.
void split(const Vec3& x, const Vec3& origin, const Vec3& axis, double& along, double& across) {
    const Vec3 d = sub(x, origin);
    along = dot(d, axis);
    const Vec3 perp{d[0] - along * axis[0], d[1] - along * axis[1], d[2] - along * axis[2]};
    across = norm(perp);
}

/// Edge of the near-wake column: a smoothed top hat 0.1 R wide (a modelling choice; the shear
/// layer of a real slipstream has about this thickness a radius or two below the disc).
constexpr double kNearEdge = 0.1;

/// The calibration case of the outwash fit: a single rotor 1 R above the ground, whose near
/// wake arrives at w = (1 + 1/√2)·v_i with radius R/√(1 + 1/√2).
const double kCalArriving = 1.0 + 1.0 / std::numbers::sqrt2;
const double kCalRadius = 1.0 / std::sqrt(kCalArriving);

/// The top-hat radius with the far jet's momentum flux and centreline speed:
/// ∫(1 + aξ²)^−4·2πξ dξ = π/(3a), so r_t = r½/√(3a), a = √2 − 1.
const double kFarTopHat = 1.0 / std::sqrt(3.0 * (std::numbers::sqrt2 - 1.0));

Vec3 toLocal(const Downwash::Box& b, const Vec3& x) {
    return rotate(conjugate(b.orientation), sub(x, b.centre));
}

/// The ray's first entry into the box (origin outside): distance along the ray, and the face
/// (axis, side); false when it misses.
bool rayBox(const Downwash::Box& b, const Vec3& origin, const Vec3& direction, double& t, int& axis,
            double& side) {
    const Vec3 o = toLocal(b, origin);
    const Vec3 d = rotate(conjugate(b.orientation), direction);
    double near = -std::numeric_limits<double>::infinity();
    double far = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 3; ++k) {
        if (std::abs(d[k]) < 1e-12) {
            if (std::abs(o[k]) > b.half[k]) {
                return false;
            }
            continue;
        }
        double t0 = (-b.half[k] - o[k]) / d[k];
        double t1 = (b.half[k] - o[k]) / d[k];
        double entrySide = -1.0;
        if (t0 > t1) {
            std::swap(t0, t1);
            entrySide = 1.0;
        }
        if (t0 > near) {
            near = t0;
            axis = k;
            side = entrySide;
        }
        far = std::min(far, t1);
    }
    if (near > far || near <= 0.0) {
        return false;
    }
    t = near;
    return true;
}

double mergeWeight(double sOverL) {
    return std::clamp((sOverL - kJetMergeStart) / (kJetMergeEnd - kJetMergeStart), 0.0, 1.0);
}

/// The jet arriving at distance s along the wake: centreline speed and top-hat radius.
void arriving(const Downwash::Group& g, double s, double& speed, double& radius) {
    double near = 0.0;
    double area = 0.0;
    for (const Downwash::Rotor& r : g.rotors) {
        near += nearWakeSpeed(r.induced, r.radius, s);
        area += r.radius * r.radius;
    }
    near /= static_cast<double>(g.rotors.size());
    // The rotors' disc area as one disc, contracted by continuity.
    const double nearRadius = std::sqrt(area) * std::sqrt(g.hoverInduced / std::max(near, 1e-9));
    const double beta = mergeWeight(s / g.length);
    speed = (1.0 - beta) * near + beta * farJetSpeed(g.hoverInduced, g.length, s);
    radius = (1.0 - beta) * nearRadius + beta * kFarTopHat * farJetHalfWidth(g.length, s);
}

}  // namespace

double nearWakeSpeed(double induced, double radius, double s) {
    if (s <= 0.0) {
        return 0.0;
    }
    return induced * (1.0 + s / std::sqrt(s * s + radius * radius));
}

double nearWakeRadius(double induced, double radius, double s) {
    const double w = nearWakeSpeed(induced, radius, std::max(s, 0.0));
    return w > 0.0 ? radius * std::sqrt(induced / w) : radius;
}

double farJetSpeed(double hoverInduced, double length, double s) {
    return hoverInduced * kJetDecay / (s / length - kJetOrigin);
}

double farJetHalfWidth(double length, double s) {
    return kJetSpread * (s - kJetOrigin * length);
}

double farJetProfile(double xi) {
    const double d = 1.0 + (std::numbers::sqrt2 - 1.0) * xi * xi;
    return 1.0 / (d * d);
}

double outwashSpeed(double arriving, double jetRadius, double rho, double z) {
    if (arriving <= 0.0 || jetRadius <= 0.0 || rho <= 0.0 || z < 0.0) {
        return 0.0;
    }
    const double r = jetRadius / kCalRadius;  // the calibration rotor's radius
    const double peakRadius = kOutwashPeakRadius * r;
    const double peak = arriving * kOutwashPeakSpeed / kCalArriving;
    const double radial = rho < peakRadius ? std::pow(rho / peakRadius, kOutwashRise)
                                           : std::pow(peakRadius / rho, kOutwashDecay);
    const double zPeak = kOutwashPeakHeight * r;
    const double layer = kOutwashLayer * r + kOutwashGrowth * std::max(0.0, rho - peakRadius);
    double vertical = 0.0;
    if (z <= zPeak) {
        vertical = std::pow(z / zPeak, 1.0 / 7.0);  // the turbulent wall layer's 1/7 power
    } else {
        const double eta = (z - zPeak) / (layer - zPeak);
        vertical = std::exp(-std::numbers::ln2 * eta * eta);
    }
    return peak * radial * vertical;
}

void Downwash::setGround(const Vec3& point, const Vec3& normal) {
    const double n = norm(normal);
    hasGround_ = n > 0.0;
    groundPoint_ = point;
    groundNormal_ = hasGround_ ? Vec3{normal[0] / n, normal[1] / n, normal[2] / n} : normal;
}

void Downwash::update(std::span<const Rotor> rotors, std::span<const Vec3> relativeAir) {
    groups_.clear();
    std::map<std::size_t, std::size_t> index;  // group id → position, in id order
    for (const Rotor& r : rotors) {
        if (r.thrust <= 0.0 || r.induced <= 0.0 || r.radius <= 0.0) {
            continue;
        }
        auto [it, added] = index.try_emplace(r.group, groups_.size());
        if (added) {
            groups_.push_back({});
            groups_.back().id = r.group;
        }
        groups_[it->second].rotors.push_back(r);
    }
    for (Group& g : groups_) {
        const auto n = static_cast<double>(g.rotors.size());
        Vec3 thrust{};
        double v2 = 0.0;
        for (const Rotor& r : g.rotors) {
            for (std::size_t k = 0; k < 3; ++k) {
                g.centre[k] += r.hub[k] / n;
                thrust[k] += r.thrustAxis[k];
            }
            v2 += r.induced * r.induced / n;
        }
        const double t = norm(thrust);
        g.down =
            t > 0.0 ? Vec3{-thrust[0] / t, -thrust[1] / t, -thrust[2] / t} : Vec3{0.0, 0.0, -1.0};
        g.hoverInduced = std::sqrt(v2);
        if (g.rotors.size() >= 2) {
            double spread = 0.0;
            for (const Rotor& r : g.rotors) {
                spread += norm(sub(r.hub, g.centre)) / n;
            }
            g.length = 2.0 * spread;
        }
        if (g.length <= 0.0) {
            // A single rotor: its diameter stands for the motor distance. TODO(verify): the
            // far-jet constants were fitted to multirotors only.
            g.length = 2.0 * g.rotors.front().radius;
        }
        // Wake skew: the column leans with the air going past the vehicle.
        Vec3 lean{g.down[0] * g.hoverInduced, g.down[1] * g.hoverInduced,
                  g.down[2] * g.hoverInduced};
        if (g.id < relativeAir.size()) {
            for (std::size_t k = 0; k < 3; ++k) {
                lean[k] += relativeAir[g.id][k];
            }
        }
        const double l = norm(lean);
        g.wakeAxis = l > 1e-9 ? Vec3{lean[0] / l, lean[1] / l, lean[2] / l} : g.down;
        findSurfaces(g);
    }
}

void Downwash::findSurfaces(Group& g) const {
    g.hits = false;
    g.hitBox = -1;
    g.walls.clear();
    double best = std::numeric_limits<double>::infinity();
    if (hasGround_) {
        const double towards = -dot(g.wakeAxis, groundNormal_);
        const double height = dot(sub(g.centre, groundPoint_), groundNormal_);
        if (towards > 1e-3 && height > 0.0) {
            best = height / towards;
            g.hits = true;
            g.hitNormal = groundNormal_;
        }
    }
    for (std::size_t b = 0; b < boxes_.size(); ++b) {
        double t = 0.0;
        int axis = 0;
        double side = 1.0;
        if (rayBox(boxes_[b], g.centre, g.wakeAxis, t, axis, side) && t < best) {
            best = t;
            g.hits = true;
            g.hitBox = static_cast<int>(b);
            g.hitAxis = axis;
            g.hitSide = side;
            Vec3 local{};
            local[static_cast<std::size_t>(axis)] = side;
            g.hitNormal = rotate(boxes_[b].orientation, local);
        }
    }
    if (g.hits) {
        g.hitPoint = {g.centre[0] + best * g.wakeAxis[0], g.centre[1] + best * g.wakeAxis[1],
                      g.centre[2] + best * g.wakeAxis[2]};
        arriving(g, best, g.arriveSpeed, g.arriveRadius);
    }
    // Upright faces near the wake, the drone outside them, mirror it.
    // As far as the jet spreads before it stops (or ten motor distances down without a surface).
    const double end = g.hits ? best : 10.0 * g.length;
    const double reach = std::max(1.5 * g.length + 2.0 * g.rotors.front().radius,
                                  2.0 * farJetHalfWidth(g.length, end));
    for (std::size_t b = 0; b < boxes_.size(); ++b) {
        const Box& box = boxes_[b];
        const Vec3 c = toLocal(box, g.centre);
        for (int k = 0; k < 3; ++k) {
            for (const double side : {-1.0, 1.0}) {
                Vec3 local{};
                local[static_cast<std::size_t>(k)] = side;
                const Vec3 n = rotate(box.orientation, local);
                const double out =
                    side * c[static_cast<std::size_t>(k)] - box.half[static_cast<std::size_t>(k)];
                if (std::abs(n[2]) > 0.3 || out < 0.0 || out > reach) {
                    continue;  // not upright, the drone not outside it, or too far
                }
                // The drone faces it: within its width (plus the reach) and below its top
                // (plus a motor distance: a wake from just above a roof still hugs its wall).
                bool facing = true;
                for (std::size_t j = 0; j < 3; ++j) {
                    if (j == static_cast<std::size_t>(k)) {
                        continue;
                    }
                    Vec3 unit{};
                    unit[j] = 1.0;
                    const bool upright = std::abs(rotate(box.orientation, unit)[2]) > 0.7;
                    facing = facing && std::abs(c[j]) <= box.half[j] + (upright ? g.length : reach);
                }
                if (facing) {
                    Vec3 point = box.centre;
                    for (std::size_t m = 0; m < 3; ++m) {
                        point[m] += n[m] * box.half[static_cast<std::size_t>(k)];
                    }
                    g.walls.emplace_back(point, n);
                }
            }
        }
    }
}

bool Downwash::onHitFace(const Group& g, const Vec3& x) const {
    if (g.hitBox < 0) {
        return true;  // the ground goes on
    }
    const Box& box = boxes_[static_cast<std::size_t>(g.hitBox)];
    const Vec3 local = toLocal(box, x);
    for (int k = 0; k < 3; ++k) {
        if (k != g.hitAxis &&
            std::abs(local[static_cast<std::size_t>(k)]) > box.half[static_cast<std::size_t>(k)]) {
            return false;  // past the roof's edge
        }
    }
    return true;
}

bool Downwash::insideObstacle(const Vec3& x) const {
    return std::ranges::any_of(boxes_, [&](const Box& b) {
        const Vec3 l = toLocal(b, x);
        return std::abs(l[0]) <= b.half[0] && std::abs(l[1]) <= b.half[1] &&
               std::abs(l[2]) <= b.half[2];
    });
}

double Downwash::nearestWall(const Vec3& x, Vec3& normal) const {
    double best = std::numeric_limits<double>::infinity();
    for (const Box& b : boxes_) {
        const Vec3 l = toLocal(b, x);
        for (std::size_t k = 0; k < 3; ++k) {
            const double out = std::abs(l[k]) - b.half[k];
            if (out < 0.0 || out >= best) {
                continue;
            }
            Vec3 local{};
            local[k] = l[k] >= 0.0 ? 1.0 : -1.0;
            const Vec3 n = rotate(b.orientation, local);
            if (std::abs(n[2]) > 0.3) {
                continue;  // a roof or a floor
            }
            bool beside = true;
            for (std::size_t j = 0; j < 3; ++j) {
                beside = beside && (j == k || std::abs(l[j]) <= b.half[j]);
            }
            if (beside) {
                best = out;
                normal = n;
            }
        }
    }
    return best;
}

double Downwash::axialSpeed(const Group& g, const Vec3& x, double& s) const {
    double across = 0.0;
    split(x, g.centre, g.wakeAxis, s, across);
    if (s <= 0.0) {
        return 0.0;
    }
    const double beta = mergeWeight(s / g.length);
    double near = 0.0;
    if (beta < 1.0) {
        for (const Rotor& r : g.rotors) {
            double si = 0.0;
            double ri = 0.0;
            split(x, r.hub, g.wakeAxis, si, ri);
            if (si <= 0.0) {
                continue;
            }
            const double edge = nearWakeRadius(r.induced, r.radius, si);
            near += nearWakeSpeed(r.induced, r.radius, si) * 0.5 *
                    (1.0 - std::tanh((ri - edge) / (kNearEdge * r.radius)));
        }
    }
    double far = 0.0;
    if (beta > 0.0) {
        far = farJetSpeed(g.hoverInduced, g.length, s) *
              farJetProfile(across / farJetHalfWidth(g.length, s));
    }
    return (1.0 - beta) * near + beta * far;
}

Vec3 Downwash::velocityAt(const Vec3& x, std::size_t exclude) const {
    Vec3 out{};
    if (hasGround_ && dot(sub(x, groundPoint_), groundNormal_) < 0.0) {
        return out;  // under the ground
    }
    if (insideObstacle(x)) {
        return out;  // inside a building
    }
    for (const Group& g : groups_) {
        if (g.id == exclude) {
            continue;
        }
        double s = 0.0;
        double u = axialSpeed(g, x, s);
        // Walls beside the wake: the image jet, so that the air does not go through them.
        for (const auto& [point, normal] : g.walls) {
            const double d = dot(sub(x, point), normal);
            if (d < 0.0) {
                continue;
            }
            const Vec3 mirrored{x[0] - 2.0 * d * normal[0], x[1] - 2.0 * d * normal[1],
                                x[2] - 2.0 * d * normal[2]};
            double si = 0.0;
            u += axialSpeed(g, mirrored, si);
        }
        Vec3 outwash{};
        if (g.hits) {
            const double z = dot(sub(x, g.hitPoint), g.hitNormal);
            if (z < 0.0) {
                continue;  // behind the surface the wake stops on: under a roof
            }
            // The jet stops at the surface: the axial speed goes to zero there, linearly within
            // a jet radius, as at a stagnation point.
            u *= std::clamp(z / g.arriveRadius, 0.0, 1.0);
            if (onHitFace(g, x)) {
                Vec3 radial = sub(x, g.hitPoint);
                const double off = dot(radial, g.hitNormal);
                for (std::size_t k = 0; k < 3; ++k) {
                    radial[k] -= off * g.hitNormal[k];
                }
                const double rho = norm(radial);
                if (rho > 1e-9) {
                    const double v = outwashSpeed(g.arriveSpeed, g.arriveRadius, rho, z);
                    for (std::size_t k = 0; k < 3; ++k) {
                        outwash[k] = v * radial[k] / rho;
                    }
                }
            }
        }
        for (std::size_t k = 0; k < 3; ++k) {
            out[k] += g.down[k] * u + outwash[k];
        }
    }
    return out;
}

double Downwash::peakOutwash() const {
    double best = 0.0;
    for (const Group& g : groups_) {
        if (!g.hits || g.hitBox >= 0) {
            continue;  // dust lies on the ground, not on roofs
        }
        const double r = g.arriveRadius / kCalRadius;
        best = std::max(best, outwashSpeed(g.arriveSpeed, g.arriveRadius, kOutwashPeakRadius * r,
                                           kOutwashPeakHeight * r));
    }
    return best;
}

std::array<double, 3> windAt(const Wind& wind, double groundZ, const std::array<double, 3>& gust,
                             const Downwash& downwash, const std::array<double, 3>& x,
                             std::size_t exclude) {
    const double from = wind.fromDegrees * std::numbers::pi / 180.0;
    const double mean = meanWindAt(wind, x[2] - groundZ);
    const Vec3 rotors = downwash.velocityAt(x, exclude);
    return {-std::sin(from) * mean + gust[0] + rotors[0],
            -std::cos(from) * mean + gust[1] + rotors[1], gust[2] + rotors[2]};
}

}  // namespace lagrange::aerodynamics
