// Rotor downwash and ground outwash (Downwash.hpp; docs/design/domains/rotor_aero.md §2).
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/aerodynamics/Downwash.hpp"

#include <algorithm>
#include <cmath>
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
    }
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
    for (const Group& g : groups_) {
        if (g.id == exclude) {
            continue;
        }
        double s = 0.0;
        double u = axialSpeed(g, x, s);
        Vec3 outwash{};
        if (hasGround_) {
            const double z = dot(sub(x, groundPoint_), groundNormal_);
            if (z < 0.0) {
                continue;  // under the ground
            }
            const double towards = -dot(g.wakeAxis, groundNormal_);
            const double height = dot(sub(g.centre, groundPoint_), groundNormal_);
            if (towards > 1e-3 && height > 0.0) {
                const double sg = height / towards;
                double speed = 0.0;
                double radius = 0.0;
                arriving(g, sg, speed, radius);
                // The jet stops at the ground: the axial speed goes to zero there, linearly
                // within a jet radius, as at a stagnation point.
                u *= std::clamp(z / radius, 0.0, 1.0);
                const Vec3 hit{g.centre[0] + sg * g.wakeAxis[0], g.centre[1] + sg * g.wakeAxis[1],
                               g.centre[2] + sg * g.wakeAxis[2]};
                Vec3 radial = sub(x, hit);
                const double off = dot(radial, groundNormal_);
                for (std::size_t k = 0; k < 3; ++k) {
                    radial[k] -= off * groundNormal_[k];
                }
                const double rho = norm(radial);
                if (rho > 1e-9) {
                    const double v = outwashSpeed(speed, radius, rho, z);
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
    if (!hasGround_) {
        return 0.0;
    }
    double best = 0.0;
    for (const Group& g : groups_) {
        const double towards = -dot(g.wakeAxis, groundNormal_);
        const double height = dot(sub(g.centre, groundPoint_), groundNormal_);
        if (towards <= 1e-3 || height <= 0.0) {
            continue;
        }
        double speed = 0.0;
        double radius = 0.0;
        arriving(g, height / towards, speed, radius);
        const double r = radius / kCalRadius;
        best = std::max(
            best, outwashSpeed(speed, radius, kOutwashPeakRadius * r, kOutwashPeakHeight * r));
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
