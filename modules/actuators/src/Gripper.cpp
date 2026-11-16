#include "lagrange/actuators/Gripper.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::actuators {

namespace {

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 along(const Vec3& p, const Vec3& d, double s) {
    return {p[0] + s * d[0], p[1] + s * d[1], p[2] + s * d[2]};
}
Vec3 unit(const Vec3& v) {
    const double n = std::sqrt(dot(v, v));
    return n > 0.0 ? Vec3{v[0] / n, v[1] / n, v[2] / n} : Vec3{0.0, 0.0, 1.0};
}

/// How far behind the lip (or the pole face) the rays start, m: behind the surface a lip pressed
/// into a part has sunk below, so that no ray starts inside the part (a ray that does reports
/// where it leaves it). The gripper's own shapes, which the start may be inside, are skipped.
constexpr double kBehind = 0.01;
/// Re-casts past the gripper's own shapes before giving up.
constexpr int kMaxSkips = 8;
/// A step past a shape's far side when re-casting, m.
constexpr double kPast = 1e-6;
/// How far a re-cast looks, m: further than any part is thick.
constexpr double kInsideLook = 1000.0;

}  // namespace

double suctionArea(const SuctionParams& p) {
    return std::numbers::pi * p.diameter * p.diameter / 4.0;
}

double iso6358MassFlow(double conductance, double upstream, double downstream, double temperature) {
    if (!(conductance > 0.0) || !(upstream > downstream) || !(upstream > 0.0)) {
        return 0.0;
    }
    const double ratio = std::max(downstream, 0.0) / upstream;
    double share = 1.0;  // choked
    if (ratio > kCriticalPressureRatio) {
        const double x = (ratio - kCriticalPressureRatio) / (1.0 - kCriticalPressureRatio);
        share = std::sqrt(std::max(0.0, 1.0 - x * x));
    }
    return conductance * upstream * kIso6358ReferenceDensity *
           std::sqrt(kReferenceTemperature / temperature) * share;
}

double evacuationTimeConstant(const SuctionParams& params, double atmosphere, double temperature) {
    return params.volume * (atmosphere - params.ultimatePressure) /
           (kAirGasConstant * temperature * params.flow);
}

double stepSuctionPressure(const SuctionParams& params, double pressure, double dt,
                           const SuctionInputs& in) {
    const double pAtm = in.atmosphere;
    const double span = pAtm - params.ultimatePressure;
    if (!in.sealed && !(params.leakConductance > 0.0)) {
        return pAtm;  // an open cup without a rated leak vents at once
    }
    if (!(span > 0.0) || !(params.volume > 0.0) || !(params.flow > 0.0)) {
        return pAtm;
    }
    const double capacity = params.volume / (kAirGasConstant * in.temperature);  // kg/Pa
    // The generator's line through (p_atm, q0) and (p_min, 0): ṁ_gen = k (p − target), with
    // target p_min while on; off, the cup vents back through it toward the atmosphere.
    const double k = params.flow / span;  // kg/(s·Pa)
    const double target = in.on ? params.ultimatePressure : pAtm;
    // Darcy leak through a porous surface, at the outside air's density.
    const double kLeak =
        in.sealed ? std::max(0.0, in.porousConductance) * pAtm / (kAirGasConstant * in.temperature)
                  : 0.0;
    // Open: ISO 6358 inflow, held over the step (it changes slowly next to the cup's own scale).
    const double inflow =
        in.sealed ? 0.0 : iso6358MassFlow(params.leakConductance, pAtm, pressure, in.temperature);
    const double rate = (k + kLeak) / capacity;  // 1/s
    const double equilibrium = (k * target + kLeak * pAtm + inflow) / (k + kLeak);
    const double next = equilibrium + (pressure - equilibrium) * std::exp(-rate * dt);
    return std::clamp(next, std::min(params.ultimatePressure, pAtm), pAtm);
}

double suctionForce(const SuctionParams& params, double pressure, double atmosphere) {
    return std::max(0.0, atmosphere - pressure) * suctionArea(params);
}

double magnetFlux(const MagnetParams& params, double current, double gap, double partPermeability) {
    if (!(partPermeability > 1.0) || !(params.turns > 0.0)) {
        return 0.0;
    }
    // Half the iron path in the core, half in the part.
    double iron = 0.0;
    if (params.ironPathLength > 0.0) {
        iron = 0.5 * params.ironPathLength / std::max(params.ironPermeability, 1.0) +
               0.5 * params.ironPathLength / partPermeability;
    }
    const double path = 2.0 * std::max(gap, 0.0) + iron;
    if (!(path > 0.0)) {
        return params.saturation > 0.0 ? params.saturation : 0.0;  // no gap and ideal iron
    }
    double b = kMu0 * params.turns * std::abs(current) / path;
    if (params.saturation > 0.0) {
        b = std::min(b, params.saturation);
    }
    return b;
}

double magnetForce(const MagnetParams& params, double current, double gap,
                   double partPermeability) {
    const double b = magnetFlux(params, current, gap, partPermeability);
    return b * b * params.poleArea / kMu0;
}

double stepCoilCurrent(const MagnetParams& params, double current, double voltage, double dt) {
    if (!(params.coilResistance > 0.0)) {
        return 0.0;
    }
    const double steady = voltage / params.coilResistance;
    if (!(params.coilInductance > 0.0)) {
        return steady;
    }
    const double tau = params.coilInductance / params.coilResistance;
    return steady + (current - steady) * std::exp(-dt / tau);
}

services::RayHit castPast(const services::SceneQuery& scene, const Vec3& origin, const Vec3& dir,
                          double reach, BodyId self, std::span<const BodyId> shapeBody) {
    Vec3 from = origin;
    double travelled = 0.0;
    for (int k = 0; k <= kMaxSkips; ++k) {
        // Past one of its own shapes, the ray may start inside a part pressed against it, which
        // it reports where it leaves, however far: look far enough to see that.
        const double look = k == 0 ? reach - travelled : kInsideLook;
        services::RayHit hit = scene.ray(from, dir, look);
        if (!hit.hit()) {
            return hit;
        }
        const bool own = hit.shape < shapeBody.size() && shapeBody[hit.shape] == self;
        if (!own) {
            if (dot(hit.normal, dir) > 0.0) {
                // It started inside this shape (it reports where it leaves it, facing along the
                // ray): what is in front is touching or pressed in, here.
                hit.distance = 0.0;
                hit.normal = {};
            }
            hit.distance += travelled;
            return hit.distance <= reach ? hit : services::RayHit{};
        }
        travelled += hit.distance + kPast;
        from = along(origin, dir, travelled);
        if (travelled >= reach) {
            break;
        }
    }
    return {};
}

Seal findSeal(const services::SceneQuery& scene, const SuctionParams& params, const Vec3& centre,
              const Vec3& axis, BodyId self, std::span<const BodyId> shapeBody) {
    const Vec3 a = unit(axis);
    // Two directions across the axis.
    const Vec3 helper = std::abs(a[0]) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    const Vec3 u = unit(cross(a, helper));
    const Vec3 v = cross(a, u);
    const double radius = params.diameter / 2.0;
    const double cosTilt = std::cos(params.maxTilt);
    Seal seal;
    for (int i = 0; i < kRimRays; ++i) {
        const double angle = 2.0 * std::numbers::pi * i / kRimRays;
        const Vec3 rim =
            along(along(centre, u, radius * std::cos(angle)), v, radius * std::sin(angle));
        const services::RayHit hit =
            castPast(scene, along(rim, a, -kBehind), a, kBehind + params.sealGap, self, shapeBody);
        if (!hit.hit() || hit.distance - kBehind > params.sealGap) {
            return {};
        }
        const BodyId body = hit.shape < shapeBody.size() ? shapeBody[hit.shape] : kWorldBody;
        if (i > 0 && body != seal.body) {
            return {};  // over an edge between two parts
        }
        const double facing = -dot(hit.normal, a);
        if (dot(hit.normal, hit.normal) > 0.0 && facing < cosTilt) {
            return {};  // tilted beyond maxTilt
        }
        seal.body = body;
        if (i == 0) {
            seal.shape = hit.shape;
        }
    }
    seal.sealed = true;
    return seal;
}

Gap findGap(const services::SceneQuery& scene, const Vec3& centre, const Vec3& axis, BodyId self,
            std::span<const BodyId> shapeBody) {
    const Vec3 a = unit(axis);
    const services::RayHit hit =
        castPast(scene, along(centre, a, -kBehind), a, kBehind + kMagnetReach, self, shapeBody);
    if (!hit.hit()) {
        return {};
    }
    return {std::max(0.0, hit.distance - kBehind), hit.shape};
}

}  // namespace lagrange::actuators
