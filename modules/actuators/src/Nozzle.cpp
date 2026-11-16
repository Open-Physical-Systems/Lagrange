// The nozzle's laws (RFC 0022 §8); rules in Nozzle.hpp.
#include "lagrange/actuators/Nozzle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::actuators {

namespace {

double norm(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

/// Where a drag-free point is after `t` seconds.
Vec3 at(const Vec3& x0, const Vec3& v0, const Vec3& g, double t) {
    return {x0[0] + v0[0] * t + 0.5 * g[0] * t * t, x0[1] + v0[1] * t + 0.5 * g[1] * t * t,
            x0[2] + v0[2] * t + 0.5 * g[2] * t * t};
}

}  // namespace

double nozzleArea(const NozzleParams& p) {
    return std::numbers::pi * p.diameter * p.diameter / 4.0;
}

double exitVelocity(double dischargeCoefficient, double pressure, double density) {
    if (!(pressure > 0.0) || !(density > 0.0)) {
        return 0.0;
    }
    return dischargeCoefficient * std::sqrt(2.0 * pressure / density);
}

double nozzleMassFlow(double density, double area, double velocity, double opening) {
    return density * area * velocity * std::clamp(opening, 0.0, 1.0);
}

double pumpPower(double pressure, double volumeFlow, double efficiency) {
    if (!(efficiency > 0.0)) {
        return 0.0;
    }
    return std::max(pressure, 0.0) * std::max(volumeFlow, 0.0) / efficiency;
}

double footprintRadius(double diameter, double sprayHalfAngle, double length) {
    return 0.5 * diameter + std::max(length, 0.0) * std::tan(std::max(sprayHalfAngle, 0.0));
}

double ballisticRange(double speed, double elevation, double drop, double g) {
    const double vx = speed * std::cos(elevation);
    const double vz = speed * std::sin(elevation);
    const double root = vz * vz + 2.0 * g * drop;
    if (!(g > 0.0) || root < 0.0) {
        return 0.0;
    }
    return vx * (vz + std::sqrt(root)) / g;
}

JetPath traceJet(const Vec3& exit, const Vec3& velocity, const Vec3& gravity,
                 const services::SceneQuery* scene, const std::function<bool(std::uint32_t)>& skip,
                 const JetLimits& limits) {
    JetPath path;
    path.points.push_back(exit);
    if (!(norm(velocity) > 0.0)) {
        return path;
    }
    constexpr double kStepPast = 1e-5;  // m: past the face of a shape passed through
    constexpr int kMostSkipped = 16;
    double t = 0.0;
    Vec3 from = exit;
    for (std::size_t i = 0; i < limits.maxSegments && path.length < limits.maxLength; ++i) {
        const Vec3 v{velocity[0] + gravity[0] * t, velocity[1] + gravity[1] * t,
                     velocity[2] + gravity[2] * t};
        const double dt = limits.segment / std::max(norm(v), 1e-6);
        const Vec3 to = at(exit, velocity, gravity, t + dt);
        const Vec3 d{to[0] - from[0], to[1] - from[1], to[2] - from[2]};
        const double chord = norm(d);
        if (scene != nullptr && chord > 0.0) {
            const Vec3 dir{d[0] / chord, d[1] / chord, d[2] / chord};
            double travelled = 0.0;
            for (int k = 0; k < kMostSkipped && travelled < chord; ++k) {
                const Vec3 origin{from[0] + dir[0] * travelled, from[1] + dir[1] * travelled,
                                  from[2] + dir[2] * travelled};
                const services::RayHit hit = scene->ray(origin, dir, chord - travelled);
                if (!hit.hit()) {
                    break;
                }
                if (skip && skip(hit.shape)) {
                    travelled += hit.distance + kStepPast;
                    continue;
                }
                const double s = travelled + hit.distance;
                path.landed = true;
                path.shape = hit.shape;
                path.impact = {from[0] + dir[0] * s, from[1] + dir[1] * s, from[2] + dir[2] * s};
                path.length += s;
                path.time = t + dt * (s / chord);
                path.points.push_back(path.impact);
                return path;
            }
        }
        path.length += chord;
        t += dt;
        path.time = t;
        path.points.push_back(to);
        from = to;
    }
    return path;
}

}  // namespace lagrange::actuators
