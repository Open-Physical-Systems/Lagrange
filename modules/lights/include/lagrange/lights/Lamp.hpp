#pragma once

// The lamp model of RFC 0021 §3: what a LampAPI_1 prim holds, its beam's taper, its luminous
// flux, its blinking, and the illuminance it gives a surface. Pure functions, used by the lights
// domain and by whatever measures or draws lamps (the light sensor, the renderer).

#include "lagrange/world/World.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::lights {

enum class LampKind : std::uint8_t { Spot, Point, Beacon };

/// One LampAPI_1 instance with its fallbacks resolved (formats.md §3.14).
struct LampSpec {
    std::string name;  // the prim's signal instance name: lamp/<name>/...
    PrimId prim{};
    BodyId body{kWorldBody};
    Pose poseInBody;  // the beam leaves along +X
    LampKind kind{LampKind::Spot};
    double intensity{0.0};         // cd, on the axis at full command
    double beamAngle{0.0};         // rad, full angle to 50 %
    double fieldAngle{0.0};        // rad, full angle to 10 %, dark beyond
    double colorTemperature{0.0};  // K; 0: color
    std::array<double, 3> color{1.0, 1.0, 1.0};
    double efficacy{0.0};        // lm/W; 0: draws no power and gives no heat
    double heatFraction{1.0};    // 1
    double riseTime{0.0};        // s
    double blinkFrequency{0.0};  // Hz
    double blinkDuty{0.5};       // 1
    double rotationSpeed{0.0};   // rad/s, beacon
    double voltageExponent{0.0};
    double initialCommand{0.0};
    bool load{false};  // ElectricalLoadAPI_1 on the same prim: power_demand to the bus
    std::string bus;   // the load's bus, for its voltage; empty when it cannot be found
};

/// Every LampAPI_1 instance of the world; an error names the first lamp that is wrong.
[[nodiscard]] std::vector<LampSpec> lampsOf(const WorldView& world, std::string& error);

/// The beam's relative intensity at `theta` (rad) off the axis: 1 on it, 0.5 at half the beam
/// angle, 0.1 at half the field angle, 0 beyond; a cosine-squared taper between (RFC 0021 §2).
/// A point lamp is 1 everywhere.
[[nodiscard]] double taper(const LampSpec& lamp, double theta);

/// ∫ taper dΩ over the sphere (sr): the luminous flux is the axis intensity times this.
[[nodiscard]] double effectiveSolidAngle(const LampSpec& lamp);

/// The solid angle of a sharp cone of half angle `alpha`: 2π(1 − cos α) (sr).
[[nodiscard]] double coneSolidAngle(double alpha);

/// ∫_0^upTo f(θ) 2π sin θ dθ by composite Simpson's rule with `intervals` (even) intervals.
template <class F>
[[nodiscard]] double integrateOverCone(F f, double upTo, int intervals);

/// Whether a blinking lamp is on at time `t` (s): a square wave of `frequency` and `duty`,
/// starting on at t = 0. A frequency of 0 is steady on.
[[nodiscard]] bool blinkOn(double t, double frequency, double duty);

/// The beam's axis in the lamp prim's frame at time `t`: +X, a beacon turned about +Z by
/// rotationSpeed · t.
[[nodiscard]] std::array<double, 3> beamAxis(const LampSpec& lamp, double t);

/// The illuminance (lx) the lamp, emitting `intensity` cd on its axis from `lampPose` (world),
/// gives a surface at `point` facing `normal` (unit), at time `t`: E = I·f(θ)·cos(incidence)/d²,
/// 0 when the surface faces away. Unshadowed. `nearLimit` (m) bounds d from below.
[[nodiscard]] double illuminance(const LampSpec& lamp, double intensity, const Pose& lampPose,
                                 const std::array<double, 3>& point,
                                 const std::array<double, 3>& normal, double t,
                                 double nearLimit = 0.01);

template <class F>
double integrateOverCone(F f, double upTo, int intervals) {
    constexpr double kTwoPi = 6.283185307179586;
    if (upTo <= 0.0 || intervals < 2) {
        return 0.0;
    }
    const int n = intervals + (intervals % 2);
    const double h = upTo / n;
    const auto g = [&](double theta) { return f(theta) * std::sin(theta); };
    double sum = g(0.0) + g(upTo);
    for (int i = 1; i < n; ++i) {
        sum += (i % 2 == 1 ? 4.0 : 2.0) * g(i * h);
    }
    return kTwoPi * sum * h / 3.0;
}

}  // namespace lagrange::lights
