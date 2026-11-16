#pragma once

// The nozzle and tank of RFC 0022 §8, free of the bus so that tests check them against their
// closed forms: the exit velocity and flow of a nozzle behind a pump, the jet's reaction, and the
// jet's ballistic, drag-free path until it meets a collider. SI units; pressures are gauge.

#include "lagrange/services/SceneQuery.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace lagrange::actuators {

using Vec3 = std::array<double, 3>;

/// Density of liquid water at 20 °C, kg/m³ (NIST Chemistry WebBook, saturated water: 998.21):
/// a hose's water, and TankAPI_1's fallback.
inline constexpr double kWaterDensity = 998.2;

/// What a nozzle sprays (act:nozzle:agent).
enum class Agent : std::uint8_t { Water, Foam, Co2 };

/// NozzleAPI_1's attributes.
struct NozzleParams {
    double diameter{0.0};              // m, the exit's bore
    double dischargeCoefficient{0.0};  // 1
    double pressure{0.0};              // Pa gauge, the pump's at the nozzle
    Agent agent{Agent::Water};
    double foamExpansion{0.0};   // 1
    double sprayHalfAngle{0.0};  // rad
    double pumpEfficiency{0.0};  // 1; 0: no pump power
    Vec3 axis{1.0, 0.0, 0.0};    // in the prim's frame, unit
};

/// The exit's area π d² / 4, m².
[[nodiscard]] double nozzleArea(const NozzleParams& p);
/// Exit velocity of a nozzle behind `pressure` (Pa gauge) of a liquid of `density` (kg/m³):
/// Bernoulli with the discharge coefficient, C_d √(2 Δp / ρ), m/s. 0 for no pressure.
[[nodiscard]] double exitVelocity(double dischargeCoefficient, double pressure, double density);
/// Mass flow ρ A v0 times the valve's opening (0 to 1), kg/s.
[[nodiscard]] double nozzleMassFlow(double density, double area, double velocity, double opening);
/// The pump's electrical power Δp Q / η (W) for a volume flow `volumeFlow` (m³/s); 0 when the
/// efficiency is not over 0.
[[nodiscard]] double pumpPower(double pressure, double volumeFlow, double efficiency);
/// Radius of the jet's landing disc after `length` (m) of path: the bore's radius widened by the
/// spray cone, d/2 + L tan(half angle), m.
[[nodiscard]] double footprintRadius(double diameter, double sprayHalfAngle, double length);

/// The closed form the path is checked against: how far (horizontally, m) a drag-free projectile
/// leaving at `speed` (m/s) and `elevation` (rad above the horizontal) lands on level ground
/// `drop` (m) below its start, under gravity `g` (m/s²):
/// R = v cos θ (v sin θ + √(v² sin² θ + 2 g h)) / g.
[[nodiscard]] double ballisticRange(double speed, double elevation, double drop, double g);

/// The jet from a nozzle, traced through the scene.
struct JetPath {
    std::vector<Vec3> points;  // from the exit to where it ends, m (world-local)
    bool landed{false};        // it met a collider
    Vec3 impact{};             // where it met it, m
    std::uint32_t shape{services::RayHit::kNone};  // the shape it met
    double length{0.0};                            // m along the path
    double time{0.0};                              // s of flight
};

/// Limits of a trace.
struct JetLimits {
    double segment{0.05};     // m: the path is cast as chords about this long
    double maxLength{200.0};  // m of path traced at most
    std::size_t maxSegments{2048};
};

/// Traces a drag-free jet leaving `exit` with `velocity` (m/s) under `gravity` (m/s², a vector)
/// through `scene` (nullptr: nothing to meet), as chords cast as rays; a shape for which `skip`
/// returns true (the robot's own) is passed through. Deterministic.
[[nodiscard]] JetPath traceJet(const Vec3& exit, const Vec3& velocity, const Vec3& gravity,
                               const services::SceneQuery* scene,
                               const std::function<bool(std::uint32_t)>& skip,
                               const JetLimits& limits = {});

}  // namespace lagrange::actuators
