#pragma once

// The air rotors blow over the ground, as the realistic renderer's grass and vegetation feel it
// (realistic.slang's groundAir): the C++ twin of the shader, and where its rotor discs become
// ground jets. A port of the ground outwash of aerodynamics::Downwash (Downwash.hpp, region 3;
// docs/design/domains/rotor_aero.md §2) with the ground a level plane and no wake skew, so the
// renderer (and the viewer, which never links simulation modules, invariant 14) draws the same
// outwash the aerodynamics domain pushes bodies with:
//
// - per group (one robot's rotors), the wake leaves the mean hub along −(sum of thrust axes) and
//   arrives at the ground with the near wake's speed v_i·(1 + s/√(s² + R²)) (Leishman,
//   Principles of Helicopter Aerodynamics, 2nd ed., 2006, §2.7) blended into the merged jet of
//   Bauersfeld et al. (2024), arXiv:2403.13321, eqs. 3, 5, 6, 9, between 1.5 l and 2.5 l;
// - the outwash there follows the fit to Tanner, Overmeyer, Jenkins, Yao and Bartram (2015),
//   "Experimental investigation of rotorcraft outwash in ground effect", AHS Forum 71
//   (NTRS 20160006428): peak 1.9 V_h at 1.7 R, 0.05 R above the ground, a layer 0.15 R deep.
//
// The constants are Downwash.hpp's, repeated here because lagrange_gpu sits below the modules;
// test_downwash checks this twin against Downwash::velocityAt.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <numbers>
#include <span>
#include <vector>

namespace lagrange::gpu {

/// A rotor disc as the renderer takes it: what rotor/<j>/thrust, induced_velocity, thrust_axis
/// and the rotor link's pose give.
struct AirRotor {
    std::array<double, 3> hub{};         // m, world
    std::array<double, 3> thrustAxis{};  // unit, world
    double radius{0.0};                  // m
    double induced{0.0};                 // m/s, momentum-theory induced velocity at the disc
    double thrust{0.0};                  // N; no wake at zero or reverse thrust
    std::size_t group{0};                // the robot it belongs to
};

/// Where one group's wake meets the ground, and the jet arriving there.
struct AirJet {
    std::array<double, 3> hit{};  // m, world, on the ground
    double speed{0.0};            // m/s, arriving centreline speed
    double radius{0.0};           // m, arriving top-hat radius
};

/// Ground jets the shader takes per frame at most (realistic.slang's kAirJets).
inline constexpr std::size_t kMaxAirJets = 8;

namespace groundair {
// Downwash.hpp's constants (sources there).
inline constexpr double kJetDecay = 10.11;
inline constexpr double kJetOrigin = -5.817;
inline constexpr double kJetSpread = 0.07668;
inline constexpr double kJetMergeStart = 1.5;
inline constexpr double kJetMergeEnd = 2.5;
inline constexpr double kOutwashPeakSpeed = 1.9;
inline constexpr double kOutwashPeakRadius = 1.7;
inline constexpr double kOutwashPeakHeight = 0.05;
inline constexpr double kOutwashLayer = 0.15;
inline constexpr double kOutwashRise = 0.8;
inline constexpr double kOutwashDecay = 1.1;
inline constexpr double kOutwashGrowth = 0.073;
inline const double kCalArriving = 1.0 + 1.0 / std::numbers::sqrt2;
inline const double kCalRadius = 1.0 / std::sqrt(kCalArriving);
inline const double kFarTopHat = 1.0 / std::sqrt(3.0 * (std::numbers::sqrt2 - 1.0));
}  // namespace groundair

/// Outwash speed (m/s) at radius `rho` from the impingement point and height `z` above the
/// ground, for a jet arriving at `arriving` m/s with top-hat radius `jetRadius` (Downwash.hpp's
/// outwashSpeed).
[[nodiscard]] inline double outwashProfile(double arriving, double jetRadius, double rho,
                                           double z) {
    using namespace groundair;
    if (arriving <= 0.0 || jetRadius <= 0.0 || rho <= 0.0 || z < 0.0) {
        return 0.0;
    }
    const double r = jetRadius / kCalRadius;
    const double peakRadius = kOutwashPeakRadius * r;
    const double peak = arriving * kOutwashPeakSpeed / kCalArriving;
    const double radial = rho < peakRadius ? std::pow(rho / peakRadius, kOutwashRise)
                                           : std::pow(peakRadius / rho, kOutwashDecay);
    const double zPeak = kOutwashPeakHeight * r;
    const double layer = kOutwashLayer * r + kOutwashGrowth * std::max(0.0, rho - peakRadius);
    double vertical = 0.0;
    if (z <= zPeak) {
        vertical = std::pow(z / zPeak, 1.0 / 7.0);
    } else {
        const double eta = (z - zPeak) / (layer - zPeak);
        vertical = std::exp(-std::numbers::ln2 * eta * eta);
    }
    return peak * radial * vertical;
}

/// The wall jet's peak height (m) for a jet arriving with top-hat radius `jetRadius`.
[[nodiscard]] inline double outwashPeakHeight(double jetRadius) {
    return groundair::kOutwashPeakHeight * jetRadius / groundair::kCalRadius;
}

/// One ground jet per group of rotors above the level ground at `groundZ` whose wake reaches it
/// (Downwash::update and findSurfaces, without skew or obstacles), at most kMaxAirJets.
/// `groundUnder(centre)` gives each group the height of the ground under its mean hub.
[[nodiscard]] inline std::vector<AirJet> groundJets(
    std::span<const AirRotor> rotors,
    const std::function<double(const std::array<double, 3>&)>& groundUnder) {
    using namespace groundair;
    std::map<std::size_t, std::vector<AirRotor>> groups;
    for (const AirRotor& r : rotors) {
        if (r.thrust > 0.0 && r.induced > 0.0 && r.radius > 0.0) {
            groups[r.group].push_back(r);
        }
    }
    std::vector<AirJet> jets;
    for (const auto& [id, g] : groups) {
        if (jets.size() == kMaxAirJets) {
            break;
        }
        const auto n = static_cast<double>(g.size());
        std::array<double, 3> centre{}, thrust{};
        double v2 = 0.0;
        for (const AirRotor& r : g) {
            for (std::size_t k = 0; k < 3; ++k) {
                centre[k] += r.hub[k] / n;
                thrust[k] += r.thrustAxis[k];
            }
            v2 += r.induced * r.induced / n;
        }
        const double t = std::hypot(thrust[0], thrust[1], thrust[2]);
        if (t <= 0.0) {
            continue;
        }
        const std::array<double, 3> down{-thrust[0] / t, -thrust[1] / t, -thrust[2] / t};
        const double hoverInduced = std::sqrt(v2);
        double length = 0.0;
        if (g.size() >= 2) {
            for (const AirRotor& r : g) {
                length += 2.0 *
                          std::hypot(r.hub[0] - centre[0], r.hub[1] - centre[1],
                                     r.hub[2] - centre[2]) /
                          n;
            }
        }
        if (length <= 0.0) {
            length = 2.0 * g.front().radius;
        }
        const double groundZ = groundUnder(centre);
        const double towards = -down[2];  // the wake's share towards the ground (normal +z)
        const double height = centre[2] - groundZ;
        if (!(towards > 1e-3) || height <= 0.0) {
            continue;  // the wake does not go down to the ground
        }
        const double s = height / towards;
        double near = 0.0;
        double area = 0.0;
        for (const AirRotor& r : g) {
            near += r.induced * (1.0 + s / std::sqrt(s * s + r.radius * r.radius));
            area += r.radius * r.radius;
        }
        near /= n;
        const double nearRadius = std::sqrt(area) * std::sqrt(hoverInduced / std::max(near, 1e-9));
        const double beta =
            std::clamp((s / length - kJetMergeStart) / (kJetMergeEnd - kJetMergeStart), 0.0, 1.0);
        const double far = hoverInduced * kJetDecay / (s / length - kJetOrigin);
        const double farRadius = kFarTopHat * kJetSpread * (s - kJetOrigin * length);
        AirJet jet;
        jet.hit = {centre[0] + s * down[0], centre[1] + s * down[1], groundZ};
        jet.speed = (1.0 - beta) * near + beta * far;
        jet.radius = (1.0 - beta) * nearRadius + beta * farRadius;
        jets.push_back(jet);
    }
    return jets;
}

[[nodiscard]] inline std::vector<AirJet> groundJets(std::span<const AirRotor> rotors,
                                                    double groundZ) {
    return groundJets(rotors, [groundZ](const std::array<double, 3>&) { return groundZ; });
}

/// How far below a jet's surface vegetation still feels its outwash (m), a drawing choice.
inline constexpr double kBelowSurface = 0.5;

/// The jets' horizontal outwash at `x` (m/s, world x and y), as realistic.slang's groundAir adds
/// it to the site wind, without its drawn ripple. Below a jet's peak height the blades are taken
/// to stand in the jet's peak (they are taller than the wall layer), so the speed there is the
/// peak's.
[[nodiscard]] inline std::array<double, 2> groundOutwash(std::span<const AirJet> jets,
                                                         const std::array<double, 3>& x) {
    std::array<double, 2> out{};
    for (const AirJet& j : jets) {
        const double dx = x[0] - j.hit[0];
        const double dy = x[1] - j.hit[1];
        const double rho = std::hypot(dx, dy);
        const double z = x[2] - j.hit[2];
        // Vegetation rooted a little below the surface the jet meets (a lower bank beside it)
        // still feels it; well below is under a roof.
        if (rho <= 1e-9 || z < -kBelowSurface) {
            continue;
        }
        const double v =
            outwashProfile(j.speed, j.radius, rho, std::max(z, outwashPeakHeight(j.radius)));
        out[0] += v * dx / rho;
        out[1] += v * dy / rho;
    }
    return out;
}

}  // namespace lagrange::gpu
