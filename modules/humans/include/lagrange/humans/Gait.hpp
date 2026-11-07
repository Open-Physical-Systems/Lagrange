#pragma once

// The walk cycle (RFC 0009 §4). Each stance foot is planted on the ground and the leg solved to
// reach it; the swing foot follows an arc to the next footfall; so a person at a steady speed
// does not skate. It is kinematics: nothing balances.

#include "lagrange/humans/Body.hpp"

#include <array>

namespace lagrange::humans {

inline constexpr double kStanceFraction = 0.62;  // of a stride, in normal walking

/// Step length and rate at a speed. Step length over cadence is nearly constant when walking
/// freely, the walk ratio, about 0.006 to 0.007 m/(step/min) (Sekiya et al. 1996), here 0.0065
/// for 1.75 m and proportional to stature. Below about 1 m/s real people's ratio rises; this
/// stays constant.
struct Stride {
    double stepLength{0.0};  // m
    double cadence{0.0};     // steps per second
};
[[nodiscard]] Stride strideAt(double height, double speed);

/// One instant of the cycle: the pose of the body relative to the ground under the pelvis.
struct GaitSample {
    Posture posture;
    double pelvisHeight{0.0};  // m above the ground
    /// Per leg (0 left, 1 right): x (m, forward of the pelvis) of the ankle the leg was asked
    /// to reach, and whether the foot is on the ground.
    std::array<bool, 2> stance{};
};

/// The cycle at `phase` in [0, 1): left heel strikes at 0, right heel at 0.5. `speed` sets the
/// stride; below kBlendSpeed the pose blends to standing, so a person starts and stops smoothly.
inline constexpr double kBlendSpeed = 0.4;  // m/s
[[nodiscard]] GaitSample gaitAt(const Proportions& body, double speed, double phase);

/// How much of the cycle one metre travelled covers (1 per stride).
[[nodiscard]] double phasePerMetre(double height, double speed);

/// The point of a foot that bears on the ground at this point of its stance, in the foot's frame:
/// the heel at first, the toe at the end. For tests of slip.
[[nodiscard]] std::array<double, 3> contactPoint(const Proportions& body, double legPhase);

}  // namespace lagrange::humans
