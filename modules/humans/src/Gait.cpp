#include "lagrange/humans/Gait.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::humans {

namespace {

constexpr double kTau = 2.0 * std::numbers::pi;
constexpr double kWalkRatio = 0.0065;      // m/(step/min), for 1.75 m (Sekiya et al. 1996)
constexpr double kHeelRockEnd = 0.12;      // of stance: the foot is flat from here
constexpr double kToeRockStart = 0.50;     // of stance: the heel rises from here
constexpr double kHeelStrikeAngle = 0.30;  // rad, toe up at heel strike (illustrative)
constexpr double kToeOffAngle = 1.10;      // rad, toe down at toe-off (illustrative)
constexpr double kPelvisHeight = 0.517;    // of stature, mean height while walking (illustrative)
constexpr double kPelvisBob = 0.0143;      // of stature, up and down, twice a stride (illustrative)
constexpr double kStanceBehind = 0.055;    // of stature: the stance centred behind the hip
constexpr double kSwingLift = 0.018;       // of stature: the arc above a straight swing
constexpr double kArmSwing = 0.32;         // rad, shoulder either way (illustrative)
constexpr double kElbowBent = 0.35;        // rad (illustrative)
constexpr double kLean = 0.035;            // rad forward (illustrative)

double smooth(double u) {
    return u * u * (3.0 - 2.0 * u);
}

/// The foot in stance: its toe-up angle, and the ankle relative to the heel's footfall.
struct FootOnGround {
    double toeUp;
    double x, z;
};

FootOnGround stanceFoot(const Proportions& b, double u) {
    const double heel = b.heel;
    const double a = b.ankle;
    if (u < kHeelRockEnd) {  // rolling on the heel
        const double angle = kHeelStrikeAngle * (1.0 - u / kHeelRockEnd);
        return {angle, heel * std::cos(angle) - a * std::sin(angle),
                heel * std::sin(angle) + a * std::cos(angle)};
    }
    if (u < kToeRockStart) {  // flat
        return {0.0, heel, a};
    }
    // Rolling on the toe, which stays at b.foot from the heel's footfall.
    const double v = (u - kToeRockStart) / (1.0 - kToeRockStart);
    const double angle = kToeOffAngle * v * v;
    const double back = b.foot - heel;
    return {-angle, b.foot - back * std::cos(angle) + a * std::sin(angle),
            back * std::sin(angle) + a * std::cos(angle)};
}

struct LegAngles {
    double hip, knee, ankle;
};

/// Hip and knee that put the ankle at (x, z) from the hip (z < 0), by the law of cosines, the
/// knee forward. An ankle out of reach is met with the leg straight, pointing at it.
LegAngles solveLeg(const Proportions& b, double x, double z, double toeUp) {
    const double t = b.thigh;
    const double s = b.shin;
    const double d = std::clamp(std::hypot(x, z), std::abs(t - s) + 1e-6, t + s - 1e-6);
    const double knee = std::numbers::pi - std::acos((t * t + s * s - d * d) / (2.0 * t * s));
    const double hip = std::atan2(x, -z) + std::acos((t * t + d * d - s * s) / (2.0 * t * d));
    return {hip, knee, toeUp - hip + knee};
}

LegAngles leg(const Proportions& b, const Stride& stride, double phase, double hipHeight) {
    const double strideLength = 2.0 * stride.stepLength;
    const double behind = kStanceBehind * b.height;
    // The heel's footfall relative to the hip, from when the foot lands (phase 0) on.
    const auto footfall = [&](double p) {
        return strideLength * (kStanceFraction / 2.0 - p) - behind - b.heel;
    };
    if (phase < kStanceFraction) {
        const FootOnGround foot = stanceFoot(b, phase / kStanceFraction);
        return solveLeg(b, footfall(phase) + foot.x, foot.z - hipHeight, foot.toeUp);
    }
    // Swing: from toe-off to the next heel strike, measured from the hip, which moves meanwhile.
    const FootOnGround off = stanceFoot(b, 1.0);
    const FootOnGround on = stanceFoot(b, 0.0);
    const double u = (phase - kStanceFraction) / (1.0 - kStanceFraction);
    const double blend = smooth(u);
    const double x0 = footfall(kStanceFraction) + off.x;
    const double x1 = footfall(0.0) + on.x;
    const double x = x0 + (x1 - x0) * blend;
    const double z =
        off.z + (on.z - off.z) * blend + kSwingLift * b.height * std::sin(u * std::numbers::pi);
    return solveLeg(b, x, z - hipHeight, off.toeUp + (on.toeUp - off.toeUp) * blend);
}

}  // namespace

Stride strideAt(double height, double speed) {
    const double ratio = kWalkRatio * height / 1.75;  // m per step/min
    // Step length L and rate f (steps/s): L = ratio · 60 f and speed = L f.
    const double length = std::sqrt(60.0 * ratio * std::max(speed, 0.05));
    return {length, std::max(speed, 0.05) / length};
}

double phasePerMetre(double height, double speed) {
    return 1.0 / (2.0 * strideAt(height, speed).stepLength);
}

GaitSample gaitAt(const Proportions& b, double speed, double phase) {
    const Stride stride = strideAt(b.height, std::max(speed, kBlendSpeed));
    const double left = phase - std::floor(phase);
    const double right = left + 0.5 - std::floor(left + 0.5);
    const double blend = smooth(std::clamp(speed / kBlendSpeed, 0.0, 1.0));
    // A longer stride needs a lower pelvis to reach both feet (illustrative).
    const double crouch = 0.03 * std::max(speed - 1.34, 0.0);
    const double hipHeight =
        b.height * (kPelvisHeight - crouch - kPelvisBob * std::cos(2.0 * kTau * left));

    GaitSample out;
    const LegAngles l = leg(b, stride, left, hipHeight);
    const LegAngles r = leg(b, stride, right, hipHeight);
    out.stance = {left < kStanceFraction, right < kStanceFraction};
    out.pelvisHeight = b.hip + (hipHeight - b.hip) * blend;
    out.posture.hipLeft = l.hip * blend;
    out.posture.kneeLeft = l.knee * blend;
    out.posture.ankleLeft = l.ankle * blend;
    out.posture.hipRight = r.hip * blend;
    out.posture.kneeRight = r.knee * blend;
    out.posture.ankleRight = r.ankle * blend;
    // Arms swing with the opposite leg.
    const double swing = kArmSwing * std::cos(kTau * left);
    out.posture.shoulderLeft = -swing * blend;
    out.posture.shoulderRight = swing * blend;
    out.posture.elbowLeft = (kElbowBent + 0.6 * std::max(-swing, 0.0)) * blend;
    out.posture.elbowRight = (kElbowBent + 0.6 * std::max(swing, 0.0)) * blend;
    out.posture.lean = kLean * blend;
    return out;
}

std::array<double, 3> contactPoint(const Proportions& b, double legPhase) {
    const double u = legPhase / kStanceFraction;
    if (u < kHeelRockEnd) {
        return {-b.heel, 0.0, -b.ankle};
    }
    if (u < kToeRockStart) {
        return {0.0, 0.0, -b.ankle};
    }
    return {b.foot - b.heel, 0.0, -b.ankle};
}

}  // namespace lagrange::humans
