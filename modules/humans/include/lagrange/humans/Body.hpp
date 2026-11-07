#pragma once

// The body of a walking person (RFC 0009 §3): 14 rigid segments, kinematic bodies that hang from
// the world and are moved by their pose commands, so sensors see the limbs.

#include "lagrange/world/Model.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::humans {

enum Segment : std::size_t {
    kPelvis,
    kTorso,
    kHead,
    kUpperArmLeft,
    kForearmLeft,
    kUpperArmRight,
    kForearmRight,
    kThighLeft,
    kShinLeft,
    kFootLeft,
    kThighRight,
    kShinRight,
    kFootRight,
    kSegments
};

/// Names as they appear in body names: human_<person>_<segment>.
inline constexpr std::array<const char*, kSegments> kSegmentNames{
    "pelvis",  "torso",  "head",   "upper_arm_l", "forearm_l", "upper_arm_r", "forearm_r",
    "thigh_l", "shin_l", "foot_l", "thigh_r",     "shin_r",    "foot_r"};

/// Segment lengths for a stature H: fractions of H from Drillis and Contini (1966), as tabulated
/// in Winter, Biomechanics and Motor Control of Human Movement. TODO(verify): the fractions are
/// from memory of that table; no source at hand confirmed them.
struct Proportions {
    explicit Proportions(double height);
    double height;
    double hip;           // m, hip joint above the ground, standing (0.530 H)
    double thigh;         // hip to knee (0.530 − 0.285 = 0.245 H)
    double shin;          // knee to ankle (0.285 − 0.039 = 0.246 H)
    double ankle;         // ankle above the sole (0.039 H)
    double foot;          // foot length (0.152 H)
    double heel;          // ankle ahead of the heel (a quarter of the foot, illustrative)
    double hipSide;       // hip joint off the mid line (half of 0.191 H)
    double shoulder;      // shoulder above the hip joint (0.818 − 0.530 H)
    double shoulderSide;  // shoulder off the mid line (half of 0.259 H)
    double upperArm;      // 0.186 H
    double forearm;       // elbow to fingertips (0.146 + 0.108 H)
};

/// Joint angles (rad) and the pelvis. Flexion is positive: thigh and upper arm forward, knee and
/// elbow bent, ankle toe-up.
struct Posture {
    double hipLeft{0.0}, hipRight{0.0};
    double kneeLeft{0.0}, kneeRight{0.0};
    double ankleLeft{0.0}, ankleRight{0.0};
    double shoulderLeft{0.0}, shoulderRight{0.0};
    double elbowLeft{0.0}, elbowRight{0.0};
    double lean{0.0};  // torso pitched forward
};

/// Where every segment's frame is: at its proximal joint, axes along the pelvis's when standing.
/// `pelvis` is the hip joints' mid point, +x the way the person faces.
[[nodiscard]] std::array<Pose, kSegments> segmentPoses(const Proportions& body, const Pose& pelvis,
                                                       const Posture& posture);

/// The pelvis of a person standing with the feet on the ground at `ground` (z the sole), facing
/// `ground`'s heading.
[[nodiscard]] Pose standingPelvis(const Proportions& body, const Pose& ground);

/// A person as a file or the command line asks for one.
struct Spec {
    std::string name;
    double height{1.75};     // m
    double walkSpeed{1.34};  // m/s
    std::string behavior{"stand"};
    std::vector<double> waypoints;  // x, y per waypoint, world ENU
    bool loop{true};
    double dwell{0.0};         // s
    double wanderRadius{5.0};  // m
    std::int64_t seed{0};
    Pose start;  // the ground under the person, world ENU
};

/// The HumanAPI_1 prims of `model`, taken out of its schema instances (the loader expands them
/// with addHuman, after everything else, so that the first bodies stay the robot's).
[[nodiscard]] std::vector<Spec> takeSpecs(Model& model);

/// Adds a person: a top prim /<name>, its segments as kinematic bodies human_<name>_<segment>
/// with capsule colliders and drawn capsules, and a HumanAPI_1 instance on the top prim for the
/// domain to read. A taken name gets a number.
void addHuman(Model& model, Spec spec);

/// The name of a body: human_<person>_<segment>.
[[nodiscard]] std::string segmentBody(const std::string& person, std::size_t segment);

}  // namespace lagrange::humans
