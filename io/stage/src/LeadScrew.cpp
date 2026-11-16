// A power screw's gear ratio and efficiencies (RFC 0020 §4.4), for the overlay's lead_screw:.
#include "lagrange/stage/Stage.hpp"

#include <cmath>
#include <numbers>
#include <string>

namespace lagrange::stage {

Expected<ScrewGear, std::string> leadScrewGear(const LeadScrew& screw) {
    if (!(screw.lead > 0.0) || !(screw.diameter > 0.0) || !std::isfinite(screw.lead) ||
        !std::isfinite(screw.diameter)) {
        return Unexpected{std::string("lead_screw: lead and diameter must be positive")};
    }
    if (!(screw.friction >= 0.0) || !std::isfinite(screw.friction)) {
        return Unexpected{std::string("lead_screw: friction must be 0 or more")};
    }
    if (!(screw.threadAngle >= 0.0 && screw.threadAngle < std::numbers::pi / 2.0)) {
        return Unexpected{
            std::string("lead_screw: thread_angle is the flank's half-angle in rad, 0 to < π/2")};
    }
    ScrewGear out;
    out.gearRatio = 2.0 * std::numbers::pi / screw.lead;
    const double lead = std::atan(screw.lead / (std::numbers::pi * screw.diameter));
    // The flank's angle measured in the plane normal to the thread (Shigley ch. 8).
    const double normal = std::atan(std::tan(screw.threadAngle) * std::cos(lead));
    const double friction = std::atan(screw.friction / std::cos(normal));
    if (lead + friction >= std::numbers::pi / 2.0) {
        return Unexpected{std::string("lead_screw: the friction is too large to drive the screw")};
    }
    out.leadAngle = lead;
    out.frictionAngle = friction;
    out.efficiency = std::tan(lead) / std::tan(lead + friction);
    out.backEfficiency = friction >= lead ? 0.0 : std::tan(lead - friction) / std::tan(lead);
    return out;
}

}  // namespace lagrange::stage
