#include "lagrange/tutorial/Odometer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lagrange::tutorial {

using namespace literals;

Odometer::Odometer(Settings settings) : settings_(std::move(settings)) {}

ModuleInfo Odometer::info() const {
    // Domain "sensors", name "odometer", version 0.1, not a substitute, deterministic, Tier 0.
    return {"sensors", "odometer", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Odometer::preferredStep() const {
    return 10_ms;  // a 100 Hz encoder readout
}

Status Odometer::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& declarer) {
    // Fail early, with a reason, when the robot has no such joints.
    const auto& joints = world.joints().names;
    for (const std::string& name : {settings_.leftJoint, settings_.rightJoint}) {
        if (std::find(joints.begin(), joints.end(), name) == joints.end()) {
            return Status::error(StatusCode::InvalidConfig, "odometer: no joint " + name);
        }
    }
    const auto input = [&](const std::string& joint) {
        SignalSpec spec;
        spec.name = "joint/" + joint + "/velocity";
        spec.unit = units::rad_per_s;  // checked against the writer's unit at wiring
        // Physics writes every 1 ms, the odometer reads every 10 ms. WindowAverage gives the
        // mean of the values committed since the odometer's previous step, so the integral
        // below misses nothing in between (ZeroOrder would take only the latest value).
        spec.hold = Hold::WindowAverage;
        return declarer.input(spec);
    };
    left_ = input(settings_.leftJoint);
    right_ = input(settings_.rightJoint);
    SignalSpec distance;
    distance.name = "base/odometer/distance";
    distance.unit = units::m;
    distance_ = declarer.output(distance);
    SignalSpec heading;
    heading.name = "base/odometer/heading";
    heading.unit = units::rad;
    heading_ = declarer.output(heading);
    return {};
}

Status Odometer::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    distanceM_ = 0.0;
    headingRad_ = 0.0;
    bus.write(distance_, distanceM_);
    bus.write(heading_, headingRad_);
    return {};
}

Status Odometer::step(const StepContext& context, SignalBus& bus) {
    // Wheel rim speeds (m/s) from the wheel speeds (rad/s).
    const double left = bus.read(left_) * settings_.wheelRadiusM;
    const double right = bus.read(right_) * settings_.wheelRadiusM;
    const double dt = context.dt.seconds();
    distanceM_ += std::abs(left + right) / 2.0 * dt;
    headingRad_ += (right - left) / settings_.trackWidthM * dt;
    bus.write(distance_, distanceM_);
    bus.write(heading_, headingRad_);
    return {};
}

// The whole dynamic state, so that a saved run restores bit for bit (determinism).
Status Odometer::saveState(StateWriter& writer) const {
    writer.put(distanceM_);
    writer.put(headingRad_);
    return {};
}

Status Odometer::restoreState(StateReader& reader) {
    reader.get(distanceM_);
    reader.get(headingRad_);
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "short odometer state")
                           : Status{};
}

}  // namespace lagrange::tutorial
