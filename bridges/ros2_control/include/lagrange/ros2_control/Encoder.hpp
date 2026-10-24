#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

namespace lagrange::ros2_control {

/// An incremental encoder on a revolute joint (roadmap 0.2-D2), as a joint's
/// `<param name="encoder_resolution">` in the URDF's `<ros2_control>` block sets it: counts per
/// revolution, after quadrature decoding (a 4096-line encoder gives 16384).
///
/// The position is a whole number of counts, rounded down as a counter is. The velocity is what
/// a driver computes from it: the change of count over the control period, so it carries the
/// quantisation of one count per period (2π / (resolution · period) rad/s).
class Encoder {
public:
    explicit Encoder(double countsPerRevolution)
        : step_(2.0 * std::numbers::pi / countsPerRevolution) {}

    /// Counts for the true joint angle (rad).
    [[nodiscard]] std::int64_t count(double angleRad) const {
        return static_cast<std::int64_t>(std::floor(angleRad / step_));
    }

    /// The angle the counter reports (rad).
    [[nodiscard]] double position(double angleRad) const {
        return static_cast<double>(count(angleRad)) * step_;
    }

    /// The velocity (rad/s) from the count change since the previous call, `periodS` seconds
    /// ago. Zero on the first call or a non-positive period.
    [[nodiscard]] double velocity(double angleRad, double periodS) {
        const std::int64_t now = count(angleRad);
        const bool first = !started_;
        const std::int64_t before = last_;
        last_ = now;
        started_ = true;
        if (first || !(periodS > 0.0)) {
            return 0.0;
        }
        return static_cast<double>(now - before) * step_ / periodS;
    }

    [[nodiscard]] double step() const { return step_; }

private:
    double step_;  // rad per count
    std::int64_t last_{0};
    bool started_{false};
};

}  // namespace lagrange::ros2_control
