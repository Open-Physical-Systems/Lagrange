#pragma once

#include <map>
#include <mutex>
#include <string>

namespace lagrange::ros2_control {

/// Which command a joint follows: the interface its controller claimed.
enum class CommandMode { None, Effort, Velocity, Position };

/// One joint as the LagrangeSystem hardware plugin sees it.
struct JointSlot {
    double position{0.0};  // rad or m
    double velocity{0.0};  // rad/s or m/s
    double effort{0.0};    // N·m or N, the torque applied in the last step
    double commandEffort{0.0};
    double commandVelocity{0.0};
    double commandPosition{0.0};
    CommandMode mode{CommandMode::None};
    /// The EMI domain's emi/<joint>/glitch_probability: per encoder edge (0 without EMI).
    double glitchProbability{0.0};
};

/// The joints shared between the simulation (ControlModule, on the scheduler) and the
/// LagrangeSystem plugin (loaded by the controller manager through pluginlib, in the same
/// process). Both run on the scheduler's thread during read/update/write, so the lock only
/// guards against the controller manager's service thread switching command modes.
class JointTable {
public:
    static JointTable& instance();

    std::mutex& mutex() { return mutex_; }
    /// Joints by name; created on first use.
    JointSlot& operator[](const std::string& joint) { return joints_[joint]; }
    [[nodiscard]] bool contains(const std::string& joint) const { return joints_.contains(joint); }
    void clear() { joints_.clear(); }

private:
    std::mutex mutex_;
    std::map<std::string, JointSlot> joints_;
};

}  // namespace lagrange::ros2_control
