#pragma once

#include "lagrange/module/IModule.hpp"

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace lagrange::server {

/// What a joint was last told (--joint-control, /joint_commands, Server::commandJoint).
struct JointTarget {
    enum class Mode : std::uint8_t { None, Position, Velocity, Effort, Voltage };
    Mode mode{Mode::None};
    double value{0.0};  // rad or m; rad/s or m/s; N·m or N; V
};

/// The commands waiting for the module, written by ROS's thread or the caller's.
class JointTargets {
public:
    void set(const std::string& joint, JointTarget target);
    void replace(std::map<std::string, JointTarget> targets);
    /// Copies the targets into `out` if they changed since version `seen`.
    void refresh(std::uint64_t& seen, std::map<std::string, JointTarget>& out) const;
    [[nodiscard]] std::map<std::string, JointTarget> all() const;

private:
    mutable std::mutex mutex_;
    std::map<std::string, JointTarget> targets_;
    std::atomic<std::uint64_t> version_{1};
};

/// The simulator at its lowest level: every moving joint driven on its own, by position, velocity
/// or effort, with no robot controller. Writes joint/<j>/torque_command from joint/<j>/position,
/// velocity and effective_inertia. Position and velocity loops are a second-order system of
/// kBandwidthHz, critically damped, on the joint's effective inertia, so they suit a finger and a
/// shoulder alike without tuning; torque is clamped to the joint's effort limit when it has one.
/// A joint not yet commanded gets no torque.
class JointControl final : public IModule {
public:
    /// `driveModes`: joints whose motor follows a velocity or a voltage (electrical::driveModes)
    /// are told that, not a torque. A velocity motor takes position and velocity targets; a voltage
    /// motor takes volts, as a voltage target or in /joint_commands' effort field.
    explicit JointControl(std::shared_ptr<JointTargets> targets,
                          std::map<std::string, std::string> driveModes = {});

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Joint {
        std::string name;
        SignalId position, velocity, inertia, torque;
        double limit{0.0};
        std::string mode{"torque"};  // what `torque` above commands: torque, velocity or voltage
    };
    std::shared_ptr<JointTargets> targets_;
    std::uint64_t seenTargets_{0};
    std::map<std::string, JointTarget> currentTargets_;
    std::map<std::string, std::string> driveModes_;
    std::set<std::string> warned_;  // joints told what their motor cannot follow
    std::vector<Joint> joints_;
};

}  // namespace lagrange::server
