#pragma once

#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <vector>

// MuJoCo's types, so this header does not include MuJoCo.
struct mjModel_;
struct mjData_;

namespace lagrange::rigid_body {

/// The rigid-body domain on MuJoCo 3.12 (docs/design/domains/rigid_body.md, spike 0.2-C1).
///
/// init() builds a MuJoCo model from the world tables (bodies, joints, colliders): no MJCF file
/// and no USD are involved, so the stage stays the only source of the robot (invariant 5).
/// Settings from the validation spike: elliptic friction cone, implicitfast integrator.
/// Bodies of one articulated tree do not collide with each other (Gazebo's self_collide false).
///
/// Every hinge and slide joint `j` outputs joint/j/position and velocity (rad or m). Actuated
/// joints add (prismatic joints use N, m, m/s, m/s², kg):
///   in  joint/j/torque             N·m, WindowAverage
///   in  joint/j/load_torque        N·m, WindowAverage, optional
///   out joint/j/position, velocity, acceleration, effective_inertia, constraint_torque
/// per dynamic body `b`:
///   in  body/b/external_wrench     Wrench N, world_local, at the centre of mass, optional
///   out body/b/pose                Pose m, world_local
///   out body/b/twist               Wrench m/s then rad/s, world_local, at the body origin
///   out body/b/acceleration        Wrench m/s² then rad/s², body frame, proper acceleration
/// and   env/friction_scale         1, optional, default 1 (wet surfaces, from weather).
///
/// Outputs describe the state at the end of each step. Accelerations and constraint torques are
/// those of the step just taken.
class MujocoBackend final : public IModule {
public:
    MujocoBackend();
    ~MujocoBackend() override;
    MujocoBackend(const MujocoBackend&) = delete;
    MujocoBackend& operator=(const MujocoBackend&) = delete;
    MujocoBackend(MujocoBackend&&) = delete;
    MujocoBackend& operator=(MujocoBackend&&) = delete;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;

    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    void shutdown() override;

    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;
    void shiftOrigin(const std::array<double, 3>& shift) override;

    /// The compiled model, for tests and tools; null before init().
    [[nodiscard]] const mjModel_* model() const { return model_; }

    /// Kinematic bodies far from every moving body are left out of MuJoCo's broad phase (on by
    /// default; the contacts are the same either way). For tests: call before init().
    void setKinematicCulling(bool on) { cullingWanted_ = on; }
    /// How many kinematic bodies the last step left out of the broad phase.
    [[nodiscard]] std::size_t culledKinematicBodies() const;

private:
    struct JointSignals {
        std::uint32_t joint{0};
        SignalId torque, loadTorque;
        SignalId position, velocity, acceleration, effectiveInertia, constraintTorque;
        SignalId reaction;
        int mujocoBody{0};  // the body the joint moves
        int qposAddress{0};
        int dofAddress{0};
    };
    struct BodySignals {
        std::uint32_t body{0};
        SignalId externalWrench, pose, twist, acceleration, contactForce;
        SignalId poseCommand;  // kinematic bodies only
        int mujocoBody{0};
        int mocap{-1};  // its mocap index, for a kinematic body
    };

    void writeOutputs(SignalBus& bus) const;
    void sumContactForces();
    void setupKinematicCulling();
    void cullKinematicContacts(bool all);

    // A kinematic (mocap) body far from every moving body cannot touch anything: MuJoCo makes
    // no contacts between bodies without degrees of freedom, but still puts them through its
    // broad phase. Such a body's collision masks are cleared for the step and put back as a
    // moving body comes near (cullKinematicContacts), which leaves the contacts unchanged.
    struct CulledBody {
        int body{0};
        int mocap{0};
        double radius{0.0};  // m, bounding sphere about its origin, margins included
        int contype{0}, conaffinity{0};
        bool culled{false};
    };
    std::vector<CulledBody> culled_;
    std::vector<int> movingGeoms_;  // geoms of bodies with degrees of freedom that collide
    std::vector<int> movingRoots_;  // their tree root, per geom of movingGeoms_
    std::vector<std::array<double, 6>> rootBoxes_;  // scratch: per root, min xyz then max xyz
    std::vector<int> rootIndex_;  // per MuJoCo body: its root's slot in rootBoxes_, or -1
    bool cullable_{false};        // false when a plane or an unbounded geom moves
    bool cullingWanted_{true};
    bool accelerationsDone_{false};  // nv = 0: mj_rnePostConstraint's results never change
    bool carriedBodies_{false};      // a body is fixed to a kinematic (mocap) one

    std::vector<JointSignals> joints_;
    std::vector<BodySignals> bodies_;
    std::vector<double> contactForces_;  // N, world frame, 3 per MuJoCo body, the last step's
    // Kinematic bodies' poses at the previous output, for their twist (MuJoCo's mocap bodies
    // have no velocity of their own).
    mutable std::vector<std::array<double, 7>> lastKinematicPose_;
    // Per body, the twist, contact force and pose last written (6 + 3 + 7; the twist and pose
    // of kinematic bodies only), so an unchanged one is not written again: the bus keeps a value
    // until its writer writes another. In the town most of the 655 kinematic bodies touch
    // nothing, and their poses change only when the traffic or the people move them (every
    // 10 ms); their accelerations (always zero) are written once. Invalid after init, a
    // restored state and a shifted origin, when every value is written.
    static constexpr std::size_t kWrittenPerBody = 16;
    mutable std::vector<double> written_;
    mutable bool writtenValid_{false};
    double dt_{0.001};
    SignalId frictionScale_;
    double appliedFrictionScale_{1.0};
    std::vector<double> baseFriction_;  // per geom, sliding coefficient at scale 1
    mjModel_* model_{nullptr};
    mjData_* data_{nullptr};
};

}  // namespace lagrange::rigid_body
