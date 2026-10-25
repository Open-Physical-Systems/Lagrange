#pragma once

// The vehicle dynamics domain (docs/design/domains/vehicle_dynamics.md): suspension, tyres,
// brakes, powertrain and steering of wheeled road vehicles, on top of the rigid-body backend.

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/services/SceneQuery.hpp"
#include "lagrange/vehicle_dynamics/Tyre.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::vehicle_dynamics {

using Vec3 = std::array<double, 3>;

/// One wheel of a vehicle, read once at load (WheelAPI_1 on its spin joint).
struct Wheel {
    std::string name;  // the spin joint's signal instance name
    std::uint32_t joint{0};
    std::optional<std::uint32_t> steerJoint;
    Vec3 anchor{};          // wheel centre at rest, chassis frame (m)
    double spin{1.0};       // +1 when a positive joint velocity rolls the wheel forward
    double steerSign{1.0};  // +1 when a positive steering joint angle turns the wheel left
    double radius{0.0}, width{0.2};
    bool front{true}, steered{false}, driven{false};
    std::optional<MagicFormula> tyre;
    double longitudinalStiffness{0.0}, corneringStiffness{0.0};
    double friction{1.0}, relaxationLength{0.3};
    double restLength{0.0}, travel{0.0}, springStiffness{0.0}, damping{0.0};
    double steerInertia{0.0};  // kg·m², what the steering joint turns
    double spinInertia{0.0};   // kg·m², the wheel about its spin axis
    double preload{0.0};       // N: the spring's force at ride height (its static load)
};

/// One VehicleAPI_1 chassis with its wheels.
struct Vehicle {
    std::string name;  // the robot's name: vehicle/<name>/...
    std::uint32_t chassis{0};
    bool engine{false};  // ice_map; else electric
    double maxDriveTorque{0.0};
    double maxSteerAngle{0.0}, steerRate{1.0}, steerTimeConstant{0.05};
    double maxBrakeTorque{0.0}, brakeBias{0.6};
    bool abs{false};
    std::vector<double> engineSpeeds, engineTorques, gearRatios;
    double idleSpeed{80.0}, finalDrive{1.0}, shiftUp{600.0}, shiftDown{250.0}, efficiency{0.92};
    double antiRollFront{0.0}, antiRollRear{0.0};
    double wheelbase{0.0};  // m, between the axles' centres
    double mass{0.0};       // kg, the whole vehicle
    std::vector<Wheel> wheels;
};

/// The vehicles of `world`; an error for a missing or invalid required value.
[[nodiscard]] Expected<std::vector<Vehicle>, std::string> vehiclesOf(const WorldView& world);

/// Ackermann geometry: the road-wheel angle of a wheel `y` metres left of the centre line, for
/// a bicycle-model angle `delta` (rad, positive left) and wheelbase `wheelbase`.
[[nodiscard]] double ackermannAngle(double delta, double wheelbase, double y);

/// Engine torque at full throttle at `speed` (rad/s), interpolated in the map; 0 outside it.
[[nodiscard]] double engineTorque(const Vehicle& v, double speed);

/// The vehicle dynamics domain at 1 ms (vehicle_dynamics.md §4).
///
/// Per vehicle v it reads vehicle/<v>/throttle (−1…1; negative drives backwards), brake (0…1)
/// and steering_angle (rad, the bicycle model's road-wheel angle, positive left), all optional
/// (zero), the chassis's pose and twist, and each wheel joint's velocity; it writes
/// body/<chassis>/wrench/vehicle (suspension and tyre forces at the chassis's centre of mass),
/// joint/<j>/load_torque (the tyre's reaction and the brake), joint/<j>/torque_command for
/// driven wheels (the powertrain) and steering joints (a PD on the Ackermann angle),
/// vehicle/<v>/speed, engine_speed and gear, and wheel/<j>/slip_ratio, slip_angle,
/// normal_force and steer_angle.
///
/// The ground is found by a ray down from each wheel's suspension mount through `ground` (the
/// fixed scenery); `groundFriction` is each shape's friction coefficient (the tyre's μ is on a
/// surface of 1), times env/friction_scale (optional). The ground is taken as level at each
/// contact (forces in the horizontal plane): a ray gives no normal.
///
/// Slip: relaxation-length states for longitudinal and lateral slip (Pacejka's first-order
/// model), integrated exactly over the step, so the tyre behaves at walking speed and at rest.
/// Config: "tire_model" pacejka (default; needs each wheel's tyre file) or linear.
///
/// With `ideal` it is the ideal substitute: the suspension holds the chassis up as usual, and
/// the planar motion follows a kinematic bicycle model (speed from throttle and brake, yaw rate
/// v·tan δ / L), enforced by a wrench.
class VehicleDynamics final : public IModule {
public:
    explicit VehicleDynamics(bool ideal = false,
                             std::shared_ptr<const services::SceneQuery> ground = nullptr,
                             std::vector<double> groundFriction = {})
        : ideal_(ideal), ground_(std::move(ground)), groundFriction_(std::move(groundFriction)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct WheelState {
        SignalId velocity, loadTorque, driveCommand, slipRatio, slipAngle, normalForce,
            steerAngleOut;
        SignalId steerPosition, steerVelocity, steerCommand;
        double longDeflection{0.0}, latDeflection{0.0};  // m, the relaxation states
        double compression{0.0};
    };
    struct VehicleState {
        Vehicle v;
        Vec3 comLocal{};
        SignalId throttle, brake, steering, gearIn, pose, twist, frictionScale;
        SignalId wrench, speed, engineSpeed, gear;
        std::vector<WheelState> wheels;
        double steerTarget{0.0};  // rad, the command after the rate limit
        double delta{0.0};        // rad, the bicycle angle after the actuator's lag
        int currentGear{1};
        double shiftHold{0.0};   // s since the last shift
        double idealSpeed{0.0};  // m/s, the ideal substitute's speed
    };
    void stepVehicle(VehicleState& s, double dt, SignalBus& bus);

    bool ideal_;
    std::shared_ptr<const services::SceneQuery> ground_;
    std::vector<double> groundFriction_;
    bool pacejka_{true};
    std::vector<VehicleState> vehicles_;
};

}  // namespace lagrange::vehicle_dynamics
