// Kind "anemometer" (sensors.md §9): an ultrasonic (2D or 3D) or cup-and-vane anemometer.
#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

// Gill Instruments WindSonic (User Manual 1405-PS-0019 Issue 30, Nov 2022, appendix A): speed
// accuracy "± 2% RMSE (at 12m/s)", resolution 0.01 m/s; direction accuracy ±2° at 12 m/s (±3° in
// the 2009 datasheet), resolution 1°; starting threshold 0.01 m/s (WindSonic datasheets). The
// speed's RMS error is its 1σ; the direction's ± is taken as 2σ.
constexpr double kSonicSpeedNoiseFraction = 0.02;                        // of the speed, 1σ
constexpr double kSonicSpeedNoise = 0.01;                                // m/s, 1σ, the least
constexpr double kSonicDirectionNoise = 1.0 * std::numbers::pi / 180.0;  // rad, 1σ
constexpr double kSonicSpeedResolution = 0.01;                           // m/s
constexpr double kSonicDirectionResolution = std::numbers::pi / 180.0;   // rad, 1°
constexpr double kSonicThreshold = 0.01;                                 // m/s
// R. M. Young 05103 Wind Monitor (manual 05103-90, rev. N062819, specifications): propeller
// threshold 1.0 m/s, vane threshold 1.1 m/s at 10° displacement, distance constant 2.7 m (63 %
// recovery).
constexpr double kCupThreshold = 1.0;         // m/s
constexpr double kVaneThreshold = 1.1;        // m/s
constexpr double kCupDistanceConstant = 2.7;  // m

class Anemometer final : public KindModule {
public:
    Anemometer(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        const std::string type = kindText(w, sensor_, "type", "ultrasonic");
        if (type != "ultrasonic" && type != "cup") {
            return Status::error(
                StatusCode::InvalidConfig,
                "anemometer " + name_ + ": type is ultrasonic or cup, not '" + type + "'");
        }
        cup_ = type == "cup";
        axes_ = static_cast<int>(kindSetting(w, sensor_, "axes", 2.0));
        if (axes_ != 2 && axes_ != 3) {
            return Status::error(StatusCode::InvalidConfig,
                                 "anemometer " + name_ + ": axes is 2 or 3");
        }
        if (cup_) {
            axes_ = 2;
        }
        noiseFraction_ =
            kindSetting(w, sensor_, "noise/fraction", cup_ ? 0.0 : kSonicSpeedNoiseFraction);
        noise_ = kindSetting(w, sensor_, "noise/speed", cup_ ? 0.0 : kSonicSpeedNoise);
        directionNoise_ =
            kindSetting(w, sensor_, "noise/direction", cup_ ? 0.0 : kSonicDirectionNoise);
        speedResolution_ = kindSetting(w, sensor_, "resolution/speed", kSonicSpeedResolution);
        directionResolution_ =
            kindSetting(w, sensor_, "resolution/direction", kSonicDirectionResolution);
        threshold_ =
            kindSetting(w, sensor_, "threshold/speed", cup_ ? kCupThreshold : kSonicThreshold);
        vaneThreshold_ =
            kindSetting(w, sensor_, "threshold/direction", cup_ ? kVaneThreshold : kSonicThreshold);
        distanceConstant_ =
            kindSetting(w, sensor_, "distance_constant", cup_ ? kCupDistanceConstant : 0.0);
        if (noiseFraction_ < 0.0 || noise_ < 0.0 || directionNoise_ < 0.0 || threshold_ < 0.0 ||
            vaneThreshold_ < 0.0 || distanceConstant_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "anemometer " + name_ + ": negative noise, threshold or constant");
        }
        motion_.declare(w, body_, d, true);
        // The air at the sensor: a rotor's air (the wind plus other robots' downwash at its hub)
        // when `air_rotor` names one, else the aerodynamics domain's wind.
        const std::string rotor = ownPrimName(w, sensor_, kindText(w, sensor_, "air_rotor"));
        if (!rotor.empty() && !isRotor(view, rotor)) {
            return Status::error(StatusCode::InvalidConfig,
                                 "anemometer " + name_ + ": air_rotor '" + rotor +
                                     "' is not a rotor (a RotorAPI_1 joint) of this run");
        }
        air_ = d.input(
            optionalNaN(rotor.empty() ? "env/wind/velocity" : "rotor/" + rotor + "/air_velocity",
                        SignalType::Vec3, units::m_per_s, "world_local"));
        velocity_ = d.output(
            kindSpec(base() + "/wind_velocity", SignalType::Vec3, units::m_per_s, "sensor"));
        speed_ = d.output(kindSpec(base() + "/wind_speed", SignalType::Scalar, units::m_per_s));
        direction_ = d.output(kindSpec(base() + "/wind_direction", SignalType::Scalar, units::rad));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        cupSpeed_ = kNaN;
        vane_ = 0.0;
        write(bus, {kNaN, kNaN, kNaN}, kNaN, kNaN, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Vec3 air = readVec3(bus, air_);
        if (!finite(air)) {
            write(bus, {kNaN, kNaN, kNaN}, kNaN, kNaN, false);  // the aerodynamics domain is off
            return {};
        }
        const Pose pose = compose(motion_.pose(bus), mount_);
        const Vec3 own = motion_.velocityAt(bus, pose.position);
        // The air relative to the sensor, in its frame.
        Vec3 u = rotate(conjugate(pose.orientation), sub(air, own));
        if (axes_ == 2) {
            u[2] = 0.0;
        }
        const double horizontal = std::hypot(u[0], u[1]);
        double speed = axes_ == 3 ? norm(u) : horizontal;
        double from = windFromDirection(u[0], u[1]);
        if (groundTruth_) {
            write(bus, u, speed, from, true);
            return {};
        }
        random::Stream rng(context.seed, context.stepIndex, 0);
        const double dt = context.dt.seconds();
        if (cup_) {
            // The cups turn only above their threshold and follow the wind as a first-order lag of
            // time constant L/V (distance constant L); the vane holds below its own.
            const double target = horizontal >= threshold_ ? horizontal : 0.0;
            const double tau =
                distanceConstant_ > 0.0 ? distanceConstant_ / std::max(target, threshold_) : 0.0;
            cupSpeed_ = firstOrderStep(cupSpeed_, target, dt, tau);
            if (horizontal >= vaneThreshold_) {
                vane_ = from;
            }
            speed = cupSpeed_;
            from = vane_;
        } else if (speed < threshold_) {
            speed = 0.0;
        }
        // Gain error on the speed, angular error on the direction.
        const double sigma = std::max(noise_, noiseFraction_ * speed);
        double measured = std::max(0.0, speed + sigma * rng.normal());
        double direction = from + directionNoise_ * rng.normal();
        measured = quantize(measured, speedResolution_);
        direction = quantize(direction, directionResolution_);
        direction = std::fmod(direction, 2.0 * std::numbers::pi);
        if (direction < 0.0) {
            direction += 2.0 * std::numbers::pi;
        }
        // The vector from speed and direction, the vertical part scaled with the speed.
        const double h = axes_ == 3 && speed > 0.0 ? measured * horizontal / speed : measured;
        const double vertical = axes_ == 3 && speed > 0.0 ? measured * u[2] / speed : 0.0;
        const Vec3 velocity{-h * std::cos(direction), h * std::sin(direction), vertical};
        write(bus, velocity, measured, direction, true);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put(cupSpeed_);
        w.put(vane_);
        return {};
    }
    Status restoreState(StateReader& r) override {
        r.get(cupSpeed_);
        r.get(vane_);
        return r.failed() ? Status::error(StatusCode::Internal, "anemometer: short state")
                          : Status{};
    }

private:
    void write(SignalBus& bus, const Vec3& v, double speed, double direction, bool valid) {
        bus.writeN(velocity_, v);
        bus.write(speed_, speed);
        bus.write(direction_, direction);
        bus.write(valid_, valid ? 1.0 : 0.0);
    }

    bool cup_{false};
    int axes_{2};
    double noiseFraction_{0.0}, noise_{0.0}, directionNoise_{0.0};
    double speedResolution_{0.0}, directionResolution_{0.0};
    double threshold_{0.0}, vaneThreshold_{0.0}, distanceConstant_{0.0};
    double cupSpeed_{kNaN}, vane_{0.0};
    BodyMotion motion_;
    SignalId air_, velocity_, speed_, direction_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeAnemometer(const World& world, std::uint32_t sensor, bool groundTruth,
                                        const KindContext&) {
    return std::make_unique<Anemometer>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
