// Kind `bumper` (sensors.md §9): a contact switch on the sensor's body. It reads what touches the
// body pushes it with (body/<b>/contact_force, rigid_body.md "Contact force"), in the sensor
// frame, and is pressed when that push, from inside its direction sector, reaches its threshold.
#include "KindScene.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

// Defaults. Illustrative, no part behind them: a bumper switch needs a few newtons to close, and
// a robot's caster rolling on the floor drags its base with well under that.
constexpr double kDefaultThreshold = 5.0;  // N
constexpr double kTwoPi = 2.0 * std::numbers::pi;

class Bumper final : public KindModule {
public:
    Bumper(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        threshold_ = kindSetting(raw, sensor_, "threshold", kDefaultThreshold);
        sectorMin_ = kindSetting(raw, sensor_, "sector_min", -std::numbers::pi);
        sectorMax_ = kindSetting(raw, sensor_, "sector_max", std::numbers::pi);
        vertical_ = kindSetting(raw, sensor_, "vertical", 0.0) != 0.0;
        if (threshold_ <= 0.0 || sectorMax_ <= sectorMin_) {
            return Status::error(
                StatusCode::InvalidConfig,
                "bumper " + name_ + " needs a threshold above 0 N and sector_max above sector_min");
        }
        poses_.watch(raw, d, body_);
        if (!poses_.moves(body_)) {
            return Status::error(StatusCode::InvalidConfig,
                                 "bumper " + name_ + " is on a body that never moves");
        }
        // The mean push over the sensor's period: a switch closes on what the contact does over
        // time, not on one physics step's impulse. Optional: without physics nothing touches it.
        SignalSpec contact =
            kindSpec("body/" + std::string(world.signalInstanceName(raw.bodies.prim[body_])) +
                         "/contact_force",
                     SignalType::Vec3, units::N, "world_local");
        contact.hold = Hold::WindowAverage;
        contact.optional = true;
        contact_ = d.input(contact);
        pressed_ = d.output(kindSpec(base() + "/pressed", SignalType::Bool, units::one));
        force_ = d.output(kindSpec(base() + "/force", SignalType::Scalar, units::N));
        direction_ = d.output(kindSpec(base() + "/direction", SignalType::Scalar, units::rad));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(pressed_, 0.0);
        bus.write(force_, 0.0);
        bus.write(direction_, 0.0);
        return {};
    }

    Status step(const StepContext&, SignalBus& bus) override {
        const Pose sensor = compose(poses_.at(bus, body_), mount_);
        const std::span<const double> fw = bus.readN(contact_);
        const std::array<double, 3> f =
            rotate(conjugate(sensor.orientation), {fw[0], fw[1], fw[2]});
        // The push is along f; what pushes sits the other way, at this angle in the sensor's
        // xy plane (0: straight ahead).
        const double horizontal = std::hypot(f[0], f[1]);
        const double magnitude = vertical_ ? std::hypot(horizontal, f[2]) : horizontal;
        const double angle = horizontal > 0.0 ? std::atan2(-f[1], -f[0]) : 0.0;
        double fromMin = std::fmod(angle - sectorMin_, kTwoPi);
        if (fromMin < 0.0) {
            fromMin += kTwoPi;
        }
        const bool inSector =
            sectorMax_ - sectorMin_ >= kTwoPi || fromMin <= sectorMax_ - sectorMin_;
        const bool pressed = magnitude >= threshold_ && inSector;
        bus.write(pressed_, pressed ? 1.0 : 0.0);
        bus.write(force_, magnitude);
        bus.write(direction_, angle);
        return {};
    }

private:
    BodyPoses poses_;
    double threshold_{kDefaultThreshold};
    double sectorMin_{0.0}, sectorMax_{0.0};
    bool vertical_{false};
    SignalId contact_, pressed_, force_, direction_;
};

}  // namespace

std::unique_ptr<IModule> makeBumper(const World& world, std::uint32_t sensor, bool groundTruth,
                                    const KindContext&) {
    return std::make_unique<Bumper>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
