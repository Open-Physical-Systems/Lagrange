// Built-in kind `tpms` (sensors.md §9): a tyre-pressure monitoring system, one pressure and
// temperature sensor in every tyre of a vehicle (WheelAPI_1 joints).
//
// The air in a tyre is an ideal gas at nearly constant volume (Gay-Lussac): its absolute pressure
// is the cold fill's, p_fill + p_atm at fill_temperature, times T / T_fill. The tyre air's
// temperature is the ambient air's (env/air_temperature, or the parameter `temperature`): the
// vehicle_dynamics domain has no tyre temperature, so the heating of a running tyre is not
// modelled. A leak lowers the fill at leak_rate. The sensor reports gauge pressure, the absolute
// less env/pressure, and warns as FMVSS No. 138 asks when a tyre is 25 % or more below the
// placard (cold fill) pressure.
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kStandardAtmospherePa = 101325.0;   // Pa (ISO 2533)
constexpr double kDefaultFillPressurePa = 230e3;     // Pa gauge, a typical car placard (2.3 bar)
constexpr double kDefaultFillTemperatureK = 293.15;  // K, filled at 20 °C
/// 49 CFR 571.138 S4.2: the warning comes on when a tyre is 25 % or more below the placard
/// cold inflation pressure.
constexpr double kFmvss138Fraction = 0.75;

class Tpms final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        const auto fail = [&](const std::string& why) {
            return Status::error(StatusCode::InvalidConfig, "tpms " + name_ + ": " + why);
        };
        placard_ = kindSetting(world, sensor_, "fill_pressure", kDefaultFillPressurePa);
        fillTemperature_ =
            kindSetting(world, sensor_, "fill_temperature", kDefaultFillTemperatureK);
        fixedTemperature_ = kindSetting(world, sensor_, "temperature", kNaN);
        leakRate_ = kindSetting(world, sensor_, "leak_rate", 0.0);
        noise_ = kindSetting(world, sensor_, "noise", 0.0);
        if (placard_ <= 0.0 || fillTemperature_ <= 0.0 || leakRate_ < 0.0 || noise_ < 0.0 ||
            !(std::isnan(fixedTemperature_) || fixedTemperature_ > 0.0)) {
            return fail(
                "fill_pressure, fill_temperature and temperature must be positive, leak_rate "
                "and noise not negative");
        }
        // The wheels: those named, else every WheelAPI_1 joint in the sensor's body tree.
        std::vector<std::string> named = words(kindText(world, sensor_, "wheels"));
        for (std::string& wheel : named) {
            wheel = ownPrimName(world, sensor_, wheel);
        }
        const std::uint32_t root = rootOf(world, body_);
        wheels_.clear();
        for (const SchemaInstance* w : view.instancesOf("WheelAPI_1")) {
            const std::string name(view.signalInstanceName(w->prim));
            std::int64_t joint = -1;
            for (std::size_t j = 0; j < world.joints.prim.size(); ++j) {
                if (world.joints.prim[j] == w->prim) {
                    joint = static_cast<std::int64_t>(j);
                }
            }
            if (joint < 0) {
                continue;
            }
            const bool chosen = named.empty()
                                    ? rootOf(world, world.joints.child[joint].index) == root
                                    : std::ranges::find(named, name) != named.end();
            if (chosen) {
                wheels_.push_back(name);
            }
        }
        if (wheels_.empty()) {
            return fail("no WheelAPI_1 wheels on its vehicle (or none of `wheels`)");
        }
        airTemperature_ =
            d.input(optionalKindSpec("env/air_temperature", SignalType::Scalar, units::K));
        airPressure_ = d.input(
            optionalKindSpec("env/pressure", SignalType::Scalar, units::Pa, kStandardAtmospherePa));
        const auto n = static_cast<std::uint32_t>(wheels_.size());
        SignalSpec p = kindSpec(base() + "/pressure", SignalType::ScalarArray, units::Pa);
        p.arrayLength = n;
        pressureOut_ = d.output(p);
        SignalSpec t = kindSpec(base() + "/temperature", SignalType::ScalarArray, units::K);
        t.arrayLength = n;
        temperatureOut_ = d.output(t);
        warningOut_ = d.output(kindSpec(base() + "/warning", SignalType::Bool, units::one));
        validOut_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        pressure_.assign(n, kNaN);
        temperature_.assign(n, kNaN);
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        elapsed_ = 0.0;
        std::fill(pressure_.begin(), pressure_.end(), kNaN);
        std::fill(temperature_.begin(), temperature_.end(), kNaN);
        publish(bus, false, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        elapsed_ += context.dt.seconds();
        double air = bus.read(airTemperature_);
        if (!std::isfinite(air)) {
            air = fixedTemperature_;
        }
        if (!std::isfinite(air)) {
            std::fill(pressure_.begin(), pressure_.end(), kNaN);
            std::fill(temperature_.begin(), temperature_.end(), kNaN);
            publish(bus, false, false);
            return {};
        }
        const double atmosphere = bus.read(airPressure_);
        const double fill = std::max(0.0, placard_ - leakRate_ * elapsed_);
        const double absolute = (fill + kStandardAtmospherePa) * air / fillTemperature_;
        random::Stream rng(context.seed, context.stepIndex, 0);
        bool warning = false;
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            double gauge = absolute - atmosphere;
            const double noise = rng.normal();
            if (!groundTruth_) {
                gauge += noise_ * noise;
            }
            pressure_[i] = gauge;
            temperature_[i] = air;
            warning = warning || gauge <= kFmvss138Fraction * placard_;
        }
        publish(bus, warning, true);
        return {};
    }

    Status saveState(StateWriter& writer) const override {
        writer.put(elapsed_);
        return {};
    }
    Status restoreState(StateReader& reader) override {
        reader.get(elapsed_);
        return {};
    }

private:
    static std::uint32_t rootOf(const World& world, std::uint32_t body) {
        for (std::size_t n = 0; n < world.bodies.parent.size(); ++n) {
            const BodyId parent = world.bodies.parent[body];
            if (parent == kWorldBody) {
                break;
            }
            body = parent.index;
        }
        return body;
    }

    void publish(SignalBus& bus, bool warning, bool valid) {
        bus.writeN(pressureOut_, pressure_);
        bus.writeN(temperatureOut_, temperature_);
        bus.write(warningOut_, warning ? 1.0 : 0.0);
        bus.write(validOut_, valid ? 1.0 : 0.0);
    }

    double placard_{kDefaultFillPressurePa}, fillTemperature_{kDefaultFillTemperatureK};
    double fixedTemperature_{kNaN}, leakRate_{0.0}, noise_{0.0};
    double elapsed_{0.0};
    std::vector<std::string> wheels_;
    std::vector<double> pressure_, temperature_;
    SignalId airTemperature_, airPressure_;
    SignalId pressureOut_, temperatureOut_, warningOut_, validOut_;
};

}  // namespace

std::unique_ptr<IModule> createTpms(const World& world, std::uint32_t sensor, bool groundTruth,
                                    const KindContext&) {
    return std::make_unique<Tpms>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
