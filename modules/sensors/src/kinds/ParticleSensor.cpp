// Built-in kind `particle_sensor` (sensors.md §9): an optical particulate-matter sensor such as
// the Sensirion SPS30, reading the dust in the air the weather domain carries.
//
// The air holds two populations of monodisperse grains: the weather's own dust
// (env/dust/concentration, env/dust/median_diameter) and what rotor outwash lifts off loose
// ground (env/dust/downwash, of `downwash_diameter`). PMx is the mass of grains with diameter
// between the sensor's lower size limit and x. The reading follows the air with a first-order
// lag, gets the datasheet's precision as noise, and saturates at the measurement range.
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"

#include <array>
#include <cmath>
#include <string>

namespace lagrange::sensors::kinds {

namespace {

// Sensirion SPS30 datasheet (PM sensor, v1.0 2020 and v2.0 2023): particle size range 0.3 µm to
// 10 µm, mass concentration range 0–1000 µg/m³, sampling interval 1 s; precision PM1 and PM2.5
// ±10 µg/m³ (0–100 µg/m³) and ±10 % of the measured value (100–1000), PM4 and PM10 ±25 µg/m³ and
// ±25 %. The precision is taken as 1σ.
constexpr double kLowerSizeM = 0.3e-6;
constexpr std::array<double, 4> kClassSizeM{1.0e-6, 2.5e-6, 4.0e-6, 10.0e-6};
constexpr std::array<const char*, 4> kClassNames{"pm1", "pm2_5", "pm4", "pm10"};
constexpr double kUgPerM3 = 1e-9;  // kg/m³ per µg/m³
constexpr std::array<double, 4> kPrecisionAbs{10.0 * kUgPerM3, 10.0 * kUgPerM3, 25.0 * kUgPerM3,
                                              25.0 * kUgPerM3};
constexpr std::array<double, 4> kPrecisionRel{0.10, 0.10, 0.25, 0.25};
constexpr double kPrecisionBreak = 100.0 * kUgPerM3;  // where the precision turns relative
constexpr double kRangeMax = 1000.0 * kUgPerM3;
/// Time constant of the reading, s: the SPS30's start-up time at 200–3000 particles/cm³ (about
/// 8 s, datasheet); the datasheet gives no step response. TODO(verify) against a measured step.
constexpr double kDefaultTimeConstantS = 8.0;
/// Lifted ground grains, m: the weather domain's ground_dust_diameter default.
constexpr double kDefaultDownwashDiameterM = 1e-4;

class ParticleSensor final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        tau_ = kindSetting(world, sensor_, "time_constant", kDefaultTimeConstantS);
        noiseScale_ = kindSetting(world, sensor_, "noise_scale", 1.0);
        downwashDiameter_ =
            kindSetting(world, sensor_, "downwash_diameter", kDefaultDownwashDiameterM);
        if (tau_ < 0.0 || noiseScale_ < 0.0 || downwashDiameter_ <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "particle_sensor " + name_ +
                                     ": time_constant and noise_scale must not be negative, "
                                     "downwash_diameter positive");
        }
        dust_ = d.input(
            optionalKindSpec("env/dust/concentration", SignalType::Scalar, units::kg_per_m3));
        diameter_ =
            d.input(optionalKindSpec("env/dust/median_diameter", SignalType::Scalar, units::m));
        lifted_ = d.input(
            optionalKindSpec("env/dust/downwash", SignalType::Scalar, units::kg_per_m3, 0.0));
        for (std::size_t k = 0; k < kClassNames.size(); ++k) {
            out_[k] = d.output(
                kindSpec(base() + "/" + kClassNames[k], SignalType::Scalar, units::kg_per_m3));
        }
        validOut_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        started_ = false;
        state_.fill(0.0);
        for (const SignalId id : out_) {
            bus.write(id, kNaN);
        }
        bus.write(validOut_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double ambient = bus.read(dust_);
        const double diameter = bus.read(diameter_);
        if (!std::isfinite(ambient) || !std::isfinite(diameter)) {
            // No weather domain: nothing to read.
            for (const SignalId id : out_) {
                bus.write(id, kNaN);
            }
            bus.write(validOut_, 0.0);
            return {};
        }
        const double lifted = std::max(0.0, bus.read(lifted_));
        const double dt = context.dt.seconds();
        const double blend = !started_ || tau_ <= 0.0 ? 1.0 : -std::expm1(-dt / tau_);
        started_ = true;
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (std::size_t k = 0; k < kClassSizeM.size(); ++k) {
            const double air = massBelow(ambient, diameter, kClassSizeM[k]) +
                               massBelow(lifted, downwashDiameter_, kClassSizeM[k]);
            if (groundTruth_) {
                bus.write(out_[k], air);
                continue;
            }
            state_[k] += (air - state_[k]) * blend;
            const double sigma =
                state_[k] <= kPrecisionBreak ? kPrecisionAbs[k] : kPrecisionRel[k] * state_[k];
            const double reading = state_[k] + noiseScale_ * sigma * rng.normal();
            bus.write(out_[k], std::clamp(reading, 0.0, kRangeMax));
        }
        bus.write(validOut_, 1.0);
        return {};
    }

    Status saveState(StateWriter& writer) const override {
        writer.put(started_ ? 1.0 : 0.0);
        for (const double s : state_) {
            writer.put(s);
        }
        return {};
    }
    Status restoreState(StateReader& reader) override {
        double started = 0.0;
        reader.get(started);
        started_ = started != 0.0;
        for (double& s : state_) {
            reader.get(s);
        }
        return {};
    }

private:
    /// The mass of monodisperse grains of `diameter` counted in a class up to `size`.
    static double massBelow(double concentration, double diameter, double size) {
        return diameter >= kLowerSizeM && diameter <= size ? concentration : 0.0;
    }

    double tau_{kDefaultTimeConstantS}, noiseScale_{1.0},
        downwashDiameter_{kDefaultDownwashDiameterM};
    bool started_{false};
    std::array<double, 4> state_{};
    SignalId dust_, diameter_, lifted_, validOut_;
    std::array<SignalId, 4> out_{};
};

}  // namespace

std::unique_ptr<IModule> createParticleSensor(const World& world, std::uint32_t sensor,
                                              bool groundTruth, const KindContext&) {
    return std::make_unique<ParticleSensor>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
