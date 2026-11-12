// Kind "airspeed" (sensors.md §9): a pitot-static probe on a differential pressure sensor.
#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::sensors::kinds {

namespace {

// TE Connectivity MS4525DO-DS5AI001DP (±1 psi differential, the PX4 and ArduPilot airspeed
// sensor; MS4525DO datasheet): 14-bit pressure output between 10 % and 90 % of 16383 counts over
// the full ±1 psi, so one count is 2·6894.76 Pa / (0.8·16383) = 1.052 Pa; 11-bit temperature.
// TODO(verify): its total error band, not modelled by default (set noise and offset).
constexpr double kPsi = 6894.757;                               // Pa per psi
constexpr double kRange = 1.0 * kPsi;                           // Pa, ±1 psi
constexpr double kResolution = 2.0 * kRange / (0.8 * 16383.0);  // Pa per count
constexpr double kTemperatureResolution = 200.0 / 2047.0;       // K: −50…150 °C, 11 bits

class Airspeed final : public KindModule {
public:
    Airspeed(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& view, const ModuleConfig& config, SignalDeclarer& d) override {
        const World& w = view.raw();
        noise_ = kindSetting(w, sensor_, "noise/stddev", 0.0);
        offset_ = kindSetting(w, sensor_, "offset", 0.0);
        range_ = kindSetting(w, sensor_, "range", kRange);
        resolution_ = kindSetting(w, sensor_, "resolution", kResolution);
        temperatureResolution_ =
            kindSetting(w, sensor_, "temperature_resolution", kTemperatureResolution);
        if (noise_ < 0.0 || range_ <= 0.0 || resolution_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "airspeed " + name_ + ": noise, range or resolution out of range");
        }
        // The air's density: the weather's when it runs, else the site's (--air-density, the
        // profile's rotor_aero.air_density), else the sensor's own `air_density`.
        siteDensity_ = config.get<double>("air_density")
                           .value_or(kindSetting(w, sensor_, "air_density", kNaN));
        siteTemperature_ = config.get<double>("ambient_temperature").value_or(kNaN);
        motion_.declare(w, body_, d, true);
        const std::string rotor = ownPrimName(w, sensor_, kindText(w, sensor_, "air_rotor"));
        if (!rotor.empty() && !isRotor(view, rotor)) {
            return Status::error(StatusCode::InvalidConfig,
                                 "airspeed " + name_ + ": air_rotor '" + rotor +
                                     "' is not a rotor (a RotorAPI_1 joint) of this run");
        }
        air_ = d.input(
            optionalNaN(rotor.empty() ? "env/wind/velocity" : "rotor/" + rotor + "/air_velocity",
                        SignalType::Vec3, units::m_per_s, "world_local"));
        density_ = d.input(optionalNaN("env/air_density", SignalType::Scalar, units::kg_per_m3));
        temperature_ = d.input(optionalNaN("env/air_temperature", SignalType::Scalar, units::K));
        pressureOut_ =
            d.output(kindSpec(base() + "/differential_pressure", SignalType::Scalar, units::Pa));
        trueOut_ = d.output(kindSpec(base() + "/airspeed", SignalType::Scalar, units::m_per_s));
        indicatedOut_ =
            d.output(kindSpec(base() + "/indicated_airspeed", SignalType::Scalar, units::m_per_s));
        temperatureOut_ = d.output(kindSpec(base() + "/temperature", SignalType::Scalar, units::K));
        validOut_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, kNaN, kNaN, kNaN, kNaN, false);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        double density = bus.read(density_);
        if (!std::isfinite(density)) {
            density = siteDensity_;
        }
        double temperature = bus.read(temperature_);
        if (!std::isfinite(temperature)) {
            temperature = siteTemperature_;
        }
        Vec3 air = readVec3(bus, air_);
        if (!finite(air)) {
            air = {0.0, 0.0, 0.0};  // no aerodynamics domain: still air, the probe sees its motion
        }
        if (!(density > 0.0)) {
            write(bus, kNaN, kNaN, kNaN, temperature, false);
            return {};
        }
        const Pose pose = compose(motion_.pose(bus), mount_);
        const Vec3 own = motion_.velocityAt(bus, pose.position);
        // The flow into the probe's mouth: the air relative to the sensor, against +x.
        const Vec3 u = rotate(conjugate(pose.orientation), sub(air, own));
        double q = pitotPressure(density, -u[0]);
        if (!groundTruth_) {
            random::Stream rng(context.seed, context.stepIndex, 0);
            q = std::clamp(q + offset_ + noise_ * rng.normal(), -range_, range_);
            q = quantize(q, resolution_);
            temperature = quantize(temperature, temperatureResolution_);
        }
        write(bus, q, airspeedFromPressure(q, density),
              airspeedFromPressure(q, kStandardSeaLevelDensity), temperature, true);
        return {};
    }

private:
    void write(SignalBus& bus, double q, double tas, double ias, double temperature, bool valid) {
        bus.write(pressureOut_, q);
        bus.write(trueOut_, tas);
        bus.write(indicatedOut_, ias);
        bus.write(temperatureOut_, temperature);
        bus.write(validOut_, valid ? 1.0 : 0.0);
    }

    double noise_{0.0}, offset_{0.0}, range_{kRange}, resolution_{kResolution};
    double temperatureResolution_{kTemperatureResolution};
    double siteDensity_{kNaN}, siteTemperature_{kNaN};
    BodyMotion motion_;
    SignalId air_, density_, temperature_;
    SignalId pressureOut_, trueOut_, indicatedOut_, temperatureOut_, validOut_;
};

}  // namespace

std::unique_ptr<IModule> makeAirspeed(const World& world, std::uint32_t sensor, bool groundTruth,
                                      const KindContext&) {
    return std::make_unique<Airspeed>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
