// The thermal domain's ambient substitute (thermal.md §5).
#include "lagrange/thermal/ThermalNetwork.hpp"

namespace lagrange::thermal {

using namespace literals;

ModuleInfo AmbientTemperature::info() const {
    return {"thermal", "ambient", 0, 1, true, true, CapabilityTier::Tier0};
}

Duration AmbientTemperature::preferredStep() const {
    return 100_ms;
}

Status AmbientTemperature::declare(const WorldView& world, const ModuleConfig& config,
                                   SignalDeclarer& d) {
    temperature_.clear();
    derating_.clear();
    overtemp_.clear();
    const std::optional<double> ambient = config.get<double>(kAmbientKey);
    if (!ambient || *ambient <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the ambient substitute needs the site's ambient temperature "
                             "(profile thermal.ambient_temperature, K); it has no default");
    }
    ambient_ = *ambient;
    for (const SchemaInstance* s : world.instancesOf("ThermalAPI_1")) {
        const std::string p = "thermal/" + std::string(world.signalInstanceName(s->prim)) + "/";
        SignalSpec spec;
        spec.name = p + "temperature";
        spec.unit = units::K;
        temperature_.push_back(d.output(spec));
        spec.name = p + "derating";
        spec.unit = units::one;
        derating_.push_back(d.output(spec));
        spec.name = p + "overtemp";
        overtemp_.push_back(d.output(spec));
    }
    return {};
}

Status AmbientTemperature::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (std::size_t i = 0; i < temperature_.size(); ++i) {
        bus.write(temperature_[i], ambient_);
        bus.write(derating_[i], 1.0);
        bus.write(overtemp_[i], 0.0);
    }
    return {};
}

Status AmbientTemperature::step(const StepContext&, SignalBus& bus) {
    for (std::size_t i = 0; i < temperature_.size(); ++i) {
        bus.write(temperature_[i], ambient_);
        bus.write(derating_[i], 1.0);
        bus.write(overtemp_[i], 0.0);
    }
    return {};
}

Status AmbientTemperature::saveState(StateWriter&) const {
    return {};
}

Status AmbientTemperature::restoreState(StateReader&) {
    return {};
}

}  // namespace lagrange::thermal
