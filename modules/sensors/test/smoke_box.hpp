#pragma once
// A module that publishes a uniform smoke box as env/smoke/extinction, as the fire domain does
// (RFC 0022 §4): σ (1/m, at 633 nm) over every voxel of a box.

#include "lagrange/fields/Field.hpp"
#include "lagrange/module/IModule.hpp"

#include <vector>

namespace lagrange::sensors::test {

class SmokeBox final : public IModule {
public:
    SmokeBox(fields::DenseBox box, double sigma) : box_(box), sigma_(sigma) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "smoke_box", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{100'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "env/smoke/extinction";
        s.type = SignalType::Field3D;
        s.unit = units::per_m;
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        const std::vector<float> values(static_cast<std::size_t>(box_.count[0]) *
                                            static_cast<std::size_t>(box_.count[1]) *
                                            static_cast<std::size_t>(box_.count[2]),
                                        static_cast<float>(sigma_));
        field_ = fields::scalarField("smoke_extinction", box_, values);
        bus.publishBuffer(out_, fields::fieldBuffer(field_.get()));
        return {};
    }
    Status step(const StepContext&, SignalBus&) override { return {}; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    fields::DenseBox box_;
    double sigma_;
    std::shared_ptr<const fields::Field> field_;
    SignalId out_;
};

/// The sensors' config with the fires' smoke on.
inline ModuleConfig smokeConfig() {
    ModuleConfigData data;
    data.values["smoke"] = true;
    return ModuleConfig(data);
}

}  // namespace lagrange::sensors::test
