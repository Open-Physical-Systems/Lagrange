#include "lagrange/module/IModule.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/module/Substitutes.hpp"

#include <array>
#include <memory>
#include <string>
#include <utility>

namespace lagrange {

std::string_view to_string(StatusCode code) {
    switch (code) {
        case StatusCode::Ok:
            return "Ok";
        case StatusCode::InvalidConfig:
            return "InvalidConfig";
        case StatusCode::UnitMismatch:
            return "UnitMismatch";
        case StatusCode::MissingInput:
            return "MissingInput";
        case StatusCode::Diverged:
            return "Diverged";
        case StatusCode::Unsupported:
            return "Unsupported";
        case StatusCode::Internal:
            return "Internal";
    }
    return "Unknown";
}

SignalId SignalDeclarer::input(SignalSpec spec) {
    if (std::string problem = checkSignalName(spec.name); !problem.empty()) {
        problems_.push_back("input: " + problem);
        return {};
    }
    if (spec.type == SignalType::ScalarArray && spec.arrayLength == 0) {
        problems_.push_back("input \"" + spec.name + "\": ScalarArray with length 0");
        return {};
    }
    inputs_.push_back(spec);
    const std::uint32_t slot = bus_->addInput(std::move(spec), module_);
    inputSlots_.push_back(slot);
    return SignalId::input(slot);
}

SignalId SignalDeclarer::output(SignalSpec spec) {
    if (std::string problem = checkSignalName(spec.name); !problem.empty()) {
        problems_.push_back("output: " + problem);
        return {};
    }
    if (spec.type == SignalType::ScalarArray && spec.arrayLength == 0) {
        problems_.push_back("output \"" + spec.name + "\": ScalarArray with length 0");
        return {};
    }
    if (spec.feedback || spec.optional || spec.hold != Hold::ZeroOrder) {
        problems_.push_back("output \"" + spec.name +
                            "\": hold, optional and feedback apply to inputs only");
        return {};
    }
    outputs_.push_back(spec);
    const std::uint32_t signal = bus_->addSignal(std::move(spec), module_);
    outputSignals_.push_back(signal);
    return SignalId::output(signal);
}

// --- substitutes ---------------------------------------------------------------------------

std::string_view to_string(SubstituteKind kind) {
    switch (kind) {
        case SubstituteKind::Ideal:
            return "ideal";
        case SubstituteKind::Constant:
            return "constant";
        case SubstituteKind::Replay:
            return "replay";
    }
    return "unknown";
}

std::optional<SubstituteChoice> substituteForToken(std::string_view token) {
    // docs/design/architecture.md §7: profile value → (kind, preset).
    struct Entry {
        std::string_view token;
        SubstituteKind kind;
        std::string_view preset;
    };
    static constexpr std::array<Entry, 8> kTable{{
        {"ideal", SubstituteKind::Ideal, "ideal_torque"},
        {"zero", SubstituteKind::Constant, "zero"},
        {"clear", SubstituteKind::Constant, "clear"},
        {"ambient", SubstituteKind::Constant, "ambient"},
        {"ground_truth", SubstituteKind::Ideal, "ground_truth"},
        {"fixed", SubstituteKind::Constant, "fixed"},
        {"load_profile", SubstituteKind::Ideal, "load_profile"},
        {"replay", SubstituteKind::Replay, ""},
    }};
    for (const Entry& entry : kTable) {
        if (entry.token == token) {
            return SubstituteChoice{entry.kind, std::string(entry.preset)};
        }
    }
    return std::nullopt;
}

SubstituteRegistry& SubstituteRegistry::instance() {
    static SubstituteRegistry registry;
    return registry;
}

void SubstituteRegistry::add(SubstituteDescriptor descriptor) {
    substitutes_.push_back(std::move(descriptor));
}

void SubstituteRegistry::addContract(DomainContract contract) {
    for (DomainContract& existing : contracts_) {
        if (existing.domain == contract.domain) {
            existing = std::move(contract);
            return;
        }
    }
    contracts_.push_back(std::move(contract));
}

const DomainContract* SubstituteRegistry::contract(std::string_view domain) const {
    for (const DomainContract& contract : contracts_) {
        if (contract.domain == domain) {
            return &contract;
        }
    }
    return nullptr;
}

std::unique_ptr<IModule> SubstituteRegistry::create(std::string_view domain, SubstituteKind kind,
                                                    std::string_view preset) const {
    for (const SubstituteDescriptor& descriptor : substitutes_) {
        if (descriptor.domain == domain && descriptor.kind == kind &&
            (kind == SubstituteKind::Replay || descriptor.preset == preset)) {
            return descriptor.create();
        }
    }
    return nullptr;
}

std::vector<const SubstituteDescriptor*> SubstituteRegistry::list(std::string_view domain) const {
    std::vector<const SubstituteDescriptor*> out;
    for (const SubstituteDescriptor& descriptor : substitutes_) {
        if (descriptor.domain == domain) {
            out.push_back(&descriptor);
        }
    }
    return out;
}

}  // namespace lagrange
