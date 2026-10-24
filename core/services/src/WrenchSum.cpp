#include "lagrange/services/WrenchSum.hpp"

#include <array>
#include <map>
#include <span>
#include <string_view>
#include <utility>

namespace lagrange::services {

using namespace literals;

namespace {

constexpr std::string_view kPrefix = "body/";
constexpr std::string_view kContribution = "/wrench/";
constexpr char kFrame[] = "world_local";
constexpr std::size_t kComponents = 6;  // force xyz (N), then torque xyz (N·m)

}  // namespace

ModuleInfo WrenchSum::info() const {
    return {"core", "wrench_sum", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration WrenchSum::preferredStep() const {
    return 1_ms;
}

Status WrenchSum::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) {
    bodies_.clear();
    // body/<b>/wrench/<source>, grouped by body in name order (deterministic).
    std::map<std::string, std::vector<SignalSpec>> found;
    for (SignalSpec& spec : declarer.declaredOutputs(kPrefix)) {
        const std::string_view name = spec.name;
        const std::size_t at = name.find(kContribution, kPrefix.size());
        if (at == std::string_view::npos ||
            name.find('/', at + kContribution.size()) != std::string_view::npos) {
            continue;  // not a contribution: body/<b>/pose, body/<b>/wrench/<a>/<b>, ...
        }
        if (spec.type != SignalType::Wrench || spec.unit != units::N || spec.frame != kFrame) {
            return Status::error(StatusCode::InvalidConfig,
                                 "wrench_sum: " + spec.name +
                                     " must be a Wrench in N (force first), frame world_local");
        }
        found[std::string(name.substr(kPrefix.size(), at - kPrefix.size()))].push_back(
            std::move(spec));
    }
    for (const auto& [body, contributions] : found) {
        Body out;
        out.name = body;
        for (const SignalSpec& spec : contributions) {
            SignalSpec in = spec;
            in.hold = Hold::WindowAverage;
            out.sources.push_back(spec.name.substr(spec.name.rfind('/') + 1));
            out.inputs.push_back(declarer.input(in));
        }
        SignalSpec sum;
        sum.name = "body/" + body + "/external_wrench";
        sum.type = SignalType::Wrench;
        sum.unit = units::N;
        sum.frame = kFrame;
        out.output = declarer.output(sum);
        bodies_.push_back(std::move(out));
    }
    return {};
}

Status WrenchSum::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    const std::array<double, kComponents> zero{};
    for (const Body& body : bodies_) {
        bus.writeN(body.output, zero);
    }
    return {};
}

Status WrenchSum::step(const StepContext&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        std::array<double, kComponents> sum{};
        for (const SignalId input : body.inputs) {
            const std::span<const double> w = bus.readN(input);
            for (std::size_t k = 0; k < kComponents && k < w.size(); ++k) {
                sum.at(k) += w[k];
            }
        }
        bus.writeN(body.output, sum);
    }
    return {};
}

// No state of its own: every step is a function of its inputs.
Status WrenchSum::saveState(StateWriter&) const {
    return {};
}

Status WrenchSum::restoreState(StateReader&) {
    return {};
}

}  // namespace lagrange::services
