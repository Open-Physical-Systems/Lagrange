#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::recording {

/// One recorded signal: uniform samples from t = 0, sample k holding the writer's value for
/// time k·dt (its commit time). Missing samples are NaN.
struct RecordedSignal {
    std::string name;
    SignalType type{SignalType::Scalar};
    Unit unit;
    std::uint32_t components{1};
    std::int64_t dtNs{0};
    std::string writer;           // "rigid_body/mujoco"
    std::vector<double> samples;  // sample-major: samples[k * components + c]

    [[nodiscard]] std::size_t sampleCount() const {
        return components == 0 ? 0 : samples.size() / components;
    }
};

/// A run recorded by `lagrange_server --record` (docs/design/formats.md §7), its signals in
/// memory. Only signals that `keep` accepts are loaded (all when it is empty).
struct Recording {
    std::string path;
    std::string profileYaml;  // RunMeta
    std::string stagePath;
    SimTime duration;
    std::vector<RecordedSignal> signals;

    [[nodiscard]] const RecordedSignal* find(std::string_view name) const;

    static Expected<Recording, std::string> open(
        const std::string& path,
        const std::function<bool(const std::string& name, const std::string& writer)>& keep = {});
};

/// A domain's `replay` substitute (CLAUDE.md §11.2): every signal of the recording written by a
/// module of `domain` ("rigid_body" → the physics backend's joint and body signals) is written
/// again, sample for sample, at its recorded rate and with its recorded unit. It reads nothing:
/// the replay is open loop, as on a test bench. At each step it commits the sample for its
/// commit time, as a live writer does. After the recording ends it holds the last samples (and
/// logs that once). Deterministic; it has no state of its own but the time.
class ReplaySubstitute final : public IModule {
public:
    ReplaySubstitute(std::string domain, std::string path);
    ~ReplaySubstitute() override;
    ReplaySubstitute(const ReplaySubstitute&) = delete;
    ReplaySubstitute& operator=(const ReplaySubstitute&) = delete;
    ReplaySubstitute(ReplaySubstitute&&) = delete;
    ReplaySubstitute& operator=(ReplaySubstitute&&) = delete;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The signals it replays, after declare.
    [[nodiscard]] std::vector<std::string> signalNames() const;

private:
    struct Impl;
    /// Loads the recording once: the step needs it before declare.
    Impl& load() const;

    std::string domain_;
    std::string path_;
    mutable std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::recording
