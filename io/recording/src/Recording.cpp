// Reading recordings and replaying them on the bus; rules in Recording.hpp.
#include "lagrange/record/v1/record.pb.h"
#include "lagrange/recording/Recording.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/viewer/v1/signals.pb.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <utility>

#include <mcap/reader.hpp>

namespace lagrange::recording {

namespace {

namespace v1 = lagrange::viewer::v1;

constexpr std::string_view kSignalsTopic = "/lagrange/signals/";

Unit unitOf(const v1::Unit& u) {
    Unit unit;
    unit.dim = {static_cast<std::int8_t>(u.length()),      static_cast<std::int8_t>(u.mass()),
                static_cast<std::int8_t>(u.time()),        static_cast<std::int8_t>(u.current()),
                static_cast<std::int8_t>(u.temperature()), static_cast<std::int8_t>(u.amount()),
                static_cast<std::int8_t>(u.luminous()),    static_cast<std::int8_t>(u.angle())};
    unit.scale = u.scale() == 0.0 ? 1.0 : u.scale();
    unit.kind = static_cast<QuantityKind>(u.kind());
    return unit;
}

}  // namespace

const RecordedSignal* Recording::find(std::string_view name) const {
    for (const RecordedSignal& s : signals) {
        if (s.name == name) {
            return &s;
        }
    }
    return nullptr;
}

Expected<Recording, std::string> Recording::open(
    const std::string& path,
    const std::function<bool(const std::string&, const std::string&)>& keep) {
    mcap::McapReader reader;
    if (const mcap::Status status = reader.open(path); !status.ok()) {
        return Unexpected{path + ": " + status.message};
    }
    Recording out;
    out.path = path;
    std::map<std::uint32_t, std::size_t> slotOf;  // catalog id -> signals index
    bool haveCatalog = false;
    std::string problem;
    const auto onProblem = [&problem](const mcap::Status& status) {
        if (problem.empty()) {
            problem = status.message;
        }
    };
    for (const mcap::MessageView& view : reader.readMessages(onProblem)) {
        const std::string& topic = view.channel->topic;
        const auto* data = view.message.data;
        const auto size = static_cast<int>(view.message.dataSize);
        if (topic == "/lagrange/meta") {
            record::v1::RunMeta meta;
            if (meta.ParseFromArray(data, size)) {
                out.profileYaml = meta.profile_yaml();
                out.stagePath = meta.stage_path();
                out.duration = SimTime{meta.duration().ns()};
            }
        } else if (topic == "/lagrange/catalog" && !haveCatalog) {
            v1::SignalCatalog catalog;
            if (!catalog.ParseFromArray(data, size)) {
                return Unexpected{path + ": its signal catalog cannot be read"};
            }
            haveCatalog = true;
            for (const v1::SignalInfo& info : catalog.signals()) {
                if (keep && !keep(info.name(), info.writer())) {
                    continue;
                }
                RecordedSignal s;
                s.name = info.name();
                s.type = static_cast<SignalType>(std::max(0, static_cast<int>(info.type()) - 1));
                s.unit = unitOf(info.unit());
                s.components = std::max<std::uint32_t>(1, info.components());
                s.dtNs = info.native_dt_ns();
                s.writer = info.writer();
                slotOf.emplace(info.id(), out.signals.size());
                out.signals.push_back(std::move(s));
            }
        } else if (std::string_view(topic).starts_with(kSignalsTopic)) {
            v1::SignalBlock block;
            if (!block.ParseFromArray(data, size)) {
                return Unexpected{path + ": a signal block on " + topic + " cannot be read"};
            }
            for (const v1::SignalRun& run : block.runs()) {
                const auto slot = slotOf.find(run.signal());
                if (slot == slotOf.end() || run.dt_ns() <= 0 ||
                    run.encoding() != v1::SAMPLE_ENCODING_F64_LE) {
                    continue;
                }
                RecordedSignal& s = out.signals[slot->second];
                const std::size_t count = run.sample_count();
                const std::size_t components = std::max<std::uint32_t>(1, run.components());
                if (components != s.components ||
                    run.samples().size() != count * components * sizeof(double)) {
                    return Unexpected{path + ": a run of " + s.name + " has the wrong size"};
                }
                const auto first = static_cast<std::size_t>(run.t0().ns() / run.dt_ns());
                const std::size_t needed = (first + count) * components;
                if (s.samples.size() < needed) {
                    s.samples.resize(needed, std::numeric_limits<double>::quiet_NaN());
                }
                std::memcpy(s.samples.data() + first * components, run.samples().data(),
                            run.samples().size());
            }
        }
    }
    if (!problem.empty()) {
        return Unexpected{path + ": " + problem};
    }
    if (!haveCatalog) {
        return Unexpected{path + ": no signal catalog: not a Lagrange recording"};
    }
    return out;
}

struct ReplaySubstitute::Impl {
    Status loadStatus;
    Recording recording;
    Duration step{1'000'000};
    struct Output {
        const RecordedSignal* signal{nullptr};
        SignalId id;
        std::vector<double> last;  // the last finite sample written, held past gaps and the end
    };
    std::vector<Output> outputs;
    bool endLogged{false};
};

ReplaySubstitute::ReplaySubstitute(std::string domain, std::string path)
    : domain_(std::move(domain)), path_(std::move(path)) {}

ReplaySubstitute::~ReplaySubstitute() = default;

ReplaySubstitute::Impl& ReplaySubstitute::load() const {
    if (impl_) {
        return *impl_;
    }
    impl_ = std::make_unique<Impl>();
    Impl& f = *impl_;
    const std::string prefix = domain_ + "/";
    auto recording = Recording::open(path_, [&prefix](const std::string&, const std::string& w) {
        return w.starts_with(prefix);
    });
    if (!recording) {
        f.loadStatus = Status::error(StatusCode::InvalidConfig, recording.error());
        return f;
    }
    f.recording = std::move(*recording);
    if (f.recording.signals.empty()) {
        f.loadStatus =
            Status::error(StatusCode::InvalidConfig, path_ + " has no signals written by the " +
                                                         domain_ + " domain to replay");
        return f;
    }
    // Every recorded rate is a multiple of the step.
    std::int64_t step = 0;
    for (const RecordedSignal& s : f.recording.signals) {
        step = std::gcd(step, std::max<std::int64_t>(s.dtNs, 1));
    }
    f.step = Duration{step};
    return f;
}

ModuleInfo ReplaySubstitute::info() const {
    return {domain_, "replay", 0, 1, true, true, CapabilityTier::Tier0};
}

Duration ReplaySubstitute::preferredStep() const {
    return load().step;
}

Status ReplaySubstitute::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    Impl& f = load();
    if (!f.loadStatus.ok()) {
        return f.loadStatus;
    }
    f.outputs.clear();
    for (const RecordedSignal& s : f.recording.signals) {
        SignalSpec spec;
        spec.name = s.name;
        spec.type = s.type;
        spec.unit = s.unit;
        spec.arrayLength = s.type == SignalType::ScalarArray ? s.components : 1;
        f.outputs.push_back({&s, d.output(spec), std::vector<double>(s.components, 0.0)});
    }
    return {};
}

Status ReplaySubstitute::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    Impl& f = *impl_;
    f.endLogged = false;
    for (Impl::Output& o : f.outputs) {
        std::fill(o.last.begin(), o.last.end(), 0.0);
        if (o.signal->sampleCount() > 0) {
            for (std::uint32_t c = 0; c < o.signal->components; ++c) {
                const double v = o.signal->samples[c];
                o.last[c] = std::isfinite(v) ? v : 0.0;
            }
        }
        bus.writeN(o.id, o.last);
    }
    return {};
}

Status ReplaySubstitute::step(const StepContext& context, SignalBus& bus) {
    Impl& f = *impl_;
    const std::int64_t end = context.t.ns + context.dt.ns;  // this step's commit time
    for (Impl::Output& o : f.outputs) {
        const RecordedSignal& s = *o.signal;
        const std::int64_t dt = std::max<std::int64_t>(s.dtNs, 1);
        if (end % dt != 0) {
            continue;
        }
        const auto k = static_cast<std::size_t>(end / dt);
        if (k < s.sampleCount()) {
            for (std::uint32_t c = 0; c < s.components; ++c) {
                const double v = s.samples[k * s.components + c];
                if (std::isfinite(v)) {
                    o.last[c] = v;
                }
            }
        } else if (!f.endLogged) {
            f.endLogged = true;
            services::log(services::LogLevel::Warning, domain_ + "/replay",
                          path_ + " ends at " +
                              std::to_string(static_cast<double>(s.sampleCount() - 1) *
                                             static_cast<double>(dt) / 1e9) +
                              " s; holding its last values");
        }
        bus.writeN(o.id, o.last);
    }
    return {};
}

Status ReplaySubstitute::saveState(StateWriter& writer) const {
    for (const Impl::Output& o : impl_->outputs) {
        writer.putDoubles(o.last);
    }
    return {};
}

Status ReplaySubstitute::restoreState(StateReader& reader) {
    for (Impl::Output& o : impl_->outputs) {
        if (!reader.getDoubles(o.last)) {
            return Status::error(StatusCode::InvalidConfig, "replay state truncated");
        }
    }
    return {};
}

std::vector<std::string> ReplaySubstitute::signalNames() const {
    std::vector<std::string> out;
    if (impl_) {
        for (const Impl::Output& o : impl_->outputs) {
            out.push_back(o.signal->name);
        }
    }
    return out;
}

}  // namespace lagrange::recording
