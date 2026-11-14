#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Faults.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <utility>

namespace lagrange::sensors {
namespace {

/// Philox streams of the faults: 2 f for fault f's dropout draw, 2 f + 1 for its noise. High, so
/// that they never meet the streams the wrapped sensor draws its own noise from (its entity
/// indices count from 0).
constexpr std::uint32_t kFaultStreams = 0xFA17'0000U;

constexpr std::string_view kFaultSegment = "fault/";

/// Whether `name`, a signal of the sensor whose prefix "sensor/<n>/" is `baseSize` long, is one
/// the faults themselves write (sensor/<n>/fault/...): by the quantity, so that a sensor named
/// "fault" (sensor/fault/range) still takes faults.
bool faultSignal(std::string_view name, std::size_t baseSize) {
    return name.substr(baseSize).starts_with(kFaultSegment);
}

struct ModeName {
    FaultMode mode;
    std::string_view name;
};
constexpr std::array<ModeName, 6> kModes{{{FaultMode::Dropout, "dropout"},
                                          {FaultMode::Stuck, "stuck"},
                                          {FaultMode::Bias, "bias"},
                                          {FaultMode::Drift, "drift"},
                                          {FaultMode::Noise, "noise"},
                                          {FaultMode::Disconnect, "disconnect"}}};

/// The keys each mode takes besides at and for (disconnect: at only).
std::string_view keysOf(FaultMode mode) {
    switch (mode) {
        case FaultMode::Dropout:
            return "at, for, probability, nan";
        case FaultMode::Stuck:
            return "at, for, component";
        case FaultMode::Bias:
            return "value, at, for, component";
        case FaultMode::Drift:
            return "rate, at, for, component";
        case FaultMode::Noise:
            return "sigma, at, for, component";
        case FaultMode::Disconnect:
            return "at, nan";
    }
    return {};
}

bool numericMode(FaultMode mode) {
    return mode == FaultMode::Bias || mode == FaultMode::Drift || mode == FaultMode::Noise;
}

std::string number(double value) {
    std::array<char, 32> text{};
    const auto [end, error] = std::to_chars(text.data(), text.data() + text.size(), value);
    return error == std::errc{} ? std::string(text.data(), end) : std::string("nan");
}

/// A fault's window in ns of simulated time: [begin, end).
struct Window {
    std::int64_t begin{0}, end{std::numeric_limits<std::int64_t>::max()};
};
Window windowOf(const SensorFault& fault) {
    Window window;
    constexpr double kLongest = 9e9;  // s: about 285 years, well inside int64 ns
    window.begin = std::llround(std::min(fault.at, kLongest) * 1e9);
    if (fault.duration < kLongest) {
        window.end = window.begin + std::llround(fault.duration * 1e9);
    }
    return window;
}

/// A module's outputs with the faults applied before they commit.
class Faulted final : public IModule {
public:
    Faulted(std::unique_ptr<IModule> inner, std::vector<SensorFault> faults)
        : inner_(std::move(inner)), faults_(std::move(faults)) {
        for (const SensorFault& fault : faults_) {
            windows_.push_back(windowOf(fault));
        }
    }

    [[nodiscard]] ModuleInfo info() const override { return inner_->info(); }
    [[nodiscard]] Duration preferredStep() const override { return inner_->preferredStep(); }

    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override {
        targets_.clear();
        groups_.clear();
        cleans_.clear();
        if (Status status = inner_->declare(world, config, declarer); !status.ok()) {
            return status;
        }
        const std::vector<SignalSpec> outputs(declarer.outputs().begin(), declarer.outputs().end());
        for (std::size_t f = 0; f < faults_.size(); ++f) {
            const SensorFault& fault = faults_[f];
            const std::string base = "sensor/" + fault.sensor + "/";
            for (const SignalSpec& spec : outputs) {
                if (!spec.name.starts_with(base) || faultSignal(spec.name, base.size()) ||
                    (fault.quantity != "*" && spec.name.substr(base.size()) != fault.quantity)) {
                    continue;
                }
                Target target;
                target.fault = f;
                target.name = spec.name;
                target.handle = isHandle(spec.type);
                target.components = componentCount(spec.type, spec.arrayLength);
                if (target.handle && numericMode(fault.mode)) {
                    return Status::error(StatusCode::InvalidConfig,
                                         "sensor fault " + to_string(fault) + ": " + spec.name +
                                             " is an image or a point buffer; " +
                                             std::string(to_string(fault.mode)) +
                                             " applies to numbers only");
                }
                if (!target.handle && fault.component >= 0 &&
                    static_cast<std::size_t>(fault.component) >= target.components) {
                    return Status::error(StatusCode::InvalidConfig,
                                         "sensor fault " + to_string(fault) + ": " + spec.name +
                                             " has " + std::to_string(target.components) +
                                             " components");
                }
                target.group = groupOf(fault.sensor);
                if (!target.handle &&
                    std::none_of(cleans_.begin(), cleans_.end(),
                                 [&](const Clean& c) { return c.name == spec.name; })) {
                    cleans_.push_back({spec.name, {}, {}});
                }
                targets_.push_back(std::move(target));
            }
        }
        for (Group& group : groups_) {
            const std::string base = "sensor/" + group.sensor + "/fault/";
            SignalSpec active;
            active.name = base + "active";
            active.type = SignalType::Bool;
            active.unit = units::one;
            group.active = declarer.output(std::move(active));
            SignalSpec dropped;
            dropped.name = base + "dropped";
            dropped.type = SignalType::Int;
            dropped.unit = units::one;
            group.dropped = declarer.output(std::move(dropped));
        }
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override {
        if (Status status = inner_->init(world, config, bus); !status.ok()) {
            return status;
        }
        for (Target& target : targets_) {
            target.id = bus.find(target.name);  // init is the cold path
            target.frozen = false;
            target.frozenValues.clear();
            target.frozenBuffer.reset();
        }
        for (Clean& clean : cleans_) {
            clean.id = bus.find(clean.name);
            clean.values.clear();
        }
        for (Group& group : groups_) {
            group.droppedCount = 0;
            bus.write(group.active, 0.0);
            bus.write(group.dropped, 0.0);
        }
        open_.assign(faults_.size(), 0);
        drop_.assign(faults_.size(), 0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (targets_.empty()) {
            return inner_->step(context, bus);
        }
        // What the sensor wrote last, without the faults: a module that does not write every
        // step leaves it as it was, and the faults apply to it afresh, never twice.
        for (const Clean& clean : cleans_) {
            if (!clean.values.empty()) {
                const std::span<double> pending = bus.writeBatch(clean.id);
                std::copy_n(clean.values.begin(), std::min(pending.size(), clean.values.size()),
                            pending.begin());
            }
        }
        Status status = inner_->step(context, bus);
        if (!status.ok()) {
            return status;
        }
        for (Clean& clean : cleans_) {
            const std::span<const double> pending = bus.writeBatch(clean.id);
            clean.values.assign(pending.begin(), pending.end());
        }
        // Samples are stamped at the end of the step, when they commit.
        const std::int64_t t = context.t.ns + context.dt.ns;
        for (std::size_t f = 0; f < faults_.size(); ++f) {
            const SensorFault& fault = faults_[f];
            open_[f] = t >= windows_[f].begin && t < windows_[f].end ? 1 : 0;
            drop_[f] = 0;
            if (open_[f] != 0 &&
                (fault.mode == FaultMode::Dropout || fault.mode == FaultMode::Disconnect)) {
                // One draw per sample for all of the fault's quantities: they go together.
                random::Stream draw(context.seed, context.stepIndex,
                                    kFaultStreams + (2U * static_cast<std::uint32_t>(f)));
                drop_[f] = fault.probability >= 1.0 || draw.uniform() < fault.probability ? 1 : 0;
            }
        }
        for (Group& group : groups_) {
            group.open = false;
            group.droppedNow = false;
        }
        for (Target& target : targets_) {
            const SensorFault& fault = faults_[target.fault];
            Group& group = groups_[target.group];
            if (open_[target.fault] == 0) {
                target.frozen = false;  // a stuck sensor that recovers reads live again
                continue;
            }
            group.open = true;
            if (bus.held(target.id)) {
                continue;  // no new sample this step (or an earlier fault dropped it)
            }
            apply(fault, target, group, context, t, bus);
        }
        for (Group& group : groups_) {
            if (group.droppedNow) {
                ++group.droppedCount;
            }
            bus.write(group.active, group.open ? 1.0 : 0.0);
            bus.write(group.dropped, static_cast<double>(group.droppedCount));
        }
        return status;
    }

    void shutdown() override { inner_->shutdown(); }

    Status saveState(StateWriter& writer) const override {
        StateWriter inner;
        if (Status status = inner_->saveState(inner); !status.ok()) {
            return status;
        }
        writer.put<std::uint64_t>(inner.bytes().size());
        writer.putBytes(inner.bytes().data(), inner.bytes().size());
        for (const Group& group : groups_) {
            writer.put(group.droppedCount);
        }
        for (const Clean& clean : cleans_) {
            writer.putDoubles(clean.values);
        }
        for (const Target& target : targets_) {
            writer.put(target.frozen);
            if (target.handle) {
                const bool has = target.frozenBuffer != nullptr;
                writer.put(has);
                if (has) {
                    const Buffer& buffer = *target.frozenBuffer;
                    writer.put(buffer.width);
                    writer.put(buffer.height);
                    writer.putString(buffer.encoding);
                    writer.put<std::uint64_t>(buffer.bytes.size());
                    writer.putBytes(buffer.bytes.data(), buffer.bytes.size());
                }
            } else {
                writer.putDoubles(target.frozenValues);
            }
        }
        return {};
    }

    Status restoreState(StateReader& reader) override {
        const auto bad = [] {
            return Status::error(StatusCode::InvalidConfig, "sensor faults: a bad saved state");
        };
        std::uint64_t size = 0;
        if (!reader.get(size) || size > reader.remaining()) {
            return bad();
        }
        std::vector<std::byte> blob(size);
        reader.getBytes(blob.data(), blob.size());
        StateReader inner(blob);
        if (Status status = inner_->restoreState(inner); !status.ok()) {
            return status;
        }
        for (Group& group : groups_) {
            reader.get(group.droppedCount);
        }
        for (Clean& clean : cleans_) {
            std::uint64_t count = 0;
            if (!reader.get(count) || count * sizeof(double) > reader.remaining()) {
                return bad();
            }
            clean.values.resize(count);
            reader.getBytes(clean.values.data(), count * sizeof(double));
        }
        for (Target& target : targets_) {
            reader.get(target.frozen);
            if (target.handle) {
                bool has = false;
                reader.get(has);
                target.frozenBuffer.reset();
                if (has) {
                    auto buffer = std::make_shared<Buffer>();
                    std::uint64_t bytes = 0;
                    reader.get(buffer->width);
                    reader.get(buffer->height);
                    reader.getString(buffer->encoding);
                    if (!reader.get(bytes) || bytes > reader.remaining()) {
                        return bad();
                    }
                    buffer->bytes.resize(bytes);
                    reader.getBytes(buffer->bytes.data(), bytes);
                    target.frozenBuffer = std::move(buffer);
                }
            } else {
                std::uint64_t count = 0;
                if (!reader.get(count) || count * sizeof(double) > reader.remaining()) {
                    return bad();
                }
                target.frozenValues.resize(count);
                reader.getBytes(target.frozenValues.data(), count * sizeof(double));
            }
        }
        return reader.failed() ? bad() : Status{};
    }

    void shiftOrigin(const std::array<double, 3>& shift) override { inner_->shiftOrigin(shift); }

private:
    struct Target {
        std::size_t fault{0};
        std::string name;
        SignalId id;
        bool handle{false};
        std::size_t components{0};
        std::uint32_t group{0};
        bool frozen{false};                          // stuck: the value is taken
        std::vector<double> frozenValues;            // every environment's
        std::shared_ptr<const Buffer> frozenBuffer;  // or the buffer
    };
    struct Clean {  // a faulted numeric output
        std::string name;
        SignalId id;
        std::vector<double> values;  // as the sensor wrote it last, every environment's
    };
    struct Group {  // a faulted sensor
        std::string sensor;
        SignalId active, dropped;
        std::int64_t droppedCount{0};
        bool open{false}, droppedNow{false};  // during step
    };

    std::uint32_t groupOf(const std::string& sensor) {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            if (groups_[g].sensor == sensor) {
                return static_cast<std::uint32_t>(g);
            }
        }
        Group group;
        group.sensor = sensor;
        groups_.push_back(std::move(group));
        return static_cast<std::uint32_t>(groups_.size() - 1);
    }

    /// The components a fault touches: one, or all of them.
    static std::pair<std::size_t, std::size_t> span(const SensorFault& fault,
                                                    std::size_t components) {
        return fault.component >= 0 ? std::pair{static_cast<std::size_t>(fault.component),
                                                static_cast<std::size_t>(fault.component) + 1}
                                    : std::pair{std::size_t{0}, components};
    }

    void apply(const SensorFault& fault, Target& target, Group& group, const StepContext& context,
               std::int64_t t, SignalBus& bus) {
        if (fault.mode == FaultMode::Dropout || fault.mode == FaultMode::Disconnect) {
            if (drop_[target.fault] == 0) {
                return;
            }
            group.droppedNow = true;
            if (fault.nan && !target.handle) {
                const std::span<double> values = bus.writeBatch(target.id);
                std::fill(values.begin(), values.end(), std::numeric_limits<double>::quiet_NaN());
            } else {
                bus.hold(target.id);  // no new sample: its stamp and readers stay as they were
            }
            return;
        }
        if (fault.mode == FaultMode::Stuck) {
            if (!target.frozen) {
                // The last value delivered: what the bus holds from the previous commit.
                target.frozen = true;
                if (target.handle) {
                    target.frozenBuffer = bus.buffer(bus.readBuffer(target.id));
                } else {
                    const std::span<const double> last = bus.readBatch(target.id);
                    target.frozenValues.assign(last.begin(), last.end());
                }
            }
            if (target.handle) {
                bus.publishBuffer(target.id, target.frozenBuffer);
                return;
            }
            const std::span<double> values = bus.writeBatch(target.id);
            const auto [first, last] = span(fault, target.components);
            for (std::size_t at = 0; at + target.components <= values.size() &&
                                     at + target.components <= target.frozenValues.size();
                 at += target.components) {
                for (std::size_t c = first; c < last; ++c) {
                    values[at + c] = target.frozenValues[at + c];
                }
            }
            return;
        }
        // Bias, drift and noise: added to the sample.
        const std::span<double> values = bus.writeBatch(target.id);
        const auto [first, last] = span(fault, target.components);
        random::Stream noise(context.seed, context.stepIndex,
                             kFaultStreams + (2U * static_cast<std::uint32_t>(target.fault)) + 1U);
        const double offset =
            fault.mode == FaultMode::Bias ? fault.value
            : fault.mode == FaultMode::Drift
                ? fault.rate * (static_cast<double>(t - windows_[target.fault].begin) * 1e-9)
                : 0.0;
        for (std::size_t at = 0; at + target.components <= values.size(); at += target.components) {
            for (std::size_t c = first; c < last; ++c) {
                values[at + c] +=
                    fault.mode == FaultMode::Noise ? fault.sigma * noise.normal() : offset;
            }
        }
    }

    std::unique_ptr<IModule> inner_;
    std::vector<SensorFault> faults_;
    std::vector<Window> windows_;  // per fault
    std::vector<Target> targets_;
    std::vector<Clean> cleans_;
    std::vector<Group> groups_;
    std::vector<std::uint8_t> open_, drop_;  // per fault, during step
};

bool parseNumber(std::string_view text, double& out) {
    const std::string copy(text);
    char* end = nullptr;
    out = std::strtod(copy.c_str(), &end);
    return !copy.empty() && end != nullptr && *end == '\0' && !std::isnan(out);
}

}  // namespace

std::string_view to_string(FaultMode mode) {
    for (const ModeName& m : kModes) {
        if (m.mode == mode) {
            return m.name;
        }
    }
    return "?";
}

std::string checkSensorFault(const SensorFault& fault) {
    const std::string mode(to_string(fault.mode));
    if (fault.sensor.empty() || fault.quantity.empty()) {
        return "a fault needs a sensor and a quantity (or *)";
    }
    if (!std::isfinite(fault.at) || fault.at < 0.0) {
        return "at is the start in seconds, 0 or more";
    }
    if (!(fault.duration > 0.0)) {
        return "for is a duration in seconds, more than 0";
    }
    if (fault.component < -1) {
        return "component is 0, 1, ...";
    }
    if (numericMode(fault.mode) && fault.quantity == "*") {
        return mode + " needs one quantity (NAME:QUANTITY=" + mode +
               "): its size is in that quantity's unit";
    }
    switch (fault.mode) {
        case FaultMode::Dropout:
            if (!(fault.probability > 0.0 && fault.probability <= 1.0)) {
                return "probability is in (0, 1]";
            }
            break;
        case FaultMode::Disconnect:
            if (!std::isinf(fault.duration)) {
                return "disconnect is for good: use dropout with for= for a window";
            }
            break;
        case FaultMode::Bias:
            if (!std::isfinite(fault.value)) {
                return "bias needs value=, a finite number";
            }
            break;
        case FaultMode::Drift:
            if (!std::isfinite(fault.rate)) {
                return "drift needs rate=, a finite number per second";
            }
            break;
        case FaultMode::Noise:
            if (!(fault.sigma > 0.0) || !std::isfinite(fault.sigma)) {
                return "noise needs sigma=, more than 0";
            }
            break;
        case FaultMode::Stuck:
            break;
    }
    return {};
}

Expected<SensorFault, std::string> parseSensorFault(std::string_view text) {
    const std::string usage =
        "--sensor-fault needs NAME:QUANTITY=MODE[,key=value...] (MODE dropout, stuck, bias, "
        "drift, noise or disconnect), not \"" +
        std::string(text) + "\"";
    const std::size_t equals = text.find('=');
    if (equals == std::string_view::npos || equals == 0) {
        return Unexpected{usage};
    }
    SensorFault fault;
    const std::string_view target = text.substr(0, equals);
    const std::size_t colon = target.find(':');
    fault.sensor = std::string(target.substr(0, colon));
    fault.quantity = colon == std::string_view::npos ? "*" : std::string(target.substr(colon + 1));
    std::string_view rest = text.substr(equals + 1);
    const std::size_t comma = rest.find(',');
    const std::string_view mode = rest.substr(0, comma);
    const auto known = std::find_if(kModes.begin(), kModes.end(),
                                    [&](const ModeName& m) { return m.name == mode; });
    if (known == kModes.end()) {
        return Unexpected{usage};
    }
    fault.mode = known->mode;
    rest = comma == std::string_view::npos ? std::string_view{} : rest.substr(comma + 1);
    std::set<std::string, std::less<>> given;
    while (!rest.empty()) {
        const std::size_t next = rest.find(',');
        const std::string_view item = rest.substr(0, next);
        rest = next == std::string_view::npos ? std::string_view{} : rest.substr(next + 1);
        const std::size_t is = item.find('=');
        const std::string key(item.substr(0, is));
        double value = 0.0;
        const std::string allowed(keysOf(fault.mode));
        const bool takes = (", " + allowed + ", ").find(", " + key + ", ") != std::string::npos;
        if (is == std::string_view::npos || !takes) {
            return Unexpected{"--sensor-fault " + std::string(text) + ": " + std::string(mode) +
                              " takes " + allowed};
        }
        if (!parseNumber(item.substr(is + 1), value)) {
            const std::string_view word = item.substr(is + 1);
            if (key == "nan" && (word == "true" || word == "false")) {
                value = word == "true" ? 1.0 : 0.0;
            } else {
                return Unexpected{"--sensor-fault " + std::string(text) + ": " + key +
                                  " needs a number"};
            }
        }
        given.insert(key);
        if (key == "at") {
            fault.at = value;
        } else if (key == "for") {
            fault.duration = value;
        } else if (key == "probability") {
            fault.probability = value;
        } else if (key == "nan") {
            fault.nan = value != 0.0;
        } else if (key == "value") {
            fault.value = value;
        } else if (key == "rate") {
            fault.rate = value;
        } else if (key == "sigma") {
            fault.sigma = value;
        } else if (key == "component") {
            if (value != std::floor(value) || value < 0.0 || value > 1e6) {
                return Unexpected{"--sensor-fault " + std::string(text) +
                                  ": component is 0, 1, ..."};
            }
            fault.component = static_cast<int>(value);
        }
    }
    for (const auto& [m, key] :
         {std::pair{FaultMode::Bias, "value"}, std::pair{FaultMode::Drift, "rate"},
          std::pair{FaultMode::Noise, "sigma"}}) {
        if (fault.mode == m && !given.contains(key)) {
            return Unexpected{"--sensor-fault " + std::string(text) + ": " + std::string(mode) +
                              " needs " + key + "="};
        }
    }
    if (std::string problem = checkSensorFault(fault); !problem.empty()) {
        return Unexpected{"--sensor-fault " + std::string(text) + ": " + problem};
    }
    return fault;
}

std::string to_string(const SensorFault& fault) {
    std::string text = fault.sensor;
    if (fault.quantity != "*") {
        text += ":" + fault.quantity;
    }
    text += "=" + std::string(to_string(fault.mode));
    const SensorFault plain;
    if (fault.at != plain.at) {
        text += ",at=" + number(fault.at);
    }
    if (fault.duration != plain.duration) {
        text += ",for=" + number(fault.duration);
    }
    if (fault.mode == FaultMode::Dropout && fault.probability != plain.probability) {
        text += ",probability=" + number(fault.probability);
    }
    if (fault.nan) {
        text += ",nan=1";
    }
    if (fault.mode == FaultMode::Bias) {
        text += ",value=" + number(fault.value);
    }
    if (fault.mode == FaultMode::Drift) {
        text += ",rate=" + number(fault.rate);
    }
    if (fault.mode == FaultMode::Noise) {
        text += ",sigma=" + number(fault.sigma);
    }
    if (fault.component >= 0) {
        text += ",component=" + std::to_string(fault.component);
    }
    return text;
}

std::unique_ptr<IModule> withFaults(std::unique_ptr<IModule> module,
                                    std::vector<SensorFault> faults) {
    if (faults.empty() || !module) {
        return module;
    }
    return std::make_unique<Faulted>(std::move(module), std::move(faults));
}

std::string checkFaultsWired(const SignalBus& bus, std::span<const SensorFault> faults) {
    for (const SensorFault& fault : faults) {
        const std::string base = "sensor/" + fault.sensor + "/";
        std::string quantities;
        bool found = false;
        for (std::size_t i = 0; i < bus.signalCount(); ++i) {
            const std::string& name = bus.specAt(i).name;
            if (!name.starts_with(base) || faultSignal(name, base.size()) ||
                name.substr(base.size()).starts_with("delayed/")) {
                continue;
            }
            const std::string quantity = name.substr(base.size());
            found = found || fault.quantity == "*" || quantity == fault.quantity;
            quantities += (quantities.empty() ? "" : ", ") + quantity;
        }
        if (quantities.empty()) {
            return "sensor fault " + to_string(fault) + ": no sensor " + fault.sensor +
                   " writes anything in this run (no such sensor, or it is switched off)";
        }
        if (!found) {
            return "sensor fault " + to_string(fault) + ": sensor " + fault.sensor +
                   " has no quantity " + fault.quantity + "; it has " + quantities;
        }
    }
    return {};
}

}  // namespace lagrange::sensors
