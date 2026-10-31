#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/DelayLine.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <span>
#include <utility>

namespace lagrange::sensors {
namespace {

constexpr std::string_view kDelayed = "/delayed/";

double setting(const std::map<std::string, double, std::less<>>& settings, const char* key) {
    const auto it = settings.find(key);
    return it == settings.end() ? 0.0 : it->second;
}

}  // namespace

bool DelayLine::needed(const WorldView& world) {
    const auto& settings = world.sensors().settings;
    return std::any_of(settings.begin(), settings.end(), [](const auto& s) {
        return setting(s, "latency") > 0.0 || setting(s, "jitter") > 0.0;
    });
}

Status DelayLine::declare(const WorldView& world, const ModuleConfig& /*config*/,
                          SignalDeclarer& declarer) {
    sensors_.clear();
    lines_.clear();
    const auto& table = world.sensors();
    for (std::size_t s = 0; s < table.names.size(); ++s) {
        const double latency = setting(table.settings[s], "latency");
        const double jitter = setting(table.settings[s], "jitter");
        if (latency <= 0.0 && jitter <= 0.0) {
            continue;
        }
        const std::string base = "sensor/" + table.names[s] + "/";
        const std::vector<SignalSpec> outputs = declarer.declaredOutputs(base);
        if (outputs.empty()) {
            continue;  // switched off, or nothing simulates it
        }
        Sensor sensor{latency, jitter, {}};
        SignalSpec sampled;
        sampled.name = base + "delayed/stamp";
        sampled.unit = units::s;
        sensor.stamp = declarer.output(std::move(sampled));
        const auto index = static_cast<std::uint32_t>(sensors_.size());
        sensors_.push_back(sensor);
        for (const SignalSpec& spec : outputs) {
            if (spec.name.find(kDelayed) != std::string::npos) {
                continue;
            }
            SignalSpec late = spec;
            late.name = base + "delayed/" + spec.name.substr(base.size());
            Line line;
            line.in = declarer.input(spec);
            line.out = declarer.output(std::move(late));
            line.handle = isHandle(spec.type);
            line.sensor = index;
            line.last = SimTime{-1};
            line.lastDue = SimTime{-1};
            lines_.push_back(std::move(line));
        }
    }
    return {};
}

Status DelayLine::init(const WorldView& /*world*/, const ModuleConfig& /*config*/, SignalBus& bus) {
    for (const Sensor& sensor : sensors_) {
        bus.write(sensor.stamp, -1.0);
    }
    return {};
}

Status DelayLine::step(const StepContext& context, SignalBus& bus) {
    const SimTime end{context.t.ns + context.dt.ns};
    for (Sensor& sensor : sensors_) {
        sensor.arrived = false;
    }
    for (std::size_t l = 0; l < lines_.size(); ++l) {
        Line& line = lines_[l];
        Sensor& sensor = sensors_[line.sensor];

        const SimTime stamp = bus.stamp(line.in);
        if (stamp.ns != line.last.ns && stamp.ns > 0) {
            line.last = stamp;
            // Gaussian, cut at three sigma, never early: all of a sensor's signals draw the same
            // number for the same sample.
            random::Stream rng(context.seed, static_cast<std::uint64_t>(stamp.ns), line.sensor);
            const double late =
                sensor.latency + (sensor.jitter * std::clamp(rng.normal(), -3.0, 3.0));
            const auto delay = static_cast<std::int64_t>(std::llround(std::max(late, 0.0) * 1e9));
            Sample sample;
            sample.stamp = stamp;
            sample.due = SimTime{std::max(stamp.ns + delay, line.lastDue.ns)};
            line.lastDue = sample.due;
            if (line.handle) {
                sample.buffer = bus.buffer(bus.readBuffer(line.in));
            } else {
                const std::span<const double> values = bus.readN(line.in);
                sample.values.assign(values.begin(), values.end());
            }
            line.waiting.push_back(std::move(sample));
        }

        // The newest that is due; those it overtook within this step are never seen. A signal
        // commits only when a value arrives, so its bus stamp says when it did.
        if (line.waiting.empty() || line.waiting.front().due.ns > end.ns) {
            bus.hold(line.out);
        }
        while (!line.waiting.empty() && line.waiting.front().due.ns <= end.ns) {
            Sample& sample = line.waiting.front();
            if (line.handle) {
                if (line.waiting.size() == 1 || line.waiting[1].due.ns > end.ns) {
                    bus.publishBuffer(line.out, sample.buffer);
                }
            } else {
                bus.writeN(line.out, sample.values);
            }
            bus.write(sensor.stamp, sample.stamp.seconds());
            sensor.arrived = true;
            line.waiting.pop_front();
        }
    }
    for (const Sensor& sensor : sensors_) {
        if (!sensor.arrived) {
            bus.hold(sensor.stamp);
        }
    }
    return {};
}

Status DelayLine::saveState(StateWriter& writer) const {
    for (const Line& line : lines_) {
        writer.put(line.last.ns);
        writer.put(line.lastDue.ns);
        writer.put<std::uint64_t>(line.waiting.size());
        for (const Sample& sample : line.waiting) {
            writer.put(sample.stamp.ns);
            writer.put(sample.due.ns);
            if (line.handle) {
                const bool has = sample.buffer != nullptr;
                writer.put(has);
                if (has) {
                    writer.put(sample.buffer->width);
                    writer.put(sample.buffer->height);
                    writer.putString(sample.buffer->encoding);
                    writer.put<std::uint64_t>(sample.buffer->bytes.size());
                    writer.putBytes(sample.buffer->bytes.data(), sample.buffer->bytes.size());
                }
            } else {
                writer.putDoubles(sample.values);
            }
        }
    }
    return {};
}

Status DelayLine::restoreState(StateReader& reader) {
    for (Line& line : lines_) {
        std::uint64_t count = 0;
        reader.get(line.last.ns);
        reader.get(line.lastDue.ns);
        reader.get(count);
        line.waiting.clear();
        for (std::uint64_t i = 0; i < count && !reader.failed(); ++i) {
            Sample sample;
            reader.get(sample.stamp.ns);
            reader.get(sample.due.ns);
            if (line.handle) {
                bool has = false;
                reader.get(has);
                if (has) {
                    auto buffer = std::make_shared<Buffer>();
                    std::uint64_t size = 0;
                    reader.get(buffer->width);
                    reader.get(buffer->height);
                    reader.getString(buffer->encoding);
                    reader.get(size);
                    if (size > reader.remaining()) {
                        return Status::error(StatusCode::InvalidConfig,
                                             "delay line: a short saved state");
                    }
                    buffer->bytes.resize(size);
                    reader.getBytes(buffer->bytes.data(), size);
                    sample.buffer = std::move(buffer);
                }
            } else {
                std::uint64_t size = 0;
                reader.get(size);
                if (size * sizeof(double) > reader.remaining()) {
                    return Status::error(StatusCode::InvalidConfig,
                                         "delay line: a short saved state");
                }
                sample.values.resize(size);
                reader.getBytes(sample.values.data(), size * sizeof(double));
            }
            line.waiting.push_back(std::move(sample));
        }
    }
    return reader.failed() ? Status::error(StatusCode::InvalidConfig, "delay line: bad saved state")
                           : Status{};
}

}  // namespace lagrange::sensors
