#include "lagrange/record/v1/record.pb.h"
#include "lagrange/viewer/CompareLog.hpp"
#include "lagrange/viewer/v1/signals.pb.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

#include <mcap/reader.hpp>
#include <yaml-cpp/yaml.h>

namespace lagrange::viewer::compare {

namespace {

namespace v1 = lagrange::viewer::v1;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::string_view kSignalsTopic = "/lagrange/signals/";

// Driving phases (Sim vs real, "Error by driving phase"): the base's planar speed, smoothed
// over ±kSmoothBins bins, and its rate of change.
constexpr double kStoppedSpeed = 0.05;  // m/s: below this the robot stands
constexpr double kCruiseAccel = 0.10;   // m/s²: |dv/dt| below this is cruising
constexpr int kSmoothBins = 5;

/// Running sums per bin while a file is read.
struct Accumulator {
    std::size_t slot{0};
    std::vector<double> sum;
    std::vector<std::uint32_t> count;
};

void add(Accumulator& acc, std::size_t bin, std::uint32_t components, const double* values) {
    const std::size_t needed = (bin + 1) * components;
    if (acc.sum.size() < needed) {
        acc.sum.resize(needed, 0.0);
        acc.count.resize(needed, 0);
    }
    for (std::uint32_t c = 0; c < components; ++c) {
        if (std::isfinite(values[c])) {
            acc.sum[bin * components + c] += values[c];
            ++acc.count[bin * components + c];
        }
    }
}

std::vector<std::string> tokens(std::string_view name) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= name.size()) {
        const std::size_t end = name.find_first_of("/._", start);
        const std::string_view part =
            name.substr(start, end == std::string_view::npos ? name.npos : end - start);
        if (!part.empty()) {
            out.emplace_back(part);
        }
        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

std::string componentLabel(const LogSignal& signal, std::uint32_t component) {
    if (signal.components <= 1) {
        return signal.name;
    }
    if (component < signal.componentNames.size()) {
        return signal.name + "." + signal.componentNames[component];
    }
    return signal.name + "." + std::to_string(component);
}

std::string jsonString(std::string_view text) {
    std::string out = "\"";
    for (const char ch : text) {
        if (ch == '"' || ch == '\\') {
            out += '\\';
            out += ch;
        } else if (static_cast<unsigned char>(ch) < 0x20) {
            char escaped[8];
            std::snprintf(escaped, sizeof(escaped), "\\u%04x", static_cast<unsigned>(ch));
            out += escaped;
        } else {
            out += ch;
        }
    }
    return out + "\"";
}

std::string jsonNumber(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    char text[40];
    std::snprintf(text, sizeof(text), "%.9g", value);
    return text;
}

}  // namespace

const LogSignal* LogFile::find(std::string_view name) const {
    for (const LogSignal& signal : signals) {
        if (signal.name == name) {
            return &signal;
        }
    }
    return nullptr;
}

const char* toString(Phase phase) {
    switch (phase) {
        case Phase::Accelerate:
            return "accelerate";
        case Phase::Cruise:
            return "cruise";
        case Phase::Brake:
            return "brake";
        case Phase::Stopped:
            return "stopped";
        case Phase::Count:
            break;
    }
    return "";
}

LoadResult loadLog(const std::string& path, std::int64_t periodNs, std::uint32_t maxComponents) {
    LoadResult result;
    if (periodNs <= 0) {
        result.error = "the resample period must be positive";
        return result;
    }
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        result.error = path + ": no such file on this machine";
        return result;
    }
    mcap::McapReader reader;
    if (const mcap::Status status = reader.open(path); !status.ok()) {
        result.error = path + ": " + status.message;
        return result;
    }
    LogFile log;
    log.path = path;
    log.periodNs = periodNs;
    log.sizeBytes = std::filesystem::file_size(path, ec);
    std::map<std::uint32_t, Accumulator> byId;  // catalog id -> sums
    bool haveCatalog = false;
    std::string problem;
    const auto onProblem = [&problem](const mcap::Status& status) {
        if (problem.empty()) {
            problem = status.message;
        }
    };
    std::int64_t lastNs = 0;
    std::vector<double> value;
    for (const mcap::MessageView& view : reader.readMessages(onProblem)) {
        const std::string& topic = view.channel->topic;
        const auto* data = view.message.data;
        const auto size = static_cast<int>(view.message.dataSize);
        if (topic == "/lagrange/meta") {
            record::v1::RunMeta meta;
            if (meta.ParseFromArray(data, size)) {
                log.profileYaml = meta.profile_yaml();
                log.stagePath = meta.stage_path();
                log.generator = meta.generator();
                log.description = meta.description();
                log.synthetic = meta.synthetic();
                log.durationNs = meta.duration().ns();
            }
        } else if (topic == "/lagrange/catalog" && !haveCatalog) {
            v1::SignalCatalog catalog;
            if (!catalog.ParseFromArray(data, size)) {
                result.error = path + ": its signal catalog cannot be read";
                return result;
            }
            haveCatalog = true;
            for (const v1::SignalInfo& info : catalog.signals()) {
                const std::uint32_t components = std::max<std::uint32_t>(1, info.components());
                if (components > maxComponents) {
                    continue;
                }
                LogSignal signal;
                signal.name = info.name();
                signal.unit = info.unit().display();
                signal.writer = info.writer();
                signal.components = components;
                signal.nativeDtNs = info.native_dt_ns();
                signal.componentNames.assign(info.component_names().begin(),
                                             info.component_names().end());
                byId[info.id()].slot = log.signals.size();
                log.signals.push_back(std::move(signal));
            }
        } else if (std::string_view(topic).starts_with(kSignalsTopic) && haveCatalog) {
            v1::SignalBlock block;
            if (!block.ParseFromArray(data, size)) {
                result.error = path + ": a signal block on " + topic + " cannot be read";
                return result;
            }
            for (const v1::SignalRun& run : block.runs()) {
                const auto it = byId.find(run.signal());
                if (it == byId.end() || run.encoding() != v1::SAMPLE_ENCODING_F64_LE ||
                    run.mode() == v1::SignalSubscription::MODE_MINMAX) {
                    continue;
                }
                const LogSignal& signal = log.signals[it->second.slot];
                const std::uint32_t components = std::max<std::uint32_t>(1, run.components());
                const std::size_t count = run.sample_count();
                if (components != signal.components ||
                    run.samples().size() != count * components * sizeof(double)) {
                    continue;
                }
                const bool stamped = run.dt_ns() <= 0;
                if (stamped && run.timestamps_ns().size() != count * sizeof(std::int64_t)) {
                    continue;
                }
                value.resize(components);
                for (std::size_t i = 0; i < count; ++i) {
                    std::int64_t t = 0;
                    if (stamped) {
                        std::memcpy(&t, run.timestamps_ns().data() + i * sizeof(std::int64_t),
                                    sizeof(t));
                    } else {
                        t = run.t0().ns() + static_cast<std::int64_t>(i) * run.dt_ns();
                    }
                    if (t < 0) {
                        continue;
                    }
                    lastNs = std::max(lastNs, t);
                    std::memcpy(value.data(),
                                run.samples().data() + i * components * sizeof(double),
                                components * sizeof(double));
                    add(it->second, static_cast<std::size_t>(t / periodNs), components,
                        value.data());
                }
            }
        }
    }
    if (!problem.empty()) {
        result.error = path + ": " + problem;
        return result;
    }
    if (!haveCatalog) {
        result.error = path + ": no signal catalog: not a Lagrange recording";
        return result;
    }
    for (auto& [id, acc] : byId) {
        LogSignal& signal = log.signals[acc.slot];
        signal.samples.resize(acc.sum.size());
        for (std::size_t i = 0; i < acc.sum.size(); ++i) {
            signal.samples[i] = acc.count[i] == 0 ? kNaN : acc.sum[i] / acc.count[i];
        }
    }
    if (log.durationNs <= 0) {
        log.durationNs = lastNs;
    }
    result.log = std::move(log);
    return result;
}

Suite loadSuite(const std::string& path) {
    Suite suite;
    suite.path = path;
    suite.name = std::filesystem::path(path).stem().string();
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (root["suite"]) {
            suite.name = root["suite"].as<std::string>();
        }
        if (const YAML::Node method = root["method"]) {
            if (method["alignment"]) {
                const auto alignment = method["alignment"].as<std::string>();
                if (alignment != "cross-correlation" && alignment != "none") {
                    suite.error = "method.alignment is cross-correlation or none, not " + alignment;
                    return suite;
                }
                suite.method.align = alignment == "cross-correlation";
            }
            if (method["resample_hz"]) {
                const auto hz = method["resample_hz"].as<double>();
                if (!(hz > 0.0) || hz > 1e6) {
                    suite.error = "method.resample_hz must be in (0, 1e6]";
                    return suite;
                }
                suite.periodNs = std::max<std::int64_t>(1, std::llround(1e9 / hz));
            }
            if (method["max_lag_s"]) {
                suite.method.maxLagNs = std::llround(method["max_lag_s"].as<double>() * 1e9);
            }
            if (const YAML::Node window = method["window"];
                window && window.IsSequence() && window.size() == 2) {
                suite.method.windowBeginNs = std::llround(window[0].as<double>() * 1e9);
                suite.method.windowEndNs = std::llround(window[1].as<double>() * 1e9);
            }
        }
        const YAML::Node signals = root["signals"];
        if (!signals || !signals.IsSequence() || signals.size() == 0) {
            suite.error = "a suite lists its signals: signals: [{sim: ..., real: ..., limit: ...}]";
            return suite;
        }
        for (const YAML::Node& entry : signals) {
            Mapping mapping;
            if (!entry["sim"]) {
                suite.error = "every entry of signals names its sim signal";
                return suite;
            }
            mapping.sim = entry["sim"].as<std::string>();
            mapping.real = entry["real"] ? entry["real"].as<std::string>() : mapping.sim;
            if (entry["component"]) {
                mapping.component = entry["component"].as<std::uint32_t>();
            }
            if (entry["limit"]) {
                mapping.limit = entry["limit"].as<double>();
            }
            suite.mappings.push_back(std::move(mapping));
        }
    } catch (const YAML::Exception& e) {
        suite.error = path + ": " + e.what();
    }
    return suite;
}

Suite sharedSuite(const LogFile& sim, const LogFile& real, double limit) {
    Suite suite;
    suite.name = "shared signals";
    suite.periodNs = sim.periodNs;
    for (const LogSignal& s : sim.signals) {
        if (s.components != 1 || s.name.starts_with("scene/") || s.writer == "recorder") {
            continue;
        }
        const LogSignal* r = real.find(s.name);
        if (r == nullptr || r->components != 1) {
            continue;
        }
        // Constant in both (a flag that never changes): nothing to compare.
        const auto spread = [](const LogSignal& signal) {
            double lo = std::numeric_limits<double>::infinity();
            double hi = -lo;
            for (const double v : signal.samples) {
                if (std::isfinite(v)) {
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
            }
            return hi > lo ? hi - lo : 0.0;
        };
        if (spread(s) == 0.0 && spread(*r) == 0.0) {
            continue;
        }
        suite.mappings.push_back({s.name, r->name, 0, limit});
    }
    return suite;
}

std::vector<Unmapped> unmapped(const LogFile& sim, const LogFile& real, const Suite& suite) {
    std::set<std::string_view> used;
    for (const Mapping& m : suite.mappings) {
        used.insert(m.real);
    }
    std::vector<Unmapped> out;
    for (const LogSignal& r : real.signals) {
        if (used.contains(r.name) || r.name.starts_with("scene/") || r.writer == "recorder") {
            continue;
        }
        Unmapped entry{r.name, {}};
        if (sim.find(r.name) != nullptr) {
            entry.suggestion = r.name;
        } else {
            const std::vector<std::string> mine = tokens(r.name);
            std::size_t best = 0;
            for (const LogSignal& s : sim.signals) {
                if (s.name.starts_with("scene/")) {
                    continue;
                }
                std::size_t score = 0;
                for (const std::string& token : tokens(s.name)) {
                    score += static_cast<std::size_t>(std::count(mine.begin(), mine.end(), token));
                }
                if (score > best) {
                    best = score;
                    entry.suggestion = s.name;
                }
            }
            if (best < 2) {
                entry.suggestion.clear();  // one shared word ("joint") is not a suggestion
            }
        }
        out.push_back(std::move(entry));
    }
    return out;
}

std::vector<Phase> drivingPhases(const LogFile& sim) {
    const LogSignal* twist = nullptr;
    for (const LogSignal& s : sim.signals) {
        if (s.name.starts_with("body/") && s.name.ends_with("/twist") && s.components >= 2) {
            twist = &s;
            break;
        }
    }
    if (twist == nullptr || twist->count() == 0) {
        return {};
    }
    const std::size_t n = twist->count();
    std::vector<double> speed(n, kNaN);
    for (std::size_t k = 0; k < n; ++k) {
        speed[k] = std::hypot(twist->at(k, 0), twist->at(k, 1));
    }
    std::vector<double> smooth(n, kNaN);
    for (std::size_t k = 0; k < n; ++k) {
        double sum = 0.0;
        int count = 0;
        const std::size_t from = k >= kSmoothBins ? k - kSmoothBins : 0;
        const std::size_t to = std::min(n - 1, k + kSmoothBins);
        for (std::size_t j = from; j <= to; ++j) {
            if (std::isfinite(speed[j])) {
                sum += speed[j];
                ++count;
            }
        }
        smooth[k] = count > 0 ? sum / count : kNaN;
    }
    const double period = static_cast<double>(sim.periodNs) * 1e-9;
    std::vector<Phase> phases(n, Phase::Cruise);
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t a = k > 0 ? k - 1 : k;
        const std::size_t b = std::min(n - 1, k + 1);
        const double accel =
            b > a ? (smooth[b] - smooth[a]) / (static_cast<double>(b - a) * period) : 0.0;
        if (!(smooth[k] >= kStoppedSpeed)) {
            phases[k] = Phase::Stopped;
        } else if (accel > kCruiseAccel) {
            phases[k] = Phase::Accelerate;
        } else if (accel < -kCruiseAccel) {
            phases[k] = Phase::Brake;
        }
    }
    return phases;
}

Result compareSignals(const LogFile& sim, const LogFile& real, const Mapping& mapping,
                      const Method& method, const std::vector<Phase>& phases) {
    Result result;
    result.real = mapping.real;
    result.limit = mapping.limit;
    for (double& value : result.phaseNrmse) {
        value = kNaN;
    }
    const LogSignal* s = sim.find(mapping.sim);
    const LogSignal* r = real.find(mapping.real);
    if (s == nullptr || r == nullptr || mapping.component >= s->components ||
        mapping.component >= r->components || sim.periodNs != real.periodNs || sim.periodNs <= 0) {
        result.sim = mapping.sim;
        return result;
    }
    result.sim = componentLabel(*s, mapping.component);
    if (r->components > 1) {
        result.real = componentLabel(*r, mapping.component);
    }
    result.unit = s->unit;
    const std::uint32_t c = mapping.component;
    const auto period = sim.periodNs;
    const auto n = static_cast<std::int64_t>(std::min(s->count(), r->count()));
    std::int64_t begin = std::clamp<std::int64_t>(method.windowBeginNs / period, 0, n);
    std::int64_t end = method.windowEndNs > 0
                           ? std::clamp<std::int64_t>(method.windowEndNs / period, begin, n)
                           : n;
    if (end - begin < 2) {
        return result;
    }
    const auto simAt = [&](std::int64_t k) { return s->at(static_cast<std::size_t>(k), c); };
    const auto realAt = [&](std::int64_t k) {
        const auto count = static_cast<std::int64_t>(r->count());
        return k >= 0 && k < count ? r->at(static_cast<std::size_t>(k), c) : kNaN;
    };

    // Alignment: the lag that maximises the normalised cross-correlation of the two
    // mean-removed series over the window.
    std::int64_t lag = 0;
    if (method.align) {
        double simMean = 0.0;
        double realMean = 0.0;
        std::size_t count = 0;
        for (std::int64_t k = begin; k < end; ++k) {
            if (std::isfinite(simAt(k)) && std::isfinite(realAt(k))) {
                simMean += simAt(k);
                realMean += realAt(k);
                ++count;
            }
        }
        if (count > 0) {
            simMean /= static_cast<double>(count);
            realMean /= static_cast<double>(count);
            const std::int64_t maxLag = std::max<std::int64_t>(0, method.maxLagNs / period);
            double best = -std::numeric_limits<double>::infinity();
            for (std::int64_t l = -maxLag; l <= maxLag; ++l) {
                double sum = 0.0;
                double ss = 0.0;
                double rr = 0.0;
                for (std::int64_t k = begin; k < end; ++k) {
                    const double a = simAt(k);
                    const double b = realAt(k + l);
                    if (std::isfinite(a) && std::isfinite(b)) {
                        sum += (a - simMean) * (b - realMean);
                        ss += (a - simMean) * (a - simMean);
                        rr += (b - realMean) * (b - realMean);
                    }
                }
                const double score = ss > 0.0 && rr > 0.0 ? sum / std::sqrt(ss * rr) : 0.0;
                // Ties (flat series) keep the smallest lag.
                if (score > best + 1e-12 ||
                    (std::abs(score - best) <= 1e-12 && std::abs(l) < std::abs(lag))) {
                    best = score;
                    lag = l;
                }
            }
        }
    }
    result.lagNs = lag * period;

    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    double squares = 0.0;
    double phaseSquares[static_cast<int>(Phase::Count)]{};
    for (std::int64_t k = begin; k < end; ++k) {
        const double a = simAt(k);
        const double b = realAt(k + lag);
        if (!std::isfinite(a) || !std::isfinite(b)) {
            continue;
        }
        const double e = b - a;
        lo = std::min(lo, b);
        hi = std::max(hi, b);
        squares += e * e;
        if (std::abs(e) > result.maxAbs || result.samples == 0) {
            result.maxAbs = std::abs(e);
            result.worstNs = k * period;
            result.simAtWorst = a;
            result.realAtWorst = b;
        }
        ++result.samples;
        if (static_cast<std::size_t>(k) < phases.size()) {
            const auto p = static_cast<int>(phases[static_cast<std::size_t>(k)]);
            phaseSquares[p] += e * e;
            ++result.phaseSamples[p];
        }
        result.t.push_back(static_cast<double>(k * period) * 1e-9);
        result.simValues.push_back(a);
        result.realValues.push_back(b);
        result.error.push_back(e);
    }
    if (result.samples < 2) {
        return result;
    }
    result.valid = true;
    result.rmse = std::sqrt(squares / static_cast<double>(result.samples));
    result.range = hi - lo;
    constexpr double kTiny = 1e-12;
    const auto normalise = [&](double rmse) {
        if (result.range > kTiny) {
            return rmse / result.range;
        }
        return rmse <= kTiny ? 0.0 : std::numeric_limits<double>::infinity();
    };
    result.nrmse = normalise(result.rmse);
    result.pass = result.nrmse <= result.limit;
    for (int p = 0; p < static_cast<int>(Phase::Count); ++p) {
        if (result.phaseSamples[p] > 0) {
            result.phaseNrmse[p] =
                normalise(std::sqrt(phaseSquares[p] / static_cast<double>(result.phaseSamples[p])));
        }
    }
    return result;
}

std::size_t Report::passed() const {
    return static_cast<std::size_t>(
        std::count_if(results.begin(), results.end(), [](const Result& r) { return r.pass; }));
}

Report runSuite(const LogFile& sim, const LogFile& real, const Suite& suite) {
    Report report;
    report.suite = suite.name;
    report.simPath = sim.path;
    report.realPath = real.path;
    const std::vector<Phase> phases = drivingPhases(sim);
    for (const Mapping& mapping : suite.mappings) {
        report.results.push_back(compareSignals(sim, real, mapping, suite.method, phases));
    }
    return report;
}

std::string toJson(const Report& report, const Suite& suite) {
    std::ostringstream out;
    out << "{\n  \"suite\": " << jsonString(report.suite)
        << ",\n  \"suite_file\": " << jsonString(suite.path)
        << ",\n  \"sim\": " << jsonString(report.simPath)
        << ",\n  \"real\": " << jsonString(report.realPath) << ",\n  \"method\": {"
        << "\"alignment\": " << jsonString(suite.method.align ? "cross-correlation" : "none")
        << ", \"resample_hz\": " << jsonNumber(1e9 / static_cast<double>(suite.periodNs))
        << ", \"metric\": \"nrmse_range\", \"window_s\": ["
        << jsonNumber(static_cast<double>(suite.method.windowBeginNs) * 1e-9) << ", "
        << jsonNumber(static_cast<double>(suite.method.windowEndNs) * 1e-9) << "]},\n"
        << "  \"passed\": " << report.passed() << ",\n  \"total\": " << report.results.size()
        << ",\n  \"results\": [";
    for (std::size_t i = 0; i < report.results.size(); ++i) {
        const Result& r = report.results[i];
        out << (i == 0 ? "\n" : ",\n") << "    {\"sim\": " << jsonString(r.sim)
            << ", \"real\": " << jsonString(r.real) << ", \"unit\": " << jsonString(r.unit)
            << ", \"valid\": " << (r.valid ? "true" : "false") << ", \"samples\": " << r.samples
            << ", \"rmse\": " << jsonNumber(r.rmse) << ", \"nrmse\": " << jsonNumber(r.nrmse)
            << ", \"max_abs\": " << jsonNumber(r.maxAbs)
            << ", \"lag_s\": " << jsonNumber(static_cast<double>(r.lagNs) * 1e-9)
            << ", \"limit\": " << jsonNumber(r.limit)
            << ", \"pass\": " << (r.pass ? "true" : "false") << "}";
    }
    out << "\n  ]\n}\n";
    return out.str();
}

}  // namespace lagrange::viewer::compare
