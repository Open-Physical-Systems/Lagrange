#pragma once

// Comparing two recordings (docs/design/ui/08-replay/SPEC.md: Compare, Sim vs real,
// Validation). Both files are Lagrange recordings (MCAP, docs/design/formats.md §7) on the
// viewer's machine: the one the server replays and a second one, "the real log", which may be
// a second simulated run (another engine, another profile) or a log converted into a
// recording. The viewer reads them as an open file format, as Foxglove or Python would; it
// never links a simulation module (invariant 14). No ImGui here: testable without a window.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::compare {

/// One recorded signal, resampled onto a uniform grid t = k · periodNs by averaging every
/// sample of the bin [k·period, (k+1)·period). Bins without a sample are NaN.
struct LogSignal {
    std::string name;
    std::string unit;    ///< Display form ("N·m").
    std::string writer;  ///< "rigid_body/mujoco"
    std::uint32_t components{1};
    std::vector<std::string> componentNames;  ///< May be empty.
    std::int64_t nativeDtNs{0};               ///< The writer's step.
    std::vector<double> samples;              ///< bin-major: samples[k * components + c]

    [[nodiscard]] std::size_t count() const {
        return components == 0 ? 0 : samples.size() / components;
    }
    [[nodiscard]] double at(std::size_t k, std::uint32_t component) const {
        return samples[k * components + component];
    }
};

/// A recording read for comparison.
struct LogFile {
    std::string path;
    std::string profileYaml;  ///< RunMeta: the profile the run used; may be empty.
    std::string stagePath;
    std::string generator;
    std::string description;
    bool synthetic{false};
    std::int64_t durationNs{0};
    std::int64_t periodNs{0};  ///< The grid the signals were resampled onto.
    std::uint64_t sizeBytes{0};
    std::vector<LogSignal> signals;

    [[nodiscard]] const LogSignal* find(std::string_view name) const;
};

struct LoadResult {
    std::optional<LogFile> log;
    std::string error;  ///< Why it could not be read; empty on success.
};

/// Reads a recording: its run facts and every signal of up to `maxComponents` components
/// (arrays such as lidar ranges are left out), resampled to `periodNs`.
[[nodiscard]] LoadResult loadLog(const std::string& path, std::int64_t periodNs,
                                 std::uint32_t maxComponents = 8);

/// How two series are compared (the Validation panel's Method section).
struct Method {
    bool align{true};                    ///< Cross-correlation alignment (else none).
    std::int64_t maxLagNs{500'000'000};  ///< Lags searched: ±0.5 s.
    std::int64_t windowBeginNs{0};       ///< Window of the run compared; end 0 = whole run.
    std::int64_t windowEndNs{0};
};

/// The driving phases of the Sim vs real panel, from the sim log's base speed.
enum class Phase : std::uint8_t { Accelerate, Cruise, Brake, Stopped, Count };
[[nodiscard]] const char* toString(Phase phase);

/// One sim signal component against one real signal component.
struct Result {
    std::string sim;   ///< "motor/left_drive_motor/current_q" or "body/base_link/twist.x"
    std::string real;  ///< The real log's name for it.
    std::string unit;
    double limit{0.05};  ///< NRMSE limit, as a fraction of the real range.
    bool valid{false};   ///< false: too few overlapping samples to say anything.
    std::size_t samples{0};
    double rmse{0.0};
    double range{0.0};  ///< max − min of the real series in the window.
    double nrmse{0.0};  ///< rmse / range (0 when both are constant and equal).
    double maxAbs{0.0};
    std::int64_t lagNs{0};  ///< real(t + lag) best matches sim(t).
    bool pass{false};
    std::int64_t worstNs{0};  ///< When |error| is largest.
    double simAtWorst{0.0};
    double realAtWorst{0.0};
    /// NRMSE per phase (normalised by the whole window's range); NaN when the phase has no
    /// sample.
    double phaseNrmse[static_cast<int>(Phase::Count)]{};
    std::size_t phaseSamples[static_cast<int>(Phase::Count)]{};
    // The aligned series, for the plots: t (s), sim, real and real − sim.
    std::vector<double> t, simValues, realValues, error;
};

/// One row of a validation suite.
struct Mapping {
    std::string sim;
    std::string real;
    std::uint32_t component{0};
    double limit{0.05};
};

/// A validation suite: what to compare and how (validation/<suite>.yaml).
struct Suite {
    std::string name;
    std::string path;  ///< Empty for the suite made from the signals both logs share.
    std::vector<Mapping> mappings;
    Method method;
    std::int64_t periodNs{10'000'000};
    std::string error;  ///< Why the file could not be read.
};

/// Reads a suite file:
///   suite: amr_drive_train
///   method: {alignment: cross-correlation | none, resample_hz: 100, max_lag_s: 0.5,
///            window: [begin_s, end_s]}
///   signals:
///     - {sim: motor/left_drive_motor/current_q, real: motor/left_drive_motor/current_q,
///        component: 0, limit: 0.05}
[[nodiscard]] Suite loadSuite(const std::string& path);

/// The suite of every scalar signal both logs have under the same name, except the
/// recorder's prim poses and series that are constant in both, each with `limit`.
[[nodiscard]] Suite sharedSuite(const LogFile& sim, const LogFile& real, double limit);

/// Topics of the real log that no mapping uses, each with the sim signal whose name is most
/// like it (empty when none is close).
struct Unmapped {
    std::string real;
    std::string suggestion;
};
[[nodiscard]] std::vector<Unmapped> unmapped(const LogFile& sim, const LogFile& real,
                                             const Suite& suite);

/// Compares one mapping. `phases` (may be empty) gives each grid bin's phase.
[[nodiscard]] Result compareSignals(const LogFile& sim, const LogFile& real, const Mapping& mapping,
                                    const Method& method, const std::vector<Phase>& phases);

/// The phase of each bin of `sim`, from the planar speed of its first body twist
/// ("body/<b>/twist"); empty when it has none.
[[nodiscard]] std::vector<Phase> drivingPhases(const LogFile& sim);

/// Runs a whole suite.
struct Report {
    std::string suite;
    std::string simPath, realPath;
    std::vector<Result> results;
    [[nodiscard]] std::size_t passed() const;
};
[[nodiscard]] Report runSuite(const LogFile& sim, const LogFile& real, const Suite& suite);

/// The report as JSON (Export report).
[[nodiscard]] std::string toJson(const Report& report, const Suite& suite);

}  // namespace lagrange::viewer::compare
