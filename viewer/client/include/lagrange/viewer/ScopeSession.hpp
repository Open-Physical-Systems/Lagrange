#pragma once

#include "lagrange/viewer/ScopeBuffer.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <array>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// One plotted signal component.
struct ScopeTrace {
    std::string name;  ///< Signal name in the catalog.
    std::uint32_t component{0};
    std::string label;   ///< Name with the component, unique among traces.
    std::string unit;    ///< Display unit; "1" for a pure number.
    std::string domain;  ///< "electrical", from the module that writes the signal.
    int color{0};        ///< Index into the series palette; kept while the trace exists.
    std::shared_ptr<ScopeBuffer> buffer;
};

/// What the Signals workspace plots, shared by its panels: the traces, the time window, the
/// cursors and the trigger. No drawing here, so it can be tested without a window.
class ScopeSession {
public:
    /// Window choices in seconds; 0 means the whole run held in the buffers.
    static constexpr std::array<double, 4> kWindowSeconds{1.0, 10.0, 60.0, 0.0};
    static constexpr int kColors = 8;  // size of the series palette

    /// The time range the plots show.
    struct View {
        std::int64_t beginNs{0};
        std::int64_t endNs{0};
        [[nodiscard]] double seconds() const { return static_cast<double>(endNs - beginNs) * 1e-9; }
    };

    struct Trigger {
        bool armed{false};
        int trace{-1};       ///< Index into traces(); -1 when no trigger is set.
        bool rising{false};  ///< Edge: rising through the level, or falling.
        double level{0.0};
        std::int64_t firedNs{-1};  ///< Time of the crossing that fired; -1 if none.
    };

    // --- traces ----------------------------------------------------------------------------
    /// Subscribes to a signal component and plots it. False if the catalog has no such
    /// signal, or it is plotted already.
    bool add(ViewerClient& client, const SignalCatalog& catalog, const std::string& name,
             std::uint32_t component = 0);
    /// Plots a buffer that is fed by someone else. For tests.
    void adopt(std::string name, std::string unit, std::shared_ptr<ScopeBuffer> buffer);
    void remove(ViewerClient& client, std::size_t index);
    /// Index of a plotted component, or -1.
    [[nodiscard]] int find(std::string_view name, std::uint32_t component = 0) const;
    [[nodiscard]] const std::vector<ScopeTrace>& traces() const { return traces_; }
    /// The distinct units, in the order their first trace was added: one plot per unit, so
    /// amperes and volts never share an axis.
    [[nodiscard]] const std::vector<std::string>& units() const { return units_; }

    // --- time ------------------------------------------------------------------------------
    /// Call once per frame: follows the newest sample and evaluates the trigger.
    void update();
    [[nodiscard]] View view() const;
    [[nodiscard]] bool paused() const { return paused_; }
    void setPaused(bool paused);
    int window{1};  ///< Index into kWindowSeconds.

    // --- cursors, in seconds before the right edge of the view (so <= 0) --------------------
    bool cursors{false};
    double cursorA{-6.6};
    double cursorB{-2.8};
    /// Turns the cursors on, placed inside the current view.
    void showCursors(bool on);
    /// Absolute time of a cursor position in the current view.
    [[nodiscard]] std::int64_t cursorTimeNs(double cursor) const;

    // --- trigger ---------------------------------------------------------------------------
    /// Arms a single-shot trigger: the scope pauses at the first crossing after now.
    void arm(int trace, bool rising, double level);
    void disarm();
    [[nodiscard]] const Trigger& trigger() const { return trigger_; }

    // --- what the last frame cost; written by the scope panel, shown by Statistics ----------
    std::size_t drawnPoints{0};  ///< Points handed to the plots after decimation.
    double plotMs{0.0};          ///< CPU time spent building the plots.

private:
    void rebuildUnits();
    void evaluateTrigger();

    std::vector<ScopeTrace> traces_;
    std::vector<std::string> units_;
    Trigger trigger_;
    std::int64_t newestNs_{0};
    std::int64_t frozenEndNs_{0};
    std::int64_t checkedNs_{0};  // trigger: samples before this time were examined
    double previous_{0.0};       // trigger: the last sample examined
    bool hasPrevious_{false};
    bool paused_{false};
};

/// Writes the raw (not decimated) samples of every trace in `view` as CSV:
/// signal,unit,time_s,value. Returns the number of sample rows written.
std::size_t writeCsv(std::ostream& out, const ScopeSession& session, ScopeSession::View view);

}  // namespace lagrange::viewer
