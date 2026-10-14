#pragma once

#include "lagrange/viewer/ScopeSession.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lagrange::viewer {

/// Draws the plots of a ScopeSession: one per unit, stacked, on a shared time axis that runs
/// from minus the window to zero, with the legend, the trigger level and the cursors. Used by
/// the Signals scope and by the scopes of other workspaces.
class ScopePlotter {
public:
    /// Fills the rest of the current window with the session's plots, and records in the
    /// session how many points were drawn and how long building the plots took.
    void draw(ScopeSession& session);

private:
    void plots(ScopeSession& session);
    void drawTrace(const ScopeTrace& trace, ScopeSession::View view, std::size_t columns);
    static void legend(const ScopeSession& session, const std::string& unit);
    static void triggerLine(double level, const char* unit, double span);

    // Scratch arrays: they grow to the plot width once and are then reused.
    std::vector<ScopeBuffer::Column> columns_;
    std::vector<double> xs_;
    std::vector<double> low_;
    std::vector<double> high_;
    std::size_t drawn_{0};
};

/// The last two parts of a signal name: enough to tell traces apart in a legend.
[[nodiscard]] const char* shortSignalName(const std::string& label);

}  // namespace lagrange::viewer
