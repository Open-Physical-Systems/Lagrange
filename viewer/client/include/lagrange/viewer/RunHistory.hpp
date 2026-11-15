#pragma once

// What the Scene workspace's History and Layers tabs show (docs/design/ui/02-scene/SPEC.md),
// read from what the server already sends (invariant 14): its log lines (the edits it applied,
// the robots it spawned and deleted, the profiles and site values it took, the session layer it
// wrote) and the scene's stage path. The protocol has no layer stack and no undo; see the
// History and Layers panels for what is shown in their place.

#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::viewer {

/// One change to the run, as the server's log reported it.
struct HistoryItem {
    enum class Kind : std::uint8_t {
        Attribute,     ///< SetAttribute applied: `target` prim path, `name`, `before` -> `after`
        SessionSaved,  ///< the edited model written as a USD layer: `target` its file
        Spawn,         ///< a robot (or a person) added to the run: `target` name, `detail` file
        Delete,        ///< a robot removed from the run: `target` name
        Profile,       ///< a profile applied (the run restarted at t = 0)
        Site,          ///< a site value set at run time: `name` = `after`, or several together
        Reset,         ///< the run restarted at t = 0
    };
    Kind kind{Kind::Attribute};
    std::int64_t simTimeNs{0};
    std::string target;
    std::string name;
    std::string before;
    std::string after;
    std::string unit;
    std::string detail;
};

/// The change a log line reports, or nothing for any other line.
[[nodiscard]] std::optional<HistoryItem> historyItemOf(const LogEntry& entry);

/// A robot file composed into the run, beside the stage the run was loaded from.
struct RunRobot {
    std::string name;
    std::string path;
    bool spawned{false};  ///< added while running (else by --add-robot at load)
};

/// The run's changes, gathered from the log as it arrives. The log book keeps only its last
/// lines, so this keeps its own list; a new connection starts it again (the server resends what
/// its log still holds).
class RunHistory {
public:
    /// Takes the lines of `book` not seen yet. `connects` is ClientStats::connects: when it
    /// changes, everything is read again from the new connection's book. True when it changed.
    bool update(const LogBook& book, std::uint64_t connects);

    /// Oldest first.
    [[nodiscard]] const std::vector<HistoryItem>& items() const { return items_; }
    /// The robots in the run beside the stage, in the order they came; deleted ones removed.
    [[nodiscard]] const std::vector<RunRobot>& robots() const { return robots_; }
    /// The session layer's file, once the server wrote one; empty before.
    [[nodiscard]] const std::string& sessionLayer() const { return sessionLayer_; }
    /// Increases whenever anything above changed.
    [[nodiscard]] std::uint64_t generation() const { return generation_; }

private:
    void take(const LogEntry& entry);

    std::vector<HistoryItem> items_;
    std::vector<RunRobot> robots_;
    std::string sessionLayer_;
    std::uint64_t seen_{0};
    std::uint64_t connects_{~std::uint64_t{0}};
    std::uint64_t generation_{0};
};

/// One row of the Layers tab: strongest first, as USD orders a layer stack.
struct LayerRow {
    enum class Role : std::uint8_t { Session, Robot, Stage, Environment };
    Role role{Role::Stage};
    std::string label;  ///< file name, or what stands in for one
    std::string path;   ///< full path on the server's machine; empty when there is no file
    std::string detail;
};

/// The run's layers from what the protocol carries: the session layer (written or not), the
/// robots spawned or added, the stage the run was loaded from, and the environment layer.
[[nodiscard]] std::vector<LayerRow> layerRows(const std::string& stagePath,
                                              const std::string& environmentLayer,
                                              const RunHistory& history);

}  // namespace lagrange::viewer
