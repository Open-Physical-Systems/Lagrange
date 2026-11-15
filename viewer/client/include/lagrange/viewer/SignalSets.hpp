#pragma once

// The Signals workspace's saved sets (docs/design/ui/05-signals: "Saved sets · N"): named
// lists of plotted signal components, kept on the viewer's machine in its config directory.
// A convenience: a file that cannot be read is no sets.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::viewer {

struct SignalSet {
    std::string name;
    /// Signal name and component, in the order they were plotted.
    std::vector<std::pair<std::string, std::uint32_t>> signals;
};

/// One line per signal: set name, signal name, component, separated by tabs. The sets keep the
/// order of their first line.
[[nodiscard]] std::vector<SignalSet> loadSignalSets(const std::string& path);
/// Best effort: returns false when the file could not be written. Tabs and line breaks in a
/// name are written as spaces.
bool saveSignalSets(const std::string& path, const std::vector<SignalSet>& sets);

/// Puts `set` in `sets`, replacing the one of the same name. Returns its index.
std::size_t storeSignalSet(std::vector<SignalSet>& sets, SignalSet set);

}  // namespace lagrange::viewer
