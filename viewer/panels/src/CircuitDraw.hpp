#pragma once

// A circuit drawn with its symbols: resistors, capacitors, batteries, grounds. The Circuit tab
// draws the running stage's, the Netlist tab the one being edited.

#include "lagrange/viewer/Circuit.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include <imgui.h>

namespace lagrange::viewer {

/// What a part's branch is doing: the current through it (A; NaN when unknown).
using PartCurrent = std::function<double(std::size_t bus, std::size_t part)>;
/// A bus's voltage (V; NaN when unknown).
using BusVoltage = std::function<double(std::size_t bus)>;

struct CircuitPick {
    std::size_t bus{0};
    std::optional<std::size_t> part;  ///< Nothing: the bus itself.
};

/// Draws the circuit at the cursor and leaves the cursor below it. Returns what was clicked.
std::optional<CircuitPick> drawCircuit(const Circuit& circuit, const PartCurrent& current,
                                       const BusVoltage& voltage,
                                       const std::optional<CircuitPick>& selected);

/// The Netlist tab, starting from the running stage's circuit (null while there is none).
struct PanelContext;
class Panel;
[[nodiscard]] std::unique_ptr<Panel> createNetlistPanel(
    std::function<const Circuit*(PanelContext&)> running);

}  // namespace lagrange::viewer
