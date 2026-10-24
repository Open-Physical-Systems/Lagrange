#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/World.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::electrical {

struct ExportOptions {
    std::string title{"lagrange_power"};  // the netlist's title, the Modelica model's name
    double duration{1.0};                 // s: .tran stop time, the experiment's stop time
    double step{1e-4};                    // s: .tran step, the experiment's interval
    /// Each motor's electrical power drawn from its bus over time (W, at times in s), as a
    /// piecewise-linear source; motors not listed draw a constant parameter, 0 W, to edit.
    std::map<std::string, std::vector<std::pair<double, double>>, std::less<>> drivePower;
    /// SPICE only: add a .control block that runs the analysis and writes every bus voltage to
    /// this file with ngspice's wrdata (time, voltage per column pair); empty: none.
    std::string writeData;
};

/// The robot's power system as an ngspice netlist (docs/design/domains/electrical.md §7), from
/// the same electrical schemas the network simulates: each bus a node with its capacitance
/// (starting at its batteries' open-circuit voltage), each battery its pack ECM (open-circuit
/// voltage at the initial state of charge, r0 and the RC pairs scaled for seriesCells ×
/// parallelCells), each load a behavioural current source (constant power off below
/// minVoltage, or a resistor), and each drive an averaged sink of its electrical power,
/// I = P/V. Not exported: state-of-charge integration over the run (the open-circuit voltage is
/// held), the charge limit, brake resistors and trips, converters.
[[nodiscard]] Expected<std::string, std::string> spiceNetlist(const WorldView& world,
                                                              const ExportOptions& options = {});

/// The same network as a Modelica model on the Modelica Standard Library 4
/// (Modelica.Electrical.Analog), with an experiment annotation.
[[nodiscard]] Expected<std::string, std::string> modelicaModel(const WorldView& world,
                                                               const ExportOptions& options = {});

}  // namespace lagrange::electrical
