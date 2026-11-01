#pragma once

// A robot's power system as a circuit: what the Electrical workspace draws and what its editor
// writes as a netlist (docs/design/ui/04-electrical/SPEC.md, formats.md §9.1). Built from the
// attributes of the stage's electrical nets; nothing here knows a particular robot.

#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::viewer {

struct CircuitPart {
    enum class Kind : std::uint8_t { Battery, Resistor, PowerLoad, Motor, Other };
    Kind kind{Kind::Other};
    std::string name;
    std::uint32_t prim{kNone};  ///< kNone in a part the editor added.
    std::string currentSignal;

    // Battery, for the whole pack.
    double voltage{0.0};                           ///< V
    double r0{0.0};                                ///< ohm, with its harness
    std::vector<std::pair<double, double>> pairs;  ///< R (ohm), C (F)
    double capacity{0.0};                          ///< C
    double cutoff{0.0};                            ///< V
    double maxCharge{0.0};                         ///< A
    double soc{1.0};
    bool sloped{false};  ///< Its voltage curve is not flat: `voltage` holds at the present charge.

    double resistance{0.0};  ///< Resistor: ohm
    double power{0.0};       ///< PowerLoad: W
    double minVoltage{0.0};  ///< PowerLoad: V

    // Motor.
    std::string joint;
    std::string motorType;
    double windingResistance{0.0};  ///< ohm
    double inductance{0.0};         ///< H
    double torqueConstant{0.0};     ///< N m/A
    double maxCurrent{0.0};         ///< A
    double gear{1.0};
    double efficiency{1.0};
    double rotorInertia{0.0};  ///< kg m²
    int polePairs{1};
};

struct CircuitBus {
    std::string name;
    std::uint32_t prim{kNone};
    std::string voltageSignal;
    double capacitance{0.0};   ///< F
    double undervoltage{0.0};  ///< V
    double overvoltage{0.0};   ///< V
    std::vector<CircuitPart> parts;
};

struct Circuit {
    std::vector<CircuitBus> buses;
};

/// The attributes of a prim, or nullptr while they have not arrived.
using AttributesOf = std::function<const PrimAttributes*(std::uint32_t)>;

/// The prims whose attributes circuitOf reads.
[[nodiscard]] std::vector<std::uint32_t> circuitPrims(const SceneStatics& statics);

/// The name of the joint a prim stands for in the scene (a joint has no prim of its own: its
/// relationships point at its moving link), or empty. From the signals named joint/<name>/...
using JointOf = std::function<std::string(std::uint32_t)>;

/// The circuit of a scene's nets. A prim without attributes yet is a part with no values.
[[nodiscard]] Circuit circuitOf(const SceneStatics& statics, const AttributesOf& attributes,
                                const JointOf& jointOf = {});

/// "4.7 mF", "100 mΩ": a value with an SI prefix.
[[nodiscard]] std::string engineering(double value, const char* unit);

/// One line of a part's values, for its node in the schematic: "51.2 V · r0 100 mΩ". Empty when
/// it has none yet.
[[nodiscard]] std::string partValues(const CircuitPart& part);

/// One line of a bus's values: "UV 42 V · OV 60 V · C 4.7 mF".
[[nodiscard]] std::string busValues(const CircuitBus& bus);

/// The bus's voltage with nothing drawn: the batteries in parallel through their resistance.
[[nodiscard]] double startVoltage(const CircuitBus& bus);

/// What keeps the circuit from being simulated, one line each; empty when it can be.
[[nodiscard]] std::vector<std::string> circuitProblems(const Circuit& circuit);

/// The circuit as a netlist that ngspice runs and `--overlay` reads (formats.md §9.1).
[[nodiscard]] std::string netlistOf(const Circuit& circuit, const std::string& title);

}  // namespace lagrange::viewer
