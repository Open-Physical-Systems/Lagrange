// The circuit the Electrical workspace draws and writes.
#include "lagrange/viewer/Circuit.hpp"

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

AttributeInfo number(std::string name, std::vector<double> values) {
    AttributeInfo a;
    a.name = std::move(name);
    a.numbers = std::move(values);
    return a;
}

/// A bus with a 16-cell pack on a sloped curve, a resistive load and a motor.
struct Fixture {
    SceneStatics statics;
    std::map<std::uint32_t, PrimAttributes> attributes;

    Fixture() {
        const auto prim = [this](const char* name, const char* schema) {
            PrimData p;
            p.name = name;
            p.apiSchemas = {schema};
            statics.prims.push_back(p);
            return static_cast<std::uint32_t>(statics.prims.size() - 1);
        };
        const std::uint32_t bus = prim("dc", "ElectricalBusAPI_1");
        const std::uint32_t pack = prim("pack", "BatteryAPI_1");
        const std::uint32_t lamp = prim("lamp", "ElectricalLoadAPI_1");
        const std::uint32_t motor = prim("drive", "ElectricalMotorAPI_1");
        const std::uint32_t joint = prim("wheel", "PhysicsRevoluteJoint");
        ElectricalNet net;
        net.name = "dc";
        net.busPrim = bus;
        net.terminals = {{pack, "dc_out", ""}, {lamp, "dc_in", ""}, {motor, "dc_in", ""}};
        statics.nets.push_back(net);
        attributes[bus].attributes = {number("elec:bus:capacitance", {0.0047}),
                                      number("elec:bus:undervoltageTrip", {42.0}),
                                      number("elec:bus:overvoltageTrip", {60.0})};
        attributes[pack].attributes = {number("elec:battery:seriesCells", {16.0}),
                                       number("elec:battery:parallelCells", {2.0}),
                                       number("elec:battery:initialSoc", {0.5}),
                                       number("elec:battery:ocvSoc", {0.0, 1.0}),
                                       number("elec:battery:ocvVoltage", {3.0, 3.4}),
                                       number("elec:battery:r0", {0.01}),
                                       number("elec:battery:r1", {0.02}),
                                       number("elec:battery:c1", {1000.0}),
                                       number("elec:port:dc_out:wireResistance", {0.05}),
                                       number("elec:battery:capacity", {72000.0}),
                                       number("elec:battery:cutoffVoltage", {44.0}),
                                       number("elec:battery:maxChargeCurrent", {10.0})};
        AttributeInfo model;
        model.name = "elec:load:model";
        model.value = "resistive";
        attributes[lamp].attributes = {model, number("elec:load:power", {100.0})};
        AttributeInfo target;
        target.name = "elec:joint";
        target.targets = {joint};
        attributes[motor].attributes = {target,
                                        number("elec:windingResistance", {0.12}),
                                        number("elec:inductance", {0.00031}),
                                        number("elec:torqueConstant", {0.085}),
                                        number("elec:maxCurrent", {8.0}),
                                        number("elec:gearRatio", {20.0})};
    }

    [[nodiscard]] Circuit circuit() const {
        return circuitOf(statics, [this](std::uint32_t prim) -> const PrimAttributes* {
            const auto found = attributes.find(prim);
            return found != attributes.end() ? &found->second : nullptr;
        });
    }
};

TEST(Circuit, PacksAndLoadsBecomeWhatTheirNodesShow) {
    const Circuit circuit = Fixture().circuit();
    ASSERT_EQ(circuit.buses.size(), 1U);
    const CircuitBus& bus = circuit.buses[0];
    EXPECT_EQ(busValues(bus), "UV 42 V · OV 60 V · C 4.7 mF");
    ASSERT_EQ(bus.parts.size(), 3U);
    const CircuitPart& pack = bus.parts[0];
    EXPECT_EQ(pack.kind, CircuitPart::Kind::Battery);
    EXPECT_DOUBLE_EQ(pack.voltage, 16.0 * 3.2);
    EXPECT_TRUE(pack.sloped);
    EXPECT_DOUBLE_EQ(pack.r0, 0.01 * 8.0 + 0.05);  // scaled by 16s2p, and its harness
    ASSERT_EQ(pack.pairs.size(), 1U);
    EXPECT_DOUBLE_EQ(pack.pairs[0].first, 0.16);
    EXPECT_DOUBLE_EQ(pack.pairs[0].second, 125.0);
    // A resistive load is its power at the voltage the bus starts at.
    EXPECT_EQ(bus.parts[1].kind, CircuitPart::Kind::Resistor);
    EXPECT_DOUBLE_EQ(bus.parts[1].resistance, 51.2 * 51.2 / 100.0);
    EXPECT_EQ(bus.parts[2].joint, "wheel");
    EXPECT_TRUE(circuitProblems(circuit).empty());
}

TEST(Circuit, APartWithoutAttributesYetShowsNoValues) {
    Fixture f;
    f.attributes.clear();
    const Circuit circuit = f.circuit();
    EXPECT_TRUE(busValues(circuit.buses[0]).empty());
    EXPECT_TRUE(partValues(circuit.buses[0].parts[0]).empty());
    EXPECT_FALSE(circuitProblems(circuit).empty());
}

TEST(Circuit, EngineeringPicksThePrefix) {
    EXPECT_EQ(engineering(0.1, "Ω"), "100 mΩ");
    EXPECT_EQ(engineering(0.0047, "F"), "4.7 mF");
    EXPECT_EQ(engineering(470e-6, "F"), "470 µF");
    EXPECT_EQ(engineering(51.2, "V"), "51.2 V");
    EXPECT_EQ(engineering(72000.0, "C"), "72 kC");
    EXPECT_EQ(engineering(0.0, "V"), "0 V");
}

TEST(Circuit, TheNetlistHasEveryPartAndNoteOfTheCurve) {
    const std::string text = netlistOf(Fixture().circuit(), "my robot");
    EXPECT_EQ(text.rfind("my robot\n", 0), 0U);
    for (const char* expected :
         {"V_pack pack_in 0 DC 51.2", "R_pack_rc1", "R_pack ", "C_dc dc 0 0.0047", "R_lamp dc 0",
          "*lagrange battery V_pack capacity=72000", "*lagrange bus dc undervoltage=42",
          "*lagrange motor wheel node=dc name=drive", "held at the charge", ".end"}) {
        EXPECT_NE(text.find(expected), std::string::npos) << expected << " in\n" << text;
    }
}

TEST(Circuit, ProblemsAreWhatTheImporterWouldRefuse) {
    Circuit circuit;
    EXPECT_FALSE(circuitProblems(circuit).empty());
    CircuitBus bus;
    bus.name = "dc";
    circuit.buses.push_back(bus);
    const auto problems = circuitProblems(circuit);
    EXPECT_EQ(problems.size(), 2U);  // no trips, no battery
}

}  // namespace
}  // namespace lagrange::viewer
