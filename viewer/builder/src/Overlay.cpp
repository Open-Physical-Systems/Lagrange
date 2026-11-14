#include "lagrange/viewer/builder/Edits.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace lagrange::viewer::builder {

namespace {

using Type = OverlayField::Type;
using Range = OverlayField::Range;

// The four sensors URDF has no element for (modules/sensors/catalog: `overlay_only`).
constexpr std::array<std::pair<std::string_view, std::string_view>, 4> kOverlaySensors{{
    {"gnss", "GnssAPI_1"},
    {"magnetometer", "MagnetometerAPI_1"},
    {"barometer", "BarometerAPI_1"},
    {"radar", "RadarAPI_1"},
}};
constexpr std::string_view kSensorSchema = "SensorAPI_1";
constexpr std::string_view kRate = "sensor:rate";
constexpr std::string_view kLatency = "sensor:latency";
constexpr std::string_view kJitter = "sensor:jitter";

// The schemas' `asset` attributes (schemas/*/schema.usda): paths relative to the overlay file.
constexpr std::array<std::string_view, 8> kAssetAttributes{
    "aero:coeffTable", "aero:rotor:propeller", "elec:battery:behaviorModel", "elec:behaviorModel",
    "emi:fieldMap",    "weather:fogField",     "weather:windField",          "wheel:tyre",
};

// --- the forms: schemas/electrical/schema.usda, in its order ----------------------------------
// Types, units (lagrangeUnit), fallbacks and "Required." are the schema's; Range is the stage
// loader's rule (modules/electrical/network/src/Network.cpp: "r0 > 0", "torqueConstant > 0",
// "maxCurrent > 0", "polePairs, gearRatio and gearEfficiency must be positive"; fractions 0 to 1
// by checkSchemaReal). schemas/test keeps no copy of this table: test_overlay checks it against
// the .usda.
constexpr std::array<std::string_view, 4> kChemistry{"li_ion_nmc", "lfp", "lead_acid", "custom"};
constexpr std::array<std::string_view, 3> kMotorType{"pmsm", "bldc", "dc_brushed"};
constexpr std::array<std::string_view, 3> kControlMode{"torque", "velocity", "voltage"};
constexpr std::array<std::string_view, 2> kRotorIntegration{"mechanical", "electrical"};
constexpr std::array<std::string_view, 4> kPortKind{"dc", "ac1", "ac3", "signal"};
constexpr std::span<const std::string_view> kNoTokens{};

const std::array<OverlayField, 16> kBatteryFields{{
    {Type::Token, Range::Any, "elec:battery:chemistry", "", "li_ion_nmc", false, kChemistry,
     "Cell chemistry, for reports"},
    {Type::Int, Range::AtLeastOne, "elec:battery:seriesCells", "1", "", true, kNoTokens,
     "Cells in series"},
    {Type::Int, Range::AtLeastOne, "elec:battery:parallelCells", "1", "1", false, kNoTokens,
     "Cells in parallel"},
    {Type::Real, Range::Positive, "elec:battery:capacity", "C", "", true, kNoTokens,
     "Rated charge of the pack (1 Ah = 3600 C)"},
    {Type::Real, Range::Fraction, "elec:battery:initialSoc", "1", "1", false, kNoTokens,
     "State of charge at the start, 0 to 1"},
    {Type::RealArray, Range::Any, "elec:battery:ocvSoc", "1", "", true, kNoTokens,
     "State-of-charge breakpoints, ascending"},
    {Type::RealArray, Range::Any, "elec:battery:ocvVoltage", "V", "", true, kNoTokens,
     "Cell open-circuit voltage at each breakpoint of ocvSoc"},
    {Type::Real, Range::Positive, "elec:battery:r0", "ohm", "", true, kNoTokens,
     "Cell series resistance at referenceTemperature"},
    {Type::Real, Range::NonNegative, "elec:battery:r1", "ohm", "0", false, kNoTokens,
     "First RC pair's resistance, per cell"},
    {Type::Real, Range::NonNegative, "elec:battery:c1", "F", "0", false, kNoTokens,
     "First RC pair's capacitance, per cell"},
    {Type::Real, Range::NonNegative, "elec:battery:r2", "ohm", "0", false, kNoTokens,
     "Second RC pair's resistance, per cell"},
    {Type::Real, Range::NonNegative, "elec:battery:c2", "F", "0", false, kNoTokens,
     "Second RC pair's capacitance, per cell"},
    {Type::Real, Range::Positive, "elec:battery:referenceTemperature", "K", "298.15", false,
     kNoTokens, "Temperature at which r0 is given"},
    {Type::Real, Range::Any, "elec:battery:r0TempCoeff", "1/K", "0", false, kNoTokens,
     "Fractional change of r0 per kelvin"},
    {Type::Real, Range::Any, "elec:battery:cutoffVoltage", "V", "", true, kNoTokens,
     "Pack under-voltage cut-off"},
    {Type::Real, Range::NonNegative, "elec:battery:maxChargeCurrent", "A", "", true, kNoTokens,
     "Regeneration limit; current beyond it raises the bus voltage"},
}};

const std::array<OverlayField, 4> kBusFields{{
    {Type::Real, Range::Positive, "elec:bus:capacitance", "F", "", false, kNoTokens,
     "Bus capacitance; unset: 1 uF with a warning"},
    {Type::Real, Range::Any, "elec:bus:undervoltageTrip", "V", "", true, kNoTokens,
     "Below this the bus browns out and its loads switch off"},
    {Type::Real, Range::Any, "elec:bus:overvoltageTrip", "V", "", true, kNoTokens,
     "Above this the drives trip"},
    {Type::Real, Range::NonNegative, "elec:bus:brakeResistor", "ohm", "0", false, kNoTokens,
     "Brake resistor switched in above overvoltageTrip; 0: none"},
}};

const std::array<OverlayField, 20> kMotorFields{{
    {Type::Token, Range::Any, "elec:motorType", "", "pmsm", false, kMotorType, "Machine type"},
    {Type::Real, Range::NonNegative, "elec:windingResistance", "ohm", "", true, kNoTokens,
     "Phase (or armature) resistance at referenceTemperature"},
    {Type::Real, Range::NonNegative, "elec:inductance", "H", "", true, kNoTokens,
     "d-axis (or armature) inductance"},
    {Type::Real, Range::NonNegative, "elec:inductanceQ", "H", "", false, kNoTokens,
     "q-axis inductance of a salient machine; unset: equal to inductance"},
    {Type::Real, Range::Positive, "elec:torqueConstant", "N.m/A", "", true, kNoTokens,
     "Torque per peak q-axis ampere, at the motor shaft"},
    {Type::Real, Range::Positive, "elec:backEmfConstant", "V.s/rad", "", false, kNoTokens,
     "Line-neutral peak volts per mechanical rad/s; unset: from torqueConstant"},
    {Type::Int, Range::AtLeastOne, "elec:polePairs", "1", "1", false, kNoTokens, "Pole pairs"},
    {Type::Real, Range::NonNegative, "elec:rotorInertia", "kg.m2", "0", false, kNoTokens,
     "Rotor inertia at the motor shaft"},
    {Type::Real, Range::Positive, "elec:gearRatio", "1", "1", false, kNoTokens,
     "Motor revolutions per joint revolution (on a slide: rad per m)"},
    {Type::Real, Range::Fraction, "elec:gearEfficiency", "1", "1", false, kNoTokens,
     "Gear efficiency, 0 to 1"},
    {Type::Real, Range::Positive, "elec:maxCurrent", "A", "", true, kNoTokens,
     "Drive current limit (peak)"},
    {Type::Real, Range::Positive, "elec:referenceTemperature", "K", "298.15", false, kNoTokens,
     "Temperature at which windingResistance is given"},
    {Type::Real, Range::Any, "elec:resistanceTempCoeff", "1/K", "0.00393", false, kNoTokens,
     "Fractional change of resistance per kelvin"},
    {Type::Token, Range::Any, "elec:controlMode", "", "torque", false, kControlMode,
     "Which command the drive follows"},
    {Type::Real, Range::Positive, "elec:velocityLoopBandwidth", "rad/s", "62.83", false, kNoTokens,
     "Speed loop of a velocity-mode drive"},
    {Type::Real, Range::Positive, "elec:currentLoopBandwidth", "rad/s", "6283", false, kNoTokens,
     "Current-loop bandwidth of the drive"},
    {Type::Real, Range::Positive, "elec:pwmFrequency", "Hz", "20000", false, kNoTokens,
     "Switching frequency, for the ripple estimate and the EMI spectrum"},
    {Type::Token, Range::Any, "elec:rotorIntegration", "", "mechanical", false, kRotorIntegration,
     "Which domain integrates the rotor speed"},
    {Type::Rel, Range::Any, "elec:joint", "", "", true, kNoTokens, "The driven joint"},
    {Type::Rel, Range::Any, "elec:powerPort", "", "", true, kNoTokens,
     "The bus or port the drive draws from"},
}};

const std::array<OverlayField, 6> kPortFields{{
    {Type::Token, Range::Any, "kind", "", "dc", false, kPortKind, "What the port carries"},
    {Type::Real, Range::Positive, "nominalVoltage", "V", "", false, kNoTokens,
     "Rated voltage, for sanity checks"},
    {Type::Real, Range::Positive, "maxCurrent", "A", "", false, kNoTokens,
     "Continuous current rating"},
    {Type::Rel, Range::Any, "connectedTo", "", "", false, kNoTokens,
     "Ports or bus prims on the same net"},
    {Type::Real, Range::NonNegative, "wireResistance", "ohm", "0", false, kNoTokens,
     "Lumped harness resistance to the net"},
    {Type::Real, Range::NonNegative, "wireInductance", "H", "0", false, kNoTokens,
     "Lumped harness inductance to the net"},
}};

// The parts between buses (RFC 0010; formats.md §9.1). Required as the schema says it
// ("Required." or "Required,"; a load's power is "required unless" a resistance is given, and a
// fuse's rating "for a fuse": checkOverlay has those rules). Ranges: the loader's
// (Network.cpp: "outputVoltage > 0, maxOutputCurrent > 0 and an efficiency in (0, 1]", "a branch
// needs elec:branch:resistance above zero", "a fuse needs elec:branch:i2tRating above zero");
// a load's power, resistance and minimum voltage and a diode's drop not negative are the
// builder's (the loader takes them as given).
constexpr std::array<std::string_view, 3> kLoadModel{"constant_power", "resistive", "signal"};
constexpr std::array<std::string_view, 4> kBranchKind{"resistor", "fuse", "switch", "diode"};
constexpr std::array<std::string_view, 2> kSwitchState{"closed", "open"};

const std::array<OverlayField, 5> kLoadFields{{
    {Type::Token, Range::Any, "elec:load:model", "", "constant_power", false, kLoadModel,
     "constant_power draws power; resistive scales with the voltage squared; signal draws "
     "load/<name>/power_demand"},
    {Type::Real, Range::NonNegative, "elec:load:power", "W", "", false, kNoTokens,
     "Power drawn (resistive: at the voltage the bus starts at)"},
    {Type::Real, Range::NonNegative, "elec:load:resistance", "ohm", "0", false, kNoTokens,
     "A resistive load's resistance, in place of power; 0: from the power"},
    {Type::Real, Range::NonNegative, "elec:load:minVoltage", "V", "0", false, kNoTokens,
     "Below this the load switches off (powered = 0)"},
    {Type::Rel, Range::Any, "elec:powerPort", "", "", true, kNoTokens,
     "The bus the load draws from"},
}};

const std::array<OverlayField, 5> kConverterFields{{
    {Type::Real, Range::Positive, "elec:conv:outputVoltage", "V", "", true, kNoTokens,
     "Regulated output voltage"},
    {Type::Real, Range::PositiveFraction, "elec:conv:efficiency", "1", "1", false, kNoTokens,
     "Power efficiency, above 0 and at most 1"},
    {Type::Real, Range::Positive, "elec:conv:maxOutputCurrent", "A", "", true, kNoTokens,
     "Output current limit"},
    {Type::Rel, Range::Any, "elec:conv:input", "", "", true, kNoTokens,
     "The bus the converter draws from"},
    {Type::Rel, Range::Any, "elec:conv:output", "", "", true, kNoTokens,
     "The bus the converter feeds (one converter per output bus)"},
}};

const std::array<OverlayField, 7> kBranchFields{{
    {Type::Token, Range::Any, "elec:branch:kind", "", "resistor", false, kBranchKind,
     "What it does between its two buses"},
    {Type::Real, Range::Positive, "elec:branch:resistance", "ohm", "", true, kNoTokens,
     "Resistance while it conducts"},
    {Type::Real, Range::NonNegative, "elec:branch:i2tRating", "A2.s", "0", false, kNoTokens,
     "A fuse opens when the integral of its current squared reaches this"},
    {Type::Real, Range::NonNegative, "elec:branch:forwardDrop", "V", "0", false, kNoTokens,
     "A diode's drop while it conducts, from input to output"},
    {Type::Token, Range::Any, "elec:branch:initialState", "", "closed", false, kSwitchState,
     "A switch's state until branch/<name>/command says otherwise"},
    {Type::Rel, Range::Any, "elec:branch:input", "", "", true, kNoTokens,
     "The bus at one end (a diode's anode); positive current flows from here"},
    {Type::Rel, Range::Any, "elec:branch:output", "", "", true, kNoTokens,
     "The bus at the other end (a diode's cathode)"},
}};

// --- presets: the sample robots' values, as their files have them -----------------------------
using Values = std::pair<std::string_view, std::string_view>;
constexpr std::string_view kAmrFile = "assets/robots/amr_diffdrive/amr_diffdrive.overlay.yaml";
constexpr std::string_view kQuadFile = "assets/robots/quad_x500/quad_x500.overlay.yaml";

constexpr std::array<Values, 12> kLfpBattery{{
    {"elec:battery:chemistry", "lfp"},
    {"elec:battery:seriesCells", "16"},
    {"elec:battery:capacity", "72000.0"},
    {"elec:battery:ocvSoc", "0.0, 0.1, 0.5, 0.9, 1.0"},
    {"elec:battery:ocvVoltage", "2.80, 3.20, 3.28, 3.33, 3.45"},
    {"elec:battery:r0", "0.0015"},
    {"elec:battery:r1", "0.0008"},
    {"elec:battery:c1", "12000.0"},
    {"elec:battery:cutoffVoltage", "44.8"},
    {"elec:battery:maxChargeCurrent", "10.0"},
    {"elec:port:dc_out:nominalVoltage", "51.2"},
    {"elec:port:dc_out:maxCurrent", "40.0"},
}};
constexpr std::array<Values, 12> kLipoBattery{{
    {"elec:battery:chemistry", "custom"},
    {"elec:battery:seriesCells", "4"},
    {"elec:battery:capacity", "18000.0"},
    {"elec:battery:ocvSoc", "0.0, 0.05, 0.1, 0.2, 0.5, 0.8, 0.9, 1.0"},
    {"elec:battery:ocvVoltage", "3.30, 3.50, 3.60, 3.70, 3.80, 3.95, 4.05, 4.20"},
    {"elec:battery:r0", "0.004"},
    {"elec:battery:r1", "0.002"},
    {"elec:battery:c1", "2000.0"},
    {"elec:battery:cutoffVoltage", "13.2"},
    {"elec:battery:maxChargeCurrent", "5.0"},
    {"elec:port:dc_out:nominalVoltage", "14.8"},
    {"elec:port:dc_out:maxCurrent", "120.0"},
}};
constexpr std::array<Values, 3> kAmrBus{{
    {"elec:bus:capacitance", "0.0047"},
    {"elec:bus:undervoltageTrip", "42.0"},
    {"elec:bus:overvoltageTrip", "60.0"},
}};
constexpr std::array<Values, 3> kQuadBus{{
    {"elec:bus:capacitance", "0.001"},
    {"elec:bus:undervoltageTrip", "12.8"},
    {"elec:bus:overvoltageTrip", "17.6"},
}};
constexpr std::array<Values, 8> kWheelMotor{{
    {"elec:windingResistance", "0.12"},
    {"elec:inductance", "0.00031"},
    {"elec:torqueConstant", "0.085"},
    {"elec:polePairs", "4"},
    {"elec:rotorInertia", "0.00012"},
    {"elec:gearRatio", "20.0"},
    {"elec:gearEfficiency", "0.92"},
    {"elec:maxCurrent", "8.0"},
}};
constexpr std::array<Values, 6> kRotorMotor{{
    {"elec:motorType", "bldc"},
    {"elec:windingResistance", "0.1"},
    {"elec:inductance", "0.00003"},
    {"elec:torqueConstant", "0.008989"},
    {"elec:polePairs", "7"},
    {"elec:maxCurrent", "25.0"},
}};

constexpr std::array<Values, 3> kComputerLoad{{
    {"elec:load:model", "constant_power"},
    {"elec:load:power", "45.0"},
    {"elec:load:minVoltage", "36.0"},
}};

const std::array<PowerPreset, 2> kBatteryPresets{{
    {"LFP 16S 20 Ah · sample AMR", kAmrFile, kLfpBattery},
    {"LiPo 4S 5 Ah · sample quadcopter", kQuadFile, kLipoBattery},
}};
const std::array<PowerPreset, 2> kBusPresets{{
    {"48 V bus · sample AMR", kAmrFile, kAmrBus},
    {"4S bus · sample quadcopter", kQuadFile, kQuadBus},
}};
const std::array<PowerPreset, 2> kMotorPresets{{
    {"wheel drive, 20:1 · sample AMR", kAmrFile, kWheelMotor},
    {"rotor motor, 920 KV · sample quadcopter", kQuadFile, kRotorMotor},
}};
const std::array<PowerPreset, 1> kLoadPresets{{
    {"computer, 45 W · sample AMR", kAmrFile, kComputerLoad},
}};

// --- text helpers -----------------------------------------------------------------------------

std::string trim(std::string_view text) {
    const std::size_t first = text.find_first_not_of(" \t");
    if (first == std::string_view::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(" \t");
    return std::string(text.substr(first, last - first + 1));
}

std::vector<std::string> splitList(std::string_view text) {
    std::string inner = trim(text);
    if (inner.size() >= 2 && inner.front() == '[' && inner.back() == ']') {
        inner = inner.substr(1, inner.size() - 2);
    }
    std::vector<std::string> items;
    if (trim(inner).empty()) {
        return items;
    }
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = inner.find(',', start);
        items.push_back(trim(std::string_view(inner).substr(
            start, comma == std::string::npos ? std::string::npos : comma - start)));
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    return items;
}

bool isAsset(std::string_view attribute) {
    return std::ranges::find(kAssetAttributes, attribute) != kAssetAttributes.end();
}

bool isUri(std::string_view text) {
    return text.find("://") != std::string_view::npos;
}

/// A scalar as YAML writes it plainly when it can, double-quoted otherwise.
std::string yamlScalar(const std::string& text) {
    const bool plain =
        !text.empty() &&
        std::ranges::all_of(text,
                            [](char c) {
                                return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                                       (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '/' ||
                                       c == '-' || c == '+' || c == ':';
                            }) &&
        text.front() != '-' && text.front() != ':' && text.back() != ':';
    // A leading minus of a number is plain YAML ("-0.5"); "-x" would be a list item.
    const bool number = parseNumber(text).has_value();
    if (plain || number) {
        return text;
    }
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    return out + "\"";
}

std::string valueText(const OverlayAttribute& attribute) {
    if (!attribute.list) {
        return attribute.values.empty() ? std::string{} : yamlScalar(attribute.values.front());
    }
    std::string out = "[";
    for (std::size_t i = 0; i < attribute.values.size(); ++i) {
        out += (i == 0 ? "" : ", ") + yamlScalar(attribute.values[i]);
    }
    return out + "]";
}

/// The schema part of an attribute name for the forms: "PortAPI_1:dc_out" and
/// "elec:port:dc_out:maxCurrent" → the port field "maxCurrent".
const OverlayField* findIn(std::span<const OverlayField> fields, std::string_view name) {
    const auto it =
        std::ranges::find_if(fields, [&](const OverlayField& f) { return f.name == name; });
    return it == fields.end() ? nullptr : &*it;
}

std::string lastPart(std::string_view path) {
    const std::size_t slash = path.rfind('/');
    return std::string(slash == std::string_view::npos ? path : path.substr(slash + 1));
}

// --- YAML in ---------------------------------------------------------------------------------

std::string scalarOf(const YAML::Node& node) {
    return node.IsScalar() ? node.Scalar() : std::string{};
}

std::optional<OverlayAttribute> readAttribute(const std::string& name, const YAML::Node& node,
                                              const std::filesystem::path& base,
                                              std::string& error) {
    OverlayAttribute attribute;
    attribute.name = name;
    if (node.IsSequence()) {
        attribute.list = true;
        for (const YAML::Node& item : node) {
            if (!item.IsScalar()) {
                error = name + ": a list holds plain values only";
                return std::nullopt;
            }
            attribute.values.push_back(item.Scalar());
        }
    } else if (node.IsScalar() || node.IsNull()) {
        attribute.values.push_back(node.IsNull() ? std::string{} : node.Scalar());
    } else {
        error = name + ": a value or a list of values";
        return std::nullopt;
    }
    if (isAsset(name) && !base.empty()) {
        for (std::string& value : attribute.values) {
            if (!value.empty() && !isUri(value) && std::filesystem::path(value).is_relative()) {
                value = (base / value).lexically_normal().string();
            }
        }
    }
    return attribute;
}

/// An entry that is an overlay-only sensor on a link: its Sensor; nullopt otherwise.
std::optional<Sensor> sensorOf(const OverlayEntry& entry, const Document& document) {
    if (entry.prim.empty() || entry.createChild.empty() || !entry.create.empty() ||
        findLink(document, entry.prim) == nullptr || !validName(entry.createChild) ||
        findSensor(document, entry.createChild) != nullptr ||
        findLink(document, entry.createChild) != nullptr ||
        findJoint(document, entry.createChild) != nullptr) {
        return std::nullopt;
    }
    Sensor sensor;
    sensor.name = entry.createChild;
    sensor.link = entry.prim;
    bool typed = false;
    for (const OverlaySchema& schema : entry.schemas) {
        if (schema.name == kSensorSchema) {
            for (const OverlayAttribute& a : schema.attributes) {
                const auto value =
                    a.list || a.values.size() != 1 ? std::nullopt : parseNumber(a.values.front());
                if (!value) {
                    return std::nullopt;
                }
                if (a.name == kRate) {
                    sensor.updateRate = *value;
                } else if (a.name == kLatency) {
                    sensor.latency = *value;
                } else if (a.name == kJitter) {
                    sensor.jitter = *value;
                } else {
                    return std::nullopt;
                }
            }
            continue;
        }
        const auto known = std::ranges::find_if(
            kOverlaySensors, [&](const auto& pair) { return pair.second == schema.name; });
        if (known == kOverlaySensors.end() || typed) {
            return std::nullopt;
        }
        typed = true;
        sensor.type = std::string(known->first);
        for (const OverlayAttribute& a : schema.attributes) {
            if (a.list) {
                return std::nullopt;
            }
            sensor.params[a.name] = a.values.empty() ? std::string{} : a.values.front();
        }
    }
    if (!typed) {
        return std::nullopt;
    }
    return sensor;
}

// --- YAML out --------------------------------------------------------------------------------

struct Writer {
    std::ostringstream out;
    const OverlayWriteOptions& options;

    void attribute(int indent, const std::string& schema, const OverlayAttribute& a) {
        OverlayAttribute shown = a;
        if (isAsset(a.name)) {
            for (std::string& value : shown.values) {
                const std::filesystem::path path(value);
                if (value.empty() || isUri(value) || !path.is_absolute() ||
                    options.absoluteAssets || options.savedAt.empty()) {
                    continue;
                }
                const std::filesystem::path relative =
                    path.lexically_relative(options.savedAt.parent_path().lexically_normal());
                if (!relative.empty()) {
                    value = relative.string();
                }
            }
        }
        std::string line =
            std::string(static_cast<std::size_t>(indent), ' ') + a.name + ": " + valueText(shown);
        if (const OverlayField* field = overlayField(schema, a.name);
            field != nullptr && !field->unit.empty() && field->unit != "1") {
            line += "  # " + displayUnit(field->unit);
        }
        out << line << "\n";
    }

    void schemas(const std::vector<OverlaySchema>& list) {
        if (list.empty()) {
            out << "    schemas: {}\n";
            return;
        }
        out << "    schemas:\n";
        for (const OverlaySchema& schema : list) {
            if (schema.attributes.empty()) {
                out << "      " << schema.name << ": {}\n";
                continue;
            }
            out << "      " << schema.name << ":\n";
            for (const OverlayAttribute& a : schema.attributes) {
                attribute(8, schema.name, a);
            }
        }
    }

    void entry(const OverlayEntry& e) {
        if (!e.create.empty()) {
            out << "  - create: " << yamlScalar(e.create) << "\n";
        } else {
            out << "  - prim: " << yamlScalar(e.prim) << "\n";
            if (!e.createChild.empty()) {
                out << "    create_child: " << yamlScalar(e.createChild) << "\n";
            }
        }
        schemas(e.schemas);
    }
};

OverlayEntry sensorEntry(const Sensor& sensor) {
    OverlayEntry entry;
    entry.prim = sensor.link;
    entry.createChild = sensor.name;
    OverlaySchema timing{std::string(kSensorSchema), {}};
    timing.attributes.push_back({std::string(kRate), {formatNumber(sensor.updateRate, 15)}, false});
    if (sensor.latency) {
        timing.attributes.push_back({std::string(kLatency), {formatNumber(*sensor.latency, 15)}});
    }
    if (sensor.jitter) {
        timing.attributes.push_back({std::string(kJitter), {formatNumber(*sensor.jitter, 15)}});
    }
    entry.schemas.push_back(std::move(timing));
    OverlaySchema typed{std::string(overlaySensorSchema(sensor.type)), {}};
    for (const auto& [path, value] : sensor.params) {
        typed.attributes.push_back({path, {value}, false});
    }
    entry.schemas.push_back(std::move(typed));
    return entry;
}

// --- tree comparison -------------------------------------------------------------------------

bool sameNode(const YAML::Node& a, const YAML::Node& b, const std::string& where,
              std::string* why) {
    const auto fail = [&](const std::string& what) {
        if (why != nullptr) {
            *why = where + ": " + what;
        }
        return false;
    };
    if (a.Type() != b.Type() && !(a.IsNull() && b.IsScalar() && b.Scalar().empty()) &&
        !(b.IsNull() && a.IsScalar() && a.Scalar().empty())) {
        return fail("a different kind of value");
    }
    if (a.IsScalar()) {
        if (a.Scalar() == b.Scalar()) {
            return true;
        }
        const auto x = parseNumber(a.Scalar());
        const auto y = parseNumber(b.Scalar());
        if (x && y && *x == *y) {
            return true;
        }
        return fail(a.Scalar() + " ≠ " + b.Scalar());
    }
    if (a.IsSequence()) {
        if (a.size() != b.size()) {
            return fail(std::to_string(a.size()) + " items ≠ " + std::to_string(b.size()));
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (!sameNode(a[i], b[i], where + "[" + std::to_string(i) + "]", why)) {
                return false;
            }
        }
        return true;
    }
    if (a.IsMap()) {
        std::set<std::string> keys;
        for (const auto& item : a) {
            keys.insert(item.first.Scalar());
        }
        std::set<std::string> other;
        for (const auto& item : b) {
            other.insert(item.first.Scalar());
        }
        if (keys != other) {
            std::string missing;
            for (const std::string& k : keys) {
                missing += other.contains(k) ? "" : " -" + k;
            }
            for (const std::string& k : other) {
                missing += keys.contains(k) ? "" : " +" + k;
            }
            return fail("keys differ:" + missing);
        }
        for (const std::string& key : keys) {
            if (!sameNode(a[key], b[key], where + "." + key, why)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

// --- names ------------------------------------------------------------------------------------

std::filesystem::path overlayPathFor(const std::filesystem::path& urdf) {
    std::filesystem::path path = urdf;
    return path.replace_extension(".overlay.yaml");
}

std::string_view overlaySensorSchema(std::string_view type) {
    const auto it =
        std::ranges::find_if(kOverlaySensors, [&](const auto& pair) { return pair.first == type; });
    return it == kOverlaySensors.end() ? std::string_view{} : it->second;
}

bool isOverlaySensor(std::string_view type) {
    return !overlaySensorSchema(type).empty();
}

// --- read and write ---------------------------------------------------------------------------

OverlayReadResult readOverlay(std::string_view yaml, Document& document,
                              const std::filesystem::path& base) {
    OverlayReadResult result;
    YAML::Node root;
    try {
        root = YAML::Load(std::string(yaml));
    } catch (const YAML::Exception& error) {
        result.error = "line " + std::to_string(error.mark.line + 1) + ": " + error.msg;
        return result;
    }
    if (!root.IsMap()) {
        result.error = "an overlay is a map with schema_version and apply";
        return result;
    }
    if (!root["schema_version"] || root["schema_version"].as<std::string>("") != "1") {
        result.error = "schema_version must be 1 (formats.md §9)";
        return result;
    }
    for (const auto& item : root) {
        const std::string key = item.first.Scalar();
        if (key != "schema_version" && key != "apply") {
            result.error = "unknown key " + key + " (schema_version and apply only)";
            return result;
        }
    }
    Overlay overlay;
    std::vector<Sensor> sensors;
    // The leading comment lines, as written.
    {
        std::istringstream in{std::string(yaml)};
        bool more = false;
        for (std::string line; std::getline(in, line);) {
            const std::string t = trim(line);
            if (t.starts_with('#')) {
                overlay.header += line + "\n";
            } else if (!t.empty()) {
                break;
            }
        }
        std::istringstream again{std::string(yaml)};
        bool body = false;
        for (std::string line; std::getline(again, line);) {
            const std::string t = trim(line);
            body = body || (!t.empty() && !t.starts_with('#'));
            more = more || (body && line.find(" #") != std::string::npos);
        }
        if (more) {
            result.notes.push_back(
                "comments after the file's header are not kept: the builder writes each "
                "value's unit as its comment");
        }
    }
    const YAML::Node apply = root["apply"];
    if (apply && !apply.IsNull() && !apply.IsSequence()) {
        result.error = "apply must be a list of entries";
        return result;
    }
    Document scratch = document;  // sensors found here must not clash with each other
    std::size_t index = 0;
    for (const YAML::Node& node : apply ? apply : YAML::Node(YAML::NodeType::Sequence)) {
        const std::string where = "apply[" + std::to_string(index++) + "]";
        if (!node.IsMap()) {
            result.error = where + ": an entry is a map";
            return result;
        }
        OverlayEntry entry;
        for (const auto& item : node) {
            const std::string key = item.first.Scalar();
            if (key == "prim") {
                entry.prim = scalarOf(item.second);
            } else if (key == "create_child") {
                entry.createChild = scalarOf(item.second);
            } else if (key == "create") {
                entry.create = scalarOf(item.second);
            } else if (key == "schemas") {
                if (!item.second.IsMap() && !item.second.IsNull()) {
                    result.error = where + ": schemas is a map of schema names";
                    return result;
                }
                for (const auto& schema : item.second) {
                    OverlaySchema instance{schema.first.Scalar(), {}};
                    if (!schema.second.IsMap() && !schema.second.IsNull()) {
                        result.error = where + ": " + instance.name + " is a map of attributes";
                        return result;
                    }
                    for (const auto& attribute : schema.second) {
                        std::string error;
                        auto read =
                            readAttribute(attribute.first.Scalar(), attribute.second, base, error);
                        if (!read) {
                            result.error = where + ": " + error;
                            return result;
                        }
                        instance.attributes.push_back(std::move(*read));
                    }
                    entry.schemas.push_back(std::move(instance));
                }
            } else {
                result.error = where + ": unknown key " + key;
                return result;
            }
        }
        if (entry.prim.empty() == entry.create.empty()) {
            result.error = where + ": an entry has prim or create (one of them)";
            return result;
        }
        if (auto sensor = sensorOf(entry, scratch)) {
            OverlayEntry placeholder;
            placeholder.sensor = sensor->name;
            overlay.entries.push_back(std::move(placeholder));
            scratch.sensors.push_back(*sensor);
            sensors.push_back(std::move(*sensor));
            continue;
        }
        overlay.entries.push_back(std::move(entry));
    }
    document.overlay = std::move(overlay);
    for (Sensor& sensor : sensors) {
        document.sensors.push_back(std::move(sensor));
    }
    result.ok = true;
    return result;
}

bool hasOverlay(const Document& document) {
    return std::ranges::any_of(document.overlay.entries,
                               [](const OverlayEntry& e) { return e.sensor.empty(); }) ||
           std::ranges::any_of(document.sensors,
                               [](const Sensor& s) { return isOverlaySensor(s.type); });
}

std::string writeOverlay(const Document& document, const OverlayWriteOptions& options) {
    if (!hasOverlay(document)) {
        return {};
    }
    Writer w{{}, options};
    if (!document.overlay.header.empty()) {
        w.out << document.overlay.header;
    } else {
        w.out << "# The overlay of " << document.robot
              << " (docs/design/formats.md §9), written by the robot builder:\n"
                 "# `lagrange_server --robot "
              << document.robot << ".urdf --overlay " << document.robot << ".overlay.yaml`.\n";
    }
    w.out << "schema_version: 1\napply:";
    std::set<std::string, std::less<>> written;
    bool any = false;
    const auto emitSensor = [&](const Sensor& sensor) {
        w.out << (any ? "" : "\n");
        any = true;
        w.entry(sensorEntry(sensor));
        written.insert(sensor.name);
    };
    for (const OverlayEntry& entry : document.overlay.entries) {
        if (!entry.sensor.empty()) {
            const Sensor* sensor = findSensor(document, entry.sensor);
            if (sensor != nullptr && isOverlaySensor(sensor->type) &&
                !written.contains(sensor->name)) {
                emitSensor(*sensor);
            }
            continue;
        }
        w.out << (any ? "" : "\n");
        any = true;
        w.entry(entry);
    }
    for (const Sensor& sensor : document.sensors) {
        if (isOverlaySensor(sensor.type) && !written.contains(sensor.name)) {
            emitSensor(sensor);
        }
    }
    if (!any) {
        w.out << " []\n";
    }
    return w.out.str();
}

bool sameOverlay(std::string_view a, std::string_view b, std::string* why) {
    try {
        return sameNode(YAML::Load(std::string(a)), YAML::Load(std::string(b)), "overlay", why);
    } catch (const YAML::Exception& error) {
        if (why != nullptr) {
            *why = error.what();
        }
        return false;
    }
}

// --- forms ------------------------------------------------------------------------------------

std::span<const OverlayField> batteryFields() {
    return kBatteryFields;
}
std::span<const OverlayField> busFields() {
    return kBusFields;
}
std::span<const OverlayField> motorFields() {
    return kMotorFields;
}
std::span<const OverlayField> portFields() {
    return kPortFields;
}
std::span<const OverlayField> loadFields() {
    return kLoadFields;
}
std::span<const OverlayField> converterFields() {
    return kConverterFields;
}
std::span<const OverlayField> branchFields() {
    return kBranchFields;
}

std::span<const OverlayField> fieldsOf(std::string_view schema) {
    if (schema == kBatterySchema) {
        return kBatteryFields;
    }
    if (schema == kBusSchema) {
        return kBusFields;
    }
    if (schema == kMotorSchema) {
        return kMotorFields;
    }
    if (schema == kLoadSchema) {
        return kLoadFields;
    }
    if (schema == kConverterSchema) {
        return kConverterFields;
    }
    if (schema == kBranchSchema) {
        return kBranchFields;
    }
    if (schema.starts_with("PortAPI_1:")) {
        return kPortFields;
    }
    return {};
}

bool branchFieldApplies(std::string_view kind, std::string_view field) {
    if (field == "elec:branch:i2tRating") {
        return kind == "fuse";
    }
    if (field == "elec:branch:forwardDrop") {
        return kind == "diode";
    }
    if (field == "elec:branch:initialState") {
        return kind == "switch";
    }
    return true;
}

const OverlayField* overlayField(std::string_view schema, std::string_view attribute) {
    if (schema.starts_with("PortAPI_1:")) {
        const std::string prefix = "elec:port:" + std::string(schema.substr(10)) + ":";
        return attribute.starts_with(prefix) ? findIn(kPortFields, attribute.substr(prefix.size()))
                                             : nullptr;
    }
    return findIn(fieldsOf(schema), attribute);
}

std::string displayUnit(std::string_view unit) {
    if (unit == "ohm") {
        return "Ω";
    }
    std::string out;
    for (std::size_t i = 0; i < unit.size(); ++i) {
        if (unit[i] == '.') {
            out += "·";
        } else if (unit[i] == '2' && i > 0 &&
                   std::isalpha(static_cast<unsigned char>(unit[i - 1])) != 0) {
            out += "²";
        } else {
            out += unit[i];
        }
    }
    return out;
}

std::string checkField(const OverlayField& field, std::string_view text) {
    const std::string value = trim(text);
    if (value.empty()) {
        return field.required ? "required: the loader refuses the robot without it" : "";
    }
    const auto inRange = [&](double v) -> std::string {
        switch (field.range) {
            case Range::Any:
                return {};
            case Range::Positive:
                return v > 0.0 ? "" : "must be above 0";
            case Range::NonNegative:
                return v >= 0.0 ? "" : "must not be negative";
            case Range::Fraction:
                return v >= 0.0 && v <= 1.0 ? "" : "must lie within 0 to 1";
            case Range::PositiveFraction:
                return v > 0.0 && v <= 1.0 ? "" : "must be above 0 and at most 1";
            case Range::AtLeastOne:
                return v >= 1.0 ? "" : "must be at least 1";
        }
        return {};
    };
    switch (field.type) {
        case Type::Real: {
            const auto v = parseNumber(value);
            return v ? inRange(*v) : "not a number";
        }
        case Type::Int: {
            const auto v = parseNumber(value);
            if (!v || std::floor(*v) != *v) {
                return "a whole number";
            }
            return inRange(*v);
        }
        case Type::Token:
            return std::ranges::find(field.tokens, value) != field.tokens.end()
                       ? ""
                       : "one of the schema's words";
        case Type::RealArray: {
            const std::vector<std::string> items = splitList(value);
            if (items.empty()) {
                return field.required ? "required: at least one value" : "";
            }
            for (const std::string& item : items) {
                if (!parseNumber(item)) {
                    return "numbers separated by commas";
                }
            }
            return {};
        }
        case Type::Rel:
        case Type::Asset:
            return {};
    }
    return {};
}

// --- the power system ---------------------------------------------------------------------

const OverlaySchema* findSchema(const OverlayEntry& entry, std::string_view schema) {
    const auto it = std::ranges::find_if(entry.schemas,
                                         [&](const OverlaySchema& s) { return s.name == schema; });
    return it == entry.schemas.end() ? nullptr : &*it;
}
OverlaySchema* findSchema(OverlayEntry& entry, std::string_view schema) {
    const auto it = std::ranges::find_if(entry.schemas,
                                         [&](const OverlaySchema& s) { return s.name == schema; });
    return it == entry.schemas.end() ? nullptr : &*it;
}

std::string attributeText(const OverlaySchema& schema, std::string_view name) {
    const auto it = std::ranges::find_if(schema.attributes,
                                         [&](const OverlayAttribute& a) { return a.name == name; });
    if (it == schema.attributes.end()) {
        return {};
    }
    std::string out;
    for (std::size_t i = 0; i < it->values.size(); ++i) {
        out += (i == 0 ? "" : ", ") + it->values[i];
    }
    return out;
}

void setAttribute(OverlaySchema& schema, const OverlayField* field, std::string_view name,
                  std::string_view text) {
    const std::string value = trim(text);
    auto it = std::ranges::find_if(schema.attributes,
                                   [&](const OverlayAttribute& a) { return a.name == name; });
    if (value.empty()) {
        if (it != schema.attributes.end()) {
            schema.attributes.erase(it);
        }
        return;
    }
    OverlayAttribute attribute{std::string(name), {}, false};
    if (field != nullptr && field->type == Type::RealArray) {
        attribute.list = true;
        attribute.values = splitList(value);
    } else {
        attribute.values = {value};
    }
    if (it != schema.attributes.end()) {
        *it = std::move(attribute);
        return;
    }
    // A new attribute goes after the last one the form's order puts before it.
    const std::span<const OverlayField> fields = fieldsOf(schema.name);
    const auto rank = [&](std::string_view attributeName) -> std::ptrdiff_t {
        std::string_view bare = attributeName;
        if (schema.name.starts_with("PortAPI_1:")) {
            bare = bare.substr(bare.rfind(':') + 1);
        }
        const auto at =
            std::ranges::find_if(fields, [&](const OverlayField& f) { return f.name == bare; });
        return at == fields.end() ? std::ptrdiff_t{1000} : at - fields.begin();
    };
    const std::ptrdiff_t mine = rank(name);
    auto at = schema.attributes.begin();
    for (auto i = schema.attributes.begin(); i != schema.attributes.end(); ++i) {
        if (rank(i->name) <= mine) {
            at = i + 1;
        }
    }
    schema.attributes.insert(at, std::move(attribute));
}

const OverlaySchema* portOf(const OverlayEntry& entry) {
    const auto it = std::ranges::find_if(
        entry.schemas, [](const OverlaySchema& s) { return s.name.starts_with("PortAPI_1:"); });
    return it == entry.schemas.end() ? nullptr : &*it;
}
OverlaySchema* portOf(OverlayEntry& entry) {
    const auto it = std::ranges::find_if(
        entry.schemas, [](const OverlaySchema& s) { return s.name.starts_with("PortAPI_1:"); });
    return it == entry.schemas.end() ? nullptr : &*it;
}
std::string portPrefix(const OverlaySchema& port) {
    return "elec:port:" + port.name.substr(port.name.find(':') + 1) + ":";
}

std::string entryPrim(const OverlayEntry& entry) {
    if (!entry.create.empty()) {
        return entry.create;
    }
    if (!entry.createChild.empty()) {
        return entry.prim + "/" + entry.createChild;
    }
    return entry.prim;
}

std::string entryName(const OverlayEntry& entry) {
    return entry.sensor.empty() ? lastPart(entryPrim(entry)) : entry.sensor;
}

std::vector<std::size_t> entriesWith(const Document& document, std::string_view schema) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < document.overlay.entries.size(); ++i) {
        if (findSchema(document.overlay.entries[i], schema) != nullptr) {
            out.push_back(i);
        }
    }
    return out;
}

int motorOf(const Document& document, std::string_view joint) {
    for (const std::size_t i : entriesWith(document, kMotorSchema)) {
        const OverlaySchema& motor = *findSchema(document.overlay.entries[i], kMotorSchema);
        if (lastPart(attributeText(motor, "elec:joint")) == joint) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int busNamed(const Document& document, std::string_view target) {
    const std::string name = lastPart(trim(target));
    if (name.empty()) {
        return -1;
    }
    for (const std::size_t i : entriesWith(document, kBusSchema)) {
        const OverlayEntry& entry = document.overlay.entries[i];
        if (entryPrim(entry) == trim(target) || entryName(entry) == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::span<const PowerPreset> batteryPresets() {
    return kBatteryPresets;
}
std::span<const PowerPreset> busPresets() {
    return kBusPresets;
}
std::span<const PowerPreset> motorPresets() {
    return kMotorPresets;
}
std::span<const PowerPreset> loadPresets() {
    return kLoadPresets;
}

namespace {

/// A name for `power/<base>` not used by an entry, a link, a joint or a sensor.
std::string freePowerName(const Document& document, std::string_view base) {
    const auto used = [&](const std::string& name) {
        return std::ranges::any_of(document.overlay.entries,
                                   [&](const OverlayEntry& e) { return entryName(e) == name; }) ||
               findLink(document, name) != nullptr || findJoint(document, name) != nullptr ||
               findSensor(document, name) != nullptr;
    };
    std::string name(base);
    for (int k = 2; used(name); ++k) {
        name = std::string(base) + "_" + std::to_string(k);
    }
    return name;
}

void applyPreset(OverlayEntry& entry, const std::string& schema, const PowerPreset* preset) {
    if (preset == nullptr) {
        return;
    }
    for (const auto& [attribute, value] : preset->values) {
        const bool port = attribute.starts_with(kBatteryPortPrefix);
        OverlaySchema* target = findSchema(entry, port ? kBatteryPort : std::string_view(schema));
        if (target == nullptr) {
            entry.schemas.push_back({std::string(port ? kBatteryPort : schema), {}});
            target = &entry.schemas.back();
        }
        setAttribute(*target, overlayField(target->name, attribute), attribute, value);
    }
}

std::string busPath(const Document& document, int bus) {
    if (bus < 0) {
        const auto buses = entriesWith(document, kBusSchema);
        if (buses.empty()) {
            return {};
        }
        bus = static_cast<int>(buses.front());
    }
    return static_cast<std::size_t>(bus) < document.overlay.entries.size()
               ? entryPrim(document.overlay.entries[static_cast<std::size_t>(bus)])
               : std::string{};
}

}  // namespace

std::string addBus(Document& document, std::string_view name, const PowerPreset* preset) {
    OverlayEntry entry;
    const std::string chosen = freePowerName(document, name.empty() ? "dc_bus" : name);
    entry.create = std::string(kPowerScope) + chosen;
    entry.schemas.push_back({std::string(kBusSchema), {}});
    applyPreset(entry, std::string(kBusSchema), preset);
    document.overlay.entries.push_back(std::move(entry));
    return chosen;
}

std::string addBattery(Document& document, std::string_view name, int bus,
                       const PowerPreset* preset) {
    OverlayEntry entry;
    const std::string chosen = freePowerName(document, name.empty() ? "battery" : name);
    entry.create = std::string(kPowerScope) + chosen;
    entry.schemas.push_back({std::string(kBatterySchema), {}});
    entry.schemas.push_back({std::string(kBatteryPort), {}});
    applyPreset(entry, std::string(kBatterySchema), preset);
    if (const std::string to = busPath(document, bus); !to.empty()) {
        const std::string attribute = std::string(kBatteryPortPrefix) + "connectedTo";
        OverlaySchema& port = *findSchema(entry, kBatteryPort);
        setAttribute(port, overlayField(port.name, attribute), attribute, to);
    }
    document.overlay.entries.push_back(std::move(entry));
    return chosen;
}

std::string addMotor(Document& document, std::string_view joint, int bus,
                     const PowerPreset* preset) {
    if (findJoint(document, joint) == nullptr || motorOf(document, joint) >= 0) {
        return {};
    }
    OverlayEntry entry;
    entry.prim = std::string(joint);
    entry.createChild = freePowerName(document, std::string(joint) + "_motor");
    entry.schemas.push_back({std::string(kMotorSchema), {}});
    applyPreset(entry, std::string(kMotorSchema), preset);
    OverlaySchema& motor = *findSchema(entry, kMotorSchema);
    setAttribute(motor, overlayField(kMotorSchema, "elec:joint"), "elec:joint", joint);
    if (const std::string to = busPath(document, bus); !to.empty()) {
        setAttribute(motor, overlayField(kMotorSchema, "elec:powerPort"), "elec:powerPort", to);
    }
    const std::string name = entry.createChild;
    document.overlay.entries.push_back(std::move(entry));
    return name;
}

std::string addLoad(Document& document, std::string_view name, int bus, const PowerPreset* preset) {
    OverlayEntry entry;
    const std::string chosen = freePowerName(document, name.empty() ? "load" : name);
    entry.create = std::string(kPowerScope) + chosen;
    entry.schemas.push_back({std::string(kLoadSchema), {}});
    applyPreset(entry, std::string(kLoadSchema), preset);
    if (const std::string to = busPath(document, bus); !to.empty()) {
        setAttribute(entry.schemas.front(), overlayField(kLoadSchema, "elec:powerPort"),
                     "elec:powerPort", to);
    }
    document.overlay.entries.push_back(std::move(entry));
    return chosen;
}

namespace {

/// A bus entry's prim, or "" for -1 (a part between buses picks both ends itself).
std::string chosenBus(const Document& document, int bus) {
    return bus < 0 ? std::string{} : busPath(document, bus);
}

}  // namespace

std::string addConverter(Document& document, std::string_view name, int input, int output) {
    OverlayEntry entry;
    const std::string chosen = freePowerName(document, name.empty() ? "dcdc" : name);
    entry.create = std::string(kPowerScope) + chosen;
    entry.schemas.push_back({std::string(kConverterSchema), {}});
    OverlaySchema& converter = entry.schemas.front();
    for (const auto& [attribute, bus] :
         {std::pair{"elec:conv:input", input}, std::pair{"elec:conv:output", output}}) {
        if (const std::string to = chosenBus(document, bus); !to.empty()) {
            setAttribute(converter, overlayField(kConverterSchema, attribute), attribute, to);
        }
    }
    document.overlay.entries.push_back(std::move(entry));
    return chosen;
}

std::string addBranch(Document& document, std::string_view name, std::string_view kind, int input,
                      int output) {
    const bool known = std::ranges::find(kBranchKind, kind) != kBranchKind.end();
    const std::string_view type = known ? kind : std::string_view("resistor");
    OverlayEntry entry;
    const std::string chosen = freePowerName(document, name.empty() ? type : name);
    entry.create = std::string(kPowerScope) + chosen;
    entry.schemas.push_back({std::string(kBranchSchema), {}});
    OverlaySchema& branch = entry.schemas.front();
    if (type != "resistor") {  // the schema's default stays unwritten
        setAttribute(branch, overlayField(kBranchSchema, "elec:branch:kind"), "elec:branch:kind",
                     type);
    }
    for (const auto& [attribute, bus] :
         {std::pair{"elec:branch:input", input}, std::pair{"elec:branch:output", output}}) {
        if (const std::string to = chosenBus(document, bus); !to.empty()) {
            setAttribute(branch, overlayField(kBranchSchema, attribute), attribute, to);
        }
    }
    document.overlay.entries.push_back(std::move(entry));
    return chosen;
}

std::vector<std::pair<std::string, std::string>> powerSignals(const OverlayEntry& entry) {
    const std::string name = entryName(entry);
    std::vector<std::pair<std::string, std::string>> out;
    const auto add =
        [&](std::string_view scope,
            std::initializer_list<std::pair<std::string_view, std::string_view>> list) {
            for (const auto& [signal, unit] : list) {
                out.emplace_back(std::string(scope) + "/" + name + "/" + std::string(signal),
                                 std::string(unit));
            }
        };
    if (findSchema(entry, kBusSchema) != nullptr) {
        add("bus", {{"voltage", "V"}, {"brownout", "1"}});
    } else if (findSchema(entry, kBatterySchema) != nullptr) {
        add("battery", {{"soc", "1"},
                        {"terminal_voltage", "V"},
                        {"current", "A"},
                        {"power", "W"},
                        {"loss_power", "W"}});
    } else if (findSchema(entry, kLoadSchema) != nullptr) {
        add("load", {{"current", "A"}, {"powered", "1"}});
    } else if (findSchema(entry, kConverterSchema) != nullptr) {
        add("converter", {{"output_current", "A"}, {"input_current", "A"}, {"loss_power", "W"}});
    } else if (findSchema(entry, kBranchSchema) != nullptr) {
        add("branch", {{"current", "A"}, {"voltage", "V"}, {"state", "1"}, {"loss_power", "W"}});
    }
    return out;
}

bool removeEntry(Document& document, std::size_t index) {
    auto& entries = document.overlay.entries;
    if (index >= entries.size()) {
        return false;
    }
    const std::string prim = entryPrim(entries[index]);
    const std::string name = entryName(entries[index]);
    entries.erase(entries.begin() + static_cast<std::ptrdiff_t>(index));
    for (OverlayEntry& entry : entries) {
        for (OverlaySchema& schema : entry.schemas) {
            for (OverlayAttribute& attribute : schema.attributes) {
                const OverlayField* field = overlayField(schema.name, attribute.name);
                if (field == nullptr || field->type != Type::Rel) {
                    continue;
                }
                std::erase_if(attribute.values,
                              [&](const std::string& v) { return v == prim || v == name; });
            }
            std::erase_if(schema.attributes,
                          [](const OverlayAttribute& a) { return a.values.empty(); });
        }
    }
    return true;
}

void renameInOverlay(Document& document, std::string_view from, std::string_view to) {
    for (OverlayEntry& entry : document.overlay.entries) {
        if (entry.prim == from) {
            entry.prim = to;
        }
        if (entry.sensor == from) {
            entry.sensor = to;
        }
        for (OverlaySchema& schema : entry.schemas) {
            for (OverlayAttribute& attribute : schema.attributes) {
                for (std::string& value : attribute.values) {
                    if (value == from) {
                        value = to;
                    }
                }
            }
        }
    }
}

void removeFromOverlay(Document& document, std::span<const std::string> names) {
    const auto gone = [&](std::string_view name) {
        return std::ranges::find(names, name) != names.end();
    };
    auto& entries = document.overlay.entries;
    for (std::size_t i = entries.size(); i-- > 0;) {
        const OverlayEntry& entry = entries[i];
        const bool named = (!entry.prim.empty() && gone(entry.prim)) ||
                           (!entry.sensor.empty() && gone(entry.sensor));
        const OverlaySchema* motor = findSchema(entry, kMotorSchema);
        const bool drives = motor != nullptr && gone(lastPart(attributeText(*motor, "elec:joint")));
        if (named || drives) {
            (void)removeEntry(document, i);
        }
    }
}

}  // namespace lagrange::viewer::builder

namespace lagrange::viewer::builder {

namespace {

void checkValues(std::vector<Problem>& out, const std::string& item, const OverlaySchema& schema,
                 std::span<const OverlayField> fields, std::string_view prefix) {
    for (const OverlayField& field : fields) {
        if (field.type == Type::Rel) {
            continue;  // the relationships have rules of their own
        }
        const std::string name = std::string(prefix) + std::string(field.name);
        const std::string value = attributeText(schema, name);
        if (const std::string why = checkField(field, value); !why.empty()) {
            out.push_back(
                {Level::Error, item, name + (value.empty() ? "" : " = " + value) + ": " + why});
        }
    }
}

std::vector<double> numbers(const OverlaySchema& schema, std::string_view name) {
    std::vector<double> out;
    for (const std::string& item : splitList(attributeText(schema, name))) {
        if (const auto v = parseNumber(item)) {
            out.push_back(*v);
        }
    }
    return out;
}

/// A relationship's bus: the entry index, or -1 (and a problem for `item`).
int busOfRelationship(const Document& document, std::vector<Problem>& out, const std::string& item,
                      const OverlaySchema& schema, std::string_view attribute,
                      std::string_view what) {
    const std::string to = attributeText(schema, attribute);
    const int bus = busNamed(document, to);
    if (bus < 0) {
        out.push_back({Level::Error, item,
                       to.empty()
                           ? std::string(what) + ": no bus picked (" + std::string(attribute) + ")"
                           : std::string(what) + " " + to + " is not a bus of the overlay (" +
                                 std::string(attribute) + ")"});
    }
    return bus;
}

}  // namespace

std::vector<Problem> checkOverlay(const Document& document) {
    std::vector<Problem> out;
    const auto& entries = document.overlay.entries;
    // What feeds a bus: a battery, then (as the network starts them) a converter from a fed bus,
    // a branch to a fed bus: a resistor, fuse or switch either way, a diode from its anode.
    std::set<int> fed;
    // Converters: their values, both ends, one per output bus, and loops among them.
    std::vector<std::pair<int, int>> conversions;  // (input, output) bus entries
    std::map<int, std::string> feeder;             // output bus → the converter feeding it
    for (const std::size_t i : entriesWith(document, kConverterSchema)) {
        const OverlayEntry& entry = entries[i];
        const std::string item = entryPrim(entry);
        const OverlaySchema& converter = *findSchema(entry, kConverterSchema);
        checkValues(out, item, converter, converterFields(), "");
        const int in = busOfRelationship(document, out, item, converter, "elec:conv:input",
                                         "the converter's input");
        const int to = busOfRelationship(document, out, item, converter, "elec:conv:output",
                                         "the converter's output");
        if (in >= 0 && in == to) {
            out.push_back({Level::Error, item,
                           "the converter's input and output are the same bus: pick another "
                           "for its output"});
            continue;
        }
        if (to >= 0) {
            if (const auto [at, added] = feeder.emplace(to, entryName(entry)); !added) {
                out.push_back({Level::Error, item,
                               "two converters feed " +
                                   entryName(entries[static_cast<std::size_t>(to)]) + " (with " +
                                   at->second + "): one converter per output bus is simulated"});
            }
        }
        if (in >= 0 && to >= 0) {
            conversions.emplace_back(in, to);
        }
    }
    // A loop of converters (a → b → a): the network does not refuse it, but power would go round
    // it, each converter's losses feeding the next, and the start-up depends on the file's order.
    for (const std::size_t i : entriesWith(document, kConverterSchema)) {
        const OverlaySchema& converter = *findSchema(entries[i], kConverterSchema);
        const int in = busNamed(document, attributeText(converter, "elec:conv:input"));
        const int to = busNamed(document, attributeText(converter, "elec:conv:output"));
        if (in < 0 || to < 0 || in == to) {
            continue;
        }
        std::set<int> reached{to};
        std::vector<int> frontier{to};
        while (!frontier.empty()) {
            const int at = frontier.back();
            frontier.pop_back();
            for (const auto& [a, b] : conversions) {
                if (a == at && reached.insert(b).second) {
                    frontier.push_back(b);
                }
            }
        }
        if (reached.contains(in)) {
            out.push_back({Level::Warning, entryPrim(entries[i]),
                           "converters in a loop: its output reaches its own input through other "
                           "converters, so power goes round; the network runs it, but it is "
                           "no real circuit"});
        }
    }
    // Branches: values for their kind, two different buses.
    std::vector<std::tuple<int, int, bool>> joins;  // (input, output, diode)
    for (const std::size_t i : entriesWith(document, kBranchSchema)) {
        const OverlayEntry& entry = entries[i];
        const std::string item = entryPrim(entry);
        const OverlaySchema& branch = *findSchema(entry, kBranchSchema);
        std::string kind = attributeText(branch, "elec:branch:kind");
        kind = kind.empty() ? "resistor" : kind;
        checkValues(out, item, branch, branchFields(), "");
        if (kind == "fuse") {
            const auto rating = parseNumber(attributeText(branch, "elec:branch:i2tRating"));
            if (!rating || *rating <= 0.0) {
                out.push_back({Level::Error, item,
                               "a fuse needs elec:branch:i2tRating above 0 (A²·s): the current "
                               "squared over time at which it opens"});
            }
        }
        for (const OverlayField& field : branchFields()) {
            const std::string name(field.name);
            if (!branchFieldApplies(kind, name) && !attributeText(branch, name).empty()) {
                out.push_back(
                    {Level::Hint, item,
                     name + " is for another kind of branch: a " + kind + " does not use it"});
            }
        }
        const int in =
            busOfRelationship(document, out, item, branch, "elec:branch:input", "the input");
        const int to =
            busOfRelationship(document, out, item, branch, "elec:branch:output", "the output");
        if (in >= 0 && in == to) {
            out.push_back({Level::Error, item, "a branch's two ends are the same bus"});
        } else if (in >= 0 && to >= 0) {
            joins.emplace_back(in, to, kind == "diode");
        }
    }
    // Loads: their values, a power unless a resistance gives it, and a bus.
    for (const std::size_t i : entriesWith(document, kLoadSchema)) {
        const OverlayEntry& entry = entries[i];
        const std::string item = entryPrim(entry);
        const OverlaySchema& load = *findSchema(entry, kLoadSchema);
        checkValues(out, item, load, loadFields(), "");
        const std::string model = attributeText(load, "elec:load:model");
        const auto resistance = parseNumber(attributeText(load, "elec:load:resistance"));
        const bool byResistance = model == "resistive" && resistance && *resistance > 0.0;
        if (attributeText(load, "elec:load:power").empty() && !byResistance) {
            out.push_back({Level::Error, item,
                           model == "resistive"
                               ? "a load needs elec:load:power, or a resistance above 0"
                               : "a load needs elec:load:power (W)"});
        }
        (void)busOfRelationship(document, out, item, load, "elec:powerPort", "the load draws from");
    }
    for (const std::size_t i : entriesWith(document, kBatterySchema)) {
        const OverlayEntry& entry = entries[i];
        const std::string item = entryPrim(entry);
        checkValues(out, item, *findSchema(entry, kBatterySchema), batteryFields(), "");
        const OverlaySchema& battery = *findSchema(entry, kBatterySchema);
        const std::vector<double> soc = numbers(battery, "elec:battery:ocvSoc");
        const std::vector<double> ocv = numbers(battery, "elec:battery:ocvVoltage");
        if (!soc.empty() &&
            (!std::ranges::is_sorted(soc) || std::ranges::adjacent_find(soc) != soc.end())) {
            out.push_back({Level::Error, item, "elec:battery:ocvSoc must ascend, no repeats"});
        }
        if (!soc.empty() && !ocv.empty() && soc.size() != ocv.size()) {
            out.push_back({Level::Error, item,
                           "ocvSoc has " + std::to_string(soc.size()) + " points, ocvVoltage " +
                               std::to_string(ocv.size()) + ": one voltage per point"});
        }
        const OverlaySchema* port = portOf(entry);
        const std::string to = port == nullptr
                                   ? std::string{}
                                   : attributeText(*port, portPrefix(*port) + "connectedTo");
        if (port != nullptr) {
            checkValues(out, item, *port, portFields(), portPrefix(*port));
        }
        const int bus = busNamed(document, to);
        if (bus < 0) {
            out.push_back({Level::Error, item,
                           to.empty() ? "the battery feeds no bus: pick one for its output"
                                      : "the battery's output goes to " + to +
                                            ", which is not a bus of the overlay"});
        } else {
            fed.insert(bus);
        }
    }
    for (bool more = true; more;) {
        more = false;
        for (const auto& [in, to] : conversions) {
            if (fed.contains(in)) {
                more = fed.insert(to).second || more;
            }
        }
        for (const auto& [in, to, diode] : joins) {
            if (fed.contains(in) && !fed.contains(to)) {
                more = fed.insert(to).second || more;
            } else if (!diode && fed.contains(to) && !fed.contains(in)) {
                more = fed.insert(in).second || more;
            }
        }
    }
    for (const std::size_t i : entriesWith(document, kBusSchema)) {
        const OverlayEntry& entry = entries[i];
        const std::string item = entryPrim(entry);
        const OverlaySchema& bus = *findSchema(entry, kBusSchema);
        checkValues(out, item, bus, busFields(), "");
        const auto under = parseNumber(attributeText(bus, "elec:bus:undervoltageTrip"));
        const auto over = parseNumber(attributeText(bus, "elec:bus:overvoltageTrip"));
        if (under && over && *over <= *under) {
            out.push_back({Level::Error, item, "overvoltageTrip must be above undervoltageTrip"});
        }
        if (!fed.contains(static_cast<int>(i))) {
            out.push_back({Level::Error, item,
                           "a bus without a battery: nothing feeds it; add a battery, or a "
                           "converter or branch from a bus a battery feeds, and pick this bus "
                           "for its output"});
        }
    }
    std::set<std::string, std::less<>> driven;
    for (const std::size_t i : entriesWith(document, kMotorSchema)) {
        const OverlayEntry& entry = entries[i];
        const OverlaySchema& motor = *findSchema(entry, kMotorSchema);
        const std::string jointName = lastPart(attributeText(motor, "elec:joint"));
        const Joint* joint = findJoint(document, jointName);
        const std::string item = joint != nullptr ? jointName : entryPrim(entry);
        checkValues(out, item, motor, motorFields(), "");
        if (jointName.empty()) {
            out.push_back({Level::Error, item, "the motor drives no joint (elec:joint)"});
        } else if (joint == nullptr) {
            out.push_back(
                {Level::Error, item, "the motor drives " + jointName + ", which is not a joint"});
        } else if (joint->type == Joint::Type::Fixed) {
            out.push_back({Level::Error, item,
                           "a motor on a fixed joint: it cannot turn; make the joint revolute, "
                           "continuous or prismatic, or remove the motor"});
        } else if (!driven.insert(jointName).second) {
            out.push_back({Level::Error, item, "a second motor on " + jointName});
        }
        const std::string port = attributeText(motor, "elec:powerPort");
        if (busNamed(document, port) < 0) {
            out.push_back({Level::Error, item,
                           port.empty() ? "the motor draws from no bus: pick one"
                                        : "the motor draws from " + port +
                                              ", which is not a bus of the overlay"});
        }
    }
    for (const OverlayEntry& entry : entries) {
        const bool plainName = !entry.prim.empty() && entry.prim.find('/') == std::string::npos;
        if (plainName && findLink(document, entry.prim) == nullptr &&
            findJoint(document, entry.prim) == nullptr &&
            findSensor(document, entry.prim) == nullptr) {
            out.push_back({Level::Error, entryPrim(entry),
                           "the overlay names " + entry.prim +
                               ", which is no link, joint or sensor of the robot"});
        }
    }
    for (const Sensor& sensor : document.sensors) {
        if (isOverlaySensor(sensor.type) && (sensor.origin.xyz != std::array<double, 3>{} ||
                                             sensor.origin.rpy != std::array<double, 3>{})) {
            out.push_back(
                {Level::Warning, sensor.name,
                 "the overlay has no mount pose: " + sensor.name + " sits at its link's origin"});
        }
    }
    return out;
}

}  // namespace lagrange::viewer::builder
