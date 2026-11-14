// The builder's overlay file (RFC 0016 phase 3; formats.md §9): read and write against every
// sample overlay, the forms' table against the electrical schema, the power-system edits and
// their validation rules.
#include "lagrange/viewer/builder/Edits.hpp"
#include "lagrange/viewer/builder/History.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

const std::filesystem::path kAssets = LAGRANGE_BUILDER_ASSETS_DIR;
const std::filesystem::path kTestDir = LAGRANGE_BUILDER_TEST_DIR;

std::string slurp(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

Document urdf(const std::filesystem::path& path) {
    ReadResult read = readUrdf(slurp(path));
    EXPECT_TRUE(read.document) << path << ": " << read.error;
    return read.document ? *read.document : Document{};
}

bool hasProblem(const std::vector<Problem>& problems, Level level, std::string_view item,
                std::string_view words) {
    return std::ranges::any_of(problems, [&](const Problem& p) {
        return p.level == level && p.item == item && p.message.find(words) != std::string::npos;
    });
}

std::string dump(const std::vector<Problem>& problems) {
    std::string out;
    for (const Problem& p : problems) {
        out += std::string(toString(p.level)) + " " + p.item + ": " + p.message + "\n";
    }
    return out;
}

// --- round trip -----------------------------------------------------------------------------

struct Sample {
    const char* overlay;
    const char* urdf;
    int sensors;  ///< overlay-only sensors it has
};

class SampleOverlays : public ::testing::TestWithParam<Sample> {};

TEST_P(SampleOverlays, ReadAndWriteMeanTheSame) {
    const Sample sample = GetParam();
    const std::filesystem::path file = kAssets / sample.overlay;
    const std::string original = slurp(file);
    ASSERT_FALSE(original.empty()) << file;
    Document document = urdf(kAssets / sample.urdf);
    const std::size_t urdfSensors = document.sensors.size();
    const OverlayReadResult read = readOverlay(original, document, file.parent_path());
    ASSERT_TRUE(read.ok) << read.error;
    EXPECT_EQ(document.sensors.size() - urdfSensors, static_cast<std::size_t>(sample.sensors));
    // Saved where it came from: relative asset paths come back as they were.
    OverlayWriteOptions options;
    options.savedAt = file;
    const std::string written = writeOverlay(document, options);
    std::string why;
    EXPECT_TRUE(sameOverlay(original, written, &why)) << why << "\n" << written;
    // And again: the writer's own output reads back to the same document.
    Document again = urdf(kAssets / sample.urdf);
    ASSERT_TRUE(readOverlay(written, again, file.parent_path()).ok);
    EXPECT_EQ(writeOverlay(again, options), written);
    // The header comment is kept; the overlay-only sensors never reach the URDF.
    EXPECT_TRUE(written.starts_with(original.substr(0, original.find('\n'))));
    const std::string text = writeUrdf(document);
    for (const Sensor& sensor : document.sensors) {
        if (isOverlaySensor(sensor.type)) {
            EXPECT_EQ(text.find("name=\"" + sensor.name + "\""), std::string::npos) << sensor.name;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Builder, SampleOverlays,
    ::testing::Values(
        Sample{"amr_diffdrive/amr_diffdrive.overlay.yaml", "amr_diffdrive/amr_diffdrive.urdf", 0},
        Sample{"quad_x500/quad_x500.overlay.yaml", "quad_x500/quad_x500.urdf", 3},
        Sample{"quad_x500/quad_x500_folding.overlay.yaml", "quad_x500/quad_x500.urdf", 3},
        Sample{"quad_x500_inverted/quad_x500_inverted.overlay.yaml",
               "quad_x500_inverted/quad_x500_inverted.urdf", 3},
        Sample{"holybro_x500_v2/holybro_x500_v2.overlay.yaml",
               "holybro_x500_v2/holybro_x500_v2.urdf", 3},
        Sample{"arm6/arm6.overlay.yaml", "arm6/arm6.urdf", 0},
        Sample{"arm6/arm6_undersized.overlay.yaml", "arm6/arm6.urdf", 0},
        Sample{"lexus_rx450h/lexus_rx450h.overlay.yaml", "lexus_rx450h/lexus_rx450h.urdf", 0}));

TEST(Overlay, TheQuadcoptersSensorsAreDocumentSensors) {
    const std::filesystem::path file = kAssets / "quad_x500/quad_x500.overlay.yaml";
    Document document = urdf(kAssets / "quad_x500/quad_x500.urdf");
    ASSERT_TRUE(readOverlay(slurp(file), document, file.parent_path()).ok);
    const Sensor* gnss = findSensor(document, "gnss");
    ASSERT_NE(gnss, nullptr);
    EXPECT_EQ(gnss->type, "gnss");
    EXPECT_EQ(gnss->link, "base_link");
    EXPECT_EQ(gnss->updateRate, 10.0);
    EXPECT_EQ(gnss->params.at("sensor:gnss:horizontalNoiseStd"), "0.3");
    EXPECT_EQ(findSensor(document, "barometer")->params.at("sensor:barometer:noiseStd"), "1.0");
    // The IMU of the overlay is not overlay-only (URDF has <sensor type="imu">): kept as written.
    EXPECT_EQ(findSensor(document, "imu"), nullptr);
    // Its propellers: absolute while in the document, relative again when written beside it.
    const int rotor = [&] {
        for (std::size_t i = 0; i < document.overlay.entries.size(); ++i) {
            if (findSchema(document.overlay.entries[i], "RotorAPI_1") != nullptr) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }();
    ASSERT_GE(rotor, 0);
    const std::string propeller = attributeText(
        *findSchema(document.overlay.entries[static_cast<std::size_t>(rotor)], "RotorAPI_1"),
        "aero:rotor:propeller");
    EXPECT_TRUE(std::filesystem::path(propeller).is_absolute()) << propeller;
    EXPECT_TRUE(std::filesystem::exists(propeller)) << propeller;
    // The sample has no motor problems, battery, bus or sensors.
    const std::vector<Problem> problems = checkOverlay(document);
    EXPECT_TRUE(problems.empty()) << dump(problems);
}

TEST(Overlay, SamplesValidateWithoutOverlayProblems) {
    for (const auto& [overlay, robot] :
         {std::pair{"amr_diffdrive/amr_diffdrive.overlay.yaml", "amr_diffdrive/amr_diffdrive.urdf"},
          std::pair{"arm6/arm6.overlay.yaml", "arm6/arm6.urdf"}}) {
        Document document = urdf(kAssets / robot);
        ASSERT_TRUE(readOverlay(slurp(kAssets / overlay), document).ok);
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(problems.empty()) << overlay << "\n" << dump(problems);
    }
}

TEST(Overlay, RefusesWhatIsNotAnOverlay) {
    Document document = newDocument();
    const Document before = document;
    EXPECT_FALSE(readOverlay("schema_version: 2\napply: []\n", document).ok);
    EXPECT_FALSE(readOverlay("schema_version: 1\napply:\n  - schemas: {}\n", document).ok);
    EXPECT_FALSE(readOverlay("schema_version: 1\nextra: 1\n", document).ok);
    const OverlayReadResult bad = readOverlay("schema_version: 1\napply: [\n", document);
    EXPECT_FALSE(bad.ok);
    EXPECT_NE(bad.error.find("line"), std::string::npos) << bad.error;
    EXPECT_TRUE(document.overlay.entries.empty());
    // No overlay content: nothing to write.
    EXPECT_FALSE(hasOverlay(document));
    EXPECT_EQ(writeOverlay(document), "");
}

TEST(Overlay, PathBesideTheUrdf) {
    EXPECT_EQ(overlayPathFor("/a/rover.urdf"), std::filesystem::path("/a/rover.overlay.yaml"));
}

// --- the forms' table against the schema ---------------------------------------------------

/// The block of an attribute in schemas/electrical/schema.usda: its declaration line and its
/// metadata up to the closing parenthesis.
std::string declaration(const std::string& usda, const std::string& schema,
                        const std::string& attribute) {
    const std::size_t cls = usda.find("class \"" + schema + "\"");
    if (cls == std::string::npos) {
        return {};
    }
    const std::size_t end = usda.find("\nclass ", cls + 1);
    const std::string body =
        usda.substr(cls, end == std::string::npos ? std::string::npos : end - cls);
    const std::regex line("\\n    (uniform )?[a-z]+(\\[\\])? " +
                          std::regex_replace(attribute, std::regex(":"), ":") + "( = [^ (]+)? \\(");
    std::smatch match;
    if (!std::regex_search(body, match, line)) {
        return {};
    }
    const std::size_t from = static_cast<std::size_t>(match.position(0));
    const std::size_t close = body.find("\n    )", from);
    return body.substr(from, close - from);
}

TEST(Overlay, FormsMatchTheElectricalSchema) {
    const std::string usda = slurp(LAGRANGE_SCHEMAS_DIR "/electrical/schema.usda");
    ASSERT_FALSE(usda.empty());
    const auto check = [&](const std::string& schema, std::span<const OverlayField> fields) {
        for (const OverlayField& field : fields) {
            const std::string name(field.name);
            const std::string text = declaration(usda, schema, name);
            ASSERT_FALSE(text.empty()) << schema << " has no " << name;
            const std::string unit = field.unit.empty() || field.type == OverlayField::Type::Token
                                         ? std::string{}
                                         : "lagrangeUnit = \"" + std::string(field.unit) + "\"";
            if (!unit.empty()) {
                EXPECT_NE(text.find(unit), std::string::npos) << schema << " " << name << "\n"
                                                              << text;
            }
            const std::string fallback = field.type == OverlayField::Type::Token
                                             ? "\"" + std::string(field.fallback) + "\""
                                             : std::string(field.fallback);
            if (!field.fallback.empty()) {
                EXPECT_NE(text.find(" = " + fallback + " ("), std::string::npos)
                    << schema << " " << name << "\n"
                    << text;
            } else {
                const std::string first = text.substr(0, text.find('(', 1));
                EXPECT_EQ(first.find(" = "), std::string::npos) << schema << " " << name;
            }
            // "Required." or "Required, above zero."; "Required unless …" and "Required for a
            // fuse." are conditions checkOverlay applies, not plain requirements.
            const bool required = text.find("Required.") != std::string::npos ||
                                  text.find("Required,") != std::string::npos;
            EXPECT_EQ(field.required, required) << schema << " " << name;
            if (field.type == OverlayField::Type::Token) {
                for (const std::string_view token : field.tokens) {
                    EXPECT_NE(text.find("\"" + std::string(token) + "\""), std::string::npos)
                        << schema << " " << name << " " << token;
                }
            }
        }
    };
    check("BatteryAPI_1", batteryFields());
    check("ElectricalBusAPI_1", busFields());
    check("ElectricalMotorAPI_1", motorFields());
    check("PortAPI_1", portFields());
    check("ElectricalLoadAPI_1", loadFields());
    check("ElectricalConverterAPI_1", converterFields());
    check("ElectricalBranchAPI_1", branchFields());
    // Every attribute the schema declares for these three has a field (none left out).
    for (const auto& [schema, fields] : {std::pair{"ElectricalLoadAPI_1", loadFields()},
                                         std::pair{"ElectricalConverterAPI_1", converterFields()},
                                         std::pair{"ElectricalBranchAPI_1", branchFields()}}) {
        const std::size_t cls = usda.find(std::string("class \"") + schema + "\"");
        const std::size_t end = usda.find("\nclass ", cls + 1);
        const std::string body = usda.substr(cls, end == std::string::npos ? end : end - cls);
        const std::regex attribute("\\n    (uniform )?[a-z]+(\\[\\])? (elec:[A-Za-z:0-9]+)");
        std::size_t declared = 0;
        for (auto it = std::sregex_iterator(body.begin(), body.end(), attribute);
             it != std::sregex_iterator(); ++it) {
            ++declared;
            EXPECT_NE(overlayField(schema, (*it)[3].str()), nullptr) << schema << " " << (*it)[3];
        }
        EXPECT_EQ(declared, fields.size()) << schema;
    }
}

TEST(Overlay, FieldChecks) {
    const OverlayField* r0 = overlayField(kBatterySchema, "elec:battery:r0");
    ASSERT_NE(r0, nullptr);
    EXPECT_EQ(checkField(*r0, "0.0015"), "");
    EXPECT_NE(checkField(*r0, "-1"), "");
    EXPECT_NE(checkField(*r0, "0"), "");
    EXPECT_NE(checkField(*r0, ""), "");  // required
    EXPECT_NE(checkField(*r0, "abc"), "");
    const OverlayField* cells = overlayField(kBatterySchema, "elec:battery:seriesCells");
    EXPECT_NE(checkField(*cells, "2.5"), "");
    EXPECT_EQ(checkField(*cells, "16"), "");
    const OverlayField* efficiency = overlayField(kMotorSchema, "elec:gearEfficiency");
    EXPECT_NE(checkField(*efficiency, "1.2"), "");
    EXPECT_EQ(checkField(*efficiency, ""), "");
    const OverlayField* type = overlayField(kMotorSchema, "elec:motorType");
    EXPECT_EQ(checkField(*type, "bldc"), "");
    EXPECT_NE(checkField(*type, "stepper"), "");
    const OverlayField* port = overlayField(kBatteryPort, "elec:port:dc_out:maxCurrent");
    ASSERT_NE(port, nullptr);
    EXPECT_EQ(displayUnit("ohm"), "Ω");
    EXPECT_EQ(displayUnit("N.m/A"), "N·m/A");
    EXPECT_EQ(displayUnit("kg.m2"), "kg·m²");
    EXPECT_EQ(displayUnit("A2.s"), "A²·s");
    // A converter's efficiency is (0, 1], as the loader has it.
    const OverlayField* eta = overlayField(kConverterSchema, "elec:conv:efficiency");
    ASSERT_NE(eta, nullptr);
    EXPECT_NE(checkField(*eta, "0"), "");
    EXPECT_EQ(checkField(*eta, "1"), "");
    EXPECT_EQ(checkField(*eta, "0.9"), "");
    EXPECT_NE(checkField(*eta, "1.1"), "");
    const OverlayField* kind = overlayField(kBranchSchema, "elec:branch:kind");
    EXPECT_EQ(checkField(*kind, "fuse"), "");
    EXPECT_NE(checkField(*kind, "relay"), "");
    EXPECT_TRUE(branchFieldApplies("fuse", "elec:branch:i2tRating"));
    EXPECT_FALSE(branchFieldApplies("switch", "elec:branch:i2tRating"));
    EXPECT_TRUE(branchFieldApplies("diode", "elec:branch:forwardDrop"));
    EXPECT_TRUE(branchFieldApplies("switch", "elec:branch:initialState"));
    EXPECT_TRUE(branchFieldApplies("resistor", "elec:branch:resistance"));
}

// --- the power system on rover_mk1 ---------------------------------------------------------

Document poweredRover() {
    Document document = urdf(kTestDir / "rover_mk1.urdf");
    const std::string bus = addBus(document, "dc_48v", &busPresets()[0]);
    EXPECT_EQ(bus, "dc_48v");
    EXPECT_EQ(addBattery(document, "battery", -1, &batteryPresets()[0]), "battery");
    EXPECT_EQ(addMotor(document, "left_wheel_joint", -1, &motorPresets()[0]),
              "left_wheel_joint_motor");
    EXPECT_EQ(addMotor(document, "right_wheel_joint", -1, &motorPresets()[0]),
              "right_wheel_joint_motor");
    return document;
}

TEST(Overlay, ABatteryABusAndTwoMotorsAreValid) {
    const Document document = poweredRover();
    const std::vector<Problem> problems = checkOverlay(document);
    EXPECT_TRUE(problems.empty()) << dump(problems);
    EXPECT_EQ(motorOf(document, "left_wheel_joint"), 2);
    const std::string text = writeOverlay(document);
    EXPECT_NE(text.find("  - prim: left_wheel_joint\n    create_child: left_wheel_joint_motor\n"),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("elec:port:dc_out:connectedTo: power/dc_48v"), std::string::npos);
    EXPECT_NE(text.find("elec:powerPort: power/dc_48v"), std::string::npos);
    EXPECT_NE(text.find("elec:battery:ocvSoc: [0.0, 0.1, 0.5, 0.9, 1.0]"), std::string::npos);
    EXPECT_NE(text.find("elec:windingResistance: 0.12  # Ω"), std::string::npos) << text;
    // Reads back to the same entries.
    Document again = urdf(kTestDir / "rover_mk1.urdf");
    ASSERT_TRUE(readOverlay(text, again).ok);
    EXPECT_EQ(writeOverlay(again), text);
    // A second motor on a joint is refused by addMotor.
    Document twice = document;
    EXPECT_EQ(addMotor(twice, "left_wheel_joint", -1, nullptr), "");
}

TEST(Overlay, ValidationRules) {
    {
        // A motor on a fixed joint.
        Document document = poweredRover();
        ASSERT_NE(addMotor(document, "caster_joint", -1, &motorPresets()[0]), "");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "caster_joint", "fixed joint"))
            << dump(checkOverlay(document));
    }
    {
        // A bus without a battery.
        Document document = poweredRover();
        (void)addBus(document, "aux", &busPresets()[1]);
        EXPECT_TRUE(
            hasProblem(checkOverlay(document), Level::Error, "power/aux", "bus without a battery"));
    }
    {
        // A bad value, and a missing required one.
        Document document = poweredRover();
        OverlayEntry& battery = document.overlay.entries[1];
        OverlaySchema& schema = *findSchema(battery, kBatterySchema);
        setAttribute(schema, overlayField(kBatterySchema, "elec:battery:r0"), "elec:battery:r0",
                     "-0.01");
        setAttribute(schema, nullptr, "elec:battery:capacity", "");
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(
            hasProblem(problems, Level::Error, "power/battery", "r0 = -0.01: must be above 0"))
            << dump(problems);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/battery", "capacity: required"));
        // And through validate(), which the Problems panel lists.
        EXPECT_TRUE(hasProblem(validate(document, nullptr, {}), Level::Error, "power/battery",
                               "r0 = -0.01"));
    }
    {
        // An ocv curve of the wrong length, a bus whose trips cross.
        Document document = poweredRover();
        OverlaySchema& battery = *findSchema(document.overlay.entries[1], kBatterySchema);
        setAttribute(battery, overlayField(kBatterySchema, "elec:battery:ocvVoltage"),
                     "elec:battery:ocvVoltage", "3.0, 3.3");
        OverlaySchema& bus = *findSchema(document.overlay.entries[0], kBusSchema);
        setAttribute(bus, nullptr, "elec:bus:overvoltageTrip", "40");
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/battery", "one voltage per point"));
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dc_48v", "overvoltageTrip"));
    }
    {
        // Removing the bus leaves the motors and the battery asking for one.
        Document document = poweredRover();
        ASSERT_TRUE(removeEntry(document, 0));
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "left_wheel_joint", "draws from no bus"))
            << dump(problems);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/battery", "feeds no bus"));
    }
    {
        // An overlay sensor somewhere other than its link's origin.
        Document document = poweredRover();
        const std::string gnss = addSensor(document, "base_link", "gnss");
        findSensor(document, gnss)->origin.xyz = {0.1, 0, 0};
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Warning, gnss, "no mount pose"));
    }
}

// --- loads, converters and branches (RFC 0010) ----------------------------------------------

/// rover_mk1 powered as above, plus a 12 V bus fed from the 48 V one by a converter, with a
/// 20 W load on it: the headless Test's robot (test_builder_power.py).
Document roverWithRail() {
    Document document = poweredRover();
    const std::string rail = addBus(document, "dc_12v", nullptr);
    OverlaySchema& bus = *findSchema(document.overlay.entries.back(), kBusSchema);
    setAttribute(bus, nullptr, "elec:bus:capacitance", "0.001");
    setAttribute(bus, nullptr, "elec:bus:undervoltageTrip", "10.0");
    setAttribute(bus, nullptr, "elec:bus:overvoltageTrip", "14.0");
    const int in = busNamed(document, "dc_48v");
    const int out = busNamed(document, rail);
    EXPECT_EQ(addConverter(document, "dcdc", in, out), "dcdc");
    OverlaySchema& converter = *findSchema(document.overlay.entries.back(), kConverterSchema);
    setAttribute(converter, nullptr, "elec:conv:outputVoltage", "12.0");
    setAttribute(converter, nullptr, "elec:conv:maxOutputCurrent", "5.0");
    setAttribute(converter, nullptr, "elec:conv:efficiency", "0.9");
    EXPECT_EQ(addLoad(document, "computer", out, nullptr), "computer");
    OverlaySchema& load = *findSchema(document.overlay.entries.back(), kLoadSchema);
    setAttribute(load, nullptr, "elec:load:power", "20.0");
    return document;
}

int entryNamed(const Document& document, std::string_view name) {
    for (std::size_t i = 0; i < document.overlay.entries.size(); ++i) {
        if (entryName(document.overlay.entries[i]) == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

TEST(Overlay, AConverterFeedsARailWithALoad) {
    const Document document = roverWithRail();
    const std::vector<Problem> problems = checkOverlay(document);
    EXPECT_TRUE(problems.empty()) << dump(problems);
    const std::string text = writeOverlay(document);
    EXPECT_NE(text.find("  - create: power/dcdc\n    schemas:\n      ElectricalConverterAPI_1:\n"
                        "        elec:conv:outputVoltage: 12.0  # V\n"
                        "        elec:conv:efficiency: 0.9\n"
                        "        elec:conv:maxOutputCurrent: 5.0  # A\n"
                        "        elec:conv:input: power/dc_48v\n"
                        "        elec:conv:output: power/dc_12v\n"),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("        elec:load:power: 20.0  # W\n        elec:powerPort: power/dc_12v"),
              std::string::npos)
        << text;
    Document again = urdf(kTestDir / "rover_mk1.urdf");
    ASSERT_TRUE(readOverlay(text, again).ok);
    EXPECT_EQ(writeOverlay(again), text);
    // The signals the run will have, by the network's names.
    const auto signals = powerSignals(
        document.overlay.entries[static_cast<std::size_t>(entryNamed(document, "computer"))]);
    ASSERT_EQ(signals.size(), 2U);
    EXPECT_EQ(signals[0].first, "load/computer/current");
    EXPECT_EQ(powerSignals(
                  document.overlay.entries[static_cast<std::size_t>(entryNamed(document, "dcdc"))])
                  .front()
                  .first,
              "converter/dcdc/output_current");
}

TEST(Overlay, ConverterRules) {
    {
        // No voltage, no current limit, efficiency 0, no input bus.
        Document document = roverWithRail();
        OverlaySchema& c = *findSchema(
            document.overlay.entries[static_cast<std::size_t>(entryNamed(document, "dcdc"))],
            kConverterSchema);
        setAttribute(c, nullptr, "elec:conv:outputVoltage", "");
        setAttribute(c, nullptr, "elec:conv:maxOutputCurrent", "-1");
        setAttribute(c, nullptr, "elec:conv:efficiency", "0");
        setAttribute(c, nullptr, "elec:conv:input", "power/nowhere");
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dcdc", "outputVoltage: required"))
            << dump(problems);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dcdc", "maxOutputCurrent = -1"));
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dcdc",
                               "efficiency = 0: must be above 0 and at most 1"));
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dcdc",
                               "input power/nowhere is not a bus of the overlay"));
        // Its rail is then fed by nothing a battery reaches.
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dc_12v", "bus without a battery"));
    }
    {
        // The same bus at both ends; a second converter onto the rail.
        Document document = roverWithRail();
        Document twice = document;
        OverlaySchema& c = *findSchema(
            document.overlay.entries[static_cast<std::size_t>(entryNamed(document, "dcdc"))],
            kConverterSchema);
        setAttribute(c, nullptr, "elec:conv:output", "power/dc_48v");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "power/dcdc", "same bus"));
        (void)addConverter(twice, "dcdc", busNamed(twice, "dc_48v"), busNamed(twice, "dc_12v"));
        EXPECT_TRUE(hasProblem(checkOverlay(twice), Level::Error, "power/dcdc_2",
                               "two converters feed dc_12v"))
            << dump(checkOverlay(twice));
    }
    {
        // A loop: 48 V → 12 V → 48 V.
        Document document = roverWithRail();
        const std::string back = addConverter(document, "back", busNamed(document, "dc_12v"),
                                              busNamed(document, "dc_48v"));
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Warning, "power/back", "converters in a loop"))
            << dump(problems);
        EXPECT_TRUE(hasProblem(problems, Level::Warning, "power/dcdc", "converters in a loop"));
    }
    {
        // Removing the rail leaves the converter's output and the load asking for a bus.
        Document document = roverWithRail();
        ASSERT_TRUE(removeEntry(document, static_cast<std::size_t>(busNamed(document, "dc_12v"))));
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/dcdc", "output: no bus picked"))
            << dump(problems);
        EXPECT_TRUE(
            hasProblem(problems, Level::Error, "power/computer", "draws from: no bus picked"));
    }
}

TEST(Overlay, LoadAndBranchRules) {
    {
        // A load without a power; a resistive one with a resistance needs none.
        Document document = roverWithRail();
        OverlaySchema& load = *findSchema(
            document.overlay.entries[static_cast<std::size_t>(entryNamed(document, "computer"))],
            kLoadSchema);
        setAttribute(load, nullptr, "elec:load:power", "");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "power/computer",
                               "needs elec:load:power"));
        setAttribute(load, nullptr, "elec:load:model", "resistive");
        setAttribute(load, nullptr, "elec:load:resistance", "7.2");
        EXPECT_TRUE(checkOverlay(document).empty()) << dump(checkOverlay(document));
        setAttribute(load, nullptr, "elec:load:minVoltage", "-3");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "power/computer",
                               "minVoltage = -3: must not be negative"));
    }
    {
        // A fuse from the 48 V bus feeds an aux bus; without a rating it is refused.
        Document document = poweredRover();
        (void)addBus(document, "aux", &busPresets()[0]);
        const std::string fuse = addBranch(document, "", "fuse", busNamed(document, "dc_48v"),
                                           busNamed(document, "aux"));
        EXPECT_EQ(fuse, "fuse");
        OverlaySchema& b = *findSchema(document.overlay.entries.back(), kBranchSchema);
        EXPECT_EQ(attributeText(b, "elec:branch:kind"), "fuse");
        setAttribute(b, nullptr, "elec:branch:resistance", "0.01");
        std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(hasProblem(problems, Level::Error, "power/fuse", "i2tRating above 0"))
            << dump(problems);
        // The aux bus is fed through the fuse: no "bus without a battery".
        EXPECT_FALSE(hasProblem(problems, Level::Error, "power/aux", "bus without a battery"));
        setAttribute(b, nullptr, "elec:branch:i2tRating", "400");
        EXPECT_TRUE(checkOverlay(document).empty()) << dump(checkOverlay(document));
        // A value for another kind is a hint; both ends on one bus an error.
        setAttribute(b, nullptr, "elec:branch:forwardDrop", "0.7");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Hint, "power/fuse", "forwardDrop"));
        setAttribute(b, nullptr, "elec:branch:output", "power/dc_48v");
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "power/fuse", "same bus"));
    }
    {
        // A diode feeds only forward: from aux to the battery's bus it feeds nothing.
        Document document = poweredRover();
        (void)addBus(document, "aux", &busPresets()[0]);
        (void)addBranch(document, "", "diode", busNamed(document, "aux"),
                        busNamed(document, "dc_48v"));
        setAttribute(*findSchema(document.overlay.entries.back(), kBranchSchema), nullptr,
                     "elec:branch:resistance", "0.05");
        EXPECT_TRUE(
            hasProblem(checkOverlay(document), Level::Error, "power/aux", "bus without a battery"));
        // Unpicked ends are said.
        (void)addBranch(document, "", "switch", -1, -1);
        EXPECT_TRUE(hasProblem(checkOverlay(document), Level::Error, "power/switch",
                               "the input: no bus picked"));
    }
}

TEST(Overlay, PartsAreUndoSteps) {
    History history(poweredRover(), "Open");
    Document next = history.current();
    (void)addConverter(next, "dcdc", 0, -1);
    history.commit(std::move(next), "Add converter dcdc", {}, 0.0);
    next = history.current();
    (void)addBranch(next, "", "switch", 0, -1);
    history.commit(std::move(next), "Add switch switch", {}, 0.0);
    EXPECT_EQ(entriesWith(history.current(), kBranchSchema).size(), 1U);
    ASSERT_TRUE(history.undo());
    EXPECT_TRUE(entriesWith(history.current(), kBranchSchema).empty());
    EXPECT_EQ(entriesWith(history.current(), kConverterSchema).size(), 1U);
    ASSERT_TRUE(history.undo());
    EXPECT_TRUE(entriesWith(history.current(), kConverterSchema).empty());
}

/// The overlays tools/ci/check_electrical.py writes beside the netlists it checks against
/// ngspice (the same circuits as their `*lagrange converter` and branch lines): a converter
/// rail with a resistive load, and a fuse, a switch and a diode to a far bus.
constexpr const char* kBattery = R"(  - create: power/a
    schemas:
      BatteryAPI_1:
        elec:battery:seriesCells: 1
        elec:battery:capacity: 360000.0
        elec:battery:ocvSoc: [0.0, 1.0]
        elec:battery:ocvVoltage: [48.0, 48.0]
        elec:battery:r0: 0.1
        elec:battery:cutoffVoltage: 1.0
        elec:battery:maxChargeCurrent: 10.0
      PortAPI_1:dc_out:
        elec:port:dc_out:connectedTo: power/dc
  - create: power/dc
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.0001
        elec:bus:undervoltageTrip: 0.5
        elec:bus:overvoltageTrip: 1000.0
)";

std::string netlistOverlay(const std::string& parts) {
    return "schema_version: 1\napply:\n" + std::string(kBattery) + parts;
}

std::string farSide(const std::string& kind, const std::string& extra) {
    return R"(  - create: power/far
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.0001
        elec:bus:undervoltageTrip: 0.0
        elec:bus:overvoltageTrip: 1000.0
  - create: power/part
    schemas:
      ElectricalBranchAPI_1:
        elec:branch:kind: )" +
           kind + R"(
        elec:branch:resistance: 0.25
        elec:branch:input: power/dc
        elec:branch:output: power/far
)" + extra +
           R"(  - create: power/load
    schemas:
      ElectricalLoadAPI_1:
        elec:load:model: resistive
        elec:load:resistance: 12.0
        elec:powerPort: power/far
)";
}

TEST(Overlay, NetlistCircuitsReadAndWriteTheSame) {
    const std::vector<std::string> overlays{
        netlistOverlay(R"(  - create: power/rail
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.0001
        elec:bus:undervoltageTrip: 0.5
        elec:bus:overvoltageTrip: 1000.0
  - create: power/load
    schemas:
      ElectricalLoadAPI_1:
        elec:load:model: resistive
        elec:load:power: 24.0
        elec:powerPort: power/rail
  - create: power/dcdc
    schemas:
      ElectricalConverterAPI_1:
        elec:conv:outputVoltage: 12.0
        elec:conv:maxOutputCurrent: 1000.0
        elec:conv:efficiency: 0.9
        elec:conv:input: power/dc
        elec:conv:output: power/rail
)"),
        netlistOverlay(farSide("resistor", "")),
        netlistOverlay(farSide("fuse", "        elec:branch:i2tRating: 50.0\n")),
        netlistOverlay(farSide("switch", "        elec:branch:initialState: open\n")),
        netlistOverlay(farSide("diode", "        elec:branch:forwardDrop: 0.7\n")),
    };
    for (const std::string& original : overlays) {
        Document document = newDocument();
        const OverlayReadResult read = readOverlay(original, document);
        ASSERT_TRUE(read.ok) << read.error;
        const std::string written = writeOverlay(document);
        std::string why;
        EXPECT_TRUE(sameOverlay(original, written, &why)) << why << "\n" << written;
        // Every part has a form, none is "kept as written", and none is a problem.
        for (const OverlayEntry& entry : document.overlay.entries) {
            EXPECT_TRUE(std::ranges::any_of(entry.schemas, [](const OverlaySchema& s) {
                return !fieldsOf(s.name).empty();
            })) << entryPrim(entry);
        }
        const std::vector<Problem> problems = checkOverlay(document);
        EXPECT_TRUE(problems.empty()) << dump(problems) << original;
        // An edit through the forms keeps the rest as it was.
        const int part = entryNamed(document, "part");
        if (part >= 0) {
            OverlaySchema& b = *findSchema(document.overlay.entries[static_cast<std::size_t>(part)],
                                           kBranchSchema);
            setAttribute(b, overlayField(kBranchSchema, "elec:branch:resistance"),
                         "elec:branch:resistance", "0.5");
            EXPECT_TRUE(sameOverlay(
                std::regex_replace(original, std::regex("resistance: 0.25"), "resistance: 0.5"),
                writeOverlay(document), &why))
                << why;
        }
    }
}

TEST(Overlay, EditsFollowRenamesAndRemovals) {
    Document document = poweredRover();
    ASSERT_EQ(rename(document, ItemKind::Joint, "left_wheel_joint", "left_drive"), "");
    EXPECT_EQ(motorOf(document, "left_drive"), 2);
    EXPECT_EQ(document.overlay.entries[2].prim, "left_drive");
    EXPECT_TRUE(checkOverlay(document).empty()) << dump(checkOverlay(document));
    // Removing the wheel's link removes its joint and so its motor.
    ASSERT_TRUE(removeLink(document, "right_wheel"));
    EXPECT_EQ(motorOf(document, "right_wheel_joint"), -1);
    EXPECT_EQ(document.overlay.entries.size(), 3U);
    // An overlay sensor: added like any sensor, written to the overlay, gone with its link.
    const std::string gnss = addSensor(document, "mast", "gnss");
    EXPECT_NE(writeOverlay(document).find("create_child: " + gnss), std::string::npos);
    EXPECT_EQ(writeUrdf(document).find("type=\"gnss\""), std::string::npos);
    ASSERT_TRUE(removeLink(document, "mast"));
    EXPECT_EQ(findSensor(document, gnss), nullptr);
    EXPECT_EQ(writeOverlay(document).find(gnss), std::string::npos);
}

TEST(Overlay, EditsAreUndoSteps) {
    History history(urdf(kTestDir / "rover_mk1.urdf"), "Open");
    Document next = history.current();
    (void)addBus(next, "dc_48v", nullptr);
    history.commit(std::move(next), "Add bus dc_48v", {}, 0.0);
    EXPECT_TRUE(hasOverlay(history.current()));
    ASSERT_TRUE(history.undo());
    EXPECT_FALSE(hasOverlay(history.current()));
    ASSERT_TRUE(history.redo());
    EXPECT_EQ(entriesWith(history.current(), kBusSchema).size(), 1U);
}

}  // namespace
}  // namespace lagrange::viewer::builder
