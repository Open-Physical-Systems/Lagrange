// The Domains workspace's secondary tabs, without a window: Diff, Schema, Coupling.
#include "lagrange/viewer/ProfileExtras.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

using Kind = DiffLine::Kind;

TEST(ProfileDiff, EqualTextsHaveNoChanges) {
    const std::vector<DiffLine> diff = diffLines("a\nb\nc\n", "a\nb\nc\n");
    ASSERT_EQ(diff.size(), 3U);
    EXPECT_TRUE(std::ranges::all_of(diff, [](const DiffLine& l) { return l.kind == Kind::Same; }));
    EXPECT_EQ(diffHunks(diff), 0U);
}

TEST(ProfileDiff, AChangedLineIsRemovedThenAdded) {
    const std::vector<DiffLine> diff = diffLines(
        "domains:\n  thermal: {enabled: true}\n  emi: {enabled: true}\n",
        "domains:\n  thermal: {enabled: false, substitute: ambient}\n  emi: {enabled: true}\n");
    ASSERT_EQ(diff.size(), 4U);
    EXPECT_EQ(diff[0].kind, Kind::Same);
    EXPECT_EQ(diff[1].kind, Kind::Removed);
    EXPECT_EQ(diff[1].beforeLine, 2);
    EXPECT_EQ(diff[1].afterLine, 0);
    EXPECT_EQ(diff[2].kind, Kind::Added);
    EXPECT_EQ(diff[2].afterLine, 2);
    EXPECT_EQ(diff[2].text, "  thermal: {enabled: false, substitute: ambient}");
    EXPECT_EQ(diff[3].kind, Kind::Same);
    EXPECT_EQ(diff[3].beforeLine, 3);
    EXPECT_EQ(diff[3].afterLine, 3);
    EXPECT_EQ(diffHunks(diff), 1U);
}

TEST(ProfileDiff, InsertionsAndDeletionsKeepLineNumbers) {
    const std::vector<DiffLine> diff = diffLines("a\nb\nc\nd\n", "a\nx\nb\nd\ny\n");
    // a, +x, b, -c, d, +y
    ASSERT_EQ(diff.size(), 6U);
    EXPECT_EQ(diff[1].kind, Kind::Added);
    EXPECT_EQ(diff[3].kind, Kind::Removed);
    EXPECT_EQ(diff[3].text, "c");
    EXPECT_EQ(diff[4].beforeLine, 4);
    EXPECT_EQ(diff[4].afterLine, 4);
    EXPECT_EQ(diff[5].kind, Kind::Added);
    EXPECT_EQ(diffHunks(diff), 3U);
}

TEST(ProfileSchema, ReadsTheShippedSchema) {
    const ProfileSchemaDoc doc = loadProfileSchema(LAGRANGE_PROFILE_SCHEMA);
    ASSERT_TRUE(doc.error.empty()) << doc.error;
    EXPECT_EQ(doc.version, 1U);
    EXPECT_NE(doc.dialect.find("2020-12"), std::string::npos);
    // The eight physics domains are required.
    EXPECT_EQ(doc.requiredDomains.size(), 8U);
    const SchemaSection* thermal = doc.domain("thermal");
    ASSERT_NE(thermal, nullptr);
    const auto find = [](const SchemaSection& section, std::string_view key) {
        const auto it = std::ranges::find(section.fields, key, &SchemaField::key);
        return it == section.fields.end() ? nullptr : &*it;
    };
    // Keys from the shared definition (common) and the domain's own.
    const SchemaField* enabled = find(*thermal, "enabled");
    ASSERT_NE(enabled, nullptr);
    EXPECT_TRUE(enabled->required);
    EXPECT_EQ(enabled->type, "boolean");
    const SchemaField* dt = find(*thermal, "dt");
    ASSERT_NE(dt, nullptr);
    EXPECT_EQ(dt->type, "number");
    EXPECT_EQ(dt->constraints, "> 0, ≤ 10");
    EXPECT_NE(dt->description.find("seconds"), std::string::npos);
    const SchemaField* fidelity = find(*thermal, "fidelity");
    ASSERT_NE(fidelity, nullptr);
    EXPECT_EQ(fidelity->type, "one of: lumped | conduction");
    const SchemaField* substitute = find(*thermal, "substitute");
    ASSERT_NE(substitute, nullptr);
    EXPECT_NE(substitute->type.find("ideal | zero"), std::string::npos);
    EXPECT_NE(substitute->type.find("{kind"), std::string::npos);
    // The rule that a disabled domain names a substitute.
    EXPECT_NE(thermal->note.find("substitute"), std::string::npos);
    // The top level, without `domains`, with nested objects one level deeper.
    EXPECT_EQ(find(doc.top, "domains"), nullptr);
    const SchemaField* stage = find(doc.top, "stage");
    ASSERT_NE(stage, nullptr);
    EXPECT_TRUE(stage->required);
    const SchemaField* rtf = find(doc.top, "real_time_factor");
    ASSERT_NE(rtf, nullptr);
    EXPECT_EQ(rtf->depth, 1);
    EXPECT_EQ(rtf->defaultValue, "1.0");
}

TEST(ProfileSchema, AMissingOrBrokenFileIsAnError) {
    EXPECT_FALSE(loadProfileSchema({}).error.empty());
    const std::filesystem::path broken =
        std::filesystem::temp_directory_path() / "lagrange_test_broken_schema.json";
    std::ofstream(broken) << "{\"title\": [unclosed";
    const ProfileSchemaDoc doc = loadProfileSchema(broken);
    EXPECT_FALSE(doc.error.empty());
    std::filesystem::remove(broken);
}

TEST(Coupling, GathersWiresByPair) {
    WiringInfo wiring;
    wiring.nodes = {
        {"electrical", "electrical", false, "averaged", 100'000, {}, {}, false},
        {"rigid_body", "rigid_body", false, "mujoco", 1'000'000, {}, {}, false},
        {"thermal", "thermal", false, "lumped", 100'000'000, {}, {}, false},
        {"aero_zero", "aerodynamics", true, "zero", 0, {}, {}, false},
    };
    wiring.wires = {
        {"electrical", "rigid_body", "joint/l/torque", "τ", "N·m", "N·m", true, true},
        {"electrical", "rigid_body", "joint/r/torque", "τ", "N·m", "N·m", true, true},
        {"rigid_body", "electrical", "joint/l/velocity", "ω", "rad/s", "rad/s", true, true},
        {"thermal", "electrical", "thermal/m/temperature", "T", "K", "K", true, false},
        {"aero_zero", "rigid_body", "link/b/ext_wrench", "F", "N", "N·m", false, false},
    };
    const std::vector<Coupling> pairs = couplings(wiring);
    ASSERT_EQ(pairs.size(), 4U);
    EXPECT_EQ(pairs[0].from, "electrical");
    EXPECT_EQ(pairs[0].wires, 2U);
    ASSERT_EQ(pairs[0].symbols.size(), 1U);
    EXPECT_EQ(pairs[0].symbols[0], "τ · N·m");
    EXPECT_TRUE(pairs[0].stiff);
    EXPECT_TRUE(pairs[0].twoWay);
    EXPECT_EQ(pairs[0].holdSteps, 1);  // the slower reader reads every new value
    // The 100 µs electrical reads the 100 ms thermal 1000 times per value.
    EXPECT_EQ(pairs[2].from, "thermal");
    EXPECT_EQ(pairs[2].holdSteps, 1000);
    EXPECT_FALSE(pairs[2].twoWay);
    EXPECT_TRUE(pairs[3].fromSubstitute);
    EXPECT_EQ(pairs[3].unitConflicts, 1U);
}

}  // namespace
}  // namespace lagrange::viewer
