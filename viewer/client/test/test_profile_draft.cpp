#include "lagrange/viewer/ProfileDraft.hpp"

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

constexpr const char* kProfile =
    "# Lagrange simulation profile\n"
    "schema_version: 1\n"
    "stage: worlds/warehouse.usda\n"
    "\n"
    "domains:\n"
    "  rigid_body: {enabled: true, backend: mujoco, dt: 0.001}\n"
    "  emi: {enabled: true, coupling: [magnetometer, can_bus], dt: 0.01}\n"
    "  aerodynamics: {enabled: false, substitute: zero}\n"
    "\n"
    "views: [physics_3d, waveforms]\n"
    "record: {format: mcap, path: \"runs/a, b.mcap\"}\n";

TEST(ProfileDraft, StartsEqualToTheAppliedProfile) {
    ProfileDraft draft;
    draft.reset(kProfile);
    EXPECT_FALSE(draft.dirty());
    EXPECT_EQ(draft.text(), kProfile);
    EXPECT_EQ(draft.lines().size(), 11U);
}

TEST(ProfileDraft, ReadsADomainLine) {
    ProfileDraft draft;
    draft.reset(kProfile);
    const auto rigid = draft.domain("rigid_body");
    ASSERT_TRUE(rigid.has_value());
    ASSERT_EQ(rigid->size(), 3U);
    EXPECT_EQ(*findEntry(*rigid, "enabled"), "true");
    EXPECT_EQ(*findEntry(*rigid, "backend"), "mujoco");
    EXPECT_EQ(*findEntry(*rigid, "dt"), "0.001");
    EXPECT_EQ(findEntry(*rigid, "substitute"), nullptr);
}

TEST(ProfileDraft, KeepsListsWithCommasWhole) {
    ProfileDraft draft;
    draft.reset(kProfile);
    const auto emi = draft.domain("emi");
    ASSERT_TRUE(emi.has_value());
    ASSERT_EQ(emi->size(), 3U);
    EXPECT_EQ(*findEntry(*emi, "coupling"), "[magnetometer, can_bus]");
}

TEST(ProfileDraft, DoesNotMistakeOtherSectionsForDomains) {
    ProfileDraft draft;
    draft.reset(kProfile);
    EXPECT_FALSE(draft.domain("record").has_value());
    EXPECT_FALSE(draft.domain("thermal").has_value());
    EXPECT_FALSE(draft.domain("rigid").has_value());
}

TEST(ProfileDraft, AnEditReplacesOneLineAndKeepsTheRest) {
    ProfileDraft draft;
    draft.reset(kProfile);
    const std::uint64_t before = draft.revision();
    ASSERT_TRUE(draft.setDomain("aerodynamics", {{"enabled", "true"}, {"fidelity", "drag"}}));
    EXPECT_GT(draft.revision(), before);
    EXPECT_TRUE(draft.dirty());
    EXPECT_EQ(draft.changedLineCount(), 1U);
    EXPECT_TRUE(draft.lineChanged(7));
    EXPECT_TRUE(draft.domainChanged("aerodynamics"));
    EXPECT_FALSE(draft.domainChanged("rigid_body"));
    EXPECT_EQ(draft.lines()[7], "  aerodynamics: {enabled: true, fidelity: drag}");
    ASSERT_NE(draft.appliedLine(7), nullptr);
    EXPECT_EQ(*draft.appliedLine(7), "  aerodynamics: {enabled: false, substitute: zero}");
    EXPECT_EQ(draft.lines()[10], "record: {format: mcap, path: \"runs/a, b.mcap\"}");
    EXPECT_EQ(*findEntry(*draft.appliedDomain("aerodynamics"), "substitute"), "zero");
}

TEST(ProfileDraft, WritingTheSameSettingsBackIsNotAChange) {
    ProfileDraft draft;
    draft.reset(kProfile);
    ASSERT_TRUE(draft.setDomain("aerodynamics", {{"enabled", "true"}}));
    ASSERT_TRUE(draft.setDomain("aerodynamics", *draft.appliedDomain("aerodynamics")));
    EXPECT_FALSE(draft.dirty());
}

TEST(ProfileDraft, ANewDomainIsAddedAtTheEndOfTheBlock) {
    ProfileDraft draft;
    draft.reset(kProfile);
    ASSERT_TRUE(draft.setDomain("comms", {{"enabled", "true"}, {"dt", "0.01"}}));
    ASSERT_EQ(draft.lines().size(), 12U);
    EXPECT_EQ(draft.lines()[8], "  comms: {enabled: true, dt: 0.01}");
    EXPECT_EQ(draft.lines()[9], "");
    EXPECT_EQ(draft.appliedLine(8), nullptr);
    EXPECT_TRUE(draft.lineChanged(8));
    EXPECT_FALSE(draft.lineChanged(9));  // lines after the insertion still match their origin
    EXPECT_FALSE(draft.lineChanged(11));
    EXPECT_EQ(draft.changedLineCount(), 1U);
}

TEST(ProfileDraft, RevertDropsEdits) {
    ProfileDraft draft;
    draft.reset(kProfile);
    ASSERT_TRUE(draft.setDomain("comms", {{"enabled", "true"}}));
    ASSERT_TRUE(draft.setDomain("emi", {{"enabled", "false"}, {"substitute", "zero"}}));
    draft.revert();
    EXPECT_FALSE(draft.dirty());
    EXPECT_EQ(draft.text(), kProfile);
}

TEST(ProfileDraft, AProfileWithoutDomainsCannotBeEdited) {
    ProfileDraft draft;
    draft.reset("stage: a.usda\n");
    EXPECT_FALSE(draft.setDomain("thermal", {{"enabled", "true"}}));
    EXPECT_FALSE(draft.dirty());
}

TEST(ProfileDraft, ACommentAfterADomainLineIsTolerated) {
    ProfileDraft draft;
    draft.reset("domains:\n  thermal: {enabled: true, dt: 0.1}  # slow\n");
    const auto thermal = draft.domain("thermal");
    ASSERT_TRUE(thermal.has_value());
    EXPECT_EQ(*findEntry(*thermal, "dt"), "0.1");
}

TEST(ProfileEntries, SetReplacesOrAppendsAndEraseRemoves) {
    std::vector<ProfileEntry> entries{{"enabled", "true"}};
    setEntry(entries, "dt", "0.001");
    setEntry(entries, "enabled", "false");
    ASSERT_EQ(entries.size(), 2U);
    EXPECT_EQ(entries[0].value, "false");
    eraseEntry(entries, "dt");
    EXPECT_EQ(entries.size(), 1U);
}

TEST(ProfileSeconds, FormatsWholeNanosecondsExactly) {
    EXPECT_EQ(formatSeconds(100'000), "0.0001");
    EXPECT_EQ(formatSeconds(1'000'000), "0.001");
    EXPECT_EQ(formatSeconds(50'000), "0.00005");
    EXPECT_EQ(formatSeconds(1'500'000'000), "1.5");
    EXPECT_EQ(formatSeconds(2'000'000'000), "2");
    EXPECT_EQ(formatSeconds(1), "0.000000001");
}

TEST(ProfileSeconds, ParsesWhatItFormats) {
    for (const std::int64_t ns : {1LL, 50'000LL, 100'000LL, 1'000'000LL, 1'500'000'000LL}) {
        EXPECT_EQ(parseSeconds(formatSeconds(ns)), ns);
    }
    EXPECT_EQ(parseSeconds("0.1"), 100'000'000);
    EXPECT_EQ(parseSeconds(" 2 "), 2'000'000'000);
}

TEST(ProfileSeconds, RejectsWhatIsNotAPositiveStep) {
    EXPECT_FALSE(parseSeconds("").has_value());
    EXPECT_FALSE(parseSeconds("0").has_value());
    EXPECT_FALSE(parseSeconds("-0.1").has_value());
    EXPECT_FALSE(parseSeconds("1e-3").has_value());
    EXPECT_FALSE(parseSeconds("0.0000000001").has_value());  // finer than a nanosecond
    EXPECT_FALSE(parseSeconds("abc").has_value());
}

}  // namespace
}  // namespace lagrange::viewer
