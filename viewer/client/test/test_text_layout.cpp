// TextFit, LabelLayout, SignalList and Fuzzy: the layout and search rules of the viewer's lists
// and overlays, without ImGui.
#include "lagrange/viewer/Fuzzy.hpp"
#include "lagrange/viewer/LabelLayout.hpp"
#include "lagrange/viewer/SignalList.hpp"
#include "lagrange/viewer/TextFit.hpp"

#include <algorithm>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

/// Every character (not byte) is 1 wide, as in a mono font.
float characters(std::string_view text) {
    float n = 0.0F;
    for (const char ch : text) {
        n += (static_cast<unsigned char>(ch) & 0xC0U) != 0x80U ? 1.0F : 0.0F;
    }
    return n;
}

TEST(TextFit, KeepsWhatFits) {
    EXPECT_EQ(fitEnd("Mobile robot", 12.0F, characters), "Mobile robot");
    EXPECT_EQ(fitMiddle("joint/left/position", 19.0F, characters), "joint/left/position");
}

TEST(TextFit, CutsTheEndWithAnEllipsis) {
    EXPECT_EQ(fitEnd("Mobile robot / AMR", 8.0F, characters), "Mobile …");
    EXPECT_EQ(fitEnd("abc", 0.5F, characters), "…");
}

TEST(TextFit, CutsTheMiddleKeepingBothEnds) {
    const std::string fitted =
        fitMiddle("joint/front_caster_swivel_joint/position", 20.0F, characters);
    EXPECT_EQ(characters(fitted), 20.0F);
    EXPECT_TRUE(fitted.starts_with("joint/fr"));
    EXPECT_TRUE(fitted.ends_with("/position"));
    EXPECT_NE(fitted.find("…"), std::string::npos);
}

TEST(TextFit, APathKeepsItsLastPartWhole) {
    const std::string fitted =
        fitPath("joint/front_caster_swivel_joint/position", 28.0F, characters);
    EXPECT_EQ(fitted, "joint/front_caster…/position");
    EXPECT_EQ(fitPath("motor/left/current_q", 30.0F, characters), "motor/left/current_q");
    // No room for the start: cut in the middle instead.
    const std::string tight = fitPath("a/very_long_last_part_of_it", 10.0F, characters);
    EXPECT_EQ(characters(tight), 10.0F);
    // A start that would keep a few characters only: the middle goes instead.
    const std::string middle =
        fitPath("joint/front_caster_swivel/constraint_torque", 24.0F, characters);
    EXPECT_EQ(characters(middle), 24.0F);
    EXPECT_TRUE(middle.starts_with("joint/front"));
    EXPECT_TRUE(middle.ends_with("_torque"));
}

TEST(TextFit, NamesThatWouldReadTheSameAreCutApart) {
    const std::vector<std::string> group{"joint/front_caster_swivel_joint/position",
                                         "joint/front_caster_wheel_joint/position"};
    // fitPath cuts both to the same text.
    ASSERT_EQ(fitPath(group[0], 26.0F, characters), fitPath(group[1], 26.0F, characters));
    const std::string swivel = fitDistinct(group[0], group, 26.0F, characters);
    const std::string wheel = fitDistinct(group[1], group, 26.0F, characters);
    EXPECT_NE(swivel, wheel);
    EXPECT_LE(characters(swivel), 26.0F);
    EXPECT_LE(characters(wheel), 26.0F);
    EXPECT_TRUE(swivel.ends_with("/position"));
    EXPECT_NE(swivel.find("swivel"), std::string::npos) << swivel;
    // Names that differ only at the start keep their starts.
    const std::vector<std::string> sides{"joint/left_wheel_joint/very_long_quantity_name",
                                         "joint/right_wheel_joint/very_long_quantity_name"};
    EXPECT_NE(fitDistinct(sides[0], sides, 30.0F, characters),
              fitDistinct(sides[1], sides, 30.0F, characters));
    // Alone, it is fitPath.
    EXPECT_EQ(fitDistinct(group[0], std::span<const std::string>{}, 26.0F, characters),
              fitPath(group[0], 26.0F, characters));
}

TEST(TextFit, NeverCutsInsideACharacter) {
    // "°" and "·" are two bytes each.
    const std::string fitted = fitEnd("°°°°°°", 4.0F, characters);
    EXPECT_EQ(fitted, "°°°…");
    const std::string middle = fitMiddle("a·b·c·d·e·f", 5.0F, characters);
    EXPECT_EQ(characters(middle), 5.0F);
    EXPECT_EQ(middle, "a·…·f");
}

TEST(LabelLayout, LabelsThatDoNotTouchStayAtTheirPoints) {
    const std::vector<float> top = stackLabels({100.0F, 200.0F}, 10.0F, 2.0F, 0.0F, 500.0F);
    EXPECT_FLOAT_EQ(top[0], 95.0F);
    EXPECT_FLOAT_EQ(top[1], 195.0F);
}

TEST(LabelLayout, LabelsAtOnePointAreStackedApart) {
    const std::vector<float> top =
        stackLabels({100.0F, 100.0F, 101.0F, 99.0F}, 10.0F, 2.0F, 0.0F, 500.0F);
    std::vector<float> sorted = top;
    std::ranges::sort(sorted);
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        EXPECT_GE(sorted[i] - sorted[i - 1], 12.0F - 1e-4F);
    }
    // The one with the highest point is on top.
    EXPECT_FLOAT_EQ(top[3], sorted[0]);
}

TEST(LabelLayout, StaysInsideTheImage) {
    const std::vector<float> top = stackLabels({495.0F, 498.0F, 499.0F}, 10.0F, 2.0F, 0.0F, 500.0F);
    for (const float y : top) {
        EXPECT_GE(y, 0.0F);
        EXPECT_LE(y + 10.0F, 500.0F);
    }
    const std::vector<float> high = stackLabels({-50.0F, 1.0F}, 10.0F, 2.0F, 0.0F, 500.0F);
    EXPECT_FLOAT_EQ(high[0], 0.0F);
    EXPECT_FLOAT_EQ(high[1], 12.0F);
}

SignalCatalog sampleCatalog() {
    SignalCatalog catalog;
    const auto add = [&catalog](const char* name, const char* unit, const char* writer,
                                std::uint32_t components, std::vector<std::string> names,
                                std::int64_t dt) {
        SignalInfo info;
        info.id = static_cast<std::uint32_t>(catalog.signals.size());
        info.name = name;
        info.unit = unit;
        info.writer = writer;
        info.components = components;
        info.componentNames = std::move(names);
        info.dtNs = dt;
        catalog.signals.push_back(info);
    };
    add("joint/left_wheel_joint/position", "rad", "rigid_body/mujoco", 1, {}, 1'000'000);
    add("body/base_link/pose", "m", "rigid_body/mujoco", 7, {"x", "y", "z", "qw", "qx", "qy", "qz"},
        1'000'000);
    std::vector<std::string> beams;
    for (int i = 0; i < 1080; ++i) {
        beams.push_back(std::to_string(i));
    }
    add("sensor/front_lidar/ranges", "m", "sensors/lidar", 1080, beams, 100'000'000);
    add("sensor/camera/image", "1", "sensors/camera", 0, {}, 33'000'000);
    return catalog;
}

TEST(SignalList, AnArrayIsOneRowUntilOpened) {
    const SignalCatalog catalog = sampleCatalog();
    std::size_t listed = 0;
    const std::vector<SignalRow> rows = signalRows(catalog, "sensors", {}, {}, &listed);
    ASSERT_EQ(rows.size(), 1U);  // the ranges as one row; an image has no samples to plot
    EXPECT_EQ(rows[0].component, SignalRow::kWhole);
    EXPECT_FALSE(rows[0].expanded);
    EXPECT_EQ(listed, 1U);

    const std::set<std::string, std::less<>> open{"sensor/front_lidar/ranges"};
    const std::vector<SignalRow> opened = signalRows(catalog, "sensors", {}, open);
    ASSERT_EQ(opened.size(), 1081U);
    EXPECT_TRUE(opened[0].expanded);
    EXPECT_EQ(opened[1080].component, 1079U);
}

TEST(SignalList, ScalarsAreRowsOfTheirOwn) {
    const SignalCatalog catalog = sampleCatalog();
    std::size_t listed = 0;
    const std::vector<SignalRow> rows = signalRows(catalog, "rigid_body", {}, {}, &listed);
    ASSERT_EQ(rows.size(), 2U);
    EXPECT_EQ(rows[0].component, 0U);
    EXPECT_EQ(rows[1].component, SignalRow::kWhole);
    EXPECT_EQ(listed, 2U);
}

TEST(SignalList, SearchFindsComponents) {
    const SignalCatalog catalog = sampleCatalog();
    SignalFilter filter;
    filter.search = "ranges.10";  // 10, 100 ... 109, 1000 ... 1079
    std::size_t listed = 0;
    const std::vector<SignalRow> rows = signalRows(catalog, "sensors", filter, {}, &listed);
    ASSERT_GE(rows.size(), 2U);
    EXPECT_EQ(rows[0].component, SignalRow::kWhole);
    EXPECT_TRUE(rows[0].expanded);
    EXPECT_EQ(rows[1].component, 10U);
    EXPECT_EQ(rows.size(), 1U + 1U + 10U + 80U);
    EXPECT_EQ(listed, 1U);

    filter.search = "pose.qw";
    const std::vector<SignalRow> pose = signalRows(catalog, "rigid_body", filter, {});
    ASSERT_EQ(pose.size(), 2U);
    EXPECT_EQ(pose[1].component, 3U);

    filter.search = "base_link";  // the name itself: the row stays closed
    const std::vector<SignalRow> named = signalRows(catalog, "rigid_body", filter, {});
    ASSERT_EQ(named.size(), 1U);
    EXPECT_FALSE(named[0].expanded);
}

TEST(SignalList, FiltersByUnitAndRate) {
    const SignalCatalog catalog = sampleCatalog();
    SignalFilter filter;
    filter.unit = "rad";
    EXPECT_EQ(signalRows(catalog, "rigid_body", filter, {}).size(), 1U);
    filter.unit.clear();
    filter.fastOnly = true;
    EXPECT_TRUE(signalRows(catalog, "sensors", filter, {}).empty());
}

TEST(SignalList, NamesComponents) {
    const SignalCatalog catalog = sampleCatalog();
    EXPECT_EQ(componentLabel(catalog.signals[1], 3), "qw");
    EXPECT_EQ(componentLabel(catalog.signals[2], 42), "[42]");
    EXPECT_EQ(componentPath(catalog.signals[2], 42), "sensor/front_lidar/ranges.42");
    EXPECT_EQ(componentPath(catalog.signals[0], 0), "joint/left_wheel_joint/position");
}

TEST(Fuzzy, MatchesInOrderIgnoringCase) {
    EXPECT_GE(fuzzyScore("left_drive_motor", "ldm"), 0);
    EXPECT_GE(fuzzyScore("left_drive_motor", "DRIVE"), 0);
    EXPECT_LT(fuzzyScore("left_drive_motor", "mdl"), 0);
    EXPECT_EQ(fuzzyScore("anything", ""), 0);
}

TEST(Fuzzy, RanksWordStartsAndSubstringsFirst) {
    const std::vector<std::string_view> texts{"/amr/base_link/visual", "Signals", "sensor/imu/gyro",
                                              "Workspace: Sensors"};
    const std::vector<std::size_t> best = fuzzyRank(texts, "sens", 10);
    ASSERT_EQ(best.size(), 3U);  // "Signals" has no e
    EXPECT_EQ(best[0], 2U);      // at the start
    EXPECT_EQ(best[1], 3U);      // a word
    EXPECT_EQ(best[2], 0U);      // scattered: b-a-S-E, li-N-k, vi-S-ual
    const std::vector<std::size_t> one = fuzzyRank(texts, "s", 1);
    EXPECT_EQ(one.size(), 1U);
}

}  // namespace
}  // namespace lagrange::viewer
