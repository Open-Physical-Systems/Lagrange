#include "lagrange/world/regions/RegionPlanner.hpp"

#include <array>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::regions {
namespace {

using Positions = std::vector<std::array<double, 3>>;

TEST(RegionPlanner, RobotsFarApartGetTheirOwnRegions) {
    const RegionPlanner planner({.radius = 500.0});
    const Positions robots{{0.0, 0.0, 0.0}, {10'000.0, 0.0, 0.0}, {300.0, 400.0, 0.0}};
    const RegionAssignment regions = planner.plan(robots);
    ASSERT_EQ(regions.size(), 2U);
    EXPECT_EQ(regions[0], (std::vector<std::uint32_t>{0, 2})) << "500 m apart: bubbles overlap";
    EXPECT_EQ(regions[1], (std::vector<std::uint32_t>{1}));
}

TEST(RegionPlanner, ChainsOfOverlappingBubblesShareARegion) {
    const RegionPlanner planner({.radius = 100.0});
    const Positions robots{{0.0, 0.0, 0.0}, {190.0, 0.0, 0.0}, {380.0, 0.0, 0.0}};
    EXPECT_EQ(planner.plan(robots).size(), 1U);
}

TEST(RegionPlanner, ARegionSplitsOnlyBeyondTheSplitDistance) {
    const RegionPlanner planner({.radius = 100.0, .splitFactor = 1.25});
    const RegionAssignment together = planner.plan(Positions{{0.0, 0.0, 0.0}, {150.0, 0.0, 0.0}});
    ASSERT_EQ(together.size(), 1U);
    // 230 m: beyond the merge distance (200 m) but within the split distance (250 m).
    EXPECT_EQ(planner.plan(Positions{{0.0, 0.0, 0.0}, {230.0, 0.0, 0.0}}, together).size(), 1U);
    EXPECT_EQ(planner.plan(Positions{{0.0, 0.0, 0.0}, {230.0, 0.0, 0.0}}).size(), 2U)
        << "without history it would be two";
    EXPECT_EQ(planner.plan(Positions{{0.0, 0.0, 0.0}, {260.0, 0.0, 0.0}}, together).size(), 2U);
}

TEST(RegionPlanner, TheAssignmentIsAFunctionOfItsInputs) {
    const RegionPlanner planner({.radius = 50.0});
    const Positions robots{{5.0, 0.0, 0.0}, {800.0, 0.0, 0.0}, {60.0, 0.0, 0.0}, {850.0, 1.0, 0.0}};
    const RegionAssignment a = planner.plan(robots);
    EXPECT_EQ(a, planner.plan(robots));
    ASSERT_EQ(a.size(), 2U);
    EXPECT_EQ(a[0], (std::vector<std::uint32_t>{0, 2}));
    EXPECT_EQ(a[1], (std::vector<std::uint32_t>{1, 3}));
}

}  // namespace
}  // namespace lagrange::regions
