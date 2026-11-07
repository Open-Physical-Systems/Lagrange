#include "lagrange/viewer/FlowLegend.hpp"

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

TEST(FlowLegend, ReadsTheBandsNames) {
    SceneStatics statics;
    MaterialData low;
    low.baseColor = {0.1F, 0.2F, 0.3F, 1.0F};
    MaterialData high;
    high.baseColor = {0.9F, 0.8F, 0.7F, 1.0F};
    statics.materials = {low, high};
    const auto prim = [&statics](const char* name, std::uint32_t material) {
        PrimData p;
        p.name = name;
        p.parent = 0;
        p.material = material;
        statics.prims.push_back(p);
    };
    prim("b03_P1p80_to_P2p40_mps", 0);
    prim("b19_P11p40_to_P12p00_mps", 1);
    prim("b00_M1p00_to_M0p94_cp", 0);
    prim("b31_P0p94_to_P1p00_cp", 1);
    prim("wheel_left", 0);  // not a band
    const auto legends = flowLegends(statics);
    ASSERT_EQ(legends.size(), 2U);
    EXPECT_EQ(legends[0].title, "pressure coefficient");
    EXPECT_DOUBLE_EQ(legends[0].low, -1.0);
    EXPECT_DOUBLE_EQ(legends[0].high, 1.0);
    EXPECT_EQ(legends[1].title, "speed · m/s");
    EXPECT_DOUBLE_EQ(legends[1].low, 1.8);
    EXPECT_DOUBLE_EQ(legends[1].high, 12.0);
    ASSERT_EQ(legends[1].colours.size(), 2U);
    EXPECT_FLOAT_EQ(legends[1].colours[1][0], 0.9F);
}

TEST(FlowLegend, AnOrdinarySceneHasNone) {
    SceneStatics statics;
    EXPECT_TRUE(flowLegends(statics).empty());
}

}  // namespace
}  // namespace lagrange::viewer
