#include "lagrange/viewer/SchematicLayout.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

TEST(SchematicLayout, ComesBackAsSavedPerStage) {
    const auto file = std::filesystem::temp_directory_path() / "lagrange_layout_test.tsv";
    SchematicLayouts layouts;
    layouts["/a b/robot.urdf"]["pack"] = {12.5F, -40.0F};
    layouts["/a b/robot.urdf"]["lights"] = {0.0F, 8.0F};
    layouts["other.usda"]["pack"] = {1.0F, 2.0F};
    ASSERT_TRUE(saveSchematicLayouts(file.string(), layouts));
    const SchematicLayouts back = loadSchematicLayouts(file.string());
    ASSERT_EQ(back.size(), 2U);
    EXPECT_FLOAT_EQ(back.at("/a b/robot.urdf").at("pack").x, 12.5F);
    EXPECT_FLOAT_EQ(back.at("/a b/robot.urdf").at("pack").y, -40.0F);
    EXPECT_FLOAT_EQ(back.at("other.usda").at("pack").y, 2.0F);
    std::filesystem::remove(file);
}

TEST(SchematicLayout, ADamagedFileIsAnEmptyLayout) {
    EXPECT_TRUE(loadSchematicLayouts("/no/such/file").empty());
    const auto file = std::filesystem::temp_directory_path() / "lagrange_layout_bad.tsv";
    std::ofstream(file) << "junk\nstage\tprim\tnot\ta number\nstage\tprim\t1\t2\n";
    const SchematicLayouts back = loadSchematicLayouts(file.string());
    ASSERT_EQ(back.size(), 1U);
    EXPECT_FLOAT_EQ(back.at("stage").at("prim").x, 1.0F);
    std::filesystem::remove(file);
}

}  // namespace
}  // namespace lagrange::viewer
