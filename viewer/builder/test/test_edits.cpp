// The Builder panels' edits and field text (WP-E): pure functions, tested without ImGui.
#include "lagrange/viewer/builder/Edits.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

Document rover() {
    Document document = newDocument("rover");
    const AddedLink mast =
        addLink(document, "base_link", Shape::Kind::Cylinder, Joint::Type::Fixed, 0.0);
    const AddedLink head =
        addLink(document, mast.link, Shape::Kind::Box, Joint::Type::Revolute, 1240.0);
    (void)addSensor(document, head.link, "camera");
    (void)addSensor(document, "base_link", "rangefinder");
    return document;
}

TEST(Edits, AddLinkSitsOnTopOfItsParentWithNoInventedLimits) {
    Document document = newDocument();
    const AddedLink added =
        addLink(document, "base_link", Shape::Kind::Box, Joint::Type::Revolute, 1240.0);
    EXPECT_EQ(added.link, "box_link");
    EXPECT_EQ(added.joint, "box_link_joint");
    const Joint* joint = findJoint(document, added.joint);
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->parent, "base_link");
    EXPECT_EQ(joint->child, "box_link");
    EXPECT_EQ(joint->type, Joint::Type::Revolute);
    // Base 0.1 m high (top at 0.05), new box 0.1 m (half 0.05): origin 0.1 above.
    EXPECT_DOUBLE_EQ(joint->origin.xyz[2], 0.1);
    EXPECT_FALSE(joint->lower.has_value());
    EXPECT_FALSE(joint->upper.has_value());
    EXPECT_FALSE(joint->effort.has_value());
    EXPECT_DOUBLE_EQ(findLink(document, "box_link")->inertial.density, 1240.0);

    const AddedLink second =
        addLink(document, "base_link", Shape::Kind::Box, Joint::Type::Fixed, 0.0);
    EXPECT_EQ(second.link, "box_link_2");
    EXPECT_TRUE(addLink(document, "nope", Shape::Kind::Box, Joint::Type::Fixed, 0.0).link.empty());
}

TEST(Edits, TreeNestsLinksByJointsWithSensorsAfterChildLinks) {
    const Document document = rover();
    const std::vector<TreeItem> items = treeItems(document);
    ASSERT_EQ(items.size(), 6U);
    EXPECT_EQ(items[0].kind, TreeItem::Kind::Robot);
    EXPECT_EQ(items[1].name, "base_link");
    EXPECT_EQ(items[1].depth, 1);
    EXPECT_EQ(items[2].name, "cylinder_link");
    EXPECT_EQ(items[2].joint, "cylinder_link_joint");
    EXPECT_EQ(items[3].name, "box_link");
    EXPECT_EQ(items[3].depth, 3);
    EXPECT_EQ(items[4].name, "camera");
    EXPECT_EQ(items[4].depth, 4);
    EXPECT_EQ(items[5].name, "rangefinder");
    EXPECT_EQ(items[5].depth, 2);
}

TEST(Edits, TreeShowsEveryItemOfABrokenDocument) {
    Document document = rover();
    document.joints[0].parent = "box_link";  // a loop: cylinder -> box -> cylinder
    document.sensors.push_back(Sensor{"lost", "missing_link", "imu", {}, 10, {}, {}, {}});
    const std::vector<TreeItem> items = treeItems(document);
    int links = 0;
    int sensors = 0;
    for (const TreeItem& item : items) {
        links += item.kind == TreeItem::Kind::Link ? 1 : 0;
        sensors += item.kind == TreeItem::Kind::Sensor ? 1 : 0;
    }
    EXPECT_EQ(links, 3);
    EXPECT_EQ(sensors, 3);
}

TEST(Edits, CountsSplitTypesFromKinds) {
    const PartCounts counts = countParts(rover(), nullptr);
    EXPECT_EQ(counts.links, 3);
    EXPECT_EQ(counts.roots, 1);
    EXPECT_EQ(counts.joints, 2);
    EXPECT_EQ(counts.revolute, 1);
    EXPECT_EQ(counts.fixed, 1);
    EXPECT_EQ(counts.sensors, 2);
    EXPECT_EQ(counts.types, 1);
    EXPECT_EQ(counts.kinds, 1);
}

TEST(Edits, RemovingALinkTakesItsSubtreeAndSensors) {
    Document document = rover();
    EXPECT_TRUE(removeLink(document, "cylinder_link"));
    EXPECT_EQ(document.links.size(), 1U);
    EXPECT_TRUE(document.joints.empty());
    ASSERT_EQ(document.sensors.size(), 1U);
    EXPECT_EQ(document.sensors[0].name, "rangefinder");

    Document again = rover();
    EXPECT_TRUE(removeJoint(again, "box_link_joint"));
    EXPECT_EQ(again.links.size(), 2U);
    EXPECT_EQ(findLink(again, "box_link"), nullptr);
    EXPECT_TRUE(removeSensor(again, "rangefinder"));
    EXPECT_TRUE(again.sensors.empty());
}

TEST(Edits, RenameFollowsEveryReference) {
    Document document = rover();
    EXPECT_EQ(rename(document, ItemKind::Link, "box_link", "head"), "");
    EXPECT_EQ(findJoint(document, "box_link_joint")->child, "head");
    EXPECT_EQ(findSensor(document, "camera")->link, "head");
    EXPECT_NE(rename(document, ItemKind::Link, "head", "base_link"), "");
    EXPECT_NE(rename(document, ItemKind::Link, "head", "2head"), "");
    document.joints[1].mimic = std::make_tuple(std::string("cylinder_link_joint"), 1.0, 0.0);
    EXPECT_EQ(rename(document, ItemKind::Joint, "cylinder_link_joint", "mast_joint"), "");
    EXPECT_EQ(std::get<0>(*document.joints[1].mimic), "mast_joint");
}

TEST(Edits, ReparentRefusesItsOwnSubtreeAndKeepsThePlace) {
    Document document = rover();
    EXPECT_NE(reparent(document, "cylinder_link", "box_link", {}), "");
    EXPECT_NE(reparent(document, "base_link", "box_link", {}), "");
    // box_link at (0, 0, 0.3) in the root, rotated 90° about z; cylinder_link at (0, 0, 0.1).
    std::vector<Frame> frames(3);
    frames[1].position = {0.0, 0.0, 0.1};
    frames[2].position = {1.0, 0.0, 0.3};
    frames[2].rotation = {std::cos(std::numbers::pi / 4), 0, 0, std::sin(std::numbers::pi / 4)};
    EXPECT_EQ(reparent(document, "box_link", "base_link", frames), "");
    const Joint* joint = findJoint(document, "box_link_joint");
    EXPECT_EQ(joint->parent, "base_link");
    EXPECT_NEAR(joint->origin.xyz[0], 1.0, 1e-12);
    EXPECT_NEAR(joint->origin.xyz[2], 0.3, 1e-12);
    EXPECT_NEAR(joint->origin.rpy[2], std::numbers::pi / 2, 1e-12);
    EXPECT_EQ(joint->type, Joint::Type::Revolute);
}

TEST(Edits, FieldText) {
    EXPECT_EQ(formatNumber(0.25), "0.25");
    EXPECT_EQ(formatNumber(-0.0), "0");
    EXPECT_EQ(formatNumber(1240.0), "1240");
    EXPECT_EQ(parseNumber(" 20 "), 20.0);
    EXPECT_EQ(parseNumber("\xE2\x88\x92"
                          "0.03"),
              -0.03);
    EXPECT_FALSE(parseNumber("1,5").has_value());
    EXPECT_FALSE(parseNumber("").has_value());
    EXPECT_FALSE(parseNumber("inf").has_value());
    EXPECT_NEAR(toRadians(toDegrees(0.5)), 0.5, 1e-15);
}

TEST(Edits, ParamsCheckedAgainstTheCatalog) {
    CatalogParam range;
    range.path = "range/max";
    range.unit = "m";
    range.min = 0.1;
    range.max = 40.0;
    EXPECT_EQ(checkParam(range, "20"), "");
    EXPECT_EQ(checkParam(range, ""), "");
    EXPECT_EQ(checkParam(range, "41"), "at most 40 m");
    EXPECT_EQ(checkParam(range, "x"), "not a number");
    CatalogParam count;
    count.type = CatalogParam::Type::Int;
    EXPECT_EQ(checkParam(count, "2.5"), "a whole number");
    CatalogParam mode;
    mode.type = CatalogParam::Type::Enum;
    mode.options = {"a", "b"};
    EXPECT_EQ(checkParam(mode, "b"), "");
    EXPECT_NE(checkParam(mode, "c"), "");
}

}  // namespace
}  // namespace lagrange::viewer::builder
