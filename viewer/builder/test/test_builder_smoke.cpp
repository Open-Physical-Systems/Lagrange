// Smoke tests of the robot builder's interfaces (RFC 0016 WP0): every header compiles, links,
// and the stubs say plainly that they are not implemented. The work packages replace these
// expectations with their own tests (docs/plan/builder-phase1-plan.md §2).
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/History.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/Materials.hpp"
#include "lagrange/viewer/builder/Primitives.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/TestServer.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <variant>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

TEST(BuilderDocument, NewDocumentHasABaseLinkBox) {
    const Document document = newDocument("rover");
    EXPECT_EQ(document.robot, "rover");
    ASSERT_EQ(document.links.size(), 1U);
    EXPECT_EQ(document.links.front().name, kNewRootLink);
    ASSERT_EQ(document.links.front().shapes.size(), 1U);
    const Shape& box = document.links.front().shapes.front();
    EXPECT_EQ(box.kind, Shape::Kind::Box);
    EXPECT_EQ(box.size, kNewRootBox);
    EXPECT_TRUE(box.visual && box.collision && box.solid);
    EXPECT_TRUE(document.joints.empty());
    EXPECT_TRUE(document.sensors.empty());
}

TEST(BuilderDocument, FindersAndNames) {
    Document document = newDocument();
    Joint joint;
    joint.name = "pan_joint";
    joint.type = Joint::Type::Revolute;
    document.joints.push_back(joint);
    EXPECT_NE(findLink(document, "base_link"), nullptr);
    EXPECT_EQ(findLink(document, "nope"), nullptr);
    ASSERT_NE(findJoint(document, "pan_joint"), nullptr);
    findJoint(document, "pan_joint")->lower = -1.0;
    EXPECT_EQ(document.joints.front().lower, -1.0);
    EXPECT_EQ(findSensor(document, "imu"), nullptr);
    for (const Joint::Type type : {Joint::Type::Fixed, Joint::Type::Revolute,
                                   Joint::Type::Continuous, Joint::Type::Prismatic}) {
        EXPECT_EQ(parseJointType(toString(type)), type);
    }
    EXPECT_FALSE(parseJointType("floating").has_value());
    EXPECT_EQ(toString(Shape::Kind::Cylinder), "cylinder");
    EXPECT_FALSE(isMoving(Joint::Type::Fixed));
    EXPECT_TRUE(isMoving(Joint::Type::Continuous));
}

TEST(BuilderValidation, ValidNameIsTheServersRule) {
    EXPECT_TRUE(validName("base_link"));
    EXPECT_TRUE(validName("_x1"));
    EXPECT_FALSE(validName(""));
    EXPECT_FALSE(validName("1st"));
    EXPECT_FALSE(validName("front-range"));
    EXPECT_FALSE(validName("rover mk1"));
    EXPECT_EQ(toString(Level::Hint), "hint");
}

TEST(BuilderKinematics, ToFrameAndComposeFollowUrdf) {
    // Yaw by 90 degrees turns +x into +y.
    Pose pose;
    pose.xyz = {1.0, 0.0, 0.0};
    pose.rpy = {0.0, 0.0, std::numbers::pi / 2.0};
    const Frame yaw = toFrame(pose);
    Frame unitX;
    unitX.position = {1.0, 0.0, 0.0};
    const Frame moved = compose(yaw, unitX);
    EXPECT_NEAR(moved.position[0], 1.0, 1e-12);
    EXPECT_NEAR(moved.position[1], 1.0, 1e-12);
    EXPECT_NEAR(moved.position[2], 0.0, 1e-12);
    // Roll then pitch then yaw about fixed axes: roll 90 then pitch 90 sends +y to +x? No:
    // R = Rz Ry Rx; with roll = pitch = 90 degrees, R·(0, 1, 0) = (1, 0, 0).
    Pose rp;
    rp.rpy = {std::numbers::pi / 2.0, std::numbers::pi / 2.0, 0.0};
    Frame unitY;
    unitY.position = {0.0, 1.0, 0.0};
    const Frame turned = compose(toFrame(rp), unitY);
    EXPECT_NEAR(turned.position[0], 1.0, 1e-12);
    EXPECT_NEAR(turned.position[1], 0.0, 1e-12);
    EXPECT_NEAR(turned.position[2], 0.0, 1e-12);
}

TEST(BuilderHistory, CommitChangesCurrentAndRevision) {
    History history(newDocument("a"), "New robot");
    EXPECT_EQ(history.labels().size(), 1U);
    EXPECT_EQ(history.labels().front(), "New robot");
    const auto revision = history.revision();
    history.commit(newDocument("b"), "Rename robot", "robot/name", 0.0);
    EXPECT_EQ(history.current().robot, "b");
    EXPECT_NE(history.revision(), revision);
    EXPECT_GT(history.editsSinceSave(), 0);
    history.markSaved();
    EXPECT_EQ(history.editsSinceSave(), 0);
}

TEST(BuilderStubs, SayTheyAreNotImplemented) {
    // WP-B
    const ReadResult read = readUrdf("<robot name=\"r\"/>");
    if (!read.document) {
        EXPECT_FALSE(read.error.empty());
    }
    EXPECT_NE(writeUrdf(newDocument("r")).find("<robot"), std::string::npos);
    // WP-F
    TestServer server;
    EXPECT_EQ(server.state(), TestServer::State::Idle);
    if (const std::string why = server.start({}); !why.empty()) {
        EXPECT_EQ(server.poll(), TestServer::State::Idle);
    }
    // WP-C: no files is an empty catalog, never an error.
    const auto catalog = SensorCatalog::load({});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog>(catalog));
    EXPECT_TRUE(std::get<SensorCatalog>(catalog).entries().empty());
    EXPECT_EQ(std::get<SensorCatalog>(catalog).find("rangefinder"), nullptr);
}

TEST(BuilderSceneAdapter, PoseKeepsTheStatics) {
    const Document document = newDocument();
    const BuiltScene scene = buildStatics(document, 7);
    ASSERT_NE(scene.statics, nullptr);
    EXPECT_EQ(scene.statics->generation, 7U);
    const auto frame = poseScene(scene, linkFrames(document, buildTree(document), {}));
    ASSERT_NE(frame, nullptr);
    EXPECT_EQ(frame->statics, scene.statics);  // plan decision 2
    EXPECT_EQ(frame->transforms.size(), scene.statics->prims.size());
    EXPECT_EQ(linkFrames(document, buildTree(document), {}).size(), document.links.size());
}

TEST(BuilderSample, RoverMk1IsThere) {
    std::ifstream file(LAGRANGE_BUILDER_TEST_DIR "/rover_mk1.urdf");
    ASSERT_TRUE(file.good());
    std::stringstream text;
    text << file.rdbuf();
    EXPECT_NE(text.str().find("<robot name=\"rover_mk1\">"), std::string::npos);
    EXPECT_NE(text.str().find("type=\"rangefinder\""), std::string::npos);
}

}  // namespace
}  // namespace lagrange::viewer::builder
