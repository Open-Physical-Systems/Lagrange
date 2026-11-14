// The builder's URDF writer and reader (RFC 0016 §6, §10; WP-B).
//
// Golden files: viewer/test/builder/golden/. After a deliberate change to the writer, write
// them again with LAGRANGE_UPDATE_GOLDENS=1 and read the diff.
#include "lagrange/viewer/builder/Urdf.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

const std::filesystem::path kTestDir = LAGRANGE_BUILDER_TEST_DIR;
const std::filesystem::path kGoldenDir = kTestDir / "golden";
const std::filesystem::path kAssets = LAGRANGE_BUILDER_ASSETS_DIR;

std::string readFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

Document readOk(const std::string& text) {
    const ReadResult read = readUrdf(text);
    EXPECT_TRUE(read.document.has_value()) << read.error;
    return read.document.value_or(Document{});
}

Link solidLink(const std::string& name, double mass, double size) {
    Link link;
    link.name = name;
    Shape shape;
    shape.kind = Shape::Kind::Box;
    shape.size = {size, size, size};
    shape.rgba = {0.5F, 0.5F, 0.5F, 1};
    link.shapes.push_back(shape);
    const double i = mass * 2 * size * size / 12;
    link.inertial.mass = mass;
    link.inertial.com = std::array<double, 3>{};
    link.inertial.inertia = std::array<double, 6>{i, 0, 0, i, 0, i};
    return link;
}

/// One golden per joint type: a base and one child on that joint (the revolute one also has
/// dynamics and a second joint that mimics it).
Document jointDocument(Joint::Type type) {
    Document d;
    d.robot = "joint_" + std::string(toString(type));
    d.links.push_back(solidLink("base_link", 2, 0.2));
    d.links.push_back(solidLink("child_link", 0.5, 0.1));
    Joint j;
    j.name = "child_joint";
    j.type = type;
    j.parent = "base_link";
    j.child = "child_link";
    j.origin = {{0.15, 0, 0.05}, {0, 0, 0.3}};
    j.axis = {0, 0, 1};
    switch (type) {
        case Joint::Type::Revolute:
            j.lower = -1.5;
            j.upper = 1.5;
            j.effort = 3;
            j.velocity = 2;
            j.damping = 0.01;
            j.friction = 0.02;
            break;
        case Joint::Type::Prismatic:
            j.lower = 0;
            j.upper = 0.1;
            j.effort = 50;
            j.velocity = 0.2;
            break;
        case Joint::Type::Continuous:
            j.effort = 1;
            j.velocity = 10;
            break;
        case Joint::Type::Fixed:
            j.axis = {1, 0, 0};
            break;
    }
    d.joints.push_back(j);
    if (type == Joint::Type::Revolute) {
        d.links.push_back(solidLink("follower_link", 0.2, 0.05));
        Joint follower = j;
        follower.name = "follower_joint";
        follower.child = "follower_link";
        follower.origin = {{-0.15, 0, 0.05}, {0, 0, 0}};
        follower.damping = 0;
        follower.friction = 0;
        follower.mimic = std::tuple<std::string, double, double>{"child_joint", -1, 0.1};
        d.joints.push_back(follower);
    }
    return d;
}

/// An input with every kind of kept block (RFC §10's table) and sensors in both kind forms.
constexpr const char* kKeptInput = R"(<?xml version="1.0"?>
<!-- A robot with blocks the builder keeps as written. -->
<robot name="kept_blocks">
  <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
  <!-- the base -->
  <link name="base_link">
    <visual><geometry><box size="0.3 0.2 0.1"/></geometry><material name="grey"/></visual>
    <collision><geometry><box size="0.3 0.2 0.1"/></geometry></collision>
    <inertial><mass value="2"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.025"/></inertial>
  </link>
  <link name="wheel">
    <visual><origin rpy="1.5707963267948966 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material></visual>
    <collision><origin rpy="1.5707963267948966 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0001"/></inertial>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/><child link="wheel"/>
    <origin xyz="0 0.12 -0.03"/><axis xyz="0 1 0"/>
    <limit effort="2" velocity="20"/>
    <safety_controller k_velocity="10"/>
  </joint>
  <transmission name="wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_joint"><hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface></joint>
  </transmission>
  <gazebo reference="wheel">
    <mu1>0.9</mu1>
    <sensor_material preset="black_rubber"/>
  </gazebo>
  <gazebo reference="base_link">
    <mu1>0.5</mu1>
    <sensor name="front_range" type="rangefinder">
      <pose>0.15 0 0 0 0 0</pose>
      <update_rate>20</update_rate>
      <range><max>8</max></range>
    </sensor>
    <sensor name="rear_range" type="rangefinder">
      <update_rate>20</update_rate>
      <rangefinder><range><max>4</max><min>0.05</min></range></rangefinder>
    </sensor>
    <sensor name="imu" type="imu">
      <update_rate>100</update_rate>
      <latency>0.002</latency>
      <imu><angular_velocity>
        <x><noise type="gaussian"><stddev>0.001</stddev></noise></x>
        <y><noise type="gaussian"><stddev>0.001</stddev></noise></y>
        <z><noise type="gaussian"><stddev>0.001</stddev></noise></z>
      </angular_velocity></imu>
    </sensor>
    <sensor name="lidar" type="gpu_lidar">
      <pose>0 0 0.08 0 0 0</pose>
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <lidar><scan><horizontal><samples>360</samples></horizontal></scan><range><max>12</max></range></lidar>
    </sensor>
  </gazebo>
  <ros2_control name="LagrangeSystem" type="system">
    <hardware><plugin>lagrange_ros2_control/LagrangeSystem</plugin></hardware>
    <joint name="wheel_joint"><command_interface name="velocity"/><state_interface name="position"/></joint>
  </ros2_control>
</robot>
)";

/// The golden documents, by file name.
std::vector<std::pair<std::string, Document>> goldens() {
    std::vector<std::pair<std::string, Document>> out;
    out.emplace_back("rover_mk1.urdf", readOk(readFile(kTestDir / "rover_mk1.urdf")));
    for (const Joint::Type type : {Joint::Type::Fixed, Joint::Type::Revolute,
                                   Joint::Type::Continuous, Joint::Type::Prismatic}) {
        out.emplace_back("joint_" + std::string(toString(type)) + ".urdf", jointDocument(type));
    }
    out.emplace_back("kept_blocks.urdf", readOk(kKeptInput));
    return out;
}

TEST(Urdf, WritesGoldenByteForByte) {
    const bool update = std::getenv("LAGRANGE_UPDATE_GOLDENS") != nullptr;
    for (const auto& [name, document] : goldens()) {
        const std::string text = writeUrdf(document);
        if (update) {
            std::filesystem::create_directories(kGoldenDir);
            ASSERT_EQ(saveAtomically(kGoldenDir / name, text), "");
        }
        EXPECT_EQ(text, readFile(kGoldenDir / name)) << name;
        EXPECT_EQ(writeUrdf(document), text) << name << ": deterministic";
    }
}

TEST(Urdf, ReadsWhatItWrites) {
    for (const auto& [name, document] : goldens()) {
        const std::string golden = readFile(kGoldenDir / name);
        const ReadResult read = readUrdf(golden);
        ASSERT_TRUE(read.document) << name << ": " << read.error;
        EXPECT_EQ(writeUrdf(*read.document), golden) << name;
        EXPECT_EQ(read.document->links.size(), document.links.size()) << name;
        EXPECT_EQ(read.document->joints.size(), document.joints.size()) << name;
        EXPECT_EQ(read.document->sensors.size(), document.sensors.size()) << name;
        for (std::size_t j = 0; j < document.joints.size(); ++j) {
            const Joint& a = document.joints[j];
            const Joint& b = read.document->joints[j];
            EXPECT_EQ(a.type, b.type);
            EXPECT_EQ(a.lower, b.lower);
            EXPECT_EQ(a.upper, b.upper);
            EXPECT_EQ(a.effort, b.effort);
            EXPECT_EQ(a.velocity, b.velocity);
            EXPECT_EQ(a.damping, b.damping);
            EXPECT_EQ(a.friction, b.friction);
            EXPECT_EQ(a.mimic, b.mimic);
            EXPECT_EQ(a.origin.xyz, b.origin.xyz);
            EXPECT_EQ(a.origin.rpy, b.origin.rpy);
        }
    }
}

TEST(Urdf, KeepsUnknownBlocksInPlace) {
    std::string input = kKeptInput;
    // The <lagrange> block (RFC §10's table) is kept too; not in the golden, which must load.
    input.replace(input.find("  <ros2_control"), 0,
                  "  <lagrange><schematic file=\"power.cir\"/></lagrange>\n");
    const ReadResult read = readUrdf(input);
    ASSERT_TRUE(read.document) << read.error;
    const std::string out = writeUrdf(*read.document);
    const auto at = [&out](const std::string& what) {
        const auto pos = out.find(what);
        EXPECT_NE(pos, std::string::npos) << what << " missing in\n" << out;
        return pos;
    };
    // Order as written: prolog comment, materials, comment, base, wheel, joint, transmission,
    // the gazebo block's kept part, its sensors, <lagrange>, ros2_control.
    EXPECT_LT(at("<!-- A robot with blocks"), at("<robot name=\"kept_blocks\">"));
    EXPECT_LT(at("<material name=\"grey\">"), at("<material name=\"black\">"));
    EXPECT_LT(at("<material name=\"black\">"), at("<!-- the base -->"));
    EXPECT_LT(at("<!-- the base -->"), at("<link name=\"base_link\">"));
    EXPECT_LT(at("<link name=\"wheel\">"), at("<joint name=\"wheel_joint\""));
    EXPECT_LT(at("<safety_controller k_velocity=\"10\"/>"), at("</joint>"));
    EXPECT_LT(at("</joint>"), at("<transmission name=\"wheel_transmission\">"));
    EXPECT_LT(at("<transmission"), at("<sensor_material preset=\"black_rubber\"/>"));
    EXPECT_LT(at("<mu1>0.5</mu1>"), at("<sensor name=\"front_range\""));
    EXPECT_LT(at("<sensor name=\"lidar\""), at("<lagrange>"));
    EXPECT_LT(at("<lagrange>"), at("<ros2_control"));
    // The kept text has markers; the display form does not.
    EXPECT_NE(read.document->keptXml.find("<!--lagrange-builder:link-->"), std::string::npos);
    const std::string shown = keptForDisplay(read.document->keptXml);
    EXPECT_EQ(shown.find("lagrange-builder"), std::string::npos);
    EXPECT_NE(shown.find("<transmission"), std::string::npos);
    EXPECT_NE(findJoint(*read.document, "wheel_joint")->keptXml.find("safety_controller"),
              std::string::npos);
    // Read again, written again: the same bytes.
    EXPECT_EQ(writeUrdf(readOk(out)), out);
}

TEST(Urdf, InertialFromFileBecomesOverride) {
    const ReadResult read = readUrdf(readFile(kTestDir / "rover_mk1.urdf"));
    ASSERT_TRUE(read.document);
    const Link* mast = findLink(*read.document, "mast");
    ASSERT_NE(mast, nullptr);
    EXPECT_EQ(mast->inertial.density, 0.0);
    ASSERT_TRUE(mast->inertial.mass);
    EXPECT_DOUBLE_EQ(*mast->inertial.mass, 0.6785840131753954);
    ASSERT_TRUE(mast->inertial.com);
    EXPECT_EQ(*mast->inertial.com, (std::array<double, 3>{0, 0, 0.1}));
    ASSERT_TRUE(mast->inertial.inertia);
    EXPECT_DOUBLE_EQ((*mast->inertial.inertia)[5], 0.0001357168026350791);
    bool noted = false;
    for (const std::string& note : read.notes) {
        noted = noted || note.find("override") != std::string::npos;
    }
    EXPECT_TRUE(noted);
    // Shapes: one visual+collision box with the body colour; solid as a collision shape is.
    const Link* base = findLink(*read.document, "base_link");
    ASSERT_EQ(base->shapes.size(), 1U);
    EXPECT_TRUE(base->shapes[0].visual && base->shapes[0].collision && base->shapes[0].solid);
    EXPECT_EQ(base->shapes[0].rgba, (std::array<float, 4>{0.66F, 0.72F, 0.81F, 1}));
}

TEST(Urdf, DensityRoundTripsAndComputedPartsAreComputedAgain) {
    Document d;
    d.robot = "dense";
    Link link;
    link.name = "base_link";
    link.inertial.density = 2700;
    Shape s;
    s.kind = Shape::Kind::Box;
    s.size = {0.1, 0.2, 0.3};
    link.shapes.push_back(s);
    Shape decoration = s;
    decoration.collision = false;
    decoration.solid = true;  // not the default for a visual-only shape
    decoration.origin.xyz = {0, 0, 0.5};
    link.shapes.push_back(decoration);
    d.links.push_back(link);
    const std::string text = writeUrdf(d);
    EXPECT_NE(text.find("lagrange_density=\"2700\""), std::string::npos) << text;
    EXPECT_NE(text.find("lagrange_overrides=\"\""), std::string::npos) << text;
    EXPECT_NE(text.find("lagrange_solid=\"true\""), std::string::npos) << text;
    // Two boxes of 2700 × 0.006 = 16.2 kg each (the closed form of RFC §4).
    EXPECT_NE(text.find("<mass value=\"32.4"), std::string::npos) << text;
    const Document back = readOk(text);
    const Link& read = back.links.at(0);
    EXPECT_EQ(read.inertial.density, 2700);
    EXPECT_FALSE(read.inertial.mass);
    EXPECT_FALSE(read.inertial.com);
    EXPECT_FALSE(read.inertial.inertia);
    ASSERT_EQ(read.shapes.size(), 2U);
    EXPECT_TRUE(read.shapes[1].solid);
    EXPECT_FALSE(read.shapes[1].collision);
    EXPECT_EQ(writeUrdf(back), text);

    d.links[0].inertial.mass = 10;  // an override: only it is listed
    const std::string overridden = writeUrdf(d);
    EXPECT_NE(overridden.find("lagrange_overrides=\"mass\""), std::string::npos);
    const Document again = readOk(overridden);
    EXPECT_EQ(again.links[0].inertial.mass, 10);
    EXPECT_FALSE(again.links[0].inertial.inertia);
}

TEST(Urdf, MalformedXmlGivesLine) {
    const ReadResult read = readUrdf("<robot name=\"r\">\n  <link name=\"a\">\n</robot>\n");
    EXPECT_FALSE(read.document);
    EXPECT_GT(read.errorLine, 0);
    EXPECT_NE(read.error.find("line"), std::string::npos);
    EXPECT_FALSE(readUrdf("<sdf version=\"1.9\"/>").document);
    const ReadResult xacro =
        readUrdf("<robot name=\"r\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\"/>");
    EXPECT_FALSE(xacro.document);
    EXPECT_NE(xacro.error.find("xacro"), std::string::npos);
}

TEST(Urdf, KindWrapperAcceptedBothWays) {
    const Document d = readOk(kKeptInput);
    const Sensor* front = findSensor(d, "front_range");
    const Sensor* rear = findSensor(d, "rear_range");
    ASSERT_NE(front, nullptr);
    ASSERT_NE(rear, nullptr);
    EXPECT_EQ(front->params.at("range/max"), "8");  // <range><max>, no wrapper
    EXPECT_EQ(rear->params.at("range/max"), "4");   // <rangefinder><range><max>
    EXPECT_EQ(rear->params.at("range/min"), "0.05");
    EXPECT_EQ(front->link, "base_link");
    EXPECT_EQ(front->updateRate, 20);
    EXPECT_EQ(front->origin.xyz, (std::array<double, 3>{0.15, 0, 0}));
    const std::string text = writeUrdf(d);
    // Both written with the wrapper (plan decision 6).
    EXPECT_NE(text.find("<rangefinder>\n        <range>\n          <max>8</max>"),
              std::string::npos)
        << text;
    // A built-in type keeps its element in the path; the IMU's equal axes are "{axis}".
    const Sensor* lidar = findSensor(d, "lidar");
    EXPECT_EQ(lidar->params.at("lidar/range/max"), "12");
    EXPECT_EQ(lidar->params.at("topic"), "scan");
    const Sensor* imu = findSensor(d, "imu");
    EXPECT_EQ(imu->params.at("imu/angular_velocity/{axis}/noise/stddev"), "0.001");
    EXPECT_EQ(imu->params.at("imu/angular_velocity/{axis}/noise/@type"), "gaussian");
    EXPECT_EQ(imu->latency, 0.002);
    EXPECT_NE(text.find("<z>\n            <noise type=\"gaussian\">"), std::string::npos) << text;
}

TEST(Urdf, MeshPathsRelativeOnSaveAbsoluteForTest) {
    Document d = newDocument("meshes");
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "meshes/part.stl";
    mesh.scale = {0.001, 0.001, 0.001};
    d.links[0].shapes.push_back(mesh);
    mesh.mesh = "/data/robots/meshes/other.stl";
    d.links[0].shapes.push_back(mesh);
    mesh.mesh = "package://my_robot/meshes/base.dae";
    d.links[0].shapes.push_back(mesh);
    mesh.mesh = "lagrange-asset://x500/arm.stl";
    d.links[0].shapes.push_back(mesh);

    const std::string plain = writeUrdf(d);
    EXPECT_NE(plain.find("filename=\"meshes/part.stl\" scale=\"0.001 0.001 0.001\""),
              std::string::npos);
    EXPECT_NE(plain.find("filename=\"/data/robots/meshes/other.stl\""), std::string::npos);

    const std::string saved = writeUrdf(d, {"/data/robots/r.urdf", false});
    EXPECT_NE(saved.find("filename=\"meshes/other.stl\""), std::string::npos) << saved;
    EXPECT_NE(saved.find("filename=\"meshes/part.stl\""), std::string::npos);

    const std::string test = writeUrdf(d, {"/data/robots/r.urdf", true});
    EXPECT_NE(test.find("filename=\"/data/robots/meshes/part.stl\""), std::string::npos) << test;
    EXPECT_NE(test.find("filename=\"package://my_robot/meshes/base.dae\""), std::string::npos);
    EXPECT_NE(test.find("filename=\"lagrange-asset://x500/arm.stl\""), std::string::npos);

    const Document back = readOk(plain);
    ASSERT_EQ(back.links[0].shapes.size(), 5U);
    EXPECT_EQ(back.links[0].shapes[1].mesh, "meshes/part.stl");
    EXPECT_EQ(back.links[0].shapes[1].scale, (std::array<double, 3>{0.001, 0.001, 0.001}));
}

TEST(Urdf, SaveIsAtomic) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("builder_save_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const std::filesystem::path file = dir / "r.urdf";
    EXPECT_EQ(saveAtomically(file, "first"), "");
    EXPECT_EQ(saveAtomically(file, "second"), "");
    EXPECT_EQ(readFile(file), "second");
    std::size_t entries = 0;
    for ([[maybe_unused]] const auto& e : std::filesystem::directory_iterator(dir)) {
        ++entries;
    }
    EXPECT_EQ(entries, 1U) << "no temporary file left";
    const std::string xacro = saveAtomically(dir / "r.urdf.xacro", "x");
    EXPECT_NE(xacro.find("xacro"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(dir / "r.urdf.xacro"));
    const std::string missing = saveAtomically(dir / "no" / "such" / "r.urdf", "x");
    EXPECT_NE(missing.find("r.urdf"), std::string::npos);
    std::filesystem::remove_all(dir);
}

TEST(Urdf, EverySampleAndTemplateReadsAndConverges) {
    // Semantic equality with the file is test_builder_roundtrip.py's; here: every sample reads,
    // and writing what was read, read again, gives the same bytes.
    int files = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(kAssets)) {
        if (entry.path().extension() != ".urdf") {
            continue;
        }
        ++files;
        const ReadResult read = readUrdf(readFile(entry.path()));
        ASSERT_TRUE(read.document) << entry.path() << ": " << read.error;
        const std::string once = writeUrdf(*read.document);
        const ReadResult again = readUrdf(once);
        ASSERT_TRUE(again.document) << entry.path() << ": " << again.error;
        EXPECT_EQ(writeUrdf(*again.document), once) << entry.path();
        EXPECT_EQ(again.document->links.size(), read.document->links.size());
        EXPECT_EQ(again.document->joints.size(), read.document->joints.size());
        EXPECT_EQ(again.document->sensors.size(), read.document->sensors.size());
    }
    EXPECT_GE(files, 14) << "11 samples and 3 templates";
}

}  // namespace
}  // namespace lagrange::viewer::builder
