// The builder's own validation (RFC 0016 §8.1, plan WP-A): one test per rule.
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

/// A valid two-link robot: a box base and an arm on a revolute joint with every limit, one
/// IMU on the base.
Document validRobot() {
    Document document = newDocument("arm_bot");
    document.links.front().inertial.density = 1000.0;
    Link arm;
    arm.name = "arm";
    Shape rod;
    rod.kind = Shape::Kind::Cylinder;
    rod.size = {0.02, 0.3, 0.0};
    arm.shapes = {rod};
    arm.inertial.density = 2700.0;
    document.links.push_back(arm);
    Joint joint;
    joint.name = "shoulder";
    joint.type = Joint::Type::Revolute;
    joint.parent = "base_link";
    joint.child = "arm";
    joint.axis = {0, 1, 0};
    joint.lower = -1.0;
    joint.upper = 1.0;
    joint.effort = 5.0;
    joint.velocity = 2.0;
    document.joints.push_back(joint);
    Sensor imu;
    imu.name = "imu";
    imu.link = "base_link";
    imu.type = "imu";
    document.sensors.push_back(imu);
    return document;
}

/// The problems about `item` whose message contains `text`.
std::size_t count(const std::vector<Problem>& problems, Level level, std::string_view item,
                  std::string_view text) {
    return static_cast<std::size_t>(std::ranges::count_if(problems, [&](const Problem& p) {
        return p.level == level && p.item == item && p.message.find(text) != std::string::npos;
    }));
}

std::string dump(const std::vector<Problem>& problems) {
    std::string out;
    for (const Problem& p : problems) {
        out += std::string(toString(p.level)) + " [" + p.item + "] " + p.message + "\n";
    }
    return out;
}

/// Sets an environment variable for the scope of a test, and puts the old value back.
class ScopedEnv {
public:
    ScopedEnv(const char* name, const std::string& value) : name_(name) {
        if (const char* old = std::getenv(name); old != nullptr) {
            old_ = old;
        }
        ::setenv(name, value.c_str(), 1);
    }
    ~ScopedEnv() {
        if (old_) {
            ::setenv(name_, old_->c_str(), 1);
        } else {
            ::unsetenv(name_);
        }
    }
    ScopedEnv(const ScopedEnv&) = delete;
    ScopedEnv& operator=(const ScopedEnv&) = delete;

private:
    const char* name_;
    std::optional<std::string> old_;
};

std::filesystem::path scratch(const std::string& name) {
    const std::filesystem::path dir =
        std::filesystem::path(testing::TempDir()) / ("lagrange_builder_validation_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

void touch(const std::filesystem::path& file) {
    std::filesystem::create_directories(file.parent_path());
    if (file.extension() == ".dae") {
        // A mesh is read once found (Addendum A): a real file, a small sample one.
        std::filesystem::copy_file(std::filesystem::path(LAGRANGE_BUILDER_ASSETS_DIR) / "amr_fms" /
                                       "meshes" / "battery.dae",
                                   file, std::filesystem::copy_options::overwrite_existing);
        return;
    }
    if (file.extension() == ".stl") {
        // A tetrahedron, closed and convex: found, read, and nothing to say about it.
        std::ofstream(file) << "solid t\n"
                               "facet normal 0 0 -1\nouter loop\nvertex 0 0 0\nvertex 0 1 0\n"
                               "vertex 1 0 0\nendloop\nendfacet\n"
                               "facet normal 0 -1 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\n"
                               "vertex 0 0 1\nendloop\nendfacet\n"
                               "facet normal -1 0 0\nouter loop\nvertex 0 0 0\nvertex 0 0 1\n"
                               "vertex 0 1 0\nendloop\nendfacet\n"
                               "facet normal 1 1 1\nouter loop\nvertex 1 0 0\nvertex 0 1 0\n"
                               "vertex 0 0 1\nendloop\nendfacet\nendsolid t\n";
        return;
    }
    std::ofstream(file) << "solid empty\nendsolid empty\n";
}

TEST(Validation, ValidRobotHasNoProblems) {
    const auto problems = validate(validRobot(), nullptr, {});
    EXPECT_TRUE(problems.empty()) << dump(problems);
}

TEST(Validation, ServerNameRule) {
    Document document = validRobot();
    document.robot = "arm bot";
    document.links[1].name = "2nd_arm";
    document.joints[0].child = "2nd_arm";
    document.joints[0].name = "shoulder-joint";
    document.sensors[0].name = "";
    const auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "", "robot name 'arm bot' is not valid"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "2nd_arm", "not valid"), 1U) << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "shoulder-joint", "not valid"), 1U) << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "", "sensor name '' is not valid"), 1U)
        << dump(problems);
    // The server's rule exactly (server/src/Server.cpp validName).
    EXPECT_TRUE(validName("_a1"));
    EXPECT_FALSE(validName("a.b"));
    EXPECT_FALSE(validName("é"));
}

TEST(Validation, NamesUnique) {
    Document document = validRobot();
    document.sensors.push_back(document.sensors[0]);
    Joint extra = document.joints[0];
    extra.child = "base_link";  // also a second parent problem; the name rule is what is tested
    document.joints.push_back(extra);
    const auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "imu", "two sensors are named 'imu'"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "two joints are named 'shoulder'"), 1U)
        << dump(problems);
    Document links = validRobot();
    links.links.push_back(links.links[1]);
    EXPECT_EQ(count(validate(links, nullptr, {}), Level::Error, "arm", "two links"), 1U);
}

TEST(Validation, OneRootNoLoopsJointsNameLinks) {
    Document twoRoots = validRobot();
    twoRoots.joints.clear();
    auto problems = validate(twoRoots, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "arm", "one root"), 1U) << dump(problems);

    Document missing = validRobot();
    missing.joints[0].parent = "chassis";
    problems = validate(missing, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "parent link 'chassis'"), 1U)
        << dump(problems);
    missing.joints[0].parent = "base_link";
    missing.joints[0].child = "forearm";
    problems = validate(missing, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "child link 'forearm'"), 1U)
        << dump(problems);

    Document loop = validRobot();
    Link third;
    third.name = "hand";
    third.inertial.mass = 0.1;
    third.inertial.inertia = std::array<double, 6>{1e-4, 0, 0, 1e-4, 0, 1e-4};
    loop.links.push_back(third);
    Joint toHand = loop.joints[0];
    toHand.name = "wrist";
    toHand.parent = "arm";
    toHand.child = "hand";
    Joint back = loop.joints[0];
    back.name = "back";
    back.parent = "hand";
    back.child = "arm";
    loop.joints = {toHand, back};  // arm ↔ hand, base_link alone
    problems = validate(loop, nullptr, {});
    EXPECT_TRUE(std::ranges::any_of(problems, [](const Problem& p) {
        return p.level == Level::Error && p.message.find("loop") != std::string::npos;
    })) << dump(problems);

    Document self = validRobot();
    self.joints[0].child = "base_link";
    problems = validate(self, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "to itself"), 1U) << dump(problems);

    Document none = validRobot();
    none.links.clear();
    none.joints.clear();
    none.sensors.clear();
    EXPECT_EQ(count(validate(none, nullptr, {}), Level::Error, "", "no link"), 1U);
}

TEST(Validation, EmptyLimitIsAnError) {
    for (const Joint::Type type : {Joint::Type::Revolute, Joint::Type::Prismatic}) {
        Document document = validRobot();
        Joint& joint = document.joints[0];
        joint.type = type;
        joint.lower.reset();
        joint.upper.reset();
        joint.effort.reset();
        joint.velocity.reset();
        const auto problems = validate(document, nullptr, {});
        for (const char* field :
             {"no lower limit", "no upper limit", "no effort limit", "no velocity limit"}) {
            EXPECT_EQ(count(problems, Level::Error, "shoulder", field), 1U) << field << "\n"
                                                                            << dump(problems);
        }
    }
    // Continuous joints need none.
    Document wheel = validRobot();
    wheel.joints[0].type = Joint::Type::Continuous;
    wheel.joints[0].lower.reset();
    wheel.joints[0].upper.reset();
    wheel.joints[0].effort.reset();
    wheel.joints[0].velocity.reset();
    EXPECT_TRUE(validate(wheel, nullptr, {}).empty()) << dump(validate(wheel, nullptr, {}));
}

TEST(Validation, LowerBelowUpperEffortAndVelocityPositive) {
    Document document = validRobot();
    document.joints[0].lower = 1.0;
    document.joints[0].upper = 1.0;
    document.joints[0].effort = 0.0;
    document.joints[0].velocity = -2.0;
    const auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "not below the upper limit"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "effort 0 must be greater than 0"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "shoulder", "velocity -2 must be greater"), 1U)
        << dump(problems);
}

TEST(Validation, NonZeroAxis) {
    Document document = validRobot();
    document.joints[0].axis = {0, 0, 0};
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "shoulder", "zero axis"), 1U);
    document.joints[0].type = Joint::Type::Fixed;  // a fixed joint's axis is not used
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "shoulder", "zero axis"), 0U);
}

TEST(Validation, PositiveMassAndValidInertia) {
    Document document = validRobot();
    document.links[1].inertial.density = 0.0;
    auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "arm", "has no mass: give it a density"), 1U)
        << dump(problems);
    document.links[1].inertial.mass = -1.0;
    problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "arm", "must be greater than 0"), 1U) << dump(problems);
    document.links[1].inertial.mass = 1.0;
    document.links[1].inertial.inertia = std::array<double, 6>{1, 0, 0, 1, 0, 3};
    problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "arm", "triangle inequality"), 1U) << dump(problems);
    document.links[1].inertial.inertia = std::array<double, 6>{1, 0, 0, 1, 0, 0};
    problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "arm", "not positive definite"), 1U) << dump(problems);
    // A massless link on a fixed joint (a sensor frame) is only a warning.
    Document frame = validRobot();
    Link mount;
    mount.name = "camera_mount";
    frame.links.push_back(mount);
    Joint fixed;
    fixed.name = "camera_mount_joint";
    fixed.parent = "base_link";
    fixed.child = "camera_mount";
    frame.joints.push_back(fixed);
    problems = validate(frame, nullptr, {});
    EXPECT_EQ(count(problems, Level::Warning, "camera_mount", "no shape is solid"), 1U)
        << dump(problems);
    // A shape of zero size.
    Document flat = validRobot();
    flat.links[0].shapes[0].size = {0.3, 0.0, 0.1};
    problems = validate(flat, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "base_link", "zero or less"), 1U) << dump(problems);
}

TEST(Validation, MimicLeaderExists) {
    Document document = validRobot();
    Link finger;
    finger.name = "finger";
    finger.shapes = {document.links[1].shapes[0]};
    finger.inertial.density = 1000.0;
    document.links.push_back(finger);
    Joint copy = document.joints[0];
    copy.name = "finger_joint";
    copy.parent = "arm";
    copy.child = "finger";
    copy.mimic = std::make_tuple(std::string("elbow"), 1.0, 0.0);
    document.joints.push_back(copy);
    auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "finger_joint", "mimics 'elbow', which does not"), 1U)
        << dump(problems);
    std::get<0>(*document.joints[1].mimic) = "shoulder";
    EXPECT_TRUE(validate(document, nullptr, {}).empty()) << dump(validate(document, nullptr, {}));
    std::get<0>(*document.joints[1].mimic) = "finger_joint";
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "finger_joint", "itself"), 1U);
}

TEST(Validation, SensorOnALinkWithAPositiveRate) {
    Document document = validRobot();
    document.sensors[0].link = "mast";
    document.sensors[0].updateRate = 0.0;
    document.sensors[0].latency = -0.01;
    const auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "imu", "link 'mast', which does not exist"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "imu", "update rate 0 Hz"), 1U) << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "imu", "cannot be negative"), 1U) << dump(problems);
}

/// The rangefinder of the catalog as a small entry written here (the loader is WP-C's).
CatalogEntry rangefinder() {
    CatalogEntry entry;
    entry.name = "rangefinder";
    CatalogParam max;
    max.path = "range/max";
    max.type = CatalogParam::Type::Number;
    max.unit = "m";
    max.min = 0.0;
    max.max = 100.0;
    CatalogParam rays;
    rays.path = "rays";
    rays.type = CatalogParam::Type::Int;
    rays.min = 1.0;
    CatalogParam enabled;
    enabled.path = "enabled";
    enabled.type = CatalogParam::Type::Bool;
    CatalogParam mode;
    mode.path = "mode";
    mode.type = CatalogParam::Type::Enum;
    mode.options = {"first", "last"};
    CatalogParam label;
    label.path = "label";
    label.type = CatalogParam::Type::Text;
    entry.params = {max, rays, enabled, mode, label};
    return entry;
}

TEST(Validation, ParamOutOfCatalogRange) {
    Sensor sensor;
    sensor.name = "front_range";
    sensor.link = "base_link";
    sensor.type = "rangefinder";
    sensor.params = {{"range/max", "20"},
                     {"rays", "4"},
                     {"enabled", "true"},
                     {"mode", "last"},
                     {"label", "any text"}};
    const CatalogEntry entry = rangefinder();
    EXPECT_TRUE(checkSensorParams(sensor, entry).empty()) << dump(checkSensorParams(sensor, entry));

    sensor.params["range/max"] = "120";
    auto problems = checkSensorParams(sensor, entry);
    EXPECT_EQ(count(problems, Level::Error, "front_range",
                    "range/max = 120 m is above the catalog's maximum 100 m"),
              1U)
        << dump(problems);
    sensor.params["range/max"] = "-1";
    problems = checkSensorParams(sensor, entry);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "below the catalog's minimum 0 m"), 1U)
        << dump(problems);
    // Of the right type.
    sensor.params["range/max"] = "12 m";
    sensor.params["rays"] = "2.5";
    sensor.params["enabled"] = "yes";
    sensor.params["mode"] = "middle";
    sensor.params["unknown/key"] = "1";
    problems = checkSensorParams(sensor, entry);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "range/max = '12 m' is not a number"),
              1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "not a whole number"), 1U);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "not true or false"), 1U);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "not one of first, last"), 1U);
    EXPECT_EQ(count(problems, Level::Warning, "front_range", "unknown/key is not a parameter"), 1U);
    // Errors come before warnings.
    EXPECT_EQ(problems.back().level, Level::Warning);

    // validate() with a catalog that does not know the type: a warning, parameters unchecked.
    // (An empty catalog: the loaded one is WP-C's; checkSensorParams above is what validate
    // applies to a type the catalog finds.)
    Document document = validRobot();
    document.sensors.push_back(sensor);
    const SensorCatalog empty;
    problems = validate(document, &empty, {});
    EXPECT_EQ(count(problems, Level::Warning, "front_range", "does not know"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "front_range", "not a number"), 0U);
}

TEST(Validation, MeshNotFound) {
    const std::filesystem::path dir = scratch("mesh");
    touch(dir / "meshes" / "body.stl");
    Document document = validRobot();
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "meshes/body.stl";
    mesh.solid = false;
    document.links[0].shapes.push_back(mesh);
    // Relative to the saved file's folder.
    EXPECT_TRUE(validate(document, nullptr, dir).empty()) << dump(validate(document, nullptr, dir));
    // Not saved yet: a relative mesh is not checked.
    EXPECT_TRUE(validate(document, nullptr, {}).empty());
    document.links[0].shapes.back().mesh = "meshes/wheel.stl";
    auto problems = validate(document, nullptr, dir);
    EXPECT_EQ(count(problems, Level::Error, "base_link", "mesh 'meshes/wheel.stl' is not found"),
              1U)
        << dump(problems);
    // Absolute and file:// paths.
    document.links[0].shapes.back().mesh = (dir / "meshes" / "body.stl").string();
    EXPECT_TRUE(validate(document, nullptr, {}).empty());
    document.links[0].shapes.back().mesh = "file://" + (dir / "nope.stl").string();
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "base_link", "not found"), 1U);
    // No file named.
    document.links[0].shapes.back().mesh.clear();
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "base_link", "no file"), 1U);
}

TEST(Validation, LagrangeAssetMeshResolvesThroughAssetDir) {
    const std::filesystem::path dir = scratch("assets");
    touch(dir / "holybro_x500_v2" / "base.stl");
    const ScopedEnv assets("LAGRANGE_ASSET_DIR", dir.string());
    Document document = validRobot();
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "lagrange-asset://holybro_x500_v2/base.stl";
    mesh.solid = false;
    document.links[0].shapes.push_back(mesh);
    EXPECT_TRUE(validate(document, nullptr, {}).empty()) << dump(validate(document, nullptr, {}));
    document.links[0].shapes.back().mesh = "lagrange-asset://holybro_x500_v2/arm.stl";
    const auto problems = validate(document, nullptr, {});
    EXPECT_EQ(count(problems, Level::Error, "base_link", "is not fetched"), 1U) << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "base_link", "fetch_assets"), 1U);
}

TEST(Validation, PackageMeshResolvesThroughAmentIndex) {
    const std::filesystem::path prefix = scratch("prefix");
    touch(prefix / "share" / "ament_index" / "resource_index" / "packages" / "my_robot");
    touch(prefix / "share" / "my_robot" / "meshes" / "base.dae");
    const ScopedEnv ament("AMENT_PREFIX_PATH", "/nonexistent:" + prefix.string());
    Document document = validRobot();
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "package://my_robot/meshes/base.dae";
    mesh.solid = false;
    document.links[0].shapes.push_back(mesh);
    EXPECT_TRUE(validate(document, nullptr, {}).empty()) << dump(validate(document, nullptr, {}));
    document.links[0].shapes.back().mesh = "package://not_a_package/x.dae";
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "base_link", "is not installed"),
              1U);
    // Not installed but the source tree is around the robot's file (as the server does).
    const std::filesystem::path source = scratch("source");
    std::ofstream(source / "package.xml") << "<package><name>src_robot</name></package>\n";
    touch(source / "meshes" / "arm.stl");
    document.links[0].shapes.back().mesh = "package://src_robot/meshes/arm.stl";
    EXPECT_TRUE(validate(document, nullptr, source / "urdf").empty())
        << dump(validate(document, nullptr, source / "urdf"));
}

TEST(Validation, SphereCasterOnAFixedJointIsAHint) {
    Document document = validRobot();
    Link caster;
    caster.name = "caster";
    Shape ball;
    ball.kind = Shape::Kind::Sphere;
    ball.size = {0.03, 0, 0};
    caster.shapes = {ball};
    caster.inertial.density = 1100.0;
    document.links.push_back(caster);
    Joint joint;
    joint.name = "caster_joint";
    joint.parent = "base_link";
    joint.child = "caster";
    document.joints.push_back(joint);
    auto problems = validate(document, nullptr, {});
    ASSERT_EQ(problems.size(), 1U) << dump(problems);
    EXPECT_EQ(count(problems, Level::Hint, "caster", "slides"), 1U);
    // On a moving joint it rolls: no hint.
    document.joints.back().type = Joint::Type::Continuous;
    EXPECT_TRUE(validate(document, nullptr, {}).empty());
}

TEST(Validation, ErrorsThenWarningsThenHintsInDocumentOrder) {
    Document document = validRobot();
    // A hint (caster), a warning (massless fixed link), two errors (zero axis, bad name).
    Link caster;
    caster.name = "caster";
    Shape ball;
    ball.kind = Shape::Kind::Sphere;
    ball.size = {0.03, 0, 0};
    caster.shapes = {ball};
    caster.inertial.density = 1100.0;
    Link mount;
    mount.name = "mount";
    document.links.push_back(caster);
    document.links.push_back(mount);
    Joint toCaster;
    toCaster.name = "caster_joint";
    toCaster.parent = "base_link";
    toCaster.child = "caster";
    Joint toMount = toCaster;
    toMount.name = "mount_joint";
    toMount.child = "mount";
    document.joints.push_back(toCaster);
    document.joints.push_back(toMount);
    document.joints[0].axis = {0, 0, 0};
    document.robot = "9lives";
    const auto problems = validate(document, nullptr, {});
    ASSERT_EQ(problems.size(), 4U) << dump(problems);
    EXPECT_EQ(problems[0].level, Level::Error);
    EXPECT_EQ(problems[0].item, "");  // the robot first
    EXPECT_EQ(problems[1].level, Level::Error);
    EXPECT_EQ(problems[1].item, "shoulder");
    EXPECT_EQ(problems[2].level, Level::Warning);
    EXPECT_EQ(problems[3].level, Level::Hint);
}

TEST(Validation, AMasslessShapelessRootNamedWorldIsAFixedBase) {
    Document document = validRobot();
    Link world;
    world.name = "world";
    document.links.insert(document.links.begin(), world);
    Joint bolt;
    bolt.name = "world_joint";
    bolt.type = Joint::Type::Fixed;
    bolt.parent = "world";
    bolt.child = document.links[1].name;
    document.joints.push_back(bolt);
    EXPECT_TRUE(validate(document, nullptr, {}).empty()) << dump(validate(document, nullptr, {}));
    // Any other massless root is still an error, and so is a "world" that is not the root.
    document.links.front().name = "ground";
    document.joints.back().parent = "ground";
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Error, "ground", "has no mass"), 1U);
}

TEST(Validation, EveryTemplateValidatesWithNoError) {
    // The builder's File > New from template (assets/robots/templates): each one opens clean,
    // its sensors checked against the real catalog (built-in and plugin kinds).
    std::vector<std::filesystem::path> files = {LAGRANGE_SENSOR_CATALOG_YAML};
    for (const auto& entry : std::filesystem::directory_iterator(LAGRANGE_PLUGIN_CATALOG_DIR)) {
        files.push_back(entry.path());
    }
    std::ranges::sort(files);
    auto loaded = SensorCatalog::load(files);
    ASSERT_TRUE(std::holds_alternative<SensorCatalog>(loaded));
    const SensorCatalog& catalog = std::get<SensorCatalog>(loaded);
    const std::filesystem::path dir =
        std::filesystem::path(LAGRANGE_BUILDER_ASSETS_DIR) / "templates";
    int templates = 0;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() != ".urdf") {
            continue;
        }
        ++templates;
        std::ifstream in(entry.path());
        const std::string text((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
        const ReadResult read = readUrdf(text);
        ASSERT_TRUE(read.document) << entry.path() << ": " << read.error;
        const auto problems = validate(*read.document, &catalog, dir);
        EXPECT_EQ(std::ranges::count(problems, Level::Error, &Problem::level), 0)
            << entry.path() << "\n"
            << dump(problems);
    }
    EXPECT_EQ(templates, 3);
}

const SensorCatalog& realCatalog() {
    static const SensorCatalog catalog = [] {
        auto loaded = SensorCatalog::load({LAGRANGE_SENSOR_CATALOG_YAML});
        EXPECT_TRUE(std::holds_alternative<SensorCatalog>(loaded));
        return std::holds_alternative<SensorCatalog>(loaded) ? std::get<SensorCatalog>(loaded)
                                                             : SensorCatalog{};
    }();
    return catalog;
}

Document rover() {
    std::ifstream in(std::filesystem::path(LAGRANGE_BUILDER_TEST_DIR) / "rover_mk1.urdf");
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const ReadResult read = readUrdf(text);
    EXPECT_TRUE(read.document) << read.error;
    return read.document ? *read.document : Document{};
}

/// The problems about one item, at one level.
std::vector<Problem> about(const std::vector<Problem>& problems, std::string_view item,
                           Level level) {
    std::vector<Problem> out;
    std::ranges::copy_if(problems, std::back_inserter(out),
                         [&](const Problem& p) { return p.item == item && p.level == level; });
    return out;
}

TEST(Validation, EveryCatalogKindWithItsDefaultsSaysWhatTheServerWillRefuse) {
    // QA F5: each URDF kind of the catalog added with its defaults on the rover's base_link.
    // The server refused these 15 (and the sonar, F10) (qa-builder per/<kind>.urdf, each alone) and
    // started the rest; the builder must say so before Test: an error for what the robot lacks, a
    // warning for what the world it runs in may give. test_sensor_catalog (lagrange_sensors) checks
    // the catalog's marks against the server's kinds both ways.
    const std::map<std::string, Level, std::less<>> refused{
        {"battery_monitor", Level::Error},
        {"current_sensor", Level::Error},
        {"voltage_sensor", Level::Error},
        {"detections", Level::Error},
        {"esc_telemetry", Level::Error},
        {"force_torque", Level::Error},
        {"joint_torque", Level::Error},
        {"gripper_sensor", Level::Error},
        {"imaging_radar", Level::Error},
        {"landing_target", Level::Warning},
        {"safety_scanner", Level::Error},
        {"tension_sensor", Level::Warning},
        {"tpms", Level::Error},
        {"uwb", Level::Warning},
        {"vehicle_can", Level::Error},
        // QA F10: beside the rover's 15 Hz camera its 160 ms ping never lines up (alone, it
        // starts): a warning naming both.
        {"sonar", Level::Warning},
    };
    std::size_t kinds = 0;
    for (const CatalogEntry& entry : realCatalog().entries()) {
        if (entry.builtinType || entry.overlayOnly) {
            continue;
        }
        ++kinds;
        Document document = rover();
        Sensor sensor;
        sensor.name = "probe";
        sensor.link = "base_link";
        sensor.type = entry.name;
        document.sensors.push_back(sensor);
        const auto problems = validate(document, &realCatalog(), {});
        const auto errors = about(problems, "probe", Level::Error);
        const auto warnings = about(problems, "probe", Level::Warning);
        const auto expected = refused.find(entry.name);
        if (expected == refused.end()) {
            EXPECT_TRUE(errors.empty() && warnings.empty())
                << entry.name << " starts on the rover, yet:\n"
                << dump(problems);
        } else if (expected->second == Level::Error) {
            EXPECT_FALSE(errors.empty()) << entry.name << ": no error\n" << dump(problems);
        } else {
            EXPECT_TRUE(errors.empty()) << entry.name << ": a world may give it\n"
                                        << dump(problems);
            EXPECT_FALSE(warnings.empty()) << entry.name << ": no warning\n" << dump(problems);
        }
    }
    EXPECT_GE(kinds, refused.size());
}

TEST(Validation, ASensorsNeedsAreMetByWhatTheRobotHas) {
    Document document = rover();
    const auto add = [&](std::string name, std::string type, std::string link,
                         std::map<std::string, std::string, std::less<>> params) {
        Sensor s;
        s.name = std::move(name);
        s.type = std::move(type);
        s.link = std::move(link);
        s.params = std::move(params);
        document.sensors.push_back(std::move(s));
    };
    add("ft", "force_torque", "tilt_link", {});  // below the tilt hinge
    add("ft_named", "force_torque", "base_link", {{"joint", "pan_joint"}});
    add("ft_fixed", "joint_torque", "mast", {});  // mast is fixed to the root
    add("labels", "detections", "base_link", {{"camera", "camera"}});
    add("labels_wrong", "detections", "base_link", {{"camera", "imu"}});
    add("monitor", "battery_monitor", "base_link", {{"battery", "pack"}});
    add("scanner", "safety_scanner", "base_link", {{"protective/radius", "1.5"}});
    add("grip", "gripper_sensor", "base_link", {{"fingers", "pan_joint nope"}});
    add("odometry", "wheel_odometry", "base_link", {});
    add("odometry_mast", "wheel_odometry", "mast", {});
    const auto problems = validate(document, &realCatalog(), {});
    for (const char* fine : {"ft", "ft_named", "labels", "monitor", "scanner", "odometry"}) {
        EXPECT_TRUE(about(problems, fine, Level::Error).empty()) << fine << "\n" << dump(problems);
    }
    EXPECT_EQ(count(problems, Level::Error, "ft_fixed", "hangs from no hinge or slide"), 1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Error, "labels_wrong", "not a camera sensor"), 1U);
    EXPECT_EQ(count(problems, Level::Error, "grip", "joint 'nope'"), 1U);
    EXPECT_EQ(count(problems, Level::Error, "odometry_mast", "has 0 such hinges"), 1U);
}

TEST(Validation, SensorRatesThatNeverLineUpAreNamed) {
    // QA F10: the catalog sonar (120 m in water: a ping every 160 ms) beside the rover's 15 Hz
    // camera (67 ms) line up every 10.72 s; the scheduler refuses over 10 s.
    Document document = rover();
    Sensor sonar;
    sonar.name = "sonar";
    sonar.type = "sonar";
    sonar.link = "base_link";
    document.sensors.push_back(sonar);
    auto problems = validate(document, &realCatalog(), {});
    EXPECT_EQ(count(problems, Level::Warning, "sonar",
                    "sensors 'camera' (every 67 ms) and 'sonar' (every 160 ms, its ping"),
              1U)
        << dump(problems);
    EXPECT_EQ(count(problems, Level::Warning, "sonar", "every 10.72 s"), 1U) << dump(problems);
    // A 10 Hz camera (100 ms) lines up with it every 800 ms.
    findSensor(document, "camera")->updateRate = 10.0;
    problems = validate(document, &realCatalog(), {});
    EXPECT_EQ(count(problems, Level::Warning, "sonar", "line up only"), 0U) << dump(problems);
}
// QA F8: a part of a tonne or more from its shapes is a size to check.
TEST(Validation, WarnsOnAnImplausiblyHeavyPart) {
    Document document = validRobot();
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Warning, "arm", "kg from its shapes"),
              0U)
        << dump(validate(document, nullptr, {}));
    document.links[1].shapes[0].size = {2.0, 100.0, 0.0};  // a 100 m aluminium rod
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Warning, "arm", "kg from its shapes"),
              1U)
        << dump(validate(document, nullptr, {}));
    document.links[1].inertial.mass = 3.0;  // the user's own mass: not second-guessed
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Warning, "arm", "kg from its shapes"),
              0U);
}

// QA F8: a mesh tens of metres across is likely a file in millimetres.
TEST(Validation, WarnsOnAMeshTooBigForARobot) {
    const std::filesystem::path dir = scratch("huge_mesh");
    touch(dir / "part.stl");  // a 1 m tetrahedron
    Document document = validRobot();
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "part.stl";
    mesh.solid = false;
    document.links[0].shapes.push_back(mesh);
    EXPECT_EQ(count(validate(document, nullptr, dir), Level::Warning, "base_link", "file in mm"),
              0U);
    document.links[0].shapes.back().scale = {20.0, 20.0, 20.0};
    EXPECT_EQ(count(validate(document, nullptr, dir), Level::Warning, "base_link", "m across"), 1U)
        << dump(validate(document, nullptr, dir));
}

// QA F20: a kept <ros2_control> naming a joint the robot lacks is a warning.
TEST(Validation, WarnsOnAStaleRos2ControlJoint) {
    Document document = validRobot();
    document.keptXml =
        "<ros2_control name=\"hw\" type=\"system\"><joint name=\"shoulder\"/>"
        "</ros2_control>";
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Warning, "", "<ros2_control>"), 0U);
    document.keptXml =
        "<ros2_control name=\"hw\" type=\"system\"><joint name=\"elbow\"/>"
        "</ros2_control>";
    EXPECT_EQ(count(validate(document, nullptr, {}), Level::Warning, "", "'elbow'"), 1U)
        << dump(validate(document, nullptr, {}));
}

}  // namespace
}  // namespace lagrange::viewer::builder
