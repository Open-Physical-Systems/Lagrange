// The builder's own validation (RFC 0016 §8.1, plan WP-A): one test per rule.
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
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

}  // namespace
}  // namespace lagrange::viewer::builder
