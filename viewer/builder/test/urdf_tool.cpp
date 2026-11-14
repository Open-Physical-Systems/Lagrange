// A test helper of the builder's URDF code (RFC 0016 WP-B), not installed:
//   builder_urdf_tool roundtrip IN.urdf OUT.urdf   read IN and write it again (notes on stderr)
//   builder_urdf_tool templates DIR                write the three templates (open question 11)
// test_builder_roundtrip.py runs the first; the second made assets/robots/templates/.
#include "lagrange/viewer/builder/Urdf.hpp"

#include <cstdio>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>

namespace builder = lagrange::viewer::builder;
using builder::Document;
using builder::Joint;
using builder::Link;
using builder::Sensor;
using builder::Shape;

namespace {

int roundTrip(const char* in, const char* out) {
    std::ifstream file(in);
    if (!file) {
        std::fprintf(stderr, "cannot read %s\n", in);
        return 2;
    }
    std::stringstream text;
    text << file.rdbuf();
    const builder::ReadResult read = builder::readUrdf(text.str());
    if (!read.document) {
        std::fprintf(stderr, "%s: %s\n", in, read.error.c_str());
        return 1;
    }
    for (const std::string& note : read.notes) {
        std::fprintf(stderr, "note: %s\n", note.c_str());
    }
    if (const std::string why = builder::saveAtomically(out, builder::writeUrdf(*read.document));
        !why.empty()) {
        std::fprintf(stderr, "%s\n", why.c_str());
        return 1;
    }
    return 0;
}

Shape box(std::array<double, 3> size, std::array<double, 3> at, std::array<float, 4> rgba) {
    Shape s;
    s.kind = Shape::Kind::Box;
    s.size = size;
    s.origin.xyz = at;
    s.rgba = rgba;
    return s;
}

Shape cylinder(double radius, double length, builder::Pose at, std::array<float, 4> rgba) {
    Shape s;
    s.kind = Shape::Kind::Cylinder;
    s.size = {radius, length, 0};
    s.origin = at;
    s.rgba = rgba;
    return s;
}

Link link(std::string name, double density, std::vector<Shape> shapes) {
    Link l;
    l.name = std::move(name);
    l.inertial.density = density;
    l.shapes = std::move(shapes);
    return l;
}

Joint joint(std::string name, Joint::Type type, std::string parent, std::string child,
            std::array<double, 3> at, std::array<double, 3> axis) {
    Joint j;
    j.name = std::move(name);
    j.type = type;
    j.parent = std::move(parent);
    j.child = std::move(child);
    j.origin.xyz = at;
    j.axis = axis;
    return j;
}

// Densities: the illustration values of RFC 0016 §4 (aluminium-like 2700, PLA-like 1240,
// rubber-like 1100 kg/m³), illustrative until the presets are verified (plan §6). Sizes, limits,
// efforts and speeds are illustrative too: a starting point to edit, not a real robot.
constexpr double kMetal = 2700, kPlastic = 1240, kRubber = 1100;
constexpr std::array<float, 4> kBody{0.66F, 0.72F, 0.81F, 1}, kDark{0.16F, 0.18F, 0.22F, 1};
constexpr double kQuarter = std::numbers::pi / 2;

std::string comment(const std::string& text) {
    return "<!-- " + text + " -->";
}

Document diffDrive() {
    Document d;
    d.robot = "diff_drive_base";
    d.keptXml = comment(
                    "Template of the robot builder (RFC 0016): a differential-drive base, two "
                    "driven wheels\n     and a caster. Sizes, densities, efforts and speeds "
                    "are illustrative; edit them.") +
                "\n<!--lagrange-builder:robot-->";
    d.links.push_back(link("base_link", kPlastic, {box({0.4, 0.3, 0.1}, {0, 0, 0}, kBody)}));
    for (const auto& [name, y] : {std::pair{"left_wheel", 0.17}, std::pair{"right_wheel", -0.17}}) {
        d.links.push_back(
            link(name, kRubber, {cylinder(0.07, 0.04, {{0, 0, 0}, {kQuarter, 0, 0}}, kDark)}));
        Joint j = joint(std::string(name) + "_joint", Joint::Type::Continuous, "base_link", name,
                        {0.08, y, -0.03}, {0, 1, 0});
        j.effort = 5;
        j.velocity = 10;
        d.joints.push_back(j);
    }
    Shape ball;
    ball.kind = Shape::Kind::Sphere;
    ball.size = {0.035, 0, 0};
    ball.rgba = kDark;
    d.links.push_back(link("caster", kRubber, {ball}));
    d.joints.push_back(joint("caster_joint", Joint::Type::Fixed, "base_link", "caster",
                             {-0.15, 0, -0.065}, {1, 0, 0}));
    Sensor imu;
    imu.name = "imu";
    imu.link = "base_link";
    imu.type = "imu";
    imu.updateRate = 100;
    d.sensors.push_back(imu);
    return d;
}

Document arm() {
    Document d;
    d.robot = "arm_3dof";
    d.keptXml = comment(
                    "Template of the robot builder (RFC 0016): a three-joint arm on a fixed "
                    "base (the root link\n     \"world\" bolts it to the ground). Sizes, "
                    "densities, limits, efforts and speeds are\n     illustrative; edit them.") +
                "\n<!--lagrange-builder:robot-->";
    Link world;
    world.name = "world";
    d.links.push_back(world);
    d.links.push_back(
        link("base_link", kMetal, {cylinder(0.08, 0.05, {{0, 0, 0.025}, {0, 0, 0}}, kBody)}));
    d.links.push_back(
        link("shoulder", kMetal, {cylinder(0.05, 0.1, {{0, 0, 0.05}, {0, 0, 0}}, kBody)}));
    d.links.push_back(link("upper_arm", kMetal, {box({0.05, 0.05, 0.3}, {0, 0, 0.15}, kBody)}));
    d.links.push_back(link("forearm", kMetal, {box({0.04, 0.04, 0.25}, {0, 0, 0.125}, kBody)}));
    d.joints.push_back(
        joint("base_joint", Joint::Type::Fixed, "world", "base_link", {0, 0, 0}, {1, 0, 0}));
    const auto revolute = [](std::string name, std::string parent, std::string child,
                             std::array<double, 3> at, std::array<double, 3> axis, double lower,
                             double upper) {
        Joint j = joint(std::move(name), Joint::Type::Revolute, std::move(parent), std::move(child),
                        at, axis);
        j.lower = lower;
        j.upper = upper;
        j.effort = 20;
        j.velocity = 2;
        return j;
    };
    d.joints.push_back(revolute("shoulder_yaw", "base_link", "shoulder", {0, 0, 0.05}, {0, 0, 1},
                                -std::numbers::pi, std::numbers::pi));
    d.joints.push_back(revolute("shoulder_pitch", "shoulder", "upper_arm", {0, 0, 0.1}, {0, 1, 0},
                                -kQuarter, kQuarter));
    d.joints.push_back(
        revolute("elbow", "upper_arm", "forearm", {0, 0, 0.3}, {0, 1, 0}, -2.5, 2.5));
    return d;
}

Document panTilt() {
    Document d;
    d.robot = "pan_tilt";
    d.keptXml = comment(
                    "Template of the robot builder (RFC 0016): a pan-tilt head with a camera on "
                    "a fixed base (the\n     root link \"world\" bolts it to the ground). "
                    "Sizes, densities, limits, efforts and speeds\n     are illustrative; edit "
                    "them.") +
                "\n<!--lagrange-builder:robot-->";
    Link world;
    world.name = "world";
    d.links.push_back(world);
    d.links.push_back(link("base_link", kMetal, {box({0.1, 0.1, 0.04}, {0, 0, 0.02}, kBody)}));
    d.links.push_back(link("pan_link", kPlastic, {box({0.06, 0.06, 0.04}, {0, 0, 0.02}, kBody)}));
    d.links.push_back(link("tilt_link", kPlastic, {box({0.04, 0.08, 0.04}, {0, 0, 0}, kDark)}));
    d.joints.push_back(
        joint("base_joint", Joint::Type::Fixed, "world", "base_link", {0, 0, 0}, {1, 0, 0}));
    Joint pan =
        joint("pan_joint", Joint::Type::Revolute, "base_link", "pan_link", {0, 0, 0.04}, {0, 0, 1});
    pan.lower = -2.9;
    pan.upper = 2.9;
    pan.effort = 2;
    pan.velocity = 3;
    d.joints.push_back(pan);
    Joint tilt = joint("tilt_joint", Joint::Type::Revolute, "pan_link", "tilt_link", {0, 0, 0.06},
                       {0, 1, 0});
    tilt.lower = -0.8;
    tilt.upper = 1.0;
    tilt.effort = 2;
    tilt.velocity = 3;
    d.joints.push_back(tilt);
    Sensor camera;
    camera.name = "camera";
    camera.link = "tilt_link";
    camera.type = "camera";
    camera.origin.xyz = {0.02, 0, 0};
    camera.updateRate = 15;
    camera.params = {{"camera/horizontal_fov", "1.2"},
                     {"camera/image/width", "640"},
                     {"camera/image/height", "480"}};
    d.sensors.push_back(camera);
    return d;
}

int templates(const std::string& dir) {
    for (const auto& [name, document] :
         {std::pair{"diff_drive_base", diffDrive()}, std::pair{"arm_3dof", arm()},
          std::pair{"pan_tilt", panTilt()}}) {
        const std::string path = dir + "/" + name + ".urdf";
        if (const std::string why = builder::saveAtomically(path, builder::writeUrdf(document));
            !why.empty()) {
            std::fprintf(stderr, "%s\n", why.c_str());
            return 1;
        }
        std::printf("wrote %s\n", path.c_str());
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "roundtrip" && argc == 4) {
        return roundTrip(argv[2], argv[3]);
    }
    if (command == "templates" && argc == 3) {
        return templates(argv[2]);
    }
    std::fprintf(stderr, "usage: builder_urdf_tool roundtrip IN.urdf OUT.urdf | templates DIR\n");
    return 2;
}
