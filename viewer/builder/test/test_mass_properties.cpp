// Mass properties from shapes (RFC 0016 §4, plan WP-A): the closed forms, rotation, the
// parallel axis theorem, overrides, the engines' checks, and the sample rover_mk1's totals.
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/Materials.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

constexpr double kPi = std::numbers::pi;

Shape box(double a, double b, double c) {
    Shape shape;
    shape.kind = Shape::Kind::Box;
    shape.size = {a, b, c};
    return shape;
}

Shape cylinder(double r, double h) {
    Shape shape;
    shape.kind = Shape::Kind::Cylinder;
    shape.size = {r, h, 0.0};
    return shape;
}

Shape sphere(double r) {
    Shape shape;
    shape.kind = Shape::Kind::Sphere;
    shape.size = {r, 0.0, 0.0};
    return shape;
}

void expectTensor(const std::array<double, 6>& actual, const std::array<double, 6>& expected,
                  double tolerance) {
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_NEAR(actual.at(k), expected.at(k), tolerance) << "component " << k;
    }
}

TEST(MassProperties, BoxCylinderSphereMatchClosedForms) {
    // Box 0.5 × 0.36 × 0.12 m of 1000 kg/m³: m = 21.6 kg.
    const MassProperties b = shapeMass(box(0.5, 0.36, 0.12), 1000.0);
    EXPECT_NEAR(b.mass, 21.6, 1e-12);
    expectTensor(
        b.inertia,
        {21.6 * (0.36 * 0.36 + 0.12 * 0.12) / 12.0, 0, 0, 21.6 * (0.5 * 0.5 + 0.12 * 0.12) / 12.0,
         0, 21.6 * (0.5 * 0.5 + 0.36 * 0.36) / 12.0},
        1e-12);
    // Cylinder r 0.08, h 0.04 of 1100 kg/m³ (the rover's wheel, along z here).
    const MassProperties c = shapeMass(cylinder(0.08, 0.04), 1100.0);
    const double mc = 1100.0 * kPi * 0.08 * 0.08 * 0.04;
    EXPECT_NEAR(c.mass, mc, 1e-12);
    EXPECT_NEAR(c.mass, 0.8846724912508859, 1e-12);  // rover_mk1.urdf's wheel
    const double across = mc * (3 * 0.08 * 0.08 + 0.04 * 0.04) / 12.0;
    expectTensor(c.inertia, {across, 0, 0, across, 0, mc * 0.08 * 0.08 / 2.0}, 1e-15);
    // Sphere r 0.035 of 1100 kg/m³ (the caster).
    const MassProperties s = shapeMass(sphere(0.035), 1100.0);
    const double ms = 1100.0 * 4.0 * kPi * std::pow(0.035, 3) / 3.0;
    EXPECT_NEAR(s.mass, ms, 1e-15);
    const double i = 2.0 * ms * 0.035 * 0.035 / 5.0;
    expectTensor(s.inertia, {i, 0, 0, i, 0, i}, 1e-15);
    // The shape's origin is its COM in the link frame.
    Shape moved = sphere(0.1);
    moved.origin.xyz = {0.1, -0.2, 0.3};
    const MassProperties m = shapeMass(moved, 1000.0);
    EXPECT_DOUBLE_EQ(m.com[0], 0.1);
    EXPECT_DOUBLE_EQ(m.com[1], -0.2);
    EXPECT_DOUBLE_EQ(m.com[2], 0.3);
    // No density, a zero size or a mesh (phase 2): no mass.
    EXPECT_EQ(shapeMass(box(1, 1, 1), 0.0).mass, 0.0);
    EXPECT_EQ(shapeMass(box(1, 0, 1), 1000.0).mass, 0.0);
    Shape mesh;
    mesh.kind = Shape::Kind::Mesh;
    mesh.mesh = "part.stl";
    EXPECT_EQ(shapeMass(mesh, 1000.0).mass, 0.0);
}

TEST(MassProperties, RotatedShapeRotatesTensor) {
    // A cylinder along z turned by roll 90° lies along -y: its axial moment moves to Iyy.
    Shape wheel = cylinder(0.08, 0.04);
    const MassProperties upright = shapeMass(wheel, 1100.0);
    wheel.origin.rpy = {kPi / 2.0, 0.0, 0.0};
    const MassProperties turned = shapeMass(wheel, 1100.0);
    EXPECT_DOUBLE_EQ(turned.mass, upright.mass);
    expectTensor(turned.inertia,
                 {upright.inertia[0], 0, 0, upright.inertia[5], 0, upright.inertia[3]}, 1e-15);
    // A box turned 45° about z: Ixx' = Iyy' = (Ixx + Iyy)/2, Ixy' = (Ixx − Iyy)/2 · sin 90°
    // with R I Rᵀ (URDF's tensor entries, not the negated products of inertia).
    Shape slab = box(0.4, 0.1, 0.1);
    const MassProperties straight = shapeMass(slab, 1000.0);
    slab.origin.rpy = {0.0, 0.0, kPi / 4.0};
    const MassProperties diagonal = shapeMass(slab, 1000.0);
    const double ixx = straight.inertia[0];
    const double iyy = straight.inertia[3];
    EXPECT_NEAR(diagonal.inertia[0], (ixx + iyy) / 2.0, 1e-15);
    EXPECT_NEAR(diagonal.inertia[3], (ixx + iyy) / 2.0, 1e-15);
    EXPECT_NEAR(diagonal.inertia[1], (ixx - iyy) / 2.0, 1e-15);
    EXPECT_NEAR(diagonal.inertia[5], straight.inertia[5], 1e-15);
    EXPECT_NEAR(diagonal.inertia[2], 0.0, 1e-15);
    EXPECT_NEAR(diagonal.inertia[4], 0.0, 1e-15);
    // The trace is invariant under rotation.
    slab.origin.rpy = {0.3, -0.7, 1.1};
    const MassProperties any = shapeMass(slab, 1000.0);
    EXPECT_NEAR(any.inertia[0] + any.inertia[3] + any.inertia[5],
                straight.inertia[0] + straight.inertia[3] + straight.inertia[5], 1e-15);
    EXPECT_EQ(checkInertia(any.inertia), InertiaCheck::Ok);
}

TEST(MassProperties, CompositeUsesParallelAxis) {
    // Two unit cubes of 1000 kg/m³ side by side along x make a 2 × 1 × 1 box.
    Link link;
    link.inertial.density = 1000.0;
    Shape left = box(1, 1, 1);
    left.origin.xyz = {-0.5, 0.0, 0.0};
    Shape right = box(1, 1, 1);
    right.origin.xyz = {0.5, 0.0, 0.0};
    link.shapes = {left, right};
    const auto mass = linkMass(link);
    ASSERT_TRUE(mass.has_value());
    EXPECT_NEAR(mass->mass, 2000.0, 1e-9);
    EXPECT_NEAR(mass->com[0], 0.0, 1e-12);
    const MassProperties whole = shapeMass(box(2, 1, 1), 1000.0);
    expectTensor(mass->inertia, whole.inertia, 1e-9);
    // Off-diagonal terms: two equal point-like spheres at (d, d, 0) and (−d, −d, 0) about their
    // common COM give ixy = −2 m d² (I = Σ m(|r|²E − r rᵀ)).
    Link pair;
    pair.inertial.density = 1000.0;
    Shape a = sphere(0.01);
    a.origin.xyz = {0.5, 0.5, 0.0};
    Shape b = sphere(0.01);
    b.origin.xyz = {-0.5, -0.5, 0.0};
    pair.shapes = {a, b};
    const auto spread = linkMass(pair);
    ASSERT_TRUE(spread.has_value());
    const double m = shapeMass(a, 1000.0).mass;
    EXPECT_NEAR(spread->inertia[1], -2.0 * m * 0.25, 1e-12);
    // A visual-only shape is massless (open question 12).
    Link withDecal = link;
    Shape decal = box(5, 5, 5);
    decal.solid = false;
    decal.collision = false;
    withDecal.shapes.push_back(decal);
    EXPECT_NEAR(linkMass(withDecal)->mass, 2000.0, 1e-9);
    // Nothing gives a mass: nullopt.
    Link empty;
    empty.shapes = {box(1, 1, 1)};
    EXPECT_FALSE(linkMass(empty).has_value());
}

TEST(MassProperties, OverrideScalesTensor) {
    Link link;
    link.inertial.density = 1000.0;
    link.shapes = {box(0.2, 0.3, 0.4)};
    const MassProperties computed = *linkMass(link);
    // A mass override keeps the distribution: the tensor scales by m_override / m_computed.
    link.inertial.mass = computed.mass * 3.0;
    const MassProperties scaled = *linkMass(link);
    EXPECT_DOUBLE_EQ(scaled.mass, computed.mass * 3.0);
    for (std::size_t k = 0; k < 6; ++k) {
        EXPECT_NEAR(scaled.inertia.at(k), computed.inertia.at(k) * 3.0, 1e-15);
    }
    // The same without a density: the shapes still give the distribution (rover's base_link).
    Link base;
    base.shapes = {box(0.5, 0.36, 0.12)};
    base.inertial.mass = 8.0;
    const MassProperties overridden = *linkMass(base);
    EXPECT_DOUBLE_EQ(overridden.mass, 8.0);
    expectTensor(overridden.inertia, {0.096, 0, 0, 0.17626666666666668, 0, 0.25306666666666666},
                 1e-15);  // rover_mk1.urdf's base_link
    // An inertia override replaces the tensor; a COM override replaces the COM.
    base.inertial.inertia = std::array<double, 6>{1, 0, 0, 2, 0, 2.5};
    base.inertial.com = std::array<double, 3>{0.01, 0.02, 0.03};
    const MassProperties replaced = *linkMass(base);
    expectTensor(replaced.inertia, {1, 0, 0, 2, 0, 2.5}, 0.0);
    EXPECT_EQ(replaced.com, (std::array<double, 3>{0.01, 0.02, 0.03}));
    EXPECT_DOUBLE_EQ(replaced.mass, 8.0);
}

TEST(MassProperties, ChecksPositiveDefiniteAndTriangle) {
    EXPECT_EQ(checkInertia({1, 0, 0, 1, 0, 1}), InertiaCheck::Ok);
    EXPECT_EQ(checkInertia(shapeMass(box(0.5, 0.36, 0.12), 1000).inertia), InertiaCheck::Ok);
    // A thin disc meets the triangle inequality with equality: Ixx + Iyy = Izz.
    EXPECT_EQ(checkInertia({1, 0, 0, 1, 0, 2}), InertiaCheck::Ok);
    EXPECT_EQ(checkInertia({0, 0, 0, 0, 0, 0}), InertiaCheck::NotPositiveDefinite);
    EXPECT_EQ(checkInertia({1, 0, 0, 1, 0, -1}), InertiaCheck::NotPositiveDefinite);
    // Positive diagonal but an off-diagonal term too large: one principal moment negative.
    EXPECT_EQ(checkInertia({1, 2, 0, 1, 0, 1}), InertiaCheck::NotPositiveDefinite);
    EXPECT_EQ(checkInertia({1, 0, 0, 1, 0, 3}), InertiaCheck::TriangleInequality);
    EXPECT_EQ(checkInertia({1, 0, 0, std::nan(""), 0, 1}), InertiaCheck::NotPositiveDefinite);
    // The triangle inequality is on the principal moments: a valid tensor rotated stays valid,
    // a broken one rotated stays broken although its diagonal alone might pass.
    Shape rod = box(0.01, 0.01, 1.0);
    rod.origin.rpy = {0.4, 0.9, -0.2};
    EXPECT_EQ(checkInertia(shapeMass(rod, 1000).inertia), InertiaCheck::Ok);
    // diag(1, 1, 3) rotated 45° about x: diagonal (1, 2, 2) passes, the tensor does not.
    EXPECT_EQ(checkInertia({1, 0, 0, 2, -1, 2}), InertiaCheck::TriangleInequality);
}

/// rover_mk1 (docs/design/ui/12-builder/README.md, viewer/test/builder/rover_mk1.urdf) as a
/// document, written here because the URDF reader is WP-B's.
Document roverMk1() {
    Document document;
    document.robot = "rover_mk1";
    const auto addLink = [&](const std::string& name, Shape shape, double density) {
        Link link;
        link.name = name;
        link.shapes = {shape};
        link.inertial.density = density;
        document.links.push_back(link);
    };
    const auto addJoint = [&](const std::string& name, Joint::Type type, const std::string& parent,
                              const std::string& child, std::array<double, 3> xyz,
                              std::array<double, 3> axis) {
        Joint joint;
        joint.name = name;
        joint.type = type;
        joint.parent = parent;
        joint.child = child;
        joint.origin.xyz = xyz;
        joint.axis = axis;
        document.joints.push_back(joint);
    };
    addLink("base_link", box(0.5, 0.36, 0.12), 0.0);
    document.links.back().inertial.mass = 8.0;  // the README's override
    Shape wheel = cylinder(0.08, 0.04);
    wheel.origin.rpy = {kPi / 2.0, 0.0, 0.0};
    addLink("left_wheel", wheel, 1100.0);
    addJoint("left_wheel_joint", Joint::Type::Continuous, "base_link", "left_wheel",
             {0.1, 0.2, -0.05}, {0, 1, 0});
    addLink("right_wheel", wheel, 1100.0);
    addJoint("right_wheel_joint", Joint::Type::Continuous, "base_link", "right_wheel",
             {0.1, -0.2, -0.05}, {0, 1, 0});
    addLink("caster", sphere(0.035), 1100.0);
    addJoint("caster_joint", Joint::Type::Fixed, "base_link", "caster", {-0.22, 0, -0.095},
             {1, 0, 0});
    Shape mast = cylinder(0.02, 0.2);
    mast.origin.xyz = {0, 0, 0.1};
    addLink("mast", mast, 2700.0);
    addJoint("mast_joint", Joint::Type::Fixed, "base_link", "mast", {-0.15, 0, 0.06}, {1, 0, 0});
    Shape pan = box(0.06, 0.06, 0.04);
    pan.origin.xyz = {0, 0, 0.02};
    addLink("pan_link", pan, 1240.0);
    addJoint("pan_joint", Joint::Type::Revolute, "base_link", "pan_link", {0.15, 0, 0.06},
             {0, 0, 1});
    addLink("tilt_link", box(0.04, 0.08, 0.04), 1240.0);
    addJoint("tilt_joint", Joint::Type::Revolute, "pan_link", "tilt_link", {0, 0, 0.06}, {0, 1, 0});
    return document;
}

TEST(MassProperties, RoverMk1TotalsTheReadme) {
    const Document rover = roverMk1();
    // Per part, as the README's table lists them (to its 3 decimals) and as rover_mk1.urdf
    // writes them.
    const std::array<std::pair<const char*, double>, 7> parts{{{"base_link", 8.0},
                                                               {"left_wheel", 0.8846724912508859},
                                                               {"right_wheel", 0.8846724912508859},
                                                               {"caster", 0.1975538180332382},
                                                               {"mast", 0.6785840131753954},
                                                               {"pan_link", 0.17856},
                                                               {"tilt_link", 0.15872000000000003}}};
    for (const auto& [name, mass] : parts) {
        const Link* link = findLink(rover, name);
        ASSERT_NE(link, nullptr) << name;
        EXPECT_NEAR(linkMass(*link)->mass, mass, 1e-12) << name;
    }
    const MassProperties total = robotMass(rover);
    // The parts add up to 10.983 kg. The README's "Total 10.99 kg" is rounded up from that
    // (rover_mk1.urdf's comment says the same): the sum is what is tested.
    EXPECT_NEAR(total.mass, 10.983, 0.0005);
    EXPECT_NEAR(total.com[0], 0.0075, 0.00005);  // README: 0.007
    EXPECT_NEAR(total.com[1], 0.0, 1e-12);
    EXPECT_NEAR(total.com[2], 0.0032, 0.00005);  // README: 0.003
    EXPECT_EQ(checkInertia(total.inertia), InertiaCheck::Ok);
    // The README's densities are presets: 1100 ABS, 1240 PLA, 2700 aluminium.
    const auto density = [](std::string_view name) {
        for (const DensityPreset& preset : densityPresets()) {
            if (preset.name == name) {
                return preset.density;
            }
        }
        return 0.0;
    };
    EXPECT_EQ(density("abs"), 1100.0);
    EXPECT_EQ(density("pla"), 1240.0);
    EXPECT_EQ(density("aluminium_6061"), 2700.0);
}

TEST(MassProperties, RobotMassFollowsJointRotation) {
    // A link hung off a joint with a rotated origin: its tensor turns into the root frame.
    Document document;
    document.robot = "r";
    Link root;
    root.name = "base";
    root.shapes = {sphere(0.01)};
    root.inertial.density = 1000.0;
    Link arm;
    arm.name = "arm";
    arm.shapes = {cylinder(0.01, 1.0)};
    arm.inertial.density = 1000.0;
    document.links = {root, arm};
    Joint joint;
    joint.name = "j";
    joint.type = Joint::Type::Fixed;
    joint.parent = "base";
    joint.child = "arm";
    joint.origin.xyz = {0, 0, 2.0};
    joint.origin.rpy = {0, kPi / 2.0, 0};  // the rod along z now lies along x
    document.joints = {joint};
    const MassProperties total = robotMass(document);
    const MassProperties rodOwn = shapeMass(cylinder(0.01, 1.0), 1000.0);
    const MassProperties ball = shapeMass(sphere(0.01), 1000.0);
    EXPECT_NEAR(total.mass, rodOwn.mass + ball.mass, 1e-12);
    const double comZ = rodOwn.mass * 2.0 / total.mass;
    EXPECT_NEAR(total.com[2], comZ, 1e-12);
    // Ixx about the COM: the rod's axial moment (its axis is x now) plus both shifts along z;
    // Iyy: its moment across plus the same shifts; Izz: across, no shift.
    const double shiftRod = rodOwn.mass * (2.0 - comZ) * (2.0 - comZ);
    const double shiftBall = ball.mass * comZ * comZ;
    EXPECT_NEAR(total.inertia[0], rodOwn.inertia[5] + ball.inertia[0] + shiftRod + shiftBall,
                1e-12);
    EXPECT_NEAR(total.inertia[3], rodOwn.inertia[0] + ball.inertia[3] + shiftRod + shiftBall,
                1e-12);
    EXPECT_NEAR(total.inertia[5], rodOwn.inertia[0] + ball.inertia[5], 1e-12);
}

TEST(Materials, PresetsMatchTheThermalTable) {
    ASSERT_FALSE(densityPresets().empty());
    for (const DensityPreset& preset : densityPresets()) {
        EXPECT_GT(preset.density, 0.0) << preset.name;
        EXPECT_FALSE(preset.source.empty()) << preset.name;
    }
    // Plan §6: the same names and numbers as RFC 0012's thermal presets. Compared against the
    // source table where the source tree is at hand (not in a package build).
    std::ifstream file(LAGRANGE_THERMAL_MATERIALS_CPP);
    if (!file) {
        GTEST_SKIP() << "no source tree: " << LAGRANGE_THERMAL_MATERIALS_CPP;
    }
    std::stringstream text;
    text << file.rdbuf();
    for (const DensityPreset& preset : densityPresets()) {
        const std::string row = "{\"" + std::string(preset.name) + "\", {";
        const std::size_t at = text.str().find(row);
        ASSERT_NE(at, std::string::npos) << preset.name << " is not a thermal preset";
        // {k, ρ, c_p, ε, axis}: ρ is the second number.
        std::istringstream numbers(text.str().substr(at + row.size()));
        double k = 0.0;
        double rho = 0.0;
        char comma = 0;
        numbers >> k >> comma >> rho;
        EXPECT_EQ(rho, preset.density) << preset.name;
    }
}

}  // namespace
}  // namespace lagrange::viewer::builder
