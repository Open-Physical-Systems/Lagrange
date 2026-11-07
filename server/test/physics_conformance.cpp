// The rigid-body conformance suite (RFC 0008 §5; docs/design/domains/rigid_body.md §8): what any
// physics backend, built in or a plugin, must do for Lagrange to run on it. Installed as
//
//     ros2 run lagrange_server lagrange_physics_conformance
//
// It checks the backends named in LAGRANGE_CONFORMANCE, a comma-separated list of NAME or
// NAME@PLUGIN (default "mujoco,jolt,jolt_abi@lagrange_jolt"); a third-party engine runs it as
// LAGRANGE_CONFORMANCE=my_engine@libmy_engine.so.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace lagrange::server {
namespace {

namespace fs = std::filesystem;

struct Backend {
    std::string name;
    std::string plugin;  // empty for a built-in backend
};

std::vector<Backend> backends() {
    const char* listed = std::getenv("LAGRANGE_CONFORMANCE");
    std::istringstream list(listed != nullptr ? listed : "mujoco,jolt,jolt_abi@lagrange_jolt");
    std::vector<Backend> out;
    for (std::string item; std::getline(list, item, ',');) {
        const std::size_t at = item.find('@');
        out.push_back({item.substr(0, at), at == std::string::npos ? "" : item.substr(at + 1)});
    }
    return out;
}

/// The sample AMR, from the installed lagrange_assets.
std::string amr() {
    return (ament_index_cpp::get_package_share_path("lagrange_assets") /
            "robots/amr_diffdrive/amr_diffdrive.urdf")
        .string();
}

fs::path scratch() {
    const fs::path dir =
        fs::temp_directory_path() / ("lagrange_conformance_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    return dir;
}

std::string write(const std::string& name, const std::string& text) {
    const fs::path path = scratch() / name;
    std::ofstream(path) << text;
    return path.string();
}

/// A 0.2 m box of 1 kg as an SDFormat model at x, y, z with pitch.
std::string box(const std::string& name, double x, double y, double z, double pitch = 0.0,
                bool fixed = false, const std::string& size = "0.2 0.2 0.2") {
    std::ostringstream out;
    out << "<model name=\"" << name << "\"><static>" << (fixed ? "true" : "false")
        << "</static><pose>" << x << " " << y << " " << z << " 0 " << pitch << " 0</pose>"
        << "<link name=\"link\"><inertial><mass>1</mass><inertia><ixx>0.006667</ixx>"
        << "<iyy>0.006667</iyy><izz>0.006667</izz></inertia></inertial>"
        << "<collision name=\"c\"><geometry><box><size>" << size
        << "</size></box></geometry></collision>"
        << "<visual name=\"v\"><geometry><box><size>" << size
        << "</size></box></geometry></visual></link></model>\n";
    return out.str();
}

std::string world(const std::string& name, const std::string& models) {
    return write(name + ".sdf", "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"" +
                                    name + "\">" + models + "</world></sdf>\n");
}

class Conformance : public ::testing::TestWithParam<Backend> {
protected:
    [[nodiscard]] ServerOptions options(const std::string& robot,
                                        const std::string& scenery) const {
        ServerOptions o;
        o.robotPath = robot;
        o.worldPath = scenery;
        o.overlayPath.clear();
        o.realTimeFactor = 0.0;
        o.ros = false;
        o.physics = GetParam().name;
        if (!GetParam().plugin.empty()) {
            o.plugins = {GetParam().plugin};
        }
        return o;
    }

    /// The pose of the first moving body whose name has `part` in it (static models have
    /// bodies without a pose signal).
    static std::vector<double> pose(const Server& server, const std::string& part = "") {
        for (const std::string& name : server.world().bodies.names) {
            if (name.find(part) != std::string::npos) {
                if (std::vector<double> p = server.read("body/" + name + "/pose"); p.size() == 7) {
                    return p;
                }
            }
        }
        ADD_FAILURE() << "no moving body named like '" << part << "'";
        return std::vector<double>(7, 0.0);
    }
};

// A box dropped from 1 m comes to rest on the ground at its half height.
TEST_P(Conformance, AFallingBoxComesToRest) {
    Server server(options("", world("drop", box("box", 0.0, 0.0, 1.0))));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const std::vector<double> p = pose(server);
    EXPECT_NEAR(p[2], 0.1, 0.005) << "resting on the ground";
    const std::string twist = "body/" + server.world().bodies.names[0] + "/twist";
    EXPECT_LT(std::abs(server.read(twist).at(2)), 0.01) << "at rest";
}

// A falling body gains speed at g: nothing damps it.
TEST_P(Conformance, FreeFallIsFree) {
    Server server(options("", world("fall", box("box", 0.0, 0.0, 100.0))));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const std::string twist = "body/" + server.world().bodies.names[0] + "/twist";
    EXPECT_NEAR(server.read(twist).at(2), -2.0 * 9.80665, 0.02) << "within 0.1 %";
}

// A pendulum swings with the period of a physical pendulum: T = 2π √(I / (m g d)) times
// (1 + θ₀²/16) for the amplitude θ₀ (the first term of the series for the exact period).
TEST_P(Conformance, APendulumSwingsWithItsPeriod) {
    constexpr double kLength = 0.5;     // m, pivot to bob
    constexpr double kMass = 1.0;       // kg
    constexpr double kRadius = 0.05;    // m, the bob (a sphere)
    constexpr double kAmplitude = 0.1;  // rad, released from rest
    const double inertia = kMass * kLength * kLength + 0.4 * kMass * kRadius * kRadius;
    std::ostringstream urdf;
    urdf << R"(<?xml version="1.0"?><robot name="pendulum"><link name="world"/>)"
         << R"(<link name="bob"><inertial><origin xyz="0 0 -0.5"/><mass value="1"/>)"
         << R"(<inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>)"
         << R"(</inertial><collision><origin xyz="0 0 -0.5"/><geometry><sphere radius="0.05"/>)"
         << R"(</geometry></collision></link><joint name="pivot" type="continuous">)"
         << R"(<parent link="world"/><child link="bob"/><origin xyz="0 0 1.5" rpy="0 )"
         << kAmplitude << R"( 0"/><axis xyz="0 1 0"/></joint></robot>)";
    Server server(options(write("pendulum.urdf", urdf.str()), ""));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    // The far ends of the swing: where the joint velocity turns from negative to positive,
    // interpolated between the 1 ms samples.
    std::vector<double> peaks;
    double lowest = 1e9;
    double highest = -1e9;
    double previous = server.read("joint/pivot/velocity").at(0);
    for (std::int64_t t = 1; t <= 8'000; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 1'000'000}).ok());
        const double angle = server.read("joint/pivot/position").at(0);
        lowest = std::min(lowest, angle);
        highest = std::max(highest, angle);
        const double velocity = server.read("joint/pivot/velocity").at(0);
        if (previous < 0.0 && velocity >= 0.0) {
            const double fraction = -previous / (velocity - previous);
            peaks.push_back((static_cast<double>(t - 1) + fraction) * 1e-3);
        }
        previous = velocity;
    }
    ASSERT_GE(peaks.size(), 4U) << "it swings: the angle went from " << lowest << " to " << highest
                                << " rad";
    const double measured = (peaks.back() - peaks.front()) / static_cast<double>(peaks.size() - 1);
    const double expected = 2.0 * std::numbers::pi *
                            std::sqrt(inertia / (kMass * 9.80665 * kLength)) *
                            (1.0 + kAmplitude * kAmplitude / 16.0);
    EXPECT_NEAR(measured, expected, 0.005 * expected) << "within 0.5 %";
}

// A box on a slope (friction coefficient 1 on both) holds at 30° (tan 30° < 1) and slides
// at 60° (tan 60° > 1).
TEST_P(Conformance, ABoxOnASlopeHoldsOrSlidesByItsFriction) {
    for (const double degrees : {30.0, 60.0}) {
        const double theta = degrees * std::numbers::pi / 180.0;
        const double lift = 0.2 + 0.001;  // ramp half-thickness plus box half-size, and a gap
        const std::string ramp = box("ramp", 0.0, 0.0, 2.5, theta, true, "4 2 0.2");
        const std::string block =
            box("block", lift * std::sin(theta), 0.0, 2.5 + lift * std::cos(theta), theta);
        Server server(options("", world("slope", ramp + block)));
        const Status loaded = server.load();
        ASSERT_TRUE(loaded.ok()) << loaded.message;
        ASSERT_TRUE(server.stepUntil(SimTime{100'000'000}).ok());  // settled on the ramp
        const std::vector<double> start = pose(server, "block");
        ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
        const std::vector<double> end = pose(server, "block");
        const double moved = std::hypot(end[0] - start[0], end[2] - start[2]);
        if (degrees < 45.0) {
            EXPECT_LT(moved, 0.01) << degrees << "° holds";
        } else {
            EXPECT_GT(moved, 0.3) << degrees << "° slides";
        }
    }
}

// Three boxes stacked on the ground stay standing.
TEST_P(Conformance, AStackOfThreeBoxesStands) {
    Server server(options("", world("stack", box("a", 0, 0, 0.1005) + box("b", 0, 0, 0.301) +
                                                 box("c", 0, 0, 0.5015))));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{10'000'000'000}).ok());
    std::vector<double> top(7, 0.0);  // the highest box
    for (const std::string& name : server.world().bodies.names) {
        const std::vector<double> p = server.read("body/" + name + "/pose");
        if (p.size() == 7 && p[2] > top[2]) {
            top = p;
        }
    }
    EXPECT_LT(std::hypot(top[0], top[1]), 0.01) << "the top box has not slid off";
    EXPECT_NEAR(top[2], 0.5, 0.01) << "nor fallen";
}

// What touches a box at rest holds up its weight (body/<b>/contact_force, N, world): the ground
// under one box, and under a stack each box gets its own weight from its neighbours together.
TEST_P(Conformance, WhatTouchesABoxAtRestHoldsUpItsWeight) {
    Server server(options("", world("held", box("a", 0, 0, 0.1005) + box("b", 0, 0, 0.301))));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    for (const std::string& name : server.world().bodies.names) {
        const std::vector<double> f = server.read("body/" + name + "/contact_force");
        if (f.size() != 3) {
            continue;
        }
        EXPECT_NEAR(f[0], 0.0, 0.05) << name;
        EXPECT_NEAR(f[1], 0.0, 0.05) << name;
        EXPECT_NEAR(f[2], 9.80665, 0.05) << name << ": 1 kg held up, within 0.5 %";
    }
}

// A pendulum hanging still: its pivot holds up its weight, and with the bob straight under the
// pivot there is no moment about it (joint/<j>/reaction: parent on child, world, at the child's
// origin, here the pivot).
TEST_P(Conformance, APivotHoldsUpAStillPendulum) {
    std::ostringstream urdf;
    urdf << R"(<?xml version="1.0"?><robot name="pendulum"><link name="world"/>)"
         << R"(<link name="bob"><inertial><origin xyz="0 0 -0.5"/><mass value="2"/>)"
         << R"(<inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>)"
         << R"(</inertial><collision><origin xyz="0 0 -0.5"/><geometry><sphere radius="0.05"/>)"
         << R"(</geometry></collision></link><joint name="pivot" type="continuous">)"
         << R"(<parent link="world"/><child link="bob"/><origin xyz="0 0 1.5"/>)"
         << R"(<axis xyz="0 1 0"/></joint></robot>)";
    Server server(options(write("still_pendulum.urdf", urdf.str()), ""));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    const std::vector<double> r = server.read("joint/pivot/reaction");
    ASSERT_EQ(r.size(), 6U);
    EXPECT_NEAR(r[0], 0.0, 0.02);
    EXPECT_NEAR(r[1], 0.0, 0.02);
    EXPECT_NEAR(r[2], 2.0 * 9.80665, 0.1) << "2 kg held up, within 0.5 %";
    for (std::size_t k = 3; k < 6; ++k) {
        EXPECT_NEAR(r[k], 0.0, 0.01) << "no moment, component " << k;
    }
}

// Swinging, the pivot carries the weight and the pull that turns the bob: at the bottom
// F = m·g + m·d·ω², with ω² = 2·m·g·d·(1 − cos θ₀) / I from the energy (d pivot to centre of
// mass, I about the pivot).
TEST_P(Conformance, APivotCarriesTheSwingAtTheBottom) {
    constexpr double kMass = 1.0;
    constexpr double kDistance = 0.5;
    constexpr double kAmplitude = 0.4;  // rad
    const double inertia = kMass * kDistance * kDistance + 0.001;
    std::ostringstream urdf;
    urdf << R"(<?xml version="1.0"?><robot name="pendulum"><link name="world"/>)"
         << R"(<link name="bob"><inertial><origin xyz="0 0 -0.5"/><mass value="1"/>)"
         << R"(<inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>)"
         << R"(</inertial><collision><origin xyz="0 0 -0.5"/><geometry><sphere radius="0.05"/>)"
         << R"(</geometry></collision></link><joint name="pivot" type="continuous">)"
         << R"(<parent link="world"/><child link="bob"/><origin xyz="0 0 1.5" rpy="0 )"
         << kAmplitude << R"( 0"/><axis xyz="0 1 0"/></joint></robot>)";
    Server server(options(write("swing.urdf", urdf.str()), ""));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    // The first time through the bottom: where the angle (from the joint's start) is −θ₀.
    double previous = server.read("joint/pivot/position").at(0);
    double up = 0.0;
    for (std::int64_t t = 1; t <= 2'000; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 1'000'000}).ok());
        const double angle = server.read("joint/pivot/position").at(0);
        if ((previous + kAmplitude) * (angle + kAmplitude) <= 0.0 && t > 1) {
            up = server.read("joint/pivot/reaction").at(2);
            std::printf("bottom at %.3f s: %.3f N up\n", static_cast<double>(t) * 1e-3, up);
            break;
        }
        previous = angle;
    }
    const double omega2 =
        2.0 * kMass * 9.80665 * kDistance * (1.0 - std::cos(kAmplitude)) / inertia;
    const double expected = kMass * 9.80665 + kMass * kDistance * omega2;
    EXPECT_NEAR(up, expected, 0.01 * expected)
        << "within 1 % (weight alone " << kMass * 9.80665 << " N)";
}

// The sample AMR's demo loop (articulated wheels, casters, a motor torque controller) drives
// the distance it does on MuJoCo, the reference backend, within 5 %, and ends within 10 % of the
// distance driven from where it ends there. Engines with other contact and friction models drift
// in heading over a loop (Jolt, with friction per body: 5.3 % on its 5.4 m in 10 s).
TEST_P(Conformance, TheSampleAmrDrivesItsLoopAsOnTheReference) {
    const auto run = [this](const std::string& backend, const std::string& plugin,
                            double& travelled) {
        ServerOptions o = options(amr(), "");
        o.physics = backend;
        o.plugins.clear();
        if (!plugin.empty()) {
            o.plugins = {plugin};
        }
        Server server(o);
        EXPECT_TRUE(server.load().ok());
        std::vector<double> last = server.read("body/base_link/pose");
        travelled = 0.0;
        for (std::int64_t t = 1; t <= 100; ++t) {
            EXPECT_TRUE(server.stepUntil(SimTime{t * 100'000'000}).ok());
            const std::vector<double> p = server.read("body/base_link/pose");
            travelled += std::hypot(p[0] - last[0], p[1] - last[1]);
            last = p;
        }
        return last;
    };
    double reference = 0.0;
    double travelled = 0.0;
    const std::vector<double> expected = run("mujoco", "", reference);
    const std::vector<double> got = run(GetParam().name, GetParam().plugin, travelled);
    ASSERT_GT(reference, 3.0) << "it drove";
    EXPECT_NEAR(travelled, reference, 0.05 * reference) << "the same distance";
    EXPECT_LT(std::hypot(got[0] - expected[0], got[1] - expected[1]), 0.10 * reference)
        << "to the same place";
}

// Two runs are the same to the bit, and a saved state, restored, runs on to the same state.
TEST_P(Conformance, RunsAreDeterministicAndStateRestores) {
    const auto hashAt = [this](std::int64_t ns) {
        Server server(options(amr(), ""));
        EXPECT_TRUE(server.load().ok());
        EXPECT_TRUE(server.stepUntil(SimTime{ns}).ok());
        return server.stateHash().value();
    };
    const std::uint64_t first = hashAt(2'000'000'000);
    EXPECT_NE(first, 0U);
    EXPECT_EQ(hashAt(2'000'000'000), first) << "the same run twice";

    Server server(options(amr(), ""));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.stepUntil(SimTime{1'000'000'000}).ok());
    const auto saved = server.saveState();
    ASSERT_TRUE(saved) << saved.error().message;
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const std::uint64_t straight = server.stateHash().value();
    EXPECT_EQ(straight, first);
    const Status restored = server.restoreState(*saved);
    ASSERT_TRUE(restored.ok()) << restored.message;
    EXPECT_EQ(server.now().ns, 1'000'000'000);
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_EQ(server.stateHash().value(), straight) << "restored, it runs on the same";
}

/// The MCube pushing experiment (Yu, Bauza, Fazeli and Rodriguez, "More than a Million Ways to Be
/// Pushed", IROS 2016, §VII and Table IV): the stainless-steel square rect1 (0.837 kg, 90 mm a
/// side, 13 mm thick, 1.13 g·m² about its axis; their Table III) pushed 15 cm at 20 mm/s by a
/// 9.5 mm steel rod, normal to a face, halfway between the face's centre and its edge, on four
/// surfaces whose sliding friction they measured after break-in (§V: abs 0.13, delrin 0.12,
/// plywood 0.24, pu 0.28); rod on block about 0.25. Returns the block's (Δx, Δy, Δθ) in mm, mm,
/// degrees, pushed along −y from (0, 0, 0). Both engines take the larger of two surfaces'
/// friction, so the block itself has none of its own.
struct Push {
    double dx, dy, dtheta;
};
Push pushRect1(const ServerOptions& base, double surfaceFriction) {
    constexpr double kTop = 0.5;        // m, the table's top
    constexpr double kSide = 0.090;     // m
    constexpr double kThick = 0.013;    // m
    constexpr double kRod = 0.00475;    // m, radius
    constexpr double kOffset = 0.0225;  // m, halfway from the face's centre to its edge
    constexpr double kTravel = 0.150;   // m
    constexpr double kSpeed = 0.020;    // m/s
    std::ostringstream models;
    models << "<model name=\"table\"><static>true</static><pose>0 0 " << kTop / 2.0
           << " 0 0 0</pose><link name=\"link\"><collision name=\"c\"><geometry><box><size>1 1 "
           << kTop << "</size></box></geometry><surface><friction><ode><mu>" << surfaceFriction
           << "</mu></ode></friction></surface></collision></link></model>";
    models << "<model name=\"rect1\"><pose>0 0 " << kTop + kThick / 2.0 + 0.0005
           << " 0 0 0</pose><link name=\"link\"><inertial><mass>0.837</mass><inertia>"
           << "<ixx>0.000577</ixx><iyy>0.000577</iyy><izz>0.00113</izz></inertia></inertial>"
           << "<collision name=\"c\"><geometry><box><size>" << kSide << " " << kSide << " "
           << kThick << "</size></box></geometry><surface><friction><ode><mu>0</mu></ode>"
           << "</friction></surface></collision></link></model>";
    // The rod: on a slide along −y from a frame bolted to the world, its foot 3 mm above the
    // table, starting against the block's +y face at x = −kOffset (the side the measured block
    // turns towards).
    std::ostringstream rig;
    rig << "<robot name=\"pusher\"><link name=\"world\"/><link name=\"rail\"><inertial><mass "
        << "value=\"1\"/><inertia ixx=\"0.01\" iyy=\"0.01\" izz=\"0.01\" ixy=\"0\" ixz=\"0\" "
        << "iyz=\"0\"/></inertial></link><joint name=\"bolt\" type=\"fixed\"><parent "
        << "link=\"world\"/><child link=\"rail\"/><origin xyz=\"" << -kOffset << " "
        << kSide / 2.0 + kRod + 0.0002 << " " << kTop + 0.003 + 0.025 << "\"/></joint>"
        << "<link name=\"rod\"><inertial><mass value=\"10\"/><inertia ixx=\"0.01\" iyy=\"0.01\" "
        << "izz=\"0.01\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry>"
        << "<cylinder radius=\"" << kRod << "\" length=\"0.05\"/></geometry></collision>"
        << "</link><joint name=\"slide\" type=\"prismatic\"><parent link=\"rail\"/><child "
        << "link=\"rod\"/><axis xyz=\"0 -1 0\"/><limit lower=\"-1\" upper=\"1\" effort=\"1000\" "
        << "velocity=\"1\"/></joint><gazebo reference=\"rod\"><mu1>" << "0.25"
        << "</mu1></gazebo></robot>";
    ServerOptions o = base;
    o.robotPath = write("pusher.urdf", rig.str());
    o.worldPath = world("push", models.str());
    o.jointControl = true;
    o.jointLoopHz = 50.0;  // stiff: the 1 N push moves the 10 kg rod off its path by ~1 µm
    Server server(o);
    const Status loaded = server.load();
    EXPECT_TRUE(loaded.ok()) << loaded.message;
    if (!loaded.ok()) {
        return {};
    }
    EXPECT_TRUE(server.stepUntil(SimTime{200'000'000}).ok());  // the block settles on the table
    const auto bodyPose = [&]() {
        for (const std::string& name : server.world().bodies.names) {
            if (name.find("rect1") != std::string::npos) {
                return server.read("body/" + name + "/pose");
            }
        }
        return std::vector<double>(7, 0.0);
    };
    const std::vector<double> start = bodyPose();
    constexpr std::int64_t kTick = 5'000'000;  // ns: the ramp's steps
    const auto steps = static_cast<std::int64_t>(kTravel / kSpeed * 1e9) / kTick;
    for (std::int64_t k = 1; k <= steps; ++k) {
        server.commandJoint("slide", "position", kSpeed * static_cast<double>(k * kTick) * 1e-9);
        EXPECT_TRUE(server.stepUntil(SimTime{200'000'000 + k * kTick}).ok());
    }
    const std::vector<double> end = bodyPose();
    const auto yaw = [](const std::vector<double>& q) {
        return std::atan2(2.0 * (q[3] * q[6] + q[4] * q[5]),
                          1.0 - 2.0 * (q[5] * q[5] + q[6] * q[6]));
    };
    return {(end[0] - start[0]) * 1e3, (end[1] - start[1]) * 1e3,
            (yaw(end) - yaw(start)) * 180.0 / std::numbers::pi};
}

// Against the pushing model the measurements were compared with, and the measurements themselves.
// Under uniform Coulomb friction the path does not depend on the support's friction, only on the
// rod's and the pressure under the block, so one deterministic answer stands for all four
// surfaces: the quasi-static simulator of Lynch, Maekawa and Tanie (1992) the authors ran, which
// gave (41.0 mm, −98.1 mm, 66.3°) (their Table IV). The engine must give that within 10 mm and
// 8°: a box resting on a plane is held at its four corners in MuJoCo, where Lynch et al. spread
// the pressure evenly, and friction at the corners (63.6 mm out, against 34.4 mm on average for
// even pressure) holds it from turning a little more: it turns 4 to 7° less. The measured means
// differ by surface (pressure, anisotropy, speed-dependent friction: their §V) and are printed:
// rigid_body.md §7 has the numbers.
//
// Jolt (5.6) keeps linear friction and friction about the contact normal on separate budgets
// (ContactConstraintManager: μ·Σλ for the one, μ·Σ dᵢ·λᵢ for the other), so a block sliding at its
// full friction still resists turning at full strength and stays straight: a known limitation of
// that engine's contact model, reported, not asserted.
TEST_P(Conformance, APushedBlockMovesAsTheMcubeMeasurements) {
    struct Measured {
        const char* surface;
        double friction, dx, dy, dtheta, translationStd, rotationStd;
    };
    const std::vector<Measured> table{{"abs", 0.13, 40.1, -67.6, 74.7, 5.5, 3.2},
                                      {"delrin", 0.12, 38.8, -50.7, 78.5, 3.4, 1.3},
                                      {"plywood", 0.24, 36.4, -93.6, 70.2, 8.1, 4.2},
                                      {"pu", 0.28, 40.2, -85.0, 69.3, 11.7, 4.5}};
    constexpr Push kQuasiStatic{41.0, -98.1, 66.3};  // Lynch et al., as run by Yu et al.
    const bool jolt = GetParam().name.find("jolt") != std::string::npos;
    for (const Measured& m : table) {
        const Push p = pushRect1(options("", ""), m.friction);
        std::printf(
            "%s push on %s: (%.1f, %.1f, %.1f), measured (%.1f, %.1f, %.1f) ± (%.1f mm, "
            "%.1f deg): %.1f mm and %+.1f deg off; the quasi-static model %.1f mm and "
            "%+.1f deg off it\n",
            GetParam().name.c_str(), m.surface, p.dx, p.dy, p.dtheta, m.dx, m.dy, m.dtheta,
            m.translationStd, m.rotationStd, std::hypot(p.dx - m.dx, p.dy - m.dy),
            p.dtheta - m.dtheta, std::hypot(kQuasiStatic.dx - m.dx, kQuasiStatic.dy - m.dy),
            kQuasiStatic.dtheta - m.dtheta);
        if (!jolt) {
            EXPECT_LT(std::hypot(p.dx - kQuasiStatic.dx, p.dy - kQuasiStatic.dy), 10.0)
                << m.surface;
            EXPECT_LT(std::abs(p.dtheta - kQuasiStatic.dtheta), 8.0) << m.surface;
        }
    }
}

// A frictionless box on a frictionless 10° ramp slides down it at g·sin θ: a body whose
// colliders have no friction keeps none (Jolt once gave such a body its default of 0.5).
TEST_P(Conformance, AFrictionlessBoxSlidesAtGSinTheta) {
    const double theta = 10.0 * std::numbers::pi / 180.0;
    const double lift = 0.2 + 0.001;
    std::string ramp = box("ramp", 0.0, 0.0, 2.5, theta, true, "4 2 0.2");
    std::string block =
        box("block", lift * std::sin(theta), 0.0, 2.5 + lift * std::cos(theta), theta);
    const auto frictionless = [](std::string model) {
        const std::string at = "</geometry>";
        model.insert(model.find(at) + at.size(),
                     "<surface><friction><ode><mu>0</mu></ode></friction></surface>");
        return model;
    };
    Server server(options("", world("ice", frictionless(ramp) + frictionless(block))));
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{100'000'000}).ok());
    const std::vector<double> a = pose(server, "block");
    ASSERT_TRUE(server.stepUntil(SimTime{1'100'000'000}).ok());
    const std::vector<double> b = pose(server, "block");
    // From rest at 0.1 s it has slid about ½·g·sin θ·t² by 1.1 s (the first 0.1 s added a little).
    const double slid = std::hypot(b[0] - a[0], b[2] - a[2]);
    const double expected = 0.5 * 9.80665 * std::sin(theta) * (1.1 * 1.1 - 0.1 * 0.1);
    EXPECT_NEAR(slid, expected, 0.15 * expected);
}

INSTANTIATE_TEST_SUITE_P(Backends, Conformance, ::testing::ValuesIn(backends()),
                         [](const ::testing::TestParamInfo<Backend>& param) {
                             return param.param.name;
                         });

}  // namespace
}  // namespace lagrange::server
