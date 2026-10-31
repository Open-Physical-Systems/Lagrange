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

INSTANTIATE_TEST_SUITE_P(Backends, Conformance, ::testing::ValuesIn(backends()),
                         [](const ::testing::TestParamInfo<Backend>& param) {
                             return param.param.name;
                         });

}  // namespace
}  // namespace lagrange::server
