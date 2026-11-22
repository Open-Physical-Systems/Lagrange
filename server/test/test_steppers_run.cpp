// RFC 0020 §4.5 in whole runs, on MuJoCo, with the sample gantry (two NEMA 17 steppers on
// chopper drivers, x on a GT2 belt, y on a T8 lead screw):
//   - joint control's profile moves both axes to their targets in the time a trapezoid at the
//     joints' velocity limits takes, and they stop on the step count to a microstep or two, with
//     no step lost;
//   - a beam too heavy for the motor's force at that acceleration falls out of step: steps are
//     lost in whole electrical periods (multiples of four full steps), with no fault raised, as a
//     real open-loop stepper loses position;
//   - the same run twice bit for bit.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

// Microsteps per metre (gantry.overlay.yaml): 50 teeth × 4 full steps a tooth × 16 microsteps per
// motor turn, times the turns per metre of the belt (1/0.04) and of the screw (1/0.008).
constexpr double kXPerMetre = 50.0 * 4.0 * 16.0 / 0.04;
constexpr double kYPerMetre = 50.0 * 4.0 * 16.0 / 0.008;

ServerOptions gantry(const std::string& robot = LAGRANGE_TEST_GANTRY_URDF) {
    ServerOptions o;
    o.robotPath = robot;
    o.overlayPath = LAGRANGE_TEST_GANTRY_OVERLAY;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    return o;
}

double at(Server& server, const std::string& signal) {
    return server.read(signal).at(0);
}

TEST(SteppersRun, JointControlMovesBothAxesToTheirTargetsByTheirSteps) {
    Server server(gantry());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{300'000'000}).ok());
    EXPECT_NEAR(at(server, "joint/x/position"), 0.0, 1.0 / kXPerMetre) << "holds where it started";
    ASSERT_TRUE(server.commandJoint("x", "position", 0.1));
    ASSERT_TRUE(server.commandJoint("y", "position", -0.03));
    // x: 0.1 m at 0.2 m/s, 0.25 s to speed and back: 0.75 s. y: 0.03 m at 0.04 m/s: also 1.0 s.
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 600'000'000}).ok());
    EXPECT_GT(at(server, "joint/x/position"), 0.05);
    EXPECT_LT(at(server, "joint/x/position"), 0.098) << "not there before the profile's time";
    ASSERT_TRUE(server.stepUntil(SimTime{server.simTime().ns + 900'000'000}).ok());
    const double x = at(server, "joint/x/position");
    const double y = at(server, "joint/y/position");
    std::printf("x at %.6f m (%.0f microsteps), y at %.6f m (%.0f microsteps)\n", x,
                at(server, "motor/x_motor/steps"), y, at(server, "motor/y_motor/steps"));
    EXPECT_EQ(at(server, "motor/x_motor/steps"), std::round(0.1 * kXPerMetre));
    EXPECT_EQ(at(server, "motor/y_motor/steps"), std::round(-0.03 * kYPerMetre));
    EXPECT_NEAR(x, 0.1, 2.0 / kXPerMetre);
    EXPECT_NEAR(y, -0.03, 2.0 / kYPerMetre);
    EXPECT_EQ(at(server, "motor/x_motor/lost_steps"), 0.0);
    EXPECT_EQ(at(server, "motor/y_motor/lost_steps"), 0.0);
    // At standstill the drivers hold at 1 A: two phase currents of a 1 A vector.
    const double ia = at(server, "motor/x_motor/current_a");
    const double ib = at(server, "motor/x_motor/current_b");
    EXPECT_NEAR(std::hypot(ia, ib), 1.0, 0.02);
}

TEST(SteppersRun, ABeamTooHeavyLosesWholeStepsWithoutAFault) {
    // The beam at 150 kg: 0.8 m/s² takes 120 N, past the 65 N (0.417 N·m × 157 rad/m) a 2 A
    // current vector gives on the belt.
    std::ifstream in(LAGRANGE_TEST_GANTRY_URDF);
    std::stringstream text;
    text << in.rdbuf();
    std::string urdf = text.str();
    const std::string light = "<mass value=\"1.0\"/>";
    const std::size_t at1 = urdf.find(light);
    ASSERT_NE(at1, std::string::npos);
    urdf.replace(at1, light.size(), "<mass value=\"150\"/>");
    const std::filesystem::path heavy = std::filesystem::temp_directory_path() /
                                        ("gantry_heavy_" + std::to_string(::getpid()) + ".urdf");
    std::ofstream(heavy) << urdf;
    Server server(gantry(heavy.string()));
    const Status loaded = server.load();
    std::filesystem::remove(heavy);
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.commandJoint("x", "position", 0.1));
    ASSERT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());
    const double lost = at(server, "motor/x_motor/lost_steps");
    std::printf("150 kg beam: %.0f full steps lost, at %.4f m of 0.1 m\n", lost,
                at(server, "joint/x/position"));
    EXPECT_GE(lost, 4.0);
    EXPECT_EQ(std::fmod(lost, 4.0), 0.0) << "whole electrical periods";
    EXPECT_EQ(at(server, "motor/x_motor/fault"), 0.0) << "an open-loop driver does not know";
    EXPECT_EQ(at(server, "motor/x_motor/steps"), std::round(0.1 * kXPerMetre));
    EXPECT_LT(at(server, "joint/x/position"), 0.1 - 4.0 / (50.0 * 4.0 / 0.04));
}

TEST(SteppersRun, SameRunTwiceIsBitIdentical) {
    std::uint64_t hashes[2] = {0, 0};
    for (std::uint64_t& hash : hashes) {
        Server server(gantry());
        ASSERT_TRUE(server.load().ok());
        ASSERT_TRUE(server.commandJoint("x", "position", -0.08));
        ASSERT_TRUE(server.commandJoint("y", "velocity", 0.02));
        ASSERT_TRUE(server.stepUntil(SimTime{800'000'000}).ok());
        hash = server.stateHash().value();
    }
    EXPECT_EQ(hashes[0], hashes[1]);
}

}  // namespace
}  // namespace lagrange::server
