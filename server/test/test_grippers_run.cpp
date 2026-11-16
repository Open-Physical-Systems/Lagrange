// RFC 0020 §4.1 in whole runs, on MuJoCo: a suction cup holds by an equal and opposite pair of
// forces and the engine's contact, so
//   (a) a part hanging from a sealed cup lets go when its weight passes Δp·A (within 1 %),
//   (c) a part held on a cup facing sideways slides when its weight passes μ·Δp·A (within 3 %),
//   the sample arm with a suction tool picks a box off the pick station and puts it on the place
//   station, the same run twice gives the same state bit for bit, and with the domain off nothing
//   is held.
// The cup rides on a heavy slide held still by --joint-control; the run starts weightless
// (--site gravity=0) so that the cup seals and evacuates first, then gravity is switched on.
#include "lagrange/server/Server.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

constexpr double kG = 9.80665;
constexpr double kAtm = 101'325.0;
constexpr double kUltimate = 20'000.0;  // Pa absolute
constexpr double kDiameter = 0.03;      // m
// Δp·A of the rig's cup at full vacuum, N.
const double kHold = (kAtm - kUltimate) * std::numbers::pi * kDiameter * kDiameter / 4.0;

std::filesystem::path writeText(const std::string& name, const std::string& text) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                       ("lagrange_grip_" + std::to_string(::getpid()) + "_" + name);
    std::ofstream(path) << text;
    return path;
}

/// A cup on a 50 kg vertical slide 1 m up, facing along `axis`, its lip at the slide's origin.
std::string rig(const char* axis, const char* cupOrigin, const char* cupRpy) {
    return std::string(R"(<?xml version="1.0"?>
<robot name="rig">
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/><child link="base"/><origin xyz="0 0 1.0"/>
  </joint>
  <link name="base"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="carriage"/><axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100000" velocity="1"/>
  </joint>
  <link name="carriage">
    <inertial><mass value="50"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="carriage_to_cup" type="fixed">
    <parent link="carriage"/><child link="cup"/><origin xyz="0 0 0" rpy=")") +
           cupRpy + R"("/>
  </joint>
  <link name="cup">
    <inertial><mass value="0.02"/><inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin xyz=")" +
           cupOrigin + R"("/><geometry><box size="0.03 0.03 0.015"/></geometry></collision>
  </link>
  <gazebo reference="cup">
    <mu1>0.5</mu1>
    <suction_cup diameter="0.03" volume="1.0e-5" flow="1.0e-4" ultimate_pressure="20000" axis=")" +
           axis + R"("/>
  </gazebo>
</robot>
)";
}

/// A free 60 mm box of `mass` kg, placed by --add-robot.
std::string box(double mass, const char* surface = "") {
    const double i = mass * 0.06 * 0.06 / 6.0;
    char text[1200];
    std::snprintf(text, sizeof text, R"(<?xml version="1.0"?>
<robot name="part">
  <link name="part">
    <inertial><origin xyz="0 0 0"/><mass value="%.9g"/><inertia ixx="%.9g" iyy="%.9g" izz="%.9g" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.06 0.06 0.06"/></geometry></collision>
  </link>
  <gazebo reference="part"><mu1>0.5</mu1>%s</gazebo>
</robot>
)",
                  mass, i, i, i, surface);
    return text;
}

ServerOptions rigRun(const std::filesystem::path& robot, const std::filesystem::path& part,
                     const std::array<double, 3>& partAt) {
    ServerOptions o;
    o.robotPath = robot.string();
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    o.jointLoopHz = 50.0;
    o.site["gravity"] = 0.0;
    o.extraRobots.push_back({"part", part.string(), Pose{partAt, {1.0, 0.0, 0.0, 0.0}}});
    return o;
}

/// How far the part moves (m) over the second second after gravity comes on, the cup sealed and
/// evacuated first: about nothing while it holds, metres once it goes.
double dropAfterGravity(const std::string& robot, double mass, const std::array<double, 3>& at,
                        const std::string& tag) {
    const auto robotFile = writeText(tag + "_rig.urdf", robot);
    const auto partFile = writeText(tag + "_part.urdf", box(mass));
    Server server(rigRun(robotFile, partFile, at));
    const Status loaded = server.load();
    EXPECT_TRUE(loaded.ok()) << loaded.message;
    if (!loaded.ok()) {
        return 0.0;
    }
    EXPECT_TRUE(server.commandJoint("slide", "position", 0.0));
    EXPECT_TRUE(server.commandActuator("cup", 1.0));
    EXPECT_TRUE(server.stepUntil(SimTime{1'500'000'000}).ok());  // 15 τ of the cup
    EXPECT_EQ(server.read("act/cup/sealed").at(0), 1.0);
    const double held = server.read("act/cup/force").at(0);
    EXPECT_NEAR(held, kHold, 1e-4 * kHold);
    EXPECT_TRUE(server.setSite("gravity", kG).ok());
    // Settled on its contact after 0.5 s; how far it goes in the next second.
    EXPECT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const std::vector<double> before = server.read("body/part_part/pose");
    EXPECT_TRUE(server.stepUntil(SimTime{3'000'000'000}).ok());
    const std::vector<double> after = server.read("body/part_part/pose");
    return std::hypot(after[0] - before[0], after[1] - before[1], after[2] - before[2]);
}

TEST(GrippersRun, AHangingPartLetsGoAtDeltaPTimesA) {
    // The cup faces down; the part hangs below it, its top at the lip.
    const std::string down = rig("0 0 -1", "0 0 0.0075", "0 0 0");
    const double limit = kHold / kG;  // kg
    const double light = dropAfterGravity(down, 0.99 * limit, {0.0, 0.0, 0.94}, "a_light");
    const double heavy = dropAfterGravity(down, 1.01 * limit, {0.0, 0.0, 0.94}, "a_heavy");
    std::printf(
        "pull-off at Δp·A = %.3f N (%.3f kg): in a second 0.99 of it moves %.5f m, 1.01 "
        "%.4f m\n",
        kHold, limit, light, heavy);
    EXPECT_LT(light, 0.001);
    EXPECT_GT(heavy, 0.1);
}

TEST(GrippersRun, APorousPartFromItsUrdfLeaks) {
    // <sensor_material air_permeability=...> on the part: the cup settles where the generator's
    // flow meets the Darcy leak through the area it covers.
    constexpr double kKappa = 2.0e-6;  // m/(Pa·s)
    const auto robotFile = writeText("porous_rig.urdf", rig("0 0 -1", "0 0 0.0075", "0 0 0"));
    const auto partFile =
        writeText("porous_part.urdf", box(0.3, R"(<sensor_material air_permeability="2.0e-6"/>)"));
    Server server(rigRun(robotFile, partFile, {0.0, 0.0, 0.94}));
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.commandJoint("slide", "position", 0.0));
    ASSERT_TRUE(server.commandActuator("cup", 1.0));
    ASSERT_TRUE(server.stepUntil(SimTime{2'000'000'000}).ok());
    const double area = std::numbers::pi * kDiameter * kDiameter / 4.0;
    const double k = 1.0e-4 / (kAtm - kUltimate);
    const double leak = kAtm / (287.05287 * 293.15) * kKappa * area;
    const double expected = (k * kUltimate + leak * kAtm) / (k + leak);
    const double p = server.read("act/cup/pressure").at(0);
    std::printf("porous part: the cup at %.0f Pa (closed form %.0f Pa), holding %.2f N\n", p,
                expected, server.read("act/cup/force").at(0));
    EXPECT_NEAR(p, expected, 1e-3 * expected);
    EXPECT_EQ(server.read("act/cup/sealed").at(0), 1.0);
}

TEST(GrippersRun, APartHeldSidewaysSlidesAtMuDeltaPTimesA) {
    // The cup faces +x; the part beside it, held by friction μ = 0.5 against its weight.
    const std::string side = rig("0 0 1", "0 0 -0.0075", "0 1.5707963267948966 0");
    const double limit = 0.5 * kHold / kG;
    const double light = dropAfterGravity(side, 0.97 * limit, {0.03, 0.0, 0.97}, "c_light");
    const double heavy = dropAfterGravity(side, 1.03 * limit, {0.03, 0.0, 0.97}, "c_heavy");
    std::printf(
        "slip at μ·Δp·A = %.3f N (%.3f kg): in a second 0.97 of it moves %.5f m, 1.03 "
        "%.4f m\n",
        0.5 * kHold, limit, light, heavy);
    // MuJoCo's friction is soft: a part held at 97 % of the limit creeps a few mm/s (7 mm/s
    // measured), one 3 % over it slides away about a hundred times faster.
    EXPECT_LT(light, 0.02);
    EXPECT_GT(heavy, 0.2);
}

// The sample arm: joint angles for the cup's lip at (x, y, z) facing down, wrist_1 and wrist_3 at
// zero, by the planar two-link solution (arm6.urdf's lengths, the suction tool's 0.135 m).
struct ArmPose {
    double pan, lift, elbow, wrist2;
};
ArmPose reach(double x, double y, double z) {
    constexpr double kUpper = 0.4;                 // shoulder to elbow
    constexpr double kFore = 0.35 + 0.08;          // elbow to wrist_2
    constexpr double kHand = 0.08 + 0.04 + 0.135;  // wrist_2 to the lip
    constexpr double kShoulder = 0.2;              // shoulder height
    const double r = std::hypot(x, y);
    const double wx = r;
    const double wz = z + kHand - kShoulder;
    const double d2 = wx * wx + wz * wz;
    const double elbow = std::acos((d2 - kUpper * kUpper - kFore * kFore) / (2.0 * kUpper * kFore));
    const double lift =
        std::atan2(wx, wz) - std::atan2(kFore * std::sin(elbow), kUpper + kFore * std::cos(elbow));
    return {std::atan2(y, x), lift, elbow, std::numbers::pi - lift - elbow};
}

void command(Server& server, const ArmPose& p) {
    server.commandJoint("shoulder_pan_joint", "position", p.pan);
    server.commandJoint("shoulder_lift_joint", "position", p.lift);
    server.commandJoint("elbow_joint", "position", p.elbow);
    server.commandJoint("wrist_1_joint", "position", 0.0);
    server.commandJoint("wrist_2_joint", "position", p.wrist2);
    server.commandJoint("wrist_3_joint", "position", 0.0);
}

ServerOptions armRun() {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_ARM6_SUCTION_URDF;
    o.worldPath = LAGRANGE_TEST_WORKCELL_SUCTION_SDF;
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.jointControl = true;
    o.jointLoopHz = 8.0;
    return o;
}

std::string parcelPose(const Server& server) {
    for (const Server::SignalEntry& s : server.signals()) {
        if (s.name.starts_with("body/") && s.name.find("parcel") != std::string::npos &&
            s.name.ends_with("/pose")) {
            return s.name;
        }
    }
    return {};
}

/// The pick and place, stepped through; returns the parcel's centre at each phase's end.
std::vector<std::array<double, 3>> pickAndPlace(Server& server, bool checkHolds) {
    const std::string pose = parcelPose(server);
    EXPECT_FALSE(pose.empty());
    std::vector<std::array<double, 3>> at;
    std::int64_t t = 0;
    ArmPose from{};  // straight up
    // Each move goes in a straight line in joint space over the first 60 % of its phase, a
    // command every 20 ms, then holds.
    const auto phase = [&](const ArmPose& p, double seconds) {
        constexpr std::int64_t kTick = 20'000'000;  // ns
        const std::int64_t end = t + static_cast<std::int64_t>(seconds * 1e9);
        const auto moving = static_cast<std::int64_t>(0.6 * seconds * 1e9);
        for (std::int64_t k = kTick; t + kTick <= end; k += kTick) {
            const double s = std::min(1.0, static_cast<double>(k) / static_cast<double>(moving));
            command(server,
                    {from.pan + s * (p.pan - from.pan), from.lift + s * (p.lift - from.lift),
                     from.elbow + s * (p.elbow - from.elbow),
                     from.wrist2 + s * (p.wrist2 - from.wrist2)});
            t += kTick;
            EXPECT_TRUE(server.stepUntil(SimTime{t}).ok());
        }
        from = p;
        const std::vector<double> q = server.read(pose);
        at.push_back({q.at(0), q.at(1), q.at(2)});
    };
    // Folded high first, the tool down, so that it never sweeps the table on its way.
    phase({std::atan2(0.25, 0.55), 0.0, 1.2, 1.9}, 2.0);
    phase(reach(0.55, 0.25, 0.42), 2.0);  // above the parcel
    phase(reach(0.55, 0.25, 0.35), 1.5);  // the lip 1 cm into its top: pressed on
    if (checkHolds) {
        EXPECT_EQ(server.read("act/suction_cup/sealed").at(0), 1.0);
    }
    EXPECT_TRUE(server.commandActuator("suction_cup", 1.0));
    phase(reach(0.55, 0.25, 0.35), 1.0);
    if (checkHolds) {
        EXPECT_EQ(server.read("act/suction_cup/held").at(0), 1.0);
        EXPECT_LT(server.read("act/suction_cup/pressure").at(0), 25'000.0);
    }
    phase(reach(0.55, 0.25, 0.47), 3.0);   // lifted
    phase(reach(0.55, -0.25, 0.47), 5.0);  // carried over
    phase(reach(0.55, -0.25, 0.37), 2.0);  // set down
    server.commandActuator("suction_cup", 0.0);
    phase(reach(0.55, -0.25, 0.37), 1.0);
    phase(reach(0.55, -0.25, 0.47), 2.0);  // away
    return at;
}

TEST(GrippersRun, TheSuctionArmPicksAndPlacesTheParcel) {
    Server server(armRun());
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    const std::vector<std::array<double, 3>> at = pickAndPlace(server, true);
    ASSERT_EQ(at.size(), 9U);
    for (std::size_t i = 0; i < at.size(); ++i) {
        std::printf("phase %zu: parcel at %.3f %.3f %.3f\n", i, at[i][0], at[i][1], at[i][2]);
    }
    // On the pick station at the start (its centre 30 mm above the 0.30 m top).
    EXPECT_NEAR(at[2][0], 0.55, 0.005);
    EXPECT_NEAR(at[2][1], 0.25, 0.005);
    EXPECT_NEAR(at[2][2], 0.33, 0.005);
    // Lifted with the cup: about 0.12 m up.
    EXPECT_GT(at[4][2], 0.40);
    // Carried over the place station.
    EXPECT_NEAR(at[5][1], -0.25, 0.03);
    EXPECT_GT(at[5][2], 0.40);
    // Let go on it, left there when the arm rises.
    EXPECT_NEAR(at[8][0], 0.55, 0.03);
    EXPECT_NEAR(at[8][1], -0.25, 0.03);
    EXPECT_NEAR(at[8][2], 0.33, 0.005);
    EXPECT_EQ(server.read("act/suction_cup/held").at(0), 0.0);
}

TEST(GrippersRun, TheSameRunTwiceGivesTheSameState) {
    const auto once = [] {
        Server server(armRun());
        EXPECT_TRUE(server.load().ok());
        (void)pickAndPlace(server, false);
        const auto hash = server.stateHash();
        EXPECT_TRUE(hash);
        return hash ? *hash : 0;
    };
    const std::uint64_t a = once();
    const std::uint64_t b = once();
    EXPECT_EQ(a, b);
    EXPECT_NE(a, 0U);
}

TEST(GrippersRun, OffNothingIsHeld) {
    ServerOptions o = armRun();
    o.substituted.insert("actuators");
    Server server(o);
    ASSERT_TRUE(server.load().ok());
    const std::vector<std::array<double, 3>> at = pickAndPlace(server, false);
    EXPECT_EQ(server.read("act/suction_cup/force").at(0), 0.0);
    EXPECT_NEAR(at[4][2], 0.33, 0.005);  // stays on the table when the arm rises
}

}  // namespace
}  // namespace lagrange::server
