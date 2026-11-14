// Profile checks (RFC 0005, roadmap 0.2-E5): the schema, and what this server can run.
#include "lagrange/server/Profile.hpp"
#include "lagrange/server/Server.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

std::string schema() {
    std::ifstream in(LAGRANGE_TEST_SCHEMA);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

ProfileContext context() {
    ProfileContext c;
    c.stage = "robots/amr.urdf";
    c.modules = {"rigid_body", "sensors"};
    c.substitutes = {{"rigid_body", {"fixed", "constant"}}, {"electrical", {"ideal"}},
                     {"thermal", {"ambient", "constant"}},  {"aerodynamics", {"zero", "constant"}},
                     {"weather", {"clear", "constant"}},    {"emi", {"zero", "constant"}},
                     {"radiation", {"zero", "constant"}},   {"sensors", {"ground_truth", "ideal"}}};
    return c;
}

/// The server's own profile, with `rigid` and `sensors` as the two domains' lines.
std::string profile(const std::string& rigid = "{enabled: true, backend: mujoco, dt: 0.001}",
                    const std::string& sensors = "{enabled: true}", const std::string& extra = "") {
    return "schema_version: 1\nstage: robots/amr.urdf\n" + extra +
           "domains:\n"
           "  rigid_body: " +
           rigid +
           "\n"
           "  electrical: {enabled: false, substitute: ideal}\n"
           "  thermal: {enabled: false, substitute: ambient}\n"
           "  aerodynamics: {enabled: false, substitute: zero}\n"
           "  weather: {enabled: false, substitute: clear}\n"
           "  emi: {enabled: false, substitute: zero}\n"
           "  radiation: {enabled: false, substitute: zero}\n"
           "  sensors: " +
           sensors + "\n";
}

bool hasIssue(const ProfileCheck& check, const std::string& pointer, bool error = true) {
    for (const ProfileIssue& issue : check.issues) {
        if (issue.pointer == pointer && issue.error == error) {
            return true;
        }
    }
    return false;
}

std::string issues(const ProfileCheck& check) {
    std::string out;
    for (const ProfileIssue& issue : check.issues) {
        out += (issue.error ? "error " : "warning ") + issue.pointer + ": " + issue.message + "\n";
    }
    return out;
}

TEST(Profile, TheServersOwnProfileIsValid) {
    const ProfileCheck check = checkProfile(profile(), schema(), context());
    EXPECT_TRUE(check.valid) << issues(check);
    ASSERT_TRUE(check.settings);
    EXPECT_TRUE(check.settings->substituted.empty());
    EXPECT_EQ(check.settings->rigidBodyDtNs, 1'000'000);
}

TEST(Profile, DisablingADomainWithAModuleSubstitutesIt) {
    const ProfileCheck check = checkProfile(profile("{enabled: false, substitute: fixed}",
                                                    "{enabled: false, substitute: ground_truth}"),
                                            schema(), context());
    EXPECT_TRUE(check.valid) << issues(check);
    ASSERT_TRUE(check.settings);
    EXPECT_EQ(check.settings->substituted, (std::set<std::string>{"rigid_body", "sensors"}));
}

TEST(Profile, RefusesWhatThisServerCannotRun) {
    // A domain with no module here, enabled.
    std::string text = profile();
    text.replace(text.find("electrical: {enabled: false, substitute: ideal}"),
                 std::string("electrical: {enabled: false, substitute: ideal}").size(),
                 "electrical: {enabled: true}");
    ProfileCheck check = checkProfile(text, schema(), context());
    EXPECT_FALSE(check.valid);
    EXPECT_TRUE(hasIssue(check, "/domains/electrical/enabled")) << issues(check);
    // A substitute that does not exist for the domain here.
    check = checkProfile(profile("{enabled: false, substitute: replay, source: run.mcap}"),
                         schema(), context());
    EXPECT_TRUE(hasIssue(check, "/domains/rigid_body/substitute")) << issues(check);
    // Another backend, and a step outside MuJoCo's range or not whole nanoseconds.
    check = checkProfile(profile("{enabled: true, backend: physx}"), schema(), context());
    EXPECT_TRUE(hasIssue(check, "/domains/rigid_body/backend")) << issues(check);
    check = checkProfile(profile("{enabled: true, dt: 0.05}"), schema(), context());
    EXPECT_TRUE(hasIssue(check, "/domains/rigid_body/dt")) << issues(check);
    check = checkProfile(profile("{enabled: true, dt: 0.0000000015}"), schema(), context());
    EXPECT_TRUE(hasIssue(check, "/domains/rigid_body/dt")) << issues(check);
    // Another stage.
    std::string other = profile();
    other.replace(other.find("robots/amr.urdf"), 15, "robots/arm.urdf");
    check = checkProfile(other, schema(), context());
    EXPECT_TRUE(hasIssue(check, "/stage")) << issues(check);
}

TEST(Profile, TheSchemaCatchesTyposAndMissingSubstitutes) {
    // unevaluatedProperties (JSON Schema 2020-12): an unknown key is an error.
    ProfileCheck check =
        checkProfile(profile("{enabled: true, backnd: mujoco}"), schema(), context());
    EXPECT_FALSE(check.valid);
    EXPECT_TRUE(hasIssue(check, "/domains/rigid_body/backnd")) << issues(check);
    // A disabled domain must name a substitute.
    check = checkProfile(profile("{enabled: false}"), schema(), context());
    EXPECT_FALSE(check.valid) << issues(check);
}

TEST(Profile, AppliesTheRealTimeFactorAndWarnsAboutWhatItIgnores) {
    const ProfileCheck check = checkProfile(
        profile("{enabled: true}", "{enabled: true}",
                "time: {real_time_factor: 2.5}\nseed: 7\nrecord: {format: mcap, path: run.mcap}\n"),
        schema(), context());
    EXPECT_TRUE(check.valid) << issues(check);
    ASSERT_TRUE(check.settings);
    EXPECT_EQ(check.settings->realTimeFactor, 2.5);
    EXPECT_TRUE(hasIssue(check, "/seed", false)) << issues(check);
    EXPECT_TRUE(hasIssue(check, "/record", false)) << issues(check);
}

TEST(Profile, PassesTheRotorSettingsToTheRotors) {
    ProfileContext c = context();
    c.modules.insert("rotor_aero");
    const ProfileCheck check = checkProfile(
        profile() +
            "  rotor_aero: {enabled: true, fidelity: bem, air_density: 1.225, "
            "speed_of_sound: 340.3, ground_effect: false, inflow_time_constant: 0.02}\n",
        schema(), c);
    EXPECT_TRUE(check.valid) << issues(check);
    ASSERT_TRUE(check.settings);
    EXPECT_EQ(check.settings->airDensityKgM3, 1.225);
    const auto& r = check.settings->rotorAero;
    EXPECT_EQ(std::get<std::string>(r.at("fidelity")), "bem");
    EXPECT_EQ(std::get<double>(r.at("speed_of_sound")), 340.3);
    EXPECT_EQ(std::get<bool>(r.at("ground_effect")), false);
    EXPECT_EQ(std::get<double>(r.at("inflow_time_constant")), 0.02);
}

TEST(Profile, TheWeatherNeedsItsSite) {
    ProfileContext c = context();
    c.modules.insert("weather");
    const auto weather = [](const std::string& line) {
        std::string yaml = profile();
        yaml.replace(yaml.find("  weather: {enabled: false, substitute: clear}"),
                     std::string("  weather: {enabled: false, substitute: clear}").size(),
                     "  weather: " + line);
        return yaml;
    };
    const ProfileCheck missing =
        checkProfile(weather("{enabled: true, visibility: 100}"), schema(), c);
    EXPECT_FALSE(missing.valid);
    EXPECT_TRUE(hasIssue(missing, "/domains/weather/pressure")) << issues(missing);
    const ProfileCheck given = checkProfile(
        weather("{enabled: true, temperature: 283.15, pressure: 100000, relative_humidity: 0.9, "
                "visibility: 100, effects: [fog]}"),
        schema(), c);
    EXPECT_TRUE(given.valid) << issues(given);
    ASSERT_TRUE(given.settings);
    EXPECT_EQ(std::get<double>(given.settings->weather.at("visibility")), 100.0);
    EXPECT_EQ(std::get<std::vector<std::string>>(given.settings->weather.at("effects")),
              std::vector<std::string>{"fog"});
    // The site from the command line or a prim is enough.
    c.weatherKnown = {"temperature", "pressure", "relative_humidity"};
    EXPECT_TRUE(checkProfile(weather("{enabled: true, visibility: 100}"), schema(), c).valid);
}

TEST(Profile, ReportsBadYamlWithItsLine) {
    const ProfileCheck check = checkProfile("schema_version: 1\ndomains: [\n", schema(), context());
    ASSERT_FALSE(check.valid);
    ASSERT_FALSE(check.issues.empty());
    EXPECT_NE(check.issues[0].message.find("line"), std::string::npos) << issues(check);
}

// `lagrange validate`, `graph` and `run` (roadmap 0.2-E7) go through these: the example profile
// for the sample AMR is valid in the real server, wires, and applies.
TEST(Profile, TheSampleAmrExampleValidatesWiresAndApplies) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
    options.profileSchemaPath = LAGRANGE_TEST_SCHEMA;
    options.stageName = "robots/amr_diffdrive/amr_diffdrive.urdf";
    options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;  // the example enables electrical
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    ASSERT_TRUE(server.load().ok());  // once only

    std::ifstream in(LAGRANGE_TEST_PROFILES "/amr_sample_urdf.yaml");
    std::stringstream text;
    text << in.rdbuf();
    const std::string example = text.str();
    const ProfileCheck check = server.validateProfile(example);
    EXPECT_TRUE(check.valid) << issues(check);
    EXPECT_TRUE(check.issues.empty()) << issues(check);

    const std::string dot = server.wiringDot();
    EXPECT_NE(dot.find("rigid_body/mujoco"), std::string::npos) << dot;
    EXPECT_NE(dot.find("sensors/lidar_front_lidar"), std::string::npos) << dot;

    // Sensors off: ground-truth substitutes stand in for the lidars in the wiring.
    std::string substituted = example;
    const std::string on = "sensors:      { enabled: true }";
    ASSERT_NE(substituted.find(on), std::string::npos);
    substituted.replace(substituted.find(on), on.size(),
                        "sensors: { enabled: false, substitute: ground_truth }");
    ASSERT_TRUE(server.applyProfile(substituted).ok());
    // Before: no substitute ran, and electrical was the network.
    EXPECT_EQ(dot.find("substitute"), std::string::npos) << dot;
    EXPECT_NE(dot.find("electrical/averaged_network\\n100 µs"), std::string::npos) << dot;
    EXPECT_NE(server.wiringDot().find("sensors/lidar_front_lidar\\n100 ms\\nsubstitute"),
              std::string::npos)
        << server.wiringDot();

    // Another stage is refused.
    std::string other = example;
    other.replace(other.find("robots/amr_diffdrive"), 20, "robots/other_robot00");
    EXPECT_TRUE(hasIssue(server.validateProfile(other), "/stage"));
    EXPECT_FALSE(server.applyProfile(other).ok());
}

// QA 2026-11-12: `lagrange run quad_x500_downwash.yaml` said "the rotors do not run without the
// site's air density" although the profile gives it: the note came from the load before the
// profile. With profileFollows that load keeps its notes; the profile's run flies.
TEST(Profile, ALoadAProfileFollowsKeepsNotesTheProfileAnswers) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_QUAD_USDA;
    options.profileSchemaPath = LAGRANGE_TEST_SCHEMA;
    options.stageName = "robots/quad_x500/quad_x500.usda";
    std::ifstream in(LAGRANGE_TEST_PROFILES "/quad_x500_hover.yaml");
    std::stringstream text;
    text << in.rdbuf();

    {
        Server plain(options);
        ::testing::internal::CaptureStderr();
        ASSERT_TRUE(plain.load().ok());
        EXPECT_NE(::testing::internal::GetCapturedStderr().find("the rotors do not run"),
                  std::string::npos)
            << "without a profile the note stands";
    }
    options.profileFollows = true;
    Server server(options);
    ::testing::internal::CaptureStderr();
    const Status loaded = server.load();
    const Status applied = loaded.ok() ? server.applyProfile(text.str()) : loaded;
    const std::string said = ::testing::internal::GetCapturedStderr();
    ASSERT_TRUE(applied.ok()) << applied.message;
    EXPECT_EQ(said.find("the rotors do not run"), std::string::npos) << said;
    ASSERT_TRUE(server.stepUntil(SimTime{4'000'000'000}).ok());
    EXPECT_GT(server.read("body/base_link/pose").at(2), 1.0) << "the profile's air carries it";
}

// `lagrange determinism` (roadmap 0.2-H2) steps two servers headless: the state must move on
// with time, and be the same in both.
TEST(Determinism, TwoHeadlessServersStayIdenticalWhileTheRobotMoves) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
    options.lidar = ServerOptions::Lidar::Cpu;
    Server a(options);
    Server b(options);
    ASSERT_TRUE(a.load().ok());
    ASSERT_TRUE(b.load().ok());
    ASSERT_TRUE(a.stepUntil(SimTime{1'000'000'000}).ok());
    ASSERT_TRUE(b.stepUntil(SimTime{1'000'000'000}).ok());
    const std::uint64_t at1 = a.stateHash().value();
    EXPECT_EQ(at1, b.stateHash().value());
    ASSERT_TRUE(a.stepUntil(SimTime{2'000'000'000}).ok());
    ASSERT_TRUE(b.stepUntil(SimTime{2'000'000'000}).ok());
    const std::uint64_t at2 = a.stateHash().value();
    EXPECT_EQ(at2, b.stateHash().value());
    EXPECT_NE(at1, at2) << "the run advanced";
}

}  // namespace
}  // namespace lagrange::server
