// The server end to end over a real Unix socket, as the viewer sees it.
#include "lagrange/proto/Connection.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <set>
#include <string>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

namespace v1 = lagrange::viewer::v1;

class ServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        char directory[] = "/tmp/lagrange-server-test-XXXXXX";
        ASSERT_NE(::mkdtemp(directory), nullptr);
        directory_ = directory;
        ServerOptions options;
        options.endpoint = proto::Endpoint::unixSocket(directory_ + "/s.sock");
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.profileSchemaPath = LAGRANGE_TEST_SCHEMA;
        options.startPaused = true;
        adjust(options);
        server_ = std::make_unique<Server>(options);
        const Status started = server_->start();
        ASSERT_TRUE(started.ok()) << started.message;

        std::error_code ec;
        client_ = proto::Connection::startConnect(server_->endpoint(), ec);
        ASSERT_EQ(client_.state(), proto::Connection::State::Connected) << ec.message();
        v1::Envelope hello;
        hello.mutable_client_hello()->mutable_version()->set_major(0);
        hello.mutable_client_hello()->mutable_version()->set_minor(1);
        ASSERT_TRUE(client_.queue(hello));
    }

    /// For fixtures that start the server differently.
    virtual void adjust(ServerOptions& /*options*/) {}

    void TearDown() override {
        client_.close();
        server_.reset();
        std::filesystem::remove_all(directory_);
    }

    /// Runs server and client until `done` holds for a received message, or 5 s pass.
    bool pumpUntil(const std::function<bool(const v1::Envelope&)>& done) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            server_->runOnce(1);
            std::error_code ec;
            (void)client_.flush(ec);
            (void)client_.receive(ec);
            for (;;) {
                const proto::FrameReader::Result frame = client_.next();
                if (frame.status != proto::FrameReader::Status::Frame) {
                    break;
                }
                v1::Envelope envelope;
                EXPECT_TRUE(proto::parsePayload(frame.payload, envelope));
                received_.push_back(envelope);
                if (done(envelope)) {
                    return true;
                }
            }
        }
        return false;
    }

    void send(const v1::Envelope& envelope) { ASSERT_TRUE(client_.queue(envelope)); }

    v1::Envelope command(std::uint64_t id) {
        v1::Envelope envelope;
        envelope.mutable_command()->set_request_id(id);
        return envelope;
    }

    std::string directory_;
    std::unique_ptr<Server> server_;
    proto::Connection client_;
    std::vector<v1::Envelope> received_;
};

TEST_F(ServerTest, HandshakeSendsHelloProfileCatalogStatusAndLogs) {
    v1::SignalCatalog catalog;
    ASSERT_TRUE(pumpUntil([&](const v1::Envelope& e) {
        if (e.has_catalog()) {
            catalog = e.catalog();
        }
        return e.has_log() && e.log().source() == "server";
    }));
    ASSERT_TRUE(received_.front().has_server_hello());
    const v1::ServerHello& hello = received_.front().server_hello();
    EXPECT_EQ(hello.server_kind(), v1::SERVER_KIND_SIM);
    EXPECT_TRUE(hello.refusal().empty());
    EXPECT_TRUE(received_[1].has_profile_state());
    EXPECT_TRUE(received_[1].profile_state().domains(0).enabled());

    bool pose = false;
    for (const v1::SignalInfo& info : catalog.signals()) {
        if (info.name() == "body/base_link/pose") {
            pose = true;
            EXPECT_EQ(info.type(), v1::SIGNAL_TYPE_POSE);
            EXPECT_EQ(info.components(), 7U);
            EXPECT_EQ(info.writer(), "rigid_body/mujoco");
            EXPECT_EQ(info.native_dt_ns(), 1'000'000);
            EXPECT_EQ(info.unit().display(), "m");
            EXPECT_NE(info.prim(), 0xFFFFFFFFU);
        }
    }
    EXPECT_TRUE(pose);
}

TEST_F(ServerTest, RefusesAnotherProtocolMajor) {
    client_.close();
    std::error_code ec;
    client_ = proto::Connection::startConnect(server_->endpoint(), ec);
    v1::Envelope hello;
    hello.mutable_client_hello()->mutable_version()->set_major(7);
    send(hello);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_server_hello(); }));
    EXPECT_NE(received_.back().server_hello().refusal().find("protocol version mismatch"),
              std::string::npos);
}

TEST_F(ServerTest, SnapshotStepAndDeltas) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope request = command(1);
    request.mutable_command()->mutable_request_snapshot();
    send(request);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
    const v1::SceneSnapshot& snapshot = received_.back().snapshot();
    EXPECT_EQ(snapshot.sequence(), 1U);
    EXPECT_GE(snapshot.prims_size(), 17);
    EXPECT_EQ(snapshot.transforms().inline_().size(),
              static_cast<std::size_t>(snapshot.prims_size()) * 52U);
    EXPECT_EQ(snapshot.prims(0).name(), "amr_diffdrive");

    // Paused: a Step command advances exactly 10 ms and sends a delta on top of the snapshot.
    v1::Envelope step = command(2);
    step.mutable_command()->mutable_step()->set_duration_ns(10'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_delta(); }));
    const v1::SceneDelta& delta = received_.back().delta();
    EXPECT_EQ(delta.base_sequence(), 1U);
    EXPECT_EQ(delta.sequence(), 2U);
    EXPECT_EQ(delta.sim_time().ns(), 10'000'000);
    EXPECT_GT(delta.transform_prims_size(), 0);
    EXPECT_EQ(delta.transform_data().size(),
              static_cast<std::size_t>(delta.transform_prims_size()) * 40U);
    EXPECT_EQ(server_->simTime(), SimTime{10'000'000});
}

// Roadmap 0.2-E5: a profile from the viewer is checked, previewed and applied; the run
// restarts with the domain switched to its substitute.
TEST_F(ServerTest, AppliesAProfileThatSwitchesADomainOff) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    std::string yaml = received_.back().profile_state().yaml();
    const std::string on = "rigid_body: {enabled: true, backend: mujoco, dt: 0.001}";
    ASSERT_NE(yaml.find(on), std::string::npos) << yaml;
    yaml.replace(yaml.find(on), on.size(), "rigid_body: {enabled: false, substitute: fixed}");

    // Validate first: valid, and the preview wiring has the fixed robot.
    v1::Envelope check = command(1);
    check.mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    send(check);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 1;
    }));
    const v1::ProfileValidation& validation = received_.back().command_result().validation();
    EXPECT_TRUE(validation.valid());
    bool fixed = false;
    for (const v1::Wiring::Node& node : validation.wiring().nodes()) {
        fixed = fixed || (node.domain() == "rigid_body" && node.detail() == "fixed");
    }
    EXPECT_TRUE(fixed) << "the preview wires the substitute";

    // Apply with a restart: the profile state now has rigid_body substituted.
    v1::Envelope apply = command(2);
    apply.mutable_command()->mutable_apply_profile()->set_yaml(yaml);
    apply.mutable_command()->mutable_apply_profile()->set_restart(true);
    send(apply);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 2;
    }));
    EXPECT_TRUE(received_.back().command_result().ok())
        << received_.back().command_result().error();
    // The new profile state goes out before the command's result.
    const v1::ProfileState* state = nullptr;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            state = &e.profile_state();
        }
    }
    ASSERT_NE(state, nullptr);
    const v1::DomainState& rigid = state->domains(0);
    EXPECT_EQ(rigid.domain(), "rigid_body");
    EXPECT_EQ(rigid.active_module(), "fixed");
    EXPECT_TRUE(rigid.substituted());

    // An invalid profile is refused and changes nothing.
    v1::Envelope bad = command(3);
    bad.mutable_command()->mutable_apply_profile()->set_yaml(
        "schema_version: 1\nstage: x\ndomains: {}\n");
    bad.mutable_command()->mutable_apply_profile()->set_restart(true);
    send(bad);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 3;
    }));
    EXPECT_FALSE(received_.back().command_result().ok());
    EXPECT_NE(received_.back().command_result().error().find("profile not applied"),
              std::string::npos);
}

// `lagrange run PROFILE` names the stage as the profile does; the profile the server sends to
// the viewer must use that name too, or the viewer's own profile would not validate.
class ServerWithStageNameTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.stageName = "robots/amr_diffdrive/amr_diffdrive.urdf";
    }
};

TEST_F(ServerWithStageNameTest, ItsOwnProfileNamesTheStageAndValidates) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    const std::string yaml = received_.back().profile_state().yaml();
    EXPECT_NE(yaml.find("stage: robots/amr_diffdrive/amr_diffdrive.urdf\n"), std::string::npos)
        << yaml;
    v1::Envelope check = command(1);
    check.mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    send(check);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 1;
    }));
    EXPECT_TRUE(received_.back().command_result().validation().valid());
}

TEST_F(ServerTest, SubscribedSignalsArriveAsGapFreeRuns) {
    v1::SignalCatalog catalog;
    ASSERT_TRUE(pumpUntil([&](const v1::Envelope& e) {
        if (e.has_catalog()) {
            catalog = e.catalog();
            return true;
        }
        return false;
    }));
    std::uint32_t velocity = 0;
    for (const v1::SignalInfo& info : catalog.signals()) {
        if (info.name() == "joint/left_wheel_joint/velocity") {
            velocity = info.id();
        }
    }
    v1::Envelope subscribe;
    subscribe.mutable_subscribe()->set_catalog_generation(catalog.generation());
    v1::SignalSubscription& full = *subscribe.mutable_subscribe()->add_signals();
    full.set_signal(velocity);
    full.set_mode(v1::SignalSubscription::MODE_FULL);
    full.set_decimation(1);
    send(subscribe);
    v1::Envelope play = command(3);
    play.mutable_command()->mutable_play();
    send(play);

    std::uint64_t expected = 0;
    std::uint64_t samples = 0;
    ASSERT_TRUE(pumpUntil([&](const v1::Envelope& e) {
        if (!e.has_signal_block()) {
            return false;
        }
        for (const v1::SignalRun& run : e.signal_block().runs()) {
            EXPECT_EQ(run.signal(), velocity);
            EXPECT_EQ(run.dt_ns(), 1'000'000);
            EXPECT_EQ(run.samples().size(), run.sample_count() * 8U);
            if (samples > 0) {
                EXPECT_EQ(run.first_index(), expected) << "a gap in the samples";
            }
            expected = run.first_index() + run.sample_count();
            samples += run.sample_count();
        }
        return samples >= 200;  // 0.2 s of simulation
    }));
}

TEST_F(ServerTest, LidarFramesSeeTheWarehouseWalls) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_sensor()->set_sensor(
        "/amr_diffdrive/base_link/lidar_link/front_lidar");
    watch.mutable_command()->mutable_subscribe_sensor()->set_enabled(true);
    send(watch);
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(200'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_sensor_frame(); }));
    const v1::SensorFrame& frame = received_.back().sensor_frame();
    ASSERT_TRUE(frame.has_points());
    const v1::PointCloudData& cloud = frame.points();
    // 720 beams in a closed hall: every beam returns.
    EXPECT_EQ(cloud.count(), 720U);
    EXPECT_EQ(cloud.data().inline_().size(), cloud.count() * 16U);
    // Beam 360 looks straight ahead (+x): the east wall's face is at x = 9.0, the lidar at 0.2.
    float x = 0.0F;
    std::memcpy(&x, cloud.data().inline_().data() + 360 * 16, sizeof(float));
    EXPECT_NEAR(x, 8.8, 0.06);
}

/// The sample quadcopter at its site (air density, geo origin from the command line).
class QuadServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_QUAD_USDA;
        options.worldPath.clear();
        options.airDensityKgM3 = 1.225;
        options.geoOrigin = coords::Geodetic{47.397742 * std::numbers::pi / 180.0,
                                             8.545594 * std::numbers::pi / 180.0, 488.0};
    }
};

TEST_F(QuadServerTest, ItsPropellersAreDrawnAsBlades) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope request = command(1);
    request.mutable_command()->mutable_request_snapshot();
    send(request);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
    int propellers = 0;
    for (const v1::Prim& prim : received_.back().snapshot().prims()) {
        propellers += prim.name() == "propeller" ? 1 : 0;
    }
    EXPECT_EQ(propellers, 4);
}

// A profile from the viewer names no site: its preview keeps the running one's, so the geo
// sensors stay in the wiring (they used to drop out, with a warning, on every check).
TEST_F(QuadServerTest, AProfileWithoutTheSiteKeepsTheRunningSite) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    const std::string yaml = received_.back().profile_state().yaml();
    ASSERT_EQ(yaml.find("geo_origin"), std::string::npos) << yaml;
    v1::Envelope check = command(1);
    check.mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    send(check);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 1;
    }));
    const v1::ProfileValidation& validation = received_.back().command_result().validation();
    EXPECT_TRUE(validation.valid());
    bool gnss = false;
    bool rotors = false;
    for (const v1::Wiring::Node& node : validation.wiring().nodes()) {
        gnss = gnss || node.detail().starts_with("gnss");
        rotors = rotors || node.domain() == "rotor_aero";
    }
    EXPECT_TRUE(gnss);
    EXPECT_TRUE(rotors);
}

/// The quadcopter with EMI victims, at the power line.
class EmiServerTest : public QuadServerTest {
protected:
    void adjust(ServerOptions& options) override {
        QuadServerTest::adjust(options);
        options.robotPath = LAGRANGE_TEST_QUAD_EMI_USDA;
        options.worldPath = LAGRANGE_TEST_POWER_LINE_USDA;
    }
};

TEST_F(EmiServerTest, ThePowerLineDisturbsTheMagnetometerAndReachesTheViewer) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_environment(); }));
    const v1::EnvironmentState env = received_.back().environment();
    ASSERT_EQ(env.emi_sources_size(), 1);
    EXPECT_EQ(env.emi_sources(0).name(), "power_line_01");
    EXPECT_EQ(env.emi_sources(0).frequency_hz(), 60.0);
    bool flux = false;
    for (const auto& f : env.fields()) {
        flux = flux || (f.name() == "emi/flux_density" && f.available());
    }
    EXPECT_TRUE(flux);
    std::set<std::string> names;
    for (const v1::Envelope& e : received_) {
        if (e.has_catalog()) {
            for (const v1::SignalInfo& info : e.catalog().signals()) {
                names.insert(info.name());
            }
        }
    }
    EXPECT_TRUE(names.contains("emi/magnetometer/b_field"));
    EXPECT_TRUE(names.contains("emi/imu/accel_noise"));

    // The flux slice at 10 m (the conductor's height) peaks next to it: 400 A at a few metres
    // is tens of µT.
    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_field()->set_field("emi/flux_density");
    watch.mutable_command()->mutable_subscribe_field()->set_enabled(true);
    watch.mutable_command()->mutable_subscribe_field()->set_height_m(9.0);
    send(watch);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_field_slice(); }));
    const v1::FieldSlice& slice = received_.back().field_slice();
    EXPECT_EQ(slice.field(), "emi/flux_density");
    EXPECT_GT(slice.value_max(), 2e-5);  // within 4 m of 400 A: above 20 µT
    EXPECT_LT(slice.value_min(), 1e-5);
    // In the profile, and the profile validates.
    std::string yaml;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            yaml = e.profile_state().yaml();
        }
    }
    EXPECT_NE(yaml.find("emi: {enabled: true"), std::string::npos) << yaml;
    EXPECT_TRUE(server_->validateProfile(yaml).valid);
}

/// The sample AMR with its Wi-Fi radio in the warehouse: the access point is outside the east
/// wall.
class CommsServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_AMR_WIFI_USDA;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_USDA;
        options.demoDrive = false;
    }
};

TEST_F(CommsServerTest, TheLinkCrossesTheWallAndTheProfileSaysSo) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope step = command(4);
    step.mutable_command()->mutable_step()->set_duration_ns(100'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 4;
    }));
    // 12 m to the access point at 2.4 GHz, exponent 3 past 1 m, one 10 dB wall, 4 dB shadowing.
    const std::vector<double> pose = server_->read("body/base_link/pose");
    const double d = std::hypot(12.0 - pose.at(0), pose.at(1), 2.5 - (pose.at(2) + 0.4));
    const double lambda = 299792458.0 / 2.437e9;
    const double free = std::pow(4.0 * std::numbers::pi / lambda, 2) * std::pow(d, 3.0);
    const double dbWithWall =
        10.0 * std::log10(server_->read("link/wifi/rx_power").at(0) * free / 0.1);
    EXPECT_NEAR(dbWithWall, -10.0, 4.0 * 4.0);  // the wall, within 4 σ of shadowing
    EXPECT_EQ(server_->read("link/wifi/up").at(0), 1.0);
    EXPECT_EQ(server_->read("can/can0/state").at(0), 0.0);
    std::string yaml;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            yaml = e.profile_state().yaml();
        }
    }
    EXPECT_NE(yaml.find("comms: {enabled: true}"), std::string::npos) << yaml;
    EXPECT_TRUE(server_->validateProfile(yaml).valid);
}

/// The sample sedan in the city block, on its demo circle.
class SedanServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_SEDAN_USDA;
        options.worldPath = LAGRANGE_TEST_CITY_USDA;
    }
};

TEST_F(SedanServerTest, TheCarDrivesItsDemoCircleOnItsTyres) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    for (std::uint64_t id = 10; id < 22; ++id) {  // 12 s, a second at a time
        v1::Envelope step = command(id);
        step.mutable_command()->mutable_step()->set_duration_ns(1'000'000'000);
        send(step);
        ASSERT_TRUE(pumpUntil([id](const v1::Envelope& e) {
            return e.has_command_result() && e.command_result().request_id() == id;
        }));
    }
    EXPECT_NEAR(server_->read("vehicle/sedan/speed").at(0), 3.0, 0.2);
    const std::vector<double> pose = server_->read("body/base_link/pose");
    EXPECT_NEAR(pose.at(2), 0.316, 0.01);                // on its wheels, at ride height
    EXPECT_LT(std::hypot(pose.at(0), pose.at(1)), 6.0);  // on the circle round the junction
    double load = 0.0;
    for (const char* w : {"front_left", "front_right", "rear_left", "rear_right"}) {
        load += server_->read(std::string("wheel/") + w + "_spin/normal_force").at(0);
    }
    EXPECT_NEAR(load, 1490.0 * 9.80665, 0.1 * 1490.0 * 9.80665);  // the car's weight, turning
    std::string yaml;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            yaml = e.profile_state().yaml();
        }
    }
    EXPECT_NE(yaml.find("vehicle_dynamics: {enabled: true, tire_model: pacejka}"),
              std::string::npos)
        << yaml;
    EXPECT_TRUE(server_->validateProfile(yaml).valid);
}

/// The cut-in scenario with the sedan as its ego, on the scenario's own roads.
class ScenarioServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_SEDAN_USDA;
        options.worldPath.clear();
        options.scenarioPath = LAGRANGE_TEST_CUT_IN;
    }
};

TEST_F(ScenarioServerTest, TheCutInRunsAroundTheSedan) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    // The sedan starts where the scenario puts the ego: road 1, lane -2, s = 30.
    std::vector<double> pose = server_->read("body/base_link/pose");
    EXPECT_NEAR(pose.at(0), 42.0, 0.05);
    EXPECT_NEAR(pose.at(1), -5.25, 0.05);
    for (std::uint64_t id = 10; id < 23; ++id) {  // 13 s, a second at a time
        v1::Envelope step = command(id);
        step.mutable_command()->mutable_step()->set_duration_ns(1'000'000'000);
        send(step);
        ASSERT_TRUE(pumpUntil([id](const v1::Envelope& e) {
            return e.has_command_result() && e.command_result().request_id() == id;
        }));
    }
    EXPECT_EQ(server_->read("scenario/state").at(0), 1.0);
    const std::vector<double> target = server_->read("scenario/entities");
    EXPECT_EQ(target.at(7), -2.0);                      // in the sedan's lane
    EXPECT_GT(target.at(0), 42.0);                      // ahead of it
    pose = server_->read("body/scenario_Target/pose");  // the kinematic body follows
    EXPECT_NEAR(pose.at(0), target.at(0), 0.2);
    pose = server_->read("body/base_link/pose");
    EXPECT_NEAR(pose.at(0), 42.0, 0.1);  // the ego waited: nothing drives it but its stack
}

/// The sample AMR in fog and rain (--weather).
class WeatherServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.weather = {{"temperature", 283.15},
                           {"pressure", 100000.0},
                           {"relative_humidity", 0.95},
                           {"visibility", 150.0},
                           {"rain_rate", 2.778e-6}};
    }
};

TEST_F(WeatherServerTest, TheWeatherRunsAndItsProfileSaysSo) {
    v1::SignalCatalog catalog;
    ASSERT_TRUE(pumpUntil([&](const v1::Envelope& e) {
        if (e.has_catalog()) {
            catalog = e.catalog();
        }
        return e.has_profile_state() && e.has_catalog() ? true : !catalog.signals().empty();
    }));
    std::set<std::string> names;
    for (const v1::SignalInfo& info : catalog.signals()) {
        names.insert(info.name());
    }
    EXPECT_TRUE(names.contains("env/extinction/nir_905"));
    EXPECT_TRUE(names.contains("env/friction_scale"));
    EXPECT_TRUE(names.contains("sensor/front_lidar/contamination"));
    // The server's own profile (sent before the catalog) carries the weather, and it validates.
    std::string yaml;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            yaml = e.profile_state().yaml();
        }
    }
    EXPECT_NE(yaml.find("weather: {enabled: true"), std::string::npos) << yaml;
    EXPECT_NE(yaml.find("visibility: 150"), std::string::npos) << yaml;
    EXPECT_TRUE(server_->validateProfile(yaml).valid);
}

TEST_F(WeatherServerTest, TheEnvironmentAndItsFogSliceReachTheViewer) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_environment(); }));
    const v1::EnvironmentState env = received_.back().environment();
    EXPECT_FALSE(env.synthetic());
    EXPECT_DOUBLE_EQ(env.weather().fog_visibility_m(), 150.0);
    EXPECT_NEAR(env.weather().rain_mm_per_h(), 10.0, 1e-3);
    EXPECT_NEAR(env.ambient_temperature_k(), 283.15, 1e-9);
    std::map<std::string, bool> available;
    for (const auto& f : env.fields()) {
        available[f.name()] = f.available();
    }
    EXPECT_TRUE(available["weather/fog_density"]);
    EXPECT_FALSE(available["weather/wind"]);  // no wind in this run
    EXPECT_FALSE(available["emi/flux_density"]);

    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_field()->set_field("weather/fog_density");
    watch.mutable_command()->mutable_subscribe_field()->set_enabled(true);
    watch.mutable_command()->mutable_subscribe_field()->set_height_m(1.0);
    send(watch);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_field_slice(); }));
    const v1::FieldSlice& slice = received_.back().field_slice();
    EXPECT_EQ(slice.field(), "weather/fog_density");
    EXPECT_EQ(slice.components(), 1U);
    ASSERT_EQ(slice.values().inline_().size(),
              std::size_t{slice.width()} * slice.height() * sizeof(float));
    // Uniform fog of 150 m visibility, plus a little rain: Koschmieder's 3.912/V and more.
    EXPECT_GT(slice.value_min(), 3.912 / 150.0);
    EXPECT_NEAR(slice.value_max(), slice.value_min(), 1e-6);
}

/// The sample arm in its workcell: a wrist camera.
class ArmServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_ARM6_URDF;
        options.worldPath = LAGRANGE_TEST_WORKCELL_URDF;
    }
};

TEST_F(ArmServerTest, CameraFramesCarryTheWristCameraImage) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_sensor()->set_sensor(
        "/arm6/base_link/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/"
        "wrist_3_link/palm_link/wrist_camera");
    watch.mutable_command()->mutable_subscribe_sensor()->set_enabled(true);
    send(watch);
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(100'000'000);
    send(step);
    if (!pumpUntil([](const v1::Envelope& e) { return e.has_sensor_frame(); })) {
        // Without a Vulkan device the cameras cannot render; the log says so.
        GTEST_SKIP() << "no camera frame (no Vulkan device?)";
    }
    const v1::SensorFrame& frame = received_.back().sensor_frame();
    ASSERT_TRUE(frame.has_image());
    EXPECT_EQ(frame.image().width(), 640U);
    EXPECT_EQ(frame.image().height(), 480U);
    EXPECT_EQ(frame.image().encoding(), "rgba8");
    EXPECT_EQ(frame.image().data().inline_().size(), 640U * 480U * 4U);
}

/// The sample AMR with its power system: motors whose parameters can be edited.
class PoweredServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;
        options.ambientTemperatureK = 298.15;
        options.sessionPath = directory_ + "/session.usda";
    }

    std::uint32_t primNamed(const std::string& name) {
        v1::Envelope ask = command(99);
        ask.mutable_command()->mutable_request_snapshot();
        send(ask);
        EXPECT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
        for (const v1::Envelope& e : received_) {
            if (e.has_snapshot()) {
                for (const v1::Prim& prim : e.snapshot().prims()) {
                    if (prim.name() == name) {
                        return prim.id();
                    }
                }
            }
        }
        return 0xFFFF'FFFFU;
    }

    v1::CommandResult run(v1::Envelope command) {
        const std::uint64_t id = command.command().request_id();
        send(command);
        EXPECT_TRUE(pumpUntil([id](const v1::Envelope& e) {
            return e.has_command_result() && e.command_result().request_id() == id;
        }));
        return received_.back().command_result();
    }

    void stepFor(std::int64_t ns, std::uint64_t id) {
        v1::Envelope step = command(id);
        step.mutable_command()->mutable_step()->set_duration_ns(ns);
        EXPECT_TRUE(run(step).ok());
    }
};

TEST_F(PoweredServerTest, AnEditChangesAMotorParameterInTheRunningSimulation) {
    // E6: the inspector reads the motor's parameters, edits one, and the run goes on with it.
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    const std::uint32_t motor = primNamed("left_drive_motor");
    ASSERT_NE(motor, 0xFFFF'FFFFU);
    v1::Envelope get = command(10);
    get.mutable_command()->mutable_get_attributes()->add_prims(motor);
    const v1::CommandResult listed = run(get);
    ASSERT_TRUE(listed.ok()) << listed.error();
    bool found = false;
    for (const v1::Attribute& a : listed.attributes().items(0).attributes()) {
        if (a.name() == "elec:windingResistance") {
            found = true;
            EXPECT_DOUBLE_EQ(a.real(), 0.12);
            EXPECT_TRUE(a.editable());
            EXPECT_EQ(a.unit().mass(), 1);  // ohm: kg·m²·s⁻³·A⁻²
            EXPECT_EQ(a.unit().current(), -2);
            EXPECT_FALSE(a.doc().empty());
        }
        if (a.name() == "elec:joint") {
            EXPECT_EQ(a.targets().prims_size(), 1);  // a relationship, to the wheel's prim
        }
    }
    ASSERT_TRUE(found);

    stepFor(2'000'000'000, 11);  // the demo drive at speed
    const double lossBefore = server_->read("motor/left_drive_motor/loss_power").at(0);
    const std::int64_t before = server_->simTime().ns;

    v1::Envelope set = command(12);
    v1::SetAttribute& edit = *set.mutable_command()->mutable_set_attribute();
    edit.set_prim(motor);
    edit.mutable_attribute()->set_name("elec:windingResistance");
    edit.mutable_attribute()->set_real(1.2);
    edit.set_save_to_session_layer(true);
    const v1::CommandResult applied = run(set);
    ASSERT_TRUE(applied.ok()) << applied.error();
    EXPECT_EQ(server_->simTime().ns, before) << "the run goes on, not restarted";

    stepFor(500'000'000, 13);
    const double lossAfter = server_->read("motor/left_drive_motor/loss_power").at(0);
    RecordProperty("loss_before_W", std::to_string(lossBefore));
    RecordProperty("loss_after_W", std::to_string(lossAfter));
    EXPECT_GT(lossAfter, 3.0 * lossBefore) << "ten times the resistance, more copper loss";

    // Read back, and from the session layer.
    const v1::CommandResult again = run(get);
    for (const v1::Attribute& a : again.attributes().items(0).attributes()) {
        if (a.name() == "elec:windingResistance") {
            EXPECT_DOUBLE_EQ(a.real(), 1.2);
        }
    }
    auto saved = stage::loadStage(directory_ + "/session.usda");
    ASSERT_TRUE(saved) << saved.error();
    bool savedValue = false;
    for (const SchemaInstance& s : saved->world.schemaInstances) {
        if (s.schema == "ElectricalMotorAPI_1" &&
            saved->world.prims.paths[s.prim.index].ends_with("left_drive_motor")) {
            savedValue = s.real("elec:windingResistance") == 1.2;
        }
    }
    EXPECT_TRUE(savedValue);
}

TEST_F(PoweredServerTest, TheSnapshotCarriesTheCollisionShapes) {
    const std::uint32_t base = primNamed("base_link");  // asks for a snapshot
    const v1::SceneSnapshot* snapshot = nullptr;
    for (const v1::Envelope& e : received_) {
        if (e.has_snapshot()) {
            snapshot = &e.snapshot();
        }
    }
    ASSERT_NE(snapshot, nullptr);
    // The sample AMR's links and the warehouse's boxes; the ground plane is left out.
    ASSERT_GT(snapshot->colliders_size(), 5);
    bool onBase = false;
    for (const v1::Collider& c : snapshot->colliders()) {
        ASSERT_LT(c.mesh(), static_cast<std::uint32_t>(snapshot->meshes_size()));
        EXPECT_GT(snapshot->meshes(static_cast<int>(c.mesh())).index_count(), 0U);
        ASSERT_EQ(c.position_size(), 3);
        ASSERT_EQ(c.rotation_size(), 4);
        EXPECT_TRUE(c.prim() == 0xFFFFFFFFU ||
                    c.prim() < static_cast<std::uint32_t>(snapshot->prims_size()));
        onBase = onBase || c.prim() == base;
    }
    EXPECT_TRUE(onBase) << "the chassis has a collider that moves with it";
}

TEST_F(PoweredServerTest, TheSnapshotCarriesThePowerSystemForTheElectricalWorkspace) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    const v1::SignalCatalog catalog = [this] {
        for (const v1::Envelope& e : received_) {
            if (e.has_catalog()) {
                return e.catalog();
            }
        }
        return v1::SignalCatalog{};
    }();
    const std::uint32_t motor = primNamed("left_drive_motor");
    const std::uint32_t battery = primNamed("battery");
    const v1::SceneSnapshot* snapshot = nullptr;
    for (const v1::Envelope& e : received_) {
        if (e.has_snapshot()) {
            snapshot = &e.snapshot();
        }
    }
    ASSERT_NE(snapshot, nullptr);
    ASSERT_EQ(snapshot->nets_size(), 1);
    const v1::ElectricalNet& net = snapshot->nets(0);
    EXPECT_EQ(net.name(), "dc_48v");
    EXPECT_EQ(net.voltage_signal(), "bus/dc_48v/voltage");
    std::map<std::uint32_t, std::string> terminals;
    for (const v1::NetTerminal& t : net.terminals()) {
        terminals[t.prim()] = t.current_signal();
    }
    EXPECT_EQ(terminals.size(), 4U);  // the battery, two motors, the computer
    EXPECT_EQ(terminals[battery], "battery/battery/current");
    EXPECT_EQ(terminals[motor], "motor/left_drive_motor/bus_current");
    std::set<std::string> limits;
    for (const v1::NetLimit& limit : net.limits()) {
        limits.insert(limit.label());
    }
    EXPECT_EQ(limits, (std::set<std::string>{"OV trip", "UV trip", "battery current"}));
    // Every terminal's current is a signal, and the motors' signals belong to their prims (the
    // schematic shows a node's other values from them).
    bool motorSignal = false;
    for (const v1::SignalInfo& info : catalog.signals()) {
        motorSignal = motorSignal ||
                      (info.name() == "motor/left_drive_motor/current_q" && info.prim() == motor);
    }
    EXPECT_TRUE(motorSignal);
    for (const auto& [prim, signal] : terminals) {
        EXPECT_TRUE(std::ranges::any_of(catalog.signals(), [&](const v1::SignalInfo& info) {
            return info.name() == signal;
        })) << signal;
    }
}

TEST_F(PoweredServerTest, BadEditsAreRefusedAndChangeNothing) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    const std::uint32_t motor = primNamed("left_drive_motor");
    const auto edit = [&](const char* name, double value, std::uint64_t id) {
        v1::Envelope set = command(id);
        v1::SetAttribute& e = *set.mutable_command()->mutable_set_attribute();
        e.set_prim(motor);
        e.mutable_attribute()->set_name(name);
        e.mutable_attribute()->set_real(value);
        return run(set);
    };
    // The loader's check: a fraction.
    const v1::CommandResult fraction = edit("elec:gearEfficiency", 1.5, 20);
    EXPECT_FALSE(fraction.ok());
    EXPECT_NE(fraction.error().find("within 0 to 1"), std::string::npos) << fraction.error();
    // The module's check, at the rebuild: a motor that cannot carry current.
    const v1::CommandResult zero = edit("elec:windingResistance", 0.0, 21);
    EXPECT_FALSE(zero.ok());
    EXPECT_NE(zero.error().find("was not applied"), std::string::npos) << zero.error();
    const v1::CommandResult unknown = edit("elec:noSuchThing", 1.0, 22);
    EXPECT_FALSE(unknown.ok());
    v1::Envelope move = command(23);
    move.mutable_command()->mutable_set_transform()->set_prim(motor);
    EXPECT_FALSE(run(move).ok());
    // The run still works, with the old value.
    stepFor(100'000'000, 24);
    EXPECT_GT(server_->simTime().ns, 0);
}

/// Replay mode (E8): a run recorded once for the suite, replayed through the protocol.
class ReplayServerTest : public ServerTest {
protected:
    static void SetUpTestSuite() {
        const std::filesystem::path dir =
            std::filesystem::path(::testing::TempDir()) / "lagrange_replay";
        std::filesystem::create_directories(dir);
        file_ = (dir / "run.mcap").string();
        std::filesystem::remove(file_);
        ServerOptions options;
        options.endpoint = proto::Endpoint::unixSocket((dir / "record.sock").string());
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;
        options.ambientTemperatureK = 298.15;
        options.realTimeFactor = 0.0;
        options.ros = false;
        {
            ServerOptions recording = options;
            recording.recordPath = file_;
            Server server(recording);
            const Status started = server.start();
            ASSERT_TRUE(started.ok()) << started.message;
            while (server.simTime().ns < 2'200'000'000) {
                server.runOnce(0);
            }
        }  // closes the recording
        // The same run again (deterministic: no ROS, the CPU lidar), for the poses to compare.
        Server reference(options);
        ASSERT_TRUE(reference.load().ok());
        ASSERT_TRUE(reference.stepUntil(SimTime{500'000'000}).ok());
        at05_ = reference.read("body/base_link/pose");
        ASSERT_TRUE(reference.stepUntil(SimTime{1'500'000'000}).ok());
        at15_ = reference.read("body/base_link/pose");
    }

    void adjust(ServerOptions& options) override {
        options.replayPath = file_;
        options.ros = false;
    }

    void expectPose(const std::vector<double>& expected) const {
        const std::vector<double> pose = server_->read("body/base_link/pose");
        ASSERT_EQ(pose.size(), expected.size());
        for (std::size_t i = 0; i < pose.size(); ++i) {
            EXPECT_NEAR(pose[i], expected[i], 1e-12) << i;
        }
    }

    v1::CommandResult run(v1::Envelope command) {
        const std::uint64_t id = command.command().request_id();
        send(command);
        EXPECT_TRUE(pumpUntil([id](const v1::Envelope& e) {
            return e.has_command_result() && e.command_result().request_id() == id;
        }));
        return received_.back().command_result();
    }

    v1::CommandResult seekTo(std::int64_t ns, std::uint64_t id) {
        v1::Envelope seek = command(id);
        seek.mutable_command()->mutable_seek()->mutable_to()->set_ns(ns);
        return run(seek);
    }

    static inline std::string file_;
    static inline std::vector<double> at05_;
    static inline std::vector<double> at15_;
};

TEST_F(ReplayServerTest, TheRecordingReplaysSampleForSampleAndSeeks) {
    // The hello sends the catalog, then the status.
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    bool canSeek = false;
    bool info = false;
    std::int64_t end = 0;
    for (const v1::Envelope& e : received_) {
        canSeek = canSeek || (e.has_server_hello() && e.server_hello().capabilities().can_seek());
        info = info || e.has_recording_info();
        if (e.has_status()) {
            end = e.status().replay_end().ns();
        }
    }
    EXPECT_TRUE(canSeek);
    EXPECT_TRUE(info);
    EXPECT_GE(end, 2'200'000'000);
    // The recorded scene, its nets included.
    v1::Envelope ask = command(1);
    ask.mutable_command()->mutable_request_snapshot();
    send(ask);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
    EXPECT_GT(received_.back().snapshot().prims_size(), 20);
    EXPECT_EQ(received_.back().snapshot().nets_size(), 1);

    v1::Envelope step = command(2);
    step.mutable_command()->mutable_step()->set_duration_ns(1'500'000'000);
    ASSERT_TRUE(run(step).ok());
    expectPose(at15_);
    // Back, then forward again past a checkpoint: the same samples.
    ASSERT_TRUE(seekTo(500'000'000, 3).ok());
    EXPECT_EQ(server_->simTime().ns, 500'000'000);
    expectPose(at05_);
    ASSERT_TRUE(seekTo(1'500'000'000, 4).ok());
    expectPose(at15_);
}

/// A simulating server (without ROS) that the viewer asks to open the recording.
class OpenRecordingTest : public ReplayServerTest {
protected:
    void adjust(ServerOptions& options) override { options.ros = false; }
};

TEST_F(OpenRecordingTest, OpeningARecordingTurnsTheServerIntoItsReplay) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    v1::Envelope open = command(5);
    open.mutable_command()->mutable_open_recording()->set_path(file_);
    const v1::CommandResult opened = run(open);
    ASSERT_TRUE(opened.ok()) << opened.error();
    EXPECT_EQ(opened.imported_stage_path(), file_);
    ASSERT_TRUE(pumpUntil(
        [](const v1::Envelope& e) { return e.has_status() && e.status().replay_end().ns() > 0; }));
    ASSERT_TRUE(seekTo(1'500'000'000, 6).ok());
    expectPose(at15_);
    // A file that is not a recording is refused and the replay stays.
    v1::Envelope bad = command(7);
    bad.mutable_command()->mutable_open_recording()->set_path("/nonexistent.mcap");
    EXPECT_FALSE(run(bad).ok());
    ASSERT_TRUE(seekTo(500'000'000, 8).ok());
    expectPose(at05_);
}

TEST_F(ServerTest, SeekingASimulationIsRefused) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope seek = command(7);
    seek.mutable_command()->mutable_seek()->mutable_to()->set_ns(0);
    send(seek);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 7;
    }));
    EXPECT_FALSE(received_.back().command_result().ok());
    EXPECT_NE(received_.back().command_result().error().find("--replay"), std::string::npos);
}

// huge-worlds.md §1.4: the floating origin follows the robot during a run. The sample AMR
// starts 999.5 m east of the origin (no rebase at load: within 1000 m) and drives its demo
// figure-eight; once it is past 1000 m the server moves the region origin to its whole-metre
// position, between ticks, and the run goes on in the new frame without a jump.
TEST(FloatingOrigin, TheServerRebasesWhenTheRobotDrivesFar) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.realTimeFactor = 0.0;
    options.ros = false;
    options.spawn.position = {999.5, 0.0, 0.0};
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    EXPECT_EQ(server.world().region.rebaseCount, 0U);
    std::array<double, 3> last{};
    double largestStep = 0.0;
    for (std::int64_t t = 100'000'000; t <= 20'000'000'000; t += 100'000'000) {
        ASSERT_TRUE(server.stepUntil(SimTime{t}).ok());
        const std::vector<double> pose = server.read("body/base_link/pose");
        std::array<double, 3> world{};
        for (std::size_t i = 0; i < 3; ++i) {
            world.at(i) = pose.at(i) + server.world().region.worldEnu.at(i);
        }
        if (t > 100'000'000) {
            largestStep = std::max(largestStep, std::hypot(world[0] - last[0], world[1] - last[1]));
        }
        last = world;
    }
    ASSERT_GE(server.world().region.rebaseCount, 1U) << "the robot crossed 1000 m";
    EXPECT_EQ(server.world().region.worldEnu[1], 0.0);
    EXPECT_GE(server.world().region.worldEnu[0], 1000.0);
    EXPECT_LT(std::abs(server.read("body/base_link/pose").at(0)), 1000.0);
    // At most the demo's speed times 100 ms: no jump in world ENU across the rebase.
    EXPECT_LT(largestStep, 0.1);
}

// huge-worlds.md §6 (v0.7): a streamed world. The AMR starts in a street of the tiled town;
// the tiles around it are in physics before the first step, the far ones are not, and two
// headless runs load the same tiles at the same times (deterministic tile selection).
TEST(Streaming, TheTilesAroundTheRobotAreLoadedAndTheRunIsDeterministic) {
    const auto run = [](std::int64_t untilNs) {
        ServerOptions options;
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_TILED_TOWN;
        options.realTimeFactor = 0.0;
        options.ros = false;
        options.spawn.position = {1036.0, 1036.0, 0.0};  // a street: x and y in [1024, 1048]
        auto server = std::make_unique<Server>(options);
        EXPECT_TRUE(server->load().ok());
        EXPECT_TRUE(server->stepUntil(SimTime{untilNs}).ok());
        return server;
    };
    const auto a = run(5'000'000'000);
    const World& world = a->world();
    // The buildings of the tiles within 600 m: each fine tile holds up to 9 boxes.
    EXPECT_GT(world.colliders.body.size(), 100U);
    double farthest = 0.0;
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.shape[c] == ShapeType::Box) {
            const std::array<double, 3>& p = world.colliders.poseInBody[c].position;
            farthest = std::max(farthest, std::hypot(p[0] + world.region.worldEnu[0] - 1036.0,
                                                     p[1] + world.region.worldEnu[1] - 1036.0));
        }
    }
    EXPECT_LT(farthest, 600.0 + 256.0 * std::sqrt(2.0)) << "only tiles within the load radius";
    const auto b = run(5'000'000'000);
    const auto ha = a->stateHash();
    const auto hb = b->stateHash();
    ASSERT_TRUE(ha && hb);
    EXPECT_EQ(*ha, *hb);
    EXPECT_EQ(a->world().colliders.body.size(), b->world().colliders.body.size());
}

}  // namespace
}  // namespace lagrange::server
