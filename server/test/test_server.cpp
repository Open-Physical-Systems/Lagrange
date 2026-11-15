// The server end to end over a real Unix socket, as the viewer sees it.
#include "lagrange/gpu/Device.hpp"
#include "lagrange/proto/Connection.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <string>
#include <sys/stat.h>
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

// QA 2026-11-12: Step{ticks = 4e9} (46 days) ran to its end before the server read anything
// else, so one client froze it for every viewer and for SIGTERM. A step that long is refused.
TEST_F(ServerTest, RefusesAStepTooLongToRunAtOnce) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope huge = command(7);
    huge.mutable_command()->mutable_step()->set_ticks(4'000'000'000U);
    send(huge);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 7;
    }));
    EXPECT_FALSE(received_.back().command_result().ok());
    EXPECT_NE(received_.back().command_result().error().find("at most"), std::string::npos);
    EXPECT_EQ(server_->simTime(), SimTime{0});

    v1::Envelope longest = command(8);
    longest.mutable_command()->mutable_step()->set_duration_ns(
        std::numeric_limits<std::int64_t>::max());
    send(longest);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 8;
    }));
    EXPECT_FALSE(received_.back().command_result().ok());
    EXPECT_EQ(server_->simTime(), SimTime{0});
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

// RFC 0012: a link's and its heat sources' signals name the link's prim, so the inspector shows
// them with the link; the cells reach a viewer that asks for them, in the link's frame.
class HeatServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.robotPath = LAGRANGE_TEST_LED_HEATSINK_URDF;
        options.worldPath.clear();
        options.ambientTemperatureK = 298.15;
    }
};

TEST_F(HeatServerTest, SourcesAreOnTheirLinkAndTheCellsReachTheViewer) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope request = command(1);
    request.mutable_command()->mutable_request_snapshot();
    send(request);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
    std::uint32_t board = 0xFFFF'FFFFU;
    const v1::SceneSnapshot snapshot = received_.back().snapshot();
    for (int i = 0; i < snapshot.prims_size(); ++i) {
        if (snapshot.prims(i).name() == "board") {
            board = static_cast<std::uint32_t>(i);
        }
    }
    ASSERT_NE(board, 0xFFFF'FFFFU);
    std::map<std::string, std::uint32_t> prims;
    for (const v1::Envelope& e : received_) {
        if (e.has_catalog()) {
            for (const v1::SignalInfo& info : e.catalog().signals()) {
                prims[info.name()] = info.prim();
            }
        }
    }
    EXPECT_EQ(prims.at("thermal/board/max_temperature"), board);
    EXPECT_EQ(prims.at("thermal/led_left/temperature"), board);
    EXPECT_EQ(prims.at("thermal/driver/heat"), board);
    v1::Envelope watch = command(2);
    watch.mutable_command()->mutable_subscribe_field()->set_field("thermal/cells");
    watch.mutable_command()->mutable_subscribe_field()->set_enabled(true);
    watch.mutable_command()->mutable_subscribe_field()->set_volume(true);
    send(watch);
    v1::Envelope run = command(3);
    run.mutable_command()->mutable_play();
    send(run);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_field_volume() && e.field_volume().field() == "thermal/cells";
    }));
    const v1::FieldVolume& volume = received_.back().field_volume();
    EXPECT_FALSE(volume.prim_path().empty());
    EXPECT_EQ(volume.components(), 1U);
    EXPECT_EQ(volume.values().inline_().size(),
              static_cast<std::size_t>(volume.nx()) * volume.ny() * volume.nz() * sizeof(float));
}

TEST_F(EmiServerTest, ThePowerLineDisturbsTheMagnetometerAndReachesTheViewer) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_environment(); }));
    const v1::EnvironmentState env = received_.back().environment();
    ASSERT_EQ(env.emi_sources_size(), 1);
    EXPECT_EQ(env.emi_sources(0).name(), "power_line_01");
    EXPECT_EQ(env.emi_sources(0).frequency_hz(), 60.0);
    // Its scene prim, so the viewer shows it with the power line's prim, not by its name.
    v1::Envelope snap = command(90);
    snap.mutable_command()->mutable_request_snapshot();
    send(snap);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_snapshot(); }));
    std::uint32_t line = 0xFFFF'FFFFU;
    const v1::SceneSnapshot& scene = received_.back().snapshot();
    for (int i = 0; i < scene.prims_size(); ++i) {
        if (scene.prims(i).name() == "power_line_01") {
            line = static_cast<std::uint32_t>(i);
        }
    }
    ASSERT_NE(line, 0xFFFF'FFFFU);
    EXPECT_EQ(env.emi_sources(0).prim(), line);
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

// Viewer QA H1: every domain that runs is on in the Domains table, with its module and a
// switch; only a domain with no module at all says so. M8: no time of day, so no clock.
TEST_F(EmiServerTest, EveryDomainThatRunsIsOnAndCanBeSwitchedOff) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_environment(); }));
    std::map<std::string, v1::DomainState> domains;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            for (const v1::DomainState& d : e.profile_state().domains()) {
                domains[d.domain()] = d;
            }
        }
    }
    for (const auto& [name, module] : std::map<std::string, std::string>{
             {"aerodynamics", "drag"}, {"emi", "analytic"}, {"rotor_aero", "coefficients"}}) {
        ASSERT_TRUE(domains.contains(name)) << name;
        EXPECT_TRUE(domains[name].enabled()) << name;
        EXPECT_TRUE(domains[name].available()) << name;
        EXPECT_FALSE(domains[name].substituted()) << name;
        EXPECT_EQ(domains[name].active_module(), module) << name;
        EXPECT_FALSE(domains[name].substitutes().empty()) << name;
    }
    EXPECT_FALSE(domains["radiation"].available());
    EXPECT_NE(domains["radiation"].unavailable_reason().find("no module in this build"),
              std::string::npos);
    EXPECT_FALSE(domains["traffic"].available());
    EXPECT_EQ(domains["traffic"].unavailable_reason().find("no module in this build"),
              std::string::npos)
        << domains["traffic"].unavailable_reason();
    const v1::EnvironmentState& env = received_.back().environment();
    EXPECT_TRUE(std::isnan(env.local_time_hours()));
    EXPECT_TRUE(std::isnan(env.sun_elevation_deg()));

    // Switched off, it stays switchable (its substitute runs in its place).
    std::string yaml = received_[1].profile_state().yaml();
    const std::string on = "  aerodynamics: {enabled: true, fidelity: drag}\n";
    ASSERT_NE(yaml.find(on), std::string::npos) << yaml;
    yaml.replace(yaml.find(on), on.size(), "  aerodynamics: {enabled: false, substitute: zero}\n");
    v1::Envelope apply = command(2);
    apply.mutable_command()->mutable_apply_profile()->set_yaml(yaml);
    apply.mutable_command()->mutable_apply_profile()->set_restart(true);
    send(apply);
    received_.clear();
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    for (const v1::DomainState& d : received_.back().profile_state().domains()) {
        if (d.domain() == "aerodynamics") {
            EXPECT_FALSE(d.enabled());
            EXPECT_TRUE(d.available());
            EXPECT_TRUE(d.substituted());
        }
    }
}

// Viewer QA H2: the preview of a profile with a domain off shows its substitute's nodes.
TEST_F(ServerTest, ThePreviewOfADomainOffShowsItsSubstitute) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    std::string yaml = received_.back().profile_state().yaml();
    const std::string on = "  sensors: {enabled: true}\n";
    ASSERT_NE(yaml.find(on), std::string::npos) << yaml;
    yaml.replace(yaml.find(on), on.size(),
                 "  sensors: {enabled: false, substitute: ground_truth}\n");
    v1::Envelope check = command(1);
    check.mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    send(check);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 1;
    }));
    const v1::ProfileValidation& validation = received_.back().command_result().validation();
    EXPECT_TRUE(validation.valid());
    int sensors = 0;
    for (const v1::Wiring::Node& node : validation.wiring().nodes()) {
        if (node.domain() == "sensors") {
            ++sensors;
            EXPECT_TRUE(node.substitute()) << node.detail();
            EXPECT_TRUE(node.detail().starts_with("ground_truth")) << node.detail();
        } else if (node.domain() == "rigid_body") {
            EXPECT_FALSE(node.substitute());
        }
    }
    EXPECT_GT(sensors, 0);
}

// The aerodynamics domain off: its zero substitute (still air) stands in, in the preview's
// wiring as in the run, as the other domains' substitutes do.
TEST_F(QuadServerTest, ThePreviewOfTheAirOffShowsItsZeroSubstitute) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_profile_state(); }));
    std::string yaml = received_.back().profile_state().yaml();
    const std::string on = "  aerodynamics: {enabled: true, fidelity: drag}\n";
    ASSERT_NE(yaml.find(on), std::string::npos) << yaml;
    yaml.replace(yaml.find(on), on.size(), "  aerodynamics: {enabled: false, substitute: zero}\n");
    v1::Envelope check = command(1);
    check.mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    send(check);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 1;
    }));
    const v1::ProfileValidation& validation = received_.back().command_result().validation();
    EXPECT_TRUE(validation.valid());
    int air = 0;
    for (const v1::Wiring::Node& node : validation.wiring().nodes()) {
        if (node.domain() == "aerodynamics") {
            ++air;
            EXPECT_TRUE(node.substitute());
            EXPECT_EQ(node.detail(), "zero");
        }
    }
    EXPECT_EQ(air, 1) << "the zero substitute, in place of drag";
}

// Viewer QA M9: a weather preset's values, sent together, rebuild the run once; wind with the
// aerodynamics domain off is said to push nothing.
TEST_F(ServerTest, SiteValuesSentTogetherRebuildTheRunOnce) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_environment(); }));
    received_.clear();
    const std::vector<std::pair<std::string, double>> preset{
        {"temperature", 288.15}, {"pressure", 100800.0},  {"relative_humidity", 0.9},
        {"rain_rate", 5.6e-7},   {"visibility", 10000.0}, {"wind_speed", 3.0}};
    std::uint64_t id = 10;
    for (const auto& [name, value] : preset) {
        v1::Envelope set = command(id++);
        v1::SetAttribute& attribute = *set.mutable_command()->mutable_set_attribute();
        attribute.set_prim(0xFFFFFFFFU);
        attribute.mutable_attribute()->set_name("site:" + name);
        attribute.mutable_attribute()->set_real(value);
        send(set);  // queued: they reach the server together
    }
    std::size_t results = 0;
    ASSERT_TRUE(pumpUntil([&](const v1::Envelope& e) {
        if (e.has_command_result() && e.command_result().request_id() >= 10) {
            EXPECT_TRUE(e.command_result().ok()) << e.command_result().error();
            ++results;
        }
        return results == preset.size();
    }));
    int fromRest = 0;
    int together = 0;
    bool windWarned = false;
    v1::EnvironmentState env;  // sent once the run is rebuilt, before the results
    for (const v1::Envelope& e : received_) {
        if (e.has_environment()) {
            env = e.environment();
        }
        if (e.has_log() && e.log().source() == "site") {
            fromRest += e.log().text().find("from rest") != std::string::npos ? 1 : 0;
            together += e.log().text().starts_with("set together") ? 1 : 0;
            windWarned = windWarned || e.log().text().find("pushes nothing") != std::string::npos;
        }
    }
    EXPECT_EQ(together, 1);
    EXPECT_LE(fromRest, 1);
    EXPECT_TRUE(windWarned);
    EXPECT_NEAR(env.weather().fog_visibility_m(), 10000.0, 1e-6);
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

// The front radar's scans reach a viewer that watches it, as points with their Doppler.
// Viewer QA M5: the World workspace lists the traffic, not only the robot.
TEST_F(SedanServerTest, TheWorldListsTheTrafficAgents) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope want = command(3);
    want.mutable_command()->mutable_set_world()->set_enabled(true);
    send(want);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_world(); }));
    std::map<std::string, int> kinds;
    int focus = 0;
    for (const v1::WorldState::Agent& agent : received_.back().world().agents()) {
        ++kinds[agent.kind()];
        focus += agent.focus() ? 1 : 0;
    }
    EXPECT_EQ(focus, 1);
    EXPECT_EQ(kinds["robot"], 1);
    EXPECT_GT(kinds["vehicle"], 0);
    EXPECT_GT(kinds["pedestrian"], 0);
}

// A served run is one region: the World message carries it with real bounds, around the
// warehouse's scenery and the robot, not left for the viewer to make up.
TEST_F(ServerTest, TheWorldReportsTheRunsRegion) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope want = command(3);
    want.mutable_command()->mutable_set_world()->set_enabled(true);
    send(want);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_world(); }));
    const v1::WorldState& world = received_.back().world();
    ASSERT_EQ(world.regions_size(), 1);
    const v1::WorldState::Region& region = world.regions(0);
    EXPECT_EQ(region.name(), "R1");
    EXPECT_GT(region.max_x() - region.min_x(), 5.0);  // m: the warehouse's walls and racks
    EXPECT_GT(region.max_y() - region.min_y(), 5.0);
    for (const v1::WorldState::Agent& agent : world.agents()) {
        EXPECT_GT(agent.x(), region.min_x());
        EXPECT_LT(agent.x(), region.max_x());
        EXPECT_GT(agent.y(), region.min_y());
        EXPECT_LT(agent.y(), region.max_y());
    }
}

/// The AMR in a street of the streamed tiled town (huge-worlds.md §6).
class TiledTownServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.worldPath = LAGRANGE_TEST_TILED_TOWN;
        options.spawn.position = {1036.0, 1036.0, 0.0};
    }
};

// The region of a streamed run covers every tile it has loaded, in world ENU, and its robot.
TEST_F(TiledTownServerTest, TheRegionCoversTheLoadedTiles) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope want = command(3);
    want.mutable_command()->mutable_set_world()->set_enabled(true);
    send(want);
    ASSERT_TRUE(pumpUntil(
        [](const v1::Envelope& e) { return e.has_world() && e.world().tile_x_size() > 0; }));
    const v1::WorldState& world = received_.back().world();
    ASSERT_EQ(world.regions_size(), 1);
    const v1::WorldState::Region& region = world.regions(0);
    const double tile = world.tile_size_m();
    ASSERT_GT(tile, 0.0);
    for (int i = 0; i < world.tile_x_size(); ++i) {
        const double x = (world.tile_x(i) + 0.5) * tile;
        const double y = (world.tile_y(i) + 0.5) * tile;
        EXPECT_GE(x, region.min_x());
        EXPECT_LE(x, region.max_x());
        EXPECT_GE(y, region.min_y());
        EXPECT_LE(y, region.max_y());
    }
    EXPECT_LT(region.min_x(), 1036.0);
    EXPECT_GT(region.max_x(), 1036.0);
    EXPECT_LT(region.min_y(), 1036.0);
    EXPECT_GT(region.max_y(), 1036.0);
    EXPECT_GT(region.max_x() - region.min_x(), 300.0);  // m: the collision radius each way
}

TEST_F(SedanServerTest, RadarFramesCarryDetectionsWithVelocity) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_sensor()->set_sensor("/sedan/base_link/front_radar");
    watch.mutable_command()->mutable_subscribe_sensor()->set_enabled(true);
    send(watch);
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(1'000'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_sensor_frame() && e.sensor_frame().points().count() > 0;
    }));
    const v1::SensorFrame& frame = received_.back().sensor_frame();
    EXPECT_EQ(frame.sensor(), "/sedan/base_link/front_radar");
    const v1::PointCloudData& cloud = frame.points();
    ASSERT_EQ(cloud.fields_size(), 6);
    EXPECT_EQ(cloud.fields(3), "velocity");
    EXPECT_EQ(cloud.fields(4), "rcs");
    EXPECT_EQ(cloud.fields(5), "snr");
    EXPECT_EQ(cloud.stride(), 24U);
    ASSERT_EQ(cloud.data().inline_().size(), cloud.count() * 24U);
    for (std::uint32_t i = 0; i < cloud.count(); ++i) {
        std::array<float, 6> p{};
        std::memcpy(p.data(), cloud.data().inline_().data() + std::size_t{i} * 24, sizeof(p));
        const float range = std::hypot(p[0], p[1], p[2]);
        EXPECT_GT(range, 0.0F);
        EXPECT_LE(range, 150.0F * 1.01F);                    // sensor:radar:rangeMax
        EXPECT_LE(std::abs(std::atan2(p[1], p[0])), 0.80F);  // ±45° and a little noise
        EXPECT_TRUE(std::isfinite(p[3]));                    // radial velocity
        EXPECT_GT(p[4], 0.0F);                               // rcs
        EXPECT_GT(p[5], 10.0F);                              // snr over its 10 dB threshold
    }
}

// Sensors QA F9: a viewer that starts watching the radar of a run that stands still (paused, or
// stepped before it connected) gets the last scan at once, not at a next scan that never comes.
TEST_F(SedanServerTest, AViewerThatWatchesAPausedRadarGetsItsLastScan) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(1'000'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_command_result() && e.command_result().request_id() == 5;
    }));
    for (const v1::Envelope& e : received_) {
        ASSERT_FALSE(e.has_sensor_frame()) << "nobody watched it yet";
    }
    const std::uint64_t before = server_->now().ns;
    v1::Envelope watch = command(6);
    watch.mutable_command()->mutable_subscribe_sensor()->set_sensor("/sedan/base_link/front_radar");
    watch.mutable_command()->mutable_subscribe_sensor()->set_enabled(true);
    send(watch);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) {
        return e.has_sensor_frame() && e.sensor_frame().points().count() > 0;
    }));
    EXPECT_EQ(received_.back().sensor_frame().sensor(), "/sedan/base_link/front_radar");
    EXPECT_EQ(server_->now().ns, before) << "no step was needed";
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

// ServerStats (Performance workspace): a rendered camera's module runs on the GPU and the GPU
// memory the HAL's allocator holds is reported, both only on a hardware device; on a software
// device (Tier 0, llvmpipe) the camera is not on the GPU and the GPU memory is 0.
TEST_F(ArmServerTest, StatsMarkTheRenderedCameraOnTheGpuAndCountItsMemory) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(100'000'000);
    send(step);
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_server_stats(); }));
    const v1::ServerStats stats = received_.back().server_stats();
    const v1::ServerStats::Module* camera = nullptr;
    for (const v1::ServerStats::Module& m : stats.modules()) {
        if (m.module() == "sensors/camera_wrist_camera") {
            camera = &m;
        }
        if (m.module().starts_with("rigid_body/") || m.module().starts_with("electrical/")) {
            EXPECT_FALSE(m.on_gpu()) << m.module();
        }
    }
    ASSERT_NE(camera, nullptr) << "no camera module in the stats";
    // The device the server's cameras take: the same automatic choice (LAGRANGE_GPU, else
    // discrete > integrated > software).
    std::string error;
    const std::unique_ptr<lagrange::gpu::Device> device = lagrange::gpu::Device::create({}, error);
    if (!device) {
        EXPECT_FALSE(camera->on_gpu());
        EXPECT_EQ(stats.gpu_memory_bytes(), 0U);
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    if (device->info().isSoftware) {
        EXPECT_FALSE(camera->on_gpu()) << device->info().name;
        EXPECT_EQ(stats.gpu_memory_bytes(), 0U) << device->info().name;
        GTEST_SKIP() << "only a software device (Tier 0): " << device->info().name;
    }
    EXPECT_TRUE(camera->on_gpu()) << device->info().name;
    // The camera's 640 x 480 targets alone are over a megabyte.
    EXPECT_GT(stats.gpu_memory_bytes(), 1'000'000U) << device->info().name;
}

// Any image signal by its name (sensors.md §9 "Images of kinds"): here the camera's depth, r32f.
TEST_F(ArmServerTest, AnImageSignalIsSentByItsName) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_catalog(); }));
    v1::Envelope watch = command(4);
    watch.mutable_command()->mutable_subscribe_sensor()->set_sensor("sensor/wrist_camera/depth");
    watch.mutable_command()->mutable_subscribe_sensor()->set_enabled(true);
    send(watch);
    v1::Envelope step = command(5);
    step.mutable_command()->mutable_step()->set_duration_ns(100'000'000);
    send(step);
    if (!pumpUntil([](const v1::Envelope& e) { return e.has_sensor_frame(); })) {
        GTEST_SKIP() << "no camera frame (no Vulkan device?)";
    }
    const v1::SensorFrame& frame = received_.back().sensor_frame();
    EXPECT_EQ(frame.sensor(), "sensor/wrist_camera/depth");
    ASSERT_TRUE(frame.has_image());
    EXPECT_EQ(frame.image().encoding(), "r32f");
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
    const std::string start = received_.back().snapshot().transforms().inline_();

    v1::Envelope step = command(2);
    step.mutable_command()->mutable_step()->set_duration_ns(1'500'000'000);
    ASSERT_TRUE(run(step).ok());
    expectPose(at15_);
    // QA 2026-11-14 (L10): the scene sent to viewers moves with the recorded poses, so the robot
    // drives in the viewer as it did in the run (it stood still, at 0.00 m/s).
    const auto late = [](const v1::Envelope& e) {
        return e.has_delta() && e.delta().sim_time().ns() >= 1'000'000'000;
    };
    if (std::ranges::none_of(received_, late)) {
        ASSERT_TRUE(pumpUntil(late));
    }
    const v1::SceneDelta moved = std::find_if(received_.rbegin(), received_.rend(), late)->delta();
    ASSERT_GT(moved.transform_prims_size(), 0);
    double farthest = 0.0;
    for (int k = 0; k < moved.transform_prims_size(); ++k) {
        constexpr std::size_t kSnapshotPose = 52;  // position, orientation, scale
        constexpr std::size_t kDeltaPose = 40;     // position, orientation
        std::array<double, 3> was{};
        std::array<double, 3> now{};
        std::memcpy(was.data(), start.data() + moved.transform_prims(k) * kSnapshotPose, 24);
        std::memcpy(now.data(), moved.transform_data().data() + k * kDeltaPose, 24);
        farthest = std::max(farthest, std::hypot(now[0] - was[0], now[1] - was[1]));
    }
    EXPECT_GT(farthest, 0.3);  // m: the sample AMR's demo drive in 1 s or more
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
    // Viewer QA M7: the domain strip follows, every recorded domain standing in by its replay.
    std::string yaml;
    for (const v1::Envelope& e : received_) {
        if (e.has_profile_state()) {
            yaml = e.profile_state().yaml();
        }
    }
    EXPECT_NE(yaml.find("rigid_body: {enabled: false, substitute: replay"), std::string::npos)
        << yaml;
    ASSERT_TRUE(seekTo(1'500'000'000, 6).ok());
    expectPose(at15_);
    // A file that is not a recording is refused and the replay stays.
    v1::Envelope bad = command(7);
    bad.mutable_command()->mutable_open_recording()->set_path("/nonexistent.mcap");
    EXPECT_FALSE(run(bad).ok());
    ASSERT_TRUE(seekTo(500'000'000, 8).ok());
    expectPose(at05_);
}

// QA 2026-11-14: the sample AMR's open-loop figure-eight wandered and, after a minute, stood
// against the warehouse's north rack. Closed-loop, it keeps to its two 1.2 m loops round the
// start (the racks' faces are 3 m to each side).
TEST(DemoDrive, TheSampleAmrKeepsToItsFigureEightInTheWarehouse) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
    options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;
    options.ros = false;
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    double farthestX = 0.0;
    double farthestY = 0.0;
    double travelled = 0.0;
    std::vector<double> last = server.read("body/base_link/pose");
    for (std::int64_t t = 1; t <= 120; ++t) {
        ASSERT_TRUE(server.stepUntil(SimTime{t * 1'000'000'000}).ok());
        const std::vector<double> pose = server.read("body/base_link/pose");
        farthestX = std::max(farthestX, std::abs(pose[0]));
        farthestY = std::max(farthestY, std::abs(pose[1]));
        travelled += std::hypot(pose[0] - last[0], pose[1] - last[1]);
        last = pose;
    }
    EXPECT_LT(farthestX, 1.35);  // m: the loops' radius, 1.2 m, and a little
    EXPECT_LT(farthestY, 2.55);  // m: their two diameters, 2.4 m, and a little
    EXPECT_GT(farthestY, 2.2);
    EXPECT_GT(travelled, 60.0);  // m: about 0.6 m/s for two minutes, never held
}

// QA 2026-11-14: the sedan's demo in the town ran onto a crossing's post at a corner of the
// junction after 6 s and stood there with its throttle open. It backs away and drives on.
TEST(DemoDrive, TheSedanBacksAwayFromWhatHoldsItAndDrivesOn) {
    ServerOptions options;
    options.robotPath = LAGRANGE_TEST_SEDAN_USDA;
    options.worldPath =
        (std::filesystem::path(LAGRANGE_TEST_CITY_USDA).parent_path().parent_path() / "town" /
         "town.usda")
            .string();
    options.ros = false;
    Server server(options);
    ASSERT_TRUE(server.load().ok());
    double longestHeld = 0.0;
    double held = 0.0;
    double travelledLate = 0.0;
    std::vector<double> last;
    for (std::int64_t k = 1; k <= 60; ++k) {  // 30 s by half seconds
        ASSERT_TRUE(server.stepUntil(SimTime{k * 500'000'000}).ok());
        const double speed = server.read("vehicle/sedan/speed")[0];
        held = std::abs(speed) < 0.1 ? held + 0.5 : 0.0;
        longestHeld = std::max(longestHeld, held);
        const std::vector<double> pose = server.read("body/base_link/pose");
        if (k > 40) {
            travelledLate += std::hypot(pose[0] - last[0], pose[1] - last[1]);
        }
        last = pose;
    }
    EXPECT_LE(longestHeld, 2.5);     // s: held 2 s, then it backs away
    EXPECT_GT(travelledLate, 20.0);  // m in the last 10 s: on its circle at 3 m/s
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
    // The buildings of the collision tiles within 300 m (world.streaming.collision_radius's
    // default): each holds up to 9 boxes.
    EXPECT_GT(world.colliders.body.size(), 50U);
    double farthest = 0.0;
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.shape[c] == ShapeType::Box) {
            const std::array<double, 3>& p = world.colliders.poseInBody[c].position;
            farthest = std::max(farthest, std::hypot(p[0] + world.region.worldEnu[0] - 1036.0,
                                                     p[1] + world.region.worldEnu[1] - 1036.0));
        }
    }
    EXPECT_LT(farthest, 300.0 + 256.0 * std::sqrt(2.0)) << "only tiles within the radius";
    const auto b = run(5'000'000'000);
    const auto ha = a->stateHash();
    const auto hb = b->stateHash();
    ASSERT_TRUE(ha && hb);
    EXPECT_EQ(*ha, *hb);
    EXPECT_EQ(a->world().colliders.body.size(), b->world().colliders.body.size());
}

// v1.0 security (docs/design/security.md): a server on tls:// with a token greets a client
// that sends the token, refuses one that does not, and will not listen beyond this machine
// without a token at all.
TEST(Security, TlsWithATokenGreetsOnlyClientsThatHaveIt) {
    char made[] = "/tmp/lagrange-tls-test-XXXXXX";
    ASSERT_NE(::mkdtemp(made), nullptr);
    const std::string dir = made;
    const std::string make =
        "openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes -days 1 "
        "-subj /CN=localhost -addext subjectAltName=IP:127.0.0.1 -keyout " +
        dir + "/key.pem -out " + dir + "/cert.pem 2>/dev/null";
    ASSERT_EQ(std::system(make.c_str()), 0) << "openssl could not make a test certificate";
    ServerOptions options;
    options.endpoint = proto::Endpoint::tcp("127.0.0.1", 0, true);
    options.tlsCertificate = dir + "/cert.pem";
    options.tlsKey = dir + "/key.pem";
    options.authToken = "s3cret-token";
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.ros = false;
    options.startPaused = true;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;
    std::string error;
    const auto trust = proto::TlsContext::client(dir + "/cert.pem", error);
    ASSERT_TRUE(trust) << error;

    const auto greet = [&](const std::string& token) -> v1::ServerHello {
        std::error_code ec;
        proto::Connection client = proto::Connection::startConnect(
            server.endpoint(), ec, proto::kDefaultMaxFrameSize, trust);
        v1::Envelope hello;
        hello.mutable_client_hello()->mutable_version()->set_major(0);
        hello.mutable_client_hello()->mutable_version()->set_minor(1);
        hello.mutable_client_hello()->set_auth_token(token);
        EXPECT_TRUE(client.queue(hello));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            server.runOnce(1);
            if (client.state() == proto::Connection::State::Connecting) {
                (void)client.finishConnect(ec);
            }
            (void)client.flush(ec);
            (void)client.receive(ec);
            const proto::FrameReader::Result frame = client.next();
            v1::Envelope reply;
            if (frame.status == proto::FrameReader::Status::Frame &&
                proto::parsePayload(frame.payload, reply) && reply.has_server_hello()) {
                return reply.server_hello();
            }
        }
        ADD_FAILURE() << "no ServerHello: " << client.tlsError();
        return {};
    };
    EXPECT_EQ(greet("s3cret-token").refusal(), "");  // over TLS: a plain client gets nothing
    EXPECT_NE(greet("wrong").refusal().find("authentication failed"), std::string::npos);
    EXPECT_NE(greet("").refusal().find("authentication failed"), std::string::npos);

    ServerOptions open = options;
    open.endpoint = proto::Endpoint::tcp("0.0.0.0", 0, false);
    open.authToken.clear();
    Server exposed(open);
    const Status refused = exposed.start();
    EXPECT_FALSE(refused.ok());
    EXPECT_NE(refused.message.find("--auth-token-file"), std::string::npos) << refused.message;
    std::filesystem::remove_all(dir);
}

// RFC 0016 §7.2: the builder's Test into the connected run. The viewer sends the robot's URDF
// text (ImportRobot.content), the server names it by its <robot name> and answers its scene
// path, and RemoveRobot takes it out again.
class ImportServerTest : public ServerTest {
protected:
    void adjust(ServerOptions& options) override { options.ros = false; }

    v1::CommandResult run(v1::Envelope envelope) {
        const std::uint64_t id = envelope.command().request_id();
        send(envelope);
        EXPECT_TRUE(pumpUntil([id](const v1::Envelope& e) {
            return e.has_command_result() && e.command_result().request_id() == id;
        }));
        return received_.empty() ? v1::CommandResult{} : received_.back().command_result();
    }

    v1::CommandResult importText(std::uint64_t id, const std::string& text,
                                 v1::ImportRobot::Format format = v1::ImportRobot::FORMAT_URDF,
                                 const std::string& overlay = {}) {
        v1::Envelope envelope = command(id);
        v1::ImportRobot& request = *envelope.mutable_command()->mutable_import_robot();
        request.set_format(format);
        request.mutable_content()->set_inline_(text);
        if (!overlay.empty()) {
            request.mutable_overlay_content()->set_inline_(overlay);
        }
        return run(envelope);
    }

    v1::CommandResult removeRobot(std::uint64_t id, const std::string& name) {
        v1::Envelope envelope = command(id);
        envelope.mutable_command()->mutable_remove_robot()->set_name(name);
        return run(envelope);
    }

    [[nodiscard]] bool hasBody(const std::string& name) const {
        const std::vector<std::string>& names = server_->world().bodies.names;
        return std::find(names.begin(), names.end(), name) != names.end();
    }

    static std::string readText(const std::string& path) {
        std::ifstream in(path);
        return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }

    /// A one-link robot as the builder writes it.
    static std::string boxRobot(const std::string& name) {
        return R"(<?xml version="1.0"?>
<robot name=")" +
               name + R"(">
  <link name="body">
    <inertial><mass value="2.0"/><inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="0.3 0.3 0.2"/></geometry></visual>
    <collision><geometry><box size="0.3 0.3 0.2"/></geometry></collision>
  </link>
</robot>
)";
    }
};

TEST_F(ImportServerTest, ContentIsNamedByItsRobotElementAndRemovedAgain) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const v1::CommandResult imported = importText(1, boxRobot("crate bot"));
    ASSERT_TRUE(imported.ok()) << imported.error();
    EXPECT_EQ(imported.imported_stage_path(), "/crate_bot") << "a valid name from <robot name>";
    EXPECT_TRUE(hasBody("crate_bot_body")) << "passive beside the AMR: its names prefixed";
    EXPECT_TRUE(hasBody("base_link")) << "the run's robot stays";

    const v1::CommandResult removed = removeRobot(2, imported.imported_stage_path());
    ASSERT_TRUE(removed.ok()) << removed.error();
    EXPECT_FALSE(hasBody("crate_bot_body"));
    EXPECT_TRUE(hasBody("base_link"));

    const v1::CommandResult again = removeRobot(3, "crate_bot");
    EXPECT_FALSE(again.ok()) << "already gone";
    EXPECT_NE(again.error().find("no robot named crate_bot"), std::string::npos) << again.error();
    const v1::CommandResult scenery = removeRobot(4, "warehouse_small");
    EXPECT_FALSE(scenery.ok()) << "the world's scenery stays";
}

// Inline content is written to a private folder of the user's ($XDG_RUNTIME_DIR/lagrange, 0700)
// as a file made exclusively (0600), and a folder someone could swap files in is refused.
TEST_F(ImportServerTest, InlineContentGoesToAPrivateFolder) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const char* was = std::getenv("XDG_RUNTIME_DIR");
    const std::string saved = was != nullptr ? was : "";
    const std::filesystem::path runtime =
        std::filesystem::temp_directory_path() / ("lagrange_runtime_" + std::to_string(::getpid()));
    std::filesystem::remove_all(runtime);
    std::filesystem::create_directories(runtime);
    ::setenv("XDG_RUNTIME_DIR", runtime.c_str(), 1);
    const std::filesystem::path dir = runtime / "lagrange";

    const v1::CommandResult imported = importText(1, boxRobot("kept"));
    ASSERT_TRUE(imported.ok()) << imported.error();
    struct stat info{};
    ASSERT_EQ(::lstat(dir.c_str(), &info), 0);
    EXPECT_TRUE(S_ISDIR(info.st_mode));
    EXPECT_EQ(info.st_mode & 0777, 0700U);
    std::size_t files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        ++files;
        ASSERT_EQ(::lstat(entry.path().c_str(), &info), 0);
        EXPECT_EQ(info.st_mode & 0777, 0600U) << entry.path();
        EXPECT_TRUE(entry.path().filename().string().starts_with("lagrange-spawn-"));
    }
    EXPECT_EQ(files, 1U);
    ASSERT_TRUE(removeRobot(2, "kept").ok());
    EXPECT_TRUE(std::filesystem::is_empty(dir)) << "removed with its robot";

    // Open to others, or a link to elsewhere: refused, nothing written.
    std::filesystem::permissions(dir, std::filesystem::perms::all);
    const v1::CommandResult open = importText(3, boxRobot("open"));
    EXPECT_FALSE(open.ok());
    EXPECT_NE(open.error().find("not a private folder"), std::string::npos) << open.error();
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(runtime / "elsewhere");
    std::filesystem::create_directory_symlink(runtime / "elsewhere", dir);
    const v1::CommandResult linked = importText(4, boxRobot("linked"));
    EXPECT_FALSE(linked.ok());
    EXPECT_TRUE(std::filesystem::is_empty(runtime / "elsewhere"));

    if (was != nullptr) {
        ::setenv("XDG_RUNTIME_DIR", saved.c_str(), 1);
    } else {
        ::unsetenv("XDG_RUNTIME_DIR");
    }
    std::filesystem::remove_all(runtime);
}

TEST_F(ImportServerTest, ContentIsRenamedWhenItsNameIsTaken) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const v1::CommandResult first = importText(1, boxRobot("crate"));
    ASSERT_TRUE(first.ok()) << first.error();
    const v1::CommandResult second = importText(2, boxRobot("crate"));
    ASSERT_TRUE(second.ok()) << second.error();
    EXPECT_EQ(first.imported_stage_path(), "/crate");
    EXPECT_EQ(second.imported_stage_path(), "/crate_2");
    EXPECT_TRUE(hasBody("crate_body"));
    EXPECT_TRUE(hasBody("crate_2_body"));
    // The run's own robot's name is taken too.
    const v1::CommandResult third = importText(3, boxRobot("amr_diffdrive"));
    ASSERT_TRUE(third.ok()) << third.error();
    EXPECT_EQ(third.imported_stage_path(), "/amr_diffdrive_2");
    // Removing the second leaves the first.
    ASSERT_TRUE(removeRobot(4, "/crate_2").ok());
    EXPECT_TRUE(hasBody("crate_body"));
    EXPECT_FALSE(hasBody("crate_2_body"));
}

TEST_F(ImportServerTest, RefusesContentThatIsNotTheFormatOrNotInline) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const std::size_t bodies = server_->world().bodies.names.size();

    const v1::CommandResult notXml = importText(1, "hello robot");
    EXPECT_FALSE(notXml.ok());
    EXPECT_NE(notXml.error().find("neither XML nor USD"), std::string::npos) << notXml.error();

    const v1::CommandResult other = importText(2, "<world/>");
    EXPECT_FALSE(other.ok());
    EXPECT_NE(other.error().find("is not URDF"), std::string::npos) << other.error();

    const v1::CommandResult sdf = importText(3, R"(<sdf version="1.9"><model name="m"/></sdf>)");
    EXPECT_FALSE(sdf.ok()) << "FORMAT_URDF with SDFormat text";
    EXPECT_NE(sdf.error().find("not URDF"), std::string::npos) << sdf.error();

    v1::Envelope blob = command(4);
    blob.mutable_command()->mutable_import_robot()->mutable_content()->set_blob_id(7);
    const v1::CommandResult byBlob = run(blob);
    EXPECT_FALSE(byBlob.ok());
    EXPECT_NE(byBlob.error().find("inline"), std::string::npos) << byBlob.error();

    const v1::CommandResult xacro = importText(5, boxRobot("x"), v1::ImportRobot::FORMAT_XACRO);
    EXPECT_FALSE(xacro.ok());
    EXPECT_NE(xacro.error().find("xacro"), std::string::npos) << xacro.error();

    v1::Envelope nothing = command(6);
    nothing.mutable_command()->mutable_import_robot()->set_format(v1::ImportRobot::FORMAT_URDF);
    const v1::CommandResult empty = run(nothing);
    EXPECT_FALSE(empty.ok());
    EXPECT_NE(empty.error().find("content"), std::string::npos) << empty.error();

    // A path and content at once: which one was meant is not guessed.
    v1::Envelope both = command(8);
    both.mutable_command()->mutable_import_robot()->set_path(LAGRANGE_TEST_AMR_URDF);
    both.mutable_command()->mutable_import_robot()->mutable_content()->set_inline_(boxRobot("b"));
    const v1::CommandResult twice = run(both);
    EXPECT_FALSE(twice.ok());
    EXPECT_NE(twice.error().find("not both"), std::string::npos) << twice.error();

    // A link without its inertia is the loader's to refuse; the run is untouched.
    const v1::CommandResult broken = importText(
        7,
        R"(<robot name="b"><link name="l"/><joint name="j" type="revolute"><parent link="l"/><child link="nope"/></joint></robot>)");
    EXPECT_FALSE(broken.ok());
    EXPECT_EQ(server_->world().bodies.names.size(), bodies) << "refusals leave the run";
}

/// An empty run (--robot none --world none), as the builder's own test server starts.
class EmptyImportServerTest : public ImportServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.ros = false;
        options.robotPath.clear();
        options.worldPath.clear();
    }
};

// ImportRobot.overlay_content is applied to the imported robot as --overlay is: the AMR's
// overlay gives it its battery and 48 V bus.
TEST_F(EmptyImportServerTest, AnInlineOverlayGivesTheImportedRobotItsPowerSystem) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const std::string urdf = readText(LAGRANGE_TEST_AMR_URDF);
    const std::string overlay = readText(LAGRANGE_TEST_AMR_OVERLAY);
    ASSERT_FALSE(urdf.empty());
    ASSERT_FALSE(overlay.empty());
    const v1::CommandResult imported = importText(1, urdf, v1::ImportRobot::FORMAT_URDF, overlay);
    ASSERT_TRUE(imported.ok()) << imported.error();
    EXPECT_EQ(imported.imported_stage_path(), "/amr_diffdrive");
    EXPECT_TRUE(hasBody("base_link")) << "the first robot in an empty run is the robot";
    const std::vector<double> volts = server_->read("bus/dc_48v/voltage");
    ASSERT_EQ(volts.size(), 1U) << "the overlay's bus is in the run";

    // A broken overlay refuses the robot with the overlay's message.
    ASSERT_TRUE(removeRobot(2, "amr_diffdrive").ok());
    const v1::CommandResult bad =
        importText(3, urdf, v1::ImportRobot::FORMAT_URDF,
                   "schema_version: 1\napply:\n  - prim: no_such_prim\n"
                   "    schemas: {ThermalAPI_1: {thermal:heatCapacity: 1.0}}\n");
    EXPECT_FALSE(bad.ok());
    EXPECT_FALSE(hasBody("base_link"));
}

// RFC 0016 §7.3: the builder's joint sliders drive a tested robot through CommandJoint, as
// /joint_commands does under joint control.
class JointCommandServerTest : public ImportServerTest {
protected:
    void adjust(ServerOptions& options) override {
        options.ros = false;
        options.jointControl = true;
        options.demoDrive = false;
    }

    v1::CommandResult commandJoint(std::uint64_t id, const std::string& joint,
                                   const std::string& mode, double value) {
        v1::Envelope envelope = command(id);
        v1::CommandJoint& request = *envelope.mutable_command()->mutable_command_joint();
        request.set_joint(joint);
        request.set_mode(mode);
        request.set_value(value);
        return run(envelope);
    }

    void stepFor(std::uint64_t id, std::int64_t ns) {
        v1::Envelope step = command(id);
        step.mutable_command()->mutable_step()->set_duration_ns(ns);
        (void)run(step);
    }
};

TEST_F(JointCommandServerTest, PositionReachesTheTarget) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const v1::CommandResult result = commandJoint(1, "front_caster_swivel", "position", 0.8);
    ASSERT_TRUE(result.ok()) << result.error();
    stepFor(2, 3'000'000'000);
    EXPECT_NEAR(server_->read("joint/front_caster_swivel/position").at(0), 0.8, 0.05);
    // The last command wins: a slider dragged back.
    ASSERT_TRUE(commandJoint(3, "front_caster_swivel", "position", -0.3).ok());
    stepFor(4, 3'000'000'000);
    EXPECT_NEAR(server_->read("joint/front_caster_swivel/position").at(0), -0.3, 0.05);
}

TEST_F(JointCommandServerTest, RefusesWhatNoJointTakes) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    const auto refused = [&](std::uint64_t id, const std::string& joint, const std::string& mode,
                             double value, const std::string& says) {
        const v1::CommandResult result = commandJoint(id, joint, mode, value);
        EXPECT_FALSE(result.ok()) << joint << " " << mode;
        EXPECT_NE(result.error().find(says), std::string::npos) << result.error();
    };
    refused(1, "elbow", "position", 0.1, "the run has no joint elbow");
    refused(2, "left_wheel_joint", "banana", 0.1, "no mode banana");
    refused(3, "left_wheel_joint", "position", std::numeric_limits<double>::quiet_NaN(),
            "not a finite number");
    // A fixed joint is merged away at load: the run does not have it.
    refused(4, "lidar_mount", "position", 0.1, "the run has no joint lidar_mount");
    // A passive robot beside it: its joints are not joint-controlled.
    const v1::CommandResult imported = importText(5, R"(<?xml version="1.0"?>
<robot name="pan">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <link name="head"><inertial><mass value="0.2"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="turn" type="revolute"><parent link="base"/><child link="head"/><origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint>
</robot>
)");
    ASSERT_TRUE(imported.ok()) << imported.error();
    refused(6, "pan_turn", "position", 0.5, "passive robot");
    // A good one still goes through.
    EXPECT_TRUE(commandJoint(7, "left_wheel_joint", "velocity", 1.0).ok());
}

/// A pan head on a base as the builder writes it: one revolute joint about z.
std::string panRobot(const std::string& name) {
    return R"(<?xml version="1.0"?>
<robot name=")" +
           name +
           R"(">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <link name="head"><inertial><mass value="0.2"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="turn" type="revolute"><parent link="base"/><child link="head"/><origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint>
</robot>
)";
}

// A joint target belongs to its robot: a robot removed and imported again (the builder's Test
// again) starts uncommanded, and a reset forgets every target.
class EmptyJointCommandServerTest : public JointCommandServerTest {
protected:
    void adjust(ServerOptions& options) override {
        JointCommandServerTest::adjust(options);
        options.robotPath.clear();
        options.worldPath.clear();
    }
};

TEST_F(EmptyJointCommandServerTest, TargetsGoWithTheirRobotAndWithAReset) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    ASSERT_TRUE(importText(1, panRobot("pan")).ok());
    ASSERT_TRUE(commandJoint(2, "turn", "position", 0.8).ok());
    stepFor(3, 2'000'000'000);
    ASSERT_NEAR(server_->read("joint/turn/position").at(0), 0.8, 0.05);

    ASSERT_TRUE(removeRobot(4, "pan").ok());
    ASSERT_TRUE(importText(5, panRobot("pan")).ok());
    stepFor(6, 2'000'000'000);
    EXPECT_NEAR(server_->read("joint/turn/position").at(0), 0.0, 0.01)
        << "the new robot went to its predecessor's target";

    ASSERT_TRUE(commandJoint(7, "turn", "position", -0.6).ok());
    v1::Envelope reset = command(8);
    reset.mutable_command()->mutable_reset();
    ASSERT_TRUE(run(reset).ok());
    stepFor(9, 2'000'000'000);
    EXPECT_NEAR(server_->read("joint/turn/position").at(0), 0.0, 0.01)
        << "a reset run went on to a target it was given before the reset";
}

// Another robot spawned or taken away leaves the run's robot its targets.
TEST_F(JointCommandServerTest, OtherRobotsComingAndGoingKeepTheTargets) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    ASSERT_TRUE(commandJoint(1, "front_caster_swivel", "position", 0.8).ok());
    ASSERT_TRUE(importText(2, panRobot("pan")).ok());
    stepFor(3, 3'000'000'000);
    EXPECT_NEAR(server_->read("joint/front_caster_swivel/position").at(0), 0.8, 0.05);
    ASSERT_TRUE(commandJoint(4, "front_caster_swivel", "position", -0.3).ok());
    ASSERT_TRUE(removeRobot(5, "pan").ok());
    stepFor(6, 3'000'000'000);
    EXPECT_NEAR(server_->read("joint/front_caster_swivel/position").at(0), -0.3, 0.05);
}

// Without joint control (the default) the run's controller drives the robot: refused, saying how.
TEST_F(ImportServerTest, CommandJointNeedsJointControl) {
    ASSERT_TRUE(pumpUntil([](const v1::Envelope& e) { return e.has_status(); }));
    v1::Envelope envelope = command(1);
    envelope.mutable_command()->mutable_command_joint()->set_joint("left_wheel_joint");
    envelope.mutable_command()->mutable_command_joint()->set_mode("velocity");
    envelope.mutable_command()->mutable_command_joint()->set_value(1.0);
    const v1::CommandResult result = run(envelope);
    EXPECT_FALSE(result.ok());
    EXPECT_NE(result.error().find("--joint-control"), std::string::npos) << result.error();
}

}  // namespace
}  // namespace lagrange::server
