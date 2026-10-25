// The server end to end over a real Unix socket, as the viewer sees it.
#include "lagrange/proto/Connection.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
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

}  // namespace
}  // namespace lagrange::server
