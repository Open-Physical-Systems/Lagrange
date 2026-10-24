// The server end to end over a real Unix socket, as the viewer sees it.
#include "lagrange/proto/Connection.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
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

}  // namespace
}  // namespace lagrange::server
