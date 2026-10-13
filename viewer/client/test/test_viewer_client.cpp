// ViewerClient against a scripted server built from the same transport classes.

#include "lagrange/proto/Connection.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace {

using namespace std::chrono_literals;
using lagrange::proto::Connection;
using lagrange::proto::Endpoint;
using lagrange::proto::Listener;
using lagrange::viewer::AttributeInfo;
using lagrange::viewer::ConnectionState;
using lagrange::viewer::kNone;
using lagrange::viewer::LogBook;
using lagrange::viewer::LogLevel;
using lagrange::viewer::RunState;
using lagrange::viewer::ViewerClient;
namespace v1 = lagrange::viewer::v1;

class FakeServer {
public:
    FakeServer() {
        std::string pattern = (std::filesystem::temp_directory_path() / "lgr-XXXXXX").string();
        directory_ = ::mkdtemp(pattern.data());
        endpoint_ = Endpoint::unixSocket(directory_ + "/s.sock");
        listen();
    }
    ~FakeServer() {
        connection_.close();
        listener_.close();
        std::error_code ec;
        std::filesystem::remove_all(directory_, ec);
    }
    FakeServer(const FakeServer&) = delete;
    FakeServer& operator=(const FakeServer&) = delete;

    void listen() {
        std::error_code ec;
        listener_ = Listener::listen(endpoint_, ec);
        ASSERT_TRUE(listener_.valid()) << ec.message();
    }
    void shutdown() {
        // Listener first: otherwise the client can reconnect into the still-open backlog in
        // the instant between the two closes, and the test would count an extra connect.
        listener_.close();
        connection_.close();
    }

    [[nodiscard]] const Endpoint& endpoint() const { return endpoint_; }

    void send(const v1::Envelope& envelope) { ASSERT_TRUE(connection_.queue(envelope)); }

    /// Runs the server's I/O until `done` is true. Everything received is kept in received().
    bool pump(const std::function<bool()>& done, std::chrono::milliseconds timeout = 5000ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done()) {
            if (std::chrono::steady_clock::now() > deadline) {
                return false;
            }
            std::error_code ec;
            if (connection_.state() == Connection::State::Closed && listener_.valid()) {
                if (auto accepted = listener_.accept(ec)) {
                    connection_ = std::move(*accepted);
                }
            }
            if (connection_.state() == Connection::State::Connected) {
                (void)connection_.receive(ec);
                for (auto frame = connection_.next();
                     frame.status == lagrange::proto::FrameReader::Status::Frame;
                     frame = connection_.next()) {
                    v1::Envelope envelope;
                    if (lagrange::proto::parsePayload(frame.payload, envelope)) {
                        received_.push_back(std::move(envelope));
                    }
                }
            }
            if (connection_.state() == Connection::State::Connected) {
                (void)connection_.flush(ec);
            }
            std::this_thread::sleep_for(1ms);
        }
        return true;
    }

    [[nodiscard]] const std::vector<v1::Envelope>& received() const { return received_; }
    [[nodiscard]] int count(v1::Envelope::BodyCase body) const {
        int n = 0;
        for (const v1::Envelope& e : received_) {
            n += e.body_case() == body ? 1 : 0;
        }
        return n;
    }
    [[nodiscard]] int countCommands(v1::Command::CommandCase kind) const {
        int n = 0;
        for (const v1::Envelope& e : received_) {
            n += e.body_case() == v1::Envelope::kCommand && e.command().command_case() == kind ? 1
                                                                                               : 0;
        }
        return n;
    }

    /// Accepts the client, answers its hello, and waits for its snapshot request.
    void handshake() {
        ASSERT_TRUE(pump([&] { return count(v1::Envelope::kClientHello) > helloSeen_; }));
        ++helloSeen_;
        v1::Envelope hello;
        hello.mutable_server_hello()->set_server_name("fake server");
        hello.mutable_server_hello()->mutable_version()->set_minor(1);
        hello.mutable_server_hello()->mutable_capabilities()->set_can_seek(true);
        send(hello);
        const int before = countCommands(v1::Command::kRequestSnapshot);
        ASSERT_TRUE(pump([&] { return countCommands(v1::Command::kRequestSnapshot) > before; }));
    }

private:
    std::string directory_;
    Endpoint endpoint_;
    Listener listener_;
    Connection connection_;
    std::vector<v1::Envelope> received_;
    int helloSeen_{0};
};

v1::Envelope snapshot(std::uint64_t sequence, double x) {
    v1::Envelope envelope;
    v1::SceneSnapshot& s = *envelope.mutable_snapshot();
    s.set_sequence(sequence);
    auto* prim = s.add_prims();
    prim->set_id(0);
    prim->set_parent(kNone);
    prim->set_name("robot");
    prim->set_mesh(kNone);
    prim->set_material(kNone);
    std::string transform(52, '\0');
    const double position[3] = {x, 0.0, 0.0};
    const float rotationAndScale[7] = {1, 0, 0, 0, 1, 1, 1};
    std::memcpy(transform.data(), position, 24);
    std::memcpy(transform.data() + 24, rotationAndScale, 28);
    s.mutable_transforms()->set_inline_(transform);
    return envelope;
}

v1::Envelope delta(std::uint64_t base, double x) {
    v1::Envelope envelope;
    v1::SceneDelta& d = *envelope.mutable_delta();
    d.set_base_sequence(base);
    d.set_sequence(base + 1);
    d.add_transform_prims(0);
    std::string data(40, '\0');
    const double position[3] = {x, 0.0, 0.0};
    const float rotation[4] = {1, 0, 0, 0};
    std::memcpy(data.data(), position, 24);
    std::memcpy(data.data() + 24, rotation, 16);
    d.set_transform_data(data);
    return envelope;
}

// A catalog with a 10 kHz scalar "a" [A] and a 1 kHz three-component "b" [m].
v1::Envelope catalog(std::uint64_t generation) {
    v1::Envelope envelope;
    v1::SignalCatalog& c = *envelope.mutable_catalog();
    c.set_generation(generation);
    auto* a = c.add_signals();
    a->set_id(0);
    a->set_name("a");
    a->set_components(1);
    a->set_native_dt_ns(100'000);
    a->mutable_unit()->set_display("A");
    auto* b = c.add_signals();
    b->set_id(1);
    b->set_name("b");
    b->set_components(3);
    b->set_native_dt_ns(1'000'000);
    b->mutable_unit()->set_display("m");
    b->add_component_names("x");
    b->add_component_names("y");
    b->add_component_names("z");
    return envelope;
}

// One run of `count` samples per component; sample k of component c has value k + c / 10.
v1::Envelope signalBlock(std::uint64_t generation, std::uint32_t signal, std::uint32_t components,
                         std::int64_t dtNs, std::uint64_t firstIndex, std::uint32_t count) {
    v1::Envelope envelope;
    envelope.mutable_signal_block()->set_catalog_generation(generation);
    v1::SignalRun& run = *envelope.mutable_signal_block()->add_runs();
    run.set_signal(signal);
    run.set_dt_ns(dtNs);
    run.set_sample_count(count);
    run.set_components(components);
    run.set_encoding(v1::SAMPLE_ENCODING_F64_LE);
    run.set_mode(v1::SignalSubscription::MODE_FULL);
    run.set_first_index(firstIndex);
    run.mutable_t0()->set_ns(static_cast<std::int64_t>(firstIndex) * dtNs);
    std::string bytes(std::size_t{count} * components * sizeof(double), '\0');
    for (std::uint32_t k = 0; k < count; ++k) {
        for (std::uint32_t c = 0; c < components; ++c) {
            const double value = static_cast<double>(firstIndex + k) + c / 10.0;
            std::memcpy(bytes.data() + (std::size_t{k} * components + c) * sizeof(double), &value,
                        sizeof(double));
        }
    }
    run.set_samples(bytes);
    return envelope;
}

double sceneX(const ViewerClient& client) {
    const auto scene = client.scene();
    return scene ? scene->transforms[0].position[0] : -1.0;
}

TEST(ViewerClient, ConnectsShakesHandsAndFollowsTheScene) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    EXPECT_EQ(client.scene(), nullptr);
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());

    const v1::ClientHello& hello = server.received().front().client_hello();
    EXPECT_EQ(hello.version().major(), 0U);
    EXPECT_EQ(hello.version().minor(), 1U);
    EXPECT_EQ(client.connection().state, ConnectionState::Ready);
    EXPECT_EQ(client.connection().serverName, "fake server");
    EXPECT_TRUE(client.connection().canSeek);

    server.send(snapshot(10, 1.5));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 1.5; }));
    server.send(delta(10, 2.5));
    server.send(delta(11, 3.5));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 3.5; }));
    EXPECT_EQ(client.scene()->sequence, 12U);
    EXPECT_EQ(client.stats().snapshots, 1U);
    EXPECT_EQ(client.stats().deltas, 2U);
    EXPECT_EQ(client.stats().resyncs, 0U);
    client.stop();
}

TEST(ViewerClient, LostDeltaTriggersOneSnapshotRequestAndRecovers) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    server.send(snapshot(10, 1.0));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 1.0; }));

    // Delta 10 -> 11 is "lost": the client sees bases 11 and 12 without having 11.
    server.send(delta(11, 7.0));
    server.send(delta(12, 8.0));
    ASSERT_TRUE(
        server.pump([&] { return server.countCommands(v1::Command::kRequestSnapshot) == 2; }));
    EXPECT_EQ(sceneX(client), 1.0);         // nothing wrong was applied
    EXPECT_EQ(client.stats().resyncs, 1U);  // one request, although two deltas were unusable

    server.send(snapshot(20, 9.0));
    server.send(delta(20, 9.5));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 9.5; }));
    client.stop();
}

TEST(ViewerClient, RefusalIsReportedWithTheServersReason) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_TRUE(server.pump([&] { return server.count(v1::Envelope::kClientHello) == 1; }));
    v1::Envelope refusal;
    refusal.mutable_server_hello()->set_refusal("protocol version mismatch: 0.1 vs 3.0");
    server.send(refusal);
    ASSERT_TRUE(server.pump([&] { return client.connection().state == ConnectionState::Refused; }));
    EXPECT_EQ(client.connection().message, "protocol version mismatch: 0.1 vs 3.0");
    client.stop();
}

TEST(ViewerClient, DifferentMajorVersionIsRefusedByTheClient) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_TRUE(server.pump([&] { return server.count(v1::Envelope::kClientHello) == 1; }));
    v1::Envelope hello;
    hello.mutable_server_hello()->mutable_version()->set_major(3);
    server.send(hello);
    ASSERT_TRUE(server.pump([&] { return client.connection().state == ConnectionState::Refused; }));
    EXPECT_NE(client.connection().message.find("3.0"), std::string::npos);
    client.stop();
}

TEST(ViewerClient, CommandsStatusAndCatalogTravel) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());

    client.play();
    client.seek(2'500'000'000);
    client.step(10'000'000);
    client.setRealTimeFactor(0.5);
    client.pause();
    ASSERT_TRUE(server.pump([&] { return server.countCommands(v1::Command::kPause) == 1; }));
    EXPECT_EQ(server.countCommands(v1::Command::kPlay), 1);
    std::vector<std::uint64_t> ids;
    for (const v1::Envelope& e : server.received()) {
        if (e.body_case() != v1::Envelope::kCommand) {
            continue;
        }
        ids.push_back(e.command().request_id());
        if (e.command().command_case() == v1::Command::kSeek) {
            EXPECT_EQ(e.command().seek().to().ns(), 2'500'000'000);
        }
        if (e.command().command_case() == v1::Command::kSetRtf) {
            EXPECT_DOUBLE_EQ(e.command().set_rtf().target(), 0.5);
        }
    }
    for (std::size_t i = 1; i < ids.size(); ++i) {
        EXPECT_GT(ids[i], ids[i - 1]);  // unique and in order
    }

    v1::Envelope status;
    status.mutable_status()->mutable_sim_time()->set_ns(1234);
    status.mutable_status()->set_state(v1::RUN_STATE_RUNNING);
    status.mutable_status()->mutable_replay_end()->set_ns(10'000'000'000);
    server.send(status);
    server.send(catalog(1));
    ASSERT_TRUE(server.pump([&] { return client.status().signalCount == 2; }));
    EXPECT_EQ(client.status().simTimeNs, 1234);
    EXPECT_EQ(client.status().runState, RunState::Running);
    EXPECT_EQ(client.status().replayEndNs, 10'000'000'000);
    client.stop();
}

TEST(ViewerClient, ReconnectsAfterTheServerRestarts) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    server.send(snapshot(1, 1.0));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 1.0; }));

    server.shutdown();
    ASSERT_TRUE(
        server.pump([&] { return client.connection().state == ConnectionState::Disconnected; }));
    EXPECT_FALSE(client.connection().message.empty());
    EXPECT_EQ(sceneX(client), 1.0);  // the last scene stays visible while disconnected

    server.listen();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_EQ(client.connection().state, ConnectionState::Ready);
    EXPECT_EQ(client.stats().connects, 2U);
    server.send(snapshot(1, 4.0));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 4.0; }));
    client.stop();
}

TEST(ViewerClient, SubscribedSignalFillsItsBufferAndUnknownOnesAreRefused) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_EQ(client.catalog(), nullptr);
    EXPECT_EQ(client.subscribe("a"), nullptr);  // no catalog yet

    server.send(catalog(1));
    ASSERT_TRUE(server.pump([&] { return client.catalog() != nullptr; }));
    ASSERT_EQ(client.catalog()->signals.size(), 2U);
    EXPECT_EQ(client.catalog()->find("b")->unit, "m");
    EXPECT_EQ(client.catalog()->find("b")->componentNames[2], "z");
    EXPECT_EQ(client.subscribe("no/such/signal"), nullptr);
    EXPECT_EQ(client.subscribe("b", 3), nullptr);  // component out of range

    const auto a = client.subscribe("a");
    const auto bz = client.subscribe("b", 2);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(bz, nullptr);
    EXPECT_EQ(a->dtNs(), 100'000);
    EXPECT_EQ(bz->dtNs(), 1'000'000);

    // The server is told which signal ids are wanted, for the current catalog.
    const auto lastSubscribe = [&]() -> const v1::Subscribe* {
        const v1::Subscribe* last = nullptr;
        for (const v1::Envelope& e : server.received()) {
            if (e.body_case() == v1::Envelope::kSubscribe) {
                last = &e.subscribe();
            }
        }
        return last;
    };
    ASSERT_TRUE(server.pump([&] {
        const v1::Subscribe* s = lastSubscribe();
        return s != nullptr && s->signals_size() == 2;
    }));
    EXPECT_EQ(lastSubscribe()->catalog_generation(), 1U);
    EXPECT_EQ(lastSubscribe()->signals(0).signal(), 0U);
    EXPECT_EQ(lastSubscribe()->signals(1).signal(), 1U);
    EXPECT_EQ(lastSubscribe()->signals(0).mode(), v1::SignalSubscription::MODE_FULL);

    server.send(signalBlock(1, 0, 1, 100'000, 500, 200));
    server.send(signalBlock(1, 1, 3, 1'000'000, 50, 20));
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 200 && bz->totalSamples() == 20; }));
    EXPECT_DOUBLE_EQ(a->latest(), 699.0);
    EXPECT_EQ(a->newestTimeNs(), 699 * 100'000);
    EXPECT_DOUBLE_EQ(bz->latest(), 69.2);  // component 2 of sample 69
    EXPECT_EQ(a->droppedSamples(), 0U);
    EXPECT_EQ(client.stats().samples, 220U);
    EXPECT_EQ(client.stats().signalBlocks, 2U);

    // A skipped run is visible as dropped samples.
    server.send(signalBlock(1, 0, 1, 100'000, 750, 10));
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 210; }));
    EXPECT_EQ(a->droppedSamples(), 50U);
    client.stop();
}

TEST(ViewerClient, MalformedSignalRunsAreRejected) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    server.send(catalog(1));
    ASSERT_TRUE(server.pump([&] { return client.catalog() != nullptr; }));
    const auto a = client.subscribe("a");
    ASSERT_NE(a, nullptr);
    ASSERT_TRUE(server.pump([&] { return server.count(v1::Envelope::kSubscribe) >= 2; }));

    v1::Envelope shortBytes = signalBlock(1, 0, 1, 100'000, 0, 100);
    shortBytes.mutable_signal_block()->mutable_runs(0)->set_sample_count(1000);  // claims more
    v1::Envelope wrongStep = signalBlock(1, 0, 1, 250'000, 0, 10);
    v1::Envelope wrongComponents = signalBlock(1, 0, 2, 100'000, 0, 10);
    v1::Envelope hugeCount = signalBlock(1, 0, 1, 100'000, 0, 1);
    hugeCount.mutable_signal_block()->mutable_runs(0)->set_sample_count(0xFFFFFFFFU);
    v1::Envelope unknownSignal = signalBlock(1, 99, 1, 100'000, 0, 10);
    v1::Envelope oldCatalog = signalBlock(7, 0, 1, 100'000, 0, 10);
    for (const v1::Envelope* bad :
         {&shortBytes, &wrongStep, &wrongComponents, &hugeCount, &unknownSignal, &oldCatalog}) {
        server.send(*bad);
    }
    server.send(signalBlock(1, 0, 1, 100'000, 0, 5));  // a good one, to know all were handled
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 5; }));
    EXPECT_EQ(client.stats().invalidRuns, 4U);  // unknown signal and old catalog are just skipped
    client.stop();
}

TEST(ViewerClient, SnapshotRestartsScopeHistoryAndReconnectResubscribes) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    server.send(catalog(1));
    ASSERT_TRUE(server.pump([&] { return client.catalog() != nullptr; }));
    const auto a = client.subscribe("a");
    ASSERT_NE(a, nullptr);
    // A server only sends what was subscribed, so wait until the request has arrived.
    ASSERT_TRUE(server.pump([&] { return server.count(v1::Envelope::kSubscribe) >= 2; }));
    server.send(signalBlock(1, 0, 1, 100'000, 0, 100));
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 100; }));

    // A seek forward: snapshot, then samples from a later index. Not a gap.
    server.send(snapshot(5, 0.0));
    server.send(signalBlock(1, 0, 1, 100'000, 40'000, 10));
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 110; }));
    EXPECT_EQ(a->droppedSamples(), 0U);
    EXPECT_EQ(a->oldestTimeNs(), 40'000 * 100'000LL);

    // The server restarts with a catalog that numbers the same signals differently.
    server.shutdown();
    ASSERT_TRUE(
        server.pump([&] { return client.connection().state == ConnectionState::Disconnected; }));
    server.listen();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    v1::Envelope renumbered = catalog(2);
    renumbered.mutable_catalog()->mutable_signals(0)->set_name("b");
    renumbered.mutable_catalog()->mutable_signals(0)->set_components(3);
    renumbered.mutable_catalog()->mutable_signals(0)->set_native_dt_ns(1'000'000);
    renumbered.mutable_catalog()->mutable_signals(1)->set_name("a");
    renumbered.mutable_catalog()->mutable_signals(1)->set_components(1);
    renumbered.mutable_catalog()->mutable_signals(1)->set_native_dt_ns(100'000);
    const int before = server.count(v1::Envelope::kSubscribe);
    server.send(renumbered);
    ASSERT_TRUE(server.pump([&] { return server.count(v1::Envelope::kSubscribe) > before; }));
    const v1::Subscribe& again = server.received().back().subscribe();
    ASSERT_EQ(again.signals_size(), 1);
    EXPECT_EQ(again.signals(0).signal(), 1U);  // "a" is id 1 now
    EXPECT_EQ(again.catalog_generation(), 2U);

    server.send(signalBlock(2, 1, 1, 100'000, 3, 7));
    ASSERT_TRUE(server.pump([&] { return a->totalSamples() == 117; }));
    EXPECT_DOUBLE_EQ(a->latest(), 9.0);

    // After unsubscribing, the buffer keeps its history and receives nothing more.
    const int unsubscribes = server.count(v1::Envelope::kUnsubscribe);
    client.unsubscribe(a);
    ASSERT_TRUE(
        server.pump([&] { return server.count(v1::Envelope::kUnsubscribe) > unsubscribes; }));
    server.send(signalBlock(2, 1, 1, 100'000, 10, 7));
    server.send(snapshot(9, 2.0));
    ASSERT_TRUE(server.pump([&] { return sceneX(client) == 2.0; }));
    EXPECT_EQ(a->totalSamples(), 117U);
    client.stop();
}

TEST(ViewerClient, OpenFileSendsTheRightCommandAndReportsTheOutcome) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_TRUE(client.notice().text.empty());

    client.openFile("/robots/arm.urdf");
    EXPECT_EQ(client.notice().text, "Loading /robots/arm.urdf ...");
    client.openFile("/runs/yesterday.mcap");
    ASSERT_TRUE(
        server.pump([&] { return server.countCommands(v1::Command::kOpenRecording) == 1; }));
    ASSERT_EQ(server.countCommands(v1::Command::kImportRobot), 1);
    std::uint64_t importId = 0;
    for (const v1::Envelope& e : server.received()) {
        if (e.body_case() == v1::Envelope::kCommand && e.command().has_import_robot()) {
            EXPECT_EQ(e.command().import_robot().path(), "/robots/arm.urdf");
            importId = e.command().request_id();
        }
        if (e.body_case() == v1::Envelope::kCommand && e.command().has_open_recording()) {
            EXPECT_EQ(e.command().open_recording().path(), "/runs/yesterday.mcap");
        }
    }

    const std::uint64_t before = client.notice().serial;
    v1::Envelope failed;
    failed.mutable_command_result()->set_request_id(importId);
    failed.mutable_command_result()->set_error("arm.urdf is not valid XML");
    server.send(failed);
    ASSERT_TRUE(server.pump([&] { return client.notice().serial > before; }));
    EXPECT_TRUE(client.notice().error);
    EXPECT_EQ(client.notice().text, "arm.urdf is not valid XML");

    v1::Envelope loaded;
    loaded.mutable_command_result()->set_ok(true);
    loaded.mutable_command_result()->set_imported_stage_path("/robots/arm.urdf");
    server.send(loaded);
    ASSERT_TRUE(server.pump([&] { return !client.notice().error; }));
    EXPECT_EQ(client.notice().text, "Loaded /robots/arm.urdf");
    client.stop();
}

TEST(ViewerClient, LogLinesFillTheConsoleBookAndOldOnesFallOut) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_TRUE(client.logs()->entries.empty());

    const auto send = [&](v1::LogLevel level, const char* text, std::int64_t ns) {
        v1::Envelope line;
        line.mutable_log()->set_level(level);
        line.mutable_log()->set_source("stage");
        line.mutable_log()->set_text(text);
        line.mutable_log()->mutable_sim_time()->set_ns(ns);
        server.send(line);
    };
    send(v1::LOG_LEVEL_INFO, "loaded", 0);
    send(v1::LOG_LEVEL_WARN, "slow", 2'000'000);
    send(v1::LOG_LEVEL_ERROR, "broken", 3'000'000);
    ASSERT_TRUE(server.pump([&] { return client.logs()->total == 3; }));
    const std::shared_ptr<const LogBook> book = client.logs();
    ASSERT_EQ(book->entries.size(), 3U);
    EXPECT_EQ(book->entries[1].level, LogLevel::Warn);
    EXPECT_EQ(book->entries[1].text, "slow");
    EXPECT_EQ(book->entries[1].source, "stage");
    EXPECT_EQ(book->entries[2].simTimeNs, 3'000'000);
    EXPECT_EQ(book->warnings, 1U);
    EXPECT_EQ(book->errors, 1U);
    EXPECT_TRUE(client.notice().text.empty());  // log lines go to the console, not the notice

    for (std::size_t i = 0; i < LogBook::kCapacity; ++i) {
        send(v1::LOG_LEVEL_INFO, "more", 4'000'000);
    }
    ASSERT_TRUE(server.pump([&] { return client.logs()->total == 3 + LogBook::kCapacity; }));
    EXPECT_EQ(client.logs()->entries.size(), LogBook::kCapacity);
    EXPECT_EQ(client.logs()->warnings, 0U);  // the warning and the error fell out
    EXPECT_EQ(client.logs()->errors, 0U);
    client.stop();
}

TEST(ViewerClient, AttributesOfAPrimAreRequestedAndDecoded) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_EQ(client.attributes(), nullptr);

    const std::uint64_t id = client.requestAttributes(8);
    ASSERT_TRUE(
        server.pump([&] { return server.countCommands(v1::Command::kGetAttributes) == 1; }));
    for (const v1::Envelope& e : server.received()) {
        if (e.body_case() == v1::Envelope::kCommand && e.command().has_get_attributes()) {
            ASSERT_EQ(e.command().get_attributes().prims_size(), 1);
            EXPECT_EQ(e.command().get_attributes().prims(0), 8U);
            EXPECT_EQ(e.command().request_id(), id);
        }
    }

    v1::Envelope answer;
    answer.mutable_command_result()->set_request_id(id);
    answer.mutable_command_result()->set_ok(true);
    v1::PrimAttributes* item = answer.mutable_command_result()->mutable_attributes()->add_items();
    item->set_prim(8);
    v1::Attribute* resistance = item->add_attributes();
    resistance->set_name("elec:windingResistance");
    resistance->set_real(0.12);
    resistance->mutable_unit()->set_display("ohm");
    v1::Attribute* model = item->add_attributes();
    model->set_name("elec:behaviorModel");
    model->set_asset("models/bldc_drive.fmu");
    v1::Attribute* port = item->add_attributes();
    port->set_name("powerPort");
    port->mutable_targets()->add_prims(7);
    v1::Attribute* ratio = item->add_attributes();
    ratio->set_name("thermal:emissivity");
    ratio->set_real(0.85);
    ratio->mutable_unit()->set_display("1");
    server.send(answer);

    ASSERT_TRUE(server.pump([&] { return client.attributes() != nullptr; }));
    const auto attributes = client.attributes();
    EXPECT_EQ(attributes->requestId, id);
    EXPECT_EQ(attributes->prim, 8U);
    ASSERT_EQ(attributes->attributes.size(), 4U);
    EXPECT_EQ(attributes->attributes[0].value, "0.12");
    EXPECT_EQ(attributes->attributes[0].unit, "ohm");
    EXPECT_EQ(attributes->attributes[0].group(), "elec");
    EXPECT_EQ(attributes->attributes[0].kind, AttributeInfo::Kind::Real);
    EXPECT_EQ(attributes->attributes[1].kind, AttributeInfo::Kind::Asset);
    EXPECT_EQ(attributes->attributes[1].value, "models/bldc_drive.fmu");
    EXPECT_EQ(attributes->attributes[2].kind, AttributeInfo::Kind::Targets);
    EXPECT_EQ(attributes->attributes[2].targets, std::vector<std::uint32_t>{7});
    EXPECT_TRUE(attributes->attributes[2].group().empty());
    EXPECT_TRUE(attributes->attributes[3].unit.empty());  // a pure number shows no unit
    client.stop();
}

TEST(ViewerClient, WatchedSensorReceivesPointCloudsAndRejectsMalformedOnes) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());

    const auto stream = client.watchSensor("/amr/sensors/lidar_front");
    ASSERT_NE(stream, nullptr);
    EXPECT_EQ(client.watchSensor("/amr/sensors/lidar_front"), stream);  // one stream per sensor
    EXPECT_EQ(stream->latest(), nullptr);
    ASSERT_TRUE(
        server.pump([&] { return server.countCommands(v1::Command::kSubscribeSensor) == 1; }));

    const auto frame = [](const std::string& sensor, std::uint32_t count, std::uint32_t stride,
                          const std::vector<float>& values) {
        v1::Envelope envelope;
        v1::SensorFrame* out = envelope.mutable_sensor_frame();
        out->set_sensor(sensor);
        out->mutable_sim_time()->set_ns(42);
        out->mutable_points()->set_count(count);
        out->mutable_points()->set_stride(stride);
        for (const char* field : {"x", "y", "z", "intensity"}) {
            out->mutable_points()->add_fields(field);
        }
        out->mutable_points()->mutable_data()->set_inline_(
            std::string(reinterpret_cast<const char*>(values.data()),  // NOLINT
                        values.size() * sizeof(float)));
        return envelope;
    };
    server.send(frame("/amr/sensors/lidar_front", 2, 16, {1, 2, 3, 0.5F, 4, 5, 6, 0.25F}));
    ASSERT_TRUE(server.pump([&] { return stream->frames() == 1; }));
    const auto cloud = stream->latest();
    ASSERT_NE(cloud, nullptr);
    EXPECT_EQ(cloud->simTimeNs, 42);
    EXPECT_EQ(cloud->count, 2U);
    EXPECT_EQ(cloud->points, (std::vector<float>{1, 2, 3, 0.5F, 4, 5, 6, 0.25F}));

    // A count that does not match the bytes, and a stride too small for four floats.
    server.send(frame("/amr/sensors/lidar_front", 5, 16, {1, 2, 3, 4}));
    server.send(frame("/amr/sensors/lidar_front", 1, 8, {1, 2}));
    // A sensor nobody watches is ignored without counting as an error.
    server.send(frame("/amr/sensors/other", 1, 16, {1, 2, 3, 4}));
    ASSERT_TRUE(server.pump([&] { return stream->rejected() == 2; }));
    EXPECT_EQ(stream->frames(), 1U);
    EXPECT_EQ(stream->latest(), cloud);  // the good frame is still the one shown
    client.stop();
}

TEST(ViewerClient, EventsStatisticsAndRecordingFactsArePublished) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    EXPECT_TRUE(client.events()->events.empty());
    EXPECT_EQ(client.statistics(), nullptr);
    EXPECT_EQ(client.recording(), nullptr);

    const auto event = [](std::uint64_t id, std::int64_t begin, std::int64_t end) {
        v1::Event out;
        out.set_id(id);
        out.mutable_begin()->set_ns(begin);
        out.mutable_end()->set_ns(end);
        out.set_domain("electrical");
        out.set_kind("regen");
        out.set_severity(v1::Event::SEVERITY_WARNING);
        out.set_text("regenerating");
        out.set_prim(6);
        out.set_signal("battery/current");
        return out;
    };
    v1::Envelope list;
    *list.mutable_event_list()->add_events() = event(2, 5'000, 9'000);
    *list.mutable_event_list()->add_events() = event(1, 1'000, 500);  // end before begin
    list.mutable_event_list()->set_complete(true);
    server.send(list);
    v1::Envelope live;
    *live.mutable_event() = event(3, 3'000, 3'000);
    server.send(live);
    ASSERT_TRUE(server.pump([&] { return client.events()->events.size() == 3; }));
    const auto log = client.events();
    EXPECT_TRUE(log->complete);
    EXPECT_EQ(log->events[0].id, 1U);  // ordered by begin time, whatever order they came in
    EXPECT_EQ(log->events[1].id, 3U);
    EXPECT_EQ(log->events[2].id, 2U);
    EXPECT_EQ(log->events[0].endNs, 1'000);  // a negative span is made an instant
    EXPECT_EQ(log->events[2].severity, lagrange::viewer::EventInfo::Severity::Warning);
    EXPECT_EQ(log->events[2].kind, "regen");
    EXPECT_EQ(log->events[2].signal, "battery/current");
    EXPECT_EQ(log->events[2].prim, 6U);

    v1::Envelope stats;
    stats.mutable_server_stats()->set_real_time_factor(0.98);
    stats.mutable_server_stats()->set_tick_ms(1.5);
    stats.mutable_server_stats()->set_memory_bytes(123'456'789);
    stats.mutable_server_stats()->set_estimated(true);
    v1::ServerStats::Module* module = stats.mutable_server_stats()->add_modules();
    module->set_module("rigid_body/mujoco");
    module->set_domain("rigid_body");
    module->set_dt_ns(1'000'000);
    module->set_cpu_ms_per_step(0.31);
    server.send(stats);
    ASSERT_TRUE(server.pump([&] { return client.statistics() != nullptr; }));
    EXPECT_DOUBLE_EQ(client.statistics()->realTimeFactor, 0.98);
    EXPECT_EQ(client.statistics()->memoryBytes, 123'456'789U);
    EXPECT_TRUE(client.statistics()->estimated);
    ASSERT_EQ(client.statistics()->modules.size(), 1U);
    EXPECT_EQ(client.statistics()->modules[0].module, "rigid_body/mujoco");
    EXPECT_DOUBLE_EQ(client.statistics()->modules[0].cpuMsPerStep, 0.31);

    v1::Envelope info;
    info.mutable_recording_info()->set_path("/runs/run_001.mcap");
    info.mutable_recording_info()->mutable_duration()->set_ns(252'000'000'000);
    info.mutable_recording_info()->set_size_bytes(1'820'000'000);
    info.mutable_recording_info()->set_compression("zstd");
    v1::RecordingInfo::Channel* channel = info.mutable_recording_info()->add_channels();
    channel->set_topic("bus/dc_48v/voltage");
    channel->set_rate_hz(10'000.0);
    channel->set_messages(2'520'000);
    server.send(info);
    ASSERT_TRUE(server.pump([&] { return client.recording() != nullptr; }));
    EXPECT_EQ(client.recording()->path, "/runs/run_001.mcap");
    EXPECT_EQ(client.recording()->durationNs, 252'000'000'000);
    EXPECT_EQ(client.recording()->compression, "zstd");
    ASSERT_EQ(client.recording()->channels.size(), 1U);
    EXPECT_DOUBLE_EQ(client.recording()->channels[0].rateHz, 10'000.0);
    client.stop();
}

TEST(ViewerClient, ProfileValidationCarriesTheWiring) {
    FakeServer server;
    ViewerClient client(server.endpoint());
    client.start();
    ASSERT_NO_FATAL_FAILURE(server.handshake());
    const std::uint64_t id = client.validateProfile("domains: {}\n");
    ASSERT_TRUE(
        server.pump([&] { return server.countCommands(v1::Command::kValidateProfile) == 1; }));

    v1::Envelope answer;
    answer.mutable_command_result()->set_request_id(id);
    answer.mutable_command_result()->set_ok(true);
    v1::ProfileValidation* validation = answer.mutable_command_result()->mutable_validation();
    validation->set_valid(true);
    v1::Wiring* wiring = validation->mutable_wiring();
    wiring->set_base_tick_ns(100'000);
    wiring->set_sync_ns(1'000'000);
    for (const char* name : {"electrical", "rigid_body"}) {
        v1::Wiring::Node* node = wiring->add_nodes();
        node->set_module(name);
        node->set_domain(name);
        node->set_dt_ns(1'000'000);
    }
    wiring->mutable_nodes(0)->add_outputs("τ");
    wiring->mutable_nodes(1)->add_inputs("τ");
    v1::Wiring::Wire* wire = wiring->add_wires();
    wire->set_from("electrical");
    wire->set_to("rigid_body");
    wire->set_symbol("τ");
    wire->set_source_unit("N·m");
    wire->set_sink_unit("N");
    wire->set_unit_ok(false);
    v1::Wiring::Wire* dangling = wiring->add_wires();
    dangling->set_from("electrical");
    dangling->set_to("nowhere");
    server.send(answer);

    ASSERT_TRUE(server.pump([&] { return client.profileCheck() != nullptr; }));
    const auto check = client.profileCheck();
    EXPECT_EQ(check->requestId, id);
    ASSERT_EQ(check->wiring.nodes.size(), 2U);
    EXPECT_EQ(check->wiring.nodes[0].outputs, std::vector<std::string>{"τ"});
    ASSERT_EQ(check->wiring.wires.size(), 1U);  // the wire to an unknown module is dropped
    EXPECT_EQ(check->wiring.wires[0].sourceUnit, "N·m");
    EXPECT_EQ(check->wiring.wires[0].sinkUnit, "N");
    EXPECT_FALSE(check->wiring.wires[0].unitOk);
    EXPECT_EQ(check->wiring.baseTickNs, 100'000);
    client.stop();
}

TEST(ViewerClient, StartsAndStopsCleanlyWithNoServer) {
    ViewerClient client(Endpoint::unixSocket("/tmp/lagrange-no-such-server.sock"));
    client.start();
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(client.connection().state, ConnectionState::Disconnected);
    EXPECT_EQ(client.scene(), nullptr);
    client.play();  // queued, never sent; must not block or crash
    client.stop();
    client.stop();  // idempotent
}

}  // namespace
