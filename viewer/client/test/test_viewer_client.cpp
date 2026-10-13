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
using lagrange::viewer::ConnectionState;
using lagrange::viewer::kNone;
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
    v1::Envelope catalog;
    catalog.mutable_catalog()->add_signals()->set_name("a");
    catalog.mutable_catalog()->add_signals()->set_name("b");
    server.send(catalog);
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
