// RFC 0014 end to end: the server's limits on connections before the handshake, and the QA
// finding it answers (docs/plan/qa-2026-11-12.md #7) as a test. A binary of its own, so that
// test_server does not grow.
#include "lagrange/proto/Connection.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

namespace v1 = lagrange::viewer::v1;

// RFC 0014: limits on connections before the handshake, a handshake timeout, a per-address rate
// of new TCP connections, and a first frame that is not a ClientHello closing the connection.
class ConnectionLimitsTest : public ::testing::Test {
protected:
    void start(proto::Endpoint endpoint, const ServerOptions::ConnectionLimits& limits,
               const std::string& token = "") {
        char made[] = "/tmp/lagrange-limits-test-XXXXXX";
        ASSERT_NE(::mkdtemp(made), nullptr);
        directory_ = made;
        ServerOptions options;
        options.endpoint = endpoint.kind == proto::Endpoint::Kind::Unix
                               ? proto::Endpoint::unixSocket(directory_ + "/s.sock")
                               : endpoint;
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.ros = false;
        options.startPaused = true;
        options.connectionLimits = limits;
        options.authToken = token;
        server_ = std::make_unique<Server>(options);
        const Status started = server_->start();
        ASSERT_TRUE(started.ok()) << started.message;
    }
    void TearDown() override {
        server_.reset();
        if (!directory_.empty()) {
            std::filesystem::remove_all(directory_);
        }
    }

    proto::Connection connect() {
        std::error_code ec;
        proto::Connection client = proto::Connection::startConnect(server_->endpoint(), ec);
        EXPECT_NE(client.state(), proto::Connection::State::Closed) << ec.message();
        return client;
    }
    static void sendHello(proto::Connection& client, const std::string& token = "") {
        v1::Envelope hello;
        hello.mutable_client_hello()->mutable_version()->set_major(0);
        hello.mutable_client_hello()->mutable_version()->set_minor(1);
        hello.mutable_client_hello()->set_auth_token(token);
        ASSERT_TRUE(client.queue(hello));
        std::error_code ec;
        (void)client.flush(ec);  // on its way before the server looks, as a viewer's is
    }

    struct Outcome {
        bool hello{false};  // a ServerHello came
        std::string refusal;
        bool closed{false};  // the server closed the connection
    };
    /// Runs the server until `client` gets a ServerHello or is closed, or `seconds` pass.
    Outcome await(proto::Connection& client, double seconds = 3.0) {
        Outcome outcome;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(seconds));
        while (std::chrono::steady_clock::now() < deadline) {
            server_->runOnce(1);
            std::error_code ec;
            if (client.state() == proto::Connection::State::Connecting) {
                (void)client.finishConnect(ec);
            }
            (void)client.flush(ec);
            const proto::IoStatus io = client.receive(ec);
            for (;;) {
                const proto::FrameReader::Result frame = client.next();
                if (frame.status != proto::FrameReader::Status::Frame) {
                    break;
                }
                v1::Envelope envelope;
                if (proto::parsePayload(frame.payload, envelope) && envelope.has_server_hello()) {
                    outcome.hello = true;
                    outcome.refusal = envelope.server_hello().refusal();
                    return outcome;
                }
            }
            if (io == proto::IoStatus::Closed || io == proto::IoStatus::Error) {
                outcome.closed = true;
                return outcome;
            }
        }
        return outcome;
    }
    void pump(double seconds) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(seconds));
        while (std::chrono::steady_clock::now() < end) {
            server_->runOnce(1);
        }
    }

    /// Reads what has arrived for a greeted viewer, keeping the latest ServerStats; false when
    /// the server has closed it.
    static bool drain(proto::Connection& viewer, std::optional<v1::ServerStats>& stats) {
        std::error_code ec;
        (void)viewer.flush(ec);
        const proto::IoStatus io = viewer.receive(ec);
        for (;;) {
            const proto::FrameReader::Result frame = viewer.next();
            if (frame.status != proto::FrameReader::Status::Frame) {
                break;
            }
            v1::Envelope envelope;
            if (proto::parsePayload(frame.payload, envelope) && envelope.has_server_stats()) {
                stats = envelope.server_stats();
            }
        }
        return io != proto::IoStatus::Closed && io != proto::IoStatus::Error;
    }
    /// Runs the server until the greeted `viewer` gets a ServerStats it did not have before.
    v1::ServerStats nextStats(proto::Connection& viewer, double seconds = 3.0) {
        std::optional<v1::ServerStats> stats;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(seconds));
        while (std::chrono::steady_clock::now() < deadline) {
            server_->runOnce(1);
            if (!drain(viewer, stats)) {
                ADD_FAILURE() << "the server closed the viewer";
                return {};
            }
            if (stats) {
                return *stats;
            }
        }
        ADD_FAILURE() << "no ServerStats within " << seconds << " s";
        return {};
    }
    /// A greeted viewer, its first ServerHello read.
    proto::Connection greetedViewer(const std::string& token = "") {
        proto::Connection viewer = connect();
        sendHello(viewer, token);
        const Outcome greeted = await(viewer);
        EXPECT_TRUE(greeted.hello);
        EXPECT_EQ(greeted.refusal, "");
        return viewer;
    }

    std::string directory_;
    std::unique_ptr<Server> server_;
};

TEST_F(ConnectionLimitsTest, SilentConnectionsAreCappedThenTimedOutAndAViewerGetsIn) {
    ServerOptions::ConnectionLimits limits;
    limits.maxPending = 3;
    limits.handshakeTimeoutS = 0.5;
    start(proto::Endpoint::unixSocket(""), limits);

    std::vector<proto::Connection> silent;
    for (int i = 0; i < 3; ++i) {
        silent.push_back(connect());
    }
    pump(0.05);
    // A fourth that says nothing is refused at once: three are already waiting.
    proto::Connection fourth = connect();
    const Outcome refused = await(fourth, 0.3);
    ASSERT_TRUE(refused.hello);
    EXPECT_NE(refused.refusal.find("waiting for their handshake"), std::string::npos)
        << refused.refusal;

    // After the timeout the silent ones are told and closed.
    for (proto::Connection& client : silent) {
        const Outcome timedOut = await(client, 2.0);
        EXPECT_TRUE(timedOut.hello || timedOut.closed);
        if (timedOut.hello) {
            EXPECT_NE(timedOut.refusal.find("handshake timeout"), std::string::npos)
                << timedOut.refusal;
        }
    }
    // A real viewer is greeted again.
    proto::Connection viewer = connect();
    sendHello(viewer);
    const Outcome greeted = await(viewer);
    ASSERT_TRUE(greeted.hello);
    EXPECT_EQ(greeted.refusal, "");
}

TEST_F(ConnectionLimitsTest, GreetedViewersCountTowardsTheConnectionLimit) {
    ServerOptions::ConnectionLimits limits;
    limits.maxConnections = 2;
    start(proto::Endpoint::unixSocket(""), limits);
    std::vector<proto::Connection> viewers;
    for (int i = 0; i < 2; ++i) {
        viewers.push_back(connect());
        sendHello(viewers.back());
        const Outcome greeted = await(viewers.back());
        ASSERT_TRUE(greeted.hello);
        EXPECT_EQ(greeted.refusal, "");
    }
    proto::Connection third = connect();
    sendHello(third);
    const Outcome refused = await(third);
    ASSERT_TRUE(refused.hello);
    EXPECT_NE(refused.refusal.find("too many viewer connections"), std::string::npos)
        << refused.refusal;
    // One leaves: the next is greeted.
    viewers.front().close();
    pump(0.05);
    proto::Connection next = connect();
    sendHello(next);
    const Outcome greeted = await(next);
    ASSERT_TRUE(greeted.hello);
    EXPECT_EQ(greeted.refusal, "");
}

TEST_F(ConnectionLimitsTest, AFirstFrameThatIsNotAMessageClosesTheConnection) {
    start(proto::Endpoint::unixSocket(""), {});
    proto::Connection client = connect();
    // A frame whose payload is no protobuf message (a field number 0 is invalid).
    const std::array<std::byte, 4> junk{std::byte{0x00}, std::byte{0x00}, std::byte{0xFF},
                                        std::byte{0xFF}};
    std::vector<std::byte> frame;
    ASSERT_TRUE(proto::appendFrame(junk, frame));
    ASSERT_EQ(::write(client.fd(), frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));
    const Outcome outcome = await(client);
    ASSERT_TRUE(outcome.hello);
    EXPECT_NE(outcome.refusal.find("ClientHello"), std::string::npos) << outcome.refusal;
}

TEST_F(ConnectionLimitsTest, ATcpAddressOpensItsBurstThenWaitsForTheRate) {
    ServerOptions::ConnectionLimits limits;
    limits.peerBurst = 3.0;
    limits.peerRatePerS = 4.0;  // a token every 0.25 s
    start(proto::Endpoint::tcp("127.0.0.1", 0, false), limits);
    std::vector<proto::Connection> burst;
    for (int i = 0; i < 3; ++i) {
        burst.push_back(connect());
        sendHello(burst.back());
        const Outcome greeted = await(burst.back());
        ASSERT_TRUE(greeted.hello);
        EXPECT_EQ(greeted.refusal, "") << i;
    }
    proto::Connection fourth = connect();
    sendHello(fourth);
    const Outcome refused = await(fourth);
    EXPECT_TRUE(refused.closed || refused.refusal.find("from this address") != std::string::npos)
        << refused.refusal;
    pump(0.4);  // a token comes back
    proto::Connection later = connect();
    sendHello(later);
    const Outcome greeted = await(later);
    ASSERT_TRUE(greeted.hello);
    EXPECT_EQ(greeted.refusal, "");
}

// RFC 0014 §1 and open question 4: connections still waiting for their handshake are not
// counted as viewers.
TEST_F(ConnectionLimitsTest, OnlyGreetedSessionsCountAsViewers) {
    start(proto::Endpoint::unixSocket(""), {});
    proto::Connection viewer = greetedViewer();
    std::vector<proto::Connection> silent;
    for (int i = 0; i < 3; ++i) {
        silent.push_back(connect());
    }
    (void)nextStats(viewer);  // the first may have been made before the silent ones came
    const v1::ServerStats stats = nextStats(viewer);
    EXPECT_EQ(stats.viewers(), 1U);
}

// RFC 0014 §1: a greeted viewer may send frames up to the normal 64 MiB, a stranger only
// 64 KiB; frames pipelined behind the ClientHello are read under the raised limit.
TEST_F(ConnectionLimitsTest, TheFrameLimitIsRaisedByTheHandshake) {
    start(proto::Endpoint::unixSocket(""), {});
    // A ClientHello and, in the same write, a frame over 64 KiB (an unknown field the server
    // ignores): the connection stays and is greeted.
    proto::Connection viewer = connect();
    v1::Envelope hello;
    hello.mutable_client_hello()->mutable_version()->set_major(0);
    hello.mutable_client_hello()->mutable_version()->set_minor(1);
    ASSERT_TRUE(viewer.queue(hello));
    v1::Envelope ping;
    ping.mutable_ping()->set_nonce(7);
    std::string padded;
    ASSERT_TRUE(ping.SerializeToString(&padded));
    // Field 1000, length-delimited, 100 KiB: unknown to v1, so parsed and skipped.
    const std::size_t padding = 100U * 1024U;
    padded += std::string{"\xC2\x3E"};      // tag (1000 << 3 | 2) as a varint
    padded += std::string{"\x80\xA0\x06"};  // 100 KiB = 102400 as a varint
    padded.append(padding, 'x');
    std::vector<std::byte> frame;
    ASSERT_TRUE(proto::appendFrame(std::as_bytes(std::span{padded}), frame));
    ASSERT_GT(frame.size(), proto::kPreHandshakeMaxFrameSize);
    std::error_code ec;
    (void)viewer.flush(ec);
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (written < frame.size() && std::chrono::steady_clock::now() < deadline) {
        const ssize_t n = ::send(viewer.fd(), frame.data() + written, frame.size() - written,
                                 MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
        }
        server_->runOnce(1);
    }
    ASSERT_EQ(written, frame.size());
    const Outcome greeted = await(viewer);
    ASSERT_TRUE(greeted.hello);
    EXPECT_EQ(greeted.refusal, "");
    std::optional<v1::ServerStats> stats;
    pump(0.2);
    EXPECT_TRUE(drain(viewer, stats)) << "a large frame after the handshake closed the viewer";

    // The same large frame from a stranger: refused at its header and closed.
    proto::Connection stranger = connect();
    ASSERT_EQ(::send(stranger.fd(), frame.data(), proto::kFrameHeaderSize, MSG_NOSIGNAL),
              static_cast<ssize_t>(proto::kFrameHeaderSize));
    const Outcome refused = await(stranger);
    ASSERT_TRUE(refused.hello);
    EXPECT_NE(refused.refusal.find("before the handshake"), std::string::npos) << refused.refusal;
}

// QA 2026-11-12 #7, part 1, as a test: 8 clients announce 60 MiB frames and send zeros, with
// no ClientHello. Each is refused at its header; the server never buffers their payloads
// (before RFC 0014 its resident memory grew by what they sent, 85 MB to 626 MB in the QA run).
TEST_F(ConnectionLimitsTest, LargeFramesBeforeTheHandshakeAreRefusedAndMemoryStaysFlat) {
    start(proto::Endpoint::unixSocket(""), {});
    constexpr std::size_t kAttackers = 8;
    constexpr std::uint32_t kAnnounced = 60U * 1024U * 1024U;
    constexpr std::size_t kPayload = 8U * 1024U * 1024U;  // sent of each announced frame
    constexpr std::uint64_t kAllowedGrowth = 10U * 1024U * 1024U;
    std::vector<std::byte> stream(proto::kFrameHeaderSize + kPayload, std::byte{0});
    for (std::size_t i = 0; i < proto::kFrameHeaderSize; ++i) {
        stream[i] = static_cast<std::byte>((kAnnounced >> (8U * i)) & 0xFFU);
    }

    proto::Connection viewer = greetedViewer();
    (void)nextStats(viewer);
    const std::uint64_t before = nextStats(viewer).memory_bytes();
    ASSERT_GT(before, 0U);

    std::vector<proto::Connection> attackers;
    std::vector<std::size_t> written(kAttackers, 0);
    std::vector<bool> stopped(kAttackers, false);
    for (std::size_t i = 0; i < kAttackers; ++i) {
        attackers.push_back(connect());
    }
    std::optional<v1::ServerStats> stats;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline &&
           std::count(stopped.begin(), stopped.end(), false) > 0) {
        for (std::size_t i = 0; i < kAttackers; ++i) {
            if (stopped[i]) {
                continue;
            }
            const ssize_t n = ::send(attackers[i].fd(), stream.data() + written[i],
                                     stream.size() - written[i], MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0) {
                written[i] += static_cast<std::size_t>(n);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                stopped[i] = true;  // EPIPE or ECONNRESET: the server closed it
            }
            stopped[i] = stopped[i] || written[i] == stream.size();
        }
        server_->runOnce(1);
        ASSERT_TRUE(drain(viewer, stats));
    }
    // Measured while the attackers are still connected (without the cap the server held
    // their 64 MiB until the handshake timeout let them go).
    const v1::ServerStats after = nextStats(viewer);
    EXPECT_LT(after.memory_bytes(), before + kAllowedGrowth)
        << "resident memory went from " << before << " to " << after.memory_bytes() << " bytes";
    EXPECT_EQ(after.viewers(), 1U);
    for (std::size_t i = 0; i < kAttackers; ++i) {
        // Closed at its header: the server read at most two receive budgets (1 MiB each) and
        // the socket buffers held the rest; nowhere near the 8 MiB offered.
        EXPECT_LT(written[i], 4U * 1024U * 1024U) << "attacker " << i;
        const Outcome outcome = await(attackers[i]);
        EXPECT_TRUE(outcome.hello || outcome.closed) << "attacker " << i;
        if (outcome.hello) {
            EXPECT_NE(outcome.refusal.find("before the handshake"), std::string::npos)
                << outcome.refusal;
        }
    }
}

// QA 2026-11-12 #7, part 2, as a test: 3000 connections in a loop (the QA run had all 3000
// accepted). At most maxPending wait at once, every other one is refused, the server's memory
// stays flat, and a real viewer is greeted at the end. On a Unix socket, then on TCP with a
// token (which a stranger does not have: the limits act before the token is checked).
class ConnectionFloodTest : public ConnectionLimitsTest,
                            public ::testing::WithParamInterface<bool> {};

TEST_P(ConnectionFloodTest, ThousandsOfConnectionsAreRefusedAndAViewerStillGetsIn) {
    const bool tcp = GetParam();
    const std::string token = tcp ? "flood-test-token" : "";
    const ServerOptions::ConnectionLimits limits;  // the defaults: 16 waiting, 32 open
    start(tcp ? proto::Endpoint::tcp("127.0.0.1", 0, false) : proto::Endpoint::unixSocket(""),
          limits, token);
    constexpr int kConnections = 3000;
    constexpr std::uint64_t kAllowedGrowth = 10U * 1024U * 1024U;

    proto::Connection monitor = greetedViewer(token);
    (void)nextStats(monitor);
    const std::uint64_t before = nextStats(monitor).memory_bytes();

    std::optional<v1::ServerStats> stats;
    std::vector<proto::Connection> waiting;  // neither refused nor closed (yet)
    std::size_t mostWaiting = 0;
    int refused = 0;
    int retries = 0;
    // Whether the server has answered `client`: a refusal or a close.
    const auto answered = [](proto::Connection& client) {
        std::error_code ec;
        if (client.state() == proto::Connection::State::Connecting) {
            (void)client.finishConnect(ec);  // TCP: non-blocking connect
        }
        const proto::IoStatus io = client.receive(ec);
        const proto::FrameReader::Result frame = client.next();
        v1::Envelope envelope;
        if (frame.status == proto::FrameReader::Status::Frame &&
            proto::parsePayload(frame.payload, envelope) && envelope.has_server_hello()) {
            EXPECT_NE(envelope.server_hello().refusal(), "");
            return true;
        }
        return io == proto::IoStatus::Closed || io == proto::IoStatus::Error;
    };
    for (int i = 0; i < kConnections; ++i) {
        std::error_code ec;
        proto::Connection client = proto::Connection::startConnect(server_->endpoint(), ec);
        if (client.state() == proto::Connection::State::Closed) {
            // The listen backlog is full: let the server accept, as a real flood would wait.
            ++retries;
            ASSERT_LT(retries, 100000) << ec.message();
            server_->runOnce(0);
            --i;
            continue;
        }
        server_->runOnce(0);
        ASSERT_TRUE(drain(monitor, stats)) << "the flood closed the viewer";
        if (answered(client)) {
            ++refused;
        } else {
            waiting.push_back(std::move(client));
        }
        if (i % 50 == 0) {
            std::erase_if(waiting, answered);  // admitted earlier, answered since (a timeout)
        }
        mostWaiting = std::max(mostWaiting, waiting.size());
    }
    pump(0.1);
    std::erase_if(waiting, answered);
    // Admitted silent connections are those that came while fewer than maxPending waited: the
    // ones a client sees unanswered can only be those (the rest were answered at accept).
    EXPECT_LE(mostWaiting, limits.maxPending);
    EXPECT_GE(refused, kConnections - static_cast<int>(limits.maxPending));

    const v1::ServerStats during = nextStats(monitor);
    EXPECT_EQ(during.viewers(), 1U);
    EXPECT_LT(during.memory_bytes(), before + kAllowedGrowth)
        << "resident memory went from " << before << " to " << during.memory_bytes() << " bytes";

    // The waiting ones leave; a real viewer gets in (on TCP once the address has a token).
    waiting.clear();
    pump(tcp ? 0.6 : 0.1);
    proto::Connection viewer = connect();
    sendHello(viewer, token);
    const Outcome greeted = await(viewer);
    ASSERT_TRUE(greeted.hello);
    EXPECT_EQ(greeted.refusal, "");
    ASSERT_TRUE(drain(monitor, stats));
}

std::string transportName(const ::testing::TestParamInfo<bool>& param) {
    return param.param ? "Tcp" : "Unix";
}

INSTANTIATE_TEST_SUITE_P(UnixAndTcp, ConnectionFloodTest, ::testing::Values(false, true),
                         transportName);

}  // namespace
}  // namespace lagrange::server
