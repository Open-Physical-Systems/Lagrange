#include "lagrange/proto/Connection.hpp"
#include "lagrange/proto/Shm.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509v3.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace {

using namespace std::chrono_literals;
using lagrange::proto::Backoff;
using lagrange::proto::Connection;
using lagrange::proto::Endpoint;
using lagrange::proto::FrameReader;
using lagrange::proto::IoStatus;
using lagrange::proto::Listener;
using lagrange::proto::parseEndpoint;
using lagrange::proto::parsePayload;
using lagrange::proto::ShmArena;
using lagrange::proto::ShmReader;
using lagrange::proto::ShmSlot;
using lagrange::proto::TlsContext;
namespace v1 = lagrange::viewer::v1;

// A private directory for socket files, removed afterwards.
class TempDir {
public:
    TempDir() {
        std::string pattern = (std::filesystem::temp_directory_path() / "lgr-XXXXXX").string();
        const char* created = ::mkdtemp(pattern.data());
        EXPECT_NE(created, nullptr);
        path_ = pattern;
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    [[nodiscard]] std::string file(const std::string& name) const { return path_ + "/" + name; }

private:
    std::string path_;
};

// Drives a set of connections on one thread until `done` returns true.
bool pump(std::vector<Connection*> connections, const std::function<bool()>& done,
          std::chrono::milliseconds timeout = 5000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::vector<pollfd> fds;
        for (const Connection* c : connections) {
            if (c->state() != Connection::State::Closed) {
                fds.push_back({c->fd(), c->pollEvents(), 0});
            }
        }
        if (!fds.empty()) {
            ::poll(fds.data(), fds.size(), 20);
        }
        for (Connection* c : connections) {
            std::error_code ec;
            if (c->state() == Connection::State::Connecting) {
                (void)c->finishConnect(ec);
            }
            if (c->state() == Connection::State::Connected) {
                (void)c->receive(ec);
            }
            if (c->state() == Connection::State::Connected) {
                (void)c->flush(ec);
            }
        }
    }
    return true;
}

struct Link {
    Listener listener;
    Connection client;
    Connection server;
};

// Connects a client to a fresh listener and accepts it.
void connectLink(const Endpoint& where, Link& link) {
    std::error_code ec;
    link.listener = Listener::listen(where, ec);
    ASSERT_TRUE(link.listener.valid()) << ec.message();
    link.client = Connection::startConnect(link.listener.endpoint(), ec);
    ASSERT_NE(link.client.state(), Connection::State::Closed) << ec.message();

    std::optional<Connection> accepted;
    ASSERT_TRUE(pump({&link.client}, [&] {
        if (!accepted) {
            accepted = link.listener.accept(ec);
        }
        return accepted.has_value() && link.client.state() == Connection::State::Connected;
    }));
    link.server = std::move(*accepted);
    ASSERT_EQ(link.server.state(), Connection::State::Connected);
}

class TransportTest : public ::testing::TestWithParam<Endpoint::Kind> {
protected:
    [[nodiscard]] Endpoint where() const {
        return GetParam() == Endpoint::Kind::Unix ? Endpoint::unixSocket(dir_.file("s.sock"))
                                                  : Endpoint::tcp("127.0.0.1", 0);
    }
    TempDir dir_;
};

TEST_P(TransportTest, HandshakeTravelsBothWays) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(where(), link));

    v1::Envelope hello;
    hello.mutable_client_hello()->set_client_name("test-client");
    ASSERT_TRUE(link.client.queue(hello));

    v1::Envelope got;
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        const auto r = link.server.next();
        return r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
    }));
    EXPECT_EQ(got.client_hello().client_name(), "test-client");

    v1::Envelope reply;
    reply.mutable_server_hello()->set_server_name("test-server");
    ASSERT_TRUE(link.server.queue(reply));
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        const auto r = link.client.next();
        return r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
    }));
    EXPECT_EQ(got.server_hello().server_name(), "test-server");
}

TEST_P(TransportTest, LargeMessageSurvivesPartialWrites) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(where(), link));

    // 8 MiB is far more than a socket buffer, so flush() must stop and resume many times.
    std::string data(8U * 1024U * 1024U, '\0');
    for (std::size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<char>(i * 2654435761U >> 24U);
    }
    v1::Envelope big;
    big.mutable_blob()->set_blob_id(5);
    big.mutable_blob()->set_data(data);
    ASSERT_TRUE(link.server.queue(big));
    EXPECT_GT(link.server.pendingOutputBytes(), data.size());

    v1::Envelope got;
    ASSERT_TRUE(pump(
        {&link.client, &link.server},
        [&] {
            const auto r = link.client.next();
            return r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
        },
        20000ms));
    EXPECT_EQ(got.blob().data(), data);
    EXPECT_EQ(link.server.pendingOutputBytes(), 0U);
}

TEST_P(TransportTest, ManySmallFramesArriveInOrder) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(where(), link));
    constexpr std::uint64_t kCount = 20000;
    for (std::uint64_t i = 0; i < kCount; ++i) {
        v1::Envelope ping;
        ping.mutable_ping()->set_nonce(i);
        ASSERT_TRUE(link.client.queue(ping));
    }
    std::uint64_t expected = 0;
    v1::Envelope got;
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        for (auto r = link.server.next(); r.status == FrameReader::Status::Frame;
             r = link.server.next()) {
            EXPECT_TRUE(parsePayload(r.payload, got));
            EXPECT_EQ(got.ping().nonce(), expected);
            ++expected;
        }
        return expected == kCount;
    }));
}

TEST_P(TransportTest, PeerCloseIsReportedAfterBufferedFramesAreDelivered) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(where(), link));
    v1::Envelope last;
    last.mutable_log()->set_text("goodbye");
    ASSERT_TRUE(link.server.queue(last));
    std::error_code ec;
    ASSERT_EQ(link.server.flush(ec), IoStatus::Ok);
    link.server.close();

    v1::Envelope got;
    bool sawFrame = false;
    ASSERT_TRUE(pump({&link.client}, [&] {
        const auto r = link.client.next();
        if (r.status == FrameReader::Status::Frame) {
            sawFrame = parsePayload(r.payload, got);
        }
        return link.client.state() == Connection::State::Closed;
    }));
    // The frame may be consumed in the same pump step that sees the close, so drain once more.
    if (!sawFrame) {
        const auto r = link.client.next();
        sawFrame = r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
    }
    EXPECT_TRUE(sawFrame);
    EXPECT_EQ(got.log().text(), "goodbye");
    EXPECT_EQ(link.client.receive(ec), IoStatus::Closed);
    EXPECT_FALSE(link.client.queue(last));
}

TEST_P(TransportTest, OversizeFrameFromPeerIsAnError) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(where(), link));
    // Rebuild the server side with a small limit, as a server would for untrusted clients.
    std::error_code ec;
    Connection strict =
        Connection::adopt(lagrange::proto::UniqueFd(::dup(link.server.fd())), GetParam(), 64);
    v1::Envelope big;
    big.mutable_blob()->set_data(std::string(1000, 'x'));
    ASSERT_TRUE(link.client.queue(big));
    ASSERT_TRUE(pump({&link.client, &strict},
                     [&] { return strict.next().status == FrameReader::Status::Error; }));
}

TEST_P(TransportTest, ClientReconnectsAfterServerRestart) {
    const Endpoint endpoint = where();
    Link first;
    ASSERT_NO_FATAL_FAILURE(connectLink(endpoint, first));
    const Endpoint bound = first.listener.endpoint();

    // Server goes away completely.
    first.server.close();
    first.listener.close();
    ASSERT_TRUE(
        pump({&first.client}, [&] { return first.client.state() == Connection::State::Closed; }));

    // While it is down, connecting fails.
    std::error_code ec;
    Connection attempt = Connection::startConnect(bound, ec);
    if (attempt.state() == Connection::State::Connecting) {
        ASSERT_TRUE(
            pump({&attempt}, [&] { return attempt.state() != Connection::State::Connecting; }));
    }
    EXPECT_EQ(attempt.state(), Connection::State::Closed);

    // It comes back on the same endpoint and the client connects again.
    Link second;
    ASSERT_NO_FATAL_FAILURE(connectLink(bound, second));
    v1::Envelope ping;
    ping.mutable_ping()->set_nonce(2);
    ASSERT_TRUE(second.client.queue(ping));
    ASSERT_TRUE(pump({&second.client, &second.server},
                     [&] { return second.server.next().status == FrameReader::Status::Frame; }));
}

INSTANTIATE_TEST_SUITE_P(Kinds, TransportTest,
                         ::testing::Values(Endpoint::Kind::Unix, Endpoint::Kind::Tcp),
                         [](const auto& param) {
                             return param.param == Endpoint::Kind::Unix ? "Unix" : "Tcp";
                         });

TEST(UnixTransport, SocketFileIsPrivateAndRemovedOnClose) {
    const TempDir dir;
    const std::string path = dir.file("private.sock");
    std::error_code ec;
    Listener listener = Listener::listen(Endpoint::unixSocket(path), ec);
    ASSERT_TRUE(listener.valid()) << ec.message();

    struct stat info{};
    ASSERT_EQ(::stat(path.c_str(), &info), 0);
    EXPECT_TRUE(S_ISSOCK(info.st_mode));
    EXPECT_EQ(info.st_mode & 0777U, 0600U);

    listener.close();
    EXPECT_NE(::stat(path.c_str(), &info), 0);
}

TEST(UnixTransport, CreatesMissingParentDirectory) {
    const TempDir dir;
    const std::string path = dir.file("lagrange/default.sock");
    std::error_code ec;
    const Listener listener = Listener::listen(Endpoint::unixSocket(path), ec);
    ASSERT_TRUE(listener.valid()) << ec.message();
    struct stat info{};
    ASSERT_EQ(::stat(dir.file("lagrange").c_str(), &info), 0);
    EXPECT_EQ(info.st_mode & 0777U, 0700U);
}

TEST(UnixTransport, StaleSocketFileIsReplacedButLiveOneIsNot) {
    const TempDir dir;
    const std::string path = dir.file("s.sock");
    const Endpoint endpoint = Endpoint::unixSocket(path);

    // What a crashed server leaves behind: a bound socket file with nobody listening.
    {
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        std::memcpy(address.sun_path, path.data(), path.size());
        const lagrange::proto::UniqueFd crashed(::socket(AF_UNIX, SOCK_STREAM, 0));
        ASSERT_TRUE(crashed.valid());
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        ASSERT_EQ(
            ::bind(crashed.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)), 0);
    }
    ASSERT_TRUE(std::filesystem::exists(path));

    std::error_code ec;
    const Listener live = Listener::listen(endpoint, ec);
    ASSERT_TRUE(live.valid()) << ec.message();

    // While `live` is listening, a second server must be refused and must not remove the file.
    const Listener second = Listener::listen(endpoint, ec);
    EXPECT_FALSE(second.valid());
    EXPECT_EQ(ec, std::errc::address_in_use);
    EXPECT_TRUE(std::filesystem::exists(path));
}

TEST(UnixTransport, RefusesToReplaceARegularFile) {
    const TempDir dir;
    const std::string path = dir.file("notes.txt");
    ASSERT_NE(std::fopen(path.c_str(), "w"), nullptr);
    std::error_code ec;
    EXPECT_FALSE(Listener::listen(Endpoint::unixSocket(path), ec).valid());
    EXPECT_EQ(ec, std::errc::file_exists);
    EXPECT_TRUE(std::filesystem::exists(path));
}

TEST(UnixTransport, ConnectToMissingSocketFails) {
    const TempDir dir;
    std::error_code ec;
    const Connection c = Connection::startConnect(Endpoint::unixSocket(dir.file("none.sock")), ec);
    EXPECT_EQ(c.state(), Connection::State::Closed);
    EXPECT_TRUE(ec);
}

TEST(UnixTransport, PathTooLongIsRejected) {
    std::error_code ec;
    const Endpoint endpoint = Endpoint::unixSocket("/tmp/" + std::string(200, 'x'));
    EXPECT_EQ(Connection::startConnect(endpoint, ec).state(), Connection::State::Closed);
    EXPECT_EQ(ec, std::errc::filename_too_long);
    EXPECT_FALSE(Listener::listen(endpoint, ec).valid());
}

TEST(UnixTransport, SharedMemoryDescriptorTravelsWithTheHello) {
    // The end-to-end fast path: the server publishes a payload in an arena and sends the
    // arena's read-only descriptor with ServerHello; the client maps it and reads the payload.
    const TempDir dir;
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(Endpoint::unixSocket(dir.file("s.sock")), link));

    std::error_code ec;
    auto arena = ShmArena::create("lagrange-scene", 1 << 20, ec);
    ASSERT_TRUE(arena.has_value()) << ec.message();
    std::vector<std::byte> mesh(300000);
    for (std::size_t i = 0; i < mesh.size(); ++i) {
        mesh[i] = static_cast<std::byte>(i % 251);
    }
    const auto slot = arena->publish(mesh);
    ASSERT_TRUE(slot.has_value());

    v1::Envelope hello;
    auto* serverHello = hello.mutable_server_hello();
    serverHello->set_shm_enabled(true);
    auto* region = serverHello->add_shm_regions();
    region->set_region(0);
    region->set_size(arena->size());
    region->set_purpose("scene");
    const std::array<int, 1> fds = {arena->shareFd()};
    ASSERT_TRUE(link.server.queue(hello, fds));

    v1::Envelope snapshot;  // a later frame that refers into the region
    auto* ref = snapshot.mutable_snapshot()->mutable_transforms();
    ref->mutable_shm()->set_region(0);
    ref->mutable_shm()->set_offset(slot->offset);
    ref->mutable_shm()->set_size(slot->size);
    ref->mutable_shm()->set_generation(slot->generation);
    ref->set_xxh3(slot->xxh3);
    ASSERT_TRUE(link.server.queue(snapshot));

    std::vector<v1::Envelope> received;
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        for (auto r = link.client.next(); r.status == FrameReader::Status::Frame;
             r = link.client.next()) {
            v1::Envelope e;
            EXPECT_TRUE(parsePayload(r.payload, e));
            received.push_back(std::move(e));
        }
        return received.size() == 2;
    }));

    auto descriptors = link.client.takeReceivedFds();
    ASSERT_EQ(descriptors.size(), 1U);
    EXPECT_TRUE(link.client.takeReceivedFds().empty());
    auto reader = ShmReader::map(descriptors[0].get(), ec);
    ASSERT_TRUE(reader.has_value()) << ec.message();
    EXPECT_EQ(reader->size(), received[0].server_hello().shm_regions(0).size());

    const auto& data = received[1].snapshot().transforms();
    const ShmSlot wanted{data.shm().offset(), data.shm().size(), data.shm().generation(),
                         data.xxh3()};
    std::vector<std::byte> out;
    ASSERT_EQ(reader->read(wanted, out), ShmReader::Status::Ok);
    EXPECT_EQ(out, mesh);
}

TEST(UnixTransport, SeveralDescriptorBatchesArriveInOrder) {
    const TempDir dir;
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(Endpoint::unixSocket(dir.file("s.sock")), link));
    std::error_code ec;
    auto a = ShmArena::create("a", 4096, ec);
    auto b = ShmArena::create("b", 8192, ec);
    auto c = ShmArena::create("c", 16384, ec);
    ASSERT_TRUE(a && b && c);

    v1::Envelope ping;
    ping.mutable_ping()->set_nonce(1);
    const std::array<int, 2> first = {a->shareFd(), b->shareFd()};
    const std::array<int, 1> second = {c->shareFd()};
    ASSERT_TRUE(link.server.queue(ping, first));
    ASSERT_TRUE(link.server.queue(ping));  // a frame without descriptors in between
    ASSERT_TRUE(link.server.queue(ping, second));

    int frames = 0;
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        while (link.client.next().status == FrameReader::Status::Frame) {
            ++frames;
        }
        return frames == 3;
    }));
    const auto descriptors = link.client.takeReceivedFds();
    ASSERT_EQ(descriptors.size(), 3U);
    const std::array<std::size_t, 3> sizes = {4096, 8192, 16384};
    for (std::size_t i = 0; i < 3; ++i) {
        auto reader = ShmReader::map(descriptors[i].get(), ec);
        ASSERT_TRUE(reader.has_value());
        EXPECT_EQ(reader->size(), sizes.at(i));
    }
}

TEST(UnixTransport, TooManyDescriptorsAreRefused) {
    const TempDir dir;
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(Endpoint::unixSocket(dir.file("s.sock")), link));
    const std::vector<int> many(lagrange::proto::kMaxFdsPerFrame + 1, STDIN_FILENO);
    const v1::Envelope empty;
    EXPECT_FALSE(link.server.queue(empty, many));
    EXPECT_EQ(link.server.pendingOutputBytes(), 0U);
}

TEST(TcpTransport, DescriptorsCannotBeSent) {
    Link link;
    ASSERT_NO_FATAL_FAILURE(connectLink(Endpoint::tcp("127.0.0.1", 0), link));
    const std::array<int, 1> fds = {STDIN_FILENO};
    const v1::Envelope empty;
    EXPECT_FALSE(link.server.queue(empty, fds));
    EXPECT_TRUE(link.server.queue(empty));
}

TEST(TcpTransport, ListenerReportsTheChosenPort) {
    std::error_code ec;
    const Listener listener = Listener::listen(Endpoint::tcp("127.0.0.1", 0), ec);
    ASSERT_TRUE(listener.valid()) << ec.message();
    EXPECT_NE(listener.endpoint().port, 0);
    EXPECT_EQ(listener.endpoint().address, "127.0.0.1");
}

TEST(TcpTransport, ConnectToClosedPortFails) {
    std::error_code ec;
    Endpoint closed;
    {
        const Listener listener = Listener::listen(Endpoint::tcp("127.0.0.1", 0), ec);
        ASSERT_TRUE(listener.valid());
        closed = listener.endpoint();
    }
    Connection c = Connection::startConnect(closed, ec);
    if (c.state() == Connection::State::Connecting) {
        ASSERT_TRUE(pump({&c}, [&] { return c.state() != Connection::State::Connecting; }));
    }
    EXPECT_EQ(c.state(), Connection::State::Closed);
}

TEST(Connection, DefaultConstructedIsClosedAndInert) {
    Connection c;
    std::error_code ec;
    EXPECT_EQ(c.state(), Connection::State::Closed);
    EXPECT_EQ(c.pollEvents(), 0);
    EXPECT_FALSE(c.queue(v1::Envelope{}));
    EXPECT_EQ(c.flush(ec), IoStatus::Closed);
    EXPECT_EQ(c.receive(ec), IoStatus::Closed);
    EXPECT_EQ(c.finishConnect(ec), IoStatus::Error);
}

TEST(ParseEndpoint, AcceptsTheDocumentedForms) {
    EXPECT_EQ(parseEndpoint("unix:/run/user/1000/lagrange/default.sock"),
              Endpoint::unixSocket("/run/user/1000/lagrange/default.sock"));
    EXPECT_EQ(parseEndpoint("/tmp/x.sock"), Endpoint::unixSocket("/tmp/x.sock"));
    EXPECT_EQ(parseEndpoint("tcp://127.0.0.1:5000"), Endpoint::tcp("127.0.0.1", 5000));
    EXPECT_EQ(parseEndpoint("tcp://sim-host"),
              Endpoint::tcp("sim-host", lagrange::proto::kDefaultTcpPort));
    EXPECT_EQ(parseEndpoint("tcp://[::1]:47610"), Endpoint::tcp("::1", 47610));
    EXPECT_EQ(parseEndpoint("tcp://[::1]"), Endpoint::tcp("::1", lagrange::proto::kDefaultTcpPort));
}

TEST(ParseEndpoint, RejectsMalformedText) {
    for (const char* text :
         {"", "unix:", "tcp://", "tcp://:5000", "tcp://host:", "tcp://host:0", "tcp://host:65536",
          "tcp://host:abc", "tcp://[::1", "tcp://[::1]x", "http://host:80", "relative/path.sock"}) {
        EXPECT_FALSE(parseEndpoint(text).has_value()) << "text: '" << text << "'";
    }
}

TEST(ParseEndpoint, RoundTripsThroughToString) {
    for (const char* text : {"unix:/tmp/x.sock", "tcp://127.0.0.1:5000", "tcp://[::1]:47610",
                             "tcp://sim-host:47610"}) {
        const auto endpoint = parseEndpoint(text);
        ASSERT_TRUE(endpoint.has_value());
        EXPECT_EQ(lagrange::proto::toString(*endpoint), text);
    }
}

TEST(DefaultUnixSocketPath, UsesTheRuntimeDirectory) {
    ::setenv("XDG_RUNTIME_DIR", "/run/user/4242", 1);
    EXPECT_EQ(lagrange::proto::defaultUnixSocketPath(), "/run/user/4242/lagrange/default.sock");
    EXPECT_EQ(lagrange::proto::defaultUnixSocketPath("bench"),
              "/run/user/4242/lagrange/bench.sock");
    ::unsetenv("XDG_RUNTIME_DIR");
    EXPECT_EQ(lagrange::proto::defaultUnixSocketPath(),
              "/tmp/lagrange-" + std::to_string(::getuid()) + "/default.sock");
}

TEST(Backoff, DoublesUpToTheMaximumAndResets) {
    Backoff backoff(100ms, 500ms);
    EXPECT_EQ(backoff.next(), 100ms);
    EXPECT_EQ(backoff.next(), 200ms);
    EXPECT_EQ(backoff.next(), 400ms);
    EXPECT_EQ(backoff.next(), 500ms);
    EXPECT_EQ(backoff.next(), 500ms);
    backoff.reset();
    EXPECT_EQ(backoff.next(), 100ms);
}

// --- TLS (tls:// endpoints) ------------------------------------------------------------------

/// A self-signed certificate for 127.0.0.1 and localhost, valid for an hour, and its key, as
/// PEM files: its own CA for the client's verification.
void writeCertificate(const std::string& certificateFile, const std::string& keyFile) {
    EVP_PKEY* key = EVP_EC_gen("P-256");
    ASSERT_NE(key, nullptr);
    X509* certificate = X509_new();
    X509_set_version(certificate, 2);
    ASN1_INTEGER_set(X509_get_serialNumber(certificate), 1);
    X509_gmtime_adj(X509_getm_notBefore(certificate), 0);
    X509_gmtime_adj(X509_getm_notAfter(certificate), 3600);
    X509_set_pubkey(certificate, key);
    X509_NAME* name = X509_get_subject_name(certificate);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                               reinterpret_cast<const unsigned char*>("localhost"), -1, -1, 0);
    X509_set_issuer_name(certificate, name);
    X509V3_CTX context;
    X509V3_set_ctx_nodb(&context);
    X509V3_set_ctx(&context, certificate, certificate, nullptr, nullptr, 0);
    for (const auto& [nid, value] : {std::pair{NID_subject_alt_name, "IP:127.0.0.1,DNS:localhost"},
                                     std::pair{NID_basic_constraints, "critical,CA:TRUE"}}) {
        X509_EXTENSION* extension = X509V3_EXT_conf_nid(nullptr, &context, nid, value);
        ASSERT_NE(extension, nullptr);
        X509_add_ext(certificate, extension, -1);
        X509_EXTENSION_free(extension);
    }
    ASSERT_GT(X509_sign(certificate, key, EVP_sha256()), 0);
    BIO* out = BIO_new_file(certificateFile.c_str(), "w");
    PEM_write_bio_X509(out, certificate);
    BIO_free(out);
    out = BIO_new_file(keyFile.c_str(), "w");
    PEM_write_bio_PrivateKey(out, key, nullptr, nullptr, 0, nullptr, nullptr);
    BIO_free(out);
    X509_free(certificate);
    EVP_PKEY_free(key);
}

/// A TLS client connected to a TLS listener and accepted (the handshake runs in the pumps).
struct TlsLink {
    Listener listener;
    Connection client;
    Connection server;
};

void connectTls(const std::shared_ptr<TlsContext>& serverContext,
                const std::shared_ptr<TlsContext>& clientContext, TlsLink& link) {
    std::error_code ec;
    link.listener = Listener::listen(Endpoint::tcp("127.0.0.1", 0, true), ec);
    ASSERT_TRUE(link.listener.valid()) << ec.message();
    ASSERT_TRUE(link.listener.endpoint().tls);
    link.client = Connection::startConnect(link.listener.endpoint(), ec,
                                           lagrange::proto::kDefaultMaxFrameSize, clientContext);
    ASSERT_NE(link.client.state(), Connection::State::Closed) << ec.message();
    std::optional<Connection> accepted;
    ASSERT_TRUE(pump({&link.client}, [&] {
        if (!accepted) {
            accepted =
                link.listener.accept(ec, lagrange::proto::kDefaultMaxFrameSize, serverContext);
        }
        return accepted.has_value() && link.client.state() == Connection::State::Connected;
    }));
    link.server = std::move(*accepted);
}

TEST(Tls, FramesTravelBothWaysEncrypted) {
    TempDir dir;
    ASSERT_NO_FATAL_FAILURE(writeCertificate(dir.file("cert.pem"), dir.file("key.pem")));
    std::string error;
    const auto serverContext = TlsContext::server(dir.file("cert.pem"), dir.file("key.pem"), error);
    ASSERT_TRUE(serverContext) << error;
    const auto clientContext = TlsContext::client(dir.file("cert.pem"), error);
    ASSERT_TRUE(clientContext) << error;
    TlsLink link;
    ASSERT_NO_FATAL_FAILURE(connectTls(serverContext, clientContext, link));
    EXPECT_TRUE(link.client.secure());
    EXPECT_TRUE(link.server.secure());

    v1::Envelope hello;
    hello.mutable_client_hello()->set_client_name(std::string(200'000, 'x'));  // many records
    ASSERT_TRUE(link.client.queue(hello));
    v1::Envelope got;
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        const auto r = link.server.next();
        return r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
    }));
    EXPECT_EQ(got.client_hello().client_name().size(), 200'000U);
    v1::Envelope reply;
    reply.mutable_server_hello()->set_server_name("tls-server");
    ASSERT_TRUE(link.server.queue(reply));
    ASSERT_TRUE(pump({&link.client, &link.server}, [&] {
        const auto r = link.client.next();
        return r.status == FrameReader::Status::Frame && parsePayload(r.payload, got);
    }));
    EXPECT_EQ(got.server_hello().server_name(), "tls-server");
}

TEST(Tls, AServerTheClientDoesNotTrustIsRefused) {
    TempDir dir;
    ASSERT_NO_FATAL_FAILURE(writeCertificate(dir.file("cert.pem"), dir.file("key.pem")));
    ASSERT_NO_FATAL_FAILURE(writeCertificate(dir.file("other.pem"), dir.file("other.key")));
    std::string error;
    const auto serverContext = TlsContext::server(dir.file("cert.pem"), dir.file("key.pem"), error);
    const auto stranger = TlsContext::client(dir.file("other.pem"), error);  // another CA
    ASSERT_TRUE(serverContext && stranger) << error;
    TlsLink link;
    ASSERT_NO_FATAL_FAILURE(connectTls(serverContext, stranger, link));
    v1::Envelope hello;
    hello.mutable_client_hello()->set_client_name("must not arrive");
    ASSERT_TRUE(link.client.queue(hello));
    ASSERT_TRUE(pump({&link.client, &link.server},
                     [&] { return link.client.state() == Connection::State::Closed; }));
    EXPECT_NE(link.client.tlsError().find("certificate"), std::string::npos)
        << link.client.tlsError();
    EXPECT_NE(link.server.next().status, FrameReader::Status::Frame);
}

TEST(Tls, ATlsEndpointNeedsAClientContext) {
    std::error_code ec;
    const auto endpoint = parseEndpoint("tls://127.0.0.1:47999");
    ASSERT_TRUE(endpoint && endpoint->tls);
    EXPECT_EQ(lagrange::proto::toString(*endpoint), "tls://127.0.0.1:47999");
    const Connection c = Connection::startConnect(*endpoint, ec);
    EXPECT_EQ(c.state(), Connection::State::Closed);
    EXPECT_EQ(ec, std::make_error_code(std::errc::invalid_argument));
}

}  // namespace
