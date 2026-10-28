#pragma once

#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/proto/UniqueFd.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace google::protobuf {
class MessageLite;
}  // namespace google::protobuf

struct ssl_st;
struct ssl_ctx_st;

namespace lagrange::proto {

/// Most file descriptors that may accompany one frame.
inline constexpr std::size_t kMaxFdsPerFrame = 8;
/// Most received file descriptors a connection holds before the caller takes them.
inline constexpr std::size_t kMaxPendingReceivedFds = 32;

/// TLS for tls:// endpoints (OpenSSL, TLS 1.2 or newer). A server context holds the
/// certificate chain and key it presents; a client context verifies the server's certificate
/// against a CA file, or the system's trust store, and the endpoint's host name or address.
class TlsContext {
public:
    [[nodiscard]] static std::shared_ptr<TlsContext> server(const std::string& certificateFile,
                                                            const std::string& keyFile,
                                                            std::string& error);
    /// `caFile` empty: the system's default trust store.
    [[nodiscard]] static std::shared_ptr<TlsContext> client(const std::string& caFile,
                                                            std::string& error);
    ~TlsContext();
    TlsContext(const TlsContext&) = delete;
    TlsContext& operator=(const TlsContext&) = delete;
    TlsContext(TlsContext&&) = delete;
    TlsContext& operator=(TlsContext&&) = delete;

    [[nodiscard]] ssl_ctx_st* get() const { return context_; }
    [[nodiscard]] bool isServer() const { return server_; }

private:
    TlsContext(ssl_ctx_st* context, bool server) : context_(context), server_(server) {}
    ssl_ctx_st* context_;
    bool server_;
};

enum class IoStatus : std::uint8_t {
    Ok,          ///< Progress was made.
    WouldBlock,  ///< Nothing could be done now; wait for the socket with poll().
    Closed,      ///< The peer closed the connection.
    Error,       ///< The connection failed; the error code says why.
};

/// One non-blocking viewer-protocol stream over a Unix socket or TCP.
///
/// The class does no waiting. The owner calls poll() on fd() with pollEvents(), then
/// finishConnect(), receive() and flush() as the socket becomes ready. It is not thread-safe:
/// one thread owns a connection.
class Connection {
public:
    enum class State : std::uint8_t { Closed, Connecting, Connected };

    Connection() = default;

    /// Starts a non-blocking connect. On return the state is Connecting or Connected, or
    /// Closed with `ec` set. For Tcp with a host name this performs a (blocking) name lookup.
    /// A tls:// endpoint needs a client `tls` context; the handshake runs in flush() and
    /// receive() once connected, and its failure is reported there (tlsError() says why).
    [[nodiscard]] static Connection startConnect(const Endpoint& endpoint, std::error_code& ec,
                                                 std::uint32_t maxFrameSize = kDefaultMaxFrameSize,
                                                 std::shared_ptr<TlsContext> tls = nullptr);

    /// Wraps an already connected socket (from Listener::accept); with a server `tls`
    /// context, TLS is negotiated before any frame.
    [[nodiscard]] static Connection adopt(UniqueFd socket, Endpoint::Kind kind,
                                          std::uint32_t maxFrameSize = kDefaultMaxFrameSize,
                                          std::shared_ptr<TlsContext> tls = nullptr);

    /// True when TLS protects the connection (possibly still negotiating).
    [[nodiscard]] bool secure() const { return static_cast<bool>(ssl_); }
    /// Why the TLS handshake or a TLS read or write failed; empty otherwise.
    [[nodiscard]] const std::string& tlsError() const { return tlsError_; }

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] int fd() const { return socket_.get(); }
    [[nodiscard]] Endpoint::Kind kind() const { return kind_; }

    /// Events to pass to poll() for this socket.
    [[nodiscard]] short pollEvents() const;

    /// Completes a connect once the socket is writable. Ok means Connected; WouldBlock means
    /// still connecting.
    [[nodiscard]] IoStatus finishConnect(std::error_code& ec);

    /// Queues one frame. `fds` are duplicated and travel with the frame as SCM_RIGHTS
    /// ancillary data (Unix sockets only). Returns false if the connection is closed, the
    /// message exceeds the frame limit, there are too many fds, or fds are given on Tcp.
    [[nodiscard]] bool queue(const google::protobuf::MessageLite& message,
                             std::span<const int> fds = {});

    /// Writes queued bytes until done or the socket is full.
    [[nodiscard]] IoStatus flush(std::error_code& ec);

    /// Reads what is available (bounded per call). Ok means bytes arrived; call next().
    [[nodiscard]] IoStatus receive(std::error_code& ec);

    /// Next complete frame from the bytes read so far. See FrameReader::next().
    [[nodiscard]] FrameReader::Result next() { return reader_.next(); }

    /// File descriptors received so far, in arrival order. A descriptor arrives no later than
    /// the frame it was queued with.
    [[nodiscard]] std::vector<UniqueFd> takeReceivedFds();

    [[nodiscard]] std::size_t pendingOutputBytes() const { return out_.size() - outPos_; }

    void close();

private:
    struct SslDeleter {
        void operator()(ssl_st* ssl) const;
    };
    /// Runs the TLS handshake until it is done; Ok when it is (or there is no TLS).
    [[nodiscard]] IoStatus tlsHandshake(std::error_code& ec);
    [[nodiscard]] IoStatus tlsFailure(std::error_code& ec, int result);
    [[nodiscard]] IoStatus flushTls(std::error_code& ec);
    [[nodiscard]] IoStatus receiveTls(std::error_code& ec);

    struct FdBatch {
        std::uint64_t streamOffset;  // offset in the outgoing byte stream of the frame's first byte
        std::vector<UniqueFd> fds;
    };

    UniqueFd socket_;
    Endpoint::Kind kind_{Endpoint::Kind::Unix};
    State state_{State::Closed};
    std::uint32_t maxFrameSize_{kDefaultMaxFrameSize};

    std::vector<std::byte> out_;
    std::size_t outPos_{0};
    std::uint64_t sentTotal_{0};  // bytes handed to the kernel so far
    std::deque<FdBatch> outFds_;

    FrameReader reader_{kDefaultMaxFrameSize};
    std::vector<UniqueFd> receivedFds_;
    std::vector<std::byte> scratch_;  // receive buffer, allocated on first use

    std::shared_ptr<TlsContext> tlsContext_;  // keeps the context alive for ssl_
    std::unique_ptr<ssl_st, SslDeleter> ssl_;
    bool tlsReady_{false};
    bool tlsWantsWrite_{false};  // the handshake waits for the socket to be writable
    std::string tlsError_;
};

/// A listening socket. Unix socket files are created with mode 0600 and removed on
/// destruction; Tcp listeners should bind to a loopback address unless the user asks otherwise.
class Listener {
public:
    Listener() = default;
    ~Listener();
    Listener(const Listener&) = delete;
    Listener& operator=(const Listener&) = delete;
    Listener(Listener&& other) noexcept;
    Listener& operator=(Listener&& other) noexcept;

    /// Starts listening. A stale Unix socket file (nobody listening) is replaced; a live one
    /// gives EADDRINUSE. Returns an invalid Listener with `ec` set on failure.
    [[nodiscard]] static Listener listen(const Endpoint& endpoint, std::error_code& ec,
                                         int backlog = 8);

    [[nodiscard]] bool valid() const { return socket_.valid(); }
    [[nodiscard]] int fd() const { return socket_.get(); }

    /// The bound endpoint, with the actual port when 0 was requested.
    [[nodiscard]] const Endpoint& endpoint() const { return endpoint_; }

    /// Accepts one pending connection without blocking. Empty with no error if none is pending.
    /// With a server `tls` context the connection negotiates TLS first.
    [[nodiscard]] std::optional<Connection> accept(
        std::error_code& ec, std::uint32_t maxFrameSize = kDefaultMaxFrameSize,
        std::shared_ptr<TlsContext> tls = nullptr);

    void close();

private:
    UniqueFd socket_;
    Endpoint endpoint_;
};

}  // namespace lagrange::proto
