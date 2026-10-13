#pragma once

#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/proto/UniqueFd.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <system_error>
#include <vector>

namespace google::protobuf {
class MessageLite;
}  // namespace google::protobuf

namespace lagrange::proto {

/// Most file descriptors that may accompany one frame.
inline constexpr std::size_t kMaxFdsPerFrame = 8;
/// Most received file descriptors a connection holds before the caller takes them.
inline constexpr std::size_t kMaxPendingReceivedFds = 32;

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
    [[nodiscard]] static Connection startConnect(const Endpoint& endpoint, std::error_code& ec,
                                                 std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

    /// Wraps an already connected socket (from Listener::accept).
    [[nodiscard]] static Connection adopt(UniqueFd socket, Endpoint::Kind kind,
                                          std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

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
    [[nodiscard]] std::optional<Connection> accept(
        std::error_code& ec, std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

    void close();

private:
    UniqueFd socket_;
    Endpoint endpoint_;
};

}  // namespace lagrange::proto
