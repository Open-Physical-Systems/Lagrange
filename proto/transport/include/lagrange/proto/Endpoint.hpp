#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace lagrange::proto {

/// Default TCP port of the viewer protocol. A project default, not an IANA registration.
inline constexpr std::uint16_t kDefaultTcpPort = 47610;

/// Where a viewer-protocol server listens.
struct Endpoint {
    enum class Kind : std::uint8_t { Unix, Tcp };

    Kind kind{Kind::Unix};
    std::string address;    ///< Socket path (Unix) or host name / IP address (Tcp).
    std::uint16_t port{0};  ///< Tcp only. 0 lets the system choose when listening.

    static Endpoint unixSocket(std::string path) { return {Kind::Unix, std::move(path), 0}; }
    static Endpoint tcp(std::string host, std::uint16_t port) {
        return {Kind::Tcp, std::move(host), port};
    }

    friend bool operator==(const Endpoint&, const Endpoint&) = default;
};

/// Parses "unix:/run/user/1000/lagrange/default.sock", "/abs/path.sock",
/// "tcp://host:port", "tcp://host" (default port) or "tcp://[::1]:47610".
[[nodiscard]] std::optional<Endpoint> parseEndpoint(std::string_view text);

[[nodiscard]] std::string toString(const Endpoint& endpoint);

/// $XDG_RUNTIME_DIR/lagrange/<instance>.sock, or /tmp/lagrange-<uid>/<instance>.sock when
/// XDG_RUNTIME_DIR is not set.
[[nodiscard]] std::string defaultUnixSocketPath(std::string_view instance = "default");

/// Delay schedule for reconnect attempts: doubles from `initial` up to `maximum`.
class Backoff {
public:
    using Milliseconds = std::chrono::milliseconds;

    explicit Backoff(Milliseconds initial = Milliseconds{100},
                     Milliseconds maximum = Milliseconds{5000})
        : initial_(initial), maximum_(maximum), current_(initial) {}

    /// Delay to wait before the next attempt.
    [[nodiscard]] Milliseconds next() {
        const Milliseconds delay = current_;
        current_ = current_ * 2 > maximum_ ? maximum_ : current_ * 2;
        return delay;
    }

    /// Call after a successful connection.
    void reset() { current_ = initial_; }

private:
    Milliseconds initial_;
    Milliseconds maximum_;
    Milliseconds current_;
};

}  // namespace lagrange::proto
