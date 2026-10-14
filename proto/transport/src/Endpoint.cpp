#include "lagrange/proto/Endpoint.hpp"

#include <charconv>
#include <cstdlib>
#include <unistd.h>

namespace lagrange::proto {

namespace {

constexpr std::string_view kUnixPrefix = "unix:";
constexpr std::string_view kTcpPrefix = "tcp://";

std::optional<std::uint16_t> parsePort(std::string_view text) {
    unsigned value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || value == 0 || value > 65535) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(value);
}

}  // namespace

std::optional<Endpoint> parseEndpoint(std::string_view text) {
    if (text.starts_with(kUnixPrefix)) {
        text.remove_prefix(kUnixPrefix.size());
        if (text.empty()) {
            return std::nullopt;
        }
        return Endpoint::unixSocket(std::string(text));
    }
    if (text.starts_with('/')) {
        return Endpoint::unixSocket(std::string(text));
    }
    if (!text.starts_with(kTcpPrefix)) {
        return std::nullopt;
    }
    text.remove_prefix(kTcpPrefix.size());

    std::string_view host;
    std::string_view port;
    if (text.starts_with('[')) {  // IPv6 literal: [::1]:47610
        const std::size_t close = text.find(']');
        if (close == std::string_view::npos) {
            return std::nullopt;
        }
        host = text.substr(1, close - 1);
        const std::string_view rest = text.substr(close + 1);
        if (!rest.empty()) {
            if (!rest.starts_with(':')) {
                return std::nullopt;
            }
            port = rest.substr(1);
            if (port.empty()) {
                return std::nullopt;
            }
        }
    } else {
        const std::size_t colon = text.rfind(':');
        if (colon == std::string_view::npos) {
            host = text;
        } else {
            host = text.substr(0, colon);
            port = text.substr(colon + 1);
            if (port.empty()) {
                return std::nullopt;
            }
        }
    }
    if (host.empty()) {
        return std::nullopt;
    }
    std::uint16_t portNumber = kDefaultTcpPort;
    if (!port.empty()) {
        const auto parsed = parsePort(port);
        if (!parsed) {
            return std::nullopt;
        }
        portNumber = *parsed;
    }
    return Endpoint::tcp(std::string(host), portNumber);
}

std::string toString(const Endpoint& endpoint) {
    if (endpoint.kind == Endpoint::Kind::Unix) {
        return std::string(kUnixPrefix) + endpoint.address;
    }
    const bool ipv6 = endpoint.address.find(':') != std::string::npos;
    return std::string(kTcpPrefix) + (ipv6 ? "[" + endpoint.address + "]" : endpoint.address) +
           ":" + std::to_string(endpoint.port);
}

std::string defaultUnixSocketPath(std::string_view instance) {
    std::string directory;
    // NOLINTNEXTLINE(concurrency-mt-unsafe): read once at startup, before threads exist.
    if (const char* runtimeDir = std::getenv("XDG_RUNTIME_DIR");
        runtimeDir != nullptr && *runtimeDir != '\0') {
        directory = std::string(runtimeDir) + "/lagrange";
    } else {
        directory = "/tmp/lagrange-" + std::to_string(::getuid());
    }
    return directory + "/" + std::string(instance) + ".sock";
}

}  // namespace lagrange::proto
