#pragma once

#include <compare>
#include <cstdint>
#include <optional>
#include <string_view>

namespace lagrange {

/// A span of simulation time as a whole number of nanoseconds.
///
/// Simulation time is never floating point: the multirate scheduler compares step sizes
/// exactly, and 0.0001 * 10 != 0.001 in binary floating point.
struct Duration {
    std::int64_t ns{0};

    /// Value in seconds, for display and for physics formulas. Not for scheduling.
    [[nodiscard]] constexpr double seconds() const { return static_cast<double>(ns) * 1e-9; }

    friend constexpr auto operator<=>(Duration, Duration) = default;
    friend constexpr Duration operator+(Duration a, Duration b) { return {a.ns + b.ns}; }
    friend constexpr Duration operator-(Duration a, Duration b) { return {a.ns - b.ns}; }
    friend constexpr Duration operator*(Duration d, std::int64_t k) { return {d.ns * k}; }
    friend constexpr Duration operator*(std::int64_t k, Duration d) { return {d.ns * k}; }
};

/// A point in simulation time: nanoseconds since the start of the run.
/// The range of a signed 64-bit count of nanoseconds is about 292 years.
struct SimTime {
    std::int64_t ns{0};

    [[nodiscard]] constexpr double seconds() const { return static_cast<double>(ns) * 1e-9; }

    friend constexpr auto operator<=>(SimTime, SimTime) = default;
    friend constexpr SimTime operator+(SimTime t, Duration d) { return {t.ns + d.ns}; }
    friend constexpr SimTime operator-(SimTime t, Duration d) { return {t.ns - d.ns}; }
    friend constexpr Duration operator-(SimTime a, SimTime b) { return {a.ns - b.ns}; }
};

namespace literals {
constexpr Duration operator""_ns(unsigned long long v) {
    return {static_cast<std::int64_t>(v)};
}
constexpr Duration operator""_us(unsigned long long v) {
    return {static_cast<std::int64_t>(v) * 1'000};
}
constexpr Duration operator""_ms(unsigned long long v) {
    return {static_cast<std::int64_t>(v) * 1'000'000};
}
constexpr Duration operator""_s(unsigned long long v) {
    return {static_cast<std::int64_t>(v) * 1'000'000'000};
}
}  // namespace literals

/// Parses a non-negative decimal number of seconds exactly, e.g. "0.0001", "1e-3", "2".
///
/// The text is read as a decimal, not converted through a floating-point type, so "0.0001"
/// gives exactly 100000 ns. Returns std::nullopt if the text is not a plain decimal number
/// (optional '+', digits, optional fraction, optional exponent), if the value is not a whole
/// number of nanoseconds, or if it does not fit in 64 bits.
[[nodiscard]] std::optional<Duration> parseDecimalSeconds(std::string_view text);

}  // namespace lagrange
