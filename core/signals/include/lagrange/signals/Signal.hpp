#pragma once

#include "lagrange/signals/Unit.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace lagrange {

/// What a signal carries. Numeric types are stored as double (docs/design/architecture.md §3).
enum class SignalType : std::uint8_t {
    Scalar,       // 1 x f64
    Vec3,         // 3 x f64, in the frame named by SignalSpec::frame
    Quat,         // 4 x f64, (w, x, y, z), unit norm
    Pose,         // 7 x f64: position (m, region-local frame) then Quat
    Wrench,       // 6 x f64: linear part (unit of the spec), then the angular part
    ScalarArray,  // N x f64, N = SignalSpec::arrayLength
    Bool,         // 1 x f64 holding 0 or 1
    Int,          // 1 x f64 holding an integer below 2^53 (counters, fault codes)
    Bytes,        // handle to a variable-length packet
    Field3D,      // handle to a NanoVDB grid
    Image,        // handle to an image buffer
    PointCloud,   // handle to a point buffer
};

/// Number of doubles one value of the type occupies; 0 for handle types.
[[nodiscard]] std::size_t componentCount(SignalType type, std::uint32_t arrayLength);

/// Whether values of the type are handles to buffers rather than numbers.
[[nodiscard]] bool isHandle(SignalType type);

[[nodiscard]] std::string_view to_string(SignalType type);

/// What a reader sees when the writer runs at another rate (RFC 0001, "Rates and holds").
enum class Hold : std::uint8_t {
    ZeroOrder,          // the last committed value (default)
    LinearExtrapolate,  // extrapolated from the last two commits to the reader's time
    WindowAverage,      // the time-weighted mean of the commits since the reader's previous step
};

[[nodiscard]] std::string_view to_string(Hold hold);

/// One input or output of a module.
struct SignalSpec {
    /// "<class>/<instance>/<quantity>[/<qualifier>]", e.g. "joint/left_wheel/torque".
    std::string name;
    SignalType type{SignalType::Scalar};
    Unit unit{};
    std::uint32_t arrayLength{1};  // for ScalarArray
    std::string frame;             // for Vec3, Pose, Wrench: "world_local", "body:<name>", ""
    Hold hold{Hold::ZeroOrder};    // inputs only
    bool optional{false};          // input may stay unconnected; reads then give defaultValue
    double defaultValue{0.0};
    bool feedback{false};  // input reads the writer's previous step, breaking a loop (inputs only)
};

/// Checks a signal name: '/'-separated, non-empty segments of [A-Za-z0-9_.-]. Returns an empty
/// string when valid, otherwise the reason.
[[nodiscard]] std::string checkSignalName(std::string_view name);

/// A resolved signal for the hot path. Outputs index the bus's signal table; inputs index the
/// reading module's input slots, where the scheduler places the held value before each step.
struct SignalId {
    static constexpr std::uint32_t kInvalid = std::numeric_limits<std::uint32_t>::max();
    static constexpr std::uint32_t kInputBit = 0x8000'0000U;

    std::uint32_t index{kInvalid};

    [[nodiscard]] constexpr bool valid() const { return index != kInvalid; }
    [[nodiscard]] constexpr bool isInput() const { return valid() && (index & kInputBit) != 0; }
    [[nodiscard]] constexpr std::uint32_t slot() const { return index & ~kInputBit; }

    friend constexpr bool operator==(SignalId, SignalId) = default;

    static constexpr SignalId output(std::uint32_t signal) { return {signal}; }
    static constexpr SignalId input(std::uint32_t slot) { return {slot | kInputBit}; }
};

/// A reference to a buffer held outside the bus (Field3D, Image, PointCloud, Bytes).
struct BufferHandle {
    std::uint64_t id{0};
    friend constexpr bool operator==(BufferHandle, BufferHandle) = default;
};

}  // namespace lagrange
