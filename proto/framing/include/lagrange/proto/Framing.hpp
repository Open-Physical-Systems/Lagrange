#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace google::protobuf {
class MessageLite;
}  // namespace google::protobuf

namespace lagrange::proto {

/// Every message on a viewer-protocol stream is one frame:
///
///     +----------------+---------------------------+
///     | length: u32 LE | payload, `length` bytes   |
///     +----------------+---------------------------+
///
/// The payload is a serialised lagrange.viewer.v1.Envelope. A zero-length payload is valid
/// (an Envelope with no body set). See docs/design/protocol.md section 2.
inline constexpr std::size_t kFrameHeaderSize = 4;

/// Largest payload accepted by default: 64 MiB. Larger data is chunked or sent through
/// shared memory.
inline constexpr std::uint32_t kDefaultMaxFrameSize = 64U * 1024U * 1024U;

/// Largest frame a server accepts from a peer that has not yet been greeted: 64 KiB
/// (RFC 0014 §1). A ClientHello is well under 1 KiB; the limit is raised to
/// kDefaultMaxFrameSize once the handshake succeeds.
inline constexpr std::uint32_t kPreHandshakeMaxFrameSize = 64U * 1024U;

/// Appends one frame (header + payload) to `out`.
/// Returns false and leaves `out` unchanged if the payload is larger than `maxFrameSize`.
[[nodiscard]] bool appendFrame(std::span<const std::byte> payload, std::vector<std::byte>& out,
                               std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

/// Serialises `message` and appends it as one frame to `out`.
/// Returns false and leaves `out` unchanged if the message is larger than `maxFrameSize` or
/// cannot be serialised.
[[nodiscard]] bool appendFrame(const google::protobuf::MessageLite& message,
                               std::vector<std::byte>& out,
                               std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

/// Parses a frame payload into `message`. Returns false if the bytes are not a valid message.
[[nodiscard]] bool parsePayload(std::span<const std::byte> payload,
                                google::protobuf::MessageLite& message);

/// Splits a byte stream into frames. It does no I/O: the caller feeds it whatever bytes
/// arrived, in pieces of any size, and takes complete frames out.
///
/// The input is untrusted (it comes from a socket). The reader never allocates based on a
/// length it has not validated against the size limit.
class FrameReader {
public:
    enum class Status : std::uint8_t {
        Frame,     ///< `payload` holds one complete frame.
        NeedMore,  ///< No complete frame is buffered. Feed more bytes.
        Error,     ///< The stream is invalid. The connection must be closed.
    };

    enum class Error : std::uint8_t {
        None,
        Oversize,  ///< A header announced a payload larger than the size limit.
    };

    struct Result {
        Status status{Status::NeedMore};
        /// Valid only when status is Frame, and only until the next call to feed().
        std::span<const std::byte> payload{};
    };

    explicit FrameReader(std::uint32_t maxFrameSize = kDefaultMaxFrameSize);

    /// Appends bytes received from the stream. Invalidates payload spans returned earlier.
    void feed(std::span<const std::byte> data);

    /// Returns the next complete frame if one is buffered. Call repeatedly until it returns
    /// NeedMore. After an Error, every later call returns Error.
    [[nodiscard]] Result next();

    [[nodiscard]] Error error() const { return error_; }

    /// Bytes fed but not yet returned as frames.
    [[nodiscard]] std::size_t bufferedBytes() const { return buffer_.size() - begin_; }

    [[nodiscard]] std::uint32_t maxFrameSize() const { return maxFrameSize_; }

    /// Changes the size limit for frames not yet returned by next(). Bytes already buffered
    /// stay; a frame's header is checked only in next(), so every frame is checked once, against
    /// the limit in force when it comes out. It does not revive a stream after an Error.
    void setMaxFrameSize(std::uint32_t maxFrameSize) { maxFrameSize_ = maxFrameSize; }

private:
    void compact();

    std::vector<std::byte> buffer_;
    std::size_t begin_{0};  // offset of the first unconsumed byte in buffer_
    std::uint32_t maxFrameSize_;
    Error error_{Error::None};
};

}  // namespace lagrange::proto
