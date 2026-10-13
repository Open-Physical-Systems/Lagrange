#pragma once

#include "lagrange/proto/UniqueFd.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace lagrange::proto {

/// Shared-memory fast path for large payloads (docs/design/protocol.md section 6).
///
/// The server owns an arena backed by a sealed memfd and sends a read-only descriptor to the
/// viewer over the Unix socket. Payloads are written into slots:
///
///     +--------------------------------------------------+---------------------+
///     | header: generation u64 | size u64 | xxh3 u64 | 0 | payload, size bytes |
///     +--------------------------------------------------+---------------------+
///
/// The arena is a ring: when it is full the oldest slots are overwritten. There is no
/// cross-process lock. A reader checks the slot generation before and after copying (a
/// seqlock) and then checks the content hash, so a payload that was overwritten during or
/// before the read is reported as stale, never returned.

inline constexpr std::size_t kShmSlotHeaderSize = 32;
inline constexpr std::size_t kShmSlotAlignment = 64;

/// Location of one payload. Mirrors lagrange.viewer.v1.ShmRef without the region index.
struct ShmSlot {
    std::uint64_t offset{0};      ///< Offset of the slot header in the region.
    std::uint64_t size{0};        ///< Payload size in bytes.
    std::uint64_t generation{0};  ///< Value the slot header must hold.
    std::uint64_t xxh3{0};        ///< XXH3-64 of the payload (DataRef.xxh3).
};

/// XXH3-64 of `data`.
[[nodiscard]] std::uint64_t contentHash(std::span<const std::byte> data);

/// Writer side. Single writer thread.
class ShmArena {
public:
    ShmArena() = default;
    ~ShmArena();
    ShmArena(const ShmArena&) = delete;
    ShmArena& operator=(const ShmArena&) = delete;
    ShmArena(ShmArena&& other) noexcept;
    ShmArena& operator=(ShmArena&& other) noexcept;

    /// Creates an arena of at least `size` bytes (rounded up to whole pages), with its size
    /// sealed so that a reader's mapping can never be truncated.
    [[nodiscard]] static std::optional<ShmArena> create(const std::string& debugName,
                                                        std::size_t size, std::error_code& ec);

    [[nodiscard]] std::size_t size() const { return size_; }

    /// A read-only descriptor of the arena, to pass to the reader process. It cannot be
    /// mapped writable. Owned by the arena.
    [[nodiscard]] int shareFd() const { return readOnlyFd_.get(); }

    /// Copies `payload` into the arena. Empty if the payload can never fit.
    [[nodiscard]] std::optional<ShmSlot> publish(std::span<const std::byte> payload);

private:
    void unmap();

    UniqueFd fd_;
    UniqueFd readOnlyFd_;
    std::byte* base_{nullptr};
    std::size_t size_{0};
    std::size_t next_{0};          // offset of the next slot
    std::uint64_t generation_{0};  // last generation handed out; 0 is never valid
};

/// Reader side. Maps a region read-only and copies payloads out of it.
class ShmReader {
public:
    enum class Status : std::uint8_t {
        Ok,
        Stale,        ///< The slot was reused. Ask the sender for the data again.
        OutOfBounds,  ///< The slot does not lie inside the region. The sender is faulty.
        Corrupt,      ///< The generation matched but the content hash did not.
    };

    ShmReader() = default;
    ~ShmReader();
    ShmReader(const ShmReader&) = delete;
    ShmReader& operator=(const ShmReader&) = delete;
    ShmReader(ShmReader&& other) noexcept;
    ShmReader& operator=(ShmReader&& other) noexcept;

    /// Maps the region behind `fd`. The descriptor may be closed afterwards. Refuses a
    /// descriptor whose size is not sealed, because a later truncation would crash the reader.
    [[nodiscard]] static std::optional<ShmReader> map(int fd, std::error_code& ec);

    [[nodiscard]] std::size_t size() const { return size_; }

    /// Copies the payload of `slot` into `out`. `out` is left empty unless the result is Ok.
    [[nodiscard]] Status read(const ShmSlot& slot, std::vector<std::byte>& out) const;

private:
    void unmap();

    const std::byte* base_{nullptr};
    std::size_t size_{0};
};

}  // namespace lagrange::proto
