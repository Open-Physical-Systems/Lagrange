#include "lagrange/proto/Shm.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#define XXH_INLINE_ALL  // header-only use: no link dependency on libxxhash
#include <xxhash.h>

namespace lagrange::proto {

namespace {

// Offsets of the header fields.
constexpr std::size_t kGenerationOffset = 0;
constexpr std::size_t kSizeOffset = 8;
constexpr std::size_t kHashOffset = 16;

std::error_code lastError() {
    return {errno, std::generic_category()};
}

std::size_t alignUp(std::size_t value, std::size_t alignment) {
    return (value + alignment - 1) / alignment * alignment;
}

// The slot generation is the only word both processes access concurrently on purpose. Slot
// offsets are multiples of 64 in a page-aligned mapping, so the word is suitably aligned for
// the compiler's atomic builtins (GCC and Clang, the project's only compilers).
std::uint64_t loadGeneration(const std::byte* header) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return __atomic_load_n(reinterpret_cast<const std::uint64_t*>(header + kGenerationOffset),
                           __ATOMIC_ACQUIRE);
}

void storeGeneration(std::byte* header, std::uint64_t value) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    __atomic_store_n(reinterpret_cast<std::uint64_t*>(header + kGenerationOffset), value,
                     __ATOMIC_RELEASE);
}

}  // namespace

std::uint64_t contentHash(std::span<const std::byte> data) {
    return XXH3_64bits(data.data(), data.size());
}

// --- ShmArena ---------------------------------------------------------------------------

ShmArena::~ShmArena() {
    unmap();
}

ShmArena::ShmArena(ShmArena&& other) noexcept
    : fd_(std::move(other.fd_))
    , readOnlyFd_(std::move(other.readOnlyFd_))
    , base_(std::exchange(other.base_, nullptr))
    , size_(std::exchange(other.size_, 0))
    , next_(std::exchange(other.next_, 0))
    , generation_(std::exchange(other.generation_, 0)) {}

ShmArena& ShmArena::operator=(ShmArena&& other) noexcept {
    if (this != &other) {
        unmap();
        fd_ = std::move(other.fd_);
        readOnlyFd_ = std::move(other.readOnlyFd_);
        base_ = std::exchange(other.base_, nullptr);
        size_ = std::exchange(other.size_, 0);
        next_ = std::exchange(other.next_, 0);
        generation_ = std::exchange(other.generation_, 0);
    }
    return *this;
}

void ShmArena::unmap() {
    if (base_ != nullptr) {
        (void)::munmap(base_, size_);
        base_ = nullptr;
    }
    size_ = 0;
}

std::optional<ShmArena> ShmArena::create(const std::string& debugName, std::size_t size,
                                         std::error_code& ec) {
    ec.clear();
    const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
    if (size == 0 || size > std::numeric_limits<std::size_t>::max() / 2) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return std::nullopt;
    }
    size = alignUp(size, page);

    ShmArena arena;
    arena.fd_.reset(::memfd_create(debugName.c_str(), MFD_CLOEXEC | MFD_ALLOW_SEALING));
    if (!arena.fd_.valid() || ::ftruncate(arena.fd_.get(), static_cast<off_t>(size)) != 0 ||
        // The size can never change again, so a reader's mapping cannot be cut short.
        ::fcntl(arena.fd_.get(), F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_SEAL) != 0) {
        ec = lastError();
        return std::nullopt;
    }
    void* mapped = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, arena.fd_.get(), 0);
    if (mapped == MAP_FAILED) {
        ec = lastError();
        return std::nullopt;
    }
    arena.base_ = static_cast<std::byte*>(mapped);
    arena.size_ = size;

    // Reopen read-only: a descriptor opened O_RDONLY cannot be mapped writable by the peer.
    const std::string path = "/proc/self/fd/" + std::to_string(arena.fd_.get());
    arena.readOnlyFd_.reset(::open(path.c_str(), O_RDONLY | O_CLOEXEC));
    if (!arena.readOnlyFd_.valid()) {
        ec = lastError();
        return std::nullopt;
    }
    return arena;
}

std::optional<ShmSlot> ShmArena::publish(std::span<const std::byte> payload) {
    if (base_ == nullptr || payload.size() > size_ - kShmSlotHeaderSize ||
        size_ < kShmSlotHeaderSize) {
        return std::nullopt;
    }
    if (next_ + kShmSlotHeaderSize + payload.size() > size_) {
        next_ = 0;  // wrap: overwrite the oldest slots
    }
    std::byte* header = base_ + next_;

    ShmSlot slot;
    slot.offset = next_;
    slot.size = payload.size();
    slot.generation = ++generation_;
    slot.xxh3 = contentHash(payload);

    // Seqlock write: invalidate, write, then publish the new generation.
    storeGeneration(header, 0);
    std::memcpy(header + kSizeOffset, &slot.size, sizeof(slot.size));
    std::memcpy(header + kHashOffset, &slot.xxh3, sizeof(slot.xxh3));
    if (!payload.empty()) {
        std::memcpy(header + kShmSlotHeaderSize, payload.data(), payload.size());
    }
    storeGeneration(header, slot.generation);

    next_ = alignUp(next_ + kShmSlotHeaderSize + payload.size(), kShmSlotAlignment);
    if (next_ + kShmSlotHeaderSize > size_) {
        next_ = 0;
    }
    return slot;
}

// --- ShmReader --------------------------------------------------------------------------

ShmReader::~ShmReader() {
    unmap();
}

ShmReader::ShmReader(ShmReader&& other) noexcept
    : base_(std::exchange(other.base_, nullptr)), size_(std::exchange(other.size_, 0)) {}

ShmReader& ShmReader::operator=(ShmReader&& other) noexcept {
    if (this != &other) {
        unmap();
        base_ = std::exchange(other.base_, nullptr);
        size_ = std::exchange(other.size_, 0);
    }
    return *this;
}

void ShmReader::unmap() {
    if (base_ != nullptr) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): munmap takes a non-const pointer.
        (void)::munmap(const_cast<std::byte*>(base_), size_);
        base_ = nullptr;
    }
    size_ = 0;
}

std::optional<ShmReader> ShmReader::map(int fd, std::error_code& ec) {
    ec.clear();
    struct stat info{};
    if (::fstat(fd, &info) != 0) {
        ec = lastError();
        return std::nullopt;
    }
    const int seals = ::fcntl(fd, F_GET_SEALS);
    if (seals < 0 || (seals & F_SEAL_SHRINK) == 0) {
        // Not a sealed memfd: the other side could truncate it and crash us with SIGBUS.
        ec = std::make_error_code(std::errc::permission_denied);
        return std::nullopt;
    }
    if (info.st_size < static_cast<off_t>(kShmSlotHeaderSize)) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return std::nullopt;
    }
    const auto size = static_cast<std::size_t>(info.st_size);
    void* mapped = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
        ec = lastError();
        return std::nullopt;
    }
    ShmReader reader;
    reader.base_ = static_cast<const std::byte*>(mapped);
    reader.size_ = size;
    return reader;
}

ShmReader::Status ShmReader::read(const ShmSlot& slot, std::vector<std::byte>& out) const {
    out.clear();
    // The slot description comes from the peer: validate it without overflowing.
    if (base_ == nullptr || slot.offset % kShmSlotAlignment != 0 ||
        slot.offset > size_ - kShmSlotHeaderSize ||
        slot.size > size_ - kShmSlotHeaderSize - slot.offset) {
        return Status::OutOfBounds;
    }
    if (slot.generation == 0) {
        return Status::Stale;  // 0 marks a slot that is being written
    }
    const std::byte* header = base_ + slot.offset;
    if (loadGeneration(header) != slot.generation) {
        return Status::Stale;
    }
    out.resize(static_cast<std::size_t>(slot.size));
    if (!out.empty()) {
        // The writer may overwrite these bytes while we copy. The generation re-check and the
        // hash below reject such a copy; the bytes are never interpreted before that.
        std::memcpy(out.data(), header + kShmSlotHeaderSize, out.size());
    }
    if (loadGeneration(header) != slot.generation) {
        out.clear();
        return Status::Stale;
    }
    if (contentHash(out) != slot.xxh3) {
        out.clear();
        return Status::Corrupt;
    }
    return Status::Ok;
}

}  // namespace lagrange::proto
