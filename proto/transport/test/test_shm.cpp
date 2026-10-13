#include "lagrange/proto/Shm.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::proto::contentHash;
using lagrange::proto::kShmSlotAlignment;
using lagrange::proto::kShmSlotHeaderSize;
using lagrange::proto::ShmArena;
using lagrange::proto::ShmReader;
using lagrange::proto::ShmSlot;
using lagrange::proto::UniqueFd;
using Status = lagrange::proto::ShmReader::Status;

// Payload whose every byte follows from (seed, index), so a reader can verify content.
std::vector<std::byte> pattern(std::uint64_t seed, std::size_t size) {
    std::vector<std::byte> data(size);
    for (std::size_t i = 0; i < size; ++i) {
        data[i] = static_cast<std::byte>((seed * 131 + i * 7) & 0xFFU);
    }
    return data;
}

struct Pair {
    ShmArena arena;
    ShmReader reader;
};

Pair makePair(std::size_t size) {
    std::error_code ec;
    auto arena = ShmArena::create("lagrange-test", size, ec);
    EXPECT_TRUE(arena.has_value()) << ec.message();
    auto reader = ShmReader::map(arena->shareFd(), ec);
    EXPECT_TRUE(reader.has_value()) << ec.message();
    return {std::move(*arena), std::move(*reader)};
}

TEST(Shm, PublishedPayloadIsReadBack) {
    Pair pair = makePair(1 << 16);
    const auto payload = pattern(1, 5000);
    const auto slot = pair.arena.publish(payload);
    ASSERT_TRUE(slot.has_value());
    EXPECT_EQ(slot->size, payload.size());
    EXPECT_EQ(slot->xxh3, contentHash(payload));
    EXPECT_NE(slot->generation, 0U);

    std::vector<std::byte> out;
    ASSERT_EQ(pair.reader.read(*slot, out), Status::Ok);
    EXPECT_EQ(out, payload);
}

TEST(Shm, EmptyPayloadIsValid) {
    Pair pair = makePair(4096);
    const auto slot = pair.arena.publish({});
    ASSERT_TRUE(slot.has_value());
    std::vector<std::byte> out = pattern(9, 3);
    EXPECT_EQ(pair.reader.read(*slot, out), Status::Ok);
    EXPECT_TRUE(out.empty());
}

TEST(Shm, SizeIsRoundedUpToPagesAndReaderSeesTheSameSize) {
    Pair pair = makePair(1000);
    EXPECT_EQ(pair.arena.size() % static_cast<std::size_t>(::sysconf(_SC_PAGESIZE)), 0U);
    EXPECT_GE(pair.arena.size(), 1000U);
    EXPECT_EQ(pair.reader.size(), pair.arena.size());
}

TEST(Shm, PayloadThatCanNeverFitIsRefused) {
    Pair pair = makePair(4096);
    const std::size_t capacity = pair.arena.size() - kShmSlotHeaderSize;
    EXPECT_FALSE(pair.arena.publish(pattern(1, capacity + 1)).has_value());
    EXPECT_TRUE(pair.arena.publish(pattern(1, capacity)).has_value());
}

TEST(Shm, SlotsAreAlignedAndDistinctUntilTheArenaWraps) {
    Pair pair = makePair(1 << 16);
    const auto a = pair.arena.publish(pattern(1, 100));
    const auto b = pair.arena.publish(pattern(2, 100));
    ASSERT_TRUE(a && b);
    EXPECT_EQ(a->offset % kShmSlotAlignment, 0U);
    EXPECT_EQ(b->offset % kShmSlotAlignment, 0U);
    EXPECT_GT(b->offset, a->offset);
    EXPECT_GT(b->generation, a->generation);

    std::vector<std::byte> out;
    EXPECT_EQ(pair.reader.read(*a, out), Status::Ok);  // both still valid
    EXPECT_EQ(pair.reader.read(*b, out), Status::Ok);
}

TEST(Shm, OverwrittenSlotIsReportedStaleNotReturned) {
    Pair pair = makePair(4096);
    const auto first = pair.arena.publish(pattern(1, 3000));
    ASSERT_TRUE(first.has_value());
    const auto second = pair.arena.publish(pattern(2, 3000));  // does not fit behind: wraps
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->offset, first->offset);

    std::vector<std::byte> out;
    EXPECT_EQ(pair.reader.read(*first, out), Status::Stale);
    EXPECT_TRUE(out.empty());
    ASSERT_EQ(pair.reader.read(*second, out), Status::Ok);
    EXPECT_EQ(out, pattern(2, 3000));
}

TEST(Shm, OldSlotBuriedInsideANewPayloadIsNeverReturned) {
    // After a wrap, an old slot's header position can lie in the middle of a new payload.
    Pair pair = makePair(8192);
    (void)pair.arena.publish(pattern(1, 1000));
    const auto buried = pair.arena.publish(pattern(2, 1000));
    ASSERT_TRUE(buried.has_value());
    ASSERT_GT(buried->offset, 0U);
    const auto big = pair.arena.publish(pattern(3, 8000));  // wraps and covers `buried`
    ASSERT_TRUE(big.has_value());
    EXPECT_EQ(big->offset, 0U);

    std::vector<std::byte> out;
    EXPECT_NE(pair.reader.read(*buried, out), Status::Ok);
    EXPECT_TRUE(out.empty());
}

TEST(Shm, WrongHashIsReportedCorrupt) {
    Pair pair = makePair(4096);
    auto slot = pair.arena.publish(pattern(1, 64));
    ASSERT_TRUE(slot.has_value());
    slot->xxh3 ^= 1;
    std::vector<std::byte> out;
    EXPECT_EQ(pair.reader.read(*slot, out), Status::Corrupt);
    EXPECT_TRUE(out.empty());
}

TEST(Shm, SlotsOutsideTheRegionAreRejected) {
    Pair pair = makePair(4096);
    const auto good = pair.arena.publish(pattern(1, 64));
    ASSERT_TRUE(good.has_value());
    const std::uint64_t size = pair.reader.size();
    std::vector<std::byte> out;

    ShmSlot slot = *good;
    slot.offset = size;  // header starts at the end
    EXPECT_EQ(pair.reader.read(slot, out), Status::OutOfBounds);
    slot.offset = size - kShmSlotAlignment;  // header fits, payload does not
    slot.size = kShmSlotAlignment;
    EXPECT_EQ(pair.reader.read(slot, out), Status::OutOfBounds);
    slot = *good;
    slot.size = UINT64_MAX;  // would overflow offset + size
    EXPECT_EQ(pair.reader.read(slot, out), Status::OutOfBounds);
    slot = *good;
    slot.offset = UINT64_MAX - 63;  // aligned, far outside
    EXPECT_EQ(pair.reader.read(slot, out), Status::OutOfBounds);
    slot = *good;
    slot.offset += 8;  // not a slot boundary
    EXPECT_EQ(pair.reader.read(slot, out), Status::OutOfBounds);
}

TEST(Shm, GenerationZeroIsNeverValid) {
    Pair pair = makePair(4096);
    ShmSlot slot;  // all zero: points at an unwritten header whose generation is 0
    slot.xxh3 = contentHash({});
    std::vector<std::byte> out;
    EXPECT_EQ(pair.reader.read(slot, out), Status::Stale);
}

TEST(Shm, SharedDescriptorCannotBeMappedWritable) {
    std::error_code ec;
    auto arena = ShmArena::create("lagrange-test", 4096, ec);
    ASSERT_TRUE(arena.has_value()) << ec.message();
    void* mapped =
        ::mmap(nullptr, arena->size(), PROT_READ | PROT_WRITE, MAP_SHARED, arena->shareFd(), 0);
    EXPECT_EQ(mapped, MAP_FAILED);
    if (mapped != MAP_FAILED) {
        ::munmap(mapped, arena->size());
    }
}

TEST(Shm, ReaderRefusesAnUnsealedDescriptor) {
    const UniqueFd fd(::memfd_create("unsealed", MFD_CLOEXEC | MFD_ALLOW_SEALING));
    ASSERT_TRUE(fd.valid());
    ASSERT_EQ(::ftruncate(fd.get(), 4096), 0);
    std::error_code ec;
    EXPECT_FALSE(ShmReader::map(fd.get(), ec).has_value());
    EXPECT_TRUE(ec);
}

TEST(Shm, ReaderRefusesAnInvalidDescriptor) {
    std::error_code ec;
    EXPECT_FALSE(ShmReader::map(-1, ec).has_value());
    EXPECT_TRUE(ec);
}

TEST(Shm, ReaderKeepsWorkingAfterTheDescriptorIsClosed) {
    std::error_code ec;
    auto arena = ShmArena::create("lagrange-test", 4096, ec);
    ASSERT_TRUE(arena.has_value());
    UniqueFd received(::fcntl(arena->shareFd(), F_DUPFD_CLOEXEC, 0));  // as if sent over a socket
    auto reader = ShmReader::map(received.get(), ec);
    ASSERT_TRUE(reader.has_value()) << ec.message();
    received.reset();

    const auto slot = arena->publish(pattern(4, 200));
    ASSERT_TRUE(slot.has_value());
    std::vector<std::byte> out;
    ASSERT_EQ(reader->read(*slot, out), Status::Ok);
    EXPECT_EQ(out, pattern(4, 200));
}

TEST(Shm, ConcurrentReaderNeverSeesWrongContent) {
    // A writer overwrites a small arena as fast as it can while a reader reads the most
    // recently published slot. Every read must be either the exact payload or "not Ok".
    Pair pair = makePair(16384);
    std::mutex mutex;
    ShmSlot latest;
    std::uint64_t latestSeed = 0;
    std::atomic<bool> done{false};

    std::thread writer([&] {
        for (std::uint64_t seed = 1; seed <= 40000; ++seed) {
            const auto payload = pattern(seed, 200 + (seed * 37) % 3000);
            const auto slot = pair.arena.publish(payload);
            ASSERT_TRUE(slot.has_value());
            const std::lock_guard lock(mutex);
            latest = *slot;
            latestSeed = seed;
        }
        done = true;
    });

    std::uint64_t okReads = 0;
    std::uint64_t rejected = 0;
    std::vector<std::byte> out;
    while (!done) {
        ShmSlot slot;
        std::uint64_t seed = 0;
        {
            const std::lock_guard lock(mutex);
            slot = latest;
            seed = latestSeed;
        }
        if (seed == 0) {
            continue;
        }
        if (pair.reader.read(slot, out) == Status::Ok) {
            ASSERT_EQ(out, pattern(seed, static_cast<std::size_t>(slot.size)));
            ++okReads;
        } else {
            ASSERT_TRUE(out.empty());
            ++rejected;
        }
    }
    writer.join();
    EXPECT_GT(okReads + rejected, 0U);
}

}  // namespace
