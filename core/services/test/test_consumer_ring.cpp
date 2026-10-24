// Roadmap 0.2-E3: tick records to consumers through bounded rings. Acceptance: slow-consumer
// tests (a slow consumer loses items or holds the producer back, as its policy says).
#include "lagrange/services/ConsumerRing.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::services {
namespace {

using namespace std::chrono_literals;

TEST(ConsumerRing, KeepsOrderAndCounts) {
    ConsumerRing<std::string> ring(4, Overflow::DropNewest);
    for (int k = 0; k < 4; ++k) {
        EXPECT_TRUE(ring.push(std::to_string(k)));
    }
    EXPECT_FALSE(ring.push("full")) << "a fifth item does not fit";
    for (int k = 0; k < 4; ++k) {
        EXPECT_EQ(ring.pop().value(), std::to_string(k));
    }
    EXPECT_FALSE(ring.pop().has_value());
    const auto stats = ring.stats();
    EXPECT_EQ(stats.pushed, 4U);
    EXPECT_EQ(stats.dropped, 1U);
    EXPECT_EQ(stats.maxDepth, 4U);
}

// A viewer that cannot keep up: the simulation never waits, the viewer sees an in-order
// subset, and every item is either delivered or counted as dropped.
TEST(ConsumerRing, ASlowLossyConsumerNeverHoldsTheProducer) {
    constexpr std::uint64_t kItems = 20'000;
    ConsumerRing<std::uint64_t> ring(64, Overflow::DropNewest);
    std::vector<std::uint64_t> got;
    std::thread consumer([&] {
        while (const auto item = ring.popWait()) {
            got.push_back(*item);
            std::this_thread::sleep_for(20us);  // slower than the producer
        }
    });
    const auto start = std::chrono::steady_clock::now();
    for (std::uint64_t k = 0; k < kItems; ++k) {
        ring.push(k);
    }
    const auto producing = std::chrono::steady_clock::now() - start;
    ring.close();
    consumer.join();

    const auto stats = ring.stats();
    EXPECT_GT(stats.dropped, 0U);
    EXPECT_EQ(stats.blocked, 0U);
    EXPECT_EQ(stats.pushed + stats.dropped, kItems);
    EXPECT_EQ(got.size(), stats.pushed);
    for (std::size_t i = 1; i < got.size(); ++i) {
        ASSERT_LT(got[i - 1], got[i]) << "in order, without duplicates";
    }
    EXPECT_LE(stats.maxDepth, ring.capacity());
    // The consumer alone would need kItems x 20 us = 0.4 s; the producer took far less.
    EXPECT_LT(producing, 200ms);
}

// The recorder: nothing may be lost, so a slow consumer holds the producer back, and memory
// stays bounded by the capacity.
TEST(ConsumerRing, ASlowLosslessConsumerGetsEverythingWithBoundedMemory) {
    constexpr std::uint64_t kItems = 5'000;
    ConsumerRing<std::vector<double>> ring(16, Overflow::Block);
    std::uint64_t next = 0;
    bool inOrder = true;
    std::thread consumer([&] {
        while (const auto item = ring.popWait()) {
            inOrder = inOrder && item->size() == 3 && (*item)[0] == static_cast<double>(next);
            ++next;
            if (next % 100 == 0) {
                std::this_thread::sleep_for(1ms);  // a slow disk now and then
            }
        }
    });
    for (std::uint64_t k = 0; k < kItems; ++k) {
        ASSERT_TRUE(ring.push({static_cast<double>(k), 0.0, 0.0}));
    }
    ring.close();
    consumer.join();

    const auto stats = ring.stats();
    EXPECT_EQ(next, kItems);
    EXPECT_TRUE(inOrder);
    EXPECT_EQ(stats.dropped, 0U);
    EXPECT_GT(stats.blocked, 0U) << "the producer was held back";
    EXPECT_LE(stats.maxDepth, ring.capacity());
}

TEST(ConsumerRing, CloseWakesAWaitingConsumerAndRefusesPushes) {
    ConsumerRing<int> ring(2, Overflow::Block);
    std::thread consumer([&] { EXPECT_FALSE(ring.popWait().has_value()); });
    std::this_thread::sleep_for(10ms);
    ring.close();
    consumer.join();
    EXPECT_FALSE(ring.push(1));
}

}  // namespace
}  // namespace lagrange::services
