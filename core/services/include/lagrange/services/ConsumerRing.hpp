#pragma once

// Tick records to consumers (roadmap 0.2-E3): one producer (the simulation thread) hands items
// to one consumer thread (recorder, protocol, ROS) through a bounded ring, so a slow consumer
// either loses items or holds the producer back, as its policy says, and never grows memory.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace lagrange::services {

/// What push() does when the ring is full.
enum class Overflow : std::uint8_t {
    DropNewest,  // the new item is dropped and counted; the producer never waits (viewers)
    Block,       // the producer waits for space (back-pressure; consumers that must not lose
                 // data, such as the recorder)
};

/// A bounded single-producer, single-consumer ring. push() is called from one thread and
/// pop()/popWait() from one other thread; close() from either. Lock-free: indices are atomics,
/// and waiting uses C++20 atomic wait/notify.
template <class T>
class ConsumerRing {
public:
    struct Stats {
        std::uint64_t pushed{0};    // items accepted
        std::uint64_t dropped{0};   // items refused (DropNewest)
        std::uint64_t blocked{0};   // times the producer had to wait (Block)
        std::uint64_t maxDepth{0};  // most items in the ring at once
    };

    ConsumerRing(std::size_t capacity, Overflow overflow)
        : slots_(capacity > 0 ? capacity : 1), overflow_(overflow) {}
    ConsumerRing(const ConsumerRing&) = delete;
    ConsumerRing& operator=(const ConsumerRing&) = delete;
    ConsumerRing(ConsumerRing&&) = delete;
    ConsumerRing& operator=(ConsumerRing&&) = delete;
    ~ConsumerRing() = default;

    /// Producer. Returns false when the item was dropped, or the ring is closed.
    bool push(T item) {
        const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
        for (;;) {
            if (closed_.load(std::memory_order_acquire)) {
                return false;
            }
            const std::uint32_t freed = freed_.load(std::memory_order_acquire);
            if (tail - head_.load(std::memory_order_acquire) < slots_.size()) {
                break;
            }
            if (overflow_ == Overflow::DropNewest) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            blocked_.fetch_add(1, std::memory_order_relaxed);
            freed_.wait(freed, std::memory_order_acquire);  // until the consumer frees a slot
        }
        slots_[tail % slots_.size()] = std::move(item);
        tail_.store(tail + 1, std::memory_order_release);
        const std::uint64_t depth = tail + 1 - head_.load(std::memory_order_acquire);
        if (depth > maxDepth_.load(std::memory_order_relaxed)) {
            maxDepth_.store(depth, std::memory_order_relaxed);
        }
        pushed_.fetch_add(1, std::memory_order_relaxed);
        signal(events_);
        return true;
    }

    /// Consumer: the next item if there is one.
    std::optional<T> pop() {
        const std::uint64_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        std::optional<T> out = std::move(slots_[head % slots_.size()]);
        slots_[head % slots_.size()].reset();
        head_.store(head + 1, std::memory_order_release);
        signal(freed_);
        return out;
    }

    /// Consumer: waits for the next item; nullopt once the ring is closed and empty.
    std::optional<T> popWait() {
        for (;;) {
            const std::uint32_t seen = events_.load(std::memory_order_acquire);
            if (std::optional<T> item = pop()) {
                return item;
            }
            if (closed_.load(std::memory_order_acquire)) {
                return pop();  // whatever arrived before the close
            }
            events_.wait(seen, std::memory_order_acquire);
        }
    }

    /// No more pushes; wakes a waiting consumer (which drains what is left) and producer.
    void close() {
        closed_.store(true, std::memory_order_release);
        signal(events_);
        signal(freed_);
    }

    [[nodiscard]] Stats stats() const {
        return {pushed_.load(std::memory_order_relaxed), dropped_.load(std::memory_order_relaxed),
                blocked_.load(std::memory_order_relaxed),
                maxDepth_.load(std::memory_order_relaxed)};
    }
    [[nodiscard]] std::size_t capacity() const { return slots_.size(); }

private:
    static void signal(std::atomic<std::uint32_t>& counter) {
        counter.fetch_add(1, std::memory_order_release);
        counter.notify_all();
    }

    std::vector<std::optional<T>> slots_;
    Overflow overflow_;
    std::atomic<std::uint64_t> head_{0};    // next item to pop (consumer)
    std::atomic<std::uint64_t> tail_{0};    // next slot to fill (producer)
    std::atomic<std::uint32_t> events_{0};  // bumped on push and close: the consumer waits on it
    std::atomic<std::uint32_t> freed_{0};   // bumped on pop and close: the producer waits on it
    std::atomic<bool> closed_{false};
    std::atomic<std::uint64_t> pushed_{0};
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<std::uint64_t> blocked_{0};
    std::atomic<std::uint64_t> maxDepth_{0};
};

}  // namespace lagrange::services
