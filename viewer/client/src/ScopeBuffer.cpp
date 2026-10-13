#include "lagrange/viewer/ScopeBuffer.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>

namespace lagrange::viewer {

namespace {

constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Smallest k with k * dt >= t, for t >= 0.
std::uint64_t ceilIndex(std::int64_t timeNs, std::int64_t dtNs) {
    if (timeNs <= 0) {
        return 0;
    }
    return static_cast<std::uint64_t>((timeNs + dtNs - 1) / dtNs);
}

}  // namespace

ScopeBuffer::ScopeBuffer(std::int64_t dtNs, std::size_t capacity)
    : capacity_(std::bit_ceil(std::max(capacity, 2 * kBlock)))
    , mask_(capacity_ - 1)
    , blockMask_(capacity_ / kBlock - 1)
    , samples_(std::make_unique<std::atomic<double>[]>(capacity_))
    , blockMin_(std::make_unique<std::atomic<float>[]>(capacity_ / kBlock))
    , blockMax_(std::make_unique<std::atomic<float>[]>(capacity_ / kBlock))
    , dtNs_(std::max<std::int64_t>(dtNs, 1)) {}

void ScopeBuffer::reset(std::int64_t dtNs) {
    // Publishing end == base makes the buffer read as empty; the next push sets a new base.
    const std::uint64_t end = end_.load(std::memory_order_relaxed);
    base_.store(end, std::memory_order_relaxed);
    dtNs_.store(std::max<std::int64_t>(dtNs, 1), std::memory_order_relaxed);
    started_ = false;
}

void ScopeBuffer::finishBlock(std::uint64_t block) {
    float low = kNaNf;
    float high = kNaNf;
    const std::uint64_t first = block * kBlock;
    for (std::uint64_t i = first; i < first + kBlock; ++i) {
        const auto value = static_cast<float>(samples_[i & mask_].load(std::memory_order_relaxed));
        if (value == value) {  // not NaN
            low = low == low ? std::min(low, value) : value;
            high = high == high ? std::max(high, value) : value;
        }
    }
    blockMin_[block & blockMask_].store(low, std::memory_order_relaxed);
    blockMax_[block & blockMask_].store(high, std::memory_order_relaxed);
}

void ScopeBuffer::push(std::uint64_t firstIndex, const std::byte* data, std::size_t count,
                       std::size_t strideBytes) {
    std::uint64_t end = end_.load(std::memory_order_relaxed);
    if (!started_ || firstIndex < end) {
        // First run of a stream, or the stream went back in time: start over at firstIndex.
        base_.store(firstIndex, std::memory_order_relaxed);
        end = firstIndex;
        end_.store(end, std::memory_order_release);
        started_ = true;
    } else if (firstIndex > end) {
        const std::uint64_t gap = firstIndex - end;
        dropped_.fetch_add(gap, std::memory_order_relaxed);
        if (gap >= capacity_) {
            base_.store(firstIndex, std::memory_order_relaxed);
            end = firstIndex;
        } else {
            while (end < firstIndex) {  // a visible gap in the trace
                samples_[end & mask_].store(kNaN, std::memory_order_relaxed);
                ++end;
                if (end % kBlock == 0) {
                    finishBlock(end / kBlock - 1);
                    end_.store(end, std::memory_order_release);
                }
            }
        }
        end_.store(end, std::memory_order_release);
    }

    for (std::size_t i = 0; i < count; ++i) {
        double value = 0.0;
        std::memcpy(&value, data + i * strideBytes, sizeof(value));
        samples_[end & mask_].store(value, std::memory_order_relaxed);
        ++end;
        if (end % kBlock == 0) {
            finishBlock(end / kBlock - 1);
            end_.store(end, std::memory_order_release);  // publish block by block
        }
    }
    end_.store(end, std::memory_order_release);
    total_.fetch_add(count, std::memory_order_relaxed);
}

ScopeBuffer::Range ScopeBuffer::range() const {
    const std::uint64_t end = end_.load(std::memory_order_acquire);
    const std::uint64_t base = base_.load(std::memory_order_relaxed);
    if (end <= base) {
        return {end, end};
    }
    // The writer publishes `end_` once per block, so it may already have written up to
    // kBlock - 1 samples past it, overwriting the oldest slots. Those are not readable.
    const std::size_t readable = capacity_ - kBlock;
    return {std::max(base, end > readable ? end - readable : 0), end};
}

std::int64_t ScopeBuffer::newestTimeNs() const {
    const Range r = range();
    return r.begin == r.end ? -1 : static_cast<std::int64_t>(r.end - 1) * dtNs();
}

std::int64_t ScopeBuffer::oldestTimeNs() const {
    const Range r = range();
    return r.begin == r.end ? -1 : static_cast<std::int64_t>(r.begin) * dtNs();
}

double ScopeBuffer::latest() const {
    const Range r = range();
    return r.begin == r.end ? kNaN : samples_[(r.end - 1) & mask_].load(std::memory_order_relaxed);
}

void ScopeBuffer::decimate(std::int64_t tBeginNs, std::int64_t tEndNs,
                           std::span<Column> out) const {
    const std::int64_t dt = dtNs();
    const Range valid = range();
    const auto columns = static_cast<std::int64_t>(out.size());
    for (std::int64_t c = 0; c < columns; ++c) {
        Column& column = out[static_cast<std::size_t>(c)];
        column = {kNaNf, kNaNf};
        if (tEndNs <= tBeginNs) {
            continue;
        }
        // Column c covers [t0, t1). Computed from the ends so that columns tile exactly.
        const std::int64_t span = tEndNs - tBeginNs;
        const std::int64_t t0 = tBeginNs + span * c / columns;
        const std::int64_t t1 = tBeginNs + span * (c + 1) / columns;
        std::uint64_t i = std::max(ceilIndex(t0, dt), valid.begin);
        const std::uint64_t last = std::min(ceilIndex(t1, dt), valid.end);
        float low = kNaNf;
        float high = kNaNf;
        const auto add = [&](float lo, float hi) {
            if (lo == lo) {
                low = low == low ? std::min(low, lo) : lo;
                high = high == high ? std::max(high, hi) : hi;
            }
        };
        while (i < last) {
            if (i % kBlock == 0 && i + kBlock <= last) {
                const std::uint64_t block = i / kBlock;
                add(blockMin_[block & blockMask_].load(std::memory_order_relaxed),
                    blockMax_[block & blockMask_].load(std::memory_order_relaxed));
                i += kBlock;
            } else {
                const auto value =
                    static_cast<float>(samples_[i & mask_].load(std::memory_order_relaxed));
                add(value, value);
                ++i;
            }
        }
        column = {low, high};
    }
    // The writer may have overwritten the oldest samples while we read. Blank the columns
    // that could have been affected instead of showing mixed data. (range() already leaves
    // out the block the writer may be ahead by.)
    const Range after = range();
    if (after.begin > valid.begin && tEndNs > tBeginNs) {
        const std::int64_t staleUntil = static_cast<std::int64_t>(after.begin) * dt;
        const std::int64_t span = tEndNs - tBeginNs;
        for (std::int64_t c = 0; c < columns; ++c) {
            if (tBeginNs + span * c / columns < staleUntil) {
                out[static_cast<std::size_t>(c)] = {kNaNf, kNaNf};
            }
        }
    }
}

std::size_t ScopeBuffer::readRaw(std::int64_t tBeginNs, std::int64_t tEndNs,
                                 std::span<double> timesSeconds, std::span<double> values) const {
    const std::int64_t dt = dtNs();
    const Range valid = range();
    std::uint64_t i = std::max(ceilIndex(tBeginNs, dt), valid.begin);
    const std::uint64_t firstRead = i;
    const std::uint64_t last = std::min(ceilIndex(tEndNs, dt), valid.end);
    const std::size_t room = std::min(timesSeconds.size(), values.size());
    std::size_t written = 0;
    for (; i < last && written < room; ++i, ++written) {
        timesSeconds[written] = static_cast<double>(static_cast<std::int64_t>(i) * dt) * 1e-9;
        values[written] = samples_[i & mask_].load(std::memory_order_relaxed);
    }
    const Range after = range();
    if (written > 0 && after.begin > firstRead) {
        return 0;  // possibly overwritten while reading; the caller draws it next frame
    }
    return written;
}

ScopeBuffer::Statistics ScopeBuffer::statistics(std::int64_t tBeginNs, std::int64_t tEndNs) const {
    const std::int64_t dt = dtNs();
    const Range valid = range();
    const std::uint64_t first = std::max(ceilIndex(tBeginNs, dt), valid.begin);
    const std::uint64_t last = std::min(ceilIndex(tEndNs, dt), valid.end);
    Statistics result;
    double sum = 0.0;
    double squares = 0.0;
    for (std::uint64_t i = first; i < last; ++i) {
        const double value = samples_[i & mask_].load(std::memory_order_relaxed);
        if (value != value) {
            continue;  // a gap
        }
        result.min = result.count == 0 ? value : std::min(result.min, value);
        result.max = result.count == 0 ? value : std::max(result.max, value);
        sum += value;
        squares += value * value;
        ++result.count;
    }
    if (result.count != 0) {
        result.mean = sum / static_cast<double>(result.count);
        result.rms = std::sqrt(squares / static_cast<double>(result.count));
    }
    // Statistics are a display aid: if the writer wrapped past the start of the range while
    // we read, the caller gets the numbers of a moment later on its next call.
    return result;
}

double ScopeBuffer::valueAt(std::int64_t tNs) const {
    const std::int64_t dt = dtNs();
    const Range valid = range();
    if (tNs < 0 || dt <= 0) {
        return kNaN;
    }
    const auto index = static_cast<std::uint64_t>((tNs + dt / 2) / dt);
    if (index < valid.begin || index >= valid.end) {
        return kNaN;
    }
    const double value = samples_[index & mask_].load(std::memory_order_relaxed);
    return range().begin > index ? kNaN : value;
}

}  // namespace lagrange::viewer
