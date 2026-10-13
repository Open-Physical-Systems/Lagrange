#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace lagrange::viewer {

/// History of one signal component for a waveform scope.
///
/// One thread writes (the protocol client's worker) and one reads (the UI) without locks.
/// Sample k of the signal is at time k * dt; the buffer keeps the most recent
/// `capacity - kBlock` samples by their absolute index, so the time axis stays exact however
/// the data arrived. (One block is held back because the writer may be that far ahead of the
/// position it has published.)
///
/// A scope usually has far more samples than pixels. The writer therefore also keeps the
/// minimum and maximum of every aligned block of kBlock samples, and decimate() uses whole
/// blocks where it can.
class ScopeBuffer {
public:
    static constexpr std::size_t kBlock = 64;
    static constexpr std::size_t kDefaultCapacity = std::size_t{1} << 20;  // 104 s at 10 kHz

    /// `capacity` is rounded up to a power of two that is at least 2 * kBlock.
    explicit ScopeBuffer(std::int64_t dtNs, std::size_t capacity = kDefaultCapacity);

    // --- writer thread ---------------------------------------------------------------------

    /// Appends `count` samples whose first has absolute index `firstIndex`. `data` points at
    /// little-endian f64 values `strideBytes` apart (so one component can be taken out of an
    /// interleaved run without copying). It need not be aligned.
    ///
    /// If `firstIndex` is beyond the next expected index, the missing samples are counted as
    /// dropped and shown as a gap. If it is before, the stream restarted and the history is
    /// discarded.
    void push(std::uint64_t firstIndex, const std::byte* data, std::size_t count,
              std::size_t strideBytes);

    /// Discards the history: the next push starts a new stream (after a seek, a restart, or a
    /// change of step size).
    void reset(std::int64_t dtNs);

    // --- reader thread ---------------------------------------------------------------------

    struct Column {
        float min;
        float max;  ///< Both NaN when the column has no sample.
    };

    /// Splits [tBeginNs, tEndNs) into out.size() equal columns and gives the minimum and
    /// maximum of the samples in each. This is what a scope draws when zoomed out.
    void decimate(std::int64_t tBeginNs, std::int64_t tEndNs, std::span<Column> out) const;

    /// Copies up to out sizes of the samples in [tBeginNs, tEndNs) with their times in
    /// seconds. Returns how many were written. For drawing when zoomed in.
    std::size_t readRaw(std::int64_t tBeginNs, std::int64_t tEndNs, std::span<double> timesSeconds,
                        std::span<double> values) const;

    [[nodiscard]] std::int64_t dtNs() const { return dtNs_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::size_t capacity() const { return capacity_; }
    /// Time of the newest sample, or -1 if the buffer is empty.
    [[nodiscard]] std::int64_t newestTimeNs() const;
    /// Time of the oldest sample still held, or -1 if the buffer is empty.
    [[nodiscard]] std::int64_t oldestTimeNs() const;
    /// The newest sample, or NaN if the buffer is empty.
    [[nodiscard]] double latest() const;
    /// Samples the stream skipped (gaps between runs) since construction.
    [[nodiscard]] std::uint64_t droppedSamples() const {
        return dropped_.load(std::memory_order_relaxed);
    }
    /// Samples received since construction.
    [[nodiscard]] std::uint64_t totalSamples() const {
        return total_.load(std::memory_order_relaxed);
    }

private:
    struct Range {
        std::uint64_t begin;  // first valid absolute index
        std::uint64_t end;    // one past the newest
    };
    [[nodiscard]] Range range() const;
    void finishBlock(std::uint64_t block);

    std::size_t capacity_;
    std::size_t mask_;
    std::size_t blockMask_;
    // Atomics with relaxed ordering: on the supported CPUs these are plain loads and stores,
    // and they make concurrent reading well defined. `end_` publishes what is readable.
    std::unique_ptr<std::atomic<double>[]> samples_;
    std::unique_ptr<std::atomic<float>[]> blockMin_;
    std::unique_ptr<std::atomic<float>[]> blockMax_;
    std::atomic<std::uint64_t> base_{0};  // first index of the current stream
    std::atomic<std::uint64_t> end_{0};
    std::atomic<std::int64_t> dtNs_;
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<std::uint64_t> total_{0};
    bool started_{false};  // writer only
};

}  // namespace lagrange::viewer
