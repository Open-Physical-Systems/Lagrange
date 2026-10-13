#include "lagrange/viewer/ScopeBuffer.hpp"

#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::ScopeBuffer;
constexpr std::int64_t kDt = 100'000;  // 10 kHz

void push(ScopeBuffer& buffer, std::uint64_t first, const std::vector<double>& values) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    buffer.push(first, reinterpret_cast<const std::byte*>(values.data()), values.size(),
                sizeof(double));
}

std::vector<double> ramp(std::uint64_t first, std::size_t count) {
    std::vector<double> values(count);
    for (std::size_t i = 0; i < count; ++i) {
        values[i] = static_cast<double>(first + i);
    }
    return values;
}

// Reference: minimum and maximum of the samples in [t0, t1) straight from the definition.
ScopeBuffer::Column bruteForce(const std::vector<double>& all, std::uint64_t firstIndex,
                               std::int64_t t0, std::int64_t t1) {
    float low = std::numeric_limits<float>::quiet_NaN();
    float high = low;
    for (std::size_t i = 0; i < all.size(); ++i) {
        const auto time = static_cast<std::int64_t>(firstIndex + i) * kDt;
        const auto value = static_cast<float>(all[i]);
        if (time >= t0 && time < t1 && value == value) {
            low = low == low ? std::min(low, value) : value;
            high = high == high ? std::max(high, value) : value;
        }
    }
    return {low, high};
}

TEST(ScopeBuffer, StartsEmpty) {
    const ScopeBuffer buffer(kDt, 1024);
    EXPECT_EQ(buffer.newestTimeNs(), -1);
    EXPECT_EQ(buffer.oldestTimeNs(), -1);
    EXPECT_TRUE(std::isnan(buffer.latest()));
    std::vector<ScopeBuffer::Column> columns(8);
    buffer.decimate(0, 1'000'000, columns);
    EXPECT_TRUE(std::isnan(columns[0].min));
}

TEST(ScopeBuffer, CapacityIsRoundedUpToAPowerOfTwo) {
    EXPECT_EQ(ScopeBuffer(kDt, 1000).capacity(), 1024U);
    EXPECT_EQ(ScopeBuffer(kDt, 1).capacity(), 2 * ScopeBuffer::kBlock);
}

TEST(ScopeBuffer, TimesFollowTheAbsoluteSampleIndex) {
    ScopeBuffer buffer(kDt, 1024);
    push(buffer, 5000, ramp(5000, 100));  // a stream that starts mid-recording
    EXPECT_EQ(buffer.oldestTimeNs(), 5000 * kDt);
    EXPECT_EQ(buffer.newestTimeNs(), 5099 * kDt);
    EXPECT_DOUBLE_EQ(buffer.latest(), 5099.0);
    EXPECT_EQ(buffer.totalSamples(), 100U);
    EXPECT_EQ(buffer.droppedSamples(), 0U);
}

TEST(ScopeBuffer, RawReadReturnsSamplesAndTimesInTheWindow) {
    ScopeBuffer buffer(kDt, 1024);
    push(buffer, 0, ramp(0, 50));
    std::vector<double> times(100);
    std::vector<double> values(100);
    const std::size_t n = buffer.readRaw(10 * kDt, 15 * kDt, times, values);
    ASSERT_EQ(n, 5U);
    EXPECT_DOUBLE_EQ(values[0], 10.0);
    EXPECT_DOUBLE_EQ(values[4], 14.0);
    EXPECT_DOUBLE_EQ(times[0], 10 * kDt * 1e-9);
    EXPECT_EQ(buffer.readRaw(10 * kDt, 15 * kDt, std::span{times}.first(2), values), 2U);
}

TEST(ScopeBuffer, DecimationMatchesBruteForceForManyWindows) {
    // Random data pushed in irregular runs, then many random windows and column counts,
    // including windows that cut through blocks and reach beyond the data.
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> value(-50.0, 50.0);
    const std::uint64_t first = 1234;  // not block-aligned on purpose
    std::vector<double> all(20'000);
    for (double& v : all) {
        v = value(rng);
    }
    ScopeBuffer buffer(kDt, 1 << 15);
    for (std::size_t offset = 0; offset < all.size();) {
        const std::size_t n = std::min<std::size_t>(rng() % 700 + 1, all.size() - offset);
        push(buffer, first + offset,
             std::vector<double>(all.begin() + static_cast<std::ptrdiff_t>(offset),
                                 all.begin() + static_cast<std::ptrdiff_t>(offset + n)));
        offset += n;
    }

    const std::int64_t dataBegin = static_cast<std::int64_t>(first) * kDt;
    const std::int64_t dataEnd = static_cast<std::int64_t>(first + all.size()) * kDt;
    for (int trial = 0; trial < 300; ++trial) {
        const std::int64_t a =
            dataBegin - 500 * kDt + static_cast<std::int64_t>(rng() % 21'000) * kDt + rng() % kDt;
        const std::int64_t b = a + 1 + static_cast<std::int64_t>(rng() % 15'000) * kDt;
        std::vector<ScopeBuffer::Column> columns(rng() % 400 + 1);
        buffer.decimate(a, b, columns);
        const auto count = static_cast<std::int64_t>(columns.size());
        for (std::int64_t c = 0; c < count; ++c) {
            const std::int64_t t0 = a + (b - a) * c / count;
            const std::int64_t t1 = a + (b - a) * (c + 1) / count;
            const ScopeBuffer::Column expected = bruteForce(all, first, t0, t1);
            const ScopeBuffer::Column& got = columns[static_cast<std::size_t>(c)];
            if (std::isnan(expected.min)) {
                ASSERT_TRUE(std::isnan(got.min)) << "trial " << trial << " column " << c;
            } else {
                ASSERT_FLOAT_EQ(got.min, expected.min) << "trial " << trial << " column " << c;
                ASSERT_FLOAT_EQ(got.max, expected.max) << "trial " << trial << " column " << c;
            }
        }
    }
    EXPECT_GT(dataEnd, dataBegin);
}

TEST(ScopeBuffer, OldSamplesFallOutWhenTheRingWraps) {
    ScopeBuffer buffer(kDt, 256);
    push(buffer, 0, ramp(0, 1000));
    EXPECT_EQ(buffer.newestTimeNs(), 999 * kDt);
    EXPECT_EQ(buffer.oldestTimeNs(), (1000 - 256) * kDt);
    std::vector<ScopeBuffer::Column> columns(1);
    buffer.decimate(0, 1000 * kDt, columns);
    EXPECT_FLOAT_EQ(columns[0].min, 744.0F);  // 0..743 are gone
    EXPECT_FLOAT_EQ(columns[0].max, 999.0F);
    buffer.decimate(0, 700 * kDt, columns);
    EXPECT_TRUE(std::isnan(columns[0].min));
}

TEST(ScopeBuffer, GapIsCountedAndShownAsMissingData) {
    ScopeBuffer buffer(kDt, 4096);
    push(buffer, 0, ramp(0, 100));
    push(buffer, 150, ramp(150, 100));  // samples 100..149 never arrived
    EXPECT_EQ(buffer.droppedSamples(), 50U);
    EXPECT_EQ(buffer.totalSamples(), 200U);
    EXPECT_EQ(buffer.newestTimeNs(), 249 * kDt);

    std::vector<ScopeBuffer::Column> columns(1);
    buffer.decimate(100 * kDt, 150 * kDt, columns);
    EXPECT_TRUE(std::isnan(columns[0].min));  // nothing invented inside the gap
    buffer.decimate(90 * kDt, 160 * kDt, columns);
    EXPECT_FLOAT_EQ(columns[0].min, 90.0F);
    EXPECT_FLOAT_EQ(columns[0].max, 159.0F);
    buffer.decimate(0, 250 * kDt, columns);  // block summaries skip the gap too
    EXPECT_FLOAT_EQ(columns[0].min, 0.0F);
    EXPECT_FLOAT_EQ(columns[0].max, 249.0F);
}

TEST(ScopeBuffer, HugeGapRestartsTheHistory) {
    ScopeBuffer buffer(kDt, 256);
    push(buffer, 0, ramp(0, 10));
    push(buffer, 1'000'000, ramp(1'000'000, 10));
    EXPECT_EQ(buffer.droppedSamples(), 1'000'000U - 10U);
    EXPECT_EQ(buffer.oldestTimeNs(), 1'000'000 * kDt);
}

TEST(ScopeBuffer, StreamThatGoesBackInTimeStartsOver) {
    ScopeBuffer buffer(kDt, 1024);
    push(buffer, 5000, ramp(5000, 100));
    push(buffer, 20, ramp(20, 10));  // e.g. the replay was rewound
    EXPECT_EQ(buffer.oldestTimeNs(), 20 * kDt);
    EXPECT_EQ(buffer.newestTimeNs(), 29 * kDt);
    EXPECT_EQ(buffer.droppedSamples(), 0U);
}

TEST(ScopeBuffer, ResetEmptiesAndAllowsANewStepSize) {
    ScopeBuffer buffer(kDt, 1024);
    push(buffer, 0, ramp(0, 100));
    buffer.reset(1'000'000);
    EXPECT_EQ(buffer.newestTimeNs(), -1);
    EXPECT_EQ(buffer.dtNs(), 1'000'000);
    push(buffer, 7, ramp(7, 3));  // a forward jump right after a reset is not a gap
    EXPECT_EQ(buffer.droppedSamples(), 0U);
    EXPECT_EQ(buffer.oldestTimeNs(), 7'000'000);
    EXPECT_EQ(buffer.newestTimeNs(), 9'000'000);
}

TEST(ScopeBuffer, TakesOneComponentOutOfAnInterleavedUnalignedRun) {
    // Three components per sample, starting at an odd address.
    std::vector<std::byte> bytes(1 + 4 * 3 * sizeof(double));
    for (int sample = 0; sample < 4; ++sample) {
        for (int component = 0; component < 3; ++component) {
            const double value = sample * 10.0 + component;
            std::memcpy(bytes.data() + 1 + (sample * 3 + component) * sizeof(double), &value,
                        sizeof(double));
        }
    }
    ScopeBuffer buffer(kDt, 256);
    buffer.push(0, bytes.data() + 1 + 2 * sizeof(double), 4, 3 * sizeof(double));  // component 2
    std::vector<double> times(4);
    std::vector<double> values(4);
    ASSERT_EQ(buffer.readRaw(0, 4 * kDt, times, values), 4U);
    EXPECT_EQ(values, (std::vector<double>{2.0, 12.0, 22.0, 32.0}));
}

TEST(ScopeBuffer, ReaderNeverSeesValuesThatWereNotWritten) {
    // The writer pushes a ramp (value == index) as fast as it can through a small ring while
    // the reader decimates. Whatever the reader gets must lie inside what could be valid for
    // the window it asked for; a torn read would show up as a value outside it.
    ScopeBuffer buffer(kDt, 4096);
    std::atomic<bool> done{false};
    std::jthread writer([&] {  // joins even if an assertion below returns early
        std::uint64_t index = 0;
        std::vector<double> chunk(173);
        while (index < 3'000'000) {
            for (std::size_t i = 0; i < chunk.size(); ++i) {
                chunk[i] = static_cast<double>(index + i);
            }
            push(buffer, index, chunk);
            index += chunk.size();
        }
        done = true;
    });
    std::vector<ScopeBuffer::Column> columns(64);
    std::uint64_t checked = 0;
    while (!done) {
        const std::int64_t newest = buffer.newestTimeNs();
        if (newest < 0) {
            continue;
        }
        const std::int64_t begin = newest - 3000 * kDt;
        const std::int64_t end = newest + kDt;
        buffer.decimate(begin, end, columns);
        for (std::size_t c = 0; c < columns.size(); ++c) {
            if (std::isnan(columns[c].min)) {
                continue;
            }
            const auto count = static_cast<std::int64_t>(columns.size());
            const std::int64_t t0 = begin + (end - begin) * static_cast<std::int64_t>(c) / count;
            const std::int64_t t1 =
                begin + (end - begin) * static_cast<std::int64_t>(c + 1) / count;
            // value == index == time / dt, so both ends must lie inside the column's span
            // (allowing float rounding of large indices).
            ASSERT_GE(columns[c].min, static_cast<float>(t0 / kDt) - 1.0F);
            ASSERT_LE(columns[c].max, static_cast<float>(t1 / kDt) + 1.0F);
            ++checked;
        }
    }
    writer.join();
    EXPECT_GT(checked, 0U);

    // The same for raw reads: every value returned must be the sample at its own time.
    ScopeBuffer second(kDt, 4096);
    std::atomic<bool> secondDone{false};
    std::jthread secondWriter([&] {
        std::vector<double> chunk(97);
        for (std::uint64_t index = 0; index < 2'000'000; index += chunk.size()) {
            for (std::size_t i = 0; i < chunk.size(); ++i) {
                chunk[i] = static_cast<double>(index + i);
            }
            push(second, index, chunk);
        }
        secondDone = true;
    });
    std::vector<double> times(512);
    std::vector<double> values(512);
    std::uint64_t rawChecked = 0;
    while (!secondDone) {
        const std::int64_t newest = second.newestTimeNs();
        if (newest < 0) {
            continue;
        }
        const std::size_t n =
            second.readRaw(newest - 3500 * kDt, newest - 3000 * kDt, times, values);
        for (std::size_t i = 0; i < n; ++i) {
            ASSERT_NEAR(values[i], times[i] * 1e9 / kDt, 0.5);
            ++rawChecked;
        }
    }
    secondWriter.join();
    EXPECT_GT(rawChecked, 0U);
}

}  // namespace
