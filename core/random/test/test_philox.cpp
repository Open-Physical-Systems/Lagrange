#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::random {
namespace {

// Known-answer vectors for philox4x32 with 10 rounds, from tests/kat_vectors in
// https://github.com/DEShawResearch/random123 (counter words, then key words, then output).
TEST(Philox, KnownAnswers) {
    EXPECT_EQ(philox4x32_10({0, 0, 0, 0}, {0, 0}),
              (Counter{0x6627e8d5U, 0xe169c58dU, 0xbc57ac4cU, 0x9b00dbd8U}));
    EXPECT_EQ(philox4x32_10({0xffffffffU, 0xffffffffU, 0xffffffffU, 0xffffffffU},
                            {0xffffffffU, 0xffffffffU}),
              (Counter{0x408f276dU, 0x41c83b0eU, 0xa20bc7c6U, 0x6d5451fdU}));
    EXPECT_EQ(philox4x32_10({0x243f6a88U, 0x85a308d3U, 0x13198a2eU, 0x03707344U},
                            {0xa4093822U, 0x299f31d0U}),
              (Counter{0xd16cfe09U, 0x94fdccebU, 0x5001e420U, 0x24126ea1U}));
}

std::vector<double> draws(std::uint64_t seed, std::uint64_t step, std::uint32_t entity, int n) {
    Stream rng(seed, step, entity);
    std::vector<double> out;
    for (int i = 0; i < n; ++i) {
        out.push_back(rng.uniform());
    }
    return out;
}

TEST(Stream, SameKeyGivesTheSameNumbersInAnyOrder) {
    const std::uint64_t seed = moduleSeed(42, 7);
    // Entities stepped in a different order, or on another thread, see the same values.
    const auto b = draws(seed, 100, 2, 16);
    const auto a = draws(seed, 100, 1, 16);
    EXPECT_EQ(draws(seed, 100, 1, 16), a);
    EXPECT_EQ(draws(seed, 100, 2, 16), b);
    EXPECT_NE(a, b);
}

TEST(Stream, StepsEntitiesModulesAndRunsAreIndependent) {
    std::set<std::vector<double>> seen;
    for (std::uint64_t run : {0ULL, 1ULL}) {
        for (std::uint32_t module : {0U, 1U}) {
            for (std::uint64_t step : {0ULL, 1ULL, 1ULL << 32U}) {
                for (std::uint32_t entity : {0U, 1U}) {
                    seen.insert(draws(moduleSeed(run, module), step, entity, 4));
                }
            }
        }
    }
    EXPECT_EQ(seen.size(), 2U * 2U * 3U * 2U);
}

TEST(Stream, ModuleSeedsDiffer) {
    std::set<std::uint64_t> seeds;
    for (std::uint32_t module = 0; module < 1000; ++module) {
        seeds.insert(moduleSeed(0, module));
    }
    EXPECT_EQ(seeds.size(), 1000U);
    EXPECT_NE(moduleSeed(0, 3), moduleSeed(1, 3));
}

TEST(Stream, UniformIsInRangeWithTheRightMean) {
    Stream rng(moduleSeed(1, 1), 0, 0);
    double sum = 0.0;
    constexpr int kCount = 200000;
    for (int i = 0; i < kCount; ++i) {
        const double u = rng.uniform();
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
        sum += u;
    }
    // Standard error of the mean of U(0,1) over 2e5 draws is about 6.5e-4.
    EXPECT_NEAR(sum / kCount, 0.5, 0.003);
}

TEST(Stream, NormalHasUnitVariance) {
    Stream rng(moduleSeed(1, 2), 0, 0);
    double sum = 0.0;
    double squares = 0.0;
    constexpr int kCount = 200000;
    for (int i = 0; i < kCount; ++i) {
        const double x = rng.normal();
        ASSERT_TRUE(std::isfinite(x));
        sum += x;
        squares += x * x;
    }
    const double mean = sum / kCount;
    EXPECT_NEAR(mean, 0.0, 0.01);
    EXPECT_NEAR(squares / kCount - mean * mean, 1.0, 0.02);
}

}  // namespace
}  // namespace lagrange::random
