#include "lagrange/time/SimTime.hpp"

#include <gtest/gtest.h>

namespace {

using lagrange::Duration;
using lagrange::parseDecimalSeconds;
using lagrange::SimTime;
using namespace lagrange::literals;

TEST(Duration, LiteralsAreExactNanoseconds) {
    EXPECT_EQ((100_us).ns, 100'000);
    EXPECT_EQ((1_ms).ns, 1'000'000);
    EXPECT_EQ((2_s).ns, 2'000'000'000);
    EXPECT_EQ(10 * 100_us, 1_ms);
}

TEST(Duration, ArithmeticAndOrdering) {
    EXPECT_EQ(1_ms + 500_us, Duration{1'500'000});
    EXPECT_EQ(1_ms - 1_us, Duration{999'000});
    EXPECT_LT(100_us, 1_ms);
    EXPECT_DOUBLE_EQ((1_ms).seconds(), 1e-3);
}

TEST(SimTime, AdvancesByDurations) {
    SimTime t{};
    for (int i = 0; i < 10; ++i) {
        t = t + 100_us;
    }
    EXPECT_EQ(t, SimTime{1'000'000});
    EXPECT_EQ(t - SimTime{}, 1_ms);
}

TEST(ParseDecimalSeconds, ReadsProfileStepSizesExactly) {
    EXPECT_EQ(parseDecimalSeconds("0.0001"), 100_us);
    EXPECT_EQ(parseDecimalSeconds("0.001"), 1_ms);
    EXPECT_EQ(parseDecimalSeconds("0.1"), 100_ms);
    EXPECT_EQ(parseDecimalSeconds("0.00005"), 50_us);
    EXPECT_EQ(parseDecimalSeconds("2"), 2_s);
    EXPECT_EQ(parseDecimalSeconds("+1.5"), Duration{1'500'000'000});
}

TEST(ParseDecimalSeconds, AcceptsExponents) {
    EXPECT_EQ(parseDecimalSeconds("1e-3"), 1_ms);
    EXPECT_EQ(parseDecimalSeconds("1E-4"), 100_us);
    EXPECT_EQ(parseDecimalSeconds("2e-9"), 2_ns);
    EXPECT_EQ(parseDecimalSeconds("0.5e1"), 5_s);
    EXPECT_EQ(parseDecimalSeconds("1e+1"), 10_s);
}

TEST(ParseDecimalSeconds, TenFastStepsEqualOneSlowStep) {
    // The reason time is not floating point: this must hold exactly.
    const auto fast = parseDecimalSeconds("0.0001");
    const auto slow = parseDecimalSeconds("0.001");
    ASSERT_TRUE(fast.has_value());
    ASSERT_TRUE(slow.has_value());
    EXPECT_EQ(10 * *fast, *slow);
}

TEST(ParseDecimalSeconds, ZeroInAnyFormIsZero) {
    EXPECT_EQ(parseDecimalSeconds("0"), Duration{0});
    EXPECT_EQ(parseDecimalSeconds("0.000000000000"), Duration{0});
    EXPECT_EQ(parseDecimalSeconds("0e50"), Duration{0});
}

TEST(ParseDecimalSeconds, RejectsFractionsOfANanosecond) {
    EXPECT_FALSE(parseDecimalSeconds("0.00000000015").has_value());
    EXPECT_FALSE(parseDecimalSeconds("1.5e-9").has_value());
    EXPECT_FALSE(parseDecimalSeconds("1e-10").has_value());
    EXPECT_FALSE(parseDecimalSeconds("0.0010000000001").has_value());
}

TEST(ParseDecimalSeconds, RejectsMalformedText) {
    for (const char* text :
         {"", "abc", ".", "-1", "1.2.3", "1e", "1e-", "0x10", " 1", "1 ", "1s"}) {
        EXPECT_FALSE(parseDecimalSeconds(text).has_value()) << "text: '" << text << "'";
    }
}

TEST(ParseDecimalSeconds, RejectsOverflow) {
    EXPECT_FALSE(parseDecimalSeconds("1e400").has_value());
    EXPECT_FALSE(parseDecimalSeconds("9223372037").has_value());  // just over 2^63 ns
    EXPECT_EQ(parseDecimalSeconds("9223372036"), Duration{9'223'372'036'000'000'000});
}

}  // namespace
