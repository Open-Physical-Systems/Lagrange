#include "lagrange/viewer/ScopeSession.hpp"

#include <cmath>
#include <sstream>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::ScopeBuffer;
using lagrange::viewer::ScopeSession;
using lagrange::viewer::writeCsv;
constexpr std::int64_t kDt = 100'000;  // 10 kHz

void push(ScopeBuffer& buffer, std::uint64_t first, const std::vector<double>& values) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    buffer.push(first, reinterpret_cast<const std::byte*>(values.data()), values.size(),
                sizeof(double));
}

std::shared_ptr<ScopeBuffer> buffer(std::int64_t dt = kDt) {
    return std::make_shared<ScopeBuffer>(dt, 1U << 16);
}

TEST(ScopeBufferStatistics, MatchTheDefinition) {
    ScopeBuffer samples(kDt, 1U << 12);
    push(samples, 0, {1.0, -3.0, 2.0, 4.0, 100.0});
    const ScopeBuffer::Statistics all = samples.statistics(0, 4 * kDt);  // the first four
    EXPECT_EQ(all.count, 4U);
    EXPECT_DOUBLE_EQ(all.min, -3.0);
    EXPECT_DOUBLE_EQ(all.max, 4.0);
    EXPECT_DOUBLE_EQ(all.mean, 1.0);
    EXPECT_DOUBLE_EQ(all.rms, std::sqrt(30.0 / 4.0));
    EXPECT_EQ(samples.statistics(10 * kDt, 20 * kDt).count, 0U);
}

TEST(ScopeBufferStatistics, ValueAtTakesTheNearestSample) {
    ScopeBuffer samples(kDt, 1U << 12);
    push(samples, 0, {10.0, 20.0, 30.0});
    EXPECT_DOUBLE_EQ(samples.valueAt(kDt), 20.0);
    EXPECT_DOUBLE_EQ(samples.valueAt(kDt + kDt / 4), 20.0);
    EXPECT_DOUBLE_EQ(samples.valueAt(2 * kDt - kDt / 4), 30.0);
    EXPECT_TRUE(std::isnan(samples.valueAt(3 * kDt)));
    EXPECT_TRUE(std::isnan(samples.valueAt(-1)));
}

TEST(ScopeSession, OnePlotPerUnitInOrderOfFirstUse) {
    ScopeSession session;
    session.adopt("a", "A", buffer());
    session.adopt("v", "V", buffer());
    session.adopt("b", "A", buffer());
    EXPECT_EQ(session.units(), (std::vector<std::string>{"A", "V"}));
    EXPECT_EQ(session.find("v"), 1);
    EXPECT_EQ(session.find("missing"), -1);
}

TEST(ScopeSession, ViewFollowsTheNewestSampleUntilPaused) {
    ScopeSession session;
    auto samples = buffer();
    session.adopt("a", "A", samples);
    push(*samples, 0, std::vector<double>(20'000, 1.0));  // 2 s
    session.window = 0;                                   // 1 s
    session.update();
    EXPECT_EQ(session.view().endNs, 19'999 * kDt);
    EXPECT_DOUBLE_EQ(session.view().seconds(), 1.0);

    session.setPaused(true);
    push(*samples, 20'000, std::vector<double>(5'000, 1.0));
    session.update();
    EXPECT_EQ(session.view().endNs, 19'999 * kDt);  // frozen
    session.setPaused(false);
    EXPECT_EQ(session.view().endNs, 24'999 * kDt);

    session.window = 3;  // the whole run held
    EXPECT_EQ(session.view().beginNs, 0);
}

TEST(ScopeSession, CursorsStartInsideTheViewAndMapToAbsoluteTime) {
    ScopeSession session;
    auto samples = buffer();
    session.adopt("a", "A", samples);
    push(*samples, 0, std::vector<double>(30'000, 1.0));
    session.update();  // 10 s window by default
    session.showCursors(true);
    EXPECT_LT(session.cursorA, session.cursorB);
    EXPECT_GT(session.cursorA, -10.0);
    EXPECT_LT(session.cursorB, 0.0);
    session.cursorB = -1.0;
    EXPECT_EQ(session.cursorTimeNs(session.cursorB), session.view().endNs - 1'000'000'000);
}

// Design acceptance (05-signals): a falling trigger pauses the scope exactly at the crossing.
TEST(ScopeSession, AFallingTriggerPausesExactlyAtTheCrossing) {
    ScopeSession session;
    auto samples = buffer();
    session.adopt("iq", "A", samples);
    push(*samples, 0, {0.0, -9.0, 0.0});  // a crossing before the trigger is armed: ignored
    session.update();
    session.arm(0, /*rising=*/false, -8.0);
    EXPECT_TRUE(session.trigger().armed);

    push(*samples, 3, {1.0, -2.0, -7.9});
    session.update();
    EXPECT_FALSE(session.paused());

    push(*samples, 6, {-8.5, -10.0, 3.0, -9.0});
    session.update();
    EXPECT_TRUE(session.paused());
    EXPECT_FALSE(session.trigger().armed);  // single shot
    EXPECT_EQ(session.trigger().firedNs, 6 * kDt);
    EXPECT_EQ(session.view().endNs, 6 * kDt);

    push(*samples, 10, {0.0, -9.0});
    session.update();
    EXPECT_EQ(session.trigger().firedNs, 6 * kDt);  // it does not fire again
}

TEST(ScopeSession, ARisingTriggerNeedsARisingEdge) {
    ScopeSession session;
    auto samples = buffer();
    session.adopt("v", "V", samples);
    push(*samples, 0, {5.0});
    session.update();
    session.arm(0, /*rising=*/true, 2.0);
    push(*samples, 1, {6.0, 1.0, 1.5});  // above the level all along, then below: no rise yet
    session.update();
    EXPECT_FALSE(session.paused());
    push(*samples, 4, {2.0});
    session.update();
    EXPECT_TRUE(session.paused());
    EXPECT_EQ(session.trigger().firedNs, 4 * kDt);
}

TEST(ScopeSession, AGapIsNotACrossing) {
    ScopeSession session;
    auto samples = buffer();
    session.adopt("iq", "A", samples);
    push(*samples, 0, {5.0});
    session.update();
    session.arm(0, false, 0.0);
    push(*samples, 50, {-5.0, -6.0});  // samples 1..49 were lost
    session.update();
    EXPECT_FALSE(session.paused());
    push(*samples, 52, {4.0, -1.0});
    session.update();
    EXPECT_TRUE(session.paused());
    EXPECT_EQ(session.trigger().firedNs, 53 * kDt);
}

TEST(ScopeSession, ArmingAnUnknownTraceDoesNothing) {
    ScopeSession session;
    session.arm(0, false, 1.0);
    EXPECT_FALSE(session.trigger().armed);
    session.update();
}

// Design acceptance (05-signals): the export holds the raw samples of the visible window.
TEST(ScopeSession, CsvExportHoldsTheRawSamplesOfTheView) {
    ScopeSession session;
    auto fast = buffer();
    auto slow = buffer(10 * kDt);
    session.adopt("motor/iq", "A", fast);
    session.adopt("bus/voltage", "V", slow);
    std::vector<double> ramp(100);
    for (std::size_t i = 0; i < ramp.size(); ++i) {
        ramp[i] = static_cast<double>(i) * 0.5;
    }
    push(*fast, 0, ramp);
    push(*slow, 0, {48.0, 47.5, 47.0});

    std::ostringstream out;
    const std::size_t rows = writeCsv(out, session, {10 * kDt, 20 * kDt});
    EXPECT_EQ(rows, 11U);  // ten 10 kHz samples and one 1 kHz sample
    const std::string text = out.str();
    EXPECT_EQ(text.substr(0, text.find('\n')), "signal,unit,time_s,value");
    EXPECT_NE(text.find("motor/iq,A,0.001,5\n"), std::string::npos);
    EXPECT_NE(text.find("motor/iq,A,0.0019,9.5\n"), std::string::npos);
    EXPECT_EQ(text.find("motor/iq,A,0.002,"), std::string::npos);  // the end is exclusive
    EXPECT_NE(text.find("bus/voltage,V,0.001,47.5\n"), std::string::npos);
}

}  // namespace
