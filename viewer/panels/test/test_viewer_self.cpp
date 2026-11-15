#include "lagrange/viewer/ViewerSelf.hpp"

#include <algorithm>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

TEST(ViewerSelf, ParsesATaskStatLineWhoseNameHoldsSpacesAndParentheses) {
    // proc(5): pid (comm) state ppid pgrp session tty_nr tpgid flags minflt cminflt majflt
    // cmajflt utime stime ...
    const auto sample = parseTaskStat(
        "4242 (a (b) c) S 1 4242 4242 0 -1 4194304 494 0 0 0 17 25 0 0 20 0 3 0 26129673\n");
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(sample->tid, 4242);
    EXPECT_EQ(sample->name, "a (b) c");
    EXPECT_EQ(sample->state, 'S');
    EXPECT_EQ(sample->cpuTicks, 17U + 25U);
}

TEST(ViewerSelf, RefusesATruncatedStatLine) {
    EXPECT_FALSE(parseTaskStat("12 (x) R 1 2 3").has_value());
    EXPECT_FALSE(parseTaskStat("").has_value());
    EXPECT_FALSE(parseTaskStat("12 x R").has_value());
}

TEST(ViewerSelf, ParsesTheMemoryLinesOfProcStatusInKilobytes) {
    const ProcessMemory memory = parseProcessStatus(
        "Name:\tlagrange_viewer\nVmPeak:\t 9000 kB\nVmSize:\t 1412052 kB\nVmHWM:\t    7864 kB\n"
        "VmRSS:\t    7000 kB\nThreads:\t1\n");
    EXPECT_TRUE(memory.known);
    EXPECT_EQ(memory.residentBytes, 7000ULL * 1024);
    EXPECT_EQ(memory.peakBytes, 7864ULL * 1024);
    EXPECT_EQ(memory.virtualBytes, 1412052ULL * 1024);
    EXPECT_FALSE(parseProcessStatus("Name:\tx\n").known);
}

TEST(ViewerSelf, ReadsItsOwnThreadsAndMemory) {
    const std::vector<ThreadSample> threads = readOwnThreads();
    ASSERT_FALSE(threads.empty());
    const int self = static_cast<int>(getpid());  // the main thread's tid is the pid
    EXPECT_TRUE(std::ranges::any_of(threads, [&](const ThreadSample& t) { return t.tid == self; }));
    const ProcessMemory memory = readOwnMemory();
    EXPECT_TRUE(memory.known);
    EXPECT_GT(memory.residentBytes, 0U);
    EXPECT_GE(memory.peakBytes, memory.residentBytes);
    EXPECT_GT(clockTicksPerSecond(), 0.0);
}

}  // namespace
}  // namespace lagrange::viewer
