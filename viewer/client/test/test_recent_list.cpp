#include "lagrange/viewer/RecentList.hpp"

#include <sstream>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::RecentEntry;
using lagrange::viewer::RecentList;

RecentEntry entry(const std::string& stage, const std::string& endpoint = "unix:/run/a.sock") {
    return {stage, "amr", "applied", endpoint, "2026-10-14 23:12"};
}

TEST(RecentList, NewestFirstAndOneEntryPerStageAndServer) {
    RecentList list;
    list.add(entry("a.usda"));
    list.add(entry("b.usda"));
    list.add(entry("a.usda"));  // opened again: moves to the front, not listed twice
    ASSERT_EQ(list.entries().size(), 2U);
    EXPECT_EQ(list.entries()[0].stage, "a.usda");
    EXPECT_EQ(list.entries()[1].stage, "b.usda");
    list.add(entry("a.usda", "tcp://box:7447"));  // the same stage from another server
    EXPECT_EQ(list.entries().size(), 3U);
}

TEST(RecentList, KeepsOnlyTheMostRecent) {
    RecentList list;
    for (std::size_t i = 0; i < RecentList::kCapacity + 5; ++i) {
        list.add(entry("stage" + std::to_string(i)));
    }
    ASSERT_EQ(list.entries().size(), RecentList::kCapacity);
    EXPECT_EQ(list.entries().front().stage, "stage" + std::to_string(RecentList::kCapacity + 4));
}

TEST(RecentList, WhatIsWrittenReadsBackTheSame) {
    RecentList list;
    list.add(entry("worlds/warehouse_a.usda"));
    RecentEntry odd = entry("C:\\odd \"name\"\n{x}.usda", "unix:/tmp/a b.sock");
    odd.robot = "左車輪";
    list.add(odd);
    std::stringstream stream;
    list.write(stream);

    RecentList other;
    other.read(stream);
    EXPECT_EQ(other.entries(), list.entries());
}

TEST(RecentList, ADamagedFileGivesWhatCouldBeReadAndNeverFails) {
    RecentList list;
    std::stringstream damaged(
        "[\n  {\"stage\": \"good.usda\", \"robot\": \"amr\"},\n  {\"stage\": 12, \"x\": [}\n"
        "  {\"robot\": \"no stage\"},\n  {\"stage\": \"cut off");
    list.read(damaged);
    ASSERT_EQ(list.entries().size(), 1U);
    EXPECT_EQ(list.entries()[0].stage, "good.usda");
    EXPECT_EQ(list.entries()[0].robot, "amr");

    std::stringstream nothing("not json at all");
    list.read(nothing);
    EXPECT_TRUE(list.entries().empty());
    EXPECT_FALSE(list.load("/nonexistent/dir/recent.json"));
}

}  // namespace
