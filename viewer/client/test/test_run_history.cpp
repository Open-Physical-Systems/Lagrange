#include "lagrange/viewer/RunHistory.hpp"

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

LogEntry line(const char* source, const char* text, std::int64_t ns = 0) {
    return {ns, LogLevel::Info, source, text};
}

void add(LogBook& book, LogEntry entry) {
    book.entries.push_back(std::move(entry));
    ++book.total;
}

TEST(RunHistory, ReadsAnAttributeEditAsTheServerLogsIt) {
    const auto item = historyItemOf(line(
        "edit", "/amr/left_drive_motor elec:windingResistance 0.12 -> 0.15 ohm (from t = 3.413 s)",
        3'413'000'000));
    ASSERT_TRUE(item);
    EXPECT_EQ(item->kind, HistoryItem::Kind::Attribute);
    EXPECT_EQ(item->target, "/amr/left_drive_motor");
    EXPECT_EQ(item->name, "elec:windingResistance");
    EXPECT_EQ(item->before, "0.12");
    EXPECT_EQ(item->after, "0.15");
    EXPECT_EQ(item->unit, "ohm");
    EXPECT_EQ(item->simTimeNs, 3'413'000'000);
}

TEST(RunHistory, AnEditWithoutAUnitHasNone) {
    const auto item =
        historyItemOf(line("edit", "/amr/base_link physics:mass 12 -> 14  (from t = 1.000 s)"));
    ASSERT_TRUE(item);
    EXPECT_EQ(item->after, "14");
    EXPECT_TRUE(item->unit.empty());
}

TEST(RunHistory, ReadsSpawnsDeletesProfilesSitesAndResets) {
    auto spawn = historyItemOf(
        line("stage", "spawned crate_2 from /tmp/crate.urdf (passive: nothing drives it)"));
    ASSERT_TRUE(spawn);
    EXPECT_EQ(spawn->kind, HistoryItem::Kind::Spawn);
    EXPECT_EQ(spawn->target, "crate_2");
    EXPECT_EQ(spawn->detail, "/tmp/crate.urdf");
    EXPECT_EQ(spawn->after, "passive: nothing drives it");

    auto gone = historyItemOf(line("stage", "deleted crate_2"));
    ASSERT_TRUE(gone);
    EXPECT_EQ(gone->kind, HistoryItem::Kind::Delete);
    EXPECT_EQ(gone->target, "crate_2");

    auto profile = historyItemOf(line("profile", "applied; the run restarted at t = 0"));
    ASSERT_TRUE(profile);
    EXPECT_EQ(profile->kind, HistoryItem::Kind::Profile);

    auto site = historyItemOf(line("site", "gravity = 1.62"));
    ASSERT_TRUE(site);
    EXPECT_EQ(site->kind, HistoryItem::Kind::Site);
    EXPECT_EQ(site->name, "gravity");
    EXPECT_EQ(site->after, "1.62");

    auto together = historyItemOf(line("site", "set together: wind_speed, wind_from"));
    ASSERT_TRUE(together);
    EXPECT_EQ(together->name, "wind_speed, wind_from");

    auto reset = historyItemOf(line("scheduler", "reset · the run restarted at t = 0"));
    ASSERT_TRUE(reset);
    EXPECT_EQ(reset->kind, HistoryItem::Kind::Reset);
}

TEST(RunHistory, OtherLinesAreNoChanges) {
    EXPECT_FALSE(historyItemOf(line("stage", "loaded /a/amr.usda · 5 bodies, 4 joints, 40 prims")));
    EXPECT_FALSE(
        historyItemOf(line("site", "wind_speed rebuilds the run, velocities from rest: x")));
    EXPECT_FALSE(historyItemOf(line("site", "the wind is set but the aerodynamics domain ...")));
    EXPECT_FALSE(historyItemOf(line("sensors", "lidar front_lidar on lidar_link at 10 Hz")));
    EXPECT_FALSE(historyItemOf(line("edit", "something else")));
}

TEST(RunHistory, KeepsItsOwnListPastTheLogBooksLength) {
    RunHistory history;
    LogBook book;
    add(book, line("edit", "/a x 1 -> 2 m (from t = 0.000 s)"));
    EXPECT_TRUE(history.update(book, 1));
    EXPECT_FALSE(history.update(book, 1));  // nothing new
    // The book drops its oldest line; the history keeps it.
    book.entries.erase(book.entries.begin());
    add(book, line("edit", "/a x 2 -> 3 m (from t = 1.000 s)"));
    EXPECT_TRUE(history.update(book, 1));
    ASSERT_EQ(history.items().size(), 2U);
    EXPECT_EQ(history.items()[1].after, "3");
    // A new connection reads its book again from the start.
    LogBook fresh;
    add(fresh, line("edit", "/a x 2 -> 3 m (from t = 1.000 s)"));
    EXPECT_TRUE(history.update(fresh, 2));
    EXPECT_EQ(history.items().size(), 1U);
}

TEST(RunHistory, LayersFollowTheRobotsAndTheSessionLayer) {
    RunHistory history;
    LogBook book;
    add(book, line("stage", "robot b from /w/b.urdf (passive: nothing drives it)"));
    add(book, line("stage", "spawned c from /w/c.urdf (the robot: ROS drives it)"));
    history.update(book, 1);
    auto rows = layerRows("/w/amr.usda", "", history);
    ASSERT_EQ(rows.size(), 4U);  // session, c, b, stage: strongest first
    EXPECT_EQ(rows[0].role, LayerRow::Role::Session);
    EXPECT_TRUE(rows[0].path.empty());
    EXPECT_EQ(rows[1].label, "c.urdf");
    EXPECT_EQ(rows[2].label, "b.urdf");
    EXPECT_EQ(rows[3].role, LayerRow::Role::Stage);
    EXPECT_EQ(rows[3].label, "amr.usda");

    add(book, line("stage", "deleted c"));
    add(book, line("edit", "session layer saved to /s/amr.session.usda"));
    history.update(book, 1);
    rows = layerRows("/w/amr.usda", "/w/rain.usda", history);
    ASSERT_EQ(rows.size(), 4U);  // session, b, stage, environment
    EXPECT_EQ(rows[0].path, "/s/amr.session.usda");
    EXPECT_EQ(rows[1].label, "b.urdf");
    EXPECT_EQ(rows[3].role, LayerRow::Role::Environment);
    EXPECT_EQ(history.sessionLayer(), "/s/amr.session.usda");
}

}  // namespace
}  // namespace lagrange::viewer
