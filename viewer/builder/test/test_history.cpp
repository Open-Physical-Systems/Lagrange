// Undo and redo by snapshots (RFC 0016 §9, plan WP-A): merge, depth, labels, saved state.
#include "lagrange/viewer/builder/History.hpp"

#include <string>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

Document named(const std::string& robot) {
    return newDocument(robot);
}

TEST(History, UndoRedoAndLabels) {
    History history(named("a"), "New robot");
    EXPECT_FALSE(history.canUndo());
    EXPECT_FALSE(history.undo());
    history.commit(named("b"), "Rename robot a → b", "", 0.0);
    history.commit(named("c"), "Rename robot b → c", "", 10.0);
    ASSERT_EQ(history.labels().size(), 3U);
    EXPECT_EQ(history.labels()[0], "New robot");
    EXPECT_EQ(history.labels()[2], "Rename robot b → c");  // newest last
    EXPECT_EQ(history.position(), 2U);
    const auto revision = history.revision();
    EXPECT_TRUE(history.undo());
    EXPECT_NE(history.revision(), revision);
    EXPECT_EQ(history.current().robot, "b");
    EXPECT_TRUE(history.undo());
    EXPECT_EQ(history.current().robot, "a");
    EXPECT_FALSE(history.undo());
    EXPECT_TRUE(history.redo());
    EXPECT_TRUE(history.redo());
    EXPECT_FALSE(history.redo());
    EXPECT_EQ(history.current().robot, "c");
    // The labels stay while undoing: the History tab shows where it is.
    history.undo();
    EXPECT_EQ(history.labels().size(), 3U);
    EXPECT_TRUE(history.canRedo());
}

TEST(History, MergeWithinOneSecond) {
    History history(named("r"), "Start");
    Document document = named("r");
    document.links[0].shapes[0].size[0] = 0.4;
    history.commit(document, "Set size x 0.3 → 0.4 m", "link/base_link/shape/0/size", 1.0);
    document.links[0].shapes[0].size[0] = 0.45;
    history.commit(document, "Set size x 0.3 → 0.45 m", "link/base_link/shape/0/size", 1.5);
    document.links[0].shapes[0].size[0] = 0.5;
    history.commit(document, "Set size x 0.3 → 0.5 m", "link/base_link/shape/0/size", 2.4);
    // Typing in one field is one step: each edit within 1 s of the last merged.
    ASSERT_EQ(history.labels().size(), 2U);
    EXPECT_EQ(history.labels()[1], "Set size x 0.3 → 0.5 m");
    EXPECT_EQ(history.current().links[0].shapes[0].size[0], 0.5);
    // One second or more later: a new step.
    document.links[0].shapes[0].size[0] = 0.6;
    history.commit(document, "Set size x 0.5 → 0.6 m", "link/base_link/shape/0/size", 3.5);
    EXPECT_EQ(history.labels().size(), 3U);
    // Another field: a new step even at once.
    document.links[0].shapes[0].size[1] = 0.3;
    history.commit(document, "Set size y", "link/base_link/shape/0/size_y", 3.6);
    EXPECT_EQ(history.labels().size(), 4U);
    // No key: never merges.
    history.commit(document, "Add link", "", 3.7);
    history.commit(document, "Add link", "", 3.7);
    EXPECT_EQ(history.labels().size(), 6U);
    // After an undo the next edit is its own step, never merged into the state undone to.
    history.commit(document, "Set mass", "link/base_link/mass", 4.0);
    EXPECT_TRUE(history.undo());
    history.commit(document, "Set mass", "link/base_link/mass", 4.1);
    EXPECT_EQ(history.labels().size(), 7U);
    EXPECT_TRUE(history.undo());
    EXPECT_EQ(history.labels()[history.position()], "Add link");
}

TEST(History, DepthDropsOldest) {
    History history(named("s0"), "state 0", 4);
    for (int i = 1; i <= 6; ++i) {
        history.commit(named("s" + std::to_string(i)), "state " + std::to_string(i), "",
                       static_cast<double>(i));
    }
    ASSERT_EQ(history.labels().size(), 4U);
    EXPECT_EQ(history.labels().front(), "state 3");
    EXPECT_EQ(history.labels().back(), "state 6");
    EXPECT_EQ(history.position(), 3U);
    int undos = 0;
    while (history.undo()) {
        ++undos;
    }
    EXPECT_EQ(undos, 3);
    EXPECT_EQ(history.current().robot, "s3");
    // A depth of zero still keeps the current state.
    History tiny(named("x"), "x", 0);
    tiny.commit(named("y"), "y", "", 0.0);
    EXPECT_EQ(tiny.labels().size(), 1U);
    EXPECT_EQ(tiny.current().robot, "y");
    EXPECT_FALSE(tiny.undo());
}

TEST(History, RedoClearedByCommit) {
    History history(named("a"), "a");
    history.commit(named("b"), "b", "", 0.0);
    history.commit(named("c"), "c", "", 5.0);
    history.undo();
    history.undo();
    EXPECT_TRUE(history.canRedo());
    history.commit(named("d"), "d", "", 10.0);
    EXPECT_FALSE(history.canRedo());
    EXPECT_FALSE(history.redo());
    ASSERT_EQ(history.labels().size(), 2U);
    EXPECT_EQ(history.labels()[1], "d");
    EXPECT_EQ(history.current().robot, "d");
}

TEST(History, GoTo) {
    History history(named("a"), "a");
    history.commit(named("b"), "b", "", 0.0);
    history.commit(named("c"), "c", "", 5.0);
    EXPECT_TRUE(history.goTo(0));
    EXPECT_EQ(history.current().robot, "a");
    EXPECT_EQ(history.position(), 0U);
    const auto revision = history.revision();
    EXPECT_TRUE(history.goTo(0));  // where it is: no change
    EXPECT_EQ(history.revision(), revision);
    EXPECT_TRUE(history.goTo(2));
    EXPECT_EQ(history.current().robot, "c");
    EXPECT_FALSE(history.goTo(3));
    EXPECT_EQ(history.current().robot, "c");
}

TEST(History, EditsSinceSave) {
    History history(named("a"), "Open rover.urdf");
    EXPECT_EQ(history.editsSinceSave(), 0);
    history.commit(named("b"), "b", "", 0.0);
    history.commit(named("c"), "c", "", 5.0);
    EXPECT_EQ(history.editsSinceSave(), 2);  // never saved: edits since state 0
    history.markSaved();
    EXPECT_EQ(history.editsSinceSave(), 0);
    history.undo();
    EXPECT_EQ(history.editsSinceSave(), 1);  // undo away from the saved state is unsaved
    history.redo();
    EXPECT_EQ(history.editsSinceSave(), 0);  // back to what is on disk
    // An edit right after saving never merges into the saved state.
    history.commit(named("c2"), "Set x", "field", 6.0);
    history.markSaved();
    history.commit(named("c3"), "Set x", "field", 6.2);
    EXPECT_EQ(history.editsSinceSave(), 1);
    history.undo();
    EXPECT_EQ(history.current().robot, "c2");
    EXPECT_EQ(history.editsSinceSave(), 0);
    // Commit after undoing past the saved state drops it: unsaved until saved again.
    history.undo();
    history.commit(named("e"), "e", "", 20.0);
    EXPECT_GT(history.editsSinceSave(), 0);
    history.markSaved();
    EXPECT_EQ(history.editsSinceSave(), 0);
}

TEST(History, MovesKeepTheStack) {
    History history(named("a"), "a");
    history.commit(named("b"), "b", "", 0.0);
    History moved(std::move(history));
    EXPECT_EQ(moved.current().robot, "b");
    EXPECT_TRUE(moved.undo());
    EXPECT_EQ(moved.current().robot, "a");
}

}  // namespace
}  // namespace lagrange::viewer::builder
