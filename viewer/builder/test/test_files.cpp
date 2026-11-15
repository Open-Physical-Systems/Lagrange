// The builder's own files (code review of 2026-11-15): the private runtime folder (L2), the
// autosaves keyed by file, robot and viewer (L6), and Save as…'s added ".urdf" (L7).
#include "lagrange/viewer/builder/Files.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <gtest/gtest.h>

namespace builder = lagrange::viewer::builder;

namespace {

class Scratch {
public:
    Scratch() {
        std::string pattern =
            (std::filesystem::temp_directory_path() / "lgr-files-XXXXXX").string();
        path_ = ::mkdtemp(pattern.data());
    }
    ~Scratch() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
    [[nodiscard]] std::filesystem::path operator/(const std::string& name) const {
        return path_ / name;
    }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

mode_t modeOf(const std::filesystem::path& path) {
    struct stat info{};
    EXPECT_EQ(::lstat(path.c_str(), &info), 0);
    return info.st_mode & 07777;
}

void touch(const std::filesystem::path& path, std::filesystem::file_time_type time) {
    std::ofstream(path) << "<robot name=\"r\"/>\n";
    std::filesystem::last_write_time(path, time);
}

}  // namespace

// --- L2: the runtime folder ----------------------------------------------------------------

TEST(PrivateDirectory, IsMade0700AndTightenedWhenItExists) {
    const Scratch scratch;
    const auto made = scratch / "made";
    EXPECT_EQ(builder::makePrivateDirectory(made, true), "");
    EXPECT_EQ(modeOf(made), 0700U);
    // One of the user's own left open to others: made private, not refused.
    const auto open = scratch / "open";
    std::filesystem::create_directory(open);
    ::chmod(open.c_str(), 0777);
    EXPECT_EQ(builder::makePrivateDirectory(open, true), "");
    EXPECT_EQ(modeOf(open), 0700U);
    // A shared parent (not exclusive) only loses others' write.
    const auto shared = scratch / "shared";
    std::filesystem::create_directory(shared);
    ::chmod(shared.c_str(), 0777);
    EXPECT_EQ(builder::makePrivateDirectory(shared, false), "");
    EXPECT_EQ(modeOf(shared), 0755U);
}

TEST(PrivateDirectory, RefusesASymlinkAFileAndAMissingParent) {
    const Scratch scratch;
    const auto target = scratch / "elsewhere";
    std::filesystem::create_directory(target);
    const auto link = scratch / "lagrange-1000";
    std::filesystem::create_directory_symlink(target, link);
    const std::string refused = builder::makePrivateDirectory(link, true);
    EXPECT_NE(refused.find("symlink"), std::string::npos) << refused;
    std::ofstream(scratch / "file") << "x";
    EXPECT_NE(builder::makePrivateDirectory(scratch / "file", true), "");
    EXPECT_NE(builder::makePrivateDirectory(scratch / "no" / "such", true), "");
}

TEST(PrivateDirectory, RefusesAnotherUsersFolder) {
    // /tmp itself is root's (or at least not a test user's own folder): a stand-in for a
    // /tmp/lagrange-<uid> made first by someone else.
    if (::geteuid() == 0) {
        GTEST_SKIP() << "root owns everything";
    }
    const std::string refused = builder::makePrivateDirectory("/tmp", true);
    EXPECT_NE(refused.find("another user"), std::string::npos) << refused;
}

// --- L3: names in file paths -----------------------------------------------------------------

TEST(DistinctFileName, IsSafeAndKeepsNamesApart) {
    EXPECT_EQ(builder::distinctFileName("arm"), "arm");
    EXPECT_EQ(builder::distinctFileName("arm.1"), "arm.1");
    for (const char* hostile : {"/etc/passwd", "..", "../x", "a/b", "arm(", ".hidden", ""}) {
        const std::string name = builder::distinctFileName(hostile);
        EXPECT_EQ(name.find('/'), std::string::npos) << hostile;
        EXPECT_NE(name.front(), '.') << hostile;
        EXPECT_FALSE(name.empty());
    }
    EXPECT_NE(builder::distinctFileName("a/b"), builder::distinctFileName("a_b"));
    EXPECT_NE(builder::distinctFileName("a/b"), builder::distinctFileName("a(b"));
    EXPECT_EQ(builder::fileKey("x").size(), 16U);
}

// --- L6: autosaves -----------------------------------------------------------------------------

TEST(Autosave, IsKeyedByFileRobotAndViewer) {
    const std::filesystem::path folder = "/state/lagrange/builder";
    const auto a = builder::autosaveFile(folder, "rover", "/robots/a/rover.urdf", 100);
    const auto b = builder::autosaveFile(folder, "rover", "/robots/b/rover.urdf", 100);
    const auto other = builder::autosaveFile(folder, "rover", "/robots/a/rover.urdf", 200);
    const auto fresh = builder::autosaveFile(folder, "rover", {}, 100);
    EXPECT_NE(a, b);      // two files of one robot name
    EXPECT_NE(a, other);  // two viewers
    EXPECT_EQ(fresh.filename(), "rover-new.100.autosave.urdf");
    EXPECT_EQ(a.parent_path(), folder);
    EXPECT_TRUE(a.filename().string().starts_with("rover-"));
    EXPECT_TRUE(a.filename().string().ends_with(".100.autosave.urdf"));
    // A crafted robot name stays in the folder.
    EXPECT_EQ(builder::autosaveFile(folder, "../../x", {}, 1).parent_path(), folder);
}

TEST(Autosave, OnlyANewerOneFromAViewerThatIsGoneIsOffered) {
    const Scratch scratch;
    const auto document = scratch / "rover.urdf";
    const auto now = std::filesystem::file_time_type::clock::now();
    touch(document, now - std::chrono::hours(1));
    const auto folder = scratch / "state";
    std::filesystem::create_directory(folder);
    const auto newer = builder::autosaveFile(folder, "rover", document, 11);
    const auto older = builder::autosaveFile(folder, "rover", document, 12);
    const auto running = builder::autosaveFile(folder, "rover", document, 13);
    const auto mine = builder::autosaveFile(folder, "rover", document, 14);
    const auto otherFile = builder::autosaveFile(folder, "rover", scratch / "else.urdf", 15);
    const auto otherRobot = builder::autosaveFile(folder, "crate", document, 16);
    touch(newer, now);
    touch(older, now - std::chrono::hours(2));
    touch(running, now);
    touch(mine, now);
    touch(otherFile, now);
    touch(otherRobot, now);
    const auto alive = [](int pid) { return pid == 13; };
    const builder::LeftAutosaves left =
        builder::findLeftAutosaves(folder, "rover", document, 14, alive);
    ASSERT_EQ(left.recoverable.size(), 1U);
    EXPECT_EQ(left.recoverable.front(), newer);
    ASSERT_EQ(left.stale.size(), 1U);
    EXPECT_EQ(left.stale.front(), older);  // the file was saved after it: removed, not offered

    // A new robot (no file): every one a gone viewer left is newer than nothing.
    const auto fresh = builder::autosaveFile(folder, "rover", {}, 21);
    touch(fresh, now - std::chrono::hours(3));
    const auto none = builder::findLeftAutosaves(folder, "rover", {}, 14, alive);
    ASSERT_EQ(none.recoverable.size(), 1U);
    EXPECT_EQ(none.recoverable.front(), fresh);
    EXPECT_TRUE(none.stale.empty());
}

TEST(Autosave, ThisProcessIsAlive) {
    EXPECT_TRUE(builder::processAlive(static_cast<int>(::getpid())));
    EXPECT_FALSE(builder::processAlive(0));
    EXPECT_FALSE(builder::processAlive(-1));
}

// --- L7: Save as… ------------------------------------------------------------------------------

TEST(SaveAs, AddsUrdfAndAsksWhenThatNameExists) {
    const Scratch scratch;
    // Named with .urdf: the dialog asked about this very name.
    std::ofstream(scratch / "kept.urdf") << "x";
    const auto asked = builder::saveAsTarget(scratch / "kept.urdf");
    EXPECT_EQ(asked.path, scratch / "kept.urdf");
    EXPECT_FALSE(asked.confirm);
    // "foo" typed, foo.urdf there: the dialog never asked about foo.urdf.
    std::ofstream(scratch / "foo.urdf") << "x";
    const auto added = builder::saveAsTarget(scratch / "foo");
    EXPECT_EQ(added.path, scratch / "foo.urdf");
    EXPECT_TRUE(added.confirm);
    // "bar" typed, no bar.urdf: nothing to ask.
    const auto fresh = builder::saveAsTarget(scratch / "bar");
    EXPECT_EQ(fresh.path, scratch / "bar.urdf");
    EXPECT_FALSE(fresh.confirm);
    // A dangling link at the added name counts as there (it would be written through).
    std::filesystem::create_symlink(scratch / "nowhere", scratch / "dangling.urdf");
    EXPECT_TRUE(builder::saveAsTarget(scratch / "dangling").confirm);
}
