// A private temporary directory for each test process, added to a test target by
// lagrange_test_private_tmpdir() (LagrangeCompileOptions.cmake).
//
// Tests write scratch files under ::testing::TempDir() or std::filesystem::temp_directory_path()
// with fixed names ("lagrange_determinism", "lagrange_replay", ...). Two runs of the same test
// on one machine (two worktrees, two CI jobs, two agents) then write the same files and break
// each other. Before main() this makes a fresh directory under the inherited $TMPDIR (or /tmp)
// and points TMPDIR and TEST_TMPDIR at it, which both lookups read, as do servers and other
// programs the test starts. The directory is removed when the process that made it exits.
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <unistd.h>

namespace {

class PrivateTmpDir {
public:
    // getenv and setenv are not thread safe (concurrency-mt-unsafe); these run before main()
    // and after it returns, when the test process has no other threads.
    PrivateTmpDir() {
        const char* parent = std::getenv("TMPDIR");  // NOLINT(concurrency-mt-unsafe)
        std::string pattern = parent != nullptr ? std::string(parent) : std::string();
        if (pattern.empty()) {
            pattern = "/tmp";
        }
        pattern += "/lagrange-test-XXXXXX";
        if (::mkdtemp(pattern.data()) == nullptr) {
            return;  // Leave the environment as it was: the shared directory still works.
        }
        path_ = pattern;
        owner_ = ::getpid();
        ::setenv("TMPDIR", path_.c_str(), 1);       // NOLINT(concurrency-mt-unsafe)
        ::setenv("TEST_TMPDIR", path_.c_str(), 1);  // NOLINT(concurrency-mt-unsafe)
    }
    ~PrivateTmpDir() {
        // A forked child that exits without exec must not remove its parent's directory.
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        const bool keep = std::getenv("LAGRANGE_KEEP_TEST_TMPDIR") != nullptr;
        if (path_.empty() || ::getpid() != owner_ || keep) {
            return;
        }
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    PrivateTmpDir(const PrivateTmpDir&) = delete;
    PrivateTmpDir& operator=(const PrivateTmpDir&) = delete;
    PrivateTmpDir(PrivateTmpDir&&) = delete;
    PrivateTmpDir& operator=(PrivateTmpDir&&) = delete;

private:
    std::string path_;
    pid_t owner_ = 0;
};

const PrivateTmpDir kPrivateTmpDir;

}  // namespace
