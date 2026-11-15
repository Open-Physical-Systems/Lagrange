// The builder's own files outside the document. See Files.hpp.
#include "lagrange/viewer/builder/Files.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

namespace lagrange::viewer::builder {

namespace {

/// Closes a file descriptor at the end of a scope.
struct Descriptor {
    int fd{-1};
    explicit Descriptor(int value) : fd(value) {}
    ~Descriptor() {
        if (fd >= 0) {
            ::close(fd);
        }
    }
    Descriptor(const Descriptor&) = delete;
    Descriptor& operator=(const Descriptor&) = delete;
};

bool allDigits(std::string_view text) {
    return !text.empty() && std::ranges::all_of(text, [](char c) { return c >= '0' && c <= '9'; });
}

}  // namespace

std::string makePrivateDirectory(const std::filesystem::path& path, bool exclusive) {
    if (::mkdir(path.c_str(), 0700) != 0 && errno != EEXIST) {
        return "cannot make " + path.string() + ": " + std::strerror(errno);
    }
    // Opened without following a link, so what is checked is what is changed (no race between
    // a check by name and a chmod by name).
    const Descriptor folder(::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
    if (folder.fd < 0) {
        return path.string() + " is not a folder of its own (" + std::strerror(errno) +
               "; a symlink is refused)";
    }
    struct stat info{};
    if (::fstat(folder.fd, &info) != 0 || !S_ISDIR(info.st_mode)) {
        return path.string() + " is not a folder";
    }
    if (info.st_uid != ::geteuid()) {
        return path.string() + " belongs to another user (uid " + std::to_string(info.st_uid) +
               "); refused";
    }
    const mode_t others = exclusive ? (S_IRWXG | S_IRWXO) : (S_IWGRP | S_IWOTH);
    if ((info.st_mode & others) != 0 && ::fchmod(folder.fd, info.st_mode & 07777 & ~others) != 0) {
        return "cannot make " + path.string() + " private: " + std::strerror(errno);
    }
    return {};
}

std::string fileKey(std::string_view text) {
    constexpr std::uint64_t kOffset = 14695981039346656037ULL;  // FNV-1a 64-bit offset basis
    constexpr std::uint64_t kPrime = 1099511628211ULL;          // FNV-1a 64-bit prime
    std::uint64_t hash = kOffset;
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= kPrime;
    }
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 15; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = kHex[hash & 0xFU];
        hash >>= 4U;
    }
    return out;
}

std::string distinctFileName(std::string_view name) {
    std::string out = safeFileName(name);
    if (out != name) {
        out += "-" + fileKey(name).substr(0, 8);
    }
    return out;
}

namespace {

std::string autosavePrefix(std::string_view robot, const std::filesystem::path& document) {
    std::string key = "new";
    if (!document.empty()) {
        std::error_code ec;
        key = fileKey(std::filesystem::absolute(document, ec).lexically_normal().string());
    }
    return distinctFileName(robot) + "-" + key + ".";
}

constexpr std::string_view kAutosaveSuffix = ".autosave.urdf";

}  // namespace

std::filesystem::path autosaveFolder() {
    const char* state = std::getenv("XDG_STATE_HOME");  // NOLINT(concurrency-mt-unsafe)
    const char* home = std::getenv("HOME");             // NOLINT(concurrency-mt-unsafe)
    const std::filesystem::path base =
        state != nullptr && *state != '\0'
            ? std::filesystem::path(state)
            : std::filesystem::path(home != nullptr ? home : "") / ".local/state";
    return base / "lagrange" / "builder";
}

std::filesystem::path autosaveFile(const std::filesystem::path& folder, std::string_view robot,
                                   const std::filesystem::path& document, int pid) {
    return folder /
           (autosavePrefix(robot, document) + std::to_string(pid) + std::string(kAutosaveSuffix));
}

LeftAutosaves findLeftAutosaves(const std::filesystem::path& folder, std::string_view robot,
                                const std::filesystem::path& document, int pid,
                                const std::function<bool(int)>& alive) {
    LeftAutosaves out;
    const std::string prefix = autosavePrefix(robot, document);
    std::error_code ec;
    std::filesystem::file_time_type documentTime{};
    const bool hasFile = !document.empty() && std::filesystem::is_regular_file(document, ec);
    if (hasFile) {
        documentTime = std::filesystem::last_write_time(document, ec);
    }
    std::vector<std::pair<std::filesystem::file_time_type, std::filesystem::path>> found;
    std::filesystem::directory_iterator it(folder, ec);
    for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (!name.starts_with(prefix) || !name.ends_with(kAutosaveSuffix)) {
            continue;
        }
        const std::string_view middle = std::string_view(name).substr(
            prefix.size(), name.size() - prefix.size() - kAutosaveSuffix.size());
        if (!allDigits(middle) || middle.size() > 9) {
            continue;
        }
        const int owner = std::stoi(std::string(middle));
        std::error_code itemError;
        if (owner == pid || !std::filesystem::is_regular_file(it->symlink_status(itemError)) ||
            (alive && alive(owner))) {
            continue;  // this viewer's, a link, or another viewer's that still runs
        }
        const auto time = std::filesystem::last_write_time(it->path(), itemError);
        if (itemError) {
            continue;
        }
        if (hasFile && time <= documentTime) {
            out.stale.push_back(it->path());
        } else {
            found.emplace_back(time, it->path());
        }
    }
    std::ranges::sort(found, [](const auto& a, const auto& b) { return a.first > b.first; });
    for (auto& entry : found) {
        out.recoverable.push_back(std::move(entry.second));
    }
    return out;
}

bool processAlive(int pid) {
    return pid > 0 && (::kill(pid, 0) == 0 || errno == EPERM);
}

SaveAsTarget saveAsTarget(const std::filesystem::path& chosen) {
    SaveAsTarget out{chosen, false};
    if (chosen.extension() != ".urdf") {
        out.path += ".urdf";
        std::error_code ec;
        out.confirm = std::filesystem::exists(std::filesystem::symlink_status(out.path, ec));
    }
    return out;
}

}  // namespace lagrange::viewer::builder
