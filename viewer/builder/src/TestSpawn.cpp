// Where Test starts the robot. See TestSpawn.hpp.
#include "lagrange/viewer/builder/TestSpawn.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <sstream>
#include <system_error>

namespace lagrange::viewer::builder {

namespace {

/// A number for the eye: "%g" without a trailing ".0" ("26", "-1.75", "0.3"), six digits.
std::string label(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.6g", value == 0.0 ? 0.0 : value);  // no "-0"
    return text;
}

/// A number to read back (the saved poses, --spawn): the shortest fixed-point text that parses
/// to the same double ("26", "-1.75", "12345.67", "100000.25"), never rounded to six digits
/// (review L6). Beyond what fits (no pose is that far): every digit %.17g keeps.
std::string exact(double value) {
    std::array<char, 64> text{};
    const auto [end, error] = std::to_chars(text.data(), text.data() + text.size(),
                                            value == 0.0 ? 0.0 : value, std::chars_format::fixed);
    if (error == std::errc{}) {
        return {text.data(), end};
    }
    std::snprintf(text.data(), text.size(), "%.17g", value);
    return text.data();
}

}  // namespace

std::string spawnKey(const std::string& world) {
    return world.empty() ? std::string("none") : world;
}

SpawnPose defaultSpawn(const std::string& world) {
    if (world.empty() || world == "none") {
        return {};
    }
    // The sample worlds by their folder: assets/worlds/<name>/<file>.
    const std::string folder = std::filesystem::path(world).parent_path().filename().string();
    if (folder == "tiled_town") {
        return {1036.0, 1036.0, 0.0, 0.0};
    }
    if (folder == "amr_fms_warehouse") {
        return {26.0, 8.0, 0.0, 180.0};
    }
    if (folder == "town") {
        return {30.0, -1.75, 0.0, 0.0};
    }
    return {};
}

std::string spawnArgument(const SpawnPose& pose) {
    if (pose.isOrigin()) {
        return {};
    }
    const double yawRad = pose.yawDeg * std::numbers::pi / 180.0;
    return exact(pose.x) + "," + exact(pose.y) + "," + exact(pose.z) + "," + exact(yawRad);
}

std::string spawnLabel(const SpawnPose& pose) {
    return label(pose.x) + ", " + label(pose.y) + ", " + label(pose.z) + " · " +
           label(pose.yawDeg) + "°";
}

void SpawnMemory::load(const std::filesystem::path& file) {
    file_ = file;
    poses_.clear();
    std::ifstream in(file);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream fields(line);
        SpawnPose pose;
        std::string key;
        if (!(fields >> pose.x >> pose.y >> pose.z >> pose.yawDeg)) {
            continue;
        }
        fields.get();  // the tab before the key
        std::getline(fields, key);
        if (key.empty() || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
            !std::isfinite(pose.z) || !std::isfinite(pose.yawDeg)) {
            continue;
        }
        poses_[key] = pose;
    }
}

bool SpawnMemory::save() const {
    if (file_.empty()) {
        return false;
    }
    std::error_code error;
    std::filesystem::create_directories(file_.parent_path(), error);
    const std::filesystem::path partial = file_.string() + ".partial";
    {
        std::ofstream out(partial, std::ios::trunc);
        if (!out) {
            return false;
        }
        out << "# The robot builder's Test start poses: x y z (m) yaw (deg), then the world\n";
        for (const auto& [key, pose] : poses_) {
            out << exact(pose.x) << '\t' << exact(pose.y) << '\t' << exact(pose.z) << '\t'
                << exact(pose.yawDeg) << '\t' << key << '\n';
        }
        if (!out.flush()) {
            return false;
        }
    }
    std::filesystem::rename(partial, file_, error);
    return !error;
}

SpawnPose SpawnMemory::at(const std::string& world) const {
    const std::string key = spawnKey(world);
    const auto found = poses_.find(key);
    return found != poses_.end() ? found->second : defaultSpawn(key);
}

void SpawnMemory::set(const std::string& world, const SpawnPose& pose) {
    poses_[spawnKey(world)] = pose;
    (void)save();
}

void SpawnMemory::reset(const std::string& world) {
    poses_.erase(spawnKey(world));
    (void)save();
}

}  // namespace lagrange::viewer::builder
