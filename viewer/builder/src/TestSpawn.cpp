// Where Test starts the robot. See TestSpawn.hpp.
#include "lagrange/viewer/builder/TestSpawn.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <sstream>

namespace lagrange::viewer::builder {

namespace {

/// A number as typed: "%g" without a trailing ".0" ("26", "-1.75", "0.3").
std::string number(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.6g", value == 0.0 ? 0.0 : value);  // no "-0"
    return text;
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
    return number(pose.x) + "," + number(pose.y) + "," + number(pose.z) + "," + number(yawRad);
}

std::string spawnLabel(const SpawnPose& pose) {
    return number(pose.x) + ", " + number(pose.y) + ", " + number(pose.z) + " · " +
           number(pose.yawDeg) + "°";
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
            out << number(pose.x) << '\t' << number(pose.y) << '\t' << number(pose.z) << '\t'
                << number(pose.yawDeg) << '\t' << key << '\n';
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
