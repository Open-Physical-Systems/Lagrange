#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/World.hpp"

#include <memory>
#include <string_view>
#include <vector>

namespace lagrange::sensors {

/// True when Lagrange simulates custom sensors of `kind` itself (sensors.md §9), with no plugin.
[[nodiscard]] bool isBuiltinKind(std::string_view kind);

/// Every built-in kind, sorted.
[[nodiscard]] std::vector<std::string_view> builtinKinds();

/// True when the built-in kind renders the camera scene (event_camera, stereo_camera): a run
/// with such a sensor needs a CameraScene as a run with a camera does.
[[nodiscard]] bool builtinKindRenders(std::string_view kind);

/// The module for custom sensor `sensor` when its kind is built in, else null (a plugin may
/// implement it). `groundTruth`: the sensors domain's ground_truth substitute. `scene`: the
/// model's scene, read only while the module is made (null: kinds that need prims by name fall
/// back to World::prims).
[[nodiscard]] std::unique_ptr<IModule> createBuiltinKind(
    const World& world, std::uint32_t sensor, bool groundTruth,
    std::shared_ptr<gpu::RayContext> rayContext, std::shared_ptr<const CameraScene> cameraScene,
    const Scene* scene = nullptr);

}  // namespace lagrange::sensors
