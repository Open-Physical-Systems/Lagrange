#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/World.hpp"

#include <cmath>
#include <memory>
#include <span>
#include <string>
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

/// The allowed range of a built-in kind's number parameter, as the sensor catalog gives it
/// (`min`/`min_exclusive`, `max`/`max_exclusive`; ±inf where it gives none).
struct KindParamRange {
    std::string_view kind;
    std::string_view path;  // the kind's own: <voltage><noise> is voltage/noise
    double min;
    double max;
    bool minExclusive;
    bool maxExclusive;
    std::string_view unit;  // as the catalog writes it ("°C", "m"); "1": a pure number

    /// Finite values only: ±inf and NaN are never in a range (a parameter has no "unbounded").
    [[nodiscard]] bool contains(double v) const {
        return std::isfinite(v) && (minExclusive ? v > min : v >= min) &&
               (maxExclusive ? v < max : v <= max);
    }
};

/// Every ranged number parameter of the built-in kinds (the catalog's, row for row).
[[nodiscard]] std::span<const KindParamRange> builtinKindParamRanges();

/// Words for the first parameter of sensor `sensor` (a built-in kind) outside its catalog range
/// ("voltage/noise = -1: must be at least 0"), or empty. createBuiltinKind refuses with it.
[[nodiscard]] std::string builtinKindRangeProblem(const World& world, std::uint32_t sensor);

/// The module for custom sensor `sensor` when its kind is built in, else null (a plugin may
/// implement it). `groundTruth`: the sensors domain's ground_truth substitute. `scene`: the
/// model's scene, read only while the module is made (null: kinds that need prims by name fall
/// back to World::prims).
[[nodiscard]] std::unique_ptr<IModule> createBuiltinKind(
    const World& world, std::uint32_t sensor, bool groundTruth,
    std::shared_ptr<gpu::RayContext> rayContext, std::shared_ptr<const CameraScene> cameraScene,
    const Scene* scene = nullptr);

}  // namespace lagrange::sensors
