#pragma once

#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <cstdint>
#include <optional>

namespace lagrange::viewer {

/// The visible prim with a mesh that `ray` hits first, or kNone. Each mesh is tested by its
/// bounding box, turned and placed like the prim, so this is exact for boxes and close for
/// everything else.
[[nodiscard]] std::uint32_t pickPrim(const SceneFrame& frame, const Ray& ray);

/// Where `ray` meets the horizontal plane z = `height`, if it does in front of its origin.
[[nodiscard]] std::optional<std::array<double, 3>> intersectGround(const Ray& ray,
                                                                   double height = 0.0);

}  // namespace lagrange::viewer
