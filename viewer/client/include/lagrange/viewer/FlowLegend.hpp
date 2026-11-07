#pragma once

// The colour scales of a wind tunnel's flow world (lagrange tunnel). Its meshes are bands of one
// colour each, named after what they span, so the scale travels with the scene and the protocol
// need not know of it: b19_P11p40_to_P12p00_mps is 11.40 to 12.00 m/s.

#include "lagrange/viewer/SceneStore.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::viewer {

struct FlowLegend {
    std::string title;  ///< "speed · m/s"
    double low{0.0};
    double high{0.0};
    std::vector<std::array<float, 4>> colours;  ///< Low to high, as the materials have them.
};

/// One per quantity in the scene; empty for any other scene.
[[nodiscard]] std::vector<FlowLegend> flowLegends(const SceneStatics& statics);

}  // namespace lagrange::viewer
