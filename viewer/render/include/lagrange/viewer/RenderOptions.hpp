#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace lagrange::viewer {

/// How the viewport colours meshes (design 02-scene: the shading menu).
enum class Shading : std::uint8_t {
    Lit,        ///< Material colour, one directional light and sky ambient.
    Unlit,      ///< Material colour as it is.
    Thermal,    ///< Temperature on the design's thermal colour map; unlit.
    Wireframe,  ///< Triangle edges in the material colour, unlit (when the device can).
    Collision,  ///< The collision shapes the server sends, lit, in place of the visuals.
};

/// What the viewport draws besides the scene's meshes.
struct RenderOptions {
    bool grid{true};
    bool axes{true};     ///< The frame's X, Y and Z axes at its origin.
    bool sensors{true};  ///< Sensor frustums: each lidar's coverage fan and latest returns.
    bool follow{false};  ///< The camera keeps looking at the robot, or at the selection.

    Shading shading{Shading::Lit};
    bool wireframeAvailable{false};  ///< Set from SceneRenderer::wireframeSupported().
    /// Thermal shading: per scene prim, the temperature (K) its meshes take (NaN: none, drawn
    /// dim grey), and the range the colour map spans.
    std::vector<float> temperature;
    float temperatureLow{0.0F};
    float temperatureHigh{1.0F};
};

/// The design's thermal colour map (design tokens colormap.thermal, "inferno") at `t` in
/// [0, 1], as linear RGB for the mesh shader.
[[nodiscard]] std::array<float, 3> thermalColor(float t);

}  // namespace lagrange::viewer
