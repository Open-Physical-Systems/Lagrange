#pragma once

namespace lagrange::viewer {

/// What the viewport draws besides the scene's meshes.
struct RenderOptions {
    bool grid{true};
    bool axes{true};     ///< The frame's X, Y and Z axes at its origin.
    bool sensors{true};  ///< Sensor frustums: each lidar's coverage fan and latest returns.
};

}  // namespace lagrange::viewer
