#pragma once

namespace lagrange::viewer {

/// What the viewport draws besides the scene's meshes.
struct RenderOptions {
    bool grid{true};
    bool axes{true};  ///< The frame's X, Y and Z axes at its origin.
};

}  // namespace lagrange::viewer
