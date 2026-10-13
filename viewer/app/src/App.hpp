#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::viewer {

struct Options {
    std::string endpoint;    ///< Empty: the default Unix socket.
    std::string gpu;         ///< Device index or part of its name; empty: automatic.
    bool validation{false};  ///< Vulkan validation layers.
    bool headless{false};    ///< No window: render offscreen, then exit.
    /// Headless: frames to render after the scene arrived (default 120 when 0).
    /// Windowed: close after this many frames; 0 means run until the window is closed.
    std::uint32_t frames{0};
    std::uint32_t waitMs{5000};             ///< Headless: how long to wait for the first scene.
    std::string screenshot;                 ///< Headless: PNG of the last frame (whole UI).
    std::vector<std::string> scopeSignals;  ///< Signals to open in the scope at start-up.
    std::string openPath;  ///< Robot description or recording to ask the server to load.
    std::uint32_t width{1600};
    std::uint32_t height{900};
};

/// Runs the viewer until the window closes (or the headless run completes).
/// Returns a process exit code.
int runViewer(const Options& options);

}  // namespace lagrange::viewer
