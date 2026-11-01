#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::viewer {

struct Options {
    std::string endpoint;    ///< Empty: the default Unix socket.
    std::string tlsCa;       ///< CA file for a tls:// server; empty: $LAGRANGE_TLS_CA or system.
    std::string authToken;   ///< Token for a TCP server; empty: $LAGRANGE_AUTH_TOKEN.
    std::string gpu;         ///< Device index or part of its name; empty: automatic.
    bool validation{false};  ///< Vulkan validation layers.
    bool headless{false};    ///< No window: render offscreen, then exit.
    /// Headless: frames to render after the scene arrived (default 120 when 0).
    /// Windowed: close after this many frames; 0 means run until the window is closed.
    std::uint32_t frames{0};
    std::uint32_t waitMs{5000};  ///< Headless: how long to wait for the first scene.
    std::string screenshot;      ///< Headless: PNG of the last frame (whole UI).
    /// Headless: folder to write a frame sequence to (frame_00000.png, ...), for videos.
    std::string recordFrames;
    std::uint32_t recordFps{15};  ///< Headless: frames per second of wall-clock time to write.
    std::vector<std::string> scopeSignals;  ///< Signals to open in the scope at start-up.
    std::string workspace;  ///< Workspace to show at start-up ("scene", "domains"); empty: scene.
    /// Domain switches to flip in the profile draft once the server's profile arrived,
    /// as (domain, enabled). They are edits, not applied, unless applyProfile is set.
    std::vector<std::pair<std::string, bool>> domainEdits;
    bool applyProfile{false};  ///< Apply the edited profile as soon as the server calls it valid.
    bool launcher{false};      ///< Start on the launcher even when a server is named.
    bool startLocal{false};    ///< Start lagrange_server from the launcher, then connect.
    bool cursors{false};       ///< Show the scope's A and B cursors from the start.
    std::string season;        ///< Start-up season: summer, fall, winter or rainy.
    std::string shading;       ///< Viewport shading at start-up: lit, unlit or thermal.
    std::string overlays;      ///< --overlays: grid,axes,sensors shown from the start.
    std::optional<std::array<double, 6>> camera;  ///< --camera: target, distance, yaw, pitch.
    double followDistance{0.0};  ///< > 0: start with the follow camera this far (m) away.
    double followPitchDeg{0.0};  ///< With followDistance: the camera's elevation; 0: default.
    std::string selectPath;      ///< Prim to select once the scene arrived, by its full path.
    std::string exportNetlist;   ///< --export-netlist: write the stage's circuit to this file.
    std::string input;           ///< --input: headless mouse and keyboard, see parseInput.
    std::string focusPanel;      ///< --panel: a panel to bring in front of its tabs, by title.
    std::string openPath;        ///< Robot description or recording to ask the server to load.
    std::uint32_t width{1600};
    std::uint32_t height{900};
};

/// Runs the viewer until the window closes (or the headless run completes).
/// Returns a process exit code.
int runViewer(const Options& options);

}  // namespace lagrange::viewer
