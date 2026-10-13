#pragma once

#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::viewer {

/// What panels may read and change during one frame. Panels never touch the socket or Vulkan.
struct PanelContext {
    ViewerClient& client;
    std::shared_ptr<const SceneFrame> scene;  ///< Latest frame; may be null.
    ServerStatus status;
    ConnectionInfo connection;
    ClientStats stats;
    Camera& camera;
    std::uint32_t& selectedPrim;  ///< kNone when nothing is selected.

    std::uint64_t viewportTexture{0};  ///< ImTextureID of the rendered scene; 0 if none.
    std::uint32_t viewportWidth{0};    ///< Size of that texture in pixels.
    std::uint32_t viewportHeight{0};
    /// Out: the size the viewport panel wants next frame, in pixels.
    std::uint32_t requestedViewportWidth{0};
    std::uint32_t requestedViewportHeight{0};

    std::shared_ptr<const SignalCatalog> catalog;  ///< Server's signal list; may be null.
    /// Signals to open in the scope at start-up (command line); the scope panel empties it.
    /// May be null.
    std::vector<std::string>* initialScopeSignals{nullptr};

    std::string deviceName;
    std::string tierName;
    float frameMs{0.0F};
    std::uint32_t drawCalls{0};
};

/// One dockable panel. draw() runs on the UI thread every frame while the panel is open and
/// must not block or allocate in the steady state.
class Panel {
public:
    virtual ~Panel() = default;
    /// Stable identifier; also the window title, which is what ImGui keys layouts on.
    [[nodiscard]] virtual const char* title() const = 0;
    virtual void draw(PanelContext& context) = 0;
};

[[nodiscard]] std::unique_ptr<Panel> createScopePanel();

/// The v0.1 panels that exist so far, in default layout order.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createDefaultPanels();

/// Draws the dock space, lays the panels out on first use, and draws each panel.
void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context);

}  // namespace lagrange::viewer
