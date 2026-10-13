#pragma once

#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/ProfileSession.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// A workspace is one saved dock layout with its own panels (docs/design/ui: workspace bar).
/// Only the workspaces that exist so far are listed; the design has ten.
enum class Workspace : std::uint8_t { Scene, Domains };

[[nodiscard]] const char* toString(Workspace workspace);
/// "scene", "domains" (any case). Empty for anything else.
[[nodiscard]] std::optional<Workspace> parseWorkspace(std::string_view name);

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

    Notice notice;  ///< Latest message for the user (load results, command errors).
    /// Out: set when the user asked for the "open file" dialog. Only shown when the
    /// application can open one (not in headless mode).
    bool canOpenFiles{false};
    bool openFileRequested{false};

    Workspace* workspace{nullptr};            ///< The workspace shown; never null in draw().
    ProfileSession* profileSession{nullptr};  ///< Profile being edited; never null in draw().

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
    /// The workspace this panel belongs to. It is drawn only while that workspace is shown.
    [[nodiscard]] virtual Workspace workspace() const { return Workspace::Scene; }
};

[[nodiscard]] std::unique_ptr<Panel> createScopePanel();
[[nodiscard]] std::unique_ptr<Panel> createDomainsPanel();
[[nodiscard]] std::unique_ptr<Panel> createProfilePanel();

/// The v0.1 panels that exist so far, in default layout order.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createDefaultPanels();

/// Draws the application chrome (menu bar, workspace bar, domain strip, status bar) and the
/// panels of the workspace shown, laying them out on first use.
void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context);

}  // namespace lagrange::viewer
