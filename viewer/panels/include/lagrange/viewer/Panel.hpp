#pragma once

#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/ProfileSession.hpp"
#include "lagrange/viewer/RecentList.hpp"
#include "lagrange/viewer/RenderOptions.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ScopeSession.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// A workspace is one saved dock layout with its own panels (docs/design/ui: workspace bar).
/// All ten of the design are listed; the ones without panels yet show a notice.
enum class Workspace : std::uint8_t {
    Scene,
    Domains,
    Electrical,
    Signals,
    Sensors,
    Environment,
    Replay,
    Ros2,
    World,
    Performance,
};

[[nodiscard]] const char* toString(Workspace workspace);
/// A workspace by its label, in any case: "scene", "domains", "ros 2", ... Empty for anything else.
[[nodiscard]] std::optional<Workspace> parseWorkspace(std::string_view name);

/// What the machine the viewer runs on offers, probed locally at start-up (design: the
/// launcher's system check). Nothing here comes from a server.
struct SystemInfo {
    std::string gpu;        ///< Device name.
    std::string gpuDetail;  ///< "Vulkan 1.4 · ray query yes · 8 GB"
    std::string tier;       ///< "Tier 2"
    bool softwareGpu{false};
    std::string os;               ///< "Ubuntu 26.04 LTS"
    std::string session;          ///< "wayland", "x11", or empty when unknown.
    float scale{1.0F};            ///< HiDPI scale of the window's display.
    std::string rosDistro;        ///< ROS_DISTRO, empty when ROS is not sourced.
    std::string rmw;              ///< RMW_IMPLEMENTATION, may be empty.
    std::string rosDomain;        ///< ROS_DOMAIN_ID, may be empty.
    std::string configDirectory;  ///< Where the viewer keeps its files.
};

/// The first screen, before the viewer is connected: choose a server and what to open.
struct LauncherState {
    bool active{false};  ///< The launcher is shown in place of the workspaces.
    SystemInfo system;
    std::string defaultEndpoint;   ///< Where a local server listens unless told otherwise.
    bool defaultAvailable{false};  ///< Something is listening there (checked now and then).
    char otherEndpoint[160]{};     ///< A server typed in by the user.
    bool showOther{false};
    int server{1};           ///< 1: the default local server, 2: the typed one.
    int robotClass{0};       ///< Selected card.
    std::string openTarget;  ///< File to ask the server to open after connecting; may be empty.
    RecentList recent;
    char recentFilter[64]{};
    std::string error;  ///< Why the last attempt to connect could not start.

    // Out, read by the application after the frame.
    bool connectRequested{false};
    std::string connectEndpoint;
    bool disconnectRequested{false};  ///< Stop the client and show the launcher again.
};

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
    /// Signals for the scope to open: from the command line at start-up, or from a panel
    /// ("Plot in Signals"). The scope panel empties it. May be null.
    std::vector<std::string>* initialScopeSignals{nullptr};

    Notice notice;  ///< Latest message for the user (load results, command errors).
    /// Out: set when the user asked for the "open file" dialog. Only shown when the
    /// application can open one (not in headless mode).
    bool canOpenFiles{false};
    bool openFileRequested{false};

    Workspace* workspace{nullptr};            ///< The workspace shown; never null in draw().
    ProfileSession* profileSession{nullptr};  ///< Profile being edited; never null in draw().
    ScopeSession* scopeSession{nullptr};      ///< What is plotted; never null in draw().
    RenderOptions* renderOptions{nullptr};    ///< Viewport overlays; never null in draw().
    /// Times the user marked on the timeline, in simulation nanoseconds, ascending. Kept for
    /// the session only. Never null in draw().
    std::vector<std::int64_t>* bookmarks{nullptr};
    LauncherState* launcher{nullptr};  ///< Never null in draw().
    bool* resetLayout{nullptr};        ///< Set to rebuild the shown workspace's default layout.

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
    /// Stable identifier, which is what ImGui keys layouts on. A panel that shows a count in
    /// its tab draws "Title · N###Title"; layouts then refer to it as "###Title".
    [[nodiscard]] virtual const char* title() const = 0;
    virtual void draw(PanelContext& context) = 0;
    /// The workspace this panel belongs to. It is drawn only while that workspace is shown.
    [[nodiscard]] virtual Workspace workspace() const { return Workspace::Scene; }
};

[[nodiscard]] std::unique_ptr<Panel> createViewportPanel();
[[nodiscard]] std::unique_ptr<Panel> createScenePanel();
[[nodiscard]] std::unique_ptr<Panel> createInspectorPanel();
[[nodiscard]] std::unique_ptr<Panel> createElectricalInspectorPanel();
/// The Electrical workspace's Network, Schematic and Scopes panels. They share one state.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createElectricalPanels();
/// The Sensors workspace's list, lidar view, IMU scopes and sensor summary. One shared state.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createSensorsPanels();
/// The Domains workspace's Wiring, Unit checks and Schedule panels.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createWiringPanels();
/// The Performance workspace's overview, scheduler, modules, history and resources panels.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createPerformancePanels();
[[nodiscard]] std::unique_ptr<Panel> createTimelinePanel();
[[nodiscard]] std::unique_ptr<Panel> createBookmarksPanel();
[[nodiscard]] std::unique_ptr<Panel> createConsolePanel();
[[nodiscard]] std::unique_ptr<Panel> createProblemsPanel();
[[nodiscard]] std::unique_ptr<Panel> createScopePanel();
[[nodiscard]] std::unique_ptr<Panel> createSignalsPanel();
[[nodiscard]] std::unique_ptr<Panel> createCursorsPanel();
[[nodiscard]] std::unique_ptr<Panel> createStatisticsPanel();
[[nodiscard]] std::unique_ptr<Panel> createDomainsPanel();
[[nodiscard]] std::unique_ptr<Panel> createProfilePanel();

/// Every panel that exists so far, for all workspaces.
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createDefaultPanels();

/// Draws the launcher (see LauncherState). Called by drawPanels() while it is active.
void drawLauncher(PanelContext& context);

/// Draws the application chrome (menu bar, workspace bar, domain strip, status bar) and the
/// panels of the workspace shown, laying them out on first use.
void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context);

}  // namespace lagrange::viewer
