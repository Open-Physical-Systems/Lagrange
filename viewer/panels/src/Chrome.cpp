// The application chrome shared by every workspace: menu bar, workspace bar, domain strip,
// dock space and status bar (docs/design/ui/shared/ui-conventions.md §1).
#include "BuilderCommon.hpp"
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <imgui_internal.h>  // DockBuilder and BeginViewportSideBar
#include <string>
#include <unordered_map>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

struct WorkspaceEntry {
    Workspace workspace;
    const char* label;
    widgets::Icon icon;
    const char* dockspace;  // nullptr: the workspace is not built yet
    ImGuiKey key;
    const char* shortcut;
    const char* design;  // its folder in docs/design/ui
};

// The ten workspaces of the design, in its order, with its shortcuts (Ctrl 1 ... Ctrl 0), and
// the Builder (RFC 0016, docs/design/ui/12-builder: Ctrl B, after a rule).
// The dock space names carry a layout version: change it when a default layout changes, so
// that a layout saved by an older build is not applied to the new panels.
constexpr std::array<WorkspaceEntry, 11> kWorkspaces{{
    {Workspace::Scene, "Scene", widgets::Icon::Scene, "lagrange-dock-scene-4", ImGuiKey_1, "Ctrl+1",
     "02-scene"},
    {Workspace::Domains, "Domains", widgets::Icon::Domains, "lagrange-dock-domains-3", ImGuiKey_2,
     "Ctrl+2", "03-domains"},
    {Workspace::Electrical, "Electrical", widgets::Icon::Electrical, "lagrange-dock-electrical-1",
     ImGuiKey_3, "Ctrl+3", "04-electrical"},
    {Workspace::Signals, "Signals", widgets::Icon::Signals, "lagrange-dock-signals-3", ImGuiKey_4,
     "Ctrl+4", "05-signals"},
    {Workspace::Sensors, "Sensors", widgets::Icon::Sensors, "lagrange-dock-sensors-2", ImGuiKey_5,
     "Ctrl+5", "06-sensors"},
    {Workspace::Environment, "Environment", widgets::Icon::Environment,
     "lagrange-dock-environment-1", ImGuiKey_6, "Ctrl+6", "07-environment"},
    {Workspace::Replay, "Replay", widgets::Icon::Replay, "lagrange-dock-replay-1", ImGuiKey_7,
     "Ctrl+7", "08-replay"},
    {Workspace::Ros2, "ROS 2", widgets::Icon::Ros2, "lagrange-dock-ros2-1", ImGuiKey_8, "Ctrl+8",
     "09-ros2"},
    {Workspace::World, "World", widgets::Icon::World, "lagrange-dock-world-1", ImGuiKey_9, "Ctrl+9",
     "10-world"},
    {Workspace::Performance, "Performance", widgets::Icon::Performance,
     "lagrange-dock-performance-1", ImGuiKey_0, "Ctrl+0", "11-performance"},
    {Workspace::Builder, "Builder", widgets::Icon::Builder, "lagrange-dock-builder-1", ImGuiKey_B,
     "Ctrl+B", "12-builder"},
}};

const WorkspaceEntry& entryOf(Workspace workspace) {
    for (const WorkspaceEntry& entry : kWorkspaces) {
        if (entry.workspace == workspace) {
            return entry;
        }
    }
    return kWorkspaces.front();
}

constexpr std::int64_t kStepNs = 10'000'000;  // one step from the toolbar or the '.' key: 10 ms

constexpr ImGuiWindowFlags kBarFlags =
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;

void buildLayout(Workspace workspace, ImGuiID dockspace, const ImVec2& size) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, size);
    ImGuiID centre = dockspace;
    const auto fraction = [](float pixels, float of) {
        return std::clamp(px(pixels) / std::max(of, 1.0F), 0.1F, 0.6F);
    };
    if (workspace == Workspace::Scene) {
        // Design: Scene 280 px left, Inspector 340 px right; under the viewport a 196 px
        // band, 62 % Timeline and 38 % Console | Problems.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(280.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(340.0F, size.x - px(280.0F)), nullptr, &centre);
        ImGuiID bottom = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down,
                                                     fraction(196.0F, size.y), nullptr, &centre);
        const ImGuiID console =
            ImGui::DockBuilderSplitNode(bottom, ImGuiDir_Right, 0.38F, nullptr, &bottom);
        ImGui::DockBuilderDockWindow("Viewport", centre);
        ImGui::DockBuilderDockWindow("Scene", left);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderDockWindow("Timeline", bottom);
        ImGui::DockBuilderDockWindow("###Bookmarks", bottom);
        ImGui::DockBuilderDockWindow("Console", console);
        ImGui::DockBuilderDockWindow("###Problems", console);
    } else if (workspace == Workspace::Electrical) {
        // Design: Network 300 px left, Inspector 340 px right; the schematic over a 372 px
        // band of scopes.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(300.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(340.0F, size.x - px(300.0F)), nullptr, &centre);
        const ImGuiID scopes = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Down, fraction(372.0F, size.y), nullptr, &centre);
        ImGui::DockBuilderDockWindow("Network", left);
        ImGui::DockBuilderDockWindow("Schematic", centre);
        ImGui::DockBuilderDockWindow("Circuit", centre);
        ImGui::DockBuilderDockWindow("Netlist", centre);
        ImGui::DockBuilderDockWindow("3D overlay", centre);
        ImGui::DockBuilderDockWindow("Scopes", scopes);
        ImGui::DockBuilderDockWindow("###InspectorElectrical", right);
    } else if (workspace == Workspace::Builder) {
        // Design (12-builder SPEC "Dock layout"): left 270 px, Robot | Files over a 212 px
        // Add | Templates; right 350 px, Properties | History; the Builder view over a 220 px
        // band, 55 % Joints | Poses and 45 % Sensor catalog | Problems.
        namespace w = builder_windows;
        ImGuiID left = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, fraction(270.0F, size.x),
                                                   nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(350.0F, size.x - px(270.0F)), nullptr, &centre);
        const ImGuiID add = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down,
                                                        fraction(212.0F, size.y), nullptr, &left);
        ImGuiID bottom = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down,
                                                     fraction(220.0F, size.y), nullptr, &centre);
        const ImGuiID catalog =
            ImGui::DockBuilderSplitNode(bottom, ImGuiDir_Right, 0.45F, nullptr, &bottom);
        ImGui::DockBuilderDockWindow(w::kRobot, left);
        ImGui::DockBuilderDockWindow(w::kFiles, left);
        ImGui::DockBuilderDockWindow(w::kAdd, add);
        ImGui::DockBuilderDockWindow(w::kTemplates, add);
        ImGui::DockBuilderDockWindow(w::kProperties, right);
        ImGui::DockBuilderDockWindow(w::kHistory, right);
        ImGui::DockBuilderDockWindow(w::kView, centre);
        ImGui::DockBuilderDockWindow(w::kJoints, bottom);
        ImGui::DockBuilderDockWindow(w::kPoses, bottom);
        ImGui::DockBuilderDockWindow(w::kCatalog, catalog);
        ImGui::DockBuilderDockWindow(w::kProblems, catalog);
        // The Test layout (SPEC state 4): Drive and the test server's Console join the bottom
        // right while a Test runs; they submit nothing in Build.
        ImGui::DockBuilderDockWindow(w::kDrive, catalog);
        ImGui::DockBuilderDockWindow(w::kTestConsole, catalog);
    } else if (workspace == Workspace::Performance) {
        // Design: a row of KPI cards on top; under it the scheduler (320 px) over the
        // modules, and on the right (380 px) the RTF history (200 px) over the resources.
        const ImGuiID top = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Up,
                                                        fraction(104.0F, size.y), nullptr, &centre);
        ImGuiID right = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right,
                                                    fraction(380.0F, size.x), nullptr, &centre);
        const ImGuiID resources =
            ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.68F, nullptr, &right);
        const ImGuiID modules =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.58F, nullptr, &centre);
        ImGui::DockBuilderDockWindow("###PerformanceOverview", top);
        ImGui::DockBuilderDockWindow("###Scheduler", centre);
        ImGui::DockBuilderDockWindow("###PerformanceModules", modules);
        ImGui::DockBuilderDockWindow("RTF history", right);
        ImGui::DockBuilderDockWindow("Resources", resources);
        // The KPI row is not a tabbed panel in the design.
        if (ImGuiDockNode* node = ImGui::DockBuilderGetNode(top)) {
            node->LocalFlags |= static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_NoTabBar) |
                                static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_NoResize);
        }
    } else if (workspace == Workspace::Replay) {
        // Design: Recording over Channels and Bookmarks, 280 px left; the timeline (400 px)
        // over the events. (Sim vs real and Validation are not built: they need a second log.)
        ImGuiID left = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, fraction(280.0F, size.x),
                                                   nullptr, &centre);
        const ImGuiID channels =
            ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.66F, nullptr, &left);
        const ImGuiID events =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.52F, nullptr, &centre);
        ImGui::DockBuilderDockWindow("Recording", left);
        ImGui::DockBuilderDockWindow("Channels", channels);
        ImGui::DockBuilderDockWindow("###ReplayBookmarks", channels);
        ImGui::DockBuilderDockWindow("###ReplayTimeline", centre);
        ImGui::DockBuilderDockWindow("###ReplayEvents", events);
    } else if (workspace == Workspace::Environment) {
        // Design: Environment 320 px left, Overlays 300 px right, the field view between.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(320.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(300.0F, size.x - px(320.0F)), nullptr, &centre);
        ImGui::DockBuilderDockWindow("###EnvironmentState", left);
        ImGui::DockBuilderDockWindow("###EnvironmentPresets", left);
        ImGui::DockBuilderDockWindow("###FieldSlice", centre);
        ImGui::DockBuilderDockWindow("Overlays", right);
    } else if (workspace == Workspace::Ros2) {
        // Design: Topics 440 px left, ros2_control 340 px right, the node graph over a
        // 300 px console.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(440.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(340.0F, size.x - px(440.0F)), nullptr, &centre);
        const ImGuiID console = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Down, fraction(300.0F, size.y), nullptr, &centre);
        ImGui::DockBuilderDockWindow("Topics", left);
        ImGui::DockBuilderDockWindow("Node graph", centre);
        ImGui::DockBuilderDockWindow("###RosConsole", console);
        ImGui::DockBuilderDockWindow("ros2_control", right);
    } else if (workspace == Workspace::World) {
        // Design: Agents 300 px left, Streaming 320 px right, the map between.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(300.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(320.0F, size.x - px(300.0F)), nullptr, &centre);
        ImGui::DockBuilderDockWindow("Agents", left);
        ImGui::DockBuilderDockWindow("Map", centre);
        ImGui::DockBuilderDockWindow("Streaming", right);
    } else if (workspace == Workspace::Sensors) {
        // Design: sensor list 260 px left, Degradation 320 px right, sensor views between.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(260.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(320.0F, size.x - px(260.0F)), nullptr, &centre);
        ImGuiID imu = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.40F, nullptr, &centre);
        // The bottom of the grid: IMU | radar (the design's magnetometer place, not built yet).
        const ImGuiID radar = ImGui::DockBuilderSplitNode(imu, ImGuiDir_Right, 0.5F, nullptr, &imu);
        // The top of the 2x2 grid: lidar | camera.
        const ImGuiID camera =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.5F, nullptr, &centre);
        ImGui::DockBuilderDockWindow("###SensorList", left);
        ImGui::DockBuilderDockWindow("###Lidar", centre);
        ImGui::DockBuilderDockWindow("###Camera", camera);
        ImGui::DockBuilderDockWindow("###Imu", imu);
        ImGui::DockBuilderDockWindow("###Radar", radar);
        ImGui::DockBuilderDockWindow("Degradation", right);
    } else if (workspace == Workspace::Domains) {
        // Design: Domains 520 px left, Profile 420 px right; between them the wiring over a
        // 268 px schedule.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(520.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(420.0F, size.x - px(520.0F)), nullptr, &centre);
        const ImGuiID schedule = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Down, fraction(268.0F, size.y), nullptr, &centre);
        ImGui::DockBuilderDockWindow("Domains", left);
        ImGui::DockBuilderDockWindow("Wiring", centre);
        ImGui::DockBuilderDockWindow("###UnitChecks", centre);
        ImGui::DockBuilderDockWindow("Schedule", schedule);
        ImGui::DockBuilderDockWindow("Profile", right);
    } else {
        // Design: Signals 300 px left; Cursors over Statistics, 300 px right; scope between.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(300.0F, size.x), nullptr, &centre);
        ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(300.0F, size.x - px(300.0F)), nullptr, &centre);
        const ImGuiID statistics =
            ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.62F, nullptr, &right);
        ImGui::DockBuilderDockWindow("Signals", left);
        ImGui::DockBuilderDockWindow("Scope", centre);
        ImGui::DockBuilderDockWindow("Cursors", right);
        ImGui::DockBuilderDockWindow("Statistics", statistics);
    }
    ImGui::DockBuilderFinish(dockspace);
}

void bottomRule() {
    const ImVec2 min = ImGui::GetWindowPos();
    const ImVec2 size = ImGui::GetWindowSize();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(min.x, min.y + size.y - 1.0F),
                                        ImVec2(min.x + size.x, min.y + size.y - 1.0F),
                                        ImGui::ColorConvertFloat4ToU32(c::line));
}

/// Puts the cursor so that an item of `height` is centred in the bar.
void centreInBar(float height) {
    ImGui::SetCursorPosY((ImGui::GetWindowHeight() - height) * 0.5F);
}

/// "key value" as the menu bar's context shows it: 12 px, key in t3, value in t2 medium.
float contextWidth(const char* key, const char* value) {
    const theme::Fonts& fonts = theme::fonts();
    const float size = px(theme::kLabelSize);
    return fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, key).x + px(5.0F) +
           fonts.sansMedium->CalcTextSizeA(size, FLT_MAX, 0.0F, value).x;
}

void contextItem(const char* key, const char* value, bool mono = false) {
    ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
    ImGui::TextColored(c::t3, "%s", key);
    ImGui::PopFont();
    ImGui::SameLine(0.0F, px(5.0F));
    // A path is mono at 11 px (12-builder reference.html: the menu bar's file).
    ImGui::PushFont(mono ? theme::fonts().mono : theme::fonts().sansMedium,
                    mono ? theme::kCaptionSize : theme::kLabelSize);
    ImGui::TextColored(c::t2, "%s", value);
    ImGui::PopFont();
}

float monoContextWidth(const char* key, const char* value) {
    const theme::Fonts& fonts = theme::fonts();
    return fonts.sans->CalcTextSizeA(px(theme::kLabelSize), FLT_MAX, 0.0F, key).x + px(5.0F) +
           fonts.mono->CalcTextSizeA(px(theme::kCaptionSize), FLT_MAX, 0.0F, value).x;
}

/// The widest the menu bar's file path may be (logical px), so it never reaches the menus.
constexpr float kContextPathWidth = 320.0F;

/// A path with the home folder written "~" (the menu bar's file, as the reference writes it).
std::string homePath(const std::filesystem::path& path) {
    std::string text = path.string();
    const char* home = std::getenv("HOME");
    if (home != nullptr && *home != '\0') {
        const std::string prefix = std::string(home) + "/";
        if (text.starts_with(prefix)) {
            text = "~/" + text.substr(prefix.size());
        }
    }
    return text;
}

/// The Builder's context (12-builder reference.html `.ctx`): the robot being built, its file
/// (or "not saved yet"), and the server this viewer is connected to, or "none".
void builderContext(const PanelContext& context) {
    const BuilderSession& session = *context.builder;
    const std::string robot = session.document().robot;
    std::string file = session.path.empty() ? std::string("not saved yet") : homePath(session.path);
    // A deep path keeps its last folders and the file name: the menus need the room.
    const float most = px(kContextPathWidth);
    std::string tail = file;
    while (monoContextWidth("file", file.c_str()) > most) {
        const std::size_t slash = tail.find('/', 1);
        if (slash == std::string::npos || tail.find('/', slash + 1) == std::string::npos) {
            break;  // the folder and the file name stay
        }
        tail = tail.substr(slash);
        file = "…" + tail;
    }
    const bool ready = context.connection.state == ConnectionState::Ready;
    const std::string server = ready && !context.connection.serverName.empty()
                                   ? context.connection.serverName
                                   : std::string("none");
    const float gap = px(12.0F);
    const float width = px(44.0F) + contextWidth("robot", robot.c_str()) + gap +
                        monoContextWidth("file", file.c_str()) + gap +
                        contextWidth("server", server.c_str()) + gap;
    ImGui::SameLine();
    widgets::AlignRight(width);
    contextItem("robot", robot.c_str());
    ImGui::SameLine(0.0F, gap);
    contextItem("file", file.c_str(), true);
    ImGui::SameLine(0.0F, gap);
    contextItem("server", server.c_str());
    ImGui::SameLine(0.0F, gap);
    widgets::Kbd("Ctrl K");
}

/// A recording seeks back to its first moment; a live run restarts at t = 0 on the server.
void resetToStart(PanelContext& context) {
    if (context.connection.canSeek) {
        context.client.seek(context.status.replayBeginNs);
    } else {
        context.client.reset();
    }
}

void drawMenuBar(PanelContext& context) {
    // Design (.mb): 28 px, bg1; the mark and "Lagrange" in semibold, a rule, then the menus
    // in t2; on the right the stage, robot and profile in use and the Ctrl K hint.
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(8.0F), px(7.5F)));  // 28 px bar
    const bool open = ImGui::BeginMainMenuBar();
    ImGui::PopStyleVar();
    if (!open) {
        return;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float bar = ImGui::GetWindowHeight();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float mark = px(16.0F);
    widgets::DrawLogo(draw, ImVec2(origin.x, ImGui::GetWindowPos().y + (bar - mark) * 0.5F), mark);
    ImGui::Dummy(ImVec2(mark, 1.0F));
    ImGui::SameLine(0.0F, px(7.0F));
    ImGui::PushFont(theme::fonts().sansSemiBold, theme::kBodySize);
    ImGui::TextUnformatted("Lagrange");
    ImGui::PopFont();
    ImGui::SameLine(0.0F, px(12.0F));
    const ImVec2 rule = ImGui::GetCursorScreenPos();
    const float top = ImGui::GetWindowPos().y + (bar - px(18.0F)) * 0.5F;
    draw->AddLine(ImVec2(rule.x, top), ImVec2(rule.x, top + px(18.0F)),
                  ImGui::ColorConvertFloat4ToU32(c::line));
    ImGui::Dummy(ImVec2(px(4.0F), 1.0F));
    ImGui::SameLine(0.0F, 0.0F);

    const bool ready = context.connection.state == ConnectionState::Ready;
    const bool running = context.status.runState == RunState::Running;
    ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(px(8.0F), px(4.0F)));
    const auto menu = [](const char* label) {
        const bool shown = ImGui::BeginMenu(label);
        if (shown) {
            ImGui::PushStyleColor(ImGuiCol_Text, c::t1);  // items inside are in primary text
        }
        return shown;
    };
    const auto endMenu = [] {
        ImGui::PopStyleColor();
        ImGui::EndMenu();
    };
    if (menu("File")) {
        if (ImGui::MenuItem("Add robot or open recording...", "Ctrl+O", false,
                            context.canOpenFiles)) {
            context.openFileRequested = true;
        }
        if (ImGui::MenuItem("Disconnect and show the launcher")) {
            context.launcher->disconnectRequested = true;
        }
        endMenu();
    }
    if (menu("Edit")) {
        if (ImGui::MenuItem("Clear selection", "Esc", false, context.selectedPrim != kNone)) {
            context.selectedPrim = kNone;
        }
        if (ImGui::MenuItem("Revert profile edits", nullptr, false,
                            context.profileSession->draft().dirty())) {
            context.profileSession->revert();
        }
        endMenu();
    }
    if (menu("View")) {
        for (const WorkspaceEntry& entry : kWorkspaces) {
            if (ImGui::MenuItem(entry.label, entry.shortcut,
                                *context.workspace == entry.workspace)) {
                *context.workspace = entry.workspace;
            }
        }
        endMenu();
    }
    if (menu("Simulation")) {
        if (ImGui::MenuItem(running ? "Pause" : "Play", "Space", false, ready)) {
            if (running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        if (ImGui::MenuItem("Step", ".", false, ready && context.connection.canStep && !running)) {
            context.client.step(kStepNs);
        }
        if (ImGui::MenuItem("Reset to start", nullptr, false, ready)) {
            resetToStart(context);
        }
        endMenu();
    }
    if (menu("Tools")) {
        ImGui::MenuItem("Command palette", "Ctrl+K", false, false);  // not built yet
        endMenu();
    }
    if (menu("Window")) {
        if (ImGui::MenuItem("Reset this workspace's layout")) {
            *context.resetLayout = true;
        }
        endMenu();
    }
    if (menu("Help")) {
        ImGui::MenuItem("Lagrange viewer, v0.1 in development", nullptr, false, false);
        ImGui::MenuItem("Apache-2.0 · fonts: IBM Plex, SIL OFL 1.1", nullptr, false, false);
        endMenu();
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();

    // Context on the right.
    if (*context.workspace == Workspace::Builder && context.builder != nullptr) {
        builderContext(context);
        ImGui::EndMainMenuBar();
        return;
    }
    // Building offline (12-builder SPEC state 5) shows no run: not the one a Test left behind
    // after Back when nothing was connected before it.
    const bool offlineBuilder = *context.workspace == Workspace::Builder &&
                                context.connection.state != ConnectionState::Ready;
    const SceneFrame* scene = offlineBuilder ? nullptr : context.scene.get();
    // The file name only: a full path does not fit beside the run controls.
    const char* stage = scene != nullptr ? scene->statics->stagePath.c_str() : "";
    if (const char* slash = std::strrchr(stage, '/'); slash != nullptr && slash[1] != '\0') {
        stage = slash + 1;
    }
    const char* robot = scene != nullptr && !scene->statics->prims.empty()
                            ? scene->statics->prims.front().name.c_str()
                            : "";
    char profile[48] = "";
    if (!offlineBuilder && context.profileSession->profile()) {
        const std::size_t changes = context.profileSession->draft().changedLineCount();
        if (changes == 0) {
            std::snprintf(profile, sizeof(profile), "applied");
        } else {
            std::snprintf(profile, sizeof(profile), "%zu unapplied", changes);
        }
    }
    const float gap = px(12.0F);
    float width = px(44.0F);  // the Ctrl K hint
    if (*stage != '\0') {
        width += contextWidth("stage", stage) + gap;
    }
    if (*robot != '\0') {
        width += contextWidth("robot", robot) + gap;
    }
    if (*profile != '\0') {
        width += contextWidth("profile", profile) + gap;
    }
    ImGui::SameLine();
    widgets::AlignRight(width);
    if (*stage != '\0') {
        contextItem("stage", stage);
        ImGui::SameLine(0.0F, gap);
    }
    if (*robot != '\0') {
        contextItem("robot", robot);
        ImGui::SameLine(0.0F, gap);
    }
    if (*profile != '\0') {
        contextItem("profile", profile);
        ImGui::SameLine(0.0F, gap);
    }
    widgets::Kbd("Ctrl K");
    ImGui::EndMainMenuBar();
}

// --- the Builder's chrome (12-builder SPEC "Modes", "Strip under the workspace bar") ---------

/// True when `program` is an executable in a $PATH folder. Looked up once per program.
bool onPath(const char* program) {
    const char* path = std::getenv("PATH");
    if (path == nullptr) {
        return false;
    }
    std::string_view rest(path);
    while (!rest.empty()) {
        const std::size_t colon = rest.find(':');
        const std::filesystem::path folder(std::string(rest.substr(0, colon)));
        std::error_code ec;
        if (!folder.empty() && std::filesystem::is_regular_file(folder / program, ec)) {
            return true;
        }
        rest = colon == std::string_view::npos ? std::string_view{} : rest.substr(colon + 1);
    }
    return false;
}

/// `lagrange check` runs on this machine (SPEC: Check loads… is hidden otherwise).
bool canCheckLoads() {
    static const bool found = onPath("lagrange");
    return found;
}

/// The document's file name as the readout shows it: the robot name until first saved.
std::string documentName(const BuilderSession& session) {
    if (!session.path.empty()) {
        return session.path.filename().string();
    }
    return session.document().robot + ".urdf";
}

/// Build mode (SPEC "Modes"): Undo, Redo, the document readout, Build, Test · target ▾, Save.
void drawBuildControls(PanelContext& context) {
    BuilderSession& session = *context.builder;
    const std::string name = documentName(session);
    const int unsaved = session.history.editsSinceSave();
    char after[48] = "";
    if (unsaved > 0 || session.path.empty()) {
        std::snprintf(after, sizeof(after), "unsaved · %d", unsaved);
    }
    const std::string target = session.testIntoRun          ? "connected run"
                               : session.testTarget.empty() ? "empty world"
                                                            : session.testTarget;
    const std::string test = "Test · " + target + "      ";  // room for the chevron
    const float gap = px(4.0F);
    const float width = px(26.0F) * 2.0F + widgets::ReadoutWidth("doc", name.c_str(), after) +
                        widgets::ButtonWidth("Build", widgets::Icon::Builder, true) +
                        widgets::ButtonWidth(test.c_str(), widgets::Icon::Play, true) +
                        widgets::ButtonWidth("Save", widgets::Icon::Save, true) + gap * 5.0F +
                        px(10.0F);
    ImGui::SameLine();
    widgets::AlignRight(width + px(4.0F));
    const ImVec2 rule = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddLine(
        ImVec2(rule.x, ImGui::GetWindowPos().y),
        ImVec2(rule.x, ImGui::GetWindowPos().y + ImGui::GetWindowHeight()),
        ImGui::ColorConvertFloat4ToU32(c::line));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(10.0F));
    centreInBar(px(26.0F));
    const float y = ImGui::GetCursorPosY();
    const auto next = [&] {
        ImGui::SameLine(0.0F, gap);
        ImGui::SetCursorPosY(y);
    };

    ImGui::BeginDisabled(!session.history.canUndo());
    if (widgets::Button("##undo", widgets::ButtonKind::Default, widgets::Icon::Undo, true)) {
        (void)session.history.undo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Undo (Ctrl Z)");
    next();
    ImGui::BeginDisabled(!session.history.canRedo());
    if (widgets::Button("##redo", widgets::ButtonKind::Default, widgets::Icon::Redo, true)) {
        (void)session.history.redo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Redo (Ctrl Shift Z)");
    next();
    widgets::Readout("doc", name.c_str(), after, c::t1, c::warn);
    if (!session.path.empty()) {
        ImGui::SetItemTooltip("%s", session.path.string().c_str());
    }
    next();
    (void)widgets::Button("Build", widgets::ButtonKind::On, widgets::Icon::Builder, true);
    ImGui::SetItemTooltip("Build mode: no simulation");
    next();

    // Test · target ▾ (the primary action). Refused while there is an error (SPEC state 3) or
    // without lagrange_server on this machine (state 5).
    // Into the connected run (RFC 0016 §7.2) needs a server that imports instead.
    const int errors = session.problems.errors();
    const bool connected = context.connection.state == ConnectionState::Ready;
    const bool intoRun = connected && context.connection.canImport;
    if (intoRun && context.launcher->localServer.empty()) {
        session.testIntoRun = true;  // SPEC state 5: then the only target
    }
    const bool installed = session.testIntoRun ? intoRun : !context.launcher->localServer.empty();
    ImGui::BeginDisabled(errors > 0 || !installed);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float testWidth = widgets::ButtonWidth(test.c_str(), widgets::Icon::Play, true);
    if (widgets::Button(test.c_str(), widgets::ButtonKind::Primary, widgets::Icon::Play, true)) {
        if (ImGui::GetIO().MousePos.x > at.x + testWidth - px(24.0F)) {
            ImGui::OpenPopup("##test-target");
        } else {
            session.testRequested = true;
            session.testWorld = session.testTarget.empty() ? "none" : session.testTarget;
        }
    }
    const float chevron = px(10.0F);
    widgets::DrawIcon(ImGui::GetWindowDrawList(), widgets::Icon::ChevronDown,
                      ImVec2(at.x + testWidth - px(10.0F) - chevron, at.y + px(8.0F)), chevron,
                      ImGui::ColorConvertFloat4ToU32(ImVec4(
                          c::onAccent.x, c::onAccent.y, c::onAccent.z,
                          errors > 0 || !installed ? ImGui::GetStyle().DisabledAlpha : 1.0F)),
                      ImGui::ColorConvertFloat4ToU32(c::accent));
    ImGui::EndDisabled();
    if (errors > 0) {
        ImGui::SetItemTooltip("Fix %d problem%s first", errors, errors == 1 ? "" : "s");
    } else if (!installed) {
        ImGui::SetItemTooltip(session.testIntoRun ? "Not connected to a server that imports robots"
                                                  : "lagrange_server is not installed");
    } else if (session.testIntoRun) {
        ImGui::SetItemTooltip("Add the robot to the run this viewer is connected to");
    } else {
        ImGui::SetItemTooltip("Test the robot in a simulator of its own on this machine");
    }
    if (ImGui::BeginPopup("##test-target")) {
        if (ImGui::MenuItem("Empty world", nullptr,
                            !session.testIntoRun && session.testTarget.empty())) {
            session.testTarget.clear();
            session.testIntoRun = false;
        }
        if (ImGui::MenuItem("A sample world…", nullptr, false, false)) {
        }
        ImGui::SetItemTooltip("Opens with WP-F's world picker");
        if (ImGui::MenuItem("A world file…", nullptr, false, false)) {
        }
        ImGui::SetItemTooltip("Opens with WP-F's file dialog");
        if (ImGui::MenuItem("Into the connected run", nullptr, session.testIntoRun, intoRun)) {
            session.testIntoRun = true;
        }
        ImGui::SetItemTooltip(
            "%s", intoRun     ? "Add the robot to the run this viewer shows; Back removes it"
                  : connected ? "This server does not import robots (a replay)"
                              : "Connect to a server first");
        ImGui::EndPopup();
    }
    next();
    if (widgets::Button("Save", widgets::ButtonKind::Default, widgets::Icon::Save, true)) {
        ImGui::OpenPopup("##save-menu");
    }
    ImGui::SetItemTooltip("Save (Ctrl S), Export USD…, Check loads…");
    if (ImGui::BeginPopup("##save-menu")) {
        if (ImGui::MenuItem("Save", "Ctrl+S")) {
            session.saveRequested = true;
            session.savePath.clear();
        }
        if (ImGui::MenuItem("Save as…", nullptr, false, context.canOpenFiles)) {
            session.saveAsDialogRequested = true;
        }
        if (!context.canOpenFiles) {
            ImGui::SetItemTooltip("Needs the windowed viewer's file dialog");
        }
        if (ImGui::MenuItem("Open…", "Ctrl+O", false, context.canOpenFiles)) {
            session.openDialogRequested = true;
        }
        if (ImGui::MenuItem("Export USD…")) {
            session.exportUsdRequested = true;
        }
        if (ImGui::MenuItem("Check loads…", nullptr, false, canCheckLoads())) {
            session.checkRequested = true;
        }
        if (errors > 0) {
            ImGui::Separator();
            ImGui::TextColored(c::warn, "%d problem%s: the file saves, but will not load yet",
                               errors, errors == 1 ? "" : "s");
        }
        ImGui::EndPopup();
    }
}

/// Testing (SPEC state 4): Back to Build, the Scene's Play/Pause, Step and readouts, and the
/// TEST pill in place of REPLAY's.
void drawTestControls(PanelContext& context) {
    BuilderSession& session = *context.builder;
    const bool ready = context.connection.state == ConnectionState::Ready &&
                       session.mode == BuilderSession::Mode::Testing;
    const bool running = context.status.runState == RunState::Running;
    char time[32];
    format::clock(context.status.simTimeNs, time, sizeof(time));
    char rate[24];
    std::snprintf(rate, sizeof(rate), "%.2f×", context.status.realTimeFactor);
    char pill[96];
    std::snprintf(pill, sizeof(pill), "%s · %s%s",
                  session.mode == BuilderSession::Mode::Testing ? "TEST" : "STARTING",
                  session.testRobotName.empty() ? session.document().robot.c_str()
                                                : session.testRobotName.c_str(),
                  session.testPassive ? " · passive" : "");
    const float gap = px(4.0F);
    ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
    const float playWidth =
        px(20.0F) + px(14.0F) + px(6.0F) + ImGui::CalcTextSize(running ? "Pause" : "Play").x;
    ImGui::PopFont();
    const float width = widgets::ButtonWidth("Back to Build", widgets::Icon::Count, true) +
                        playWidth + px(26.0F) + widgets::ReadoutWidth("sim t", time) +
                        widgets::ReadoutWidth("RTF", rate) + widgets::PillWidth(pill) + gap * 6.0F +
                        px(10.0F);
    ImGui::SameLine();
    widgets::AlignRight(width + px(4.0F));
    const ImVec2 rule = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddLine(
        ImVec2(rule.x, ImGui::GetWindowPos().y),
        ImVec2(rule.x, ImGui::GetWindowPos().y + ImGui::GetWindowHeight()),
        ImGui::ColorConvertFloat4ToU32(c::line));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(10.0F));
    centreInBar(px(26.0F));
    const float y = ImGui::GetCursorPosY();
    const auto next = [&] {
        ImGui::SameLine(0.0F, gap);
        ImGui::SetCursorPosY(y);
    };
    if (widgets::Button("Back to Build", widgets::ButtonKind::Default, widgets::Icon::Count,
                        true)) {
        session.backRequested = true;
    }
    ImGui::SetItemTooltip("Stop the test and edit again (Esc)");
    next();
    ImGui::BeginDisabled(!ready);
    if (widgets::Button(running ? "Pause" : "Play", widgets::ButtonKind::Primary,
                        running ? widgets::Icon::Pause : widgets::Icon::Play, true)) {
        if (running) {
            context.client.pause();
        } else {
            context.client.play();
        }
    }
    next();
    ImGui::BeginDisabled(!context.connection.canStep || running);
    if (widgets::Button("##step", widgets::ButtonKind::Default, widgets::Icon::Step, true)) {
        context.client.step(kStepNs);
    }
    ImGui::SetItemTooltip("Step 10 ms");
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    next();
    widgets::Readout("sim t", time);
    next();
    widgets::Readout("RTF", rate);
    next();
    ImGui::SetCursorPosY(y + px(4.0F));
    widgets::Pill(pill, widgets::PillKind::Info);
    if (session.testPassive) {
        ImGui::SetItemTooltip("passive: the run already has a robot");
    } else if (session.testIntoRun) {
        ImGui::SetItemTooltip("In the connected run; Back to Build removes it");
    }
}

/// The Robot strip (SPEC "Strip under the workspace bar"), in place of the domain chips.
void drawRobotStrip(PanelContext& context) {
    BuilderSession& session = *context.builder;
    const builder::Document& document = session.document();
    const builder::PartCounts counts = builder::countParts(document, &session.catalog);
    centreInBar(ImGui::GetTextLineHeight());
    widgets::Caption("Robot");
    const ImVec4 rigid = widgets::domainColor("rigid_body");
    const ImVec4 sensors = widgets::domainColor("sensors");
    const ImVec4 neutral(0.0F, 0.0F, 0.0F, 0.0F);
    const auto chip = [&](const char* id, const ImVec4& color, const std::string& name,
                          const std::string& detail) {
        ImGui::SameLine(0.0F, px(6.0F));
        centreInBar(px(22.0F));
        return widgets::DomainChip(id, color, name.c_str(), detail.c_str(), widgets::ChipState::On);
    };
    const auto plural = [](int n, const char* word) {
        return std::to_string(n) + " " + word + (n == 1 ? "" : "s");
    };
    (void)chip("##links", rigid, plural(counts.links, "link"),
               counts.roots == 1 ? "1 root" : std::to_string(counts.roots) + " roots");
    std::string types;
    const auto add = [&](int n, const char* word) {
        if (n > 0) {
            types += (types.empty() ? "" : " · ") + std::to_string(n) + " " + word;
        }
    };
    add(counts.revolute, "rev");
    add(counts.continuous, "cont");
    add(counts.prismatic, "prism");
    add(counts.fixed, "fixed");
    (void)chip("##joints", rigid, plural(counts.joints, "joint"), types);
    std::string kinds = plural(counts.types, "type");
    kinds += " · " + plural(counts.kinds, "kind");
    (void)chip("##sensors", sensors, plural(counts.sensors, "sensor"),
               counts.sensors == 0 ? "" : kinds);
    // Mass and COM at zero preview, in the root link's frame (RFC 0016 §4).
    const builder::MassProperties mass = builder::robotMass(document);
    char text[64];
    std::snprintf(text, sizeof(text), "%.2f kg", mass.mass);
    (void)chip("##mass", neutral, "mass", text);
    std::snprintf(text, sizeof(text), "%.3f · %.3f · %.3f m", mass.com[0], mass.com[1],
                  mass.com[2]);
    (void)chip("##com", neutral, "COM", text);

    // Right: what Build mode does, the validity, Check loads….
    const char* info = session.mode == BuilderSession::Mode::Build
                           ? "Build · no simulation · sensors not rendered · Lit"
                       : session.testIntoRun ? "Test · the robot runs in the connected run"
                                             : "Test · the robot runs in its own simulator";
    // QA F9: the pill counts what the Problems tab lists (its title's number), by level.
    const BuilderSession::Problems& problems = session.problems;
    const int errors = problems.errors() + (session.catalogError.empty() ? 0 : 1);
    int total = session.catalogError.empty() ? 0 : 1;
    int warnings = 0;
    int hints = 0;
    int fileErrors = 0;
    for (const auto* list :
         {&problems.builder, &problems.loader, &problems.check, &problems.files}) {
        for (const builder::Problem& problem : *list) {
            ++total;
            warnings += problem.level == builder::Level::Warning ? 1 : 0;
            hints += problem.level == builder::Level::Hint ? 1 : 0;
            fileErrors += list == &problems.files && problem.level == builder::Level::Error ? 1 : 0;
        }
    }
    char validity[96];
    if (errors > 0) {
        std::snprintf(validity, sizeof(validity), "%d problem%s", total, total == 1 ? "" : "s");
    } else {
        std::string text = "URDF valid";
        const auto add = [&](int n, const char* what) {
            if (n > 0) {
                text += " · " + std::to_string(n) + " " + what + (n == 1 ? "" : "s");
            }
        };
        add(fileErrors, "file error");
        add(warnings, "warning");
        add(hints, "hint");
        if (total == 0) {
            text += " · 0 hints";
        }
        std::snprintf(validity, sizeof(validity), "%s", text.c_str());
    }
    const bool check = canCheckLoads() && session.mode == BuilderSession::Mode::Build;
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const float button = check ? ImGui::CalcTextSize("Check loads…").x + px(16.0F) : 0.0F;
    ImGui::PopFont();
    ImGui::SameLine();
    widgets::AlignRight(widgets::PillWidth(info) + px(6.0F) + widgets::PillWidth(validity) +
                        (check ? px(8.0F) + button : 0.0F));
    centreInBar(ImGui::GetTextLineHeight());
    widgets::Pill(info, widgets::PillKind::Info);
    ImGui::SameLine(0.0F, px(6.0F));
    widgets::Pill(validity, errors > 0 ? widgets::PillKind::Error : widgets::PillKind::Ok);
    if (ImGui::IsItemClicked()) {
        ImGui::SetWindowFocus(builder_windows::kProblems);
    }
    if (errors > 0) {
        ImGui::SetItemTooltip("Open Problems");
    }
    if (check) {
        ImGui::SameLine(0.0F, px(8.0F));
        centreInBar(px(22.0F));
        if (widgets::Button("Check loads…", widgets::ButtonKind::Ghost)) {
            session.checkRequested = true;
        }
        ImGui::SetItemTooltip("What each joint must hold against gravity (lagrange check)");
    }
}

/// The Builder's keys (SPEC "Modes"): Ctrl Z, Ctrl Shift Z / Ctrl Y, Ctrl S; Esc while testing.
void builderShortcuts(PanelContext& context) {
    BuilderSession& session = *context.builder;
    if (ImGui::GetIO().WantTextInput) {
        return;  // a field's own undo
    }
    if (session.mode == BuilderSession::Mode::Build) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z)) {
            (void)session.history.undo();
        }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey_Z) ||
            ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y)) {
            (void)session.history.redo();
        }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_S)) {
            session.saveRequested = true;
            session.savePath.clear();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) && !ImGui::IsAnyItemActive() &&
            ImGui::GetTopMostPopupModal() == nullptr) {
            builder_ui::deleteSelection(session);
        }
    } else if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        session.backRequested = true;
    }
}

void drawWorkspaceBar(PanelContext& context, ImGuiViewport* viewport) {
    // Design (.tb): 40 px, bg1; the workspace tabs, then on the right, after a rule, the run
    // controls and readouts.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(6.0F), 0.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    const bool open =
        ImGui::BeginViewportSideBar("##workspace-bar", viewport, ImGuiDir_Up, px(40.0F), kBarFlags);
    ImGui::PopStyleVar(2);
    if (open) {
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        for (const WorkspaceEntry& entry : kWorkspaces) {
            if (entry.workspace == Workspace::Builder) {
                // Design (12-builder: .ws.sepl): a 1 px rule sets the Builder apart.
                const float x = ImGui::GetCursorScreenPos().x;
                const float top = ImGui::GetWindowPos().y;
                ImGui::GetWindowDrawList()->AddLine(ImVec2(x, top),
                                                    ImVec2(x, top + ImGui::GetWindowHeight()),
                                                    ImGui::ColorConvertFloat4ToU32(c::line));
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(1.0F));
            }
            if (widgets::BarTab(entry.label, entry.icon, *context.workspace == entry.workspace,
                                entry.dockspace != nullptr)) {
                *context.workspace = entry.workspace;
            }
            ImGui::SameLine();
        }
        ImGui::PopStyleVar();

        if (*context.workspace == Workspace::Builder && !context.launcher->active) {
            // The Builder's own controls replace the run controls (SPEC "Modes").
            if (context.builder->mode == BuilderSession::Mode::Build) {
                drawBuildControls(context);
            } else {
                drawTestControls(context);
            }
            bottomRule();
            ImGui::End();
            return;
        }

        const bool ready = context.connection.state == ConnectionState::Ready;
        const bool running = context.status.runState == RunState::Running;
        char time[32];
        format::clock(context.status.simTimeNs, time, sizeof(time));
        char rate[24];
        std::snprintf(rate, sizeof(rate), "%.2f×", context.status.realTimeFactor);
        char target[24];
        std::snprintf(target, sizeof(target), "/ %.2f", context.status.realTimeFactorTarget);

        // Width of the control group, to right-align it: three buttons, two readouts, Record.
        const float gap = px(4.0F);
        ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
        const float playWidth =
            px(20.0F) + px(14.0F) + px(6.0F) + ImGui::CalcTextSize(running ? "Pause" : "Play").x;
        const float recordWidth = px(20.0F) + px(8.0F) + px(6.0F) + ImGui::CalcTextSize("Record").x;
        ImGui::PopFont();
        const float width = px(26.0F) * 2.0F + playWidth + recordWidth +
                            widgets::ReadoutWidth("sim t", time) +
                            widgets::ReadoutWidth("RTF", rate, target) + gap * 5.0F + px(10.0F);
        ImGui::SameLine();
        widgets::AlignRight(width + px(4.0F));
        const ImVec2 rule = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(rule.x, ImGui::GetWindowPos().y),
            ImVec2(rule.x, ImGui::GetWindowPos().y + ImGui::GetWindowHeight()),
            ImGui::ColorConvertFloat4ToU32(c::line));
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(10.0F));
        centreInBar(px(26.0F));
        const float y = ImGui::GetCursorPosY();
        const auto next = [&] {
            ImGui::SameLine(0.0F, gap);
            ImGui::SetCursorPosY(y);
        };

        ImGui::BeginDisabled(!ready);
        if (widgets::Button("##reset", widgets::ButtonKind::Default, widgets::Icon::Reset, true)) {
            resetToStart(context);
        }
        ImGui::SetItemTooltip("Reset the run to its start");
        ImGui::EndDisabled();
        next();
        ImGui::BeginDisabled(!ready);
        if (widgets::Button(running ? "Pause" : "Play", widgets::ButtonKind::Primary,
                            running ? widgets::Icon::Pause : widgets::Icon::Play, true)) {
            if (running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        ImGui::EndDisabled();
        next();
        ImGui::BeginDisabled(!ready || !context.connection.canStep || running);
        if (widgets::Button("##step", widgets::ButtonKind::Default, widgets::Icon::Step, true)) {
            context.client.step(kStepNs);
        }
        ImGui::SetItemTooltip("Step 10 ms");
        ImGui::EndDisabled();
        next();
        widgets::Readout("sim t", time);
        next();
        widgets::Readout("RTF", rate, target);
        next();
        // Record (design: its red dot means recording): a live run, on the server's machine.
        const bool canRecord = ready && !context.connection.canSeek;  // not a replay
        const bool recording = !context.status.recordingPath.empty();
        ImGui::BeginDisabled(!canRecord);
        const ImVec2 record = ImGui::GetCursorScreenPos();
        if (widgets::Button("     Record", widgets::ButtonKind::Default, widgets::Icon::Count,
                            true)) {
            context.client.setRecording(!recording);
        }
        const ImVec4 dot = recording ? c::error : c::t3;
        ImGui::GetWindowDrawList()->AddCircleFilled(
            ImVec2(record.x + px(14.0F), record.y + px(13.0F)), px(4.0F),
            ImGui::ColorConvertFloat4ToU32(
                ImVec4(dot.x, dot.y, dot.z, canRecord ? 1.0F : ImGui::GetStyle().DisabledAlpha)));
        ImGui::EndDisabled();
        if (!canRecord) {
            ImGui::SetItemTooltip(ready ? "Recording is not available from a replay"
                                        : "Recording needs a running simulation");
        } else if (recording) {
            ImGui::SetItemTooltip("Recording to %s (on the server). Click or Ctrl+R to stop",
                                  context.status.recordingPath.c_str());
        } else {
            ImGui::SetItemTooltip("Record the run to an MCAP file on the server (Ctrl+R)");
        }
        bottomRule();
    }
    ImGui::End();
}

void drawDomainStrip(PanelContext& context, ImGuiViewport* viewport) {
    // Design (.ds): 32 px on bg0; an 11 px label, the chips 6 px apart, a pill on the right.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(8.0F), px(5.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, c::bg0);
    const bool open =
        ImGui::BeginViewportSideBar("##domain-strip", viewport, ImGuiDir_Up, px(32.0F), kBarFlags);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
    if (open) {
        if (*context.workspace == Workspace::Builder) {
            drawRobotStrip(context);  // the Robot strip in place of the domains (SPEC)
            bottomRule();
            ImGui::End();
            return;
        }
        ProfileSession& session = *context.profileSession;
        centreInBar(ImGui::GetTextLineHeight());
        widgets::Caption("Domains");
        if (session.domains().empty()) {
            ImGui::SameLine(0.0F, px(10.0F));
            widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                       ? "waiting for the server's profile"
                                       : "not connected");
        }
        for (const DomainView& view : session.domains()) {
            if (!format::isPhysicsDomain(view.name)) {
                continue;  // additional modules are listed in the Domains workspace
            }
            char detail[96];
            format::domainDetail(view, detail, sizeof(detail));
            widgets::ChipState state = widgets::ChipState::On;
            if (session.domainHasError(view.name)) {
                state = widgets::ChipState::Error;
            } else if (view.pending) {
                state = widgets::ChipState::Pending;
            } else if (!view.enabled) {
                state = widgets::ChipState::Substitute;
            }
            ImGui::SameLine(0.0F, px(6.0F));
            centreInBar(px(22.0F));
            const std::string_view label = widgets::domainShortLabel(view.name);
            char name[32];
            std::snprintf(name, sizeof(name), "%.*s", static_cast<int>(label.size()), label.data());
            if (widgets::DomainChip(view.name.c_str(), widgets::domainColor(view.name), name,
                                    detail, state)) {
                *context.workspace = Workspace::Domains;
                session.selectedDomain = view.name;
            }
        }

        // On the right: what the profile's state is, and the way to edit it.
        char text[64] = "";
        widgets::PillKind kind = widgets::PillKind::Ok;
        const std::size_t changes = session.draft().changedLineCount();
        if (changes != 0) {
            std::snprintf(text, sizeof(text), "%zu %s · not applied", changes,
                          changes == 1 ? "change" : "changes");
            kind = widgets::PillKind::Accent;
        } else if (session.validity() == ProfileSession::Validity::Valid) {
            std::snprintf(text, sizeof(text), "profile valid");
        } else if (session.validity() == ProfileSession::Validity::Invalid) {
            std::snprintf(text, sizeof(text), "profile not valid");
            kind = widgets::PillKind::Error;
        }
        ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
        const float button = ImGui::CalcTextSize("Edit profile").x + px(16.0F);
        ImGui::PopFont();
        const float pill = *text != '\0' ? widgets::PillWidth(text) + px(8.0F) : 0.0F;
        ImGui::SameLine();
        widgets::AlignRight(pill + button);
        if (*text != '\0') {
            centreInBar(ImGui::GetTextLineHeight());
            widgets::Pill(text, kind);
            ImGui::SameLine(0.0F, px(8.0F));
        }
        centreInBar(px(22.0F));
        if (widgets::Button("Edit profile", widgets::ButtonKind::Ghost)) {
            *context.workspace = Workspace::Domains;
        }
        bottomRule();
    }
    ImGui::End();
}

void statusItem(const char* key, const ImVec4& color, const char* format, ...) IM_FMTARGS(3);
void statusItem(const char* key, const ImVec4& color, const char* format, ...) {
    // Design (.sb .it): 11 px; key in t3, value in mono.
    if (key != nullptr) {
        widgets::Caption("%s", key);
        ImGui::SameLine(0.0F, px(6.0F));
    }
    char text[200];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
    ImGui::TextColored(color, "%s", text);
    ImGui::PopFont();
}

/// The version in the package.xml installed beside `program` (<prefix>/lib/<pkg>/<program> →
/// <prefix>/share/<pkg>/package.xml), or empty. Read once per program.
std::string installedVersion(const std::string& program) {
    static std::string cachedFor;
    static std::string cached;
    if (program == cachedFor) {
        return cached;
    }
    cachedFor = program;
    cached.clear();
    const std::filesystem::path binary(program);
    const std::filesystem::path package = binary.parent_path();
    const std::filesystem::path manifest =
        package.parent_path().parent_path() / "share" / package.filename() / "package.xml";
    if (std::FILE* file = std::fopen(manifest.c_str(), "r")) {
        char line[512];
        while (std::fgets(line, sizeof(line), file) != nullptr) {
            const char* open = std::strstr(line, "<version>");
            const char* close = open != nullptr ? std::strstr(open, "</version>") : nullptr;
            if (close != nullptr) {
                cached.assign(open + 9, close);
                break;
            }
        }
        std::fclose(file);
    }
    return cached;
}

/// "lyrical · domain 0 · fastrtps" from the environment ROS 2 programs read: ROS_DISTRO,
/// ROS_DOMAIN_ID (0 when unset, ROS 2's default) and RMW_IMPLEMENTATION (rmw_fastrtps_cpp when
/// unset, Lyrical's default, CLAUDE.md §1a). `domain` overrides the domain (a test server's).
/// Empty when no ROS 2 is sourced.
std::string rosLine(const std::string& domain) {
    const char* distro = std::getenv("ROS_DISTRO");
    if (distro == nullptr || *distro == '\0') {
        return {};
    }
    const char* id = std::getenv("ROS_DOMAIN_ID");
    std::string rmw = std::getenv("RMW_IMPLEMENTATION") != nullptr
                          ? std::getenv("RMW_IMPLEMENTATION")
                          : "rmw_fastrtps_cpp";
    if (rmw.starts_with("rmw_")) {
        rmw = rmw.substr(4);
    }
    if (rmw.ends_with("_cpp")) {
        rmw.resize(rmw.size() - 4);
    }
    const std::string shown =
        !domain.empty() ? domain : (id != nullptr && *id != '\0' ? std::string(id) : "0");
    return std::string(distro) + " · domain " + shown + " · " + rmw;
}

void drawStatusBar(const PanelContext& context, ImGuiViewport* viewport) {
    // Design (.sb): 24 px, bg1, 11 px text, items 16 px apart.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(10.0F), px(4.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    const bool open =
        ImGui::BeginViewportSideBar("##status-bar", viewport, ImGuiDir_Down, px(24.0F), kBarFlags);
    ImGui::PopStyleVar(2);
    if (open) {
        const ImVec2 min = ImGui::GetWindowPos();
        ImGui::GetWindowDrawList()->AddLine(min, ImVec2(min.x + ImGui::GetWindowWidth(), min.y),
                                            ImGui::ColorConvertFloat4ToU32(c::line));
        const float gap = px(16.0F);
        const ConnectionState state = context.connection.state;
        const ImVec4 dot = state == ConnectionState::Ready
                               ? c::ok
                               : (state == ConnectionState::Refused ? c::error : c::warn);
        widgets::Dot(dot);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        const bool builderShown = *context.workspace == Workspace::Builder;
        if (builderShown && state != ConnectionState::Ready) {
            // The Builder works without a server (SPEC state 5).
            ImGui::TextColored(c::t1, "Not connected · building offline");
        } else {
            ImGui::TextColored(c::t1, "%s",
                               state == ConnectionState::Ready ? "Connected" : toString(state));
        }
        ImGui::PopFont();
        // Building offline: the run readouts would be the last server's (a Test's after Back).
        const bool offline = builderShown && state != ConnectionState::Ready;
        if (offline) {
            // The server a Test would start (SPEC state 5; the Launcher's wording when none).
            ImGui::SameLine(0.0F, gap);
            const std::string& program = context.launcher->localServer;
            if (program.empty()) {
                statusItem("test server", c::t2, "lagrange_server is not installed");
            } else {
                const std::string version = installedVersion(program);
                statusItem("test server", c::t2, "lagrange_server%s%s · this machine",
                           version.empty() ? "" : " ", version.c_str());
            }
        }
        if (builderShown && context.builder->autosaveTime > 0.0) {
            ImGui::SameLine(0.0F, gap);
            statusItem("autosave", c::t1, "%.0f s ago",
                       ImGui::GetTime() - context.builder->autosaveTime);
        }
        if (!offline && !context.connection.serverName.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem("server", c::t2, "%s", context.connection.serverName.c_str());
        }
        if (!offline && !context.connection.message.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem(nullptr, c::warn, "%s", context.connection.message.c_str());
        }
        if (!offline) {
            ImGui::SameLine(0.0F, gap);
            statusItem("signals", c::t1, "%u", context.status.signalCount);
            ImGui::SameLine(0.0F, gap);
            statusItem("deltas", c::t1, "%llu",
                       static_cast<unsigned long long>(context.stats.deltas));
            ImGui::SameLine(0.0F, gap);
            statusItem("dropped", context.status.droppedDeltas == 0 ? c::t1 : c::warn, "%llu",
                       static_cast<unsigned long long>(context.status.droppedDeltas));
        }
        // The latest thing to tell the user: a file that loaded, a command that failed.
        if (!context.notice.text.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem(nullptr, context.notice.error ? c::error : c::info, "%s",
                       context.notice.text.c_str());
        }

        char tier[32];
        std::snprintf(tier, sizeof(tier), "%s", context.tierName.c_str());
        if (char* detail = std::strstr(tier, " (")) {
            *detail = '\0';
        }
        char gpu[160];
        std::snprintf(gpu, sizeof(gpu), "%s · %s", context.deviceName.c_str(), tier);
        const float fps = context.frameMs > 0.0F ? 1000.0F / context.frameMs : 0.0F;
        char viewer[64];
        std::snprintf(viewer, sizeof(viewer), "%.1f ms · %.0f fps",
                      static_cast<double>(context.frameMs), static_cast<double>(fps));
        // The ROS 2 a run would join (ui-conventions §1: distro, domain and RMW); a Test's
        // server has a domain of its own.
        const bool testing = builderShown && context.builder->mode != BuilderSession::Mode::Build;
        std::string ros = rosLine(testing ? context.builder->testRosDomain : std::string{});
        if (!ros.empty() && testing && !context.builder->testIntoRun &&
            context.builder->testRosDomain.empty()) {
            ros = "off for this test (--no-ros)";  // BuilderHost started it without ROS
        }
        const theme::Fonts& fonts = theme::fonts();
        const float size = px(theme::kCaptionSize);
        float width = fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, "GPU").x +
                      fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, "viewer").x +
                      fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, gpu).x +
                      fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, viewer).x + px(6.0F) * 2.0F +
                      gap;
        if (!ros.empty()) {
            width += fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, "ROS 2").x +
                     fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, ros.c_str()).x + px(6.0F) + gap;
        }
        ImGui::SameLine();
        widgets::AlignRight(width);
        statusItem("GPU", c::t2, "%s", gpu);
        ImGui::SameLine(0.0F, gap);
        statusItem("viewer", c::t2, "%s", viewer);
        if (!ros.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem("ROS 2", c::t2, "%s", ros.c_str());
        }
    }
    ImGui::End();
}

/// One host window fills what the bars left over. It holds a dock space per workspace; the
/// ones not shown are kept alive, so their panels stay docked where the user put them.
/// Returns true when a default layout was built this frame.
bool drawDockSpaces(Workspace shown, ImGuiViewport* viewport, bool reset) {
    bool built = false;
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(4.0F), px(4.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, c::bg0);
    ImGui::Begin("##dock-host", nullptr, kBarFlags | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
    const ImVec2 size = ImGui::GetContentRegionAvail();
    for (const WorkspaceEntry& entry : kWorkspaces) {
        if (entry.dockspace == nullptr) {
            continue;
        }
        const ImGuiID id = ImGui::GetID(entry.dockspace);
        // A layout made in the first frames is made again once the bars have taken their
        // room (ImGui applies a side bar's room to the work area a frame late): the dock
        // space shrinks then, and a split away from the central node (the Builder's 212 px
        // Add) would shrink with it instead of keeping its design size.
        struct Made {
            ImVec2 size;
            int frame{0};
        };
        static std::unordered_map<ImGuiID, Made> made;
        constexpr int kSettleFrames = 3;
        const auto found = made.find(id);
        const bool unsettled = found != made.end() &&
                               ImGui::GetFrameCount() - found->second.frame < kSettleFrames &&
                               (found->second.size.x != size.x || found->second.size.y != size.y);
        // Lay out when there is no saved layout for this dock space, or on request.
        if (ImGui::DockBuilderGetNode(id) == nullptr || (reset && entry.workspace == shown) ||
            unsettled) {
            buildLayout(entry.workspace, id, size);
            made[id] = Made{size, ImGui::GetFrameCount()};
            built = true;
        }
        if (entry.workspace == shown) {
            ImGui::DockSpace(id, size);
        } else {
            ImGui::DockSpace(id, size, ImGuiDockNodeFlags_KeepAliveOnly);
        }
    }
    const WorkspaceEntry& entry = entryOf(shown);
    if (entry.dockspace == nullptr) {
        // A workspace of the design that has no panels yet: say so, and where its design is.
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float icon = px(40.0F);
        const ImVec2 centre(origin.x + size.x * 0.5F, origin.y + size.y * 0.5F);
        widgets::DrawIcon(ImGui::GetWindowDrawList(), entry.icon,
                          ImVec2(centre.x - icon * 0.5F, centre.y - icon - px(18.0F)), icon,
                          ImGui::ColorConvertFloat4ToU32(c::lineStrong),
                          ImGui::ColorConvertFloat4ToU32(c::bg0));
        char title[64];
        std::snprintf(title, sizeof(title), "%s is not built yet", entry.label);
        ImGui::PushFont(theme::fonts().sansSemiBold, theme::kTitleSize);
        const float titleWidth = ImGui::CalcTextSize(title).x;
        ImGui::SetCursorScreenPos(ImVec2(centre.x - titleWidth * 0.5F, centre.y));
        ImGui::TextColored(c::t2, "%s", title);
        ImGui::PopFont();
        char where[96];
        std::snprintf(where, sizeof(where), "design: docs/design/ui/%s", entry.design);
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        const float whereWidth = ImGui::CalcTextSize(where).x;
        ImGui::SetCursorScreenPos(ImVec2(centre.x - whereWidth * 0.5F, centre.y + px(24.0F)));
        ImGui::TextColored(c::t3, "%s", where);
        ImGui::PopFont();
    }
    ImGui::End();
    return built;
}

}  // namespace

const char* toString(Workspace workspace) {
    for (const WorkspaceEntry& entry : kWorkspaces) {
        if (entry.workspace == workspace) {
            return entry.label;
        }
    }
    return "";
}

std::optional<Workspace> parseWorkspace(std::string_view name) {
    for (const WorkspaceEntry& entry : kWorkspaces) {
        const std::string_view label = entry.label;
        if (std::ranges::equal(name, label, [](char a, char b) {
                return std::tolower(static_cast<unsigned char>(a)) ==
                       std::tolower(static_cast<unsigned char>(b));
            })) {
            return entry.workspace;
        }
    }
    return std::nullopt;
}

std::vector<std::unique_ptr<Panel>> createDefaultPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(createViewportPanel());
    panels.push_back(createScenePanel());
    panels.push_back(createInspectorPanel());
    panels.push_back(createTimelinePanel());
    panels.push_back(createBookmarksPanel());
    panels.push_back(createConsolePanel());
    panels.push_back(createProblemsPanel());
    panels.push_back(createScopePanel());
    panels.push_back(createSignalsPanel());
    panels.push_back(createCursorsPanel());
    panels.push_back(createStatisticsPanel());
    for (std::unique_ptr<Panel>& panel : createElectricalPanels()) {
        panels.push_back(std::move(panel));
    }
    for (std::unique_ptr<Panel>& panel : createSensorsPanels()) {
        panels.push_back(std::move(panel));
    }
    for (std::unique_ptr<Panel>& panel : createPerformancePanels()) {
        panels.push_back(std::move(panel));
    }
    for (std::unique_ptr<Panel>& panel : createReplayPanels()) {
        panels.push_back(std::move(panel));
    }
    panels.push_back(createReplayBookmarksPanel());
    panels.push_back(createReplayTimelinePanel());
    for (auto* create : {createEnvironmentPanels, createRos2Panels, createWorldPanels}) {
        for (std::unique_ptr<Panel>& panel : create()) {
            panels.push_back(std::move(panel));
        }
    }
    panels.push_back(createRosConsolePanel());
    panels.push_back(createElectricalInspectorPanel());
    panels.push_back(createDomainsPanel());
    for (std::unique_ptr<Panel>& panel : createWiringPanels()) {
        panels.push_back(std::move(panel));
    }
    panels.push_back(createProfilePanel());
    for (std::unique_ptr<Panel>& panel : createBuilderPanels()) {
        panels.push_back(std::move(panel));
    }
    return panels;
}

void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context) {
    if (context.launcher->active) {
        drawLauncher(context);  // the first screen: nothing else is shown until connected
        return;
    }
    context.profileSession->update(context.client,
                                   context.connection.state == ConnectionState::Ready,
                                   context.stats.connects, ImGui::GetTime());

    context.scopeSession->update();  // every frame, so a trigger fires in any workspace

    if (context.canOpenFiles && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) {
        context.openFileRequested = true;
    }
    for (const WorkspaceEntry& entry : kWorkspaces) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | entry.key)) {
            *context.workspace = entry.workspace;
        }
    }
    // Run shortcuts of the design, when no text field has the keyboard. Not while building:
    // a connected run is left alone there (RFC 0016 open question 7, R2).
    const bool building = *context.workspace == Workspace::Builder &&
                          context.builder->mode == BuilderSession::Mode::Build;
    if (!ImGui::GetIO().WantTextInput && context.connection.state == ConnectionState::Ready &&
        !building) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            if (context.status.runState == RunState::Running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_R) && !context.connection.canSeek) {
            context.client.setRecording(context.status.recordingPath.empty());
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Period, false) && context.connection.canStep &&
            context.status.runState != RunState::Running) {
            context.client.step(kStepNs);
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !ImGui::GetIO().WantTextInput) {
        context.selectedPrim = kNone;
    }
    if (*context.workspace == Workspace::Builder) {
        builderShortcuts(context);
    }

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    bool resetLayout = false;
    context.resetLayout = &resetLayout;
    drawMenuBar(context);
    drawWorkspaceBar(context, viewport);
    drawDomainStrip(context, viewport);
    drawStatusBar(context, viewport);
    const Workspace shown = *context.workspace;  // a click this frame takes effect next frame
    const bool built = drawDockSpaces(shown, viewport, resetLayout);
    for (const std::unique_ptr<Panel>& panel : panels) {
        if (panel->workspace() == shown) {
            panel->draw(context);
        }
    }
    if (built && shown == Workspace::Scene) {
        // In a fresh layout the first tab of a group is the one in front.
        ImGui::SetWindowFocus("Console");
        ImGui::SetWindowFocus("Timeline");
        ImGui::SetWindowFocus("Viewport");
    }
    if (built && shown == Workspace::Domains) {
        ImGui::SetWindowFocus("Wiring");
    }
    if (built && shown == Workspace::Electrical) {
        ImGui::SetWindowFocus("Schematic");
    }
    if (built && shown == Workspace::Builder) {
        // The first tab of each group in front (SPEC: Robot, Add, Properties, Joints, catalog).
        for (const char* window :
             {builder_windows::kRobot, builder_windows::kAdd, builder_windows::kProperties,
              builder_windows::kJoints, builder_windows::kCatalog, builder_windows::kView}) {
            ImGui::SetWindowFocus(window);
        }
    }
    // A panel asked for on the command line, in front once it exists.
    if (context.focusPanel != nullptr && !context.focusPanel->empty() &&
        ImGui::FindWindowByName(context.focusPanel->c_str()) != nullptr) {
        ImGui::SetWindowFocus(context.focusPanel->c_str());
        context.focusPanel->clear();
    }
}

}  // namespace lagrange::viewer
