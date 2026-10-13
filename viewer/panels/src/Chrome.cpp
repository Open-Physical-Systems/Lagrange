// The application chrome shared by every workspace: menu bar, workspace bar, domain strip,
// dock space and status bar (docs/design/ui/shared/ui-conventions.md §1).
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
#include <cstring>
#include <imgui_internal.h>  // DockBuilder and BeginViewportSideBar

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

// The ten workspaces of the design, in its order, with its shortcuts (Ctrl 1 ... Ctrl 0).
// The dock space names carry a layout version: change it when a default layout changes, so
// that a layout saved by an older build is not applied to the new panels.
constexpr std::array<WorkspaceEntry, 10> kWorkspaces{{
    {Workspace::Scene, "Scene", widgets::Icon::Scene, "lagrange-dock-scene-4", ImGuiKey_1, "Ctrl+1",
     "02-scene"},
    {Workspace::Domains, "Domains", widgets::Icon::Domains, "lagrange-dock-domains-3", ImGuiKey_2,
     "Ctrl+2", "03-domains"},
    {Workspace::Electrical, "Electrical", widgets::Icon::Electrical, "lagrange-dock-electrical-1",
     ImGuiKey_3, "Ctrl+3", "04-electrical"},
    {Workspace::Signals, "Signals", widgets::Icon::Signals, "lagrange-dock-signals-3", ImGuiKey_4,
     "Ctrl+4", "05-signals"},
    {Workspace::Sensors, "Sensors", widgets::Icon::Sensors, "lagrange-dock-sensors-1", ImGuiKey_5,
     "Ctrl+5", "06-sensors"},
    {Workspace::Environment, "Environment", widgets::Icon::Environment, nullptr, ImGuiKey_6,
     "Ctrl+6", "07-environment"},
    {Workspace::Replay, "Replay", widgets::Icon::Replay, nullptr, ImGuiKey_7, "Ctrl+7",
     "08-replay"},
    {Workspace::Ros2, "ROS 2", widgets::Icon::Ros2, nullptr, ImGuiKey_8, "Ctrl+8", "09-ros2"},
    {Workspace::World, "World", widgets::Icon::World, nullptr, ImGuiKey_9, "Ctrl+9", "10-world"},
    {Workspace::Performance, "Performance", widgets::Icon::Performance,
     "lagrange-dock-performance-1", ImGuiKey_0, "Ctrl+0", "11-performance"},
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
        ImGui::DockBuilderDockWindow("Scopes", scopes);
        ImGui::DockBuilderDockWindow("###InspectorElectrical", right);
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
    } else if (workspace == Workspace::Sensors) {
        // Design: sensor list 260 px left, Degradation 320 px right, sensor views between.
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Left, fraction(260.0F, size.x), nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            centre, ImGuiDir_Right, fraction(320.0F, size.x - px(260.0F)), nullptr, &centre);
        const ImGuiID imu =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.40F, nullptr, &centre);
        ImGui::DockBuilderDockWindow("###SensorList", left);
        ImGui::DockBuilderDockWindow("###Lidar", centre);
        ImGui::DockBuilderDockWindow("###Imu", imu);
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

void contextItem(const char* key, const char* value) {
    ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
    ImGui::TextColored(c::t3, "%s", key);
    ImGui::PopFont();
    ImGui::SameLine(0.0F, px(5.0F));
    ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
    ImGui::TextColored(c::t2, "%s", value);
    ImGui::PopFont();
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
        if (ImGui::MenuItem("Open robot or recording...", "Ctrl+O", false, context.canOpenFiles)) {
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
        if (ImGui::MenuItem("Reset to start", nullptr, false,
                            ready && context.connection.canSeek)) {
            context.client.seek(context.status.replayBeginNs);
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
    const char* stage = context.scene ? context.scene->statics->stagePath.c_str() : "";
    const char* robot = context.scene && !context.scene->statics->prims.empty()
                            ? context.scene->statics->prims.front().name.c_str()
                            : "";
    char profile[48] = "";
    if (context.profileSession->profile()) {
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
            if (widgets::BarTab(entry.label, entry.icon, *context.workspace == entry.workspace,
                                entry.dockspace != nullptr)) {
                *context.workspace = entry.workspace;
            }
            ImGui::SameLine();
        }
        ImGui::PopStyleVar();

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

        ImGui::BeginDisabled(!ready || !context.connection.canSeek);
        if (widgets::Button("##reset", widgets::ButtonKind::Default, widgets::Icon::Reset, true)) {
            context.client.seek(context.status.replayBeginNs);
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
        // Recording needs a simulation server; the mock server replays a recording.
        ImGui::BeginDisabled(true);
        const ImVec2 record = ImGui::GetCursorScreenPos();
        widgets::Button("     Record", widgets::ButtonKind::Default, widgets::Icon::Count, true);
        ImGui::GetWindowDrawList()->AddCircleFilled(
            ImVec2(record.x + px(14.0F), record.y + px(13.0F)), px(4.0F),
            ImGui::ColorConvertFloat4ToU32(
                ImVec4(c::error.x, c::error.y, c::error.z, ImGui::GetStyle().DisabledAlpha)));
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("Recording is not available from a replay");
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
        ImGui::TextColored(c::t1, "%s",
                           state == ConnectionState::Ready ? "Connected" : toString(state));
        ImGui::PopFont();
        if (!context.connection.serverName.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem("server", c::t2, "%s", context.connection.serverName.c_str());
        }
        if (!context.connection.message.empty()) {
            ImGui::SameLine(0.0F, gap);
            statusItem(nullptr, c::warn, "%s", context.connection.message.c_str());
        }
        ImGui::SameLine(0.0F, gap);
        statusItem("signals", c::t1, "%u", context.status.signalCount);
        ImGui::SameLine(0.0F, gap);
        statusItem("deltas", c::t1, "%llu", static_cast<unsigned long long>(context.stats.deltas));
        ImGui::SameLine(0.0F, gap);
        statusItem("dropped", context.status.droppedDeltas == 0 ? c::t1 : c::warn, "%llu",
                   static_cast<unsigned long long>(context.status.droppedDeltas));
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
        const theme::Fonts& fonts = theme::fonts();
        const float size = px(theme::kCaptionSize);
        const float width = fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, "GPU").x +
                            fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, "viewer").x +
                            fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, gpu).x +
                            fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, viewer).x +
                            px(6.0F) * 2.0F + gap;
        ImGui::SameLine();
        widgets::AlignRight(width);
        statusItem("GPU", c::t2, "%s", gpu);
        ImGui::SameLine(0.0F, gap);
        statusItem("viewer", c::t2, "%s", viewer);
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
        // Lay out when there is no saved layout for this dock space, or on request.
        if (ImGui::DockBuilderGetNode(id) == nullptr || (reset && entry.workspace == shown)) {
            buildLayout(entry.workspace, id, size);
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
    panels.push_back(createElectricalInspectorPanel());
    panels.push_back(createDomainsPanel());
    for (std::unique_ptr<Panel>& panel : createWiringPanels()) {
        panels.push_back(std::move(panel));
    }
    panels.push_back(createProfilePanel());
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
    // Run shortcuts of the design, when no text field has the keyboard.
    if (!ImGui::GetIO().WantTextInput && context.connection.state == ConnectionState::Ready) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            if (context.status.runState == RunState::Running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Period, false) && context.connection.canStep &&
            context.status.runState != RunState::Running) {
            context.client.step(kStepNs);
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !ImGui::GetIO().WantTextInput) {
        context.selectedPrim = kNone;
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
}

}  // namespace lagrange::viewer
