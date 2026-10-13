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
#include <cstdio>
#include <imgui_internal.h>  // DockBuilder and BeginViewportSideBar

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

struct WorkspaceEntry {
    Workspace workspace;
    const char* label;
    const char* dockspace;
    ImGuiKey key;
    const char* shortcut;
};

constexpr std::array<WorkspaceEntry, 2> kWorkspaces{{
    {Workspace::Scene, "Scene", "lagrange-dockspace", ImGuiKey_1, "Ctrl+1"},
    {Workspace::Domains, "Domains", "lagrange-dockspace-domains", ImGuiKey_2, "Ctrl+2"},
}};

constexpr ImGuiWindowFlags kBarFlags =
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;

void buildLayout(Workspace workspace, ImGuiID dockspace, const ImVec2& size) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, size);
    ImGuiID centre = dockspace;
    if (workspace == Workspace::Scene) {
        const ImGuiID bottom =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.15F, nullptr, &centre);
        const ImGuiID scope =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.42F, nullptr, &centre);
        const ImGuiID left =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, 0.20F, nullptr, &centre);
        const ImGuiID right =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.26F, nullptr, &centre);
        ImGui::DockBuilderDockWindow("Viewport", centre);
        ImGui::DockBuilderDockWindow("Scene", left);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderDockWindow("Timeline", bottom);
        ImGui::DockBuilderDockWindow("Scope", scope);
    } else {
        // Design: Domains 520 px on the left, Profile on the right. The wiring and schedule
        // panels that sit between them are not built yet, so Profile takes that room for now.
        const float fraction = std::clamp(px(520.0F) / std::max(size.x, 1.0F), 0.2F, 0.6F);
        const ImGuiID left =
            ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, fraction, nullptr, &centre);
        ImGui::DockBuilderDockWindow("Domains", left);
        ImGui::DockBuilderDockWindow("Profile", centre);
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

void centreLine(float contentHeight) {
    ImGui::SetCursorPosY((ImGui::GetWindowHeight() - contentHeight) * 0.5F);
}

void drawMenuBar(PanelContext& context) {
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }
    widgets::Strong("Lagrange");
    ImGui::SameLine(0.0F, px(14.0F));
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open robot or recording...", "Ctrl+O", false, context.canOpenFiles)) {
            context.openFileRequested = true;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        for (const WorkspaceEntry& entry : kWorkspaces) {
            if (ImGui::MenuItem(entry.label, entry.shortcut,
                                *context.workspace == entry.workspace)) {
                *context.workspace = entry.workspace;
            }
        }
        ImGui::EndMenu();
    }
    if (context.scene) {
        const std::string& stage = context.scene->statics->stagePath;
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float width = ImGui::CalcTextSize(stage.c_str()).x;
        ImGui::PopFont();
        const float label = ImGui::CalcTextSize("stage").x + px(6.0F);
        widgets::AlignRight(width + label + px(8.0F));
        ImGui::TextDisabled("stage");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(c::t1, "%s", stage.c_str());
    }
    ImGui::EndMainMenuBar();
}

void drawWorkspaceBar(PanelContext& context, ImGuiViewport* viewport) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(8.0F), 0.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    const bool open =
        ImGui::BeginViewportSideBar("##workspace-bar", viewport, ImGuiDir_Up, px(40.0F), kBarFlags);
    ImGui::PopStyleVar(2);
    if (open) {
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(px(2.0F), 0.0F));
        for (const WorkspaceEntry& entry : kWorkspaces) {
            if (widgets::BarTab(entry.label, *context.workspace == entry.workspace)) {
                *context.workspace = entry.workspace;
            }
            ImGui::SameLine();
        }
        ImGui::PopStyleVar();

        // Simulation controls and readouts, on the right.
        const bool ready = context.connection.state == ConnectionState::Ready;
        const bool running = context.status.runState == RunState::Running;
        char time[32];
        format::clock(context.status.simTimeNs, time, sizeof(time));
        char rate[48];
        std::snprintf(rate, sizeof(rate), "%.2fx / %.2f", context.status.realTimeFactor,
                      context.status.realTimeFactorTarget);
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float readouts = ImGui::CalcTextSize(time).x + ImGui::CalcTextSize(rate).x;
        ImGui::PopFont();
        const float width = px(78.0F) + readouts + ImGui::CalcTextSize("sim t").x +
                            ImGui::CalcTextSize("RTF").x + px(64.0F);
        ImGui::SameLine();
        widgets::AlignRight(width);
        centreLine(ImGui::GetFrameHeight());
        ImGui::BeginDisabled(!ready);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(12.0F), px(4.0F)));
        if (widgets::PrimaryButton(running ? "Pause" : "Play")) {
            if (running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        ImGui::PopStyleVar();
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("sim t");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(c::t1, "%s", time);
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::TextDisabled("RTF");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(c::t1, "%s", rate);
        bottomRule();
    }
    ImGui::End();
}

void drawDomainStrip(PanelContext& context, ImGuiViewport* viewport) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(10.0F), px(5.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    const bool open =
        ImGui::BeginViewportSideBar("##domain-strip", viewport, ImGuiDir_Up, px(32.0F), kBarFlags);
    ImGui::PopStyleVar(2);
    if (open) {
        ProfileSession& session = *context.profileSession;
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Domains");
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
            ImGui::SameLine(0.0F, px(8.0F));
            const std::string_view label = widgets::domainShortLabel(view.name);
            char name[32];
            std::snprintf(name, sizeof(name), "%.*s", static_cast<int>(label.size()), label.data());
            if (widgets::DomainChip(view.name.c_str(), widgets::domainColor(view.name), name,
                                    detail, state)) {
                *context.workspace = Workspace::Domains;
                session.selectedDomain = view.name;
            }
        }

        // Context pill on the right: what would happen on Apply.
        const std::size_t changes = session.draft().changedLineCount();
        if (changes != 0) {
            char text[48];
            std::snprintf(text, sizeof(text), "%zu %s · not applied", changes,
                          changes == 1 ? "change" : "changes");
            ImGui::SameLine();
            widgets::AlignRight(widgets::PillWidth(text));
            widgets::Pill(text, widgets::PillKind::Accent);
        }
        bottomRule();
    }
    ImGui::End();
}

void drawStatusBar(const PanelContext& context, ImGuiViewport* viewport) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(10.0F), px(3.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    const bool open =
        ImGui::BeginViewportSideBar("##status-bar", viewport, ImGuiDir_Down, px(24.0F), kBarFlags);
    ImGui::PopStyleVar(2);
    if (open) {
        const ConnectionState state = context.connection.state;
        const ImVec4 dot = state == ConnectionState::Ready
                               ? c::ok
                               : (state == ConnectionState::Refused ? c::error : c::warn);
        widgets::Dot(dot);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::TextUnformatted(state == ConnectionState::Ready ? "Connected" : toString(state));
        if (!context.connection.serverName.empty()) {
            ImGui::SameLine(0.0F, px(16.0F));
            ImGui::TextDisabled("server");
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Mono(c::t2, "%s", context.connection.serverName.c_str());
        }
        if (!context.connection.message.empty()) {
            ImGui::SameLine(0.0F, px(16.0F));
            widgets::Mono(c::warn, "%s", context.connection.message.c_str());
        }
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::TextDisabled("signals");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(c::t1, "%u", context.status.signalCount);
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::TextDisabled("deltas");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(c::t1, "%llu", static_cast<unsigned long long>(context.stats.deltas));
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::TextDisabled("dropped");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Mono(context.status.droppedDeltas == 0 ? c::t1 : c::warn, "%llu",
                      static_cast<unsigned long long>(context.status.droppedDeltas));

        char right[200];
        const float fps = context.frameMs > 0.0F ? 1000.0F / context.frameMs : 0.0F;
        std::snprintf(right, sizeof(right), "%s · %s    %.1f ms · %.0f fps",
                      context.deviceName.c_str(), context.tierName.c_str(), context.frameMs, fps);
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float width = ImGui::CalcTextSize(right).x;
        ImGui::PopFont();
        ImGui::SameLine();
        widgets::AlignRight(width);
        widgets::Mono(c::t2, "%s", right);
    }
    ImGui::End();
}

/// One host window fills what the bars left over. It holds a dock space per workspace; the
/// ones not shown are kept alive, so their panels stay docked where the user put them.
void drawDockSpaces(Workspace shown, ImGuiViewport* viewport) {
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
        const ImGuiID id = ImGui::GetID(entry.dockspace);
        // Lay out only when there is no saved layout for this dock space.
        if (ImGui::DockBuilderGetNode(id) == nullptr) {
            buildLayout(entry.workspace, id, size);
        }
        if (entry.workspace == shown) {
            ImGui::DockSpace(id, size);
        } else {
            ImGui::DockSpace(id, size, ImGuiDockNodeFlags_KeepAliveOnly);
        }
    }
    ImGui::End();
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

void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context) {
    context.profileSession->update(context.client,
                                   context.connection.state == ConnectionState::Ready,
                                   context.stats.connects, ImGui::GetTime());

    if (context.canOpenFiles && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) {
        context.openFileRequested = true;
    }
    for (const WorkspaceEntry& entry : kWorkspaces) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | entry.key)) {
            *context.workspace = entry.workspace;
        }
    }

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    drawMenuBar(context);
    drawWorkspaceBar(context, viewport);
    drawDomainStrip(context, viewport);
    drawStatusBar(context, viewport);
    const Workspace shown = *context.workspace;  // a click this frame takes effect next frame
    drawDockSpaces(shown, viewport);
    for (const std::unique_ptr<Panel>& panel : panels) {
        if (panel->workspace() == shown) {
            panel->draw(context);
        }
    }
}

}  // namespace lagrange::viewer
