// The command palette (docs/design/ui/shared/ui-conventions.md §3: Ctrl K, fuzzy search over
// prims, signals, topics and commands). One list, best match first; Enter runs the chosen line.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Fuzzy.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::size_t kShown = 12;          // lines listed at once
constexpr std::size_t kMostSignals = 4000;  // the palette's share of a huge catalog
constexpr const char* kPopup = "##command-palette";

enum class Kind { Command, Prim, Signal, Topic };

enum class Action {
    Workspace,
    PlayPause,
    Step,
    Reset,
    Record,
    ClearSelection,
    FocusSelection,
    ResetLayout,
    OpenFile,
    Disconnect,
    BuildRobot,
    ConnectDefault,
    StartLocal,
    SelectPrim,
    PlotSignal,
    ShowSignal,
    ShowTopic,
};

struct Item {
    Kind kind;
    Action action;
    std::string text;        // what is searched and shown
    std::string detail;      // shortcut, unit or type, in t3 on the right
    std::uint32_t index{0};  // workspace, prim or signal index
};

struct Palette {
    bool requested{false};  // from the menu
    bool open{false};
    bool focus{false};
    char query[128]{};
    std::string last;  // the query the matches are for
    std::vector<Item> items;
    std::vector<std::string_view> texts;
    std::vector<std::size_t> matches;
    int chosen{0};
};

Palette& palette() {
    static Palette state;
    return state;
}

const char* kindName(Kind kind) {
    switch (kind) {
        case Kind::Command:
            return "command";
        case Kind::Prim:
            return "prim";
        case Kind::Signal:
            return "signal";
        case Kind::Topic:
            return "topic";
    }
    return "";
}

struct WorkspaceName {
    Workspace workspace;
    const char* name;
    const char* shortcut;
};
constexpr WorkspaceName kWorkspaceNames[]{
    {Workspace::Scene, "Scene", "Ctrl+1"},
    {Workspace::Domains, "Domains", "Ctrl+2"},
    {Workspace::Electrical, "Electrical", "Ctrl+3"},
    {Workspace::Signals, "Signals", "Ctrl+4"},
    {Workspace::Sensors, "Sensors", "Ctrl+5"},
    {Workspace::Environment, "Environment", "Ctrl+6"},
    {Workspace::Replay, "Replay", "Ctrl+7"},
    {Workspace::Ros2, "ROS 2", "Ctrl+8"},
    {Workspace::World, "World", "Ctrl+9"},
    {Workspace::Performance, "Performance", "Ctrl+0"},
    {Workspace::Builder, "Builder", "Ctrl+B"},
};

/// What the palette offers now: commands, then the scene's prims, the signals and the topics.
void collect(PanelContext& context, Palette& state) {
    state.items.clear();
    const auto command = [&state](Action action, std::string text, std::string detail,
                                  std::uint32_t index = 0) {
        state.items.push_back({Kind::Command, action, std::move(text), std::move(detail), index});
    };
    if (context.launcher->active) {
        command(Action::BuildRobot, "Build a robot", "Ctrl+B");
        command(Action::ConnectDefault, "Connect to the server on the default socket", "");
        if (!context.launcher->localServer.empty() && !context.launcher->defaultAvailable) {
            command(Action::StartLocal, "Start local simulator and connect", "");
        }
        if (context.canOpenFiles) {
            command(Action::OpenFile, "Open robot or recording...", "Ctrl+O");
        }
        return;
    }
    for (std::uint32_t i = 0; i < std::size(kWorkspaceNames); ++i) {
        command(Action::Workspace, std::string("Go to ") + kWorkspaceNames[i].name,
                kWorkspaceNames[i].shortcut, i);
    }
    const bool ready = context.connection.state == ConnectionState::Ready;
    if (ready) {
        const bool running = context.status.runState == RunState::Running;
        command(Action::PlayPause, running ? "Pause the run" : "Play the run", "Space");
        if (context.connection.canStep && !running) {
            command(Action::Step, "Step 10 ms", ".");
        }
        command(Action::Reset, "Reset the run to its start", "");
        if (!context.connection.canSeek &&
            context.status.replayEndNs <= context.status.replayBeginNs) {
            command(Action::Record,
                    context.status.recordingPath.empty() ? "Start recording" : "Stop recording",
                    "Ctrl+R");
        }
    }
    if (context.selectedPrim != kNone) {
        command(Action::ClearSelection, "Clear selection", "Esc");
        command(Action::FocusSelection, "Focus the camera on the selection", "F");
    }
    command(Action::ResetLayout, "Reset this workspace's layout", "");
    if (context.canOpenFiles) {
        command(Action::OpenFile, "Add robot or open recording...", "Ctrl+O");
    }
    command(Action::Disconnect, "Disconnect and show the launcher", "");

    if (context.scene) {
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        for (std::uint32_t i = 0; i < prims.size(); ++i) {
            state.items.push_back(
                {Kind::Prim, Action::SelectPrim, prims[i].path, prims[i].typeName, i});
        }
    }
    if (context.catalog) {
        const std::vector<SignalInfo>& signals = context.catalog->signals;
        for (std::uint32_t i = 0; i < signals.size() && i < kMostSignals; ++i) {
            const SignalInfo& info = signals[i];
            if (info.components == 0) {
                continue;  // an image: shown in Sensors, not plotted
            }
            std::string detail = info.unit == "1" ? std::string("-") : info.unit;
            if (info.components > 1) {
                detail += " · [" + std::to_string(info.components) + "]";
            }
            state.items.push_back({Kind::Signal,
                                   info.components == 1 ? Action::PlotSignal : Action::ShowSignal,
                                   info.name, std::move(detail), i});
        }
    }
    context.client.setRosGraphWanted(true);
    if (const std::shared_ptr<const RosGraphInfo> graph = context.client.rosGraph()) {
        for (const RosGraphInfo::Topic& topic : graph->topics) {
            const std::size_t slash = topic.type.rfind('/');
            state.items.push_back({Kind::Topic, Action::ShowTopic, topic.name,
                                   topic.type.substr(slash == std::string::npos ? 0 : slash + 1),
                                   0});
        }
    }
    state.texts.clear();
    for (const Item& item : state.items) {
        state.texts.emplace_back(item.text);
    }
}

void run(PanelContext& context, const Item& item) {
    switch (item.action) {
        case Action::Workspace:
            *context.workspace = kWorkspaceNames[item.index].workspace;
            break;
        case Action::PlayPause:
            if (context.status.runState == RunState::Running) {
                context.client.pause();
            } else {
                context.client.play();
            }
            break;
        case Action::Step:
            context.client.step(10'000'000);
            break;
        case Action::Reset:
            if (context.connection.canSeek) {
                context.client.seek(context.status.replayBeginNs);
            } else {
                context.client.reset();
            }
            break;
        case Action::Record:
            context.client.setRecording(context.status.recordingPath.empty());
            break;
        case Action::ClearSelection:
            context.selectedPrim = kNone;
            break;
        case Action::FocusSelection:
            if (context.scene && context.selectedPrim < context.scene->transforms.size()) {
                context.camera.target = context.scene->transforms[context.selectedPrim].position;
            }
            break;
        case Action::ResetLayout:
            if (context.resetLayout != nullptr) {
                *context.resetLayout = true;
            }
            break;
        case Action::OpenFile:
            context.openFileRequested = true;
            break;
        case Action::Disconnect:
            context.launcher->disconnectRequested = true;
            break;
        case Action::BuildRobot:
            context.launcher->builderRequested = true;
            break;
        case Action::ConnectDefault:
            context.launcher->connectRequested = true;
            context.launcher->connectEndpoint = context.launcher->defaultEndpoint;
            break;
        case Action::StartLocal:
            context.launcher->connectRequested = true;
            context.launcher->startLocalRequested = true;
            context.launcher->connectEndpoint = context.launcher->defaultEndpoint;
            break;
        case Action::SelectPrim:
            context.selectedPrim = item.index;
            // Selection is global; show it where a prim is inspected.
            if (*context.workspace != Workspace::Scene &&
                *context.workspace != Workspace::Electrical) {
                *context.workspace = Workspace::Scene;
            }
            break;
        case Action::PlotSignal:
            if (context.initialScopeSignals != nullptr) {
                context.initialScopeSignals->push_back(item.text);
            }
            *context.workspace = Workspace::Signals;
            break;
        case Action::ShowSignal:
            requestSignalSearch(item.text);
            *context.workspace = Workspace::Signals;
            break;
        case Action::ShowTopic:
            requestRosTopic(item.text);
            *context.workspace = Workspace::Ros2;
            break;
    }
}

}  // namespace

void openCommandPalette() {
    palette().requested = true;
}

void drawCommandPalette(PanelContext& context) {
    Palette& state = palette();
    const bool requested = std::exchange(state.requested, false);
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_K) || (requested && !state.open)) {
        state.open = !state.open;
        if (state.open) {
            state.query[0] = '\0';
            state.last = "\x01";  // matched again below
            state.chosen = 0;
            state.focus = true;
            collect(context, state);
        }
    }
    if (!state.open) {
        return;
    }
    if (!ImGui::IsPopupOpen(kPopup)) {
        ImGui::OpenPopup(kPopup);
    }
    if (state.last != state.query) {
        state.last = state.query;
        state.matches = fuzzyRank(state.texts, state.query, kShown * 8);
        state.chosen = 0;
    }

    // Design tokens: a bg1 sheet with a lineStrong border, 560 px wide, under the menu bar.
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float width = std::min(px(560.0F), viewport->WorkSize.x - px(32.0F));
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + (viewport->WorkSize.x - width) * 0.5F,
                                   viewport->WorkPos.y + px(56.0F)));
    ImGui::SetNextWindowSize(ImVec2(width, 0.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(8.0F), px(8.0F)));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, c::bg1);
    ImGui::PushStyleColor(ImGuiCol_Border, c::lineStrong);
    const bool shown = ImGui::BeginPopup(kPopup, ImGuiWindowFlags_NoMove);
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
    if (!shown) {
        state.open = false;  // closed by a click outside or Esc
        return;
    }
    if (state.focus) {
        ImGui::SetKeyboardFocusHere();
        state.focus = false;
    }
    widgets::SearchField("##palette-query", "Search prims, signals, topics and commands",
                         state.query, sizeof(state.query), ImGui::GetContentRegionAvail().x);
    const int count = static_cast<int>(std::min(state.matches.size(), kShown * 8));
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
        state.chosen = std::min(state.chosen + 1, std::max(count - 1, 0));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
        state.chosen = std::max(state.chosen - 1, 0);
    }
    const bool enter = ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
                       ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false);
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));

    int ran = -1;
    if (count == 0) {
        widgets::Caption("  Nothing matches.");
    } else {
        const float row = px(24.0F);
        ImGui::BeginChild("##palette-list",
                          ImVec2(0.0F, row * static_cast<float>(std::min<std::size_t>(
                                                 static_cast<std::size_t>(count), kShown))));
        const theme::Fonts& fonts = theme::fonts();
        for (int i = 0; i < count; ++i) {
            const Item& item = state.items[state.matches[static_cast<std::size_t>(i)]];
            ImGui::PushID(i);
            const ImVec2 min = ImGui::GetCursorScreenPos();
            const float w = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable("##line", i == state.chosen, ImGuiSelectableFlags_None,
                                  ImVec2(w, row))) {
                ran = i;
            }
            if (i == state.chosen && (ImGui::IsKeyPressed(ImGuiKey_DownArrow) ||
                                      ImGui::IsKeyPressed(ImGuiKey_UpArrow))) {
                ImGui::SetScrollHereY();
            }
            ImDrawList* draw = ImGui::GetWindowDrawList();
            const float y = min.y + (row - px(theme::kBodySize)) * 0.5F;
            // The kind on the left, the text, and its detail on the right.
            const char* kind = kindName(item.kind);
            draw->AddText(fonts.sans, px(theme::kCaptionSize),
                          ImVec2(min.x + px(6.0F), y + px(1.0F)),
                          ImGui::ColorConvertFloat4ToU32(c::t3), kind);
            const float detailWidth =
                fonts.mono
                    ->CalcTextSizeA(px(theme::kCaptionSize), FLT_MAX, 0.0F, item.detail.c_str())
                    .x;
            const float textX = min.x + px(70.0F);
            const bool mono = item.kind != Kind::Command;
            fit::draw(draw, mono ? fonts.mono : fonts.sans,
                      px(mono ? theme::kMonoSize : theme::kBodySize), ImVec2(textX, y),
                      ImGui::ColorConvertFloat4ToU32(c::t1), item.text,
                      min.x + w - detailWidth - px(16.0F) - textX, mono);
            draw->AddText(fonts.mono, px(theme::kCaptionSize),
                          ImVec2(min.x + w - detailWidth - px(6.0F), y + px(1.0F)),
                          ImGui::ColorConvertFloat4ToU32(c::t3), item.detail.c_str());
            ImGui::PopID();
        }
        ImGui::EndChild();
    }
    if (enter && count > 0) {
        ran = state.chosen;
    }
    widgets::Caption("↑ ↓ choose  ·  Enter run  ·  Esc close");
    if (ran >= 0) {
        const Item item = state.items[state.matches[static_cast<std::size_t>(ran)]];
        state.open = false;
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        run(context, item);
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        state.open = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

}  // namespace lagrange::viewer
