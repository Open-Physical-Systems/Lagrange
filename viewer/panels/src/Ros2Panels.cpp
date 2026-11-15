// ROS 2 workspace: topics, services, actions and parameters, the node graph and the TF tree,
// and the ros2_control interface (docs/design/ui/09-ros2/SPEC.md). The viewer does not join
// the ROS domain: everything comes from the simulation server's graph report and its scene.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/RosFrames.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

bool containsNoCase(std::string_view text, std::string_view needle) {
    const auto it =
        std::search(text.begin(), text.end(), needle.begin(), needle.end(), [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a)) ==
                   std::tolower(static_cast<unsigned char>(b));
        });
    return it != text.end();
}

/// Asks for the graph while a ROS 2 panel is shown, and says what is missing when there is
/// none. Returns the graph, or nullptr after writing one line of guidance.
std::shared_ptr<const RosGraphInfo> graphOrHint(PanelContext& context) {
    context.client.setRosGraphWanted(true);
    std::shared_ptr<const RosGraphInfo> graph = context.client.rosGraph();
    if (!graph) {
        widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                   ? "Waiting for the server's ROS 2 graph ..."
                                   : "Not connected. The ROS 2 graph appears when a simulation "
                                     "server is reachable.");
    }
    return graph;
}

void rightMono(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    widgets::AlignRight(ImGui::CalcTextSize(text).x + px(8.0F));
    ImGui::PopFont();
    widgets::Mono(color, "%s", text);
}

void bandwidth(double bitsPerSecond, char* out, std::size_t size) {
    const double bytes = bitsPerSecond / 8.0;
    if (bytes <= 0.0) {
        std::snprintf(out, size, "-");
    } else if (bytes >= 1e6) {
        std::snprintf(out, size, "%.1f MB/s", bytes / 1e6);
    } else if (bytes >= 1e3) {
        std::snprintf(out, size, "%.0f kB/s", bytes / 1e3);
    } else {
        std::snprintf(out, size, "%.0f B/s", bytes);
    }
}

std::string& requestedTopic();

/// A hidden ROS name: one of its parts starts with "_" ("/points/_buf_cpu"). `ros2 topic list`
/// leaves them out unless asked; so does this workspace.
bool hiddenName(std::string_view name) {
    for (std::size_t at = name.find('_'); at != std::string_view::npos;
         at = name.find('_', at + 1)) {
        if (at == 0 || name[at - 1] == '/') {
            return true;
        }
    }
    return false;
}

/// How many of `names` are not hidden.
std::size_t shownCount(const std::vector<std::string>& names) {
    return static_cast<std::size_t>(
        std::ranges::count_if(names, [](const std::string& name) { return !hiddenName(name); }));
}

/// "sensor_msgs/msg/PointCloud2" -> "PointCloud2".
const char* shortType(const std::string& type) {
    const std::size_t slash = type.rfind('/');
    return type.c_str() + (slash == std::string::npos ? 0 : slash + 1);
}

// --- topics -------------------------------------------------------------------------------

class TopicsPanel final : public Panel {
public:
    explicit TopicsPanel(std::shared_ptr<std::string> selected) : selected_(std::move(selected)) {}
    [[nodiscard]] const char* title() const override { return "Topics"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        if (!requestedTopic().empty()) {
            *selected_ = std::move(requestedTopic());
            requestedTopic().clear();
            filter_[0] = '\0';
            scope_ = 1;  // all: the palette lists every topic
        }
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 top = ImGui::GetCursorPos();  // below the tab bar
        ImGui::SetCursorPos(ImVec2(top.x + px(8.0F), top.y + px(6.0F)));
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (!graph) {
            ImGui::End();
            return;
        }
        // Search, and whether to list only what the simulator's bridge publishes.
        static constexpr std::array<const char*, 2> kScopes{"sim", "all"};
        const float scopeWidth = px(74.0F);
        widgets::SearchField("##filter", "Filter topics", filter_, sizeof(filter_),
                             ImGui::GetContentRegionAvail().x - scopeWidth - px(16.0F));
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + px(1.0F));
        widgets::Segmented("##scope", kScopes.data(), 2, &scope_);
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));

        const RosGraphInfo::Topic* chosen = nullptr;
        const float details = px(150.0F);
        if (ImGui::BeginTable("##topics", 4,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoPadOuterX,
                              ImVec2(0.0F, -details))) {
            ImGui::TableSetupColumn("topic", ImGuiTableColumnFlags_None, 3.0F);
            ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_None, 1.8F);
            ImGui::TableSetupColumn("Hz", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("BW", ImGuiTableColumnFlags_None, 1.3F);
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            for (const char* name : {"topic", "type"}) {
                ImGui::TableNextColumn();
                widgets::Caption("  %s", name);
            }
            for (const char* name : {"Hz", "BW"}) {
                ImGui::TableNextColumn();
                widgets::AlignRight(ImGui::CalcTextSize(name).x + px(8.0F));
                widgets::Caption("%s", name);
            }
            for (const RosGraphInfo::Topic& topic : graph->topics) {
                // Hidden topics (an action's, a buffer's) stay out, as `ros2 topic list`
                // leaves them out; the Actions tab lists actions from theirs.
                if (isHiddenRosName(topic.name) || (scope_ == 0 && !topic.fromBridge) ||
                    (filter_[0] != '\0' && !containsNoCase(topic.name, filter_))) {
                    continue;
                }
                const bool selected = *selected_ == topic.name;
                if (selected) {
                    chosen = &topic;
                }
                ImGui::PushID(topic.name.c_str());
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                if (ImGui::Selectable(
                        "##topic", selected,
                        ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
                    *selected_ = topic.name;
                }
                ImGui::SameLine(px(8.0F));
                // A topic with a problem is written in the warning colour (design).
                widgets::Mono(topic.status.empty() ? c::t1 : c::warn, "%s", topic.name.c_str());
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", shortType(topic.type));
                ImGui::TableNextColumn();
                char text[32];
                if (topic.rateHz > 0.0) {
                    std::snprintf(text, sizeof(text), topic.rateHz >= 100.0 ? "%.0f" : "%.1f",
                                  topic.rateHz);
                } else {
                    std::snprintf(text, sizeof(text), "-");
                }
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                bandwidth(topic.bandwidthBps, text, sizeof(text));
                rightMono(c::t2, text);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        // The selected topic.
        if (chosen == nullptr) {
            ImGui::SetCursorPosX(px(8.0F));
            widgets::Caption("Select a topic to see who publishes it.");
        } else if (widgets::SectionHeader(chosen->name.c_str(), shortType(chosen->type))) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("type", chosen->type.c_str(), nullptr);
            char text[64];
            std::snprintf(text, sizeof(text), "%u publishing · %u subscribed", chosen->publishers,
                          chosen->subscribers);
            widgets::PropertyRow("endpoints", text, nullptr);
            widgets::PropertyRow(
                "from",
                chosen->fromBridge ? "the simulator's bridge" : "another node in the domain",
                nullptr);
            if (!chosen->status.empty()) {
                widgets::Pill(chosen->status.c_str(), widgets::PillKind::Warn);
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    std::shared_ptr<std::string> selected_;
    char filter_[96]{};
    int scope_{1};
};

// --- node graph ---------------------------------------------------------------------------

class NodeGraphPanel final : public Panel {
public:
    explicit NodeGraphPanel(std::shared_ptr<std::string> selected)
        : selected_(std::move(selected)) {}
    [[nodiscard]] const char* title() const override { return "Node graph"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 top = ImGui::GetCursorPos();  // below the tab bar
        ImGui::SetCursorPos(ImVec2(top.x + px(8.0F), top.y + px(8.0F)));
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (!graph) {
            ImGui::End();
            return;
        }
        ImGui::SetCursorPos(top);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##canvas", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::PopStyleColor();
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 visible = ImGui::GetContentRegionAvail();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        if (graph->nodes.empty()) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            widgets::Caption("The graph lists no nodes.");
            if (!graph->note.empty()) {
                // A server without a bridge says why; that is the whole content of the graph.
                ImGui::SetCursorPosX(px(8.0F));
                fit::pill(graph->note, widgets::PillKind::Warn,
                          ImGui::GetContentRegionAvail().x - px(8.0F));
            }
            ImGui::EndChild();
            ImGui::End();
            return;
        }
        // Layout: nodes that publish (the bridge first) in a left column, the others in a
        // right column. Enough for the graphs a simulator's bridge reports.
        const float nodeW = px(230.0F);
        const float nodeH = px(60.0F);
        const float gapY = px(34.0F);
        places_.assign(graph->nodes.size(), ImVec2());
        float leftY = px(30.0F);
        float rightY = px(30.0F);
        const float rightX = std::max(visible.x - nodeW - px(40.0F), nodeW + px(220.0F));
        for (std::size_t i = 0; i < graph->nodes.size(); ++i) {
            const bool publisher = !graph->nodes[i].publishes.empty();
            if (publisher) {
                places_[i] = ImVec2(origin.x + px(30.0F), origin.y + leftY);
                leftY += nodeH + gapY;
            } else {
                places_[i] = ImVec2(origin.x + rightX, origin.y + rightY);
                rightY += nodeH + gapY;
            }
        }
        // Edges: one per topic between the node that publishes it and each that subscribes.
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        for (std::size_t from = 0; from < graph->nodes.size(); ++from) {
            for (const std::string& topic : graph->nodes[from].publishes) {
                if (hiddenName(topic)) {
                    continue;
                }
                for (std::size_t to = 0; to < graph->nodes.size(); ++to) {
                    const auto& wanted = graph->nodes[to].subscribes;
                    if (from == to || isHiddenRosName(topic) ||
                        std::ranges::find(wanted, topic) == wanted.end()) {
                        continue;
                    }
                    const ImVec2 a(places_[from].x + nodeW, places_[from].y + nodeH * 0.5F);
                    const ImVec2 b(places_[to].x, places_[to].y + nodeH * 0.5F);
                    const bool selected = *selected_ == topic;
                    const ImU32 color = u32(selected ? c::accent : c::lineStrong);
                    const float bend = (b.x - a.x) * 0.45F;
                    draw->AddBezierCubic(a, ImVec2(a.x + bend, a.y), ImVec2(b.x - bend, b.y), b,
                                         color, px(1.5F));
                    draw->AddTriangleFilled(b, ImVec2(b.x - px(7.0F), b.y - px(4.0F)),
                                            ImVec2(b.x - px(7.0F), b.y + px(4.0F)), color);
                    const ImVec2 mid((a.x + b.x) * 0.5F, (a.y + b.y) * 0.5F - small - px(2.0F));
                    const float width =
                        fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, topic.c_str()).x;
                    draw->AddRectFilled(
                        ImVec2(mid.x - width * 0.5F - px(3.0F), mid.y),
                        ImVec2(mid.x + width * 0.5F + px(3.0F), mid.y + small + px(2.0F)),
                        u32(c::bg0));
                    draw->AddText(fonts.mono, small, ImVec2(mid.x - width * 0.5F, mid.y),
                                  u32(selected ? c::accent : c::t3), topic.c_str());
                }
            }
        }
        for (std::size_t i = 0; i < graph->nodes.size(); ++i) {
            const RosGraphInfo::Node& node = graph->nodes[i];
            const ImVec2 min = places_[i];
            const ImVec2 max(min.x + nodeW, min.y + nodeH);
            // The simulator's bridge is the accent node of the graph (design).
            const bool bridge = node.name.find("lagrange") != std::string::npos;
            draw->AddRectFilled(min, max, u32(c::bg2), px(4.0F));
            draw->AddRect(min, max, u32(bridge ? c::accent : c::lineStrong), px(4.0F),
                          bridge ? px(2.0F) : 1.0F);
            draw->AddLine(ImVec2(min.x, min.y + px(24.0F)), ImVec2(max.x, min.y + px(24.0F)),
                          u32(c::line));
            draw->AddCircleFilled(ImVec2(min.x + px(12.0F), min.y + px(12.0F)), px(4.0F),
                                  u32(node.external ? c::t3 : c::ok));
            draw->PushClipRect(min, max, true);
            const char* name = node.name.c_str() + (node.name.front() == '/' ? 1 : 0);
            draw->AddText(fonts.sansSemiBold, px(theme::kLabelSize),
                          ImVec2(min.x + px(22.0F), min.y + px(5.0F)),
                          u32(node.external ? c::t3 : c::t1), name);
            char line[160];
            if (!node.detail.empty()) {
                std::snprintf(line, sizeof(line), "%s", node.detail.c_str());
            } else {
                std::snprintf(line, sizeof(line), "pub %zu · sub %zu", shownCount(node.publishes),
                              shownCount(node.subscribes));
            }
            draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), min.y + px(34.0F)),
                          u32(c::t2), line);
            draw->PopClipRect();
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::Dummy(ImVec2(rightX + nodeW + px(30.0F), std::max(leftY, rightY)));
        // A graph that no real bridge stands behind says so, on the canvas.
        if (!graph->note.empty()) {
            ImGui::SetCursorScreenPos(
                ImVec2(origin.x + ImGui::GetScrollX() + px(10.0F),
                       origin.y + ImGui::GetScrollY() + visible.y - px(28.0F)));
            fit::pill(graph->note, widgets::PillKind::Warn, visible.x - px(20.0F));
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    std::shared_ptr<std::string> selected_;
    std::vector<ImVec2> places_;
};

// --- services and parameters --------------------------------------------------------------

/// A tab whose content the server's graph report (protocol v1's RosGraph) does not carry:
/// it says so, and how to see the same thing from a terminal in the ROS domain.
class UnreportedPanel final : public Panel {
public:
    UnreportedPanel(const char* title, const char* what, const char* command)
        : title_(title), what_(what), command_(command) {}
    [[nodiscard]] const char* title() const override { return title_; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (graph) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "This server's ROS 2 graph does not report %s: its report holds "
                "topics, nodes and ros2_control.",
                what_);
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Caption("From a terminal in ROS domain %u:", graph->domainId);
            widgets::Mono(c::t1, "%s", command_);
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }

private:
    const char* title_;
    const char* what_;
    const char* command_;
};

// --- actions ------------------------------------------------------------------------------

/// "/a, /b" from a list of node names, or "-" for none.
std::string joined(const std::vector<std::string>& names) {
    std::string out;
    for (const std::string& name : names) {
        out += (out.empty() ? "" : ", ") + name;
    }
    return out.empty() ? "-" : out;
}

class ActionsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Actions"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 top = ImGui::GetCursorPos();
        ImGui::SetCursorPos(ImVec2(top.x + px(8.0F), top.y + px(6.0F)));
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (!graph) {
            ImGui::End();
            return;
        }
        // Worked out again only when a new graph arrives.
        if (graph != source_) {
            source_ = graph;
            actions_ = rosActions(*graph);
        }
        widgets::SearchField("##filter", "Filter actions", filter_, sizeof(filter_),
                             ImGui::GetContentRegionAvail().x - px(8.0F));
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        if (actions_.empty()) {
            ImGui::SetCursorPosX(px(8.0F));
            ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
            widgets::Caption(
                "No action servers in the domain. Actions appear when a node "
                "serves one (Nav2's navigate_to_pose, MoveIt's move_action).");
            ImGui::PopTextWrapPos();
            ImGui::End();
            return;
        }
        const RosAction* chosen = nullptr;
        if (ImGui::BeginTable("##actions", 3,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoPadOuterX,
                              ImVec2(0.0F, -px(150.0F)))) {
            ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_None, 3.0F);
            ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_None, 2.2F);
            ImGui::TableSetupColumn("server", ImGuiTableColumnFlags_None, 2.0F);
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            for (const char* name : {"action", "type", "server"}) {
                ImGui::TableNextColumn();
                widgets::Caption("  %s", name);
            }
            for (const RosAction& action : actions_) {
                if (filter_[0] != '\0' && !containsNoCase(action.name, filter_)) {
                    continue;
                }
                const bool selected = selected_ == action.name;
                if (selected) {
                    chosen = &action;
                }
                ImGui::PushID(action.name.c_str());
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                if (ImGui::Selectable(
                        "##action", selected,
                        ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
                    selected_ = action.name;
                }
                ImGui::SameLine(px(8.0F));
                // An action nobody serves any more is written in the warning colour.
                widgets::Mono(action.servers.empty() ? c::warn : c::t1, "%s", action.name.c_str());
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", action.type.empty() ? "-" : shortType(action.type));
                ImGui::TableNextColumn();
                widgets::Mono(c::t2, "%s", joined(action.servers).c_str());
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (chosen == nullptr) {
            ImGui::SetCursorPosX(px(8.0F));
            widgets::Caption("Select an action to see its server and clients.");
        } else if (widgets::SectionHeader(chosen->name.c_str(), chosen->type.empty()
                                                                    ? nullptr
                                                                    : shortType(chosen->type))) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("type", chosen->type.empty() ? "-" : chosen->type.c_str(),
                                 nullptr);
            widgets::PropertyRow("server", joined(chosen->servers).c_str(), nullptr);
            widgets::PropertyRow("clients", joined(chosen->clients).c_str(), nullptr);
            if (chosen->servers.empty()) {
                widgets::Pill("no server", widgets::PillKind::Warn);
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    std::shared_ptr<const RosGraphInfo> source_;
    std::vector<RosAction> actions_;
    std::string selected_;
    char filter_[96]{};
};

// --- TF tree ------------------------------------------------------------------------------

const char* kindName(TfFrame::Kind kind) {
    switch (kind) {
        case TfFrame::Kind::Odom:
            return "odometry";
        case TfFrame::Kind::Link:
            return "link";
        case TfFrame::Kind::Sensor:
            return "sensor";
        case TfFrame::Kind::Optical:
            return "optical";
    }
    return "";
}

class TfTreePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "TF tree"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (!graph) {
            ImGui::End();
            return;
        }
        if (!context.scene) {
            widgets::Caption("Waiting for the scene ...");
            ImGui::End();
            return;
        }
        const bool running = context.status.runState == RunState::Running;
        if (graph != source_ || context.scene->statics != statics_ || running != running_) {
            source_ = graph;
            statics_ = context.scene->statics;
            running_ = running;
            frames_ = tfFrames(*statics_, *graph, running);
        }
        if (frames_.empty()) {
            widgets::Caption("The scene has no robot with bodies, so there is no TF tree.");
            ImGui::End();
            return;
        }
        std::size_t stale = 0;
        std::size_t unpublished = 0;
        for (const TfFrame& frame : frames_) {
            stale += frame.stale ? 1 : 0;
            unpublished += frame.topic.empty() && frame.kind != TfFrame::Kind::Odom ? 1 : 0;
        }
        // One line on the whole tree, then a warning for what is stale or not published.
        widgets::Caption("%zu frames · /tf and /tf_static as the graph reports them",
                         frames_.size());
        if (stale > 0) {
            ImGui::SameLine();
            char text[48];
            std::snprintf(text, sizeof(text), "%zu stale", stale);
            widgets::Pill(text, widgets::PillKind::Error);
        }
        if (unpublished > 0) {
            ImGui::SameLine();
            char text[48];
            std::snprintf(text, sizeof(text), "%zu not on /tf", unpublished);
            widgets::Pill(text, widgets::PillKind::Warn);
        }
        const float details = px(170.0F);
        ImGui::BeginChild("##tree", ImVec2(0.0F, -details));
        int hiddenBelow = -1;
        const TfFrame* chosen = nullptr;
        for (std::size_t i = 0; i < frames_.size(); ++i) {
            const TfFrame& frame = frames_[i];
            if (hiddenBelow >= 0 && frame.depth > hiddenBelow) {
                continue;
            }
            hiddenBelow = -1;
            const bool hasChildren =
                i + 1 < frames_.size() && frames_[i + 1].parent == static_cast<int>(i);
            const bool folded = folded_.contains(frame.name);
            if (folded) {
                hiddenBelow = frame.depth;
            }
            // The type column: how it moves and at what rate.
            char type[96];
            if (frame.kind == TfFrame::Kind::Odom) {
                std::snprintf(type, sizeof(type), "root");
            } else if (frame.topic.empty()) {
                std::snprintf(type, sizeof(type), "not published");
            } else if (frame.rateKnown) {
                std::snprintf(type, sizeof(type), "%s · %.0f Hz", frame.topic.c_str(),
                              frame.rateHz);
            } else {
                std::snprintf(type, sizeof(type), "%s",
                              frame.topic == "/tf_static" ? "static" : frame.topic.c_str());
            }
            widgets::TreeRowSpec row;
            row.depth = frame.depth;
            row.hasChildren = hasChildren;
            row.folded = folded;
            row.selected = selected_ == frame.name;
            row.icon = frame.kind == TfFrame::Kind::Sensor || frame.kind == TfFrame::Kind::Optical
                           ? widgets::Icon::Sensors
                           : widgets::Icon::Frames;
            row.name = frame.name.c_str();
            row.strong = frame.kind == TfFrame::Kind::Odom || row.selected;
            row.type = type;
            row.error = frame.stale;
            ImGui::PushID(static_cast<int>(i));
            const widgets::TreeHit hit = widgets::TreeRow(frame.name.c_str(), row);
            ImGui::PopID();
            if (!frame.note.empty()) {
                ImGui::SetItemTooltip("%s", frame.note.c_str());
            }
            if (hit == widgets::TreeHit::Caret) {
                if (!folded_.erase(frame.name)) {
                    folded_.insert(frame.name);
                }
            } else if (hit != widgets::TreeHit::None) {
                selected_ = frame.name;
                if (frame.prim != kNone) {
                    context.selectedPrim = frame.prim;  // the same body in the Scene workspace
                }
            }
        }
        for (const TfFrame& frame : frames_) {
            if (selected_ == frame.name) {
                chosen = &frame;
            }
        }
        ImGui::EndChild();
        if (chosen == nullptr) {
            widgets::Caption("Select a frame to see who publishes it.");
        } else if (widgets::SectionHeader(chosen->name.c_str(), kindName(chosen->kind))) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow(
                "parent",
                chosen->parent < 0 ? "-"
                                   : frames_[static_cast<std::size_t>(chosen->parent)].name.c_str(),
                nullptr);
            widgets::PropertyRow("topic", chosen->topic.empty() ? "-" : chosen->topic.c_str(),
                                 nullptr);
            widgets::PropertyRow("published by",
                                 chosen->publisher.empty() ? "-" : chosen->publisher.c_str(),
                                 nullptr);
            char text[48];
            if (chosen->rateKnown) {
                std::snprintf(text, sizeof(text), "%.1f", chosen->rateHz);
                widgets::PropertyRow("rate", text, "Hz");
            }
            if (!chosen->note.empty()) {
                ImGui::PushTextWrapPos(0.0F);
                if (chosen->stale) {
                    widgets::Pill("stale", widgets::PillKind::Error);
                } else if (chosen->topic.empty()) {
                    widgets::Pill("not published", widgets::PillKind::Warn);
                }
                widgets::Caption("%s", chosen->note.c_str());
                ImGui::PopTextWrapPos();
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    std::shared_ptr<const RosGraphInfo> source_;
    std::shared_ptr<const SceneStatics> statics_;
    bool running_{false};
    std::vector<TfFrame> frames_;
    std::set<std::string, std::less<>> folded_;
    std::string selected_;
};

// --- ros2_control and time ----------------------------------------------------------------

class ControlPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "ros2_control"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Ros2; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const std::shared_ptr<const RosGraphInfo> graph = graphOrHint(context);
        if (!graph) {
            ImGui::End();
            return;
        }
        char text[96];
        if (!graph->hasControl) {
            widgets::Caption("This run has no ros2_control hardware interface.");
        } else {
            ImGui::PushFont(theme::fonts().sansSemiBold, theme::kTitleSize);
            ImGui::TextUnformatted(graph->hardware.c_str());
            ImGui::PopFont();
            const char* state = graph->controlActive ? "active" : "inactive";
            ImGui::SameLine();
            widgets::AlignRight(widgets::PillWidth(state));
            widgets::Pill(state,
                          graph->controlActive ? widgets::PillKind::Ok : widgets::PillKind::Warn);
            char hz[24];
            format::step(graph->controlRateHz > 0.0
                             ? static_cast<std::int64_t>(1e9 / graph->controlRateHz)
                             : 0,
                         hz, sizeof(hz));
            widgets::Mono(c::t3, "%s", graph->plugin.c_str());
            widgets::Mono(c::t3, "read/write every %s", hz);
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "Effort commands go through the electrical domain, not straight to "
                "the joints.");
            ImGui::PopTextWrapPos();
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            if (ImGui::BeginTable(
                    "##joints", 3,
                    ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("joint", ImGuiTableColumnFlags_None, 2.4F);
                ImGui::TableSetupColumn("command", ImGuiTableColumnFlags_None, 1.0F);
                ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_None, 1.0F);
                ImGui::TableNextRow();
                for (const char* name : {"joint · interface", "command", "state"}) {
                    ImGui::TableNextColumn();
                    widgets::Caption("%s", name);
                }
                for (const RosGraphInfo::Joint& joint : graph->joints) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                    ImGui::TableNextColumn();
                    // Cut to its cell with an ellipsis, all of it on hover (it was clipped).
                    std::snprintf(text, sizeof(text), "%s · %s", joint.name.c_str(),
                                  joint.interface.c_str());
                    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
                    fit::text(text, ImGui::GetContentRegionAvail().x);
                    ImGui::PopFont();
                    ImGui::TableNextColumn();
                    std::snprintf(text, sizeof(text), "%.2f", joint.command);
                    rightMono(c::t1, text);
                    ImGui::TableNextColumn();
                    std::snprintf(text, sizeof(text), "%.2f", joint.state);
                    rightMono(c::t1, text);
                }
                ImGui::EndTable();
            }
            if (!graph->joints.empty()) {
                widgets::Caption("command → state in %s", graph->joints.front().unit.c_str());
            }
            if (widgets::SectionHeader("Controllers", nullptr)) {
                for (const RosGraphInfo::Controller& controller : graph->controllers) {
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted(controller.name.c_str());
                    std::snprintf(text, sizeof(text), "%.0f Hz", controller.rateHz);
                    ImGui::SameLine(px(190.0F));
                    widgets::Mono(c::t3, "%s", text);
                    ImGui::SameLine();
                    widgets::AlignRight(widgets::PillWidth(controller.state.c_str()));
                    widgets::Pill(controller.state.c_str(), controller.state == "active"
                                                                ? widgets::PillKind::Ok
                                                                : widgets::PillKind::Neutral);
                }
            }
        }
        if (widgets::SectionHeader("Simulated time", nullptr)) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("use_sim_time", graph->useSimTime ? "true" : "false", nullptr);
            std::snprintf(text, sizeof(text), "%.3f",
                          static_cast<double>(context.status.simTimeNs) * 1e-9);
            widgets::PropertyRow("/clock", text, "s");
            std::snprintf(text, sizeof(text), "%.2f × · %.2f", context.status.realTimeFactor,
                          context.status.realTimeFactorTarget);
            widgets::PropertyRow("RTF · target", text, nullptr);
            ImGui::Unindent(px(8.0F));
        }
        if (widgets::SectionHeader("Domain", nullptr)) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("distro", graph->distro.c_str(), nullptr);
            std::snprintf(text, sizeof(text), "%u", graph->domainId);
            widgets::PropertyRow("ROS_DOMAIN_ID", text, nullptr);
            widgets::PropertyRow("RMW", graph->rmw.c_str(), nullptr);
            const auto topics = std::ranges::count_if(
                graph->topics,
                [](const RosGraphInfo::Topic& topic) { return !hiddenName(topic.name); });
            std::snprintf(text, sizeof(text), "%zu nodes · %zu topics", graph->nodes.size(),
                          static_cast<std::size_t>(topics));
            widgets::PropertyRow("graph", text, nullptr);
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }
};

}  // namespace

namespace {

/// A topic the command palette asked to show; the Topics panel takes it.
std::string& requestedTopic() {
    static std::string topic;
    return topic;
}

}  // namespace

void requestRosTopic(std::string name) {
    requestedTopic() = std::move(name);
}

std::vector<std::unique_ptr<Panel>> createRos2Panels() {
    auto selected = std::make_shared<std::string>();  // the topic chosen in the table
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<TopicsPanel>(selected));
    panels.push_back(
        std::make_unique<UnreportedPanel>("Services", "services", "ros2 service list -t"));
    panels.push_back(std::make_unique<ActionsPanel>());
    panels.push_back(std::make_unique<UnreportedPanel>("Params", "parameters", "ros2 param list"));
    panels.push_back(std::make_unique<NodeGraphPanel>(selected));
    panels.push_back(std::make_unique<TfTreePanel>());
    panels.push_back(std::make_unique<ControlPanel>());
    return panels;
}

}  // namespace lagrange::viewer
