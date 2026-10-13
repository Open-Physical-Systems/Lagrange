// ROS 2 workspace: topics and their rates, the node graph, and the ros2_control interface
// (docs/design/ui/09-ros2/SPEC.md). The viewer does not join the ROS domain: everything
// comes from the simulation server's graph report.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
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
                if ((scope_ == 0 && !topic.fromBridge) ||
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
                for (std::size_t to = 0; to < graph->nodes.size(); ++to) {
                    const auto& wanted = graph->nodes[to].subscribes;
                    if (from == to || std::ranges::find(wanted, topic) == wanted.end()) {
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
                std::snprintf(line, sizeof(line), "pub %zu · sub %zu", node.publishes.size(),
                              node.subscribes.size());
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
            widgets::Pill(graph->note.c_str(), widgets::PillKind::Warn);
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    std::shared_ptr<std::string> selected_;
    std::vector<ImVec2> places_;
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
                    widgets::Mono(c::t1, "%s · %s", joint.name.c_str(), joint.interface.c_str());
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
            std::snprintf(text, sizeof(text), "%zu nodes · %zu topics", graph->nodes.size(),
                          graph->topics.size());
            widgets::PropertyRow("graph", text, nullptr);
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createRos2Panels() {
    auto selected = std::make_shared<std::string>();  // the topic chosen in the table
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<TopicsPanel>(selected));
    panels.push_back(std::make_unique<NodeGraphPanel>(selected));
    panels.push_back(std::make_unique<ControlPanel>());
    return panels;
}

}  // namespace lagrange::viewer
