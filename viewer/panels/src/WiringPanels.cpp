// Domains workspace, centre: how the modules would be wired (graph and unit checks) and how
// they would be scheduled (docs/design/ui/03-domains/SPEC.md). All three draw the wiring the
// server sent with its validation of the profile being edited.
#include "PanelFormat.hpp"
#include "ScheduleChart.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr float kNodeHeight = 96.0F;
constexpr float kRowPitch = 176.0F;
constexpr float kMaxNodeWidth = 190.0F;
constexpr float kMinNodeWidth = 150.0F;
constexpr float kMinGap = 56.0F;  // between columns: room for a wire and its label

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// One line of guidance when there is no wiring to show. Returns true if it wrote one.
bool emptyState(const PanelContext& context) {
    const ProfileSession& session = *context.profileSession;
    if (!session.wiring().empty()) {
        return false;
    }
    const char* text = "Waiting for the server to check the profile ...";
    if (!session.profile()) {
        text = context.connection.state == ConnectionState::Ready
                   ? "Waiting for the server's profile ..."
                   : "Not connected. The wiring appears when a simulation server is reachable.";
    } else if (session.wiringIsCurrent()) {
        text = "This server does not describe its wiring.";
    }
    widgets::Caption("%s", text);
    return true;
}

/// Where a module sits in the graph, as (column, row). The eight domains and the common
/// extra modules have the places they have in the design; anything else is put below.
struct Slot {
    std::string_view module;
    int column;
    int row;
};
constexpr std::array<Slot, 10> kSlots{{
    {"aerodynamics", 1, 0},
    {"weather", 2, 0},
    {"electrical", 0, 1},
    {"rigid_body", 1, 1},
    {"sensors", 2, 1},
    {"thermal", 0, 2},
    {"comms", 1, 2},
    {"emi", 2, 2},
    {"radiation", 0, 3},
    {"traffic", 1, 3},
}};

std::string joined(const std::vector<std::string>& items) {
    std::string text;
    for (std::size_t i = 0; i < items.size(); ++i) {
        // The same symbol may come from several wires; list it once.
        if (std::find(items.begin(), items.begin() + static_cast<std::ptrdiff_t>(i), items[i]) !=
            items.begin() + static_cast<std::ptrdiff_t>(i)) {
            continue;
        }
        text += text.empty() ? "" : ", ";
        text += items[i];
    }
    return text;
}

// --- graph --------------------------------------------------------------------------------

class WiringPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Wiring"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        ProfileSession& session = *context.profileSession;
        // Design: a pill in the header while the graph shows edits that are not applied.
        if (session.draft().dirty()) {
            const char* pill = "preview — not applied";
            const float width = widgets::PillWidth(pill);
            if (widgets::BeginPanelTools("##wiring-tools", width + px(4.0F))) {
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + px(2.0F));
                widgets::Pill(pill, widgets::PillKind::Accent);
                widgets::EndPanelTools();
            }
        }
        if (session.wiring().empty()) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            emptyState(context);
            ImGui::End();
            return;
        }
        const WiringInfo& wiring = session.wiring();
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##canvas", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::PopStyleColor();
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImDrawList* draw = ImGui::GetWindowDrawList();

        // Three columns that fit the panel: nodes shrink a little before the canvas scrolls.
        const float available = ImGui::GetContentRegionAvail().x - px(48.0F);
        nodeWidth_ = std::clamp((available - px(kMinGap) * 2.0F) / 3.0F, px(kMinNodeWidth),
                                px(kMaxNodeWidth));
        const float pitch =
            nodeWidth_ + std::max((available - nodeWidth_ * 3.0F) * 0.5F, px(kMinGap));
        // Place the nodes.
        places_.resize(wiring.nodes.size());
        int extraRow = 4;
        int extraColumn = 0;
        float width = 0.0F;
        float height = 0.0F;
        for (std::size_t i = 0; i < wiring.nodes.size(); ++i) {
            int column = -1;
            int row = 0;
            for (const Slot& slot : kSlots) {
                if (slot.module == wiring.nodes[i].module) {
                    column = slot.column;
                    row = slot.row;
                }
            }
            if (column < 0) {
                column = extraColumn;
                row = extraRow;
                extraRow += ++extraColumn / 3;
                extraColumn %= 3;
            }
            places_[i] = ImVec2(origin.x + px(24.0F) + pitch * static_cast<float>(column),
                                origin.y + px(20.0F) + px(kRowPitch) * static_cast<float>(row));
            width = std::max(width, places_[i].x + nodeWidth_ - origin.x);
            height = std::max(height, places_[i].y + px(kNodeHeight) - origin.y);
        }
        const auto indexOf = [&](const std::string& module) {
            for (std::size_t i = 0; i < wiring.nodes.size(); ++i) {
                if (wiring.nodes[i].module == module) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        };

        // Wires first, so that nodes cover their ends.
        for (const WiringInfo::Wire& wire : wiring.wires) {
            const int from = indexOf(wire.from);
            const int to = indexOf(wire.to);
            if (from >= 0 && to >= 0) {
                drawWire(draw, wire, wiring.nodes[static_cast<std::size_t>(from)],
                         places_[static_cast<std::size_t>(from)],
                         places_[static_cast<std::size_t>(to)]);
            }
        }
        for (std::size_t i = 0; i < wiring.nodes.size(); ++i) {
            drawNode(session, draw, wiring.nodes[i], places_[i]);
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::Dummy(ImVec2(width + px(24.0F), height + px(24.0F)));
        ImGui::EndChild();
        ImGui::End();
    }

private:
    /// A wire from the writer to the reader, with "symbol · unit" at its middle. Wires out
    /// of a substitute are dashed in the accent; a unit conflict is red.
    void drawWire(ImDrawList* draw, const WiringInfo::Wire& wire, const WiringInfo::Node& writer,
                  const ImVec2& fromNode, const ImVec2& toNode) const {
        const ImVec2 size(nodeWidth_, px(kNodeHeight));
        ImVec2 a(fromNode.x + size.x * 0.5F, fromNode.y + size.y * 0.5F);
        ImVec2 b(toNode.x + size.x * 0.5F, toNode.y + size.y * 0.5F);
        // Two modules that feed each other get two wires side by side, not one on the other.
        const bool backward = wire.from > wire.to;
        const float shift = px(backward ? 14.0F : -14.0F);
        if (std::abs(a.x - b.x) > std::abs(a.y - b.y)) {
            a.y += shift;
            b.y += shift;
            a.x += a.x < b.x ? size.x * 0.5F : -size.x * 0.5F;
            b.x += a.x < b.x ? -size.x * 0.5F : size.x * 0.5F;
        } else {
            a.x += shift;
            b.x += shift;
            a.y += a.y < b.y ? size.y * 0.5F : -size.y * 0.5F;
            b.y += a.y < b.y ? -size.y * 0.5F : size.y * 0.5F;
        }
        ImVec4 color = c::lineStrong;
        if (!wire.unitOk) {
            color = c::error;
        } else if (writer.substitute) {
            color = c::accent;
        }
        const float length = std::hypot(b.x - a.x, b.y - a.y);
        if (length < 1.0F) {
            return;
        }
        const ImVec2 unit((b.x - a.x) / length, (b.y - a.y) / length);
        if (writer.substitute) {
            for (float at = 0.0F; at < length; at += px(7.0F)) {
                const float end = std::min(at + px(4.0F), length);
                draw->AddLine(ImVec2(a.x + unit.x * at, a.y + unit.y * at),
                              ImVec2(a.x + unit.x * end, a.y + unit.y * end), u32(color), px(1.5F));
            }
        } else {
            draw->AddLine(a, b, u32(color), px(1.5F));
        }
        // Arrow head at the reader.
        const float head = px(6.0F);
        const ImVec2 normal(-unit.y, unit.x);
        draw->AddTriangleFilled(b,
                                ImVec2(b.x - unit.x * head + normal.x * head * 0.5F,
                                       b.y - unit.y * head + normal.y * head * 0.5F),
                                ImVec2(b.x - unit.x * head - normal.x * head * 0.5F,
                                       b.y - unit.y * head - normal.y * head * 0.5F),
                                u32(color));
        // Label: what flows and in which unit; both units when they disagree.
        char label[96];
        if (wire.unitOk) {
            std::snprintf(label, sizeof(label), "%s · %s", wire.symbol.c_str(),
                          wire.sourceUnit.c_str());
        } else {
            std::snprintf(label, sizeof(label), "%s · %s -> %s", wire.symbol.c_str(),
                          wire.sourceUnit.c_str(), wire.sinkUnit.c_str());
        }
        ImFont* font = theme::fonts().mono;
        const float text = px(theme::kCaptionSize);
        const ImVec2 extent = font->CalcTextSizeA(text, FLT_MAX, 0.0F, label);
        const ImVec2 mid((a.x + b.x) * 0.5F, (a.y + b.y) * 0.5F);
        const ImVec2 at(
            mid.x - extent.x * 0.5F + normal.x * px(2.0F),
            mid.y - extent.y * 0.5F +
                (std::abs(unit.x) > 0.5F ? -px(9.0F) : 0.0F) * (backward ? -1.0F : 1.0F));
        draw->AddRectFilled(ImVec2(at.x - px(3.0F), at.y - px(1.0F)),
                            ImVec2(at.x + extent.x + px(3.0F), at.y + extent.y + px(1.0F)),
                            u32(c::bg0));
        draw->AddText(font, text, at,
                      u32(wire.unitOk ? (writer.substitute ? c::accent : c::t3) : c::error), label);
        if (wire.stiff && !backward) {
            // A tightly coupled pair: say how the scheduler treats it (design: warn pill).
            const char* stiff = "stiff · implicit";
            ImFont* sans = theme::fonts().sansMedium;
            const ImVec2 pill = sans->CalcTextSizeA(text, FLT_MAX, 0.0F, stiff);
            const ImVec2 min(mid.x - pill.x * 0.5F - px(7.0F), mid.y + px(4.0F));
            const ImVec2 max(min.x + pill.x + px(14.0F), min.y + px(18.0F));
            draw->AddRectFilled(min, max, u32(theme::hex(0x241E12)), px(9.0F));
            draw->AddText(sans, text, ImVec2(min.x + px(7.0F), min.y + px(3.0F)), u32(c::warn),
                          stiff);
        }
    }

    /// A module (design: .node): header with the domain dot, name and detail; then what it
    /// reads and writes and its step. A substitute has a dashed accent border.
    void drawNode(ProfileSession& session, ImDrawList* draw, const WiringInfo::Node& node,
                  const ImVec2& min) const {
        const ImVec2 max(min.x + nodeWidth_, min.y + px(kNodeHeight));
        ImGui::SetCursorScreenPos(min);
        if (ImGui::InvisibleButton(node.module.c_str(), ImVec2(nodeWidth_, px(kNodeHeight)))) {
            session.selectedDomain = node.domain;  // the same selection as the Domains table
        }
        const bool selected = session.selectedDomain == node.domain;
        const float rounding = px(4.0F);
        draw->AddRectFilled(min, max, u32(node.substitute ? c::bg1 : c::bg2), rounding);
        const ImU32 border =
            u32(node.substitute ? c::accent : (selected ? c::accent : c::lineStrong));
        if (node.substitute) {
            for (float x = min.x; x < max.x; x += px(7.0F)) {
                draw->AddLine(ImVec2(x, min.y), ImVec2(std::min(x + px(4.0F), max.x), min.y),
                              border);
                draw->AddLine(ImVec2(x, max.y), ImVec2(std::min(x + px(4.0F), max.x), max.y),
                              border);
            }
            for (float y = min.y; y < max.y; y += px(7.0F)) {
                draw->AddLine(ImVec2(min.x, y), ImVec2(min.x, std::min(y + px(4.0F), max.y)),
                              border);
                draw->AddLine(ImVec2(max.x, y), ImVec2(max.x, std::min(y + px(4.0F), max.y)),
                              border);
            }
        } else {
            draw->AddRect(min, max, border, rounding, selected ? px(2.0F) : 1.0F);
        }
        const float header = px(24.0F);
        draw->AddLine(ImVec2(min.x, min.y + header), ImVec2(max.x, min.y + header), u32(c::line));

        const theme::Fonts& fonts = theme::fonts();
        const float label = px(theme::kLabelSize);
        const float small = px(theme::kCaptionSize);
        const ImVec4 domain = widgets::domainColor(node.domain);
        const ImVec2 dot(min.x + px(12.0F), min.y + header * 0.5F);
        if (node.substitute) {
            draw->AddCircle(dot, px(3.5F), u32(c::accent), 0, px(1.5F));
        } else {
            draw->AddCircleFilled(dot, px(4.0F), u32(domain));
        }
        draw->PushClipRect(min, max, true);
        char name[96];
        if (node.substitute) {
            std::snprintf(name, sizeof(name), "%s → %s", node.module.c_str(), node.detail.c_str());
        } else {
            std::snprintf(name, sizeof(name), "%s", node.module.c_str());
        }
        draw->AddText(fonts.sansSemiBold, label,
                      ImVec2(min.x + px(22.0F), min.y + (header - label) * 0.5F - px(1.0F)),
                      u32(node.substitute ? c::accent : c::t1), name);
        const char* tag = node.substitute ? "sub" : node.detail.c_str();
        const float tagWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, tag).x;
        draw->AddText(fonts.mono, small,
                      ImVec2(max.x - tagWidth - px(8.0F), min.y + (header - small) * 0.5F),
                      u32(node.substitute ? c::accent : c::t3), tag);

        char line[160];
        float y = min.y + header + px(7.0F);
        std::snprintf(line, sizeof(line), "in  %s",
                      node.inputs.empty() ? "-" : joined(node.inputs).c_str());
        draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), y), u32(c::t2), line);
        y += px(19.0F);
        std::snprintf(line, sizeof(line), "out %s",
                      node.outputs.empty() ? "-" : joined(node.outputs).c_str());
        draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), y), u32(c::t2), line);
        y += px(19.0F);
        if (node.substitute) {
            std::snprintf(line, sizeof(line), "replaces the module");
        } else if (node.dtNs > 0) {
            char step[24];
            format::step(node.dtNs, step, sizeof(step));
            std::snprintf(line, sizeof(line), "%s%s", step, node.onGpu ? " · GPU" : "");
        } else {
            std::snprintf(line, sizeof(line), "%s", node.onGpu ? "per frame · GPU" : "on demand");
        }
        draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), y), u32(c::t3), line);
        draw->PopClipRect();
    }

    std::vector<ImVec2> places_;
    float nodeWidth_{190.0F};  // in pixels, set every frame
};

// --- unit checks ---------------------------------------------------------------------------

class UnitChecksPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Unit checks"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        const ProfileSession& session = *context.profileSession;
        // The tab says how many wires fail, so a conflict is seen without opening it.
        char label[64];
        const std::size_t conflicts = session.unitConflicts();
        if (conflicts == 0) {
            std::snprintf(label, sizeof(label), "Unit checks###UnitChecks");
        } else {
            std::snprintf(label, sizeof(label), "Unit checks · %zu fail###UnitChecks", conflicts);
        }
        if (!ImGui::Begin(label)) {
            ImGui::End();
            return;
        }
        if (emptyState(context)) {
            ImGui::End();
            return;
        }
        const WiringInfo& wiring = session.wiring();
        widgets::Pill(conflicts == 0 ? "all wires agree" : "unit conflict · Apply is blocked",
                      conflicts == 0 ? widgets::PillKind::Ok : widgets::PillKind::Error);
        ImGui::SameLine(0.0F, px(10.0F));
        widgets::Caption("%zu wires  ·  a unit must match exactly; nothing is converted",
                         wiring.wires.size());
        if (ImGui::BeginTable("##units", 5,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoPadOuterX)) {
            ImGui::TableSetupColumn("writer", ImGuiTableColumnFlags_None, 1.4F);
            ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_None, 2.4F);
            ImGui::TableSetupColumn("writes", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("reader", ImGuiTableColumnFlags_None, 1.4F);
            ImGui::TableSetupColumn("expects", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            for (const char* name : {"writer", "signal", "writes", "reader", "expects"}) {
                ImGui::TableNextColumn();
                widgets::Caption("  %s", name);
            }
            for (const WiringInfo::Wire& wire : wiring.wires) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                widgets::Dot(widgets::domainColor(wire.from));
                ImGui::SameLine(0.0F, px(6.0F));
                ImGui::TextUnformatted(wire.from.c_str());
                ImGui::TableNextColumn();
                widgets::Mono(c::t1, "%s", wire.signal.c_str());
                ImGui::TableNextColumn();
                widgets::Mono(c::t2, "[%s]", wire.sourceUnit.c_str());
                ImGui::TableNextColumn();
                widgets::Dot(widgets::domainColor(wire.to));
                ImGui::SameLine(0.0F, px(6.0F));
                ImGui::TextUnformatted(wire.to.c_str());
                ImGui::TableNextColumn();
                widgets::Mono(wire.unitOk ? c::t2 : c::error, "[%s]", wire.sinkUnit.c_str());
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::IconItem(wire.unitOk ? widgets::Icon::Check : widgets::Icon::Warning,
                                  12.0F, wire.unitOk ? c::ok : c::error);
            }
            ImGui::EndTable();
        }
        ImGui::End();
    }
};

// --- schedule -----------------------------------------------------------------------------

class SchedulePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Schedule"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (emptyState(context)) {
            ImGui::End();
            return;
        }
        const WiringInfo& wiring = context.profileSession->wiring();
        char sync[24];
        format::step(wiring.syncNs, sync, sizeof(sync));
        widgets::Caption("10 ms window  ·  modules exchange signals every %s", sync);

        rows_.clear();
        for (const WiringInfo::Node& node : wiring.nodes) {
            if (!node.substitute) {  // a substitute costs nothing and is not scheduled
                rows_.push_back({node.module.c_str(), node.domain, node.dtNs, node.onGpu});
            }
        }
        drawScheduleChart(rows_, wiring.syncNs);
        ImGui::End();
    }

private:
    std::vector<ScheduleRow> rows_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createWiringPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<WiringPanel>());
    panels.push_back(std::make_unique<UnitChecksPanel>());
    panels.push_back(std::make_unique<SchedulePanel>());
    return panels;
}

}  // namespace lagrange::viewer
