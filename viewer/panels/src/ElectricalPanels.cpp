// Electrical workspace: the network table, the schematic with live current flow, and the
// scopes of the bus and its branches (docs/design/ui/04-electrical/SPEC.md).
//
// Everything is built from the stage's electrical nets (SceneStatics::nets) and from signals:
// nothing here knows a particular robot.
#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::size_t kNodeSignals = 2;             // further live values shown in a schematic node
constexpr std::int64_t kRecentNs = 10'000'000'000;  // "last 10 s" of the bus summary
constexpr double kSummaryIntervalSeconds = 0.2;
// Branch currents are compared with the source current; this much disagreement is noise.
constexpr double kBalanceToleranceAmperes = 0.05;

const ImVec4 kDischarge = theme::kDomainColor[static_cast<int>(theme::Domain::Electrical)];
const ImVec4 kRegen = c::info;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// A further live value of a component, shown in its schematic node.
struct NodeSignal {
    std::string label;
    std::string unit;
    std::shared_ptr<ScopeBuffer> buffer;
};

struct TerminalLive {
    std::shared_ptr<ScopeBuffer> current;  // may be null
    std::vector<NodeSignal> signals;
};

struct NetLive {
    std::shared_ptr<ScopeBuffer> voltage;  // may be null
    std::vector<TerminalLive> terminals;   // parallel to ElectricalNet::terminals
};

double latest(const std::shared_ptr<ScopeBuffer>& buffer) {
    return buffer ? buffer->latest() : NAN;
}

void number(double value, const char* format, char* out, std::size_t size) {
    if (std::isnan(value)) {
        std::snprintf(out, size, "-");
    } else {
        std::snprintf(out, size, format, value);
    }
}

/// What the three panels share: the subscriptions for the stage's nets and the scope traces.
class ElectricalState {
public:
    /// Call from every panel's draw(). Subscribes again when the stage or the signal list
    /// changed, and advances the scopes once per frame.
    void follow(PanelContext& context) {
        const std::uint64_t scene = context.scene ? context.scene->statics->generation : 0;
        const std::uint64_t catalog = context.catalog ? context.catalog->generation : 0;
        if (scene != scene_ || catalog != catalog_) {
            scene_ = scene;
            catalog_ = catalog;
            subscribe(context);
        }
        if (const int frame = ImGui::GetFrameCount(); frame != frame_) {
            frame_ = frame;
            scopes.update();
        }
    }

    std::vector<NetLive> nets;  // parallel to SceneStatics::nets
    ScopeSession scopes;
    ScopePlotter plotter;

private:
    void subscribe(PanelContext& context) {
        for (const NetLive& net : nets) {
            context.client.unsubscribe(net.voltage);
            for (const TerminalLive& terminal : net.terminals) {
                context.client.unsubscribe(terminal.current);
                for (const NodeSignal& signal : terminal.signals) {
                    context.client.unsubscribe(signal.buffer);
                }
            }
        }
        nets.clear();
        while (!scopes.traces().empty()) {
            scopes.remove(context.client, scopes.traces().size() - 1);
        }
        if (!context.scene || !context.catalog) {
            return;
        }
        const SignalCatalog& catalog = *context.catalog;
        for (const ElectricalNet& net : context.scene->statics->nets) {
            NetLive live;
            if (catalog.find(net.voltageSignal) != nullptr) {
                live.voltage = context.client.subscribe(net.voltageSignal);
                scopes.add(context.client, catalog, net.voltageSignal);
            }
            for (const NetTerminal& terminal : net.terminals) {
                TerminalLive entry;
                if (catalog.find(terminal.currentSignal) != nullptr) {
                    entry.current = context.client.subscribe(terminal.currentSignal);
                    scopes.add(context.client, catalog, terminal.currentSignal);
                }
                // Other scalar signals of the same prim: state of charge, temperature, ...
                for (const SignalInfo& info : catalog.signals) {
                    if (info.prim != terminal.prim || info.components != 1 ||
                        info.name == terminal.currentSignal ||
                        entry.signals.size() >= kNodeSignals) {
                        continue;
                    }
                    NodeSignal signal;
                    signal.buffer = context.client.subscribe(info.name);
                    if (!signal.buffer) {
                        continue;
                    }
                    signal.label = info.name.substr(info.name.rfind('/') + 1);
                    std::ranges::replace(signal.label, '_', ' ');
                    signal.unit = info.unit;
                    entry.signals.push_back(std::move(signal));
                }
                live.terminals.push_back(std::move(entry));
            }
            nets.push_back(std::move(live));
        }
    }

    std::uint64_t scene_{0};
    std::uint64_t catalog_{0};
    int frame_{-1};
};

/// One line of guidance when there is nothing to show. Returns true if it wrote one.
bool emptyState(const PanelContext& context, const ElectricalState& state) {
    if (!context.scene) {
        widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                   ? "Waiting for the scene ..."
                                   : "Not connected. The power system appears when a "
                                     "simulation server is reachable.");
        return true;
    }
    if (context.scene->statics->nets.empty() || state.nets.empty()) {
        widgets::Caption("%s", context.scene->statics->nets.empty()
                                   ? "This stage has no electrical topology."
                                   : "Waiting for the signal list ...");
        return true;
    }
    return false;
}

/// True when the profile being run has the electrical domain switched off.
bool electricalOff(const PanelContext& context) {
    const DomainView* view = context.profileSession->find("electrical");
    return view != nullptr && !view->enabled && !view->pending;
}

// --- Network ------------------------------------------------------------------------------

class NetworkPanel final : public Panel {
public:
    explicit NetworkPanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Network"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (emptyState(context, *state_)) {
            ImGui::End();
            return;
        }
        const SceneStatics& statics = *context.scene->statics;
        if (ImGui::BeginTable("##network", 4,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_NoPadOuterX)) {
            ImGui::TableSetupColumn("component", ImGuiTableColumnFlags_None, 2.6F);
            ImGui::TableSetupColumn("V", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("I", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("P", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            for (const char* name : {"component", "V", "I", "P"}) {
                ImGui::TableNextColumn();
                if (name[1] == '\0') {
                    rightCaption(name);
                } else {
                    widgets::Caption("  %s", name);
                }
            }
            for (std::size_t n = 0; n < statics.nets.size() && n < state_->nets.size(); ++n) {
                const ElectricalNet& net = statics.nets[n];
                const NetLive& live = state_->nets[n];
                const double volts = latest(live.voltage);
                // Sources first, then the bus, then what it feeds (indented), as in the design.
                for (std::size_t t = 0; t < net.terminals.size(); ++t) {
                    if (net.terminals[t].isSource()) {
                        row(context, statics, net.terminals[t].prim, 0, volts,
                            latest(live.terminals[t].current));
                    }
                }
                if (net.busPrim != kNone) {
                    row(context, statics, net.busPrim, 0, volts, NAN, " bus");
                }
                for (std::size_t t = 0; t < net.terminals.size(); ++t) {
                    if (!net.terminals[t].isSource()) {
                        row(context, statics, net.terminals[t].prim, 1, volts,
                            latest(live.terminals[t].current));
                    }
                }
            }
            ImGui::EndTable();
        }
        widgets::Caption("  V in volts  ·  I in amps  ·  P in watts");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        if (widgets::SectionHeader("Limits and events", "last 10 s")) {
            limits(context, statics);
        }
        if (widgets::SectionHeader("Bus summary", "last 10 s")) {
            summary(statics);
        }
        ImGui::End();
    }

private:
    static void rightCaption(const char* text) {
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        widgets::AlignRight(ImGui::CalcTextSize(text).x);
        ImGui::PopFont();
        widgets::Caption("%s", text);
    }

    static void rightMono(const ImVec4& color, const char* text) {
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        widgets::AlignRight(ImGui::CalcTextSize(text).x);
        ImGui::PopFont();
        widgets::Mono(color, "%s", text);
    }

    static void row(PanelContext& context, const SceneStatics& statics, std::uint32_t prim,
                    int depth, double volts, double amperes, const char* suffix = "") {
        ImGui::PushID(static_cast<int>(prim) * 2 + (*suffix != '\0' ? 1 : 0));
        ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
        ImGui::TableNextColumn();
        const bool selected = context.selectedPrim == prim;
        if (ImGui::Selectable(
                "##row", selected,
                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
            context.selectedPrim = prim;  // the selection is global: tree, viewport, inspector
        }
        ImGui::SameLine(px(8.0F) + px(18.0F) * static_cast<float>(depth));
        if (selected) {
            widgets::Dot(kDischarge);
            ImGui::SameLine(0.0F, px(6.0F));
        }
        ImGui::Text("%s%s", statics.prims[prim].name.c_str(), suffix);
        char text[32];
        ImGui::TableNextColumn();
        number(volts, "%.1f", text, sizeof(text));
        rightMono(c::t1, text);
        ImGui::TableNextColumn();
        number(amperes, "%.2f", text, sizeof(text));
        rightMono(std::isnan(amperes) ? c::t3 : c::t1, text);
        ImGui::TableNextColumn();
        number(volts * amperes, "%.0f", text, sizeof(text));
        rightMono(std::isnan(amperes) ? c::t3 : c::t1, text);
        ImGui::PopID();
    }

    /// What the bus did over the last ten seconds, and whether the branch currents add up to
    /// what the sources supply (Kirchhoff): the table shows both, so they have to agree.
    void summary(const SceneStatics& statics) {
        const double now = ImGui::GetTime();
        if (now - computed_ >= kSummaryIntervalSeconds || lines_.size() != state_->nets.size()) {
            computed_ = now;
            lines_.assign(state_->nets.size(), {});
            for (std::size_t n = 0; n < state_->nets.size() && n < statics.nets.size(); ++n) {
                const NetLive& live = state_->nets[n];
                Line& line = lines_[n];
                if (live.voltage && live.voltage->newestTimeNs() >= 0) {
                    const std::int64_t end = live.voltage->newestTimeNs();
                    line.voltage = live.voltage->statistics(end - kRecentNs, end + 1);
                }
                for (std::size_t t = 0; t < live.terminals.size(); ++t) {
                    const double amperes = latest(live.terminals[t].current);
                    if (std::isnan(amperes)) {
                        line.complete = false;
                    } else if (statics.nets[n].terminals[t].isSource()) {
                        line.supplied += amperes;
                    } else {
                        line.drawn += amperes;
                    }
                }
            }
        }
        ImGui::Indent(px(8.0F));
        for (std::size_t n = 0; n < lines_.size() && n < statics.nets.size(); ++n) {
            const Line& line = lines_[n];
            char text[96];
            widgets::Mono(kDischarge, "%s", statics.nets[n].name.c_str());
            if (line.voltage.count != 0) {
                std::snprintf(text, sizeof(text), "%.2f V", line.voltage.min);
                widgets::PropertyRow("lowest", text, nullptr);
                std::snprintf(text, sizeof(text), "%.2f V", line.voltage.max);
                widgets::PropertyRow("highest", text, nullptr);
                std::snprintf(text, sizeof(text), "%.2f V", line.voltage.mean);
                widgets::PropertyRow("mean", text, nullptr);
            }
            if (line.complete) {
                const bool balanced =
                    std::abs(line.supplied - line.drawn) <= kBalanceToleranceAmperes;
                widgets::Pill(balanced ? "ok" : "check",
                              balanced ? widgets::PillKind::Ok : widgets::PillKind::Warn);
                ImGui::SameLine(0.0F, px(8.0F));
                std::snprintf(text, sizeof(text), "branches draw %.2f A  ·  sources supply %.2f A",
                              line.drawn, line.supplied);
                ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
                ImGui::TextUnformatted(text);
                ImGui::PopFont();
            }
        }
        ImGui::Unindent(px(8.0F));
    }

    /// Design: meters for the limits the net's protection acts on (warn colour past 85 %),
    /// then what happened lately in the electrical domain.
    void limits(PanelContext& context, const SceneStatics& statics) const {
        ImGui::Indent(px(8.0F));
        bool any = false;
        for (std::size_t n = 0; n < statics.nets.size() && n < state_->nets.size(); ++n) {
            const ElectricalNet& net = statics.nets[n];
            const NetLive& live = state_->nets[n];
            for (const NetLimit& limit : net.limits) {
                // The value now: from the signal the limit names, which the net already plots.
                double value = NAN;
                if (limit.signal == net.voltageSignal) {
                    value = latest(live.voltage);
                }
                for (std::size_t t = 0; t < net.terminals.size(); ++t) {
                    if (net.terminals[t].currentSignal == limit.signal) {
                        value = latest(live.terminals[t].current);
                    }
                }
                any = true;
                // How close to the limit: 1 means at it. An upper limit is approached from
                // below, a lower one from above.
                double used = 0.0;
                if (!std::isnan(value) && limit.value != 0.0) {
                    used = limit.upper ? std::abs(value) / std::abs(limit.value)
                                       : std::abs(limit.value) / std::max(std::abs(value), 1e-9);
                }
                char text[64];
                if (std::isnan(value)) {
                    std::snprintf(text, sizeof(text), "- / %.1f %s", limit.value,
                                  limit.unit.c_str());
                } else {
                    std::snprintf(text, sizeof(text), "%.1f / %.1f %s", value, limit.value,
                                  limit.unit.c_str());
                }
                ImGui::AlignTextToFramePadding();
                ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
                ImGui::TextUnformatted(limit.label.c_str());
                ImGui::PopStyleColor();
                ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
                const float width = ImGui::CalcTextSize(text).x;
                ImGui::PopFont();
                const float meter = px(84.0F);
                ImGui::SameLine();
                widgets::AlignRight(meter + px(8.0F) + width);
                widgets::Meter(static_cast<float>(used),
                               used >= 1.0 ? c::error : (used > 0.85 ? c::warn : c::ok), meter);
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::Mono(used >= 1.0 ? c::error : c::t1, "%s", text);
            }
        }
        if (!any) {
            widgets::Caption("The nets of this stage declare no limits.");
        }

        // Electrical events of the last ten seconds, newest first.
        const std::shared_ptr<const EventLog> log = context.client.events();
        const std::int64_t now = context.status.simTimeNs;
        int shown = 0;
        for (std::size_t i = log->events.size(); i-- > 0 && shown < 4;) {
            const EventInfo& event = log->events[i];
            if (event.domain != "electrical" || event.beginNs > now ||
                event.endNs < now - kRecentNs) {
                continue;
            }
            widgets::Pill(event.kind.c_str(), event.severity == EventInfo::Severity::Error
                                                  ? widgets::PillKind::Error
                                                  : (event.severity == EventInfo::Severity::Warning
                                                         ? widgets::PillKind::Warn
                                                         : widgets::PillKind::Info));
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
            ImGui::TextUnformatted(event.text.c_str());
            ImGui::PopFont();
            char ago[24];
            std::snprintf(ago, sizeof(ago), "%.1f s",
                          static_cast<double>(event.beginNs - now) * 1e-9);
            ImGui::SameLine();
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            widgets::AlignRight(ImGui::CalcTextSize(ago).x);
            ImGui::PopFont();
            widgets::Mono(c::t3, "%s", ago);
            ++shown;
        }
        if (shown == 0) {
            widgets::Caption("No electrical events in the last 10 s.");
        }
        ImGui::Unindent(px(8.0F));
    }

    struct Line {
        ScopeBuffer::Statistics voltage;
        double supplied{0.0};
        double drawn{0.0};
        bool complete{true};
    };

    std::shared_ptr<ElectricalState> state_;
    std::vector<Line> lines_;
    double computed_{-1.0};
};

// --- Schematic ----------------------------------------------------------------------------

class SchematicPanel final : public Panel {
public:
    explicit SchematicPanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Schematic"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        headerTools();
        if (!context.scene || context.scene->statics->nets.empty() || state_->nets.empty()) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            emptyState(context, *state_);
            ImGui::End();
            return;
        }
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##canvas", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::PopStyleColor();
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 visible = ImGui::GetContentRegionAvail();
        const bool off = electricalOff(context);
        ImDrawList* draw = ImGui::GetWindowDrawList();
        dots(draw, origin, visible);

        // Auto-layout from the topology: sources on the left, the bus as a vertical bar, what
        // it feeds stacked on the right; one net under the other.
        const SceneStatics& statics = *context.scene->statics;
        float top = px(44.0F);
        float width = 0.0F;
        if (off) {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.45F);  // design: greyed out when off
        }
        for (std::size_t n = 0; n < statics.nets.size() && n < state_->nets.size(); ++n) {
            const ImVec2 extent = net(context, draw, statics, statics.nets[n], state_->nets[n],
                                      ImVec2(origin.x + px(24.0F), origin.y + top), off);
            top += extent.y + px(56.0F);
            width = std::max(width, extent.x);
        }
        if (off) {
            ImGui::PopStyleVar();
        }
        // The canvas is as large as the drawing, so that it scrolls when the panel is small.
        ImGui::SetCursorScreenPos(origin);
        ImGui::Dummy(ImVec2(width + px(48.0F), top));

        legend(draw, ImVec2(origin.x + ImGui::GetScrollX() + visible.x - px(150.0F),
                            origin.y + ImGui::GetScrollY() + visible.y - px(62.0F)));
        if (off) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + ImGui::GetScrollX() + px(12.0F),
                                             origin.y + ImGui::GetScrollY() + px(10.0F)));
            widgets::Pill("Electrical is off → ideal torque source. Currents are not simulated.",
                          widgets::PillKind::Warn);
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    static constexpr float kNodeWidth = 212.0F;
    static constexpr float kHeader = 24.0F;
    static constexpr float kLine = 19.0F;
    static constexpr float kWire = 120.0F;  // length of a wire between a node and the bus
    static constexpr float kGap = 22.0F;    // between stacked nodes

    static void headerTools() {
        // The design writes the "proportional to" sign; IBM Plex has no such glyph.
        const char* hint = "wire width follows |I| · dashes move with current";
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        const float hintWidth = ImGui::CalcTextSize(hint).x;
        ImGui::PopFont();
        const float gap = px(6.0F);
        const float width = hintWidth + widgets::ButtonWidth("Export SPICE") +
                            widgets::ButtonWidth("Export Modelica") + gap * 2.0F + px(8.0F);
        if (!widgets::BeginPanelTools("##schematic-tools", width)) {
            return;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("%s", hint);
        ImGui::SameLine(0.0F, px(8.0F));
        // A netlist needs the component models; the viewer only has the topology.
        ImGui::BeginDisabled(true);
        widgets::Button("Export SPICE");
        ImGui::SameLine(0.0F, gap);
        widgets::Button("Export Modelica");
        ImGui::EndDisabled();
        widgets::EndPanelTools();
    }

    /// The dot grid of the canvas.
    static void dots(ImDrawList* draw, const ImVec2& origin, const ImVec2& size) {
        const float step = px(24.0F);
        const ImU32 color = u32(c::line);
        const float left = origin.x + ImGui::GetScrollX();
        const float topEdge = origin.y + ImGui::GetScrollY();
        for (float y = topEdge + std::fmod(step - std::fmod(ImGui::GetScrollY(), step), step);
             y < topEdge + size.y; y += step) {
            for (float x = left + std::fmod(step - std::fmod(ImGui::GetScrollX(), step), step);
                 x < left + size.x; x += step) {
                draw->AddRectFilled(ImVec2(x, y), ImVec2(x + 1.0F, y + 1.0F), color);
            }
        }
    }

    static float nodeHeight(const TerminalLive& live) {
        return px(kHeader) + px(8.0F) + px(kLine) * static_cast<float>(1 + live.signals.size()) +
               px(6.0F);
    }

    /// Draws one net at `at`. Returns the size it took.
    ImVec2 net(PanelContext& context, ImDrawList* draw, const SceneStatics& statics,
               const ElectricalNet& net, const NetLive& live, const ImVec2& at, bool off) {
        // Heights of the two columns.
        float sources = 0.0F;
        float loads = 0.0F;
        for (std::size_t t = 0; t < net.terminals.size(); ++t) {
            float& column = net.terminals[t].isSource() ? sources : loads;
            column += nodeHeight(live.terminals[t]) + px(kGap);
        }
        sources = std::max(sources - px(kGap), 0.0F);
        loads = std::max(loads - px(kGap), 0.0F);
        const float height = std::max({sources, loads, px(80.0F)});
        const float busX = at.x + px(kNodeWidth) + px(kWire);
        const float loadX = busX + px(kWire);
        const double volts = latest(live.voltage);

        // The bus: a vertical bar in the electrical colour, with its name and voltage above.
        const float barWidth = px(6.0F);
        draw->AddRectFilled(ImVec2(busX - barWidth * 0.5F, at.y),
                            ImVec2(busX + barWidth * 0.5F, at.y + height), u32(kDischarge),
                            barWidth * 0.5F);
        const theme::Fonts& fonts = theme::fonts();
        const float mono = px(theme::kMonoSize);
        char text[64];
        number(volts, "%.1f V", text, sizeof(text));
        const float nameWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, net.name.c_str()).x;
        const float valueWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, text).x;
        const float labelX = busX - (nameWidth + px(8.0F) + valueWidth) * 0.5F;
        draw->AddText(fonts.mono, mono, ImVec2(labelX, at.y - px(24.0F)), u32(kDischarge),
                      net.name.c_str());
        draw->AddText(fonts.mono, mono, ImVec2(labelX + nameWidth + px(8.0F), at.y - px(24.0F)),
                      u32(c::t1), text);

        float sourceY = at.y + (height - sources) * 0.5F;
        float loadY = at.y + (height - loads) * 0.5F;
        for (std::size_t t = 0; t < net.terminals.size(); ++t) {
            const NetTerminal& terminal = net.terminals[t];
            const TerminalLive& entry = live.terminals[t];
            const bool source = terminal.isSource();
            const float nodeH = nodeHeight(entry);
            const ImVec2 min(source ? at.x : loadX, source ? sourceY : loadY);
            (source ? sourceY : loadY) += nodeH + px(kGap);
            const double amperes = latest(entry.current);
            const float midY = min.y + nodeH * 0.5F;
            // The wire runs in the direction the current normally flows: source to bus, bus
            // to load. A negative current is regeneration and flows the other way.
            if (source) {
                wire(draw, ImVec2(min.x + px(kNodeWidth), midY), ImVec2(busX, midY), amperes, off);
            } else {
                wire(draw, ImVec2(busX, midY), ImVec2(min.x, midY), amperes, off);
            }
            node(context, draw, statics, terminal, entry, min, nodeH, volts, amperes);
        }
        return {loadX + px(kNodeWidth) - at.x, height};
    }

    /// A wire: width follows |I|, dashes move along the current, blue and reversed for
    /// regeneration, with the current written above it.
    static void wire(ImDrawList* draw, ImVec2 from, ImVec2 to, double amperes, bool off) {
        const bool known = !std::isnan(amperes);
        const bool reversed = known && amperes < 0.0;
        const double magnitude = known ? std::abs(amperes) : 0.0;
        const ImU32 color = u32(reversed ? kRegen : kDischarge);
        const float thickness =
            std::clamp(px(1.5F) + static_cast<float>(magnitude) * px(0.35F), px(1.5F), px(6.0F));
        // A faint solid line under the dashes keeps the connection readable at any phase.
        draw->AddLine(
            from, to,
            u32(ImVec4(reversed ? kRegen.x : kDischarge.x, reversed ? kRegen.y : kDischarge.y,
                       reversed ? kRegen.z : kDischarge.z, 0.30F)),
            thickness);
        const float length = to.x - from.x;
        const float dash = px(9.0F);
        const float period = dash + px(7.0F);
        // Dashes travel faster with more current; they stand still when the domain is off.
        const double speed = off ? 0.0 : 14.0 + std::min(magnitude, 30.0) * 9.0;
        auto phase = static_cast<float>(std::fmod(ImGui::GetTime() * speed, period));
        if (reversed) {
            phase = period - phase;
        }
        for (float x = phase - period; x < length; x += period) {
            const float a = std::max(x, 0.0F);
            const float b = std::min(x + dash, length);
            if (b > a) {
                draw->AddLine(ImVec2(from.x + a, from.y), ImVec2(from.x + b, from.y), color,
                              thickness);
            }
        }
        // Arrow head in the middle, pointing the way the current flows.
        const float midX = (from.x + to.x) * 0.5F;
        const float head = px(5.0F) + thickness * 0.5F;
        const float direction = reversed ? -1.0F : 1.0F;
        draw->AddTriangleFilled(ImVec2(midX + head * direction, from.y),
                                ImVec2(midX - head * direction, from.y - head),
                                ImVec2(midX - head * direction, from.y + head), color);
        char text[32];
        number(magnitude, "%.2f A", text, sizeof(text));
        if (!known) {
            std::snprintf(text, sizeof(text), "- A");
        }
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kCaptionSize);
        draw->AddText(font, size, ImVec2(from.x + px(10.0F), from.y - px(20.0F)),
                      u32(reversed ? kRegen : c::t1), text);
    }

    /// A component (design: .node): bg2 box with a 24 px header and mono rows of live values.
    static void node(PanelContext& context, ImDrawList* draw, const SceneStatics& statics,
                     const NetTerminal& terminal, const TerminalLive& live, const ImVec2& min,
                     float height, double volts, double amperes) {
        const ImVec2 max(min.x + px(kNodeWidth), min.y + height);
        ImGui::SetCursorScreenPos(min);
        ImGui::PushID(static_cast<int>(terminal.prim));
        if (ImGui::InvisibleButton("##node", ImVec2(px(kNodeWidth), height))) {
            context.selectedPrim = terminal.prim;
        }
        ImGui::PopID();
        const bool selected = context.selectedPrim == terminal.prim;
        const float rounding = px(4.0F);
        draw->AddRectFilled(min, max, u32(ImGui::IsItemHovered() ? c::bg3 : c::bg2), rounding);
        draw->AddRect(min, max, u32(selected ? c::accent : c::lineStrong), rounding,
                      selected ? px(2.0F) : 1.0F);
        const float headerBottom = min.y + px(kHeader);
        draw->AddLine(ImVec2(min.x, headerBottom), ImVec2(max.x, headerBottom), u32(c::line));

        const theme::Fonts& fonts = theme::fonts();
        const float label = px(theme::kLabelSize);
        const float small = px(theme::kCaptionSize);
        const float mono = px(theme::kMonoSize);
        draw->AddCircleFilled(ImVec2(min.x + px(12.0F), min.y + px(kHeader) * 0.5F), px(4.0F),
                              u32(kDischarge));
        draw->PushClipRect(min, max, true);
        draw->AddText(fonts.sansSemiBold, label,
                      ImVec2(min.x + px(22.0F), min.y + (px(kHeader) - label) * 0.5F - px(1.0F)),
                      u32(c::t1), statics.prims[terminal.prim].name.c_str());
        const float portWidth =
            fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, terminal.port.c_str()).x;
        draw->AddText(fonts.mono, small,
                      ImVec2(max.x - portWidth - px(8.0F), min.y + (px(kHeader) - small) * 0.5F),
                      u32(c::t3), terminal.port.c_str());

        // First row: terminal voltage, current and power.
        char text[64];
        float y = headerBottom + px(7.0F);
        number(volts, "%.1f V", text, sizeof(text));
        draw->AddText(fonts.mono, mono, ImVec2(min.x + px(10.0F), y), u32(c::t1), text);
        number(amperes, "%.2f A", text, sizeof(text));
        const float currentWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, text).x;
        draw->AddText(fonts.mono, mono, ImVec2(max.x - currentWidth - px(10.0F), y),
                      u32(!std::isnan(amperes) && amperes < 0.0 ? kRegen : c::t1), text);
        // Then the component's other live values, as "name  value unit".
        for (const NodeSignal& signal : live.signals) {
            y += px(kLine);
            const double value = signal.buffer->latest();
            const bool ratio = signal.unit == "1" || signal.unit.empty();
            draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), y + px(1.0F)), u32(c::t3),
                          signal.label.c_str());
            if (std::isnan(value)) {
                std::snprintf(text, sizeof(text), "-");
            } else if (ratio) {
                std::snprintf(text, sizeof(text), "%.1f %%", value * 100.0);
            } else {
                std::snprintf(text, sizeof(text), "%.4g %s", value, signal.unit.c_str());
            }
            const float width = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, text).x;
            draw->AddText(fonts.mono, mono, ImVec2(max.x - width - px(10.0F), y), u32(c::t1), text);
            if (ratio && !std::isnan(value)) {
                // A ratio (state of charge) also gets a meter between its name and value.
                const float labelWidth =
                    fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, signal.label.c_str()).x;
                const ImVec2 a(min.x + px(18.0F) + labelWidth, y + px(6.0F));
                const ImVec2 b(max.x - width - px(18.0F), y + px(12.0F));
                if (b.x > a.x + px(12.0F)) {
                    draw->AddRectFilled(a, b, u32(c::bg4), px(3.0F));
                    const float fill = static_cast<float>(std::clamp(value, 0.0, 1.0));
                    draw->AddRectFilled(a, ImVec2(a.x + (b.x - a.x) * fill, b.y), u32(c::ok),
                                        px(3.0F));
                }
            }
        }
        draw->PopClipRect();
    }

    static void legend(ImDrawList* draw, const ImVec2& min) {
        const ImVec2 max(min.x + px(138.0F), min.y + px(50.0F));
        draw->AddRectFilled(min, max, u32(c::bg1), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        ImFont* font = theme::fonts().sans;
        const float size = px(theme::kLabelSize);
        const auto entry = [&](float y, const ImVec4& color, const char* text) {
            draw->AddLine(ImVec2(min.x + px(10.0F), y + size * 0.55F),
                          ImVec2(min.x + px(30.0F), y + size * 0.55F), u32(color), px(2.0F));
            draw->AddText(font, size, ImVec2(min.x + px(38.0F), y), u32(c::t2), text);
        };
        entry(min.y + px(8.0F), kDischarge, "discharge");
        entry(min.y + px(28.0F), kRegen, "regen (reversed)");
    }

    std::shared_ptr<ElectricalState> state_;
};

// --- Scopes -------------------------------------------------------------------------------

class ElectricalScopesPanel final : public Panel {
public:
    explicit ElectricalScopesPanel(std::shared_ptr<ElectricalState> state)
        : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Scopes"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        ScopeSession& scopes = state_->scopes;
        if (scopes.traces().empty()) {
            if (!emptyState(context, *state_)) {
                widgets::Caption("The nets of this stage name no signals to plot.");
            }
            ImGui::End();
            return;
        }
        static constexpr std::array<const char*, 3> kWindows{"1 s", "10 s", "60 s"};
        widgets::Pill(scopes.paused() ? "FROZEN" : "LIVE",
                      scopes.paused() ? widgets::PillKind::Warn : widgets::PillKind::Ok);
        ImGui::SameLine(0.0F, px(10.0F));
        scopes.window = std::clamp(scopes.window, 0, 2);
        widgets::Segmented("##window", kWindows.data(), 3, &scopes.window);
        ImGui::SameLine(0.0F, px(12.0F));
        ImGui::AlignTextToFramePadding();
        widgets::Caption("min/max decimation · %zu points drawn", scopes.drawnPoints);
        ImGui::SameLine();
        widgets::AlignRight(widgets::ButtonWidth("Freeze"));
        if (widgets::Button(
                "Freeze", scopes.paused() ? widgets::ButtonKind::On : widgets::ButtonKind::Ghost)) {
            scopes.setPaused(!scopes.paused());
        }
        state_->plotter.draw(scopes);
        ImGui::End();
    }

private:
    std::shared_ptr<ElectricalState> state_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createElectricalPanels() {
    auto state = std::make_shared<ElectricalState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<NetworkPanel>(state));
    panels.push_back(std::make_unique<SchematicPanel>(state));
    panels.push_back(std::make_unique<ElectricalScopesPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
