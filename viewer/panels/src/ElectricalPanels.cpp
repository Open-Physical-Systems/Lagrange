// Electrical workspace: the network table, the schematic with live current flow, and the
// scopes of the bus and its branches (docs/design/ui/04-electrical/SPEC.md).
//
// Everything is built from the stage's electrical nets (SceneStatics::nets) and from signals:
// nothing here knows a particular robot.
#include "CircuitDraw.hpp"
#include "ScopePlots.hpp"
#include "lagrange/viewer/Circuit.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/LabelLayout.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Picking.hpp"
#include "lagrange/viewer/RenderOptions.hpp"
#include "lagrange/viewer/SchematicLayout.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
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

/// Whether the prim applies a version of the schema family `family` ("BatteryAPI").
bool hasSchema(const PanelContext& context, std::uint32_t prim, std::string_view family) {
    if (!context.scene || prim >= context.scene->statics->prims.size()) {
        return false;
    }
    for (const std::string& schema : context.scene->statics->prims[prim].apiSchemas) {
        if (schema.starts_with(family) && schema.size() > family.size() &&
            schema[family.size()] == '_') {
            return true;
        }
    }
    return false;
}

/// The joint whose signals belong to this prim, by their names: joint/<name>/...
std::string jointNamed(const PanelContext& context, std::uint32_t prim) {
    if (!context.catalog) {
        return {};
    }
    for (const SignalInfo& info : context.catalog->signals) {
        if (info.prim == prim && info.name.starts_with("joint/")) {
            const std::size_t end = info.name.find('/', 6);
            return info.name.substr(6, end == std::string::npos ? end : end - 6);
        }
    }
    return {};
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
            circuitScene_ = 0;  // the joints' names come with the signal list
            if (context.scene) {
                context.client.requestAttributesOf(circuitPrims(*context.scene->statics));
            }
        }
        // What the nets are made of: their values, as they arrive.
        if (const auto known = context.client.attributesOf();
            context.scene && (known != attributes_ || scene_ != circuitScene_)) {
            attributes_ = known;
            circuitScene_ = scene_;
            circuit = circuitOf(
                *context.scene->statics,
                [&known](std::uint32_t prim) -> const PrimAttributes* {
                    const auto found = known->find(prim);
                    return found != known->end() ? &found->second : nullptr;
                },
                [&context](std::uint32_t prim) { return jointNamed(context, prim); });
        }
        if (const int frame = ImGui::GetFrameCount(); frame != frame_) {
            frame_ = frame;
            scopes.update();
        }
    }

    std::vector<NetLive> nets;  // parallel to SceneStatics::nets
    Circuit circuit;            // buses parallel to SceneStatics::nets, parts to their terminals
    ScopeSession scopes;
    ScopePlotter plotter;

private:
    std::shared_ptr<const std::map<std::uint32_t, PrimAttributes>> attributes_;
    std::uint64_t circuitScene_{0};

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
                // Design (Scopes): the bus voltage, the motors' q-axis currents and the battery's
                // power (its current when the server sends no power); the branches' currents are
                // in the Network table.
                const bool motor = hasSchema(context, terminal.prim, "ElectricalMotorAPI");
                const bool source = hasSchema(context, terminal.prim, "BatteryAPI");
                const SignalInfo* power = nullptr;
                if (source) {
                    for (const SignalInfo& info : catalog.signals) {
                        if (info.prim == terminal.prim && info.name.ends_with("/power")) {
                            power = &info;
                        }
                    }
                }
                if (power != nullptr) {
                    scopes.add(context.client, catalog, power->name);
                }
                if (catalog.find(terminal.currentSignal) != nullptr) {
                    entry.current = context.client.subscribe(terminal.currentSignal);
                    if ((source && power == nullptr) ||
                        (!motor && !source &&
                         !hasSchema(context, terminal.prim, "ElectricalLoadAPI"))) {
                        scopes.add(context.client, catalog, terminal.currentSignal);
                    }
                }
                if (motor) {
                    for (const SignalInfo& info : catalog.signals) {
                        if (info.prim == terminal.prim && info.name.ends_with("/current_q")) {
                            scopes.add(context.client, catalog, info.name);
                        }
                    }
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
                // Design (Interactions): Alt-clicking a value adds it to the scopes.
                for (std::size_t t = 0; t < net.terminals.size(); ++t) {
                    if (net.terminals[t].isSource() &&
                        row(context, statics, net.terminals[t].prim, 0, volts,
                            latest(live.terminals[t].current))) {
                        probe(context, net.terminals[t].currentSignal);
                    }
                }
                if (net.busPrim != kNone &&
                    row(context, statics, net.busPrim, 0, volts, NAN, " bus")) {
                    probe(context, net.voltageSignal);
                }
                for (std::size_t t = 0; t < net.terminals.size(); ++t) {
                    if (!net.terminals[t].isSource() &&
                        row(context, statics, net.terminals[t].prim, 1, volts,
                            latest(live.terminals[t].current))) {
                        probe(context, net.terminals[t].currentSignal);
                    }
                }
            }
            ImGui::EndTable();
        }
        widgets::Caption("  V in volts  ·  I in amps  ·  P in watts  ·  Alt-click: probe");
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

    /// Adds a signal to the Electrical scopes (a probe), when the run has it.
    void probe(PanelContext& context, const std::string& signal) const {
        if (context.catalog && !signal.empty()) {
            (void)state_->scopes.add(context.client, *context.catalog, signal);
        }
    }

    /// One row of the table; true when it was Alt-clicked (a probe asked for).
    static bool row(PanelContext& context, const SceneStatics& statics, std::uint32_t prim,
                    int depth, double volts, double amperes, const char* suffix = "") {
        ImGui::PushID(static_cast<int>(prim) * 2 + (*suffix != '\0' ? 1 : 0));
        ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
        ImGui::TableNextColumn();
        const bool selected = context.selectedPrim == prim;
        bool probed = false;
        if (ImGui::Selectable(
                "##row", selected,
                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
            probed = ImGui::GetIO().KeyAlt;
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
        return probed;
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
        followLayout(context);
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
            const ImVec2 extent = net(context, draw, statics, n, state_->nets[n],
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
    static constexpr float kWire = 120.0F;    // length of a wire between a node and the bus
    static constexpr float kGap = 22.0F;      // between stacked nodes
    static constexpr float kMinWire = 40.0F;  // the shortest a dragged node's wire gets

    /// The offsets of this stage's nodes, as they were left.
    void followLayout(const PanelContext& context) {
        if (layoutPath_.empty() && !context.configDirectory.empty()) {
            layoutPath_ = context.configDirectory + "/schematic_layouts.tsv";
            layouts_ = loadSchematicLayouts(layoutPath_);
        }
        configDirectory_ = context.configDirectory;
        if (context.scene) {
            stage_ = context.scene->statics->stagePath;
        }
    }

    void headerTools() {
        // The design writes the "proportional to" sign; IBM Plex has no such glyph.
        const char* hint = "wire width follows |I| · dashes move with current";
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        const float hintWidth = ImGui::CalcTextSize(hint).x;
        ImGui::PopFont();
        const float gap = px(6.0F);
        const float width = hintWidth + widgets::ButtonWidth("Auto-layout") +
                            widgets::ButtonWidth("Export SPICE") +
                            widgets::ButtonWidth("Export Modelica") + gap * 3.0F + px(8.0F);
        if (!widgets::BeginPanelTools("##schematic-tools", width)) {
            return;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("%s", hint);
        ImGui::SameLine(0.0F, px(8.0F));
        // The design's automatic layout, undoing what was dragged. Nothing to undo: disabled.
        const auto moved = layouts_.find(stage_);
        ImGui::BeginDisabled(moved == layouts_.end() || moved->second.empty());
        if (widgets::Button("Auto-layout", widgets::ButtonKind::Ghost)) {
            layouts_.erase(stage_);
            saveLayouts();
        }
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, gap);
        // Beside the viewer's other files, named after the stage.
        const std::vector<std::string> problems = circuitProblems(state_->circuit);
        ImGui::BeginDisabled(!problems.empty());
        if (widgets::Button("Export SPICE")) {
            exportSpice();
        }
        ImGui::EndDisabled();
        if (!problems.empty()) {
            ImGui::SetItemTooltip("%s", problems.front().c_str());
        } else if (!exported_.empty()) {
            ImGui::SetItemTooltip("%s", exported_.c_str());
        }
        ImGui::SameLine(0.0F, gap);
        // The model needs what the server has, over a command the protocol does not have yet.
        ImGui::BeginDisabled(true);
        widgets::Button("Export Modelica");
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("lagrange export modelica --robot FILE");
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

    static float nodeHeight(const TerminalLive& live, const std::string& values) {
        return px(kHeader) + px(8.0F) +
               px(kLine) * static_cast<float>(1 + live.signals.size() + (values.empty() ? 0 : 1)) +
               px(6.0F);
    }

    /// The values of a net's terminal (its schematic node's last row), when they have arrived.
    [[nodiscard]] std::string valuesOf(std::size_t net, std::size_t terminal) const {
        const std::vector<CircuitBus>& buses = state_->circuit.buses;
        return net < buses.size() && terminal < buses[net].parts.size()
                   ? partValues(buses[net].parts[terminal])
                   : std::string{};
    }

    /// Draws one net at `at`. Returns the size it took.
    ImVec2 net(PanelContext& context, ImDrawList* draw, const SceneStatics& statics,
               std::size_t netIndex, const NetLive& live, const ImVec2& at, bool off) {
        const ElectricalNet& net = statics.nets[netIndex];
        // Heights of the two columns.
        float sources = 0.0F;
        float loads = 0.0F;
        for (std::size_t t = 0; t < net.terminals.size(); ++t) {
            float& column = net.terminals[t].isSource() ? sources : loads;
            column += nodeHeight(live.terminals[t], valuesOf(netIndex, t)) + px(kGap);
        }
        sources = std::max(sources - px(kGap), 0.0F);
        loads = std::max(loads - px(kGap), 0.0F);
        float height = std::max({sources, loads, px(80.0F)});
        const float busX = at.x + px(kNodeWidth) + px(kWire);
        const float loadX = busX + px(kWire);
        const double volts = latest(live.voltage);

        // Where each node goes: its place in the automatic layout, then where it was dragged.
        struct Placed {
            ImVec2 min;
            float height{0.0F};
        };
        std::vector<Placed> placed(net.terminals.size());
        const float baseHeight = height;
        float sourceY = at.y + (baseHeight - sources) * 0.5F;
        float loadY = at.y + (baseHeight - loads) * 0.5F;
        float barTop = at.y;
        float barBottom = at.y + baseHeight;
        float right = loadX + px(kNodeWidth);
        for (std::size_t t = 0; t < net.terminals.size(); ++t) {
            const bool source = net.terminals[t].isSource();
            const float nodeH = nodeHeight(live.terminals[t], valuesOf(netIndex, t));
            ImVec2 min(source ? at.x : loadX, source ? sourceY : loadY);
            (source ? sourceY : loadY) += nodeH + px(kGap);
            const NodeOffset moved = offsetOf(statics, net.terminals[t].prim);
            min.x += px(moved.x);
            min.y = std::max(min.y + px(moved.y), at.y);  // not above the canvas: it cannot scroll
            // A wire reaches the bus from its own side.
            min.x = source ? std::min(min.x, busX - px(kNodeWidth) - px(kMinWire))
                           : std::max(min.x, busX + px(kMinWire));
            placed[t] = {min, nodeH};
            barTop = std::min(barTop, min.y + nodeH * 0.5F - px(8.0F));
            barBottom = std::max(barBottom, min.y + nodeH * 0.5F + px(8.0F));
            right = std::max(right, min.x + px(kNodeWidth));
        }
        height = barBottom - at.y;

        // The bus: a vertical bar in the electrical colour, with its name and voltage above.
        const float barWidth = px(6.0F);
        draw->AddRectFilled(ImVec2(busX - barWidth * 0.5F, barTop),
                            ImVec2(busX + barWidth * 0.5F, barBottom), u32(kDischarge),
                            barWidth * 0.5F);
        const theme::Fonts& fonts = theme::fonts();
        const float mono = px(theme::kMonoSize);
        char text[64];
        number(volts, "%.1f V", text, sizeof(text));
        const float nameWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, net.name.c_str()).x;
        const float valueWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, text).x;
        const float labelX = busX - (nameWidth + px(8.0F) + valueWidth) * 0.5F;
        draw->AddText(fonts.mono, mono, ImVec2(labelX, barTop - px(24.0F)), u32(kDischarge),
                      net.name.c_str());
        draw->AddText(fonts.mono, mono, ImVec2(labelX + nameWidth + px(8.0F), barTop - px(24.0F)),
                      u32(c::t1), text);

        for (std::size_t t = 0; t < net.terminals.size(); ++t) {
            const NetTerminal& terminal = net.terminals[t];
            const TerminalLive& entry = live.terminals[t];
            const bool source = terminal.isSource();
            const std::string values = valuesOf(netIndex, t);
            const float nodeH = placed[t].height;
            const ImVec2 min = placed[t].min;
            const double amperes = latest(entry.current);
            const float midY = min.y + nodeH * 0.5F;
            // The wire runs in the direction the current normally flows: source to bus, bus
            // to load. A negative current is regeneration and flows the other way.
            if (source) {
                wire(draw, ImVec2(min.x + px(kNodeWidth), midY), ImVec2(busX, midY), amperes, off);
            } else {
                wire(draw, ImVec2(busX, midY), ImVec2(min.x, midY), amperes, off);
            }
            const std::vector<CircuitBus>& buses = state_->circuit.buses;
            const std::string detail = netIndex < buses.size() && t < buses[netIndex].parts.size()
                                           ? buses[netIndex].parts[t].detail
                                           : std::string{};
            node(context, draw, statics, terminal, entry, values,
                 detail.empty() ? terminal.port : detail, min, nodeH, volts, amperes);
            dragNode(statics, terminal.prim);
        }
        // Design: the bus's limits under its bar, with its capacitance after them.
        float below = 0.0F;
        if (netIndex < state_->circuit.buses.size()) {
            const std::string limits = busValues(state_->circuit.buses[netIndex]);
            if (!limits.empty()) {
                const float small = px(theme::kCaptionSize);
                const float limitsWidth =
                    fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, limits.c_str()).x;
                below = px(24.0F);
                draw->AddText(fonts.mono, small,
                              ImVec2(busX - limitsWidth * 0.5F, barBottom + px(8.0F)), u32(c::t3),
                              limits.c_str());
            }
        }
        return {right - at.x, height + below};
    }

    [[nodiscard]] NodeOffset offsetOf(const SceneStatics& statics, std::uint32_t prim) const {
        const auto layout = layouts_.find(stage_);
        if (layout != layouts_.end()) {
            if (const auto found = layout->second.find(statics.prims[prim].name);
                found != layout->second.end()) {
                return found->second;
            }
        }
        return {};
    }

    /// The last item is a node's button: dragging it moves the node, in logical pixels.
    void dragNode(const SceneStatics& statics, std::uint32_t prim) {
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            NodeOffset& offset = layouts_[stage_][statics.prims[prim].name];
            offset.x += ImGui::GetIO().MouseDelta.x / px(1.0F);
            offset.y += ImGui::GetIO().MouseDelta.y / px(1.0F);
        }
        if (ImGui::IsItemDeactivated()) {
            saveLayouts();
        }
    }

    void exportSpice() {
        const std::filesystem::path stage(stage_);
        const std::filesystem::path file =
            std::filesystem::path(configDirectory_) / (stage.stem().string() + ".cir");
        std::ofstream out(file);
        out << netlistOf(state_->circuit, stage_);
        out.close();
        exported_ = out ? "Wrote " + file.string() : "Could not write " + file.string();
    }

    void saveLayouts() const {
        if (!layoutPath_.empty()) {
            saveSchematicLayouts(layoutPath_, layouts_);
        }
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
                     const NetTerminal& terminal, const TerminalLive& live,
                     const std::string& values, const std::string& detail, const ImVec2& min,
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
        const float portWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, detail.c_str()).x;
        draw->AddText(fonts.mono, small,
                      ImVec2(max.x - portWidth - px(8.0F), min.y + (px(kHeader) - small) * 0.5F),
                      u32(c::t3), detail.c_str());

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
        // Last, what the component is: its values from the stage, which do not move.
        if (!values.empty()) {
            y += px(kLine);
            draw->AddText(fonts.mono, small, ImVec2(min.x + px(10.0F), y + px(1.0F)), u32(c::t2),
                          values.c_str());
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
    SchematicLayouts layouts_;
    std::string layoutPath_;
    std::string stage_;
    std::string configDirectory_;
    std::string exported_;
};

// --- Circuit --------------------------------------------------------------------------------

/// The same power system as the Schematic, with its symbols and values (SPEC, Proposed).
class CircuitPanel final : public Panel {
public:
    explicit CircuitPanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Circuit"; }
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
        ImGui::BeginChild("##circuit", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        const std::vector<NetLive>& nets = state_->nets;
        const Circuit& circuit = state_->circuit;
        // The inspector's selection is the circuit's, by prim.
        std::optional<CircuitPick> selected;
        for (std::size_t b = 0; b < circuit.buses.size(); ++b) {
            if (circuit.buses[b].prim == context.selectedPrim && context.selectedPrim != kNone) {
                selected = CircuitPick{b, std::nullopt};
            }
            for (std::size_t p = 0; p < circuit.buses[b].parts.size(); ++p) {
                if (circuit.buses[b].parts[p].prim == context.selectedPrim &&
                    context.selectedPrim != kNone) {
                    selected = CircuitPick{b, p};
                }
            }
        }
        const auto picked = drawCircuit(
            circuit,
            [&nets](std::size_t b, std::size_t p) {
                return b < nets.size() && p < nets[b].terminals.size()
                           ? latest(nets[b].terminals[p].current)
                           : NAN;
            },
            [&nets](std::size_t b) { return b < nets.size() ? latest(nets[b].voltage) : NAN; },
            selected);
        if (picked) {
            const CircuitBus& bus = circuit.buses[picked->bus];
            context.selectedPrim = picked->part ? bus.parts[*picked->part].prim : bus.prim;
        }
        const std::string notes = slopedNote(circuit);
        if (!notes.empty()) {
            widgets::Caption("%s", notes.c_str());
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    static std::string slopedNote(const Circuit& circuit) {
        for (const CircuitBus& bus : circuit.buses) {
            for (const CircuitPart& part : bus.parts) {
                if (!part.ocvSoc.empty()) {
                    return "A battery's voltage is the one it starts at; it follows its charge in "
                           "the run.";
                }
            }
        }
        return {};
    }

    std::shared_ptr<ElectricalState> state_;
};

// --- 3D overlay -----------------------------------------------------------------------------

/// The design's third tab: the robot with each motor's link tinted by its current, and a wire from
/// each part to its bus whose width follows the current. Takes the shared view's shading while it
/// is in front; the Viewport takes it back.
class OverlayPanel final : public Panel {
public:
    explicit OverlayPanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "3D overlay"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(
            title(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();
        RenderOptions& options = *context.renderOptions;
        if (!open) {
            if (options.shading == Shading::Current) {
                options.shading = options.beforeCurrent;
            }
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (emptyState(context, *state_)) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            ImGui::End();
            return;
        }
        if (options.shading != Shading::Current) {
            options.beforeCurrent = options.shading;
            options.shading = Shading::Current;
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        context.requestedViewportWidth =
            static_cast<std::uint32_t>(std::max(available.x * scale.x, 1.0F));
        context.requestedViewportHeight =
            static_cast<std::uint32_t>(std::max(available.y * scale.y, 1.0F));
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        if (context.viewportTexture != 0) {
            ImGui::Image(static_cast<ImTextureID>(context.viewportTexture), available);
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::InvisibleButton(
            "##overlay", available,
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        camera(context, available);
        // A click that was not a drag selects what is under it, as in the Viewport.
        if (ImGui::IsItemHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x == 0.0F &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).y == 0.0F) {
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const double aspect = static_cast<double>(available.x) / std::max(available.y, 1.0F);
            context.selectedPrim = pickPrim(
                *context.scene,
                context.camera.ray((mouse.x - origin.x) / available.x * 2.0 - 1.0,
                                   (mouse.y - origin.y) / available.y * 2.0 - 1.0, aspect));
        }
        tint(context);
        wires(context, origin, available);
        if (electricalOff(context)) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + px(12.0F), origin.y + px(10.0F)));
            widgets::Pill("Electrical is off → ideal torque source. Currents are not simulated.",
                          widgets::PillKind::Warn);
        }
        ImGui::End();
    }

private:
    static constexpr double kOrbitRadiansPerPixel = 0.005;
    static constexpr double kZoomPerWheelStep = 0.9;

    static void camera(PanelContext& context, const ImVec2& size) {
        const ImGuiIO& io = ImGui::GetIO();
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            context.camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                                 io.MouseDelta.y * kOrbitRadiansPerPixel);
        }
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            context.camera.pan(-io.MouseDelta.x / std::max(size.y, 1.0F),
                               io.MouseDelta.y / std::max(size.y, 1.0F));
        }
        if (ImGui::IsItemHovered() && io.MouseWheel != 0.0F) {
            context.camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
        }
    }

    /// Each motor's share of the largest current on the link it turns, and on what that link
    /// carries until another motor's link.
    void tint(PanelContext& context) {
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        const auto known = context.client.attributesOf();
        own_.assign(prims.size(), NAN);
        double largest = kCurrentFloorAmperes;
        const auto each = [this](const auto& visit) {
            const std::vector<CircuitBus>& buses = state_->circuit.buses;
            for (std::size_t b = 0; b < buses.size() && b < state_->nets.size(); ++b) {
                for (std::size_t p = 0;
                     p < buses[b].parts.size() && p < state_->nets[b].terminals.size(); ++p) {
                    visit(buses[b].parts[p], latest(state_->nets[b].terminals[p].current));
                }
            }
        };
        each([&largest](const CircuitPart&, double amperes) {
            if (!std::isnan(amperes)) {
                largest = std::max(largest, std::abs(amperes));
            }
        });
        each([&](const CircuitPart& part, double amperes) {
            if (part.kind != CircuitPart::Kind::Motor) {
                return;
            }
            const auto found = known->find(part.prim);
            if (found == known->end()) {
                return;
            }
            for (const AttributeInfo& a : found->second.attributes) {
                if (a.name == "elec:joint" && !a.targets.empty() && a.targets[0] < prims.size()) {
                    own_[a.targets[0]] =
                        std::isnan(amperes) ? 0.0F : static_cast<float>(amperes / largest);
                }
            }
        });
        std::vector<float>& shares = context.renderOptions->current;
        shares.assign(prims.size(), NAN);
        for (std::uint32_t i = 0; i < prims.size(); ++i) {
            for (std::uint32_t p = i; p != kNone && p < prims.size(); p = prims[p].parent) {
                if (!std::isnan(own_[p])) {
                    shares[i] = own_[p];
                    break;
                }
            }
        }
    }

    /// From each part to its bus, over the image: the same wire as the schematic's, in 3D.
    void wires(PanelContext& context, const ImVec2& origin, const ImVec2& size) const {
        const double aspect = static_cast<double>(size.x) / std::max(size.y, 1.0F);
        const auto matrix = context.camera.viewProjection(aspect);
        const std::array<double, 3> eye = context.camera.eye();
        const auto& transforms = context.scene->transforms;
        const auto project = [&](std::uint32_t prim) -> std::optional<ImVec2> {
            if (prim >= transforms.size()) {
                return std::nullopt;
            }
            const auto& p = transforms[prim].position;
            const auto x = static_cast<float>(p[0] - eye[0]);
            const auto y = static_cast<float>(p[1] - eye[1]);
            const auto z = static_cast<float>(p[2] - eye[2]);
            const float w = matrix[12] * x + matrix[13] * y + matrix[14] * z;
            if (w <= 0.01F) {
                return std::nullopt;  // behind the camera
            }
            const float nx = (matrix[0] * x + matrix[1] * y + matrix[2] * z) / w;
            const float ny = (matrix[4] * x + matrix[5] * y + matrix[6] * z) / w;
            return ImVec2(origin.x + (nx + 1.0F) * 0.5F * size.x,
                          origin.y + (ny + 1.0F) * 0.5F * size.y);
        };
        const auto apart = [&transforms](std::uint32_t a, std::uint32_t b) {
            if (a >= transforms.size() || b >= transforms.size()) {
                return false;
            }
            double sum = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                const double d = transforms[a].position.at(k) - transforms[b].position.at(k);
                sum += d * d;
            }
            return sum > kSamePlaceMetres * kSamePlaceMetres;
        };
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        const auto known = context.client.attributesOf();
        const std::vector<CircuitBus>& buses = state_->circuit.buses;

        // Labels in the image: one per placed part and bus. Parts of a robot sit within
        // centimetres of each other, so their labels are stacked in a column beside the points
        // with a leader line to each, never on top of each other (M10).
        struct Label {
            ImVec2 anchor;
            std::string text;
            ImU32 color;
        };
        std::vector<Label> labels;
        // The legend in the corner: every part with its current, and each bus with its voltage.
        struct Entry {
            std::string name;
            std::string value;
            ImU32 color;
            bool bus;
        };
        std::vector<Entry> legend;
        for (std::size_t b = 0; b < buses.size() && b < state_->nets.size(); ++b) {
            const auto bus = project(buses[b].prim);
            char volts[48];
            number(latest(state_->nets[b].voltage), "%.1f V", volts, sizeof(volts));
            legend.push_back({buses[b].name, volts, u32(kDischarge), true});
            for (std::size_t p = 0;
                 p < buses[b].parts.size() && p < state_->nets[b].terminals.size(); ++p) {
                const CircuitPart& part = buses[b].parts[p];
                // A motor is where the link it turns is; its own prim is on the parent link.
                std::uint32_t where = part.prim;
                if (const auto found = known->find(part.prim); found != known->end()) {
                    for (const AttributeInfo& a : found->second.attributes) {
                        if (a.name == "elec:joint" && !a.targets.empty()) {
                            where = a.targets[0];
                        }
                    }
                }
                const double amperes = latest(state_->nets[b].terminals[p].current);
                const double magnitude = std::isnan(amperes) ? 0.0 : std::abs(amperes);
                const ImU32 color = u32(amperes < 0.0 ? kRegen : kDischarge);
                char text[48];
                std::snprintf(text, sizeof(text), "%.2f A", magnitude);
                legend.push_back({part.name, text, color, false});
                const auto at = project(where);
                if (bus && at && apart(where, buses[b].prim)) {
                    const float width = std::clamp(
                        px(1.5F) + static_cast<float>(magnitude) * px(0.35F), px(1.5F), px(6.0F));
                    draw->AddLine(*bus, *at, color, width);
                    draw->AddCircleFilled(*at, px(3.5F), color);
                    labels.push_back({*at, part.name + "  " + text, color});
                }
            }
            if (bus) {
                draw->AddCircleFilled(*bus, px(5.0F), u32(kDischarge));
                labels.push_back({*bus, buses[b].name + "  " + volts, u32(kDischarge)});
            }
        }

        // The column: right of the rightmost point, or left of the leftmost when there is no
        // room on the right.
        if (!labels.empty()) {
            float widest = 0.0F;
            float left = FLT_MAX;
            float right = -FLT_MAX;
            std::vector<float> anchors;
            for (const Label& label : labels) {
                widest = std::max(
                    widest, fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, label.text.c_str()).x);
                left = std::min(left, label.anchor.x);
                right = std::max(right, label.anchor.x);
                anchors.push_back(label.anchor.y);
            }
            const float pad = px(4.0F);
            const float boxWidth = widest + pad * 2.0F;
            const float reach = px(36.0F);
            float x = right + reach;
            const bool onRight = x + boxWidth <= origin.x + size.x - px(8.0F);
            if (!onRight) {
                x = std::max(left - reach - boxWidth, origin.x + px(8.0F));
            }
            const float height = small + pad;
            const std::vector<float> tops = stackLabels(
                anchors, height, px(3.0F), origin.y + px(8.0F), origin.y + size.y - px(8.0F));
            for (std::size_t i = 0; i < labels.size(); ++i) {
                const Label& label = labels[i];
                const ImVec2 boxMin(x, tops[i]);
                const ImVec2 boxMax(x + boxWidth, tops[i] + height);
                const ImVec2 end(onRight ? boxMin.x : boxMax.x, (boxMin.y + boxMax.y) * 0.5F);
                draw->AddLine(label.anchor, end, u32(c::t3), px(1.0F));
                draw->AddRectFilled(boxMin, boxMax,
                                    u32(ImVec4(c::bg0.x, c::bg0.y, c::bg0.z, 0.85F)), px(3.0F));
                draw->AddRect(boxMin, boxMax, label.color, px(3.0F));
                draw->AddText(fonts.mono, small, ImVec2(boxMin.x + pad, boxMin.y + pad * 0.5F),
                              u32(c::t1), label.text.c_str());
            }
        }

        // The legend, top left: what flows where, every part included (under the pill that says
        // the domain is off, when it is).
        float y = origin.y + (electricalOff(context) ? px(42.0F) : px(10.0F));
        const float x = origin.x + px(12.0F);
        float nameWidth = 0.0F;
        for (const Entry& entry : legend) {
            nameWidth = std::max(
                nameWidth, fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, entry.name.c_str()).x);
        }
        if (!legend.empty()) {
            const float row = px(16.0F);
            draw->AddRectFilled(ImVec2(x - px(6.0F), y - px(4.0F)),
                                ImVec2(x + px(32.0F) + nameWidth + px(64.0F),
                                       y + row * static_cast<float>(legend.size()) + px(2.0F)),
                                u32(ImVec4(c::bg0.x, c::bg0.y, c::bg0.z, 0.75F)), px(3.0F));
        }
        for (const Entry& entry : legend) {
            const float indent = entry.bus ? 0.0F : px(10.0F);
            if (entry.bus) {
                draw->AddRectFilled(ImVec2(x, y + small * 0.5F - px(3.0F)),
                                    ImVec2(x + px(6.0F), y + small * 0.5F + px(3.0F)), entry.color);
            } else {
                draw->AddCircleFilled(ImVec2(x + indent + px(3.0F), y + small * 0.5F), px(3.5F),
                                      entry.color);
            }
            draw->AddText(fonts.mono, small, ImVec2(x + indent + px(12.0F), y),
                          u32(entry.bus ? c::t1 : c::t2), entry.name.c_str());
            draw->AddText(fonts.mono, small, ImVec2(x + px(32.0F) + nameWidth, y), u32(c::t1),
                          entry.value.c_str());
            y += px(16.0F);
        }
        draw->PopClipRect();
    }

    static constexpr double kSamePlaceMetres = 0.01;
    // Below this the tint is not worth seeing: a stopped motor stays grey, not full colour.
    static constexpr double kCurrentFloorAmperes = 0.5;
    std::shared_ptr<ElectricalState> state_;
    std::vector<float> own_;
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

// --- Probes, + scope, History (design: the tabs beside Network, Scopes and Inspector) ------

/// Every scalar signal of the stage's power system, in the nets' order: each net's voltage,
/// then the signals of the prims on it (sources first). What a probe can be put on.
std::vector<const SignalInfo*> electricalSignals(const PanelContext& context) {
    std::vector<const SignalInfo*> out;
    if (!context.scene || !context.catalog) {
        return out;
    }
    const SignalCatalog& catalog = *context.catalog;
    const auto take = [&out](const SignalInfo* info) {
        if (info != nullptr && info->components == 1 && !info->image &&
            std::ranges::find(out, info) == out.end()) {
            out.push_back(info);
        }
    };
    for (const ElectricalNet& net : context.scene->statics->nets) {
        take(catalog.find(net.voltageSignal));
        for (const bool sources : {true, false}) {
            for (const NetTerminal& terminal : net.terminals) {
                if (terminal.isSource() != sources) {
                    continue;
                }
                take(catalog.find(terminal.currentSignal));
                for (const SignalInfo& info : catalog.signals) {
                    if (info.prim == terminal.prim) {
                        take(&info);
                    }
                }
            }
        }
    }
    // Signals of the electrical domain on no prim of a net (fuses, converters' own outputs).
    for (const SignalInfo& info : catalog.signals) {
        if (info.writer.find("electrical") != std::string::npos) {
            take(&info);
        }
    }
    return out;
}

/// The probes: what the Electrical scopes plot, each with its value now and its range over the
/// scopes' window; Alt-click in Network or "+ scope" adds one, x removes it.
class ProbesPanel final : public Panel {
public:
    explicit ProbesPanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return title_.c_str(); }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        state_->follow(context);
        ScopeSession& scopes = state_->scopes;
        title_ = "Probes · " + std::to_string(scopes.traces().size()) + "###ElectricalProbes";
        if (!ImGui::Begin(title_.c_str())) {
            ImGui::End();
            return;
        }
        if (emptyState(context, *state_)) {
            ImGui::End();
            return;
        }
        const ScopeSession::View view = scopes.view();
        std::optional<std::size_t> remove;
        // One probe per row: its name over its range in the scopes' window, its value now on
        // the right (the names are too long for a range column in 300 px).
        if (ImGui::BeginTable("##probes", 3,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_NoPadOuterX)) {
            ImGui::TableSetupColumn("probe", ImGuiTableColumnFlags_None, 3.0F);
            ImGui::TableSetupColumn("now", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, px(18.0F));
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            ImGui::TableNextColumn();
            widgets::Caption("  probe  ·  min … max");
            ImGui::TableNextColumn();
            rightCaption("now");
            ImGui::TableNextColumn();
            const std::vector<ScopeTrace>& traces = scopes.traces();
            for (std::size_t i = 0; i < traces.size(); ++i) {
                const ScopeTrace& trace = traces[i];
                ImGui::PushID(static_cast<int>(i));
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(38.0F));
                ImGui::TableNextColumn();
                const SignalInfo* info =
                    context.catalog ? context.catalog->find(trace.name) : nullptr;
                const bool selected =
                    info != nullptr && context.selectedPrim == info->prim && info->prim != kNone;
                if (ImGui::Selectable(
                        "##row", selected,
                        ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap,
                        ImVec2(0.0F, px(34.0F))) &&
                    info != nullptr && info->prim != kNone) {
                    context.selectedPrim = info->prim;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", trace.label.c_str());
                }
                const ImVec2 top = ImGui::GetItemRectMin();
                ImGui::SetCursorScreenPos(ImVec2(top.x + px(8.0F), top.y + px(2.0F)));
                widgets::Swatch(theme::kSeries[static_cast<std::size_t>(trace.color) % 8]);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "%s", shortSignalName(trace.label));
                ScopeBuffer::Statistics range;
                if (trace.buffer) {
                    range = trace.buffer->statistics(view.beginNs, view.endNs + 1);
                }
                char text[64];
                ImGui::SetCursorScreenPos(ImVec2(top.x + px(26.0F), ImGui::GetCursorScreenPos().y));
                if (range.count == 0) {
                    widgets::Mono(c::t3, "–");
                } else {
                    widgets::Mono(c::t3, "%.4g … %.4g", range.min, range.max);
                }
                const double now = trace.buffer ? trace.buffer->latest() : NAN;
                ImGui::TableNextColumn();
                ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, top.y + px(2.0F)));
                std::snprintf(text, sizeof(text), "%.4g %s", now, unitText(trace.unit));
                rightMono(std::isnan(now) ? c::t3 : c::t1, std::isnan(now) ? "–" : text);
                ImGui::TableNextColumn();
                ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, top.y + px(2.0F)));
                if (ImGui::SmallButton("×")) {
                    remove = i;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Remove this probe from the scopes");
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (remove) {
            scopes.remove(context.client, *remove);
        }
        char window[48];
        std::snprintf(window, sizeof(window), "%.0f s", view.seconds());
        widgets::Caption("  range over the scopes' window (%s)", window);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(
            "  Alt-click a row of Network, or pick a signal in + scope, to add a probe.");
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    static const char* unitText(const std::string& unit) { return unit == "1" ? "" : unit.c_str(); }
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

    std::shared_ptr<ElectricalState> state_;
    std::string title_ = "Probes###ElectricalProbes";
};

/// "+ scope": every signal of the power system not plotted yet, with a filter; a click adds it
/// to the scopes and brings them in front.
class AddScopePanel final : public Panel {
public:
    explicit AddScopePanel(std::shared_ptr<ElectricalState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "+ scope###ElectricalAddScope"; }
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
        ScopeSession& scopes = state_->scopes;
        (void)widgets::SearchField("##filter", "filter signals", filter_.data(), filter_.size(),
                                   px(260.0F));
        ImGui::SameLine(0.0F, px(10.0F));
        ImGui::AlignTextToFramePadding();
        const std::vector<const SignalInfo*> signals = electricalSignals(context);
        std::size_t plotted = 0;
        for (const SignalInfo* info : signals) {
            plotted += scopes.find(info->name) >= 0 ? 1 : 0;
        }
        widgets::Caption("%zu signals of the power system · %zu on the scopes", signals.size(),
                         plotted);
        const std::string_view filter(filter_.data());
        ImGui::BeginChild("##signals");
        std::string added;
        for (const SignalInfo* info : signals) {
            if (!filter.empty() && info->name.find(filter) == std::string::npos) {
                continue;
            }
            const bool on = scopes.find(info->name) >= 0;
            ImGui::PushID(info->name.c_str());
            ImGui::BeginDisabled(on);
            if (ImGui::Selectable("##add", false, ImGuiSelectableFlags_AllowOverlap,
                                  ImVec2(0.0F, px(22.0F))) &&
                context.catalog) {
                if (scopes.add(context.client, *context.catalog, info->name)) {
                    added = info->name;
                }
            }
            ImGui::SameLine(px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Mono(on ? c::t3 : c::t1, "%s%s", on ? "" : "+ ", info->name.c_str());
            ImGui::SameLine();
            const char* unit = info->unit == "1" ? "" : info->unit.c_str();
            char right[64];
            std::snprintf(right, sizeof(right), "%s%s%s", unit, *unit != '\0' ? "  ·  " : "",
                          on ? "on a scope" : rate(info->dtNs).c_str());
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            widgets::AlignRight(ImGui::CalcTextSize(right).x + px(8.0F));
            ImGui::TextColored(c::t3, "%s", right);
            ImGui::PopFont();
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndChild();
        if (!added.empty()) {
            ImGui::SetWindowFocus("Scopes");
        }
        ImGui::End();
    }

private:
    static std::string rate(std::int64_t dtNs) {
        if (dtNs <= 0) {
            return "on change";
        }
        const double hz = 1e9 / static_cast<double>(dtNs);
        char text[32];
        if (hz >= 1000.0) {
            std::snprintf(text, sizeof(text), "%.0f kHz", hz / 1000.0);
        } else {
            std::snprintf(text, sizeof(text), "%.0f Hz", hz);
        }
        return text;
    }

    std::shared_ptr<ElectricalState> state_;
    std::array<char, 64> filter_{};
};

/// History (beside the Inspector): the electrical events of the run so far, newest first, and
/// what each bus did while this viewer watched (its lowest and highest voltage, the largest
/// current of each branch) — the scopes' buffers, not a new request.
class ElectricalHistoryPanel final : public Panel {
public:
    explicit ElectricalHistoryPanel(std::shared_ptr<ElectricalState> state)
        : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "History###ElectricalHistory"; }
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
        const std::shared_ptr<const EventLog> log = context.client.events();
        std::size_t count = 0;
        for (const EventInfo& event : log->events) {
            count += event.domain == "electrical" ? 1 : 0;
        }
        char meta[32];
        std::snprintf(meta, sizeof(meta), "%zu", count);
        if (widgets::SectionHeader("Events", meta)) {
            ImGui::Indent(px(8.0F));
            int shown = 0;
            for (std::size_t i = log->events.size(); i-- > 0 && shown < kEventsShown;) {
                const EventInfo& event = log->events[i];
                if (event.domain != "electrical") {
                    continue;
                }
                ++shown;
                ImGui::PushID(static_cast<int>(i));
                widgets::Pill(event.kind.c_str(), event.severity == EventInfo::Severity::Error
                                                      ? widgets::PillKind::Error
                                                  : event.severity == EventInfo::Severity::Warning
                                                      ? widgets::PillKind::Warn
                                                      : widgets::PillKind::Info);
                ImGui::SameLine(0.0F, px(8.0F));
                ImGui::PushTextWrapPos(ImGui::GetContentRegionMax().x - px(70.0F));
                widgets::Paragraph(event.text);
                ImGui::PopTextWrapPos();
                if (ImGui::IsItemClicked() && event.prim != kNone) {
                    context.selectedPrim = event.prim;
                }
                char at[32];
                std::snprintf(at, sizeof(at), "%.2f s", static_cast<double>(event.beginNs) * 1e-9);
                ImGui::SameLine();
                ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
                widgets::AlignRight(ImGui::CalcTextSize(at).x);
                ImGui::PopFont();
                widgets::Mono(c::t3, "%s", at);
                ImGui::PopID();
            }
            if (shown == 0) {
                widgets::Caption("No electrical event in this run yet.");
            }
            ImGui::Unindent(px(8.0F));
        }
        if (widgets::SectionHeader("Since this viewer connected", nullptr)) {
            ImGui::Indent(px(8.0F));
            for (std::size_t n = 0; n < statics.nets.size() && n < state_->nets.size(); ++n) {
                const ElectricalNet& net = statics.nets[n];
                const NetLive& live = state_->nets[n];
                widgets::Mono(kDischarge, "%s", net.name.c_str());
                char text[64];
                if (const auto range = held(live.voltage); range.count != 0) {
                    std::snprintf(text, sizeof(text), "%.2f … %.2f", range.min, range.max);
                    widgets::PropertyRow("bus voltage", text, "V");
                }
                for (std::size_t t = 0; t < net.terminals.size() && t < live.terminals.size();
                     ++t) {
                    const auto range = held(live.terminals[t].current);
                    if (range.count == 0 || net.terminals[t].prim >= statics.prims.size()) {
                        continue;
                    }
                    const double peak = std::max(std::abs(range.min), std::abs(range.max));
                    std::snprintf(text, sizeof(text), "%.2f", peak);
                    // The largest |I| of each branch: its name, then the value in "A peak".
                    widgets::PropertyRow(statics.prims[net.terminals[t].prim].name.c_str(), text,
                                         "A peak");
                }
                if (live.voltage) {
                    const std::int64_t oldest = live.voltage->oldestTimeNs();
                    const std::int64_t newest = live.voltage->newestTimeNs();
                    if (oldest >= 0 && newest >= oldest) {
                        widgets::Caption("held %.1f s of samples (the scopes' buffers)",
                                         static_cast<double>(newest - oldest) * 1e-9);
                    }
                }
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    static constexpr int kEventsShown = 200;

    static ScopeBuffer::Statistics held(const std::shared_ptr<ScopeBuffer>& buffer) {
        if (!buffer || buffer->newestTimeNs() < 0) {
            return {};
        }
        return buffer->statistics(buffer->oldestTimeNs(), buffer->newestTimeNs() + 1);
    }

    std::shared_ptr<ElectricalState> state_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createElectricalPanels() {
    auto state = std::make_shared<ElectricalState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<NetworkPanel>(state));
    panels.push_back(std::make_unique<ProbesPanel>(state));
    panels.push_back(std::make_unique<SchematicPanel>(state));
    panels.push_back(std::make_unique<CircuitPanel>(state));
    panels.push_back(std::make_unique<OverlayPanel>(state));
    panels.push_back(createNetlistPanel([state](PanelContext& context) -> const Circuit* {
        state->follow(context);
        return state->nets.empty() ? nullptr : &state->circuit;
    }));
    panels.push_back(std::make_unique<ElectricalScopesPanel>(state));
    panels.push_back(std::make_unique<AddScopePanel>(state));
    panels.push_back(std::make_unique<ElectricalHistoryPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
