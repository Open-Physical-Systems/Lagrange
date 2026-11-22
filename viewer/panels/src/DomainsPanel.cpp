// Domains workspace, left panel: one row per domain with its switch, mode, step and cost,
// the substitute picker, and the additional modules (docs/design/ui/03-domains/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/ProfileExtras.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <imgui_internal.h>  // the list's clip rectangle (InnerClipRect)
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr ImGuiTableFlags kTableFlags =
    ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoPadOuterX;
constexpr float kRowHeight = 46.0F;  // two text lines, as in the reference
constexpr float kModuleRowHeight = 32.0F;
// A profile fits real time when one millisecond of simulation costs under one of CPU.
constexpr double kRealTimeBudgetMs = 1.0;
constexpr std::size_t kListedReaders = 6;  // consumers named under the substitute cards

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

void setupColumns() {
    ImGui::TableSetupColumn("##bar", ImGuiTableColumnFlags_WidthFixed, px(4.0F));
    ImGui::TableSetupColumn("domain", ImGuiTableColumnFlags_WidthFixed, px(150.0F));
    ImGui::TableSetupColumn("on", ImGuiTableColumnFlags_WidthFixed, px(34.0F));
    ImGui::TableSetupColumn("mode · substitute when off", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("step", ImGuiTableColumnFlags_WidthFixed, px(84.0F));
    ImGui::TableSetupColumn("cost", ImGuiTableColumnFlags_WidthFixed, px(66.0F));
}

void headerRow() {
    ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
    for (int column = 1; column < ImGui::TableGetColumnCount(); ++column) {
        ImGui::TableSetColumnIndex(column);
        const char* name = ImGui::TableGetColumnName(column);
        if (column == 5) {
            ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
            widgets::AlignRight(ImGui::CalcTextSize(name).x);
            ImGui::PopFont();
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("%s", name);
    }
}

/// Centres the next frame-height item in a row of `rowHeight`.
void centreInRow(float rowTop, float rowHeight) {
    ImGui::SetCursorPosY(rowTop + (rowHeight - ImGui::GetFrameHeight()) * 0.5F);
}

void rightMono(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    widgets::AlignRight(ImGui::CalcTextSize(text).x);
    ImGui::PopFont();
    ImGui::AlignTextToFramePadding();
    widgets::Mono(color, "%s", text);
}

class DomainsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Domains"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        ProfileSession& session = *context.profileSession;
        const std::shared_ptr<const ProfileInfo>& profile = session.profile();
        const bool connected = context.connection.state == ConnectionState::Ready;
        measure(context);
        if (!profile) {
            // Loading / disconnected with nothing to show yet: one line of guidance.
            widgets::Caption("%s", connected ? "Waiting for the server's profile ..."
                                             : "Not connected. Domains appear when a "
                                               "simulation server is reachable.");
            ImGui::End();
            return;
        }
        if (!connected) {
            widgets::Pill("Disconnected - showing the last profile", widgets::PillKind::Warn);
        }

        const float footer = ImGui::GetFrameHeightWithSpacing();
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        ImGui::BeginChild("##body", ImVec2(0.0F, -footer), ImGuiChildFlags_None);
        ImGui::PopStyleVar();
        ImGui::BeginDisabled(!connected);

        if (ImGui::BeginTable("##domains", 6, kTableFlags)) {
            setupColumns();
            headerRow();
            // The physics domains, then the other domains this run has (fire, lights, ...).
            for (const bool physics : {true, false}) {
                for (const DomainInfo& info : profile->domains) {
                    if (physics ? format::isPhysicsDomain(info.name) : inRun(session, info)) {
                        row(session, info, kRowHeight);
                    }
                }
            }
            ImGui::EndTable();
        }
        if (session.selectedDomain.empty()) {
            // Select a row before any click, so the picker and the Schema tab have one.
            session.selectedDomain = pickerDomain(session, *profile);
        }
        substitutePicker(session, *profile);

        if (widgets::SectionHeader("Additional modules", "from robot class")) {
            if (ImGui::BeginTable("##modules", 6, kTableFlags)) {
                setupColumns();
                for (const DomainInfo& info : profile->domains) {
                    if (!format::isPhysicsDomain(info.name) && !inRun(session, info)) {
                        row(session, info, kModuleRowHeight);
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::EndDisabled();
        ImGui::EndChild();

        // Footer: what the applied profile costs.
        double total = 0.0;
        for (const DomainInfo& info : profile->domains) {
            total += info.enabled ? costOf(info) : 0.0;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("est. %.2f ms CPU per 1 ms of sim  ·  %s%s", total,
                         total <= kRealTimeBudgetMs ? "fits real time" : "slower than real time",
                         session.draft().dirty() ? "  ·  for the applied profile" : "");
        ImGui::End();
    }

private:
    static bool inRun(const ProfileSession& session, const DomainInfo& info) {
        const DomainView* view = session.find(info.name);
        return view != nullptr && format::isRunDomain(*view, &info);
    }

    void row(ProfileSession& session, const DomainInfo& info, float logicalHeight) {
        const DomainView* view = session.find(info.name);
        if (view == nullptr) {
            return;
        }
        const float height = px(logicalHeight);
        const bool twoLines = logicalHeight >= kRowHeight;
        ImGui::PushID(info.name.c_str());
        ImGui::TableNextRow(ImGuiTableRowFlags_None, height);

        // Colour bar: the domain's colour, or the accent while the row has unapplied edits.
        ImGui::TableSetColumnIndex(0);
        const ImVec4 bar = view->pending ? c::accent : widgets::domainColor(info.name);
        ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                               u32(info.available ? bar : ImVec4(0, 0, 0, 0)));

        // Name and one line about it. The whole row selects the domain.
        ImGui::TableSetColumnIndex(1);
        const float top = ImGui::GetCursorPosY();
        const bool selected = session.selectedDomain == info.name;
        if (info.available) {
            if (ImGui::Selectable(
                    "##select", selected,
                    ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap,
                    ImVec2(0.0F, height))) {
                session.selectedDomain = info.name;
            }
            ImGui::SetCursorPosY(top);
        }
        const std::string_view label = widgets::domainLabel(info.name);
        const float indent = px(10.0F);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
        if (twoLines) {
            ImGui::SetCursorPosY(top + px(6.0F));
            widgets::Strong("%.*s", static_cast<int>(label.size()), label.data());
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
            ImGui::SetCursorPosY(top + px(25.0F));
            describe(session, info, *view);
        } else {
            centreInRow(top, height);
            ImGui::AlignTextToFramePadding();
            if (info.available) {
                ImGui::TextUnformatted(label.data(), label.data() + label.size());
            } else {
                ImGui::TextDisabled("%.*s", static_cast<int>(label.size()), label.data());
            }
        }

        if (!info.available) {
            ImGui::TableSetColumnIndex(3);
            centreInRow(top, height);
            ImGui::AlignTextToFramePadding();
            // The reason has the step and cost columns too (an unavailable domain has neither),
            // cut with an ellipsis and whole in the tooltip (QA2 F10).
            const float cell = ImGui::GetStyle().CellPadding.x;
            const float span =
                ImGui::GetContentRegionAvail().x + px(84.0F) + px(66.0F) + 4.0F * cell - px(8.0F);
            const ImVec2 at = ImGui::GetCursorScreenPos();
            // Across the columns, but never below or above the scrolled list it is in.
            const ImRect list = ImGui::GetCurrentWindow()->InnerClipRect;
            ImGui::PushClipRect(ImVec2(at.x, std::max(at.y - height, list.Min.y)),
                                ImVec2(at.x + span, std::min(at.y + height, list.Max.y)), false);
            ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
            ImGui::PushStyleColor(ImGuiCol_Text, c::t3);
            fit::text(info.unavailableReason, span);
            ImGui::PopStyleColor();
            ImGui::PopFont();
            ImGui::PopClipRect();
            ImGui::PopID();
            return;
        }

        ImGui::TableSetColumnIndex(2);
        centreInRow(top, height);
        bool enabled = view->enabled;
        if (widgets::Switch("##on", &enabled)) {
            session.setEnabled(info.name, enabled);
            session.selectedDomain = info.name;
            view = session.find(info.name);  // the edit rebuilt the views
        }

        ImGui::TableSetColumnIndex(3);
        centreInRow(top, height);
        modeCombo(session, info, *view);
        view = session.find(info.name);

        ImGui::TableSetColumnIndex(4);
        centreInRow(top, height);
        stepField(session, info, *view);
        view = session.find(info.name);

        ImGui::TableSetColumnIndex(5);
        centreInRow(top, height);
        cost(info, *view, costOf(info));
        ImGui::PopID();
    }

    static void describe(const ProfileSession& session, const DomainInfo& info,
                         const DomainView& view) {
        if (session.domainHasError(info.name)) {
            widgets::Caption("not a valid setting - see Profile");
            return;
        }
        if (!view.enabled) {
            widgets::Caption("%s", view.pending ? "off · pending apply" : "off");
            return;
        }
        if (view.pending) {
            widgets::Caption("changed · pending apply");
            return;
        }
        if (info.dependsOn.empty()) {
            widgets::Caption("reads no other domain");
            return;
        }
        char text[160];
        int used = std::snprintf(text, sizeof(text), "reads ");
        for (std::size_t i = 0; i < info.dependsOn.size() && used < static_cast<int>(sizeof(text));
             ++i) {
            used += std::snprintf(text + used, sizeof(text) - static_cast<std::size_t>(used),
                                  "%s%s", i == 0 ? "" : ", ", info.dependsOn[i].c_str());
        }
        widgets::Caption("%s", text);
    }

    static void modeCombo(ProfileSession& session, const DomainInfo& info, const DomainView& view) {
        const std::vector<std::string>& choices =
            info.backends.empty() ? info.fidelities : info.backends;
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        if (view.enabled && choices.empty()) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("built-in");
            ImGui::PopFont();
            return;
        }
        char preview[64];
        std::snprintf(preview, sizeof(preview), "%s%s", view.enabled ? "" : "→ ",
                      view.mode.c_str());
        ImGui::SetNextItemWidth(-px(8.0F));
        if (view.pending) {
            ImGui::PushStyleColor(ImGuiCol_Border, c::accentBorder);  // edited, not applied
        }
        const bool open = ImGui::BeginCombo("##mode", preview);
        if (view.pending) {
            ImGui::PopStyleColor();
        }
        if (open) {
            // Copy the choice first: the edit rebuilds the views `view` points into.
            std::string chosen;
            if (view.enabled) {
                for (const std::string& choice : choices) {
                    if (ImGui::Selectable(choice.c_str(), choice == view.mode)) {
                        chosen = choice;
                    }
                }
            } else {
                for (const SubstituteOption& option : info.substitutes) {
                    char label[64];
                    std::snprintf(label, sizeof(label), "→ %s", option.token.c_str());
                    if (ImGui::Selectable(label, option.token == view.mode)) {
                        chosen = option.token;
                    }
                    ImGui::SetItemTooltip("%s", option.description.c_str());
                }
            }
            ImGui::EndCombo();
            if (!chosen.empty()) {
                session.setMode(info.name, chosen);
            }
        }
        ImGui::PopFont();
    }

    static void stepField(ProfileSession& session, const DomainInfo& info, const DomainView& view) {
        if (!view.enabled || view.dtNs <= 0) {
            ImGui::AlignTextToFramePadding();
            widgets::Mono(c::t3, "-");
            return;
        }
        // Show the step in the unit a person would write it in.
        const char* unit = "s";
        double unitNs = 1e9;
        if (view.dtNs < 1'000'000) {
            unit = "µs";
            unitNs = 1e3;
        } else if (view.dtNs < 1'000'000'000) {
            unit = "ms";
            unitNs = 1e6;
        }
        double value = static_cast<double>(view.dtNs) / unitNs;
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(-px(8.0F));
        const bool edited = ImGui::InputDouble("##dt", &value, 0.0, 0.0, "%g");
        const ImVec2 min = ImGui::GetItemRectMin();
        const ImVec2 max = ImGui::GetItemRectMax();
        const ImVec2 size = ImGui::CalcTextSize(unit);
        ImGui::GetWindowDrawList()->AddText(
            ImVec2(max.x - size.x - px(6.0F), min.y + (max.y - min.y - size.y) * 0.5F), u32(c::t3),
            unit);
        ImGui::PopFont();
        if (info.minDtNs > 0 && info.maxDtNs > 0 && ImGui::IsItemHovered()) {
            char low[24];
            char high[24];
            format::step(info.minDtNs, low, sizeof(low));
            format::step(info.maxDtNs, high, sizeof(high));
            ImGui::SetTooltip("%s to %s", low, high);
        }
        if (edited && std::isfinite(value) && value > 0.0) {
            const double ns = std::round(value * unitNs);
            if (ns >= 1.0 && ns < 9e18) {
                session.setStep(info.name, static_cast<std::int64_t>(ns));
            }
        }
    }

    /// Each domain's CPU per simulated second (ms) from the server's latest statistics, which
    /// it measures every report; the profile's own figure only until the first report.
    void measure(const PanelContext& context) {
        const std::shared_ptr<const ServerStatistics> stats = context.client.statistics();
        if (!stats || stats->serial == statsSerial_) {
            return;
        }
        statsSerial_ = stats->serial;
        measured_.clear();
        for (const ServerStatistics::Module& module : stats->modules) {
            if (module.dtNs > 0) {
                measured_[module.domain] +=
                    module.cpuMsPerStep * 1e9 / static_cast<double>(module.dtNs);
            }
        }
    }

    /// ms of CPU per 1 ms of simulation.
    [[nodiscard]] double costOf(const DomainInfo& info) const {
        const auto it = measured_.find(info.name);
        const double perSimSecond = it != measured_.end() ? it->second : info.cpuMsPerSimS;
        return perSimSecond / 1000.0;
    }

    static void cost(const DomainInfo& info, const DomainView& view, double measured) {
        const double applied = info.enabled ? measured : 0.0;
        char text[32];
        if (view.pending && !view.enabled && applied > 0.0) {
            // What switching it off saves.
            std::snprintf(text, sizeof(text), applied < 0.01 ? "-<0.01" : "-%.2f", applied);
            rightMono(c::accent, text);
        } else if (view.pending) {
            rightMono(c::t3, "...");  // the server has not estimated the edited setting yet
        } else if (!view.enabled) {
            rightMono(c::t3, "0");
        } else if (applied < 0.01) {
            rightMono(c::t2, "<0.01");
        } else {
            std::snprintf(text, sizeof(text), "%.2f ms", applied);
            rightMono(c::t1, text);
        }
    }

    /// The domain whose substitutes the picker offers: the selected row, else the first
    /// domain with an edit waiting, else the first that is off, else the first that has
    /// substitutes, so the picker is there before anything is clicked (SPEC 03).
    static std::string pickerDomain(const ProfileSession& session, const ProfileInfo& profile) {
        if (!session.selectedDomain.empty()) {
            return session.selectedDomain;
        }
        const auto first = [&](auto&& wanted) -> const DomainInfo* {
            for (const DomainInfo& info : profile.domains) {
                const DomainView* view = session.find(info.name);
                if (info.available && !info.substitutes.empty() && view != nullptr &&
                    format::isPhysicsDomain(info.name) && wanted(*view)) {
                    return &info;
                }
            }
            return nullptr;
        };
        for (const DomainInfo* info : {first([](const DomainView& v) { return v.pending; }),
                                       first([](const DomainView& v) { return !v.enabled; }),
                                       first([](const DomainView&) { return true; })}) {
            if (info != nullptr) {
                return info->name;
            }
        }
        return {};
    }

    void substitutePicker(ProfileSession& session, const ProfileInfo& profile) {
        const std::string shown = pickerDomain(session, profile);
        const DomainInfo* info = profile.find(shown);
        const DomainView* view = session.find(shown);
        if (info == nullptr || view == nullptr || info->substitutes.empty()) {
            return;
        }
        const float pad = px(10.0F);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        ImGui::Indent(pad);
        const std::string_view label = widgets::domainLabel(info->name);
        widgets::Strong("%.*s substitute", static_cast<int>(label.size()), label.data());

        // Who reads this domain's signals: they will read from the substitute instead. The
        // modules of the wiring the server would build, as the Coupling tab lists them (a
        // controller is a module of no domain); the profile's dependencies before it arrives.
        struct Reader {
            std::string module;
            std::vector<std::string> signals;
            std::string unit;  // of the first signal
        };
        std::vector<Reader> readers;
        for (const Coupling& pair : couplings(session.wiring())) {
            if (pair.fromDomain != info->name || pair.toDomain == info->name) {
                continue;
            }
            auto reader = std::ranges::find(readers, pair.to, &Reader::module);
            if (reader == readers.end()) {
                readers.push_back({pair.to, {}, {}});
                reader = std::prev(readers.end());
            }
            for (const WiringInfo::Wire& wire : session.wiring().wires) {
                if (wire.from == pair.from && wire.to == pair.to &&
                    std::ranges::find(reader->signals, wire.signal) == reader->signals.end()) {
                    if (reader->signals.empty()) {
                        reader->unit = wire.sinkUnit;
                    }
                    reader->signals.push_back(wire.signal);
                }
            }
        }
        if (session.wiring().empty()) {
            for (const DomainInfo& other : profile.domains) {
                if (other.enabled &&
                    std::ranges::find(other.dependsOn, info->name) != other.dependsOn.end()) {
                    readers.push_back({other.name, {}, {}});
                }
            }
        }
        const std::size_t count = readers.size();
        if (count != 0) {
            char pill[48];
            std::snprintf(pill, sizeof(pill), "affects %zu %s", count,
                          count == 1 ? "consumer" : "consumers");
            ImGui::SameLine(0.0F, px(10.0F));
            widgets::Pill(pill, widgets::PillKind::Warn);
        }

        const auto options = static_cast<float>(std::min<std::size_t>(info->substitutes.size(), 3));
        const float gap = px(8.0F);
        const float width =
            (ImGui::GetContentRegionAvail().x - pad - gap * (options - 1.0F)) / options;
        std::string chosen;
        for (std::size_t i = 0; i < info->substitutes.size(); ++i) {
            const SubstituteOption& option = info->substitutes[i];
            if (i % 3 != 0) {
                ImGui::SameLine(0.0F, gap);
            }
            char name[96];
            std::snprintf(name, sizeof(name), "%s · %s", option.kind.c_str(), option.token.c_str());
            const bool selected = !view->enabled && view->mode == option.token;
            // The server's description, else what the kind means (CLAUDE.md §11.2).
            const char* description = option.description.c_str();
            if (option.description.empty()) {
                description = option.kind == "ideal"      ? "the ideal value, no losses"
                              : option.kind == "constant" ? "a fixed value or model"
                              : option.kind == "replay"   ? "recorded channels from an MCAP file"
                                                          : "";
            }
            if (widgets::OptionCard(option.token.c_str(), name, description, selected,
                                    ImVec2(width, px(58.0F)))) {
                chosen = option.token;
            }
        }
        if (!chosen.empty()) {
            const std::string domain = info->name;
            if (session.find(domain)->enabled) {
                session.setEnabled(domain, false);
            }
            session.setMode(domain, chosen);
            view = session.find(domain);
        }

        if (!view->enabled && view->needsSource) {
            // Keep the text field in step with the profile unless the user is typing in it.
            if (sourceFor_ != view->name || (!sourceActive_ && view->source != source_)) {
                sourceFor_ = view->name;
                std::snprintf(source_, sizeof(source_), "%s", view->source.c_str());
            }
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - pad);
            const std::string domain = view->name;
            if (ImGui::InputTextWithHint("##source", "recording to replay, e.g. logs/run.mcap",
                                         source_, sizeof(source_))) {
                session.setSource(domain, source_);
            }
            sourceActive_ = ImGui::IsItemActive();
            ImGui::PopFont();
        }
        // Each reader with the first signal it takes from this domain:
        // "→ sensors.body/base_link/pose +3 [m] will read from the substitute".
        const char* reads = session.find(info->name)->enabled ? "would read from the substitute"
                                                              : "will read from the substitute";
        for (std::size_t i = 0; i < std::min(count, kListedReaders); ++i) {
            const Reader& reader = readers[i];
            widgets::Mono(c::info, "→");
            ImGui::SameLine(0.0F, px(6.0F));
            if (reader.signals.empty()) {
                widgets::Mono(c::t1, "%s", reader.module.c_str());
            } else {
                widgets::Mono(c::t1, "%s.%s", reader.module.c_str(), reader.signals[0].c_str());
                if (reader.signals.size() > 1) {
                    ImGui::SameLine(0.0F, px(6.0F));
                    widgets::Mono(c::t3, "+%zu", reader.signals.size() - 1);
                    std::string all;
                    for (const std::string& signal : reader.signals) {
                        all += (all.empty() ? "" : "\n") + signal;
                    }
                    ImGui::SetItemTooltip("%s", all.c_str());
                }
            }
            ImGui::SameLine(0.0F, px(6.0F));
            if (reader.unit.empty()) {
                widgets::Caption("%s", reads);
            } else {
                widgets::Caption("[%s] %s", reader.unit.c_str(), reads);
            }
        }
        if (count > kListedReaders) {
            widgets::Caption("+%zu more consumers (Coupling tab)", count - kListedReaders);
        }
        ImGui::Unindent(pad);
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
    }

    std::map<std::string, double, std::less<>> measured_;  // ms per simulated second
    std::uint64_t statsSerial_{0};
    char source_[256]{};
    std::string sourceFor_;
    bool sourceActive_{false};
};

}  // namespace

std::unique_ptr<Panel> createDomainsPanel() {
    return std::make_unique<DomainsPanel>();
}

}  // namespace lagrange::viewer
