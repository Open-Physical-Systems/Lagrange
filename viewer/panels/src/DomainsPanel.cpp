// Domains workspace, left panel: one row per domain with its switch, mode, step and cost,
// the substitute picker, and the additional modules (docs/design/ui/03-domains/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

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
            for (const DomainInfo& info : profile->domains) {
                if (format::isPhysicsDomain(info.name)) {
                    row(session, info, kRowHeight);
                }
            }
            ImGui::EndTable();
        }
        substitutePicker(session, *profile);

        if (widgets::SectionHeader("Additional modules", "from robot class")) {
            if (ImGui::BeginTable("##modules", 6, kTableFlags)) {
                setupColumns();
                for (const DomainInfo& info : profile->domains) {
                    if (!format::isPhysicsDomain(info.name)) {
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
            total += info.enabled ? info.cpuMsPerSimS / 1000.0 : 0.0;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("est. %.2f ms CPU per 1 ms of sim  ·  %s%s", total,
                         total <= kRealTimeBudgetMs ? "fits real time" : "slower than real time",
                         session.draft().dirty() ? "  ·  for the applied profile" : "");
        ImGui::End();
    }

private:
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
            widgets::Caption("%s", info.unavailableReason.c_str());
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
        cost(info, *view);
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

    static void cost(const DomainInfo& info, const DomainView& view) {
        const double applied = info.enabled ? info.cpuMsPerSimS / 1000.0 : 0.0;
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

    void substitutePicker(ProfileSession& session, const ProfileInfo& profile) {
        const DomainInfo* info = profile.find(session.selectedDomain);
        const DomainView* view = session.find(session.selectedDomain);
        if (info == nullptr || view == nullptr || info->substitutes.empty()) {
            return;
        }
        const float pad = px(10.0F);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        ImGui::Indent(pad);
        const std::string_view label = widgets::domainLabel(info->name);
        widgets::Strong("%.*s substitute", static_cast<int>(label.size()), label.data());

        // Who reads this domain's signals: they will read from the substitute instead.
        char consumers[160] = "";
        int used = 0;
        int count = 0;
        for (const DomainInfo& other : profile.domains) {
            const bool reads =
                std::ranges::find(other.dependsOn, info->name) != other.dependsOn.end();
            if (reads && other.enabled && used < static_cast<int>(sizeof(consumers))) {
                used += std::snprintf(consumers + used,
                                      sizeof(consumers) - static_cast<std::size_t>(used), "%s%s",
                                      count == 0 ? "" : ", ", other.name.c_str());
                ++count;
            }
        }
        if (count != 0) {
            char pill[48];
            std::snprintf(pill, sizeof(pill), "affects %d %s", count,
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
            if (widgets::OptionCard(option.token.c_str(), name, option.description.c_str(),
                                    selected, ImVec2(width, px(58.0F)))) {
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
        if (count != 0) {
            widgets::Mono(c::info, "→");
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Mono(c::t1, "%s", consumers);
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Caption("%s", session.find(info->name)->enabled
                                       ? "would read from the substitute"
                                       : "will read from the substitute");
        }
        ImGui::Unindent(pad);
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
    }

    char source_[256]{};
    std::string sourceFor_;
    bool sourceActive_{false};
};

}  // namespace

std::unique_ptr<Panel> createDomainsPanel() {
    return std::make_unique<DomainsPanel>();
}

}  // namespace lagrange::viewer
