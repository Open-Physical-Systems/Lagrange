// Domains workspace, the secondary tabs of docs/design/ui/03-domains/SPEC.md: "Modules · N"
// beside Domains, "Coupling" beside Schedule, "Diff · N" and "Schema" beside Profile. They
// show only what came through the protocol (the server's statistics, its wiring of the draft,
// its applied profile) and the profile schema shipped with this install (invariant 14).
#include "DomainsTabs.hpp"
#include "PanelFormat.hpp"
#include "YamlText.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/ProfileExtras.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// The design's table header row: captions on the raised surface.
void headerRow(std::initializer_list<const char*> names, std::initializer_list<int> rightAligned) {
    ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
    int column = 0;
    for (const char* name : names) {
        ImGui::TableSetColumnIndex(column);
        if (std::ranges::find(rightAligned, column) != rightAligned.end()) {
            ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
            widgets::AlignRight(ImGui::CalcTextSize(name).x + px(8.0F));
            ImGui::PopFont();
        } else {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption("%s", name);
        ++column;
    }
}

void rightMono(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    widgets::AlignRight(ImGui::CalcTextSize(text).x + px(8.0F));
    ImGui::PopFont();
    widgets::Mono(color, "%s", text);
}

/// The panel's one line of guidance before the profile arrives. Returns true if it wrote one.
bool noProfile(const PanelContext& context, const char* what) {
    if (context.profileSession->profile()) {
        return false;
    }
    if (context.connection.state == ConnectionState::Ready) {
        widgets::Caption("Waiting for the server's profile ...");
    } else {
        widgets::Caption("Not connected. %s appears when a simulation server is reachable.", what);
    }
    return true;
}

constexpr ImGuiTableFlags kTableFlags = ImGuiTableFlags_BordersInnerH |
                                        ImGuiTableFlags_SizingStretchProp |
                                        ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoPadOuterX;

// --- Modules · N ----------------------------------------------------------------------------

/// Every module the server runs, as it measures them: its step, rate and CPU, and what one
/// millisecond of simulation costs. Substitutes the wiring lists cost nothing and say so.
class ModulesPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Modules"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        const std::shared_ptr<const ServerStatistics> stats = context.client.statistics();
        const WiringInfo& wiring = context.profileSession->wiring();
        std::size_t substitutes = 0;
        for (const WiringInfo::Node& node : wiring.nodes) {
            substitutes += node.substitute ? 1U : 0U;
        }
        const std::size_t count = (stats ? stats->modules.size() : 0U) + substitutes;
        char label[64];
        std::snprintf(label, sizeof(label), "Modules · %zu###DomainsModules", count);
        if (!ImGui::Begin(label)) {
            ImGui::End();
            return;
        }
        if (noProfile(context, "The module list")) {
            ImGui::End();
            return;
        }
        if (!stats || stats->modules.empty()) {
            widgets::Caption("Waiting for the server's measurements ...");
            ImGui::End();
            return;
        }
        widgets::Caption("%zu running · %zu %s  ·  CPU per step, measured by the server",
                         stats->modules.size(), substitutes,
                         substitutes == 1 ? "substitute" : "substitutes");
        if (stats->estimated) {
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill("estimates", widgets::PillKind::Warn);
        }
        const float footer = ImGui::GetFrameHeightWithSpacing();
        double total = 0.0;
        if (ImGui::BeginTable("##modules", 6, kTableFlags, ImVec2(0.0F, -footer))) {
            ImGui::TableSetupColumn("module", ImGuiTableColumnFlags_None, 2.6F);
            ImGui::TableSetupColumn("step", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("steps/s", ImGuiTableColumnFlags_None, 1.1F);
            ImGui::TableSetupColumn("per step", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableSetupColumn("max", ImGuiTableColumnFlags_None, 1.1F);
            ImGui::TableSetupColumn("per 1 ms", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableSetupScrollFreeze(0, 1);
            headerRow({"module", "step", "steps/s", "per step", "max", "per 1 ms"},
                      {1, 2, 3, 4, 5});
            for (const ServerStatistics::Module& module : stats->modules) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(28.0F));
                ImGui::TableNextColumn();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
                ImGui::AlignTextToFramePadding();
                widgets::Dot(widgets::domainColor(module.domain));
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "%s", module.module.c_str());
                ImGui::SetItemTooltip("%s · %s", module.domain.c_str(),
                                      module.onGpu ? "GPU work" : "CPU");
                char text[32];
                ImGui::TableNextColumn();
                format::step(module.dtNs, text, sizeof(text));
                rightMono(c::t2, module.dtNs > 0 ? text : "-");
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.0f", module.stepsPerSecond);
                rightMono(c::t2, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f ms", module.cpuMsPerStep);
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f", module.cpuMsPerStepMax);
                rightMono(c::t3, text);
                ImGui::TableNextColumn();
                // CPU per 1 ms of simulation: steps in 1 ms times the cost of one.
                const double perMs =
                    module.dtNs > 0 ? module.cpuMsPerStep * 1e6 / static_cast<double>(module.dtNs)
                                    : 0.0;
                total += perMs;
                if (perMs < 0.01) {
                    std::snprintf(text, sizeof(text), "<0.01%s", module.onGpu ? " gpu" : "");
                } else {
                    std::snprintf(text, sizeof(text), "%.2f%s", perMs, module.onGpu ? " gpu" : "");
                }
                rightMono(perMs < 0.01 ? c::t2 : c::t1, text);
            }
            for (const WiringInfo::Node& node : wiring.nodes) {
                if (!node.substitute) {
                    continue;
                }
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(28.0F));
                ImGui::TableNextColumn();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
                ImGui::AlignTextToFramePadding();
                widgets::Dot(widgets::domainColor(node.domain), false);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::accent, "%s → %s", node.domain.c_str(), node.detail.c_str());
                ImGui::TableNextColumn();
                rightMono(c::t3, "-");
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                widgets::Caption("substitute");
                ImGui::TableNextColumn();
                ImGui::TableNextColumn();
                rightMono(c::t3, "0");
            }
            ImGui::EndTable();
        }
        ImGui::AlignTextToFramePadding();
        widgets::Caption(
            "est. %.2f ms CPU per 1 ms of sim  ·  RTF %.2f  ·  tick %.2f ms (max %.2f)", total,
            stats->realTimeFactor, stats->tickMs, stats->tickMsMax);
        ImGui::End();
    }
};

// --- Coupling ------------------------------------------------------------------------------

/// How the modules are coupled: each writer → reader pair of the wiring, what passes between
/// them, at which rates, and whether the pair is stiff (solved implicitly) or read through a
/// hold (a fast reader of a slow writer).
class CouplingPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Coupling"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (noProfile(context, "The coupling")) {
            ImGui::End();
            return;
        }
        const ProfileSession& session = *context.profileSession;
        if (session.wiring().empty()) {
            widgets::Caption("Waiting for the server to check the profile ...");
            ImGui::End();
            return;
        }
        const WiringInfo& wiring = session.wiring();
        pairs_ = couplings(wiring);
        std::size_t stiff = 0;
        for (const Coupling& pair : pairs_) {
            stiff += pair.stiff ? 1U : 0U;
        }
        char sync[24];
        format::step(wiring.syncNs, sync, sizeof(sync));
        widgets::Caption("%zu couplings · %zu wires · %zu stiff  ·  signals exchanged every %s",
                         pairs_.size(), wiring.wires.size(), stiff, sync);
        if (session.draft().dirty()) {
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill("preview — not applied", widgets::PillKind::Accent);
        }
        if (ImGui::BeginTable("##coupling", 4, kTableFlags)) {
            ImGui::TableSetupColumn("writer → reader", ImGuiTableColumnFlags_None, 3.0F);
            ImGui::TableSetupColumn("signals", ImGuiTableColumnFlags_None, 2.8F);
            ImGui::TableSetupColumn("rates", ImGuiTableColumnFlags_None, 1.4F);
            ImGui::TableSetupColumn("coupling", ImGuiTableColumnFlags_None, 1.4F);
            ImGui::TableSetupScrollFreeze(0, 1);
            headerRow({"writer → reader", "signals", "rates", "coupling"}, {});
            for (const Coupling& pair : pairs_) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(26.0F));
                ImGui::TableNextColumn();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
                ImGui::AlignTextToFramePadding();
                widgets::Dot(widgets::domainColor(pair.fromDomain), !pair.fromSubstitute);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(pair.fromSubstitute ? c::accent : c::t1, "%s", pair.from.c_str());
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t3, "→");
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Dot(widgets::domainColor(pair.toDomain));
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "%s", pair.to.c_str());

                ImGui::TableNextColumn();
                std::string symbols;
                for (const std::string& symbol : pair.symbols) {
                    symbols += (symbols.empty() ? "" : ", ") + symbol;
                }
                ImGui::AlignTextToFramePadding();
                widgets::Mono(pair.unitConflicts == 0 ? c::t2 : c::error, "%s", symbols.c_str());
                ImGui::SetItemTooltip("%zu %s%s", pair.wires, pair.wires == 1 ? "wire" : "wires",
                                      pair.unitConflicts == 0 ? "" : " · unit conflict");

                ImGui::TableNextColumn();
                char from[24];
                char to[24];
                format::step(pair.fromDtNs, from, sizeof(from));
                format::step(pair.toDtNs, to, sizeof(to));
                ImGui::AlignTextToFramePadding();
                widgets::Mono(c::t2, "%s → %s", pair.fromDtNs > 0 ? from : "-",
                              pair.toDtNs > 0 ? to : "-");

                ImGui::TableNextColumn();
                if (pair.unitConflicts != 0) {
                    widgets::Pill("unit conflict", widgets::PillKind::Error);
                } else if (pair.stiff) {
                    widgets::Pill("stiff · implicit", widgets::PillKind::Warn);
                } else if (pair.fromSubstitute) {
                    widgets::Pill("from substitute", widgets::PillKind::Accent);
                } else if (pair.holdSteps > 1) {
                    char hold[32];
                    std::snprintf(hold, sizeof(hold), "hold ×%lld",
                                  static_cast<long long>(pair.holdSteps));
                    widgets::Pill(hold, widgets::PillKind::Neutral);
                    ImGui::SetItemTooltip(
                        "The reader steps faster than the writer: it reads each value %lld times.",
                        static_cast<long long>(pair.holdSteps));
                } else {
                    ImGui::AlignTextToFramePadding();
                    widgets::Caption("%s", pair.twoWay ? "two-way, explicit" : "one-way");
                }
            }
            ImGui::EndTable();
        }
        ImGui::End();
    }

private:
    std::vector<Coupling> pairs_;
};

// --- Diff · N ------------------------------------------------------------------------------

/// The applied profile against the one being edited, line by line, both line numbers in the
/// gutter (removed lines on a red tint, added on a green one, as in the Profile tab).
class DiffPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Diff"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        const ProfileSession& session = *context.profileSession;
        const std::shared_ptr<const ProfileInfo>& profile = session.profile();
        if (profile && (profile->serial != serial_ || session.draft().revision() != revision_)) {
            serial_ = profile->serial;
            revision_ = session.draft().revision();
            diff_ = diffLines(profile->yaml, session.draft().text());
            hunks_ = diffHunks(diff_);
            added_ = 0;
            removed_ = 0;
            for (const DiffLine& line : diff_) {
                added_ += line.kind == DiffLine::Kind::Added ? 1U : 0U;
                removed_ += line.kind == DiffLine::Kind::Removed ? 1U : 0U;
            }
        }
        char label[48];
        if (profile && hunks_ != 0) {
            std::snprintf(label, sizeof(label), "Diff · %zu###ProfileDiff", hunks_);
        } else {
            std::snprintf(label, sizeof(label), "Diff###ProfileDiff");
        }
        if (!ImGui::Begin(label)) {
            ImGui::End();
            return;
        }
        if (noProfile(context, "The diff")) {
            ImGui::End();
            return;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Mono(c::t1, "applied → draft");
        ImGui::SameLine(0.0F, px(10.0F));
        if (hunks_ == 0) {
            widgets::Caption("no changes");
        } else {
            widgets::Mono(c::ok, "+%zu", added_);
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Mono(c::error, "−%zu", removed_);
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Caption("in %zu %s", hunks_, hunks_ == 1 ? "place" : "places");
        }
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##diff", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Borders);
        ImGui::PopStyleColor();
        if (hunks_ == 0) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            ImGui::Indent(px(10.0F));
            widgets::Caption(
                "The draft is the profile the server runs. Edit a domain or the "
                "YAML and the changes show here.");
            ImGui::Unindent(px(10.0F));
        } else {
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(diff_.size()), px(yaml_text::kLineHeight));
            while (clipper.Step()) {
                for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                    const DiffLine& line = diff_[static_cast<std::size_t>(i)];
                    char gutter[24];
                    yaml_text::Tint tint = yaml_text::Tint::None;
                    switch (line.kind) {
                        case DiffLine::Kind::Removed:
                            std::snprintf(gutter, sizeof(gutter), "%d   −", line.beforeLine);
                            tint = yaml_text::Tint::Removed;
                            break;
                        case DiffLine::Kind::Added:
                            std::snprintf(gutter, sizeof(gutter), "%d +", line.afterLine);
                            tint = yaml_text::Tint::Added;
                            break;
                        default:
                            std::snprintf(gutter, sizeof(gutter), "%d %3d", line.beforeLine,
                                          line.afterLine);
                            break;
                    }
                    yaml_text::codeLine(line.text, gutter, tint, kDiffGutter);
                }
            }
            ImGui::PopStyleVar();
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    static constexpr float kDiffGutter = 78.0F;  // two line numbers and a sign (logical px)

    std::uint64_t serial_{0};
    std::uint64_t revision_{static_cast<std::uint64_t>(-1)};
    std::vector<DiffLine> diff_;
    std::size_t hunks_{0};
    std::size_t added_{0};
    std::size_t removed_{0};
};

// --- Schema --------------------------------------------------------------------------------

/// The profile schema this install ships (profiles/schema.json, not the server's data): the
/// selected domain's keys first, with the values the draft gives them, then the top level and
/// every other domain.
class SchemaPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Schema"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (!loaded_) {
            doc_ = loadProfileSchema(findProfileSchema());
            loaded_ = true;
        }
        if (!doc_.error.empty()) {
            widgets::Pill("schema not found", widgets::PillKind::Warn);
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", doc_.error.c_str());
            ImGui::PopTextWrapPos();
            ImGui::End();
            return;
        }
        ImGui::AlignTextToFramePadding();
        widgets::Mono(c::t1, "profiles/schema.json");
        ImGui::SetItemTooltip("%s", doc_.path.string().c_str());
        char pill[48];
        std::snprintf(pill, sizeof(pill), "schema v%u · JSON Schema 2020-12", doc_.version);
        ImGui::SameLine();
        widgets::AlignRight(widgets::PillWidth(pill));
        widgets::Pill(pill, widgets::PillKind::Neutral);

        ProfileSession& session = *context.profileSession;
        ImGui::BeginChild("##schema", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None);
        // The domain selected in the Domains table; rigid_body before anything is selected.
        const std::string selected =
            session.selectedDomain.empty() ? std::string("rigid_body") : session.selectedDomain;
        if (const SchemaSection* section = doc_.domain(selected)) {
            drawSection(session, *section, true, "selected");
        }
        drawSection(session, doc_.top, false, "top level");
        for (const SchemaSection& section : doc_.domains) {
            if (section.name != selected) {
                const bool required = std::ranges::find(doc_.requiredDomains, section.name) !=
                                      doc_.requiredDomains.end();
                drawSection(session, section, false, required ? "required" : "optional");
            }
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    void drawSection(const ProfileSession& session, const SchemaSection& section, bool open,
                     const char* meta) {
        const bool domain = section.name != "profile";
        const ImVec4 color = widgets::domainColor(section.name);
        // The header's text is its ID too: the PushID below keeps the titles unique.
        const std::string heading = domain ? "domains." + section.name : "profile";
        ImGui::PushID(section.name.c_str());
        // Opened by the selection: open it once when the selection changes.
        if (open && section.name != openedFor_) {
            ImGui::SetNextItemOpen(true);
            openedFor_ = section.name;
        }
        if (!widgets::SectionHeader(heading.c_str(), meta, open, domain ? &color : nullptr)) {
            ImGui::PopID();
            return;
        }
        // The values the draft gives this domain, to show beside the keys.
        std::optional<std::vector<ProfileEntry>> entries;
        if (domain) {
            entries = session.draft().domain(section.name);
        }
        ImGui::Indent(px(10.0F));
        if (!section.note.empty()) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", section.note.c_str());
            ImGui::PopTextWrapPos();
        }
        for (const SchemaField& field : section.fields) {
            drawField(field, entries ? findEntry(*entries, field.key) : nullptr, field.depth);
        }
        ImGui::Unindent(px(10.0F));
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        ImGui::PopID();
    }

    static void drawField(const SchemaField& field, const std::string* value, int depth) {
        const float indent = px(14.0F) * static_cast<float>(depth);
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
        widgets::Mono(theme::hex(0x8FB8FF), "%s", field.key.c_str());  // the YAML key colour
        if (field.required) {
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Caption("required");
        }
        if (value != nullptr && depth == 0) {
            ImGui::SameLine(0.0F, px(10.0F));
            widgets::Mono(c::accent, "= %s", value->c_str());
        }
        std::string line = field.type;
        if (!field.constraints.empty()) {
            line += (line.empty() ? "" : "  ") + field.constraints;
        }
        if (!field.defaultValue.empty()) {
            line += (line.empty() ? "default " : "  default ") + field.defaultValue;
        }
        if (!line.empty()) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent + px(10.0F));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Mono(c::t2, "%s", line.c_str());
            ImGui::PopTextWrapPos();
        }
        if (!field.description.empty()) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent + px(10.0F));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", field.description.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    bool loaded_{false};
    ProfileSchemaDoc doc_;
    std::string openedFor_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createDomainsTabPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<ModulesPanel>());
    panels.push_back(std::make_unique<CouplingPanel>());
    panels.push_back(std::make_unique<DiffPanel>());
    panels.push_back(std::make_unique<SchemaPanel>());
    return panels;
}

}  // namespace lagrange::viewer
