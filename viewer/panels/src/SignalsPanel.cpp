// Signals workspace: the signal browser on the left, and the cursor table and statistics on
// the right (docs/design/ui/05-signals/SPEC.md). Their second tabs (Saved sets, Measure, FFT)
// are in SignalsTabs.cpp.
#include "PanelFormat.hpp"
#include "SignalsCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/SignalList.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using signals::headerCell;
using signals::kTableFlags;
using signals::number;
using signals::rightMono;
using signals::traceCell;
using widgets::px;

constexpr std::int64_t kFastDtNs = 10'000'000;      // "rate >= 100 Hz"
constexpr double kStatisticsIntervalSeconds = 0.2;  // statistics are recomputed at 5 Hz

/// A search the command palette asked for; the Signals panel takes it.
std::string& requestedSearch() {
    static std::string search;
    return search;
}

void rate(std::int64_t dtNs, char* out, std::size_t size) {
    const double hz = 1e9 / static_cast<double>(std::max<std::int64_t>(dtNs, 1));
    if (hz >= 1000.0) {
        std::snprintf(out, size, "%g kHz", hz / 1000.0);
    } else {
        std::snprintf(out, size, "%g Hz", hz);
    }
}

// --- browser ------------------------------------------------------------------------------

class SignalsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Signals"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (!context.catalog) {
            widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                       ? "Waiting for the signal list ..."
                                       : "Not connected. Signals appear when a simulation "
                                         "server is reachable.");
            ImGui::End();
            return;
        }
        const SignalCatalog& catalog = *context.catalog;
        if (!requestedSearch().empty()) {
            std::snprintf(search_, sizeof(search_), "%s", requestedSearch().c_str());
            requestedSearch().clear();
            for (Group& group : groups_) {
                group.selected = false;
            }
            unit_.clear();
            fastOnly_ = false;
            dirty_ = true;
        }
        if (catalog.generation != generation_) {
            generation_ = catalog.generation;
            index(catalog);
            dirty_ = true;
        }

        // Search, with how many signals match.
        char count[32];
        std::snprintf(count, sizeof(count), "%zu of %zu", matches_, total_);
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        const float countWidth = ImGui::CalcTextSize(count).x;
        ImGui::PopFont();
        const ImVec2 at = ImGui::GetCursorPos();
        const float field = ImGui::GetContentRegionAvail().x - countWidth - px(10.0F);
        if (widgets::SearchField("##search", "Search name or unit", search_, sizeof(search_),
                                 field)) {
            dirty_ = true;
        }
        ImGui::SameLine(0.0F, 0.0F);
        ImGui::SetCursorPos(ImVec2(at.x + field + px(8.0F), at.y));
        ImGui::AlignTextToFramePadding();
        widgets::Mono(c::t3, "%s", count);

        // Filter chips: one per domain that has signals, and the rate.
        for (std::size_t i = 0; i < groups_.size(); ++i) {
            if (i != 0) {
                ImGui::SameLine(0.0F, px(6.0F));
            }
            const std::string_view label = widgets::domainLabel(groups_[i].domain);
            char text[64];
            std::snprintf(text, sizeof(text), "%.*s##chip%zu", static_cast<int>(label.size()),
                          label.data(), i);
            if (widgets::FilterChip(text, widgets::domainColor(groups_[i].domain),
                                    groups_[i].selected)) {
                groups_[i].selected = !groups_[i].selected;
                dirty_ = true;
            }
        }
        // The unit: any, or one of those the catalog has (the design's "unit: any ▾").
        ImGui::SameLine(0.0F, px(6.0F));
        char unitText[64];
        std::snprintf(unitText, sizeof(unitText), "unit: %s##unit",
                      unit_.empty() ? "any" : (unit_ == "1" ? "-" : unit_.c_str()));
        if (widgets::DropdownChip(unitText, c::info, !unit_.empty())) {
            ImGui::OpenPopup("##units");
        }
        if (ImGui::BeginPopup("##units")) {
            if (ImGui::Selectable("any", unit_.empty())) {
                unit_.clear();
                dirty_ = true;
            }
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            for (const std::string& u : units_) {
                if (ImGui::Selectable(u == "1" ? "-" : u.c_str(), unit_ == u)) {
                    unit_ = u;
                    dirty_ = true;
                }
            }
            ImGui::PopFont();
            ImGui::EndPopup();
        }
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::FilterChip("rate ≥ 100 Hz", c::info, fastOnly_)) {
            fastOnly_ = !fastOnly_;
            dirty_ = true;
        }
        if (dirty_) {
            filter(catalog);
        }

        const float footer = ImGui::GetTextLineHeightWithSpacing() + px(2.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        ImGui::BeginChild("##list", ImVec2(0.0F, -footer));
        ImGui::PopStyleVar();
        ScopeSession& session = *context.scopeSession;
        for (Group& group : groups_) {
            if (group.rows.empty()) {
                continue;
            }
            const std::string_view label = widgets::domainLabel(group.domain);
            char name[64];
            std::snprintf(name, sizeof(name), "%.*s", static_cast<int>(label.size()), label.data());
            char meta[24];
            std::snprintf(meta, sizeof(meta), "%zu", group.signals);  // signals, not rows
            if (widgets::SectionHeader(name, meta)) {
                // Only the rows in view: a big robot has thousands.
                ImGuiListClipper clipper;
                clipper.Begin(static_cast<int>(group.rows.size()));
                while (clipper.Step()) {
                    for (int k = clipper.DisplayStart; k < clipper.DisplayEnd; ++k) {
                        if (row(context, session, catalog,
                                group.rows[static_cast<std::size_t>(k)])) {
                            dirty_ = true;  // opened or closed: the rows change next frame
                        }
                    }
                }
            }
        }
        if (matches_ == 0) {
            widgets::Caption("  No signal matches.");
        }
        ImGui::EndChild();
        widgets::Caption("tick a signal or drag it onto a scope  ·  %zu plotted",
                         session.traces().size());
        ImGui::End();
    }

private:
    struct Group {
        std::string domain;
        bool selected{false};  // filter chip
        std::vector<SignalRow> rows;
        std::size_t signals{0};  // listed in rows
    };

    /// The domains of the catalog, in the order they first appear.
    void index(const SignalCatalog& catalog) {
        std::vector<Group> groups;
        total_ = 0;
        for (const SignalInfo& info : catalog.signals) {
            total_ += info.components == 0 ? 0 : 1;  // signals, not their components
            const std::string domain = signalDomain(info);
            const bool known = std::ranges::any_of(
                groups, [&](const Group& group) { return group.domain == domain; });
            if (!known) {
                Group group;
                group.domain = domain;
                for (const Group& old : groups_) {
                    group.selected = group.selected || (old.domain == domain && old.selected);
                }
                groups.push_back(std::move(group));
            }
        }
        groups_ = std::move(groups);
        units_.clear();
        for (const SignalInfo& info : catalog.signals) {
            units_.push_back(info.unit);
        }
        std::ranges::sort(units_);
        units_.erase(std::unique(units_.begin(), units_.end()), units_.end());
        // The unit and rate columns: as wide as their widest entries.
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        unitColumn_ = px(24.0F);
        for (const std::string& u : units_) {
            unitColumn_ = std::max(unitColumn_, ImGui::CalcTextSize(u.c_str()).x + px(8.0F));
        }
        unitColumn_ = std::min(unitColumn_, px(90.0F));
        rateColumn_ = px(24.0F);
        char hz[24];
        for (const SignalInfo& info : catalog.signals) {
            rate(info.dtNs, hz, sizeof(hz));
            rateColumn_ = std::max(rateColumn_, ImGui::CalcTextSize(hz).x);
        }
        ImGui::PopFont();
        if (!unit_.empty() && !std::ranges::binary_search(units_, unit_)) {
            unit_.clear();  // no longer in the run
        }
    }

    void filter(const SignalCatalog& catalog) {
        const bool anyDomain =
            std::ranges::any_of(groups_, [](const Group& group) { return group.selected; });
        SignalFilter filter;
        filter.search = search_;
        filter.unit = unit_;
        filter.fastOnly = fastOnly_;
        filter.fastDtNs = kFastDtNs;
        matches_ = 0;
        for (Group& group : groups_) {
            group.rows.clear();
            group.signals = 0;
            if (anyDomain && !group.selected) {
                continue;
            }
            group.rows = signalRows(catalog, group.domain, filter, open_, &group.signals);
            matches_ += group.signals;
        }
        dirty_ = false;
    }

    /// One row. Returns true when an array's row was opened or closed.
    bool row(PanelContext& context, ScopeSession& session, const SignalCatalog& catalog,
             const SignalRow& entry) {
        const SignalInfo& info = catalog.signals[entry.signal];
        const bool whole = entry.component == SignalRow::kWhole;
        const std::uint32_t component = whole ? 0 : entry.component;
        const int plotted = whole ? -1 : session.find(info.name, component);
        ImGui::PushID(static_cast<int>(entry.signal));
        ImGui::PushID(static_cast<int>(entry.component));
        // Columns at fixed places from the right edge: unit, then rate, each as wide as the
        // widest of the catalog needs; the name has the rest.
        const float rowEnd = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - px(8.0F);
        const float unitX = rowEnd - rateColumn_ - px(8.0F) - unitColumn_;
        bool toggled = false;
        bool opened = false;
        if (whole) {
            // An array or a pose: a caret in place of the box, its size after the name.
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(10.0F));
            const ImVec2 at = ImGui::GetCursorScreenPos();
            const float side = px(14.0F);  // as wide as the box of a plain row
            const float height = ImGui::GetFrameHeight();
            if (ImGui::InvisibleButton("##open", ImVec2(side, height))) {
                opened = true;
            }
            ImGui::SetItemTooltip("%s", entry.expanded ? "Close" : "Show each component");
            const ImVec2 mid(at.x + side * 0.5F, at.y + height * 0.5F);
            const float r = px(3.5F);
            const ImU32 ink =
                ImGui::ColorConvertFloat4ToU32(ImGui::IsItemHovered() ? c::t1 : c::t3);
            if (entry.expanded) {
                ImGui::GetWindowDrawList()->AddTriangleFilled(ImVec2(mid.x - r, mid.y - r * 0.5F),
                                                              ImVec2(mid.x + r, mid.y - r * 0.5F),
                                                              ImVec2(mid.x, mid.y + r * 0.7F), ink);
            } else {
                ImGui::GetWindowDrawList()->AddTriangleFilled(ImVec2(mid.x - r * 0.5F, mid.y - r),
                                                              ImVec2(mid.x - r * 0.5F, mid.y + r),
                                                              ImVec2(mid.x + r * 0.7F, mid.y), ink);
            }
        } else {
            // A component of an opened row is indented under it.
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                                 (info.components > 1 ? px(32.0F) : px(10.0F)));
            bool on = plotted >= 0;
            toggled = widgets::Checkbox("##plot", &on);
        }
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::AlignTextToFramePadding();
        if (plotted >= 0) {
            widgets::Swatch(
                theme::kSeries[session.traces()[static_cast<std::size_t>(plotted)].color %
                               ScopeSession::kColors]);
            ImGui::SameLine(0.0F, px(6.0F));
        }
        // The full path of what the row plots, as the tooltip and the drag payload give it.
        const std::string path = whole ? info.name : componentPath(info, component);
        // The name: a component row shows its component only; an array its size.
        std::string shown;
        if (whole) {
            shown = info.name;
        } else if (info.components > 1) {
            shown = "." + componentLabel(info, component);
        } else {
            shown = info.name;
        }
        char size[24] = "";
        if (whole) {
            std::snprintf(size, sizeof(size), " [%u]", info.components);
        }
        // The unit and the rate keep their place on the right; a long name is cut in the middle
        // (both its start and its end say what it is) and is whole in the tooltip.
        char hz[24];
        rate(info.dtNs, hz, sizeof(hz));
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float sizeWidth = *size != '\0' ? ImGui::CalcTextSize(size).x : 0.0F;
        const float nameWidth =
            std::max(unitX - ImGui::GetCursorPosX() - px(8.0F) - sizeWidth, px(40.0F));
        const std::string fitted =
            fit::cut(ImGui::GetFont(), ImGui::GetFontSize(), shown, nameWidth, true);
        ImGui::TextUnformatted(fitted.c_str());
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s%s", path.c_str(), size);
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                toggled = toggled || !whole;
                opened = opened || whole;
            }
        } else if (whole && ImGui::IsItemClicked()) {
            opened = true;
        }
        // Dragged onto a scope, it is plotted there (on the plot of its unit).
        if (!whole && ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
            char payload[256];
            const int n =
                std::snprintf(payload, sizeof(payload), "%s\n%u", info.name.c_str(), component);
            ImGui::SetDragDropPayload(kSignalPayload, payload,
                                      static_cast<std::size_t>(std::max(n, 0)) + 1);
            ImGui::TextUnformatted(path.c_str());
            ImGui::EndDragDropSource();
        }
        if (*size != '\0') {
            ImGui::SameLine(0.0F, 0.0F);
            ImGui::TextColored(c::t3, "%s", size);
        }
        ImGui::PopFont();
        ImGui::SameLine(unitX);
        widgets::Mono(c::t3, "%s", info.unit == "1" ? "-" : info.unit.c_str());
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float hzWidth = ImGui::CalcTextSize(hz).x;
        ImGui::PopFont();
        ImGui::SameLine(rowEnd - hzWidth);
        widgets::Mono(c::t3, "%s", hz);
        ImGui::PopID();
        ImGui::PopID();
        if (toggled) {
            if (plotted >= 0) {
                session.remove(context.client, static_cast<std::size_t>(plotted));
            } else {
                session.add(context.client, catalog, info.name, component);
            }
        }
        if (opened) {
            if (entry.expanded && open_.contains(info.name)) {
                open_.erase(info.name);
            } else if (entry.expanded) {
                // Opened by the search: closing it clears the search's hold on it.
                search_[0] = '\0';
            } else {
                open_.insert(info.name);
            }
            return true;
        }
        return false;
    }

    std::vector<Group> groups_;
    std::uint64_t generation_{0};
    std::size_t total_{0};
    std::size_t matches_{0};
    char search_[96]{};
    std::set<std::string, std::less<>> open_;  // arrays the user opened, by signal name
    float unitColumn_{0.0F};
    float rateColumn_{0.0F};
    std::vector<std::string> units_;  // of the catalog, sorted
    std::string unit_;                // the unit filter; empty: any
    bool fastOnly_{false};
    bool dirty_{true};
};

// --- cursors ------------------------------------------------------------------------------

class CursorsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Cursors"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        ScopeSession& session = *context.scopeSession;
        if (session.traces().empty()) {
            widgets::Caption("No signals plotted.");
        } else if (!session.cursors) {
            widgets::Caption("Cursors are off.");
            if (widgets::Button("Show cursors A·B")) {
                session.showCursors(true);
            }
        } else if (ImGui::BeginTable("##cursors", 4, kTableFlags)) {
            ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_None, 2.2F);
            ImGui::TableSetupColumn("A", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("B", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("B-A", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableNextRow();
            headerCell("signal", false);
            headerCell("A", true);
            headerCell("B", true);
            headerCell("B − A", true);

            char text[32];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("t (s)");
            ImGui::TableNextColumn();
            number(session.cursorA, text, sizeof(text), "%.3f");
            rightMono(c::t1, text);
            ImGui::TableNextColumn();
            number(session.cursorB, text, sizeof(text), "%.3f");
            rightMono(c::accent, text);
            ImGui::TableNextColumn();
            number(session.cursorB - session.cursorA, text, sizeof(text), "%.3f");
            rightMono(c::t1, text);

            const std::int64_t timeA = session.cursorTimeNs(session.cursorA);
            const std::int64_t timeB = session.cursorTimeNs(session.cursorB);
            for (const ScopeTrace& trace : session.traces()) {
                const double a = trace.buffer->valueAt(timeA);
                const double b = trace.buffer->valueAt(timeB);
                ImGui::TableNextRow();
                traceCell(trace);
                ImGui::TableNextColumn();
                number(a, text, sizeof(text));
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                number(b, text, sizeof(text));
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                number(b - a, text, sizeof(text), "%+.4g");
                rightMono(c::t1, text);
            }
            ImGui::EndTable();
            widgets::Caption("units follow each signal  ·  drag A or B on any plot");
        }
        ImGui::End();
    }
};

// --- statistics ---------------------------------------------------------------------------

class StatisticsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Statistics"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const ScopeSession& session = *context.scopeSession;
        const std::vector<ScopeTrace>& traces = session.traces();
        if (traces.empty()) {
            widgets::Caption("No signals plotted.");
            ImGui::End();
            return;
        }
        // Over the raw samples of the visible window. Recomputed a few times a second: the
        // numbers would not be readable any faster.
        const double now = ImGui::GetTime();
        if (statistics_.size() != traces.size() || now - computed_ >= kStatisticsIntervalSeconds) {
            const ScopeSession::View view = session.view();
            statistics_.resize(traces.size());
            samples_ = 0;
            fill_ = 0.0F;
            dropped_ = 0;
            for (std::size_t i = 0; i < traces.size(); ++i) {
                const ScopeBuffer& buffer = *traces[i].buffer;
                statistics_[i] = buffer.statistics(view.beginNs, view.endNs + 1);
                samples_ += statistics_[i].count;
                dropped_ += buffer.droppedSamples();
                if (buffer.newestTimeNs() >= 0) {
                    const double held =
                        static_cast<double>(buffer.newestTimeNs() - buffer.oldestTimeNs()) /
                        static_cast<double>(buffer.dtNs());
                    fill_ = std::max(
                        fill_, static_cast<float>(held / static_cast<double>(buffer.capacity())));
                }
            }
            computed_ = now;
        }

        widgets::Caption("over the visible window");
        if (ImGui::BeginTable("##statistics", 5, kTableFlags)) {
            ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_None, 3.0F);
            for (const char* name : {"min", "max", "mean", "rms"}) {
                ImGui::TableSetupColumn(name, ImGuiTableColumnFlags_None, 1.0F);
            }
            ImGui::TableNextRow();
            headerCell("signal", false);
            for (const char* name : {"min", "max", "mean", "rms"}) {
                headerCell(name, true);
            }
            char text[32];
            for (std::size_t i = 0; i < traces.size(); ++i) {
                const ScopeBuffer::Statistics& s = statistics_[i];
                ImGui::TableNextRow();
                traceCell(traces[i]);
                for (const double value : {s.min, s.max, s.mean, s.rms}) {
                    ImGui::TableNextColumn();
                    number(s.count == 0 ? NAN : value, text, sizeof(text), "%.3g");
                    rightMono(c::t1, text);
                }
            }
            ImGui::EndTable();
        }

        if (widgets::SectionHeader("Stream health", nullptr)) {
            char text[48];
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(samples_));
            line("samples in window", text, c::t1);
            std::snprintf(text, sizeof(text), "%zu", session.drawnPoints);
            line("drawn after min/max decimation", text, c::t1);

            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("ring buffer");
            std::snprintf(text, sizeof(text), "%.0f %%", static_cast<double>(fill_) * 100.0);
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            const float width = ImGui::CalcTextSize(text).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(px(80.0F) + px(8.0F) + width);
            widgets::Meter(fill_, fill_ > 0.9F ? c::warn : c::ok, px(80.0F));
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Mono(c::t1, "%s", text);

            std::snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(dropped_));
            line("dropped samples", text, dropped_ == 0 ? c::ok : c::error);
            std::snprintf(text, sizeof(text), "%.2f ms / frame", session.plotMs);
            line("plot time", text, c::t1);
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    static void line(const char* label, const char* value, const ImVec4& color) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::SameLine();
        rightMono(color, value);
    }

    std::vector<ScopeBuffer::Statistics> statistics_;
    std::uint64_t samples_{0};
    std::uint64_t dropped_{0};
    float fill_{0.0F};
    double computed_{-1.0};
};

}  // namespace

void requestSignalSearch(std::string name) {
    requestedSearch() = std::move(name);
}

std::unique_ptr<Panel> createSignalsPanel() {
    return std::make_unique<SignalsPanel>();
}

std::unique_ptr<Panel> createCursorsPanel() {
    return std::make_unique<CursorsPanel>();
}

std::unique_ptr<Panel> createStatisticsPanel() {
    return std::make_unique<StatisticsPanel>();
}

}  // namespace lagrange::viewer
