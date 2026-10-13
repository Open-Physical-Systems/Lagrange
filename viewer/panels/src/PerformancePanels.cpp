// Performance workspace: why the simulation is or is not real time
// (docs/design/ui/11-performance/SPEC.md). Everything comes from the server's statistics and
// from what the viewer measures of itself.
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
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::size_t kHistory = 120;        // design: the last 120 s of the real-time factor
constexpr double kBudgetMsPerSimMs = 1.0;    // real time: 1 ms of CPU per 1 ms of simulation
constexpr float kFrameBudgetMs = 16.6F;      // CLAUDE.md §13.4: 60 fps
constexpr double kRealTimeTolerance = 0.95;  // below this fraction of the target is "slow"
constexpr std::int64_t kSyncNs = 1'000'000;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// CPU milliseconds a module costs per millisecond of simulation.
double costPerSimMs(const ServerStatistics::Module& module) {
    return module.dtNs > 0 ? module.cpuMsPerStep * 1e6 / static_cast<double>(module.dtNs) : 0.0;
}

/// What the panels share: the latest statistics and what is kept over time.
class PerformanceState {
public:
    void follow(PanelContext& context) {
        const int frame = ImGui::GetFrameCount();
        if (frame == frame_) {
            return;
        }
        frame_ = frame;
        stats = context.client.statistics();
        if (stats && stats->serial != serial_) {
            serial_ = stats->serial;
            if (history.size() >= kHistory) {
                history.erase(history.begin());
            }
            history.push_back(static_cast<float>(stats->realTimeFactor));
            cpuPerSimMs = 0.0;
            for (const ServerStatistics::Module& module : stats->modules) {
                cpuPerSimMs += costPerSimMs(module);
            }
        }
        if (!stats) {
            history.clear();  // another connection: its history starts again
            serial_ = 0;
        }
        // Scene deltas per second, measured by the viewer.
        const double now = ImGui::GetTime();
        if (now - deltasAt_ >= 1.0) {
            deltasPerSecond =
                static_cast<double>(context.stats.deltas - deltas_) / (now - deltasAt_);
            deltas_ = context.stats.deltas;
            deltasAt_ = now;
        }
    }

    std::shared_ptr<const ServerStatistics> stats;
    std::vector<float> history;  // real-time factor, one value per report, oldest first
    double cpuPerSimMs{0.0};
    double deltasPerSecond{0.0};

private:
    int frame_{-1};
    std::uint64_t serial_{0};
    std::uint64_t deltas_{0};
    double deltasAt_{0.0};
};

bool emptyState(const PanelContext& context, const PerformanceState& state) {
    if (state.stats) {
        return false;
    }
    widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                               ? "Waiting for the server's statistics ..."
                               : "Not connected. Statistics appear when a simulation server "
                                 "is reachable.");
    return true;
}

void rightMono(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    widgets::AlignRight(ImGui::CalcTextSize(text).x);
    ImGui::PopFont();
    widgets::Mono(color, "%s", text);
}

// --- KPI row ------------------------------------------------------------------------------

class OverviewPanel final : public Panel {
public:
    explicit OverviewPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Overview"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(4.0F), px(4.0F)));
        const bool open =
            ImGui::Begin("Overview###PerformanceOverview", nullptr, ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        const ServerStatistics* stats = state_->stats.get();
        const float gap = px(6.0F);
        const float width = (ImGui::GetContentRegionAvail().x - gap * 4.0F) / 5.0F;
        const float height = std::max(ImGui::GetContentRegionAvail().y, px(84.0F));
        char value[48];
        char detail[96];

        // Real-time factor.
        const double target = context.status.realTimeFactorTarget;
        const bool running = context.status.runState == RunState::Running;
        if (stats != nullptr) {
            std::snprintf(value, sizeof(value), "%.2f×", stats->realTimeFactor);
            const bool slow = running && stats->realTimeFactor < target * kRealTimeTolerance;
            std::snprintf(detail, sizeof(detail), "target %.2f · %s", target,
                          !running ? "paused" : (slow ? "slower than asked" : "on target"));
            card("real-time factor", value, detail, slow ? c::warn : c::t1, -1.0F, width, height);
        } else {
            card("real-time factor", "-", "no statistics yet", c::t3, -1.0F, width, height);
        }
        ImGui::SameLine(0.0F, gap);

        // CPU per millisecond of simulation, against the real-time budget.
        if (stats != nullptr) {
            std::snprintf(value, sizeof(value), "%.2f ms", state_->cpuPerSimMs);
            std::snprintf(detail, sizeof(detail), "budget %.2f ms%s", kBudgetMsPerSimMs,
                          stats->estimated ? " · estimated" : "");
            card("CPU per 1 ms of sim", value, detail,
                 state_->cpuPerSimMs > kBudgetMsPerSimMs ? c::warn : c::t1,
                 static_cast<float>(state_->cpuPerSimMs / kBudgetMsPerSimMs), width, height);
        } else {
            card("CPU per 1 ms of sim", "-", "", c::t3, -1.0F, width, height);
        }
        ImGui::SameLine(0.0F, gap);

        // The server's own loop.
        if (stats != nullptr) {
            std::snprintf(value, sizeof(value), "%.2f ms", stats->tickMs);
            std::snprintf(detail, sizeof(detail), "worst %.2f ms · %u %s", stats->tickMsMax,
                          stats->viewers, stats->viewers == 1 ? "viewer" : "viewers");
            card("server tick", value, detail, c::t1, -1.0F, width, height);
        } else {
            card("server tick", "-", "", c::t3, -1.0F, width, height);
        }
        ImGui::SameLine(0.0F, gap);

        // The viewer's frame, a separate process.
        const float fps = context.frameMs > 0.0F ? 1000.0F / context.frameMs : 0.0F;
        std::snprintf(value, sizeof(value), "%.1f ms", static_cast<double>(context.frameMs));
        std::snprintf(detail, sizeof(detail), "%.0f fps · budget %.1f", static_cast<double>(fps),
                      static_cast<double>(kFrameBudgetMs));
        card("viewer frame", value, detail, context.frameMs > kFrameBudgetMs ? c::warn : c::t1,
             context.frameMs / kFrameBudgetMs, width, height);
        ImGui::SameLine(0.0F, gap);

        // The protocol.
        std::snprintf(value, sizeof(value), "%.0f Δ/s", state_->deltasPerSecond);
        std::snprintf(detail, sizeof(detail), "%.1f kB/s in · %llu dropped",
                      stats != nullptr ? stats->bytesPerSecondOut / 1000.0 : 0.0,
                      static_cast<unsigned long long>(context.status.droppedDeltas));
        card("protocol", value, detail, context.status.droppedDeltas != 0 ? c::warn : c::t1, -1.0F,
             width, height);
        ImGui::End();
    }

private:
    /// A KPI card: caption, a large mono value, a detail line, and a meter when `fraction`
    /// is not negative.
    static void card(const char* label, const char* value, const char* detail, const ImVec4& color,
                     float fraction, float width, float height) {
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const ImVec2 max(min.x + width, min.y + height);
        ImGui::Dummy(ImVec2(width, height));
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(min, max, u32(c::bg1), px(3.0F));
        draw->AddRect(min, max, u32(c::line), px(3.0F));
        const theme::Fonts& fonts = theme::fonts();
        const float pad = px(10.0F);
        draw->PushClipRect(min, max, true);
        draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(min.x + pad, min.y + px(8.0F)),
                      u32(c::t3), label);
        draw->AddText(fonts.mono, px(theme::kKpiSize), ImVec2(min.x + pad, min.y + px(26.0F)),
                      u32(color), value);
        draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(min.x + pad, min.y + px(54.0F)),
                      u32(c::t3), detail);
        draw->PopClipRect();
        if (fraction >= 0.0F) {
            const ImVec2 a(min.x + pad, max.y - px(12.0F));
            const ImVec2 b(max.x - pad, max.y - px(6.0F));
            draw->AddRectFilled(a, b, u32(c::bg4), px(3.0F));
            const float filled = (b.x - a.x) * std::clamp(fraction, 0.0F, 1.0F);
            draw->AddRectFilled(
                a, ImVec2(a.x + std::max(filled, px(3.0F)), b.y),
                u32(fraction > 1.0F ? c::error : (fraction > 0.85F ? c::warn : c::ok)), px(3.0F));
        }
    }

    std::shared_ptr<PerformanceState> state_;
};

// --- scheduler ----------------------------------------------------------------------------

class SchedulerPanel final : public Panel {
public:
    explicit SchedulerPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Scheduler"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin("Scheduler · last 10 ms###Scheduler")) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (emptyState(context, *state_)) {
            ImGui::End();
            return;
        }
        widgets::Caption(
            "sync barrier every 1 ms  ·  the steps each module takes in 10 ms of "
            "simulation");
        rows_.clear();
        for (const ServerStatistics::Module& module : state_->stats->modules) {
            rows_.push_back({module.module.c_str(), module.domain, module.dtNs, module.onGpu});
        }
        if (rows_.empty()) {
            widgets::Caption("No module is running.");
        } else {
            drawScheduleChart(rows_, kSyncNs);
        }
        ImGui::End();
    }

private:
    std::shared_ptr<PerformanceState> state_;
    std::vector<ScheduleRow> rows_;
};

// --- modules ------------------------------------------------------------------------------

class ModulesPanel final : public Panel {
public:
    explicit ModulesPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Modules"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Modules###PerformanceModules");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (!state_->stats) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            emptyState(context, *state_);
            ImGui::End();
            return;
        }
        const ServerStatistics& stats = *state_->stats;
        if (ImGui::BeginTable("##modules", 6,
                              ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_NoPadOuterX)) {
            static constexpr std::array<const char*, 6> kColumns{"module", "dt",    "steps / s",
                                                                 "mean",   "worst", "budget share"};
            ImGui::TableSetupColumn(kColumns[0], ImGuiTableColumnFlags_None, 2.6F);
            for (std::size_t i = 1; i < kColumns.size(); ++i) {
                ImGui::TableSetupColumn(kColumns.at(i), ImGuiTableColumnFlags_None, 1.0F);
            }
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
            for (std::size_t i = 0; i < kColumns.size(); ++i) {
                ImGui::TableNextColumn();
                if (i == 0) {
                    widgets::Caption("  %s", kColumns.at(i));
                } else {
                    ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
                    widgets::AlignRight(ImGui::CalcTextSize(kColumns.at(i)).x + px(8.0F));
                    ImGui::PopFont();
                    widgets::Caption("%s", kColumns.at(i));
                }
            }
            char text[48];
            for (const ServerStatistics::Module& module : stats.modules) {
                const double perSimMs = costPerSimMs(module);
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
                widgets::Dot(widgets::domainColor(module.domain));
                ImGui::SameLine(0.0F, px(6.0F));
                ImGui::TextUnformatted(module.module.c_str());
                ImGui::TableNextColumn();
                format::step(module.dtNs, text, sizeof(text));
                cell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.0f", module.stepsPerSecond);
                cell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text),
                              module.cpuMsPerStep < 0.01 ? "%.4f ms" : "%.3f ms",
                              module.cpuMsPerStep);
                cell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text),
                              module.cpuMsPerStepMax < 0.01 ? "%.4f ms" : "%.3f ms",
                              module.cpuMsPerStepMax);
                // Warn when the worst step alone would use the whole step's time.
                const bool late = module.dtNs > 0 &&
                                  module.cpuMsPerStepMax * 1e6 > static_cast<double>(module.dtNs);
                cell(late ? c::warn : c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.0f %%", perSimMs / kBudgetMsPerSimMs * 100.0);
                cell(c::t1, text);
            }
            // The shares add up to the CPU figure of the overview: say so with the sum.
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
            ImGui::TableNextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
            widgets::Caption("all modules%s", stats.estimated ? "  ·  estimated costs" : "");
            for (int i = 0; i < 4; ++i) {
                ImGui::TableNextColumn();
            }
            ImGui::TableNextColumn();
            std::snprintf(text, sizeof(text), "%.2f ms = %.0f %%", state_->cpuPerSimMs,
                          state_->cpuPerSimMs / kBudgetMsPerSimMs * 100.0);
            cell(state_->cpuPerSimMs > kBudgetMsPerSimMs ? c::warn : c::t2, text);
            ImGui::EndTable();
        }
        ImGui::End();
    }

private:
    static void cell(const ImVec4& color, const char* text) {
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        widgets::AlignRight(ImGui::CalcTextSize(text).x + px(8.0F));
        ImGui::PopFont();
        widgets::Mono(color, "%s", text);
    }

    std::shared_ptr<PerformanceState> state_;
};

// --- real-time factor over time -----------------------------------------------------------

class RtfHistoryPanel final : public Panel {
public:
    explicit RtfHistoryPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "RTF history"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

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
        const std::vector<float>& history = state_->history;
        const double target = context.status.realTimeFactorTarget;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::Dummy(size);
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const float top = origin.y + px(6.0F);
        const float bottom = origin.y + size.y - px(22.0F);
        // The vertical range holds zero, the target and whatever was measured.
        float highest = static_cast<float>(target) * 1.2F;
        for (const float value : history) {
            highest = std::max(highest, value * 1.1F);
        }
        highest = std::max(highest, 0.1F);
        const auto toY = [&](float value) { return bottom - value / highest * (bottom - top); };
        // Dashed line at the target.
        const float targetY = toY(static_cast<float>(target));
        for (float x = origin.x; x < origin.x + size.x; x += px(7.0F)) {
            draw->AddLine(ImVec2(x, targetY),
                          ImVec2(std::min(x + px(3.0F), origin.x + size.x), targetY),
                          u32(c::lineStrong));
        }
        // One point per report, the newest at the right edge.
        const float step = size.x / static_cast<float>(kHistory - 1);
        for (std::size_t i = 1; i < history.size(); ++i) {
            const float x1 = origin.x + size.x - step * static_cast<float>(history.size() - 1 - i);
            draw->AddLine(ImVec2(x1 - step, toY(history[i - 1])), ImVec2(x1, toY(history[i])),
                          u32(theme::kSeries[0]), px(1.3F));
        }
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        char text[64];
        std::snprintf(text, sizeof(text), "%.2f target", target);
        draw->AddText(fonts.sans, small, ImVec2(origin.x, bottom + px(6.0F)), u32(c::t3), text);
        if (!history.empty()) {
            const float lowest = *std::ranges::min_element(history);
            std::snprintf(text, sizeof(text), "lowest %.2f in the last %zu s",
                          static_cast<double>(lowest), history.size());
            const float width = fonts.sans->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
            draw->AddText(
                fonts.sans, small, ImVec2(origin.x + size.x - width, bottom + px(6.0F)),
                u32(lowest < static_cast<float>(target * kRealTimeTolerance) ? c::warn : c::t3),
                text);
        }
        ImGui::End();
    }

private:
    std::shared_ptr<PerformanceState> state_;
};

// --- resources and suggestions ------------------------------------------------------------

class ResourcesPanel final : public Panel {
public:
    explicit ResourcesPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Resources"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

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
        const ServerStatistics& stats = *state_->stats;
        char text[64];
        std::snprintf(text, sizeof(text), "%.0f MB",
                      static_cast<double>(stats.memoryBytes) / (1024.0 * 1024.0));
        row("server memory", text, c::t1);
        if (stats.gpuMemoryBytes != 0) {
            std::snprintf(text, sizeof(text), "%.0f MB",
                          static_cast<double>(stats.gpuMemoryBytes) / (1024.0 * 1024.0));
            row("server GPU memory", text, c::t1);
        } else {
            row("server GPU memory", "not used", c::t3);
        }
        std::snprintf(text, sizeof(text), "%.1f kB/s", stats.bytesPerSecondOut / 1000.0);
        row("sent to this viewer", text, c::t1);
        std::snprintf(text, sizeof(text), "%.2f ms mean · %.2f ms worst", stats.tickMs,
                      stats.tickMsMax);
        row("server tick", text, c::t1);
        std::snprintf(text, sizeof(text), "%llu",
                      static_cast<unsigned long long>(context.stats.droppedSamples));
        row("samples lost by scopes", text, context.stats.droppedSamples == 0 ? c::ok : c::error);

        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        widgets::Strong("Suggestions");
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        bool any = false;
        const double target = context.status.realTimeFactorTarget;
        // Rules, each from numbers shown on this screen.
        if (context.status.runState == RunState::Running &&
            stats.realTimeFactor < target * kRealTimeTolerance) {
            std::snprintf(text, sizeof(text), "running at %.2f of the %.2f asked for",
                          stats.realTimeFactor, target);
            suggestion("RTF", widgets::PillKind::Warn, text);
            any = true;
        }
        if (state_->cpuPerSimMs > kBudgetMsPerSimMs && !stats.modules.empty()) {
            const auto costliest = std::ranges::max_element(
                stats.modules, {},
                [](const ServerStatistics::Module& module) { return costPerSimMs(module); });
            char advice[160];
            std::snprintf(advice, sizeof(advice),
                          "%s takes most of the budget; a larger step or a cheaper fidelity "
                          "would bring it under 1 ms per ms.",
                          costliest->module.c_str());
            suggestion("CPU", widgets::PillKind::Warn, advice);
            any = true;
        }
        if (context.frameMs > kFrameBudgetMs) {
            suggestion("viewer", widgets::PillKind::Warn,
                       "The viewer's frame is over 16.6 ms; close scopes or shorten their window.");
            any = true;
        }
        if (stats.estimated) {
            suggestion("note", widgets::PillKind::Info,
                       "Module costs are the profile's estimates: this server replays a "
                       "recording and does not step the modules.");
            any = true;
        }
        if (!any) {
            widgets::Caption("Nothing to suggest: the run is within its budgets.");
        }
        ImGui::End();
    }

private:
    static void row(const char* label, const char* value, const ImVec4& color) {
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        ImGui::TextUnformatted(label);
        ImGui::PopStyleColor();
        ImGui::SameLine();
        rightMono(color, value);
    }

    static void suggestion(const char* tag, widgets::PillKind kind, const char* sentence) {
        widgets::Pill(tag, kind);
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::PushTextWrapPos(0.0F);
        ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
        ImGui::TextUnformatted(sentence);
        ImGui::PopFont();
        ImGui::PopTextWrapPos();
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
    }

    std::shared_ptr<PerformanceState> state_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createPerformancePanels() {
    auto state = std::make_shared<PerformanceState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<OverviewPanel>(state));
    panels.push_back(std::make_unique<SchedulerPanel>(state));
    panels.push_back(std::make_unique<ModulesPanel>(state));
    panels.push_back(std::make_unique<RtfHistoryPanel>(state));
    panels.push_back(std::make_unique<ResourcesPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
