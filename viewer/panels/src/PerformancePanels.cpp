// Performance workspace: why the simulation is or is not real time
// (docs/design/ui/11-performance/SPEC.md). Everything comes from the server's statistics and
// from what the viewer measures of itself.
#include "PanelFormat.hpp"
#include "ScheduleChart.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/ViewerSelf.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
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
        const float width = (ImGui::GetContentRegionAvail().x - gap * 5.0F) / 6.0F;
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

        // GPU per sensor frame: the modules the server marks as running on the GPU
        // (ServerStats.Module.on_gpu), each step being one frame of its sensor.
        gpuCard(stats, width, height);
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
        ImGui::SameLine(0.0F, gap);

        // Determinism: ServerStats carries no determinism check (proto v1 as frozen by RFC
        // 0007), so the card says so instead of guessing.
        card("determinism", "not reported",
             stats != nullptr ? "this server sends no determinism check" : "", c::t3, -1.0F, width,
             height);
        ImGui::End();
    }

private:
    static void gpuCard(const ServerStatistics* stats, float width, float height) {
        if (stats == nullptr) {
            card("GPU per sensor frame", "-", "", c::t3, -1.0F, width, height);
            return;
        }
        double total = 0.0;
        char detail[96] = "";
        std::size_t used = 0;
        bool any = false;
        for (const ServerStatistics::Module& module : stats->modules) {
            if (!module.onGpu) {
                continue;
            }
            any = true;
            total += module.cpuMsPerStep;
            // "lidar_top 1.4 · cam 0.7": the part of the label after the domain.
            const std::string_view label = module.module;
            const std::size_t slash = label.find('/');
            const std::string_view name =
                slash == std::string_view::npos ? label : label.substr(slash + 1);
            if (used < sizeof(detail)) {
                const int n = std::snprintf(detail + used, sizeof(detail) - used, "%s%.*s %.1f",
                                            used == 0 ? "" : " · ", static_cast<int>(name.size()),
                                            name.data(), module.cpuMsPerStep);
                used += n > 0 ? static_cast<std::size_t>(n) : 0;
            }
        }
        if (!any) {
            card("GPU per sensor frame", "not reported", "this server marks no module as GPU",
                 c::t3, -1.0F, width, height);
            return;
        }
        char value[32];
        std::snprintf(value, sizeof(value), "%.1f ms", total);
        card("GPU per sensor frame", value, detail, c::t1, -1.0F, width, height);
    }

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

// --- shared table parts of the tabs ----------------------------------------------------------

/// "212 MB", "3.8 GB": a size the way the design writes it.
void bytesText(std::uint64_t bytes, char* text, std::size_t size) {
    const double mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
    if (mb >= 1024.0) {
        std::snprintf(text, size, "%.1f GB", mb / 1024.0);
    } else {
        std::snprintf(text, size, "%.0f MB", mb);
    }
}

/// A right-aligned mono cell of a table, 8 px from the column's right edge.
void rightCell(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    widgets::AlignRight(ImGui::CalcTextSize(text).x + px(8.0F));
    ImGui::PopFont();
    widgets::Mono(color, "%s", text);
}

/// The first, left-aligned cell of a row: an optional dot and the row's name.
void nameCell(const char* name, const ImVec4* dot = nullptr) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
    if (dot != nullptr) {
        widgets::Dot(*dot);
        ImGui::SameLine(0.0F, px(6.0F));
    }
    ImGui::TextUnformatted(name);
}

/// The header row of the Modules table's style: captions on bg2, numbers' columns right.
void headerRow(std::span<const char* const> columns) {
    ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
    for (std::size_t i = 0; i < columns.size(); ++i) {
        ImGui::TableNextColumn();
        if (i == 0) {
            widgets::Caption("  %s", columns[i]);
        } else {
            ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
            widgets::AlignRight(ImGui::CalcTextSize(columns[i]).x + px(8.0F));
            ImGui::PopFont();
            widgets::Caption("%s", columns[i]);
        }
    }
}

bool beginTable(const char* id, std::span<const char* const> columns, float firstWeight) {
    if (!ImGui::BeginTable(id, static_cast<int>(columns.size()),
                           ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                               ImGuiTableFlags_NoPadOuterX)) {
        return false;
    }
    ImGui::TableSetupColumn(columns[0], ImGuiTableColumnFlags_None, firstWeight);
    for (std::size_t i = 1; i < columns.size(); ++i) {
        ImGui::TableSetupColumn(columns[i], ImGuiTableColumnFlags_None, 1.0F);
    }
    headerRow(columns);
    return true;
}

void sectionGap() {
    ImGui::Dummy(ImVec2(0.0F, px(10.0F)));
}

/// A caption 8 px in from the left, wrapped to the panel.
void note(const char* text) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
    ImGui::PushTextWrapPos(ImGui::GetContentRegionMax().x - px(8.0F));
    widgets::Caption("%s", text);
    ImGui::PopTextWrapPos();
}

/// Opens a tab of the bottom or top dock without padding, like Modules.
bool beginTab(const char* name) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    const bool open = ImGui::Begin(name);
    ImGui::PopStyleVar();
    return open;
}

// --- threads ------------------------------------------------------------------------------

/// The viewer's own threads (from /proc, once a second) and its UI thread's frame. The
/// simulation's threads are not in ServerStats, so they are not shown.
class ThreadsPanel final : public Panel {
public:
    explicit ThreadsPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Threads"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        if (!beginTab("Threads###PerformanceThreads")) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        sample();
        char text[48];
        static constexpr std::array<const char*, 4> kFrame{"viewer · UI thread", "per frame",
                                                           "share", ""};
        if (const ViewerTimings* t = context.viewerTimings;
            t != nullptr && beginTable("##frame", kFrame, 2.6F)) {
            const std::array<std::pair<const char*, double>, 4> phases{
                {{"build the panels", t->uiMs},
                 {"record the GPU commands", t->recordMs},
                 {"wait for the GPU", t->waitMs},
                 {"present", t->presentMs}}};
            // Shares of the phases' sum, so that they add up to 100 %; the frame below also
            // holds the loop's own pause and event handling.
            double frame = 1e-9;
            for (const auto& phase : phases) {
                frame += phase.second;
            }
            for (const auto& [name, ms] : phases) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                nameCell(name);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.2f ms", ms);
                rightCell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.0f %%", ms / frame * 100.0);
                rightCell(c::t2, text);
                ImGui::TableNextColumn();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
                widgets::Meter(static_cast<float>(ms / frame), theme::kSeries[0],
                               ImGui::GetContentRegionAvail().x - px(16.0F));
            }
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
            ImGui::TableNextColumn();
            nameCell("frame");
            ImGui::TableNextColumn();
            std::snprintf(text, sizeof(text), "%.2f ms", static_cast<double>(context.frameMs));
            rightCell(context.frameMs > kFrameBudgetMs ? c::warn : c::t1, text);
            ImGui::EndTable();
        }
        sectionGap();
        static constexpr std::array<const char*, 4> kThreads{"viewer · threads", "tid", "state",
                                                             "CPU (one core)"};
        if (beginTable("##threads", kThreads, 2.6F)) {
            for (const Row& row : rows_) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                nameCell(row.name.c_str());
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%d", row.tid);
                rightCell(c::t2, text);
                ImGui::TableNextColumn();
                rightCell(row.state == 'R' ? c::ok : c::t2, stateName(row.state));
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.1f %%", row.cpu * 100.0);
                rightCell(row.cpu > 0.9 ? c::warn : c::t1, text);
            }
            ImGui::EndTable();
        }
        if (rows_.empty()) {
            note("Reading the viewer's threads ...");
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        note(state_->stats ? "The simulation server does not report its threads; its modules' cost "
                             "per step is under Modules."
                           : "Not connected: only the viewer's own threads are shown.");
        ImGui::End();
    }

private:
    struct Row {
        int tid{0};
        std::string name;
        char state{'?'};
        double cpu{0.0};  // fraction of one core over the last second
    };

    static const char* stateName(char state) {
        switch (state) {
            case 'R':
                return "running";
            case 'S':
                return "sleeping";
            case 'D':
                return "disk wait";
            case 'T':
            case 't':
                return "stopped";
            default:
                return "other";
        }
    }

    /// Once a second, while the tab is in front.
    void sample() {
        const double now = ImGui::GetTime();
        if (sampledAt_ >= 0.0 && now - sampledAt_ < 1.0) {
            return;
        }
        const std::vector<ThreadSample> threads = readOwnThreads();
        const double seconds = sampledAt_ >= 0.0 ? now - sampledAt_ : 0.0;
        const double ticks = clockTicksPerSecond();
        std::vector<Row> rows;
        rows.reserve(threads.size());
        for (const ThreadSample& thread : threads) {
            Row row{thread.tid, thread.name, thread.state, 0.0};
            const auto before = std::ranges::find(previous_, thread.tid, &ThreadSample::tid);
            if (seconds > 0.0 && before != previous_.end() && thread.cpuTicks >= before->cpuTicks) {
                row.cpu = static_cast<double>(thread.cpuTicks - before->cpuTicks) / ticks / seconds;
            }
            rows.push_back(std::move(row));
        }
        std::ranges::stable_sort(rows, std::greater{}, &Row::cpu);
        rows_ = std::move(rows);
        previous_ = threads;
        sampledAt_ = now;
    }

    std::shared_ptr<PerformanceState> state_;
    std::vector<ThreadSample> previous_;
    std::vector<Row> rows_;
    double sampledAt_{-1.0};
};

// --- GPU passes ---------------------------------------------------------------------------

/// The viewer's own GPU passes (Vulkan timestamps), and the server's GPU modules when it marks
/// any (ServerStats.Module.on_gpu).
class GpuPassesPanel final : public Panel {
public:
    explicit GpuPassesPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "GPU passes"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        if (!beginTab("GPU passes###PerformanceGpuPasses")) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        char text[64];
        const ViewerTimings* t = context.viewerTimings;
        static constexpr std::array<const char*, 5> kViewer{"viewer · pass", "frames run", "mean",
                                                            "worst", "share"};
        if (t != nullptr && t->gpuTimed && beginTable("##viewerpasses", kViewer, 2.6F)) {
            // Each pass's mean is over the frames that ran it; per frame, a pass costs its mean
            // times the share of frames that ran it. The shares are of that per-frame sum.
            double perFrame = 0.0;
            for (const ViewerTimings::Pass& pass : t->passes) {
                perFrame += pass.meanMs * pass.frames / std::max(t->window, 1U);
            }
            for (const ViewerTimings::Pass& pass : t->passes) {
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                nameCell(pass.name);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%u / %u", pass.frames, t->window);
                rightCell(c::t2, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f ms", pass.meanMs);
                rightCell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f ms", pass.maxMs);
                rightCell(pass.maxMs > kFrameBudgetMs ? c::warn : c::t1, text);
                ImGui::TableNextColumn();
                const double share =
                    perFrame > 0.0 ? pass.meanMs * pass.frames / std::max(t->window, 1U) / perFrame
                                   : 0.0;
                std::snprintf(text, sizeof(text), "%.0f %%", share * 100.0);
                rightCell(c::t1, text);
            }
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
            ImGui::TableNextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
            widgets::Caption("viewer GPU per frame, over the last %u frames", t->window);
            for (int i = 0; i < 3; ++i) {
                ImGui::TableNextColumn();
            }
            ImGui::TableNextColumn();
            std::snprintf(text, sizeof(text), "%.3f ms", perFrame);
            rightCell(c::t2, text);
            ImGui::EndTable();
        } else {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            note(t == nullptr        ? "The viewer's GPU passes are not measured here."
                 : !t->gpuTimestamps ? "This GPU reports no timestamps on the viewer's queue."
                                     : "Measuring the viewer's GPU passes ...");
        }
        sectionGap();
        const ServerStatistics* stats = state_->stats.get();
        bool any = false;
        if (stats != nullptr) {
            static constexpr std::array<const char*, 4> kServer{"simulation · GPU module", "dt",
                                                                "mean per step", "worst"};
            for (const ServerStatistics::Module& module : stats->modules) {
                if (!module.onGpu) {
                    continue;
                }
                if (!any && !beginTable("##serverpasses", kServer, 2.6F)) {
                    break;
                }
                any = true;
                ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
                ImGui::TableNextColumn();
                const ImVec4 dot = widgets::domainColor(module.domain);
                nameCell(module.module.c_str(), &dot);
                ImGui::TableNextColumn();
                format::step(module.dtNs, text, sizeof(text));
                rightCell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f ms", module.cpuMsPerStep);
                rightCell(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%.3f ms", module.cpuMsPerStepMax);
                rightCell(c::t1, text);
            }
            if (any) {
                ImGui::EndTable();
            }
        }
        if (!any) {
            note(stats != nullptr
                     ? "The simulation server reports no GPU pass timings. A GPU sensor's step "
                       "(its CPU time and the GPU work it waits for) is under Modules."
                     : "Not connected: the simulation's GPU passes appear when a server is "
                       "reachable.");
        }
        ImGui::End();
    }

private:
    std::shared_ptr<PerformanceState> state_;
};

// --- memory -------------------------------------------------------------------------------

/// The server's memory as ServerStats gives it, and the viewer's own (from /proc and
/// VK_EXT_memory_budget).
class MemoryPanel final : public Panel {
public:
    explicit MemoryPanel(std::shared_ptr<PerformanceState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Memory"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Performance; }

    void draw(PanelContext& context) override {
        if (!beginTab("Memory###PerformanceMemory")) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (const double now = ImGui::GetTime(); sampledAt_ < 0.0 || now - sampledAt_ >= 1.0) {
            own_ = readOwnMemory();
            sampledAt_ = now;
        }
        const ServerStatistics* stats = state_->stats.get();
        static constexpr std::array<const char*, 5> kColumns{"memory", "process", "used", "of", ""};
        if (beginTable("##memory", kColumns, 2.2F)) {
            char used[32];
            char of[48];
            if (stats != nullptr) {
                bytesText(stats->memoryBytes, used, sizeof(used));
                row("resident", "simulation server", used, "", -1.0F, c::t1);
                if (stats->gpuMemoryBytes != 0) {
                    bytesText(stats->gpuMemoryBytes, used, sizeof(used));
                    row("GPU", "simulation server", used, "", -1.0F, c::t1);
                } else {
                    row("GPU", "simulation server", "not reported", "", -1.0F, c::t3);
                }
            } else {
                row("resident", "simulation server", "not connected", "", -1.0F, c::t3);
            }
            if (own_.known) {
                bytesText(own_.residentBytes, used, sizeof(used));
                char peak[32];
                bytesText(own_.peakBytes, peak, sizeof(peak));
                std::snprintf(of, sizeof(of), "peak %s", peak);
                row("resident", "viewer", used, of, -1.0F, c::t1);
            }
            if (const ViewerTimings* t = context.viewerTimings; t != nullptr && t->vramKnown) {
                bytesText(t->vramUsedBytes, used, sizeof(used));
                char budget[32];
                bytesText(t->vramBudgetBytes, budget, sizeof(budget));
                std::snprintf(of, sizeof(of), "budget %s", budget);
                const float fraction = static_cast<float>(
                    static_cast<double>(t->vramUsedBytes) /
                    static_cast<double>(std::max<std::uint64_t>(t->vramBudgetBytes, 1)));
                row("video (device-local)", "viewer", used, of, fraction, c::t1);
            } else {
                row("video (device-local)", "viewer", "not reported", "", -1.0F, c::t3);
            }
            ImGui::EndTable();
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        note(
            "The viewer's video memory is what this process holds on the device, as the "
            "driver reports it (VK_EXT_memory_budget).");
        ImGui::End();
    }

private:
    static void row(const char* what, const char* process, const char* used, const char* of,
                    float fraction, const ImVec4& color) {
        ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
        ImGui::TableNextColumn();
        nameCell(what);
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        widgets::AlignRight(ImGui::CalcTextSize(process).x + px(8.0F));
        ImGui::TextUnformatted(process);
        ImGui::PopStyleColor();
        ImGui::TableNextColumn();
        rightCell(color, used);
        ImGui::TableNextColumn();
        rightCell(c::t2, of);
        ImGui::TableNextColumn();
        if (fraction >= 0.0F) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
            widgets::Meter(fraction, fraction > 0.9F ? c::warn : c::ok,
                           ImGui::GetContentRegionAvail().x - px(16.0F));
        }
    }

    std::shared_ptr<PerformanceState> state_;
    ProcessMemory own_;
    double sampledAt_{-1.0};
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createPerformancePanels() {
    auto state = std::make_shared<PerformanceState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<OverviewPanel>(state));
    panels.push_back(std::make_unique<SchedulerPanel>(state));
    panels.push_back(std::make_unique<ThreadsPanel>(state));
    panels.push_back(std::make_unique<ModulesPanel>(state));
    panels.push_back(std::make_unique<GpuPassesPanel>(state));
    panels.push_back(std::make_unique<MemoryPanel>(state));
    panels.push_back(std::make_unique<RtfHistoryPanel>(state));
    panels.push_back(std::make_unique<ResourcesPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
