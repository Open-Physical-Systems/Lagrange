#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstdio>

#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::size_t kMaxColumns = 4096;  // more columns than any plot is pixels wide

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

const char* shortName(const std::string& label) {
    return shortSignalName(label);
}

}  // namespace

const char* shortSignalName(const std::string& label) {
    const std::size_t last = label.rfind('/');
    if (last == std::string::npos || last == 0) {
        return label.c_str();
    }
    const std::size_t before = label.rfind('/', last - 1);
    return label.c_str() + (before == std::string::npos ? 0 : before + 1);
}

void ScopePlotter::draw(ScopeSession& session) {
    const auto start = std::chrono::steady_clock::now();
    drawn_ = 0;
    plots(session);
    session.drawnPoints = drawn_;
    session.plotMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void ScopePlotter::plots(ScopeSession& session) {
    const ScopeSession::View view = session.view();
    const double span = view.seconds();
    const std::vector<std::string>& units = session.units();

    const float available = ImGui::GetContentRegionAvail().y;
    const float spacing = ImGui::GetStyle().ItemSpacing.y;
    const auto count = static_cast<float>(units.size());
    const float height = std::max((available - spacing * (count - 1.0F)) / count, px(70.0F));
    const ScopeSession::Trigger& trigger = session.trigger();

    for (std::size_t u = 0; u < units.size(); ++u) {
        ImGui::PushID(static_cast<int>(u));
        const bool last = u + 1 == units.size();
        if (ImPlot::BeginPlot("##plot", ImVec2(-1.0F, height), ImPlotFlags_CanvasOnly)) {
            // Time runs from -window to 0 at the right edge, the same on every plot.
            ImPlot::SetupAxis(ImAxis_X1, nullptr,
                              ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight |
                                  (last ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoTickLabels));
            ImPlot::SetupAxis(
                ImAxis_Y1, nullptr,
                ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight);
            ImPlot::SetupAxisLimits(ImAxis_X1, -span, 0.0, ImPlotCond_Always);
            if (last) {
                ImPlot::SetupAxisFormat(ImAxis_X1, "%g s");
            }
            ImPlot::SetupFinish();

            const auto columns = std::clamp(static_cast<std::size_t>(ImPlot::GetPlotSize().x),
                                            std::size_t{2}, kMaxColumns);
            for (const ScopeTrace& trace : session.traces()) {
                if (trace.unit == units[u]) {
                    drawTrace(trace, view, columns);
                }
            }
            if (trigger.trace >= 0 &&
                session.traces()[static_cast<std::size_t>(trigger.trace)].unit == units[u] &&
                (trigger.armed || trigger.firedNs >= 0)) {
                triggerLine(trigger.level, units[u].c_str(), span);
            }
            if (session.cursors) {
                // Draggable on any plot; the tags are on the first one only.
                ImPlot::DragLineX(0, &session.cursorA, c::t1, 1.0F, ImPlotDragToolFlags_NoFit);
                ImPlot::DragLineX(1, &session.cursorB, c::accent, 1.0F, ImPlotDragToolFlags_NoFit);
                session.cursorA = std::clamp(session.cursorA, -span, 0.0);
                session.cursorB = std::clamp(session.cursorB, -span, 0.0);
                if (u == 0) {
                    ImPlot::TagX(session.cursorA, c::t1, "A");
                    ImPlot::TagX(session.cursorB, c::accent, "B");
                }
            }
            legend(session, units[u]);
            ImPlot::EndPlot();
        }
        ImGui::PopID();
    }
}

/// Title at the top right of a plot: the unit, then each trace with its colour.
void ScopePlotter::legend(const ScopeSession& session, const std::string& unit) {
    const theme::Fonts& fonts = theme::fonts();
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    const ImVec2 origin = ImPlot::GetPlotPos();
    const ImVec2 size = ImPlot::GetPlotSize();
    const float text = px(theme::kLabelSize);
    float x = origin.x + size.x - px(10.0F);
    const float y = origin.y + px(8.0F);
    ImPlot::PushPlotClipRect();
    for (std::size_t i = session.traces().size(); i-- > 0;) {
        const ScopeTrace& trace = session.traces()[i];
        if (trace.unit != unit) {
            continue;
        }
        const char* name = shortName(trace.label);
        const float width = fonts.sans->CalcTextSizeA(text, FLT_MAX, 0.0F, name).x;
        x -= width;
        draw->AddText(fonts.sans, text, ImVec2(x, y), u32(c::t2), name);
        x -= px(18.0F);
        draw->AddLine(ImVec2(x, y + text * 0.55F), ImVec2(x + px(12.0F), y + text * 0.55F),
                      u32(theme::kSeries[trace.color % ScopeSession::kColors]), px(2.0F));
        x -= px(12.0F);
    }
    const char* shown = unit == "1" ? "ratio" : unit.c_str();
    const float mono = px(theme::kMonoSize);
    const float width = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, shown).x;
    draw->AddText(fonts.mono, mono, ImVec2(x - width, y), u32(c::t3), shown);
    ImPlot::PopPlotClipRect();
}

/// The trigger level: a dotted line in the warning colour, with its value.
void ScopePlotter::triggerLine(double level, const char* unit, double span) {
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    const ImVec2 from = ImPlot::PlotToPixels(-span, level);
    const ImVec2 to = ImPlot::PlotToPixels(0.0, level);
    ImPlot::PushPlotClipRect();
    for (float x = from.x; x < to.x; x += px(6.0F)) {
        draw->AddLine(ImVec2(x, from.y), ImVec2(std::min(x + px(2.0F), to.x), from.y),
                      u32(c::warn));
    }
    char text[48];
    std::snprintf(text, sizeof(text), "trigger %g %s", level, unit);
    draw->AddText(theme::fonts().mono, px(theme::kCaptionSize),
                  ImVec2(from.x + px(6.0F), from.y - px(15.0F)), u32(c::warn), text);
    ImPlot::PopPlotClipRect();
}

void ScopePlotter::drawTrace(const ScopeTrace& trace, ScopeSession::View view,
                             std::size_t columns) {
    if (view.endNs <= view.beginNs) {
        return;
    }
    const ImVec4 color = theme::kSeries[trace.color % ScopeSession::kColors];
    ImPlotSpec line;
    line.LineColor = color;
    line.LineWeight = theme::kSeriesLineWeight;
    const double samplesPerColumn = static_cast<double>(view.endNs - view.beginNs) /
                                    static_cast<double>(trace.buffer->dtNs()) /
                                    static_cast<double>(columns);
    // Scratch arrays are members: they grow to the plot width once and are then reused.
    xs_.resize(kMaxColumns);
    low_.resize(kMaxColumns);
    high_.resize(kMaxColumns);
    const double end = static_cast<double>(view.endNs) * 1e-9;

    if (samplesPerColumn <= 1.0) {
        // Zoomed in: fewer samples than pixels. Draw the samples themselves.
        const std::size_t n = trace.buffer->readRaw(view.beginNs, view.endNs + 1, xs_, low_);
        for (std::size_t i = 0; i < n; ++i) {
            xs_[i] -= end;
        }
        if (n > 0) {
            ImPlot::PlotLine(trace.label.c_str(), xs_.data(), low_.data(), static_cast<int>(n),
                             line);
            drawn_ += n;
        }
        return;
    }

    // Zoomed out: one min/max pair per pixel column, so that no peak is lost.
    columns_.resize(columns);
    trace.buffer->decimate(view.beginNs, view.endNs, columns_);
    const double step = view.seconds() / static_cast<double>(columns);
    std::size_t n = 0;
    const auto flush = [&] {
        if (n >= 2) {
            ImPlotSpec fill;
            fill.FillColor = color;
            fill.FillAlpha = 0.45F;
            ImPlot::PlotShaded(trace.label.c_str(), xs_.data(), low_.data(), high_.data(),
                               static_cast<int>(n), fill);
            ImPlot::PlotLine(trace.label.c_str(), xs_.data(), low_.data(), static_cast<int>(n),
                             line);
            ImPlot::PlotLine(trace.label.c_str(), xs_.data(), high_.data(), static_cast<int>(n),
                             line);
            drawn_ += n * 2;
        }
        n = 0;
    };
    for (std::size_t col = 0; col < columns; ++col) {
        const ScopeBuffer::Column& column = columns_[col];
        if (column.min != column.min) {
            flush();  // a gap: end this piece of the trace
            continue;
        }
        xs_[n] = -view.seconds() + (static_cast<double>(col) + 0.5) * step;
        low_[n] = static_cast<double>(column.min);
        high_[n] = static_cast<double>(column.max);
        ++n;
    }
    flush();
}

}  // namespace lagrange::viewer
