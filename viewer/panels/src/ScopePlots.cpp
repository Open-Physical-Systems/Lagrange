#include "PanelFormat.hpp"
#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <string_view>

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

/// Title at the top right of a plot: the unit, then each trace with its colour. When they do not
/// fit the plot's width, the longest names are cut to one width (fit::cutAll, so that two names
/// never read the same) and the legend starts inside the plot's left edge.
void ScopePlotter::legend(const ScopeSession& session, const std::string& unit) {
    const theme::Fonts& fonts = theme::fonts();
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    const ImVec2 origin = ImPlot::GetPlotPos();
    const ImVec2 size = ImPlot::GetPlotSize();
    const float text = px(theme::kLabelSize);
    const float mono = px(theme::kMonoSize);
    const float swatch = px(30.0F);  // 12 px line, 6 px to its name, 12 px to the one before
    float x = origin.x + size.x - px(10.0F);
    const float y = origin.y + px(8.0F);
    const char* shown = unit == "1" ? "ratio" : unit.c_str();
    const float unitWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, shown).x;

    // The names of this plot, cut when they do not fit (cached: cut again only when the names or
    // the room change, so that drawing a frame allocates nothing).
    LegendNames& names = legendNames_[unit];
    bool same = true;
    std::size_t count = 0;
    for (const ScopeTrace& trace : session.traces()) {
        if (trace.unit == unit) {
            same = same && count < names.full.size() &&
                   names.full[count] == std::string_view(shortName(trace.label));
            ++count;
        }
    }
    same = same && count == names.full.size();
    const float room = std::max(size.x - px(20.0F) - unitWidth, 0.0F);
    if (!same || room != names.room) {
        names.full.clear();
        for (const ScopeTrace& trace : session.traces()) {
            if (trace.unit == unit) {
                names.full.emplace_back(shortName(trace.label));
            }
        }
        names.room = room;
        names.shown = names.full;
        std::vector<float> widths;
        float total = 0.0F;
        for (const std::string& name : names.full) {
            widths.push_back(fonts.sans->CalcTextSizeA(text, FLT_MAX, 0.0F, name.c_str()).x);
            total += swatch + widths.back();
        }
        if (total > room && !widths.empty()) {
            // The widest width every name may take so that all of them fit (water filling).
            const float budget = std::max(room - swatch * static_cast<float>(widths.size()), 0.0F);
            std::vector<float> sorted = widths;
            std::sort(sorted.begin(), sorted.end());
            float cap = 0.0F;
            float used = 0.0F;
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                const float left = static_cast<float>(sorted.size() - i);
                cap = (budget - used) / left;
                if (cap <= sorted[i]) {
                    break;
                }
                used += sorted[i];
                cap = sorted[i];
            }
            // Too narrow for even the start of each name: the part that differs from trace to
            // trace, the component after the last '.' or '/' ("x" of imu/specific_force.x).
            std::vector<std::string> tails;
            bool tailsDiffer = true;
            if (cap < px(40.0F)) {
                for (const std::string& name : names.full) {
                    const std::size_t at = name.find_last_of("./");
                    tails.push_back(at == std::string::npos ? name : name.substr(at + 1));
                }
                for (std::size_t i = 0; i < tails.size() && tailsDiffer; ++i) {
                    for (std::size_t j = i + 1; j < tails.size(); ++j) {
                        tailsDiffer = tailsDiffer && tails[i] != tails[j];
                    }
                }
            }
            if (!tails.empty() && tailsDiffer) {
                names.shown = fit::cutAll(fonts.sans, text, tails, std::max(cap, px(12.0F)));
            } else {
                names.shown = fit::cutAll(fonts.sans, text, names.full, cap);
            }
        }
    }

    float total = unitWidth;
    for (const std::string& name : names.shown) {
        total += swatch + fonts.sans->CalcTextSizeA(text, FLT_MAX, 0.0F, name.c_str()).x;
    }
    ImPlot::PushPlotClipRect();
    // A backdrop under the legend, so that it stays readable over the newest samples (the
    // traces end at the right edge, under it).
    const ImVec2 backMin(std::max(x - total - px(6.0F), origin.x + px(4.0F)), y - px(3.0F));
    const ImVec2 backMax(x + px(6.0F), y + text + px(4.0F));
    draw->AddRectFilled(backMin, backMax, u32(ImVec4(c::bg0.x, c::bg0.y, c::bg0.z, 0.82F)),
                        px(3.0F));
    // A cut legend shows the whole names when hovered.
    if (names.shown != names.full && ImGui::IsMouseHoveringRect(backMin, backMax)) {
        ImGui::BeginTooltip();
        for (const std::string& name : names.full) {
            ImGui::TextUnformatted(name.c_str());
        }
        ImGui::EndTooltip();
    }
    std::size_t n = names.shown.size();
    for (std::size_t i = session.traces().size(); i-- > 0;) {
        const ScopeTrace& trace = session.traces()[i];
        if (trace.unit != unit || n == 0) {
            continue;
        }
        const std::string& name = names.shown[--n];
        const float width = fonts.sans->CalcTextSizeA(text, FLT_MAX, 0.0F, name.c_str()).x;
        x -= width;
        draw->AddText(fonts.sans, text, ImVec2(x, y), u32(c::t2), name.c_str());
        x -= px(18.0F);
        draw->AddLine(ImVec2(x, y + text * 0.55F), ImVec2(x + px(12.0F), y + text * 0.55F),
                      u32(theme::kSeries[trace.color % ScopeSession::kColors]), px(2.0F));
        x -= px(12.0F);
    }
    if (x - unitWidth >= origin.x + px(4.0F)) {  // whole or not at all
        draw->AddText(fonts.mono, mono, ImVec2(x - unitWidth, y), u32(c::t3), shown);
    }
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
