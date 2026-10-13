#include "lagrange/viewer/Panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

constexpr double kWindowsSeconds[] = {0.02, 0.1, 0.5, 2.0, 5.0, 10.0, 30.0};
constexpr const char* kWindowLabels[] = {"20 ms", "100 ms", "500 ms", "2 s", "5 s", "10 s", "30 s"};
constexpr int kDefaultWindow = 2;
constexpr std::size_t kMaxColumns = 4096;  // more columns than any plot is pixels wide

/// Waveform scope. Signals with the same unit share a plot; different units are stacked, so
/// amperes and volts never share an axis.
class ScopePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Scope"; }

    void draw(PanelContext& context) override {
        // Signals requested on the command line; they need the catalog first.
        if (context.initialScopeSignals != nullptr && !context.initialScopeSignals->empty() &&
            context.catalog) {
            for (const std::string& name : *context.initialScopeSignals) {
                addTrace(context, name, 0);
            }
            context.initialScopeSignals->clear();
        }
        if (ImGui::Begin(title())) {
            drawToolbar(context);
            if (traces_.empty()) {
                ImGui::TextDisabled(context.catalog ? "Add a signal with the button above"
                                                    : "No signal list from the server yet");
            } else {
                drawPlots();
            }
        }
        ImGui::End();
    }

private:
    struct Trace {
        std::string name;
        std::uint32_t component;
        std::string label;  // legend text, unique per trace
        std::string unit;
        std::shared_ptr<ScopeBuffer> buffer;
    };

    void addTrace(PanelContext& context, const std::string& name, std::uint32_t component) {
        for (const Trace& existing : traces_) {
            if (existing.name == name && existing.component == component) {
                return;
            }
        }
        const SignalInfo* info = context.catalog ? context.catalog->find(name) : nullptr;
        std::shared_ptr<ScopeBuffer> buffer = context.client.subscribe(name, component);
        if (info == nullptr || !buffer) {
            return;
        }
        Trace trace{name, component, name, info->unit.empty() ? "1" : info->unit,
                    std::move(buffer)};
        if (info->components > 1) {
            trace.label += component < info->componentNames.size()
                               ? "." + info->componentNames[component]
                               : "[" + std::to_string(component) + "]";
        }
        traces_.push_back(std::move(trace));
        rebuildUnits();
    }

    void removeTrace(PanelContext& context, std::size_t index) {
        context.client.unsubscribe(traces_[index].buffer);
        traces_.erase(traces_.begin() + static_cast<std::ptrdiff_t>(index));
        rebuildUnits();
    }

    void rebuildUnits() {
        units_.clear();
        for (const Trace& trace : traces_) {
            if (std::find(units_.begin(), units_.end(), trace.unit) == units_.end()) {
                units_.push_back(trace.unit);
            }
        }
    }

    void drawToolbar(PanelContext& context) {
        ImGui::BeginDisabled(!context.catalog);
        if (ImGui::Button("+ Signal")) {
            ImGui::OpenPopup("add-signal");
        }
        ImGui::EndDisabled();
        if (ImGui::BeginPopup("add-signal")) {
            ImGui::SetNextItemWidth(280.0F * ImGui::GetStyle().FontScaleDpi);
            ImGui::InputTextWithHint("##filter", "filter", filter_, sizeof(filter_));
            ImGui::BeginChild("signals", ImVec2(420.0F * ImGui::GetStyle().FontScaleDpi,
                                                300.0F * ImGui::GetStyle().FontScaleDpi));
            if (context.catalog) {
                for (const SignalInfo& info : context.catalog->signals) {
                    if (filter_[0] != '\0' && info.name.find(filter_) == std::string::npos) {
                        continue;
                    }
                    for (std::uint32_t c = 0; c < info.components; ++c) {
                        ImGui::PushID(static_cast<int>(info.id * 64 + c));
                        char text[256];
                        if (info.components > 1) {
                            std::snprintf(
                                text, sizeof(text), "%s.%s  [%s]  %.4g kHz", info.name.c_str(),
                                c < info.componentNames.size() ? info.componentNames[c].c_str()
                                                               : "?",
                                info.unit.c_str(), 1e6 / static_cast<double>(info.dtNs));
                        } else {
                            std::snprintf(text, sizeof(text), "%s  [%s]  %.4g kHz",
                                          info.name.c_str(), info.unit.c_str(),
                                          1e6 / static_cast<double>(info.dtNs));
                        }
                        if (ImGui::Selectable(text)) {
                            addTrace(context, info.name, c);
                        }
                        ImGui::PopID();
                    }
                }
            }
            ImGui::EndChild();
            ImGui::EndPopup();
        }

        ImGui::SameLine();
        ImGui::Checkbox("Freeze", &frozen_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0F * ImGui::GetStyle().FontScaleDpi);
        ImGui::Combo("##window", &window_, kWindowLabels, IM_ARRAYSIZE(kWindowLabels));

        std::uint64_t dropped = 0;
        for (const Trace& trace : traces_) {
            dropped += trace.buffer->droppedSamples();
        }
        ImGui::SameLine();
        if (dropped == 0) {
            ImGui::TextDisabled("no dropped samples");
        } else {
            ImGui::TextColored(ImVec4(1.0F, 0.55F, 0.35F, 1.0F), "%llu dropped samples",
                               static_cast<unsigned long long>(dropped));
        }

        // Legend row: latest value of each trace, and a button to remove it.
        for (std::size_t i = 0; i < traces_.size(); ++i) {
            const Trace& trace = traces_[i];
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::SmallButton("x")) {
                removeTrace(context, i);
                ImGui::PopID();
                break;  // indices changed; the rest is drawn next frame
            }
            ImGui::SameLine();
            const double value = trace.buffer->latest();
            ImGui::TextColored(ImPlot::GetColormapColor(static_cast<int>(i)), "%s",
                               trace.label.c_str());
            ImGui::SameLine();
            if (value == value) {
                ImGui::Text("%.4g %s", value, trace.unit.c_str());
            } else {
                ImGui::TextDisabled("no data");
            }
            if (i + 1 < traces_.size()) {
                ImGui::SameLine(0.0F, 18.0F * ImGui::GetStyle().FontScaleDpi);
            }
            ImGui::PopID();
        }
    }

    void drawPlots() {
        // The right edge follows the newest sample of any trace, so that a paused or seeking
        // replay shows what is there, not an empty window.
        if (!frozen_) {
            std::int64_t newest = -1;
            for (const Trace& trace : traces_) {
                newest = std::max(newest, trace.buffer->newestTimeNs());
            }
            if (newest >= 0) {
                rightEdgeNs_ = newest;
            }
        }
        const double windowSeconds = kWindowsSeconds[window_];
        const double right = static_cast<double>(rightEdgeNs_) * 1e-9;
        const double left = right - windowSeconds;

        const float available = ImGui::GetContentRegionAvail().y;
        const float spacing = ImGui::GetStyle().ItemSpacing.y;
        const auto count = static_cast<float>(units_.size());
        const float height = std::max((available - spacing * (count - 1.0F)) / count,
                                      60.0F * ImGui::GetStyle().FontScaleDpi);

        for (std::size_t u = 0; u < units_.size(); ++u) {
            ImGui::PushID(static_cast<int>(u));
            if (ImPlot::BeginPlot("##plot", ImVec2(-1.0F, height),
                                  ImPlotFlags_NoTitle | ImPlotFlags_NoLegend)) {
                const bool lastPlot = u + 1 == units_.size();
                ImPlot::SetupAxis(ImAxis_X1, lastPlot ? "time [s]" : nullptr,
                                  lastPlot ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoTickLabels);
                ImPlot::SetupAxis(ImAxis_Y1, units_[u].c_str(), ImPlotAxisFlags_AutoFit);
                if (!frozen_) {
                    ImPlot::SetupAxisLimits(ImAxis_X1, left, right, ImPlotCond_Always);
                }
                ImPlot::SetupFinish();

                // What is actually visible (the user may have panned while frozen).
                const ImPlotRect limits = ImPlot::GetPlotLimits();
                const auto beginNs = static_cast<std::int64_t>(limits.X.Min * 1e9);
                const auto endNs = static_cast<std::int64_t>(limits.X.Max * 1e9);
                const auto columns = std::clamp(static_cast<std::size_t>(ImPlot::GetPlotSize().x),
                                                std::size_t{2}, kMaxColumns);
                for (std::size_t i = 0; i < traces_.size(); ++i) {
                    if (traces_[i].unit == units_[u]) {
                        drawTrace(traces_[i], static_cast<int>(i), beginNs, endNs, columns);
                    }
                }
                ImPlot::EndPlot();
            }
            ImGui::PopID();
        }
    }

    void drawTrace(const Trace& trace, int colorIndex, std::int64_t beginNs, std::int64_t endNs,
                   std::size_t columns) {
        if (endNs <= beginNs) {
            return;
        }
        const ImVec4 color = ImPlot::GetColormapColor(colorIndex);
        ImPlotSpec line;
        line.LineColor = color;
        line.LineWeight = 1.0F;
        const double samplesPerColumn = static_cast<double>(endNs - beginNs) /
                                        static_cast<double>(trace.buffer->dtNs()) /
                                        static_cast<double>(columns);
        // Scratch arrays are members: they grow to the plot width once and are then reused.
        xs_.resize(kMaxColumns);
        low_.resize(kMaxColumns);
        high_.resize(kMaxColumns);

        if (samplesPerColumn <= 1.0) {
            // Zoomed in: fewer samples than pixels. Draw the samples themselves.
            const std::size_t n = trace.buffer->readRaw(beginNs, endNs, xs_, low_);
            if (n > 0) {
                ImPlot::PlotLine(trace.label.c_str(), xs_.data(), low_.data(), static_cast<int>(n),
                                 line);
            }
            return;
        }

        // Zoomed out: one min/max pair per pixel column, so that no peak is lost.
        columns_.resize(columns);
        trace.buffer->decimate(beginNs, endNs, columns_);
        const double step =
            static_cast<double>(endNs - beginNs) * 1e-9 / static_cast<double>(columns);
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
            }
            n = 0;
        };
        for (std::size_t c = 0; c < columns; ++c) {
            const ScopeBuffer::Column& column = columns_[c];
            if (column.min != column.min) {
                flush();  // a gap: end this piece of the trace
                continue;
            }
            xs_[n] = static_cast<double>(beginNs) * 1e-9 + (static_cast<double>(c) + 0.5) * step;
            low_[n] = static_cast<double>(column.min);
            high_[n] = static_cast<double>(column.max);
            ++n;
        }
        flush();
    }

    std::vector<Trace> traces_;
    std::vector<std::string> units_;
    std::vector<ScopeBuffer::Column> columns_;
    std::vector<double> xs_;
    std::vector<double> low_;
    std::vector<double> high_;
    std::int64_t rightEdgeNs_{0};
    int window_{kDefaultWindow};
    bool frozen_{false};
    char filter_[96] = {};
};

}  // namespace

std::unique_ptr<Panel> createScopePanel() {
    return std::make_unique<ScopePanel>();
}

}  // namespace lagrange::viewer
