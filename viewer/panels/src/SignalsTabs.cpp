// The Signals workspace's second tabs (docs/design/ui/05-signals, reference.html): "Saved sets"
// beside the signal list, "Measure" beside Cursors and "FFT" beside Statistics.
//
// Measure: what each plotted trace does between cursors A and B (its raw samples, not the
// decimated points). FFT: the amplitude spectrum of one trace over the visible window. Saved
// sets: named lists of plotted signals, kept in the viewer's config directory.
#include "SignalsCommon.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/SignalAnalysis.hpp"
#include "lagrange/viewer/SignalSets.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using signals::headerCell;
using signals::kTableFlags;
using signals::number;
using signals::rightMono;
using signals::traceCell;
using widgets::px;

constexpr double kMeasureIntervalSeconds = 0.2;   // while the cursors stand still: 5 Hz
constexpr double kSpectrumIntervalSeconds = 0.5;  // a live spectrum is recomputed at 2 Hz
constexpr std::size_t kSpectrumSamples = 65536;   // 6.6 s at 10 kHz; the newest are used
constexpr double kDbFloor = -160.0;               // what 0 shows as in dB
// Second tabs: appearing must not take the focus, or ImGui brings them in front of the first.
constexpr ImGuiWindowFlags kTabFlags = ImGuiWindowFlags_NoFocusOnAppearing;

/// What the tabs computed last, for the headless report (tests read it).
struct Report {
    struct Measured {
        std::string label;
        Measurement m;
    };
    std::vector<Measured> measured;
    std::string spectrumTrace;
    Spectrum spectrum;
    std::size_t savedSets{0};
    std::size_t plottedFromSet{0};
};

Report& report() {
    static Report r;
    return r;
}

const char* unitText(const std::string& unit) {
    return unit == "1" ? "" : unit.c_str();
}

void readout(const char* key, const char* value) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(key);
    ImGui::SameLine();
    rightMono(c::t1, value);
}

// --- measure ------------------------------------------------------------------------------

class MeasurePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Measure"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title(), nullptr, kTabFlags)) {
            ImGui::End();
            return;
        }
        ScopeSession& session = *context.scopeSession;
        const std::vector<ScopeTrace>& traces = session.traces();
        if (traces.empty()) {
            widgets::Caption("No signals plotted.");
            report().measured.clear();
            ImGui::End();
            return;
        }
        if (!session.cursors) {
            widgets::Caption("Measure works between cursors A and B.");
            if (widgets::Button("Show cursors A·B")) {
                session.showCursors(true);
            }
            ImGui::End();
            return;
        }
        const std::int64_t timeA = session.cursorTimeNs(session.cursorA);
        const std::int64_t timeB = session.cursorTimeNs(session.cursorB);
        // Again when a cursor moved (the table follows a drag), else a few times a second.
        const double now = ImGui::GetTime();
        if (measured_.size() != traces.size() || timeA != timeA_ || timeB != timeB_ ||
            now - computed_ >= kMeasureIntervalSeconds) {
            measured_.resize(traces.size());
            report().measured.resize(traces.size());
            for (std::size_t i = 0; i < traces.size(); ++i) {
                measured_[i] = measure(*traces[i].buffer, timeA, timeB);
                report().measured[i] = {traces[i].label, measured_[i]};
            }
            timeA_ = timeA;
            timeB_ = timeB;
            computed_ = now;
        }

        char text[64];
        const double seconds = std::abs(session.cursorB - session.cursorA);
        std::snprintf(text, sizeof(text), "%.3f s", seconds);
        readout("Δt  B − A", text);
        if (seconds > 0.0) {
            std::snprintf(text, sizeof(text), "%.4g Hz", 1.0 / seconds);
        } else {
            std::snprintf(text, sizeof(text), "-");
        }
        readout("1 / Δt", text);

        if (ImGui::BeginTable("##measure", 5, kTableFlags)) {
            ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_None, 3.0F);
            for (const char* name : {"min", "max", "mean", "rms"}) {
                ImGui::TableSetupColumn(name, ImGuiTableColumnFlags_None, 1.0F);
            }
            ImGui::TableNextRow();
            headerCell("signal", false);
            for (const char* name : {"min", "max", "mean", "rms"}) {
                headerCell(name, true);
            }
            for (std::size_t i = 0; i < traces.size(); ++i) {
                const Measurement& m = measured_[i];
                ImGui::TableNextRow();
                traceCell(traces[i]);
                for (const double value : {m.min, m.max, m.mean, m.rms}) {
                    ImGui::TableNextColumn();
                    number(m.count == 0 ? NAN : value, text, sizeof(text), "%.3g");
                    rightMono(c::t1, text);
                }
            }
            ImGui::EndTable();
        }
        if (ImGui::BeginTable("##delta", 4, kTableFlags)) {
            ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_None, 3.0F);
            ImGui::TableSetupColumn("p-p", ImGuiTableColumnFlags_None, 1.0F);
            ImGui::TableSetupColumn("slope", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableSetupColumn("samples", ImGuiTableColumnFlags_None, 1.2F);
            ImGui::TableNextRow();
            headerCell("signal", false);
            headerCell("p − p", true);
            headerCell("slope / s", true);
            headerCell("samples", true);
            for (std::size_t i = 0; i < traces.size(); ++i) {
                const Measurement& m = measured_[i];
                ImGui::TableNextRow();
                traceCell(traces[i]);
                ImGui::TableNextColumn();
                number(m.count == 0 ? NAN : m.peakToPeak, text, sizeof(text), "%.3g");
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                number(m.slope, text, sizeof(text), "%+.3g");
                rightMono(c::t1, text);
                ImGui::TableNextColumn();
                std::snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(m.count));
                rightMono(c::t3, text);
            }
            ImGui::EndTable();
        }
        widgets::Caption("raw samples from A to B  ·  units follow each signal");
        ImGui::End();
    }

private:
    std::vector<Measurement> measured_;
    std::int64_t timeA_{-1};
    std::int64_t timeB_{-1};
    double computed_{-1.0};
};

// --- FFT ----------------------------------------------------------------------------------

class FftPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "FFT"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title(), nullptr, kTabFlags)) {
            ImGui::End();
            return;
        }
        const ScopeSession& session = *context.scopeSession;
        const std::vector<ScopeTrace>& traces = session.traces();
        if (traces.empty()) {
            widgets::Caption("No signals plotted.");
            report().spectrumTrace.clear();
            report().spectrum = {};
            ImGui::End();
            return;
        }
        // The trace: the one chosen, while it is plotted; else the first.
        int index = -1;
        for (std::size_t i = 0; i < traces.size(); ++i) {
            if (traces[i].label == traceLabel_) {
                index = static_cast<int>(i);
            }
        }
        if (index < 0) {
            index = 0;
            traceLabel_ = traces[0].label;
            computed_ = -1.0;
        }
        const ScopeTrace& trace = traces[static_cast<std::size_t>(index)];

        // Which trace; then the window, the amplitude scale and the frequency axis.
        const float pickerWidth = ImGui::GetContentRegionAvail().x;
        const std::string shown = fit::cut(theme::fonts().mono, px(theme::kMonoSize), trace.label,
                                           pickerWidth - px(28.0F), true);
        if (widgets::DropdownField("##ffttrace", shown.c_str(), unitText(trace.unit),
                                   pickerWidth)) {
            ImGui::OpenPopup("##ffttraces");
        }
        ImGui::SetItemTooltip("The signal whose spectrum is shown: %s", trace.label.c_str());
        if (ImGui::BeginPopup("##ffttraces")) {
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            for (const ScopeTrace& t : traces) {
                if (ImGui::Selectable(t.label.c_str(), t.label == traceLabel_)) {
                    traceLabel_ = t.label;
                    computed_ = -1.0;
                }
            }
            ImGui::PopFont();
            ImGui::EndPopup();
        }
        static constexpr const char* kWindows[] = {"Hann", "rect"};
        if (widgets::Segmented("##fftwindow", kWindows, 2, &window_)) {
            computed_ = -1.0;
        }
        ImGui::SetItemTooltip("Window: Hann (leaks less) or rectangular (none)");
        ImGui::SameLine(0.0F, px(6.0F));
        static constexpr const char* kScales[] = {"lin", "dB"};
        widgets::Segmented("##fftscale", kScales, 2, &scale_);
        ImGui::SetItemTooltip("Amplitude: linear, or in dB of the signal's unit");
        ImGui::SameLine(0.0F, px(6.0F));
        static constexpr const char* kAxes[] = {"f lin", "f log"};
        if (widgets::Segmented("##fftaxis", kAxes, 2, &axis_)) {
            fitX_ = true;
        }
        ImGui::SetItemTooltip("Frequency axis: linear or logarithmic");

        // Over the raw samples of the visible window; live a couple of times a second, paused
        // only when the view changes.
        const ScopeSession::View view = session.view();
        const double now = ImGui::GetTime();
        const bool moved = view.beginNs != viewBegin_ || view.endNs != viewEnd_;
        if (computed_ < 0.0 ||
            (moved && (session.paused() || now - computed_ >= kSpectrumIntervalSeconds))) {
            spectrum_ =
                spectrumOf(*trace.buffer, view.beginNs, view.endNs + 1,
                           window_ == 0 ? SpectrumWindow::Hann : SpectrumWindow::Rectangular,
                           kSpectrumSamples);
            viewBegin_ = view.beginNs;
            viewEnd_ = view.endNs;
            computed_ = now;
            fitX_ = fitX_ || report().spectrumTrace != trace.label;
            report().spectrumTrace = trace.label;
            report().spectrum = spectrum_;
        }
        if (spectrum_.samples == 0) {
            widgets::Caption("Too few samples in the visible window for a spectrum.");
            ImGui::End();
            return;
        }

        const char* unit = unitText(trace.unit);
        const float footer = 3.0F * ImGui::GetFrameHeightWithSpacing() + px(4.0F);
        const float height = std::max(ImGui::GetContentRegionAvail().y - footer, px(120.0F));
        plot(trace, height);

        char text[96];
        std::snprintf(text, sizeof(text), "%.4g Hz  ·  %.3g %s", spectrum_.peakHz,
                      spectrum_.peakAmplitude, unit);
        readout("peak", text);
        std::snprintf(text, sizeof(text), "%.4g %s", spectrum_.dc, unit);
        readout("mean (taken out)", text);
        std::snprintf(text, sizeof(text), "N %zu  ·  %.3g s  ·  Δf %.3g Hz", spectrum_.samples,
                      static_cast<double>(spectrum_.samples) * spectrum_.sampleSeconds,
                      spectrum_.binHz);
        readout(window_ == 0 ? "Hann" : "rectangular", text);
        ImGui::End();
    }

private:
    void plot(const ScopeTrace& trace, float height) {
        // Peak-preserving decimation: the largest bin of each pixel column.
        const std::size_t bins = spectrum_.amplitude.size();
        const auto columns =
            static_cast<std::size_t>(std::max(ImGui::GetContentRegionAvail().x, 64.0F));
        const std::size_t per = std::max<std::size_t>(1, bins / columns);
        xs_.clear();
        ys_.clear();
        for (std::size_t k = 1; k < bins; k += per) {  // bin 0 is the mean, taken out
            std::size_t best = k;
            for (std::size_t j = k; j < std::min(k + per, bins); ++j) {
                if (spectrum_.amplitude[j] > spectrum_.amplitude[best]) {
                    best = j;
                }
            }
            xs_.push_back(spectrum_.hz[best]);
            ys_.push_back(value(spectrum_.amplitude[best]));
        }
        if (ImPlot::BeginPlot("##spectrum", ImVec2(-1.0F, height), ImPlotFlags_CanvasOnly)) {
            // Frequency: the whole spectrum when it changes; then the mouse zooms (double-click
            // fits again).
            ImPlot::SetupAxis(ImAxis_X1, nullptr,
                              ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight);
            ImPlot::SetupAxisScale(ImAxis_X1, axis_ == 1 ? ImPlotScale_Log10 : ImPlotScale_Linear);
            ImPlot::SetupAxisLimits(ImAxis_X1, axis_ == 1 ? spectrum_.binHz : 0.0,
                                    spectrum_.hz.back(),
                                    fitX_ ? ImPlotCond_Always : ImPlotCond_Once);
            fitX_ = false;
            ImPlot::SetupAxis(
                ImAxis_Y1, nullptr,
                ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoMenus | ImPlotAxisFlags_NoHighlight);
            // Short labels on the log axis: a decade is about 50 px wide here.
            ImPlot::SetupAxisFormat(ImAxis_X1, axis_ == 1 ? "%g" : "%g Hz");
            ImPlot::SetupAxisFormat(ImAxis_Y1, scale_ == 1 ? "%g dB" : "%g");
            ImPlot::SetupFinish();
            const ImVec4 color = theme::kSeries[trace.color % ScopeSession::kColors];
            ImPlotSpec line;
            line.LineColor = color;
            line.LineWeight = theme::kSeriesLineWeight;
            ImPlot::PlotLine(trace.label.c_str(), xs_.data(), ys_.data(),
                             static_cast<int>(xs_.size()), line);
            // The peak: a tag on the frequency axis.
            ImPlot::TagX(spectrum_.peakHz, c::accent, "%.3g Hz", spectrum_.peakHz);
            ImPlot::EndPlot();
        }
    }

    [[nodiscard]] double value(double amplitude) const {
        if (scale_ == 0) {
            return amplitude;
        }
        return amplitude > 0.0 ? std::max(20.0 * std::log10(amplitude), kDbFloor) : kDbFloor;
    }

    std::string traceLabel_;
    int window_{0};  // 0 Hann, 1 rectangular
    int scale_{1};   // 0 linear, 1 dB
    int axis_{1};    // frequency axis: 0 linear, 1 logarithmic
    bool fitX_{true};
    Spectrum spectrum_;
    std::int64_t viewBegin_{-1};
    std::int64_t viewEnd_{-1};
    double computed_{-1.0};
    std::vector<double> xs_;
    std::vector<double> ys_;
};

// --- saved sets ---------------------------------------------------------------------------

class SavedSetsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "###SavedSets"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        if (!loaded_ && !context.configDirectory.empty()) {
            path_ = context.configDirectory + "/signal_sets.tsv";
            sets_ = loadSignalSets(path_);
            // The home directory as "~": the path fits the panel.
            shownPath_ = path_;
            const char* home = std::getenv("HOME");
            if (home != nullptr && *home != '\0' && shownPath_.starts_with(home)) {
                shownPath_ = "~" + shownPath_.substr(std::string_view(home).size());
            }
            loaded_ = true;
        }
        report().savedSets = sets_.size();
        char label[64];
        std::snprintf(label, sizeof(label), "Saved sets · %zu###SavedSets", sets_.size());
        if (!ImGui::Begin(label, nullptr, kTabFlags)) {
            ImGui::End();
            return;
        }
        ScopeSession& session = *context.scopeSession;
        const std::vector<ScopeTrace>& traces = session.traces();

        // Save what is plotted under a name; the same name replaces that set.
        const float button = widgets::ButtonWidth("Save plotted");
        const float field =
            std::max(ImGui::GetContentRegionAvail().x - button - px(6.0F), px(60.0F));
        if (name_.empty()) {
            name_ = "set " + std::to_string(sets_.size() + 1);
        }
        widgets::CommitField("##setname", &name_, nullptr, false, field, false);
        ImGui::SetItemTooltip("The name to save the plotted signals under");
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(traces.empty());
        if (widgets::PrimaryButton("Save plotted")) {
            SignalSet set;
            set.name = name_;
            for (const ScopeTrace& trace : traces) {
                set.signals.emplace_back(trace.name, trace.component);
            }
            storeSignalSet(sets_, std::move(set));
            save();
            name_.clear();
        }
        ImGui::EndDisabled();
        if (traces.empty()) {
            widgets::Caption("Plot signals to save them as a set.");
        }

        ImGui::Spacing();
        if (sets_.empty()) {
            widgets::Caption("No saved sets yet.");
        }
        const SignalCatalog* catalog = context.catalog.get();
        std::size_t erase = sets_.size();
        for (std::size_t i = 0; i < sets_.size(); ++i) {
            const SignalSet& set = sets_[i];
            ImGui::PushID(static_cast<int>(i));
            std::size_t missing = 0;
            if (catalog != nullptr) {
                for (const auto& [name, component] : set.signals) {
                    const SignalInfo* info = catalog->find(name);
                    missing += (info == nullptr ||
                                component >= std::max<std::uint32_t>(info->components, 1))
                                   ? 1
                                   : 0;
                }
            }
            const bool current = isPlotted(session, set);
            ImGui::Separator();
            ImGui::AlignTextToFramePadding();
            if (current) {
                widgets::Dot(c::accent);
                ImGui::SameLine(0.0F, px(6.0F));
            }
            widgets::Strong("%s", set.name.c_str());
            if (ImGui::IsItemHovered()) {
                std::string list;
                for (const auto& [name, component] : set.signals) {
                    list += name;
                    if (component != 0) {
                        list += "[" + std::to_string(component) + "]";
                    }
                    list += "\n";
                }
                ImGui::SetTooltip("%s", list.c_str());
            }
            // Right: Plot, Add, delete.
            const float plotWidth = widgets::ButtonWidth("Plot");
            const float addWidth = widgets::ButtonWidth("Add");
            const float deleteWidth = widgets::ButtonWidth("##delete", widgets::Icon::Delete);
            ImGui::SameLine();
            widgets::AlignRight(plotWidth + addWidth + deleteWidth + px(8.0F));
            ImGui::BeginDisabled(catalog == nullptr || missing == set.signals.size());
            if (widgets::Button("Plot",
                                current ? widgets::ButtonKind::On : widgets::ButtonKind::Default)) {
                plot(context, session, set, true);
            }
            ImGui::SetItemTooltip("Plot this set in place of what is plotted");
            ImGui::SameLine(0.0F, px(4.0F));
            if (widgets::Button("Add")) {
                plot(context, session, set, false);
            }
            ImGui::SetItemTooltip("Add this set to what is plotted");
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(4.0F));
            if (widgets::Button("##delete", widgets::ButtonKind::Ghost, widgets::Icon::Delete)) {
                erase = i;
            }
            ImGui::SetItemTooltip("Delete this set");
            // Under the name: how many signals, and those this run does not have.
            char meta[64];
            std::snprintf(meta, sizeof(meta), "%zu signal%s", set.signals.size(),
                          set.signals.size() == 1 ? "" : "s");
            widgets::Mono(c::t3, "%s", meta);
            if (missing > 0) {
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::Mono(c::warn, "%zu not in this run", missing);
            }
            ImGui::PopID();
        }
        if (erase < sets_.size()) {
            sets_.erase(sets_.begin() + static_cast<std::ptrdiff_t>(erase));
            save();
        }
        ImGui::Spacing();
        if (!path_.empty()) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("kept on this machine in %s", shownPath_.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }

private:
    static bool isPlotted(const ScopeSession& session, const SignalSet& set) {
        const std::vector<ScopeTrace>& traces = session.traces();
        if (traces.size() != set.signals.size()) {
            return false;
        }
        return std::ranges::all_of(set.signals, [&](const auto& signal) {
            return session.find(signal.first, signal.second) >= 0;
        });
    }

    static void plot(PanelContext& context, ScopeSession& session, const SignalSet& set,
                     bool replace) {
        if (!context.catalog) {
            return;
        }
        if (replace) {
            while (!session.traces().empty()) {
                session.remove(context.client, session.traces().size() - 1);
            }
        }
        std::size_t plotted = 0;
        for (const auto& [name, component] : set.signals) {
            plotted += session.add(context.client, *context.catalog, name, component) ? 1 : 0;
        }
        report().plottedFromSet = plotted;
    }

    void save() {
        if (!path_.empty()) {
            saveSignalSets(path_, sets_);
        }
    }

    bool loaded_{false};
    std::string path_;
    std::string shownPath_;
    std::vector<SignalSet> sets_;
    std::string name_;
};

std::string jsonText(const std::string& text) {
    std::string out;
    for (const char ch : text) {
        if (ch == '"' || ch == '\\') {
            out += '\\';
        }
        if (static_cast<unsigned char>(ch) >= 0x20) {
            out += ch;
        }
    }
    return out;
}

std::string jsonNumber(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    char text[48];
    std::snprintf(text, sizeof(text), "%.9g", value);
    return text;
}

}  // namespace

std::vector<std::unique_ptr<Panel>> createSignalsTabPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<SavedSetsPanel>());
    panels.push_back(std::make_unique<MeasurePanel>());
    panels.push_back(std::make_unique<FftPanel>());
    return panels;
}

std::string signalsTabsReport() {
    const Report& r = report();
    std::string text = "{\"measure\": {";
    for (std::size_t i = 0; i < r.measured.size(); ++i) {
        const Measurement& m = r.measured[i].m;
        text += (i == 0 ? "\"" : ", \"") + jsonText(r.measured[i].label) +
                "\": {\"count\": " + std::to_string(m.count) + ", \"min\": " + jsonNumber(m.min) +
                ", \"max\": " + jsonNumber(m.max) + ", \"mean\": " + jsonNumber(m.mean) +
                ", \"rms\": " + jsonNumber(m.rms) + ", \"delta\": " + jsonNumber(m.delta) +
                ", \"seconds\": " + jsonNumber(m.seconds) + "}";
    }
    const Spectrum& s = r.spectrum;
    text += "}, \"fft\": {\"trace\": \"" + jsonText(r.spectrumTrace) +
            "\", \"samples\": " + std::to_string(s.samples) +
            ", \"bin_hz\": " + jsonNumber(s.binHz) + ", \"peak_hz\": " + jsonNumber(s.peakHz) +
            ", \"peak_amplitude\": " + jsonNumber(s.peakAmplitude) +
            ", \"dc\": " + jsonNumber(s.dc) + "}, \"saved_sets\": " + std::to_string(r.savedSets) +
            ", \"plotted_from_set\": " + std::to_string(r.plottedFromSet) + "}";
    return text;
}

}  // namespace lagrange::viewer
