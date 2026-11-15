// Replay workspace: comparing the replayed run with a second recording
// (docs/design/ui/08-replay/SPEC.md): the Compare tab, Sim vs real, Validation and its
// History, and the Source strip that replaces the domain strip while a recording plays.
//
// "Real" is whatever second recording the user loads: a log of a real robot converted into a
// recording, or, as the project has no hardware logs, another simulated run (another engine,
// profile or parameter). Both files are read on this machine (CompareLog.hpp); a recording the
// server replays from another machine cannot be compared yet (that needs the protocol
// additions listed in the panel and in docs/design/viewer.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/CompareLog.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Icons.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
namespace cmp = compare;
using widgets::px;

constexpr double kDefaultLimit = 0.05;  // NRMSE limit of the shared-signals suite (5 %)
constexpr double kDefaultResampleHz = 100.0;
constexpr const char* kCompareTitle = "Compare";
constexpr const char* kSimVsRealTitle = "Sim vs real";

ImU32 u32(const ImVec4& color, float alpha = 1.0F) {
    return ImGui::ColorConvertFloat4ToU32(ImVec4(color.x, color.y, color.z, color.w * alpha));
}

std::string fileName(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

/// "1.82 GB", "36.7 MB"
void bytes(std::uint64_t size, char* out, std::size_t length) {
    const double megabytes = static_cast<double>(size) / (1024.0 * 1024.0);
    if (megabytes >= 1024.0) {
        std::snprintf(out, length, "%.2f GB", megabytes / 1024.0);
    } else {
        std::snprintf(out, length, "%.1f MB", megabytes);
    }
}

/// A value with a sensible number of digits.
void value(double v, const std::string& unit, char* out, std::size_t length) {
    const double a = std::abs(v);
    const char* format = a >= 100.0 ? "%.0f %s" : (a >= 10.0 ? "%.1f %s" : "%.3g %s");
    std::snprintf(out, length, format, v, unit.c_str());
}

void percent(double fraction, char* out, std::size_t length) {
    if (!std::isfinite(fraction)) {
        std::snprintf(out, length, "-");
    } else {
        std::snprintf(out, length, fraction >= 0.1 ? "%.0f %%" : "%.1f %%", fraction * 100.0);
    }
}

/// Width of `text` as widgets::Mono draws it.
float monoWidth(const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    const float width = ImGui::CalcTextSize(text).x;
    ImGui::PopFont();
    return width;
}

/// The meter colour of an NRMSE against its limit (design: green, amber near it, red over).
ImVec4 verdictColor(double nrmse, double limit) {
    if (!(nrmse <= limit)) {
        return c::error;
    }
    return nrmse > 0.8 * limit ? c::warn : c::ok;
}

// --- shared state -------------------------------------------------------------------------

/// What the worker found: both logs, the suite, its report.
struct Analysis {
    std::string simPath;
    std::string realPath;
    std::shared_ptr<const cmp::LogFile> sim;
    std::shared_ptr<const cmp::LogFile> real;
    cmp::Suite suite;
    cmp::Report report;
    std::vector<cmp::Unmapped> unmapped;
    std::string error;  ///< Why there is no report.
    std::string finishedAt;
};

struct HistoryEntry {
    std::string when;
    std::string suite;
    std::string real;
    std::size_t passed{0};
    std::size_t total{0};
};

struct Request {
    std::string simPath;
    std::string realPath;
    std::string suitePath;  ///< Empty: the shared-signals suite.
    std::int64_t periodNs{0};
    cmp::Method method;
    bool methodGiven{false};  ///< The Method fields override a suite file's method.
};

std::shared_ptr<const Analysis> analyse(const Request& request,
                                        std::shared_ptr<const cmp::LogFile> sim,
                                        std::shared_ptr<const cmp::LogFile> real) {
    auto out = std::make_shared<Analysis>();
    out->simPath = request.simPath;
    out->realPath = request.realPath;
    cmp::Suite suite;
    std::int64_t period = request.periodNs;
    if (!request.suitePath.empty()) {
        suite = cmp::loadSuite(request.suitePath);
        if (!suite.error.empty()) {
            out->error = suite.error;
            return out;
        }
        if (!request.methodGiven) {
            period = suite.periodNs;
        }
    }
    // Logs already read at this period are kept; anything else is read again.
    const auto load = [&](const std::string& path, std::shared_ptr<const cmp::LogFile>& log) {
        if (log && log->path == path && log->periodNs == period) {
            return true;
        }
        cmp::LoadResult loaded = cmp::loadLog(path, period);
        if (!loaded.log) {
            out->error = loaded.error;
            return false;
        }
        log = std::make_shared<const cmp::LogFile>(std::move(*loaded.log));
        return true;
    };
    if (!load(request.simPath, sim) || !load(request.realPath, real)) {
        return out;
    }
    out->sim = sim;
    out->real = real;
    if (request.suitePath.empty()) {
        suite = cmp::sharedSuite(*sim, *real, kDefaultLimit);
    }
    suite.periodNs = period;
    if (request.methodGiven || request.suitePath.empty()) {
        suite.method = request.method;
    }
    out->suite = suite;
    out->report = cmp::runSuite(*sim, *real, suite);
    out->unmapped = cmp::unmapped(*sim, *real, suite);
    const std::time_t now = std::time(nullptr);
    char when[32];
    std::strftime(when, sizeof(when), "%H:%M:%S", std::localtime(&now));
    out->finishedAt = when;
    return out;
}

/// What the Compare, Sim vs real, Validation and History panels share.
class CompareState {
public:
    // Inputs the user types.
    char realPath[512]{};
    char suitePath[512]{};
    int alignment{0};  ///< 0 cross-correlation, 1 none
    char resampleKhz[24]{"0.1"};
    char window[48]{"whole run"};
    bool methodEdited{false};

    std::shared_ptr<const Analysis> analysis;
    /// The Validation rows' texts as drawn (fit::cutAll), for `rowsFor` at `rowsRoom` px.
    std::shared_ptr<const Analysis> rowsFor;
    float rowsRoom{-1.0F};
    std::vector<std::string> rowTexts;
    std::size_t selected{0};
    std::vector<HistoryEntry> history;
    std::string exported;  ///< The last report written, or why it could not be.
    std::string simPath;   ///< The replayed recording as this machine sees it.
    bool simReadable{false};

    [[nodiscard]] bool busy() const { return job_.valid(); }

    /// Starts the worker for the current inputs.
    void run(bool blocking) {
        if (busy() || simPath.empty() || realPath[0] == '\0') {
            return;
        }
        Request request;
        request.simPath = simPath;
        request.realPath = realPath;
        request.suitePath = suitePath;
        request.method.align = alignment == 0;
        request.methodGiven = methodEdited;
        const double khz = std::atof(resampleKhz);
        request.periodNs = khz > 0.0 && khz <= 1000.0 ? std::llround(1e6 / khz)
                                                      : std::llround(1e9 / kDefaultResampleHz);
        parseWindow(request.method);
        std::shared_ptr<const cmp::LogFile> sim = analysis ? analysis->sim : nullptr;
        std::shared_ptr<const cmp::LogFile> real = analysis ? analysis->real : nullptr;
        job_ = std::async(std::launch::async,
                          [request, sim, real] { return analyse(request, sim, real); });
        if (blocking) {
            job_.wait();
        }
    }

    /// Takes the worker's result when it is done. UI thread, once per frame.
    void poll() {
        if (!job_.valid() || job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;
        }
        analysis = job_.get();
        selected = 0;
        if (analysis->error.empty()) {
            // Show the first failing row, as the design does.
            const auto& results = analysis->report.results;
            for (std::size_t i = 0; i < results.size(); ++i) {
                if (results[i].valid && !results[i].pass) {
                    selected = i;
                    break;
                }
            }
            history.push_back({analysis->finishedAt, analysis->suite.name,
                               fileName(analysis->realPath), analysis->report.passed(),
                               analysis->report.results.size()});
            // The Method fields show what was used.
            alignment = analysis->suite.method.align ? 0 : 1;
            std::snprintf(resampleKhz, sizeof(resampleKhz), "%g",
                          1e6 / static_cast<double>(analysis->suite.periodNs));
            if (analysis->suite.method.windowEndNs > 0) {
                std::snprintf(window, sizeof(window), "%.1f-%.1f",
                              static_cast<double>(analysis->suite.method.windowBeginNs) * 1e-9,
                              static_cast<double>(analysis->suite.method.windowEndNs) * 1e-9);
            } else {
                std::snprintf(window, sizeof(window), "whole run");
            }
        }
    }

    [[nodiscard]] const cmp::Result* selectedResult() const {
        if (!analysis || !analysis->error.empty() || analysis->report.results.empty()) {
            return nullptr;
        }
        return &analysis->report.results[std::min(selected, analysis->report.results.size() - 1)];
    }

    [[nodiscard]] bool ready() const { return analysis && analysis->error.empty(); }

private:
    void parseWindow(cmp::Method& method) const {
        double from = 0.0;
        double to = 0.0;
        if (std::sscanf(window, "%lf-%lf", &from, &to) == 2 && to > from && from >= 0.0) {
            method.windowBeginNs = std::llround(from * 1e9);
            method.windowEndNs = std::llround(to * 1e9);
        }
    }

    std::future<std::shared_ptr<const Analysis>> job_;
};

/// Set from the command line (--compare, --suite) before the first frame.
struct Pending {
    std::string real;
    std::string suite;
    bool blocking{false};
};
Pending& pending() {
    static Pending value;
    return value;
}

std::shared_ptr<CompareState>& sharedState() {
    static std::shared_ptr<CompareState> state = std::make_shared<CompareState>();
    return state;
}

/// Once per frame, from whichever compare panel draws first: follows the replayed recording
/// and starts what the command line asked for.
void update(PanelContext& context, CompareState& state) {
    static int lastFrame = -1;
    if (ImGui::GetFrameCount() == lastFrame) {
        return;
    }
    lastFrame = ImGui::GetFrameCount();
    state.poll();
    const std::shared_ptr<const RecordingFacts> facts = context.client.recording();
    const std::string path = facts ? facts->path : std::string();
    if (path != state.simPath) {
        state.simPath = path;
        std::error_code ec;
        state.simReadable = !path.empty() && std::filesystem::is_regular_file(path, ec);
        if (state.analysis && state.analysis->simPath != path) {
            state.analysis.reset();  // another recording was opened
        }
    }
    Pending& asked = pending();
    if (!asked.real.empty() && state.simReadable) {
        std::snprintf(state.realPath, sizeof(state.realPath), "%s", asked.real.c_str());
        std::snprintf(state.suitePath, sizeof(state.suitePath), "%s", asked.suite.c_str());
        asked.real.clear();
        state.run(asked.blocking);
        state.poll();
    }
}

/// Recordings on this machine the user may pick: the server's default recording directory
/// and the folder of the replayed recording, newest first.
std::vector<std::string> recordingsHere(const std::string& simPath) {
    std::vector<std::filesystem::path> folders;
    if (const char* state = std::getenv("XDG_STATE_HOME"); state != nullptr && *state != '\0') {
        folders.emplace_back(std::filesystem::path(state) / "lagrange" / "recordings");
    } else if (const char* home = std::getenv("HOME"); home != nullptr) {
        folders.emplace_back(std::filesystem::path(home) / ".local/state/lagrange/recordings");
    }
    if (!simPath.empty()) {
        folders.push_back(std::filesystem::path(simPath).parent_path());
    }
    std::vector<std::pair<std::filesystem::file_time_type, std::string>> found;
    for (const std::filesystem::path& folder : folders) {
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
            if (entry.path().extension() == ".mcap" && entry.path().string() != simPath) {
                const std::string name = entry.path().string();
                if (std::none_of(found.begin(), found.end(),
                                 [&](const auto& f) { return f.second == name; })) {
                    found.emplace_back(entry.last_write_time(ec), name);
                }
            }
        }
    }
    std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a > b; });
    std::vector<std::string> out;
    for (auto& [time, name] : found) {
        out.push_back(std::move(name));
    }
    return out;
}

/// The SPEC's "no compare log" state, shared by Compare and Sim vs real: a path and the
/// recordings found on this machine.
void drawPicker(PanelContext& context, CompareState& state, std::vector<std::string>& listed,
                double& listedAt) {
    (void)context;
    const double now = ImGui::GetTime();
    if (listedAt < 0.0 || now - listedAt > 5.0) {
        listed = recordingsHere(state.simPath);  // a directory listing every few seconds
        listedAt = now;
    }
    const float button = widgets::ButtonWidth("Compare");
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - button - px(6.0F));
    const bool entered =
        ImGui::InputTextWithHint("##realpath", "second recording (.mcap)", state.realPath,
                                 sizeof(state.realPath), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine(0.0F, px(6.0F));
    ImGui::BeginDisabled(state.busy() || state.realPath[0] == '\0' || !state.simReadable);
    if (widgets::Button("Compare", widgets::ButtonKind::Primary) || (entered && !state.busy())) {
        state.run(false);
    }
    ImGui::EndDisabled();
    if (!listed.empty()) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::Caption("recordings on this machine");
        for (std::size_t i = 0; i < listed.size() && i < 12; ++i) {
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable("##rec", false, 0, ImVec2(0.0F, px(22.0F)))) {
                std::snprintf(state.realPath, sizeof(state.realPath), "%s", listed[i].c_str());
                state.run(false);
            }
            ImGui::SetItemTooltip("%s", listed[i].c_str());
            ImGui::SameLine(px(14.0F));
            widgets::Mono(c::t1, "%s", fileName(listed[i]).c_str());
            ImGui::PopID();
        }
    }
}

/// Why nothing can be compared, or nullptr when something can.
const char* blocker(const PanelContext& context, const CompareState& state) {
    if (context.connection.state != ConnectionState::Ready) {
        return "Not connected.";
    }
    const std::shared_ptr<const RecordingFacts> facts = context.client.recording();
    if (!facts) {
        return "This server is not replaying a recording. Open one (File, Open recording) to "
               "compare it with another run.";
    }
    if (facts->path.empty()) {
        return "This replay is a generated run with no file, so there is nothing to compare.";
    }
    if (!state.simReadable) {
        return "The recording is on the server's machine, not on this one. Comparing reads both "
               "files here; a remote replay needs the protocol to carry the comparison "
               "(proposed, not in v1).";
    }
    return nullptr;
}

/// A status line while the worker runs or after it failed. Returns true if it drew one.
bool workerState(const CompareState& state) {
    if (state.busy()) {
        widgets::Caption("reading both recordings and comparing...");
        return true;
    }
    if (state.analysis && !state.analysis->error.empty()) {
        widgets::ParagraphF(theme::fonts().sans, theme::kLabelSize, c::error, "%s",
                            state.analysis->error.c_str());
        return true;
    }
    return false;
}

// --- Compare (left, beside Recording) -------------------------------------------------------

class ComparePanel final : public Panel {
public:
    explicit ComparePanel(std::shared_ptr<CompareState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return kCompareTitle; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Replay; }

    void draw(PanelContext& context) override {
        CompareState& state = *state_;
        update(context, state);
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (const char* why = blocker(context, state)) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", why);
            ImGui::PopTextWrapPos();
            ImGui::End();
            return;
        }
        if (!state.ready()) {
            widgets::Strong("Load a second recording to compare");
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "Another run of the same robot: a real robot's log converted into a "
                "recording, or another simulated run (another engine or profile).");
            ImGui::PopTextWrapPos();
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            workerState(state);
            drawPicker(context, state, listed_, listedAt_);
            ImGui::End();
            return;
        }
        const Analysis& a = *state.analysis;
        const cmp::LogFile& real = *a.real;
        char text[128];
        widgets::PropertyRow("compare", fileName(real.path).c_str(), nullptr, false,
                             real.path.c_str());
        format::clock(real.durationNs, text, sizeof(text));
        widgets::PropertyRow("duration", text, nullptr);
        bytes(real.sizeBytes, text, sizeof(text));
        widgets::PropertyRow("size", text, nullptr);
        std::snprintf(text, sizeof(text), "%zu", real.signals.size());
        widgets::PropertyRow("signals", text, nullptr);
        if (!real.generator.empty()) {
            widgets::PropertyRow("written by", real.generator.c_str(), nullptr);
        }
        widgets::PropertyRow("stage", fileName(real.stagePath).c_str(), nullptr, false,
                             real.stagePath.c_str());
        if (real.stagePath != a.sim->stagePath) {
            widgets::ParagraphF(theme::fonts().sans, theme::kLabelSize, c::warn,
                                "Another stage than the replay's (%s).",
                                fileName(a.sim->stagePath).c_str());
        }
        if (real.synthetic) {
            widgets::Pill("synthetic", widgets::PillKind::Warn);
        }
        std::snprintf(text, sizeof(text), "%zu mapped", a.suite.mappings.size());
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::SectionHeader("Signals", text);
        // Topics without a mapping, with the sim signal most like them (SPEC "States").
        std::snprintf(text, sizeof(text), "%zu", a.unmapped.size());
        if (widgets::SectionHeader("Unmapped", text,
                                   !a.unmapped.empty() && a.unmapped.size() < 40)) {
            ImGui::BeginChild("##unmapped", ImVec2(0.0F, -px(64.0F)));
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(a.unmapped.size()), px(20.0F));
            while (clipper.Step()) {
                for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                    const cmp::Unmapped& u = a.unmapped[static_cast<std::size_t>(i)];
                    widgets::Mono(c::t2, "%s", u.real.c_str());
                    if (!u.suggestion.empty()) {
                        ImGui::SetItemTooltip("suggestion: %s", u.suggestion.c_str());
                    }
                }
            }
            ImGui::EndChild();
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        if (widgets::Button("Compare another", widgets::ButtonKind::Ghost)) {
            state.analysis.reset();
        }
        ImGui::End();
    }

private:
    std::shared_ptr<CompareState> state_;
    std::vector<std::string> listed_;
    double listedAt_{-1.0};
};

// --- Sim vs real (centre, under the timeline) -----------------------------------------------

struct Line {
    const std::vector<double>* y{nullptr};
    ImVec4 color;
    float weight{1.3F};
    bool dashed{false};
};

/// Strokes a polyline, dashed (4 on, 2 off, as the design's real series) when asked.
void stroke(ImDrawList* draw, const std::vector<ImVec2>& points, ImU32 color, float weight,
            bool dashed) {
    if (points.size() < 2) {
        return;
    }
    if (!dashed) {
        draw->AddPolyline(points.data(), static_cast<int>(points.size()), color, ImDrawFlags_None,
                          weight);
        return;
    }
    const float on = px(4.0F);
    const float off = px(2.0F);
    float phase = 0.0F;  // distance into the current on+off period
    for (std::size_t i = 1; i < points.size(); ++i) {
        ImVec2 from = points[i - 1];
        const ImVec2 to = points[i];
        float length = std::hypot(to.x - from.x, to.y - from.y);
        const ImVec2 dir = length > 0.0F
                               ? ImVec2((to.x - from.x) / length, (to.y - from.y) / length)
                               : ImVec2(0.0F, 0.0F);
        while (length > 0.0F) {
            const bool drawing = phase < on;
            const float left = drawing ? on - phase : on + off - phase;
            const float step = std::min(left, length);
            const ImVec2 next(from.x + dir.x * step, from.y + dir.y * step);
            if (drawing) {
                draw->AddLine(from, next, color, weight);
            }
            from = next;
            length -= step;
            phase += step;
            if (phase >= on + off) {
                phase = 0.0F;
            }
        }
    }
}

/// A plot box drawn as the design's SVG plots: bg0 box, grid at the top and bottom labels, a
/// zero line, the series, an optional tolerance band, the playhead. Click to seek.
/// Returns the time clicked (s), or NaN.
double plotBox(const char* id, float height, const std::vector<double>& t,
               const std::vector<Line>& lines, double band, double nowS) {
    const float width = ImGui::GetContentRegionAvail().x;
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    ImGui::InvisibleButton(id, ImVec2(width, height));
    const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(c::bg0), px(3.0F));
    draw->AddRect(min, max, u32(c::line), px(3.0F));
    if (t.size() < 2) {
        return std::nan("");
    }
    double lo = band > 0.0 ? -band : std::numeric_limits<double>::infinity();
    double hi = band > 0.0 ? band : -std::numeric_limits<double>::infinity();
    for (const Line& line : lines) {
        for (const double v : *line.y) {
            if (std::isfinite(v)) {
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    }
    if (!(hi > lo)) {
        hi = lo + 1.0;
        lo -= 1.0;
    }
    const double pad = (hi - lo) * 0.08;
    lo -= pad;
    hi += pad;
    const float left = min.x + px(44.0F);
    const float right = max.x - px(12.0F);
    const float top = min.y + px(8.0F);
    const float bottom = max.y - px(8.0F);
    const double t0 = t.front();
    const double t1 = t.back() > t0 ? t.back() : t0 + 1.0;
    const auto X = [&](double s) {
        return left + static_cast<float>((s - t0) / (t1 - t0)) * (right - left);
    };
    const auto Y = [&](double v) {
        return bottom - static_cast<float>((v - lo) / (hi - lo)) * (bottom - top);
    };
    // Tolerance band, grid lines and their labels.
    if (band > 0.0) {
        draw->AddRectFilled(ImVec2(left, Y(band)), ImVec2(right, Y(-band)), u32(c::ok, 0.08F));
    }
    draw->AddLine(ImVec2(left, top), ImVec2(right, top), u32(c::bg3));
    draw->AddLine(ImVec2(left, bottom), ImVec2(right, bottom), u32(c::bg3));
    if (lo < 0.0 && hi > 0.0) {
        draw->AddLine(ImVec2(left, Y(0.0)), ImVec2(right, Y(0.0)), u32(c::lineStrong));
    }
    ImFont* mono = theme::fonts().mono;
    const float labelSize = px(10.0F);
    const auto label = [&](double v, float y) {
        char text[24];
        std::snprintf(text, sizeof(text), "%.3g", v);
        const ImVec2 extent = mono->CalcTextSizeA(labelSize, FLT_MAX, 0.0F, text);
        draw->AddText(mono, labelSize, ImVec2(left - px(6.0F) - extent.x, y - extent.y * 0.5F),
                      u32(c::t3), text);
    };
    label(hi - pad, Y(hi - pad));
    label(lo + pad, Y(lo + pad));
    if (lo < 0.0 && hi > 0.0 && std::abs(Y(0.0) - Y(hi - pad)) > labelSize &&
        std::abs(Y(0.0) - Y(lo + pad)) > labelSize) {
        label(0.0, Y(0.0));
    }
    // Series: at most two points per pixel column (its min and max), so no peak is lost.
    draw->PushClipRect(ImVec2(left, min.y), ImVec2(right, max.y), true);
    const auto columns = static_cast<std::size_t>(std::max(1.0F, right - left));
    std::vector<ImVec2> points;
    for (const Line& line : lines) {
        points.clear();
        const std::vector<double>& y = *line.y;
        const std::size_t n = std::min(t.size(), y.size());
        if (n <= columns * 2) {
            for (std::size_t i = 0; i < n; ++i) {
                if (std::isfinite(y[i])) {
                    points.emplace_back(X(t[i]), Y(y[i]));
                }
            }
        } else {
            std::size_t i = 0;
            for (std::size_t col = 0; col < columns && i < n; ++col) {
                const double end =
                    t0 + (t1 - t0) * static_cast<double>(col + 1) / static_cast<double>(columns);
                double cmin = std::numeric_limits<double>::infinity();
                double cmax = -cmin;
                bool minFirst = true;
                std::size_t kmin = i;
                std::size_t kmax = i;
                for (; i < n && t[i] <= end; ++i) {
                    if (std::isfinite(y[i])) {
                        if (y[i] < cmin) {
                            cmin = y[i];
                            kmin = i;
                        }
                        if (y[i] > cmax) {
                            cmax = y[i];
                            kmax = i;
                        }
                    }
                }
                if (cmax >= cmin) {
                    minFirst = kmin <= kmax;
                    const float x = left + static_cast<float>(col) + 0.5F;
                    points.emplace_back(x, Y(minFirst ? cmin : cmax));
                    points.emplace_back(x, Y(minFirst ? cmax : cmin));
                }
            }
        }
        stroke(draw, points, u32(line.color), px(line.weight), line.dashed);
    }
    if (std::isfinite(nowS) && nowS >= t0 && nowS <= t1) {
        draw->AddLine(ImVec2(X(nowS), top), ImVec2(X(nowS), bottom), u32(c::accent), px(1.0F));
    }
    draw->PopClipRect();
    if (clicked) {
        const float mx = ImGui::GetIO().MousePos.x;
        return t0 + (t1 - t0) *
                        static_cast<double>(std::clamp((mx - left) / (right - left), 0.0F, 1.0F));
    }
    return std::nan("");
}

/// The legend at the top right of a plot: bold name, unit, then entries (swatch + label).
struct LegendItem {
    ImVec4 color;
    bool dashed{false};
    bool block{false};  ///< A filled block (the band) instead of a line.
    std::string text;
};
void legend(const ImVec2& boxMin, float boxWidth, const char* name, const std::string& unit,
            const std::vector<LegendItem>& items) {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImFont* bold = theme::fonts().sansSemiBold;
    ImFont* sans = theme::fonts().sans;
    const float size = px(theme::kLabelSize);
    const float gap = px(10.0F);
    const float swatch = px(14.0F);
    float width = bold->CalcTextSizeA(size, FLT_MAX, 0.0F, name).x + gap +
                  sans->CalcTextSizeA(size, FLT_MAX, 0.0F, unit.c_str()).x;
    for (const LegendItem& item : items) {
        width +=
            gap + swatch + px(5.0F) + sans->CalcTextSizeA(size, FLT_MAX, 0.0F, item.text.c_str()).x;
    }
    float x = boxMin.x + boxWidth - px(12.0F) - width;
    const float y = boxMin.y + px(6.0F);
    // On the plot's own background, so that a curve passing under it does not run through the
    // text (QA2 F10).
    draw->AddRectFilled(ImVec2(x - px(6.0F), y - px(2.0F)),
                        ImVec2(x + width + px(6.0F), y + size + px(3.0F)), u32(c::bg0, 0.9F),
                        px(3.0F));
    draw->AddText(bold, size, ImVec2(x, y), u32(c::t1), name);
    x += bold->CalcTextSizeA(size, FLT_MAX, 0.0F, name).x + gap;
    draw->AddText(sans, size, ImVec2(x, y), u32(c::t3), unit.c_str());
    x += sans->CalcTextSizeA(size, FLT_MAX, 0.0F, unit.c_str()).x + gap;
    const float mid = y + size * 0.55F;
    for (const LegendItem& item : items) {
        if (item.block) {
            draw->AddRectFilled(ImVec2(x, mid - px(3.0F)), ImVec2(x + swatch, mid + px(3.0F)),
                                u32(item.color), px(1.0F));
        } else {
            stroke(draw, {ImVec2(x, mid), ImVec2(x + swatch, mid)}, u32(item.color), px(2.0F),
                   item.dashed);
        }
        x += swatch + px(5.0F);
        draw->AddText(sans, size, ImVec2(x, y), u32(c::t2), item.text.c_str());
        x += sans->CalcTextSizeA(size, FLT_MAX, 0.0F, item.text.c_str()).x + gap;
    }
}

/// One metric card: a caption over a 15 px mono value.
void metricCard(const char* caption, const char* text, const ImVec4& color, float width) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const float height = px(44.0F);
    ImGui::Dummy(ImVec2(width, height));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, ImVec2(min.x + width, min.y + height), u32(c::bg0), px(3.0F));
    draw->AddRect(min, ImVec2(min.x + width, min.y + height), u32(c::line), px(3.0F));
    draw->AddText(theme::fonts().sans, px(theme::kCaptionSize),
                  ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(c::t3), caption);
    draw->AddText(theme::fonts().mono, px(15.0F), ImVec2(min.x + px(8.0F), min.y + px(20.0F)),
                  u32(color), text);
}

class SimVsRealPanel final : public Panel {
public:
    explicit SimVsRealPanel(std::shared_ptr<CompareState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return kSimVsRealTitle; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Replay; }

    void draw(PanelContext& context) override {
        CompareState& state = *state_;
        update(context, state);
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (const char* why = blocker(context, state)) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", why);
            ImGui::PopTextWrapPos();
            ImGui::End();
            return;
        }
        const cmp::Result* r = state.selectedResult();
        if (r == nullptr) {
            // SPEC "States": no compare log.
            widgets::Strong("Load a real log to compare");
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "Any second recording of this robot: a real run converted into a "
                "recording, or another simulated run.");
            ImGui::PopTextWrapPos();
            workerState(state);
            if (state.ready() && state.analysis->report.results.empty()) {
                widgets::Caption("The two recordings share no signal that changes.");
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            drawPicker(context, state, listed_, listedAt_);
            ImGui::End();
            return;
        }
        const cmp::Suite& suite = state.analysis->suite;
        char text[160];
        // The header line (design: the panel bar's tools).
        if (suite.method.windowEndNs > 0) {
            std::snprintf(text, sizeof(text), "window %.1f-%.1f s · %s",
                          static_cast<double>(suite.method.windowBeginNs) * 1e-9,
                          static_cast<double>(suite.method.windowEndNs) * 1e-9,
                          suite.method.align ? "aligned by cross-correlation" : "not aligned");
        } else {
            std::snprintf(text, sizeof(text), "whole run · %s",
                          suite.method.align ? "aligned by cross-correlation" : "not aligned");
        }
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        widgets::AlignRight(ImGui::CalcTextSize(text).x);
        ImGui::PopFont();
        widgets::Caption("%s", text);
        if (!r->valid) {
            widgets::Caption("%s: too few samples in both recordings to compare.", r->sim.c_str());
            ImGui::End();
            return;
        }
        const double nowS = static_cast<double>(context.status.simTimeNs) * 1e-9;
        const bool canSeek =
            context.connection.state == ConnectionState::Ready && context.connection.canSeek;
        const ImVec4 simColor = theme::kSeries[1];

        // The overlay: sim solid in a series colour, real dashed in t2.
        ImVec2 boxMin = ImGui::GetCursorScreenPos();
        const float boxWidth = ImGui::GetContentRegionAvail().x;
        double clicked = plotBox(
            "##overlay", px(140.0F), r->t,
            {Line{&r->realValues, c::t2, 1.2F, true}, Line{&r->simValues, simColor, 1.4F, false}},
            0.0, nowS);
        std::string realLabel = "real · " + r->real;
        legend(boxMin, boxWidth, r->sim.c_str(), r->unit,
               {{simColor, false, false, "sim"}, {c::t2, true, false, realLabel}});
        // The error, real − sim, with the tolerance band (the limit, as a share of the range).
        // A constant reference has no range: its band is the RMSE allowed instead.
        const double band = r->constantReference ? r->absLimit : r->limit * r->range;
        boxMin = ImGui::GetCursorScreenPos();
        const double clickedError = plotBox("##error", px(64.0F), r->t,
                                            {Line{&r->error, c::accent, 1.1F, false}}, band, nowS);
        char bandText[48];
        value(band, r->unit, bandText, sizeof(bandText));
        legend(boxMin, boxWidth, "error real − sim", r->unit,
               {{c::ok, false, true, std::string("±") + bandText + " band"}});
        if (std::isnan(clicked)) {
            clicked = clickedError;
        }
        if (!std::isnan(clicked) && canSeek) {
            context.client.seek(std::llround(clicked * 1e9));
        }

        // Metric cards.
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float gap = px(6.0F);
        const float card = (ImGui::GetContentRegionAvail().x - 4.0F * gap) / 5.0F;
        char cardText[48];
        value(r->rmse, r->unit, cardText, sizeof(cardText));
        metricCard("RMSE", cardText, c::t1, card);
        ImGui::SameLine(0.0F, gap);
        if (r->constantReference) {
            metricCard("NRMSE (range)", "const ref", r->pass ? c::t2 : c::error, card);
            ImGui::SetItemTooltip("The real signal is constant: no range to divide by.");
        } else {
            percent(r->nrmse, cardText, sizeof(cardText));
            metricCard("NRMSE (range)", cardText, verdictColor(r->nrmse, r->limit), card);
        }
        ImGui::SameLine(0.0F, gap);
        value(r->maxAbs, r->unit, cardText, sizeof(cardText));
        metricCard("max |error|", cardText, c::t1, card);
        ImGui::SameLine(0.0F, gap);
        std::snprintf(cardText, sizeof(cardText), "%.1f ms", static_cast<double>(r->lagNs) * 1e-6);
        // A lag at the end of the searched range says the runs do not line up, not a delay.
        const bool atLimit = suite.method.align && suite.method.maxLagNs > 0 &&
                             std::abs(r->lagNs) >= suite.method.maxLagNs - suite.periodNs;
        metricCard(atLimit ? "lag (at the search limit)" : "lag", cardText,
                   atLimit ? c::warn : c::t1, card);
        ImGui::SameLine(0.0F, gap);
        char caption[96];
        if (r->constantReference) {
            value(r->absLimit, r->unit, cardText, sizeof(cardText));
            std::snprintf(caption, sizeof(caption), "RMSE vs %s", cardText);
        } else {
            percent(r->limit, cardText, sizeof(cardText));
            std::snprintf(caption, sizeof(caption), "vs tolerance %s", cardText);
        }
        metricCard(caption, r->pass ? "pass" : "fail", r->pass ? c::ok : c::error, card);

        // Error by driving phase.
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
        ImGui::TextColored(c::t2, "Error by driving phase");
        ImGui::PopFont();
        percent(r->limit, cardText, sizeof(cardText));
        std::snprintf(caption, sizeof(caption), "NRMSE · limit %s", cardText);
        ImGui::SameLine();
        widgets::AlignRight(monoWidth(caption));
        widgets::Mono(c::t3, "%s", caption);
        bool anyPhase = false;
        for (int p = 0; p < static_cast<int>(cmp::Phase::Count) && !r->constantReference; ++p) {
            if (r->phaseSamples[p] == 0) {
                continue;
            }
            anyPhase = true;
            const double nrmse = r->phaseNrmse[p];
            ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
            ImGui::TextColored(c::t2, "%s", cmp::toString(static_cast<cmp::Phase>(p)));
            ImGui::PopFont();
            ImGui::SameLine(px(100.0F));
            const float meterWidth = ImGui::GetContentRegionAvail().x - px(64.0F);
            const float fraction = std::isfinite(nrmse)
                                       ? static_cast<float>(std::clamp(nrmse / r->limit, 0.0, 1.0))
                                       : 1.0F;
            widgets::Meter(fraction, verdictColor(nrmse, r->limit), meterWidth);
            ImGui::SameLine();
            percent(nrmse, cardText, sizeof(cardText));
            widgets::AlignRight(monoWidth(cardText));
            widgets::Mono(c::t1, "%s", cardText);
        }
        if (r->constantReference) {
            widgets::Caption(
                "The real signal is constant: there is no range to give an NRMSE "
                "by phase.");
        } else if (!anyPhase) {
            widgets::Caption("The replay has no body twist to tell the driving phases from.");
        }
        ImGui::End();
    }

private:
    std::shared_ptr<CompareState> state_;
    std::vector<std::string> listed_;
    double listedAt_{-1.0};
};

// --- Validation and History (right) --------------------------------------------------------

/// The ✓ / ✗ pill of a row: Check icon, or a cross, on the ok or error tint.
void verdictPill(bool pass, float x, float midY) {
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec4& color = pass ? c::ok : c::error;
    const ImVec2 min(x, midY - px(9.0F));
    const ImVec2 max(x + px(36.0F), midY + px(9.0F));
    draw->AddRectFilled(min, max, u32(color, 0.15F), px(9.0F));
    const float s = px(10.0F);
    const ImVec2 at(min.x + px(8.0F), midY - s * 0.5F);
    if (pass) {
        widgets::DrawIcon(draw, widgets::Icon::Check, at, s, u32(color), u32(c::bg1));
    } else {
        draw->AddLine(at, ImVec2(at.x + s * 0.8F, at.y + s * 0.8F), u32(color), px(1.5F));
        draw->AddLine(ImVec2(at.x + s * 0.8F, at.y), ImVec2(at.x, at.y + s * 0.8F), u32(color),
                      px(1.5F));
    }
}

class ValidationPanel final : public Panel {
public:
    explicit ValidationPanel(std::shared_ptr<CompareState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Validation"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Replay; }

    void draw(PanelContext& context) override {
        CompareState& state = *state_;
        update(context, state);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const float pad = px(10.0F);
        if (const char* why = blocker(context, state)) {
            // Below the tab bar (the cursor's start): at (pad, pad) its first line was hidden
            // under the tabs.
            ImGui::SetCursorPos(ImVec2(pad, ImGui::GetCursorStartPos().y + pad));
            ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
            widgets::Caption("%s", why);
            ImGui::PopTextWrapPos();
            ImGui::End();
            return;
        }
        const float footer = px(38.0F);
        ImGui::BeginChild("##validation", ImVec2(0.0F, -footer));
        if (!state.ready()) {
            ImGui::SetCursorPos(ImVec2(pad, pad));
            ImGui::BeginGroup();
            ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x - pad);
            widgets::Caption(
                "A suite compares the replay with a second recording (Compare, on "
                "the left). Without a suite file every signal both share is "
                "compared, with a limit of 5 %% NRMSE.");
            ImGui::PopTextWrapPos();
            workerState(state);
            ImGui::EndGroup();
        } else {
            drawSuite(context, state);
        }
        drawMethod(state);
        ImGui::EndChild();
        drawFooter(context, state);
        ImGui::End();
    }

private:
    void drawSuite(PanelContext& context, CompareState& state) {
        const Analysis& a = *state.analysis;
        const float pad = px(10.0F);
        char text[160];
        // Suite header: name, N / M pass, file and the real log.
        ImGui::SetCursorPos(ImVec2(pad, pad));
        ImGui::BeginGroup();
        ImGui::PushFont(theme::fonts().sansSemiBold, theme::kTitleSize);
        ImGui::TextUnformatted(a.suite.name.c_str());
        ImGui::PopFont();
        std::snprintf(text, sizeof(text), "%zu / %zu pass", a.report.passed(),
                      a.report.results.size());
        ImGui::SameLine();
        widgets::AlignRight(widgets::PillWidth(text) + pad);
        widgets::Pill(text, a.report.passed() == a.report.results.size() ? widgets::PillKind::Ok
                                                                         : widgets::PillKind::Warn);
        std::snprintf(
            text, sizeof(text), "suite %s · real log %s",
            a.suite.path.empty() ? "shared signals of both logs" : fileName(a.suite.path).c_str(),
            fileName(a.realPath).c_str());
        widgets::Caption("%s", text);
        ImGui::EndGroup();
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        rule();

        // The mapping table: sim ↔ real, NRMSE, limit, ✓/✗.
        const float width = ImGui::GetContentRegionAvail().x;
        const float colPill = width - pad - px(36.0F);
        const float colLimit = colPill - px(10.0F);
        const float colNrmse = colLimit - px(60.0F);
        {
            const ImVec2 min = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(min, ImVec2(min.x + width, min.y + px(24.0F)),
                                                      u32(c::bg2));
            const float y = min.y + (px(24.0F) - px(theme::kCaptionSize)) * 0.5F;
            ImFont* font = theme::fonts().sansMedium;
            const float size = px(theme::kCaptionSize);
            ImDrawList* draw = ImGui::GetWindowDrawList();
            draw->AddText(font, size, ImVec2(min.x + pad, y), u32(c::t3),
                          "sim signal ↔ real topic");
            const float nrmseWidth = font->CalcTextSizeA(size, FLT_MAX, 0.0F, "NRMSE").x;
            draw->AddText(font, size, ImVec2(min.x + colNrmse - nrmseWidth, y), u32(c::t3),
                          "NRMSE");
            const float limitWidth = font->CalcTextSizeA(size, FLT_MAX, 0.0F, "limit").x;
            draw->AddText(font, size, ImVec2(min.x + colLimit - limitWidth, y), u32(c::t3),
                          "limit");
            ImGui::Dummy(ImVec2(width, px(24.0F)));
        }
        const float rowHeight = px(26.0F);
        const auto& results = a.report.results;
        const float listHeight =
            std::min(static_cast<float>(results.size()) * rowHeight, px(26.0F * 10.0F));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        ImGui::BeginChild("##rows", ImVec2(0.0F, listHeight));
        const auto pairOf = [](const cmp::Result& r) {
            return r.sim == r.real ? r.sim : r.sim + " ↔ " + r.real;
        };
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(results.size()), rowHeight);
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                const cmp::Result& r = results[static_cast<std::size_t>(i)];
                const auto index = static_cast<std::size_t>(i);
                ImGui::PushID(i);
                const ImVec2 min = ImGui::GetCursorScreenPos();
                const float rowWidth = ImGui::GetContentRegionAvail().x;
                if (ImGui::InvisibleButton("##row", ImVec2(rowWidth, rowHeight))) {
                    state.selected = index;
                }
                ImDrawList* draw = ImGui::GetWindowDrawList();
                if (state.selected == index) {
                    draw->AddRectFilled(min, ImVec2(min.x + rowWidth, min.y + rowHeight),
                                        u32(c::accent, 0.10F));
                } else if (ImGui::IsItemHovered()) {
                    draw->AddRectFilled(min, ImVec2(min.x + rowWidth, min.y + rowHeight),
                                        u32(c::bg3));
                }
                draw->AddLine(ImVec2(min.x, min.y + rowHeight - 1.0F),
                              ImVec2(min.x + rowWidth, min.y + rowHeight - 1.0F), u32(c::line));
                ImFont* mono = theme::fonts().mono;
                const float size = px(theme::kCaptionSize);
                const float midY = min.y + rowHeight * 0.5F;
                const float textY = midY - size * 0.55F;
                // "sim ↔ real", cut before the NRMSE column's widest value ("const ref", QA3
                // F7), its last part whole; rows that would read the same are cut where they
                // differ (QA2 F10, as the Signals rows).
                std::string pair = pairOf(r);
                const float numberRoom = std::max(
                    px(54.0F), mono->CalcTextSizeA(size, FLT_MAX, 0.0F, "const ref").x + px(10.0F));
                // The NRMSE may move left of its column by up to numberShift for a wide limit
                // (a constant reference's RMSE limit in its unit, "0.163 N·m").
                const float numberShift = px(14.0F);
                const float room = colNrmse - numberRoom - numberShift - pad;
                if (state.rowsFor != state.analysis || state.rowsRoom != room) {
                    // Every row cut once for this analysis and width, no two alike.
                    std::vector<std::string> texts;
                    texts.reserve(results.size());
                    for (const cmp::Result& other : results) {
                        texts.push_back(pairOf(other));
                    }
                    state.rowTexts = fit::cutAll(mono, size, texts, room);
                    state.rowsFor = state.analysis;
                    state.rowsRoom = room;
                }
                if (index < state.rowTexts.size()) {
                    pair = state.rowTexts[index];
                }
                draw->AddText(mono, size, ImVec2(min.x + pad, textY), u32(c::t1), pair.c_str());
                if (ImGui::IsItemHovered()) {
                    if (r.valid && r.constantReference) {
                        char rmse[40];
                        char allowed[40];
                        value(r.rmse, r.unit, rmse, sizeof(rmse));
                        value(r.absLimit, r.unit, allowed, sizeof(allowed));
                        ImGui::SetTooltip(
                            "%s ↔ %s\nThe real signal is constant here, so the NRMSE (error over "
                            "its range) is undefined:\njudged by the RMSE, %s, against %s.",
                            r.sim.c_str(), r.real.c_str(), rmse, allowed);
                    } else {
                        ImGui::SetTooltip("%s ↔ %s", r.sim.c_str(), r.real.c_str());
                    }
                }
                char number[24];
                if (!r.valid) {
                    std::snprintf(number, sizeof(number), "no data");
                } else if (r.constantReference) {
                    std::snprintf(number, sizeof(number), "const ref");
                } else {
                    percent(r.nrmse, number, sizeof(number));
                }
                char limit[24];
                if (r.valid && r.constantReference) {
                    value(r.absLimit, r.unit, limit, sizeof(limit));  // an RMSE, in the unit
                } else {
                    percent(r.limit, limit, sizeof(limit));
                }
                const float lw = mono->CalcTextSizeA(size, FLT_MAX, 0.0F, limit).x;
                // Right-aligned on its column, or left of a limit that reaches into it (QA3 F7:
                // "const ref" ran into "0.163 N·m").
                const float numberRight =
                    std::max(std::min(colNrmse, colLimit - lw - px(8.0F)), colNrmse - numberShift);
                const float nw = mono->CalcTextSizeA(size, FLT_MAX, 0.0F, number).x;
                draw->AddText(mono, size, ImVec2(min.x + numberRight - nw, textY),
                              u32(r.valid && !r.pass ? c::error : c::t1), number);
                draw->AddText(mono, size, ImVec2(min.x + colLimit - lw, textY), u32(c::t3), limit);
                verdictPill(r.pass, min.x + colPill, midY);
                ImGui::PopID();
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();

        // The explanation card of the selected row when it fails (design: the failure card).
        if (const cmp::Result* r = state.selectedResult(); r != nullptr && r->valid && !r->pass) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            ImGui::SetCursorPosX(pad);
            ImGui::BeginGroup();
            widgets::IconItem(widgets::Icon::Warning, 14.0F, c::error);
            ImGui::SameLine(0.0F, px(6.0F));
            ImGui::PushFont(theme::fonts().sansMedium, theme::kBodySize);
            char nrmse[40];
            if (r->constantReference) {
                value(r->rmse, r->unit, nrmse, sizeof(nrmse));  // RMSE: no range to divide by
            } else {
                percent(r->nrmse, nrmse, sizeof(nrmse));
            }
            ImGui::Text("%s off by %s", r->sim.c_str(), nrmse);
            ImGui::PopFont();
            char when[24];
            format::clock(r->worstNs, when, sizeof(when));
            char simValue[40];
            char realValue[40];
            value(r->simAtWorst, r->unit, simValue, sizeof(simValue));
            value(r->realAtWorst, r->unit, realValue, sizeof(realValue));
            char limit[40];
            ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x - pad);
            ImFont* sans = theme::fonts().sans;
            if (r->constantReference) {
                value(r->absLimit, r->unit, limit, sizeof(limit));
                widgets::ParagraphF(sans, theme::kLabelSize, c::t2,
                                    "Largest error at %s: sim %s, real %s (lag %.1f ms). The real "
                                    "signal is constant, so the RMSE is held to %s instead of a "
                                    "share of its range.",
                                    when, simValue, realValue, static_cast<double>(r->lagNs) * 1e-6,
                                    limit);
            } else {
                percent(r->limit, limit, sizeof(limit));
                widgets::ParagraphF(
                    sans, theme::kLabelSize, c::t2,
                    "Largest error at %s: sim %s, real %s (lag %.1f ms). The limit is %s of the "
                    "real range.",
                    when, simValue, realValue, static_cast<double>(r->lagNs) * 1e-6, limit);
            }
            ImGui::PopTextWrapPos();
            const SignalInfo* info = context.catalog
                                         ? context.catalog->find(r->sim.substr(0, r->sim.find('.')))
                                         : nullptr;
            const bool hasPrim = info != nullptr && info->prim != kNone;
            ImGui::BeginDisabled(!hasPrim);
            if (widgets::Button("Open in Inspector")) {
                context.selectedPrim = info->prim;
                *context.workspace = Workspace::Scene;
            }
            ImGui::EndDisabled();
            if (!hasPrim) {
                ImGui::SetItemTooltip("The signal is about no prim of the scene");
            }
            ImGui::SameLine(0.0F, px(6.0F));
            if (widgets::Button("Plot both", widgets::ButtonKind::Ghost) &&
                context.focusPanel != nullptr) {
                *context.focusPanel = kSimVsRealTitle;
            }
            ImGui::SameLine(0.0F, px(6.0F));
            if (widgets::Button("Go to it", widgets::ButtonKind::Ghost) &&
                context.connection.canSeek) {
                context.client.seek(r->worstNs);
            }
            ImGui::EndGroup();
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            rule();
        }
    }

    void drawMethod(CompareState& state) {
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        if (!widgets::SectionHeader("Method", nullptr)) {
            return;
        }
        ImGui::Indent(px(8.0F));
        static constexpr const char* kAlignments[] = {"cross-correlation", "none"};
        if (widgets::DropdownRow("alignment", kAlignments[state.alignment])) {
            ImGui::OpenPopup("##alignment");
        }
        if (ImGui::BeginPopup("##alignment")) {
            for (int i = 0; i < 2; ++i) {
                if (ImGui::Selectable(kAlignments[i], state.alignment == i)) {
                    state.alignment = i;
                    state.methodEdited = true;
                }
            }
            ImGui::EndPopup();
        }
        if (widgets::PropertyEdit("resample to", state.resampleKhz, sizeof(state.resampleKhz),
                                  "kHz", false, "Both logs are averaged into bins of this rate")) {
            state.methodEdited = true;
        }
        widgets::DropdownRow("metric", "NRMSE · range");
        if (widgets::PropertyEdit("window", state.window, sizeof(state.window), "s", false,
                                  "whole run, or FROM-TO in seconds", false)) {
            state.methodEdited = true;
        }
        if (widgets::PropertyEdit("suite file", state.suitePath, sizeof(state.suitePath), nullptr,
                                  false, "validation/<suite>.yaml; empty: shared signals", false)) {
            state.methodEdited = false;  // a suite file brings its own method
        }
        ImGui::Unindent(px(8.0F));
    }

    void drawFooter(PanelContext& context, CompareState& state) {
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(min, ImVec2(min.x + width, min.y + px(38.0F)), u32(c::bg2));
        draw->AddLine(min, ImVec2(min.x + width, min.y), u32(c::line));
        ImGui::SetCursorScreenPos(ImVec2(min.x + px(8.0F), min.y + px(8.0F)));
        ImGui::AlignTextToFramePadding();
        // The design says "also runs nightly in CI"; nothing runs this suite in CI yet, so the
        // footer says where it runs.
        widgets::Caption(
            "%s", state.exported.empty() ? "computed on this machine" : state.exported.c_str());
        const float run = widgets::ButtonWidth("Run suite");
        const float exportWidth = widgets::ButtonWidth("Export report");
        ImGui::SameLine();
        widgets::AlignRight(run + exportWidth + px(6.0F + 8.0F));
        ImGui::BeginDisabled(!state.ready());
        if (widgets::Button("Export report")) {
            exportReport(context, state);
        }
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(state.busy() || state.realPath[0] == '\0' || !state.simReadable);
        if (widgets::Button("Run suite", widgets::ButtonKind::Primary)) {
            state.run(false);
        }
        ImGui::EndDisabled();
    }

    static void exportReport(PanelContext& context, CompareState& state) {
        const Analysis& a = *state.analysis;
        std::filesystem::path folder = context.configDirectory.empty()
                                           ? std::filesystem::temp_directory_path()
                                           : std::filesystem::path(context.configDirectory);
        folder /= "validation";
        std::error_code ec;
        std::filesystem::create_directories(folder, ec);
        std::string name = a.suite.name;
        std::replace_if(
            name.begin(), name.end(),
            [](char ch) {
                return !(std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-');
            },
            '_');
        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
        const std::filesystem::path file = folder / (name + "-" + stamp + ".json");
        std::ofstream out(file);
        out << cmp::toJson(a.report, a.suite);
        state.exported = out ? "wrote " + file.string() : "cannot write " + file.string();
    }

    static void rule() {
        const ImVec2 min = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(
            min, ImVec2(min.x + ImGui::GetContentRegionAvail().x, min.y), u32(c::line));
        ImGui::Dummy(ImVec2(0.0F, 1.0F));
    }

    std::shared_ptr<CompareState> state_;
};

class HistoryPanel final : public Panel {
public:
    explicit HistoryPanel(std::shared_ptr<CompareState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "History###ReplayHistory"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Replay; }

    void draw(PanelContext& context) override {
        CompareState& state = *state_;
        update(context, state);
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (state.history.empty()) {
            widgets::Caption("No suite has run in this session.");
            ImGui::End();
            return;
        }
        for (auto it = state.history.rbegin(); it != state.history.rend(); ++it) {
            widgets::Mono(c::t3, "%s", it->when.c_str());
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::TextUnformatted(it->suite.c_str());
            char text[48];
            std::snprintf(text, sizeof(text), "%zu / %zu", it->passed, it->total);
            ImGui::SameLine();
            widgets::AlignRight(widgets::PillWidth(text));
            widgets::Pill(
                text, it->passed == it->total ? widgets::PillKind::Ok : widgets::PillKind::Warn);
            widgets::Mono(c::t2, "  %s", it->real.c_str());
        }
        ImGui::End();
    }

private:
    std::shared_ptr<CompareState> state_;
};

}  // namespace

void setReplayCompare(const std::string& realLog, const std::string& suite, bool blocking) {
    pending() = {realLog, suite, blocking};
}

std::string replayCompareJson() {
    const CompareState& state = *sharedState();
    if (!state.analysis) {
        return state.busy() ? "{\"state\": \"busy\"}" : "{\"state\": \"none\"}";
    }
    const Analysis& a = *state.analysis;
    if (!a.error.empty()) {
        std::string error;
        for (const char ch : a.error) {
            error += ch == '"' || ch == '\\' ? std::string("\\") + ch : std::string(1, ch);
        }
        return "{\"state\": \"error\", \"error\": \"" + error + "\"}";
    }
    char text[512];
    const cmp::Result* r = state.selectedResult();
    std::snprintf(text, sizeof(text),
                  "{\"state\": \"ready\", \"suite\": \"%s\", \"passed\": %zu, \"total\": %zu, "
                  "\"unmapped\": %zu, \"selected\": \"%s\", \"nrmse\": %.6g, \"lag_s\": %.6g}",
                  a.suite.name.c_str(), a.report.passed(), a.report.results.size(),
                  a.unmapped.size(), r != nullptr ? r->sim.c_str() : "",
                  r != nullptr && std::isfinite(r->nrmse) ? r->nrmse : -1.0,
                  r != nullptr ? static_cast<double>(r->lagNs) * 1e-9 : 0.0);
    return text;
}

bool drawReplaySourceStrip(PanelContext& context) {
    const std::shared_ptr<const RecordingFacts> facts = context.client.recording();
    if (!context.connection.canSeek || !facts) {
        return false;  // a live run: the domain strip
    }
    const auto centre = [](float height) {
        ImGui::SetCursorPosY((ImGui::GetWindowHeight() - height) * 0.5F);
    };
    centre(ImGui::GetTextLineHeight());
    widgets::Caption("Source");
    // Each physics domain reads the recording (its replay substitute) or was not recorded.
    for (const DomainView& view : context.profileSession->domains()) {
        if (!format::isPhysicsDomain(view.name)) {
            continue;
        }
        const bool replayed = !view.enabled && view.mode == "replay";
        const std::string_view label = widgets::domainShortLabel(view.name);
        char name[32];
        std::snprintf(name, sizeof(name), "%.*s", static_cast<int>(label.size()), label.data());
        ImGui::SameLine(0.0F, px(6.0F));
        centre(px(22.0F));
        if (widgets::DomainChip(
                view.name.c_str(), widgets::domainColor(view.name), name,
                replayed ? "replay" : "not recorded",
                replayed ? widgets::ChipState::Substitute : widgets::ChipState::Off)) {
            *context.workspace = Workspace::Domains;
            context.profileSession->selectedDomain = view.name;
        }
    }
    // On the right: what wrote the recording, and Re-simulate from here.
    char text[200];
    std::snprintf(text, sizeof(text), "recorded with %s",
                  facts->generator.empty() ? "an unknown program" : facts->generator.c_str());
    const char* button = "Re-simulate from here…";
    const float width = widgets::ButtonWidth(button);
    ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
    const float textWidth = ImGui::CalcTextSize(text).x;
    ImGui::PopFont();
    ImGui::SameLine();
    widgets::AlignRight(textWidth + px(8.0F) + width);
    centre(ImGui::GetTextLineHeight());
    widgets::Caption("%s", text);
    ImGui::SameLine(0.0F, px(8.0F));
    centre(px(22.0F));
    // A live run seeded from a recorded time needs the server to start from that state: the
    // protocol has no command for it yet (Resimulate, proposed for owner approval).
    ImGui::BeginDisabled(true);
    widgets::Button(button);
    ImGui::EndDisabled();
    ImGui::SetItemTooltip(
        "Starting a live simulation from this time needs a protocol addition (proposed: "
        "Command.resimulate with the time and the profile); not in protocol v1 yet.");
    return true;
}

std::vector<std::unique_ptr<Panel>> createReplayComparePanels() {
    std::shared_ptr<CompareState> state = sharedState();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<ComparePanel>(state));
    panels.push_back(std::make_unique<SimVsRealPanel>(state));
    panels.push_back(std::make_unique<ValidationPanel>(state));
    panels.push_back(std::make_unique<HistoryPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
