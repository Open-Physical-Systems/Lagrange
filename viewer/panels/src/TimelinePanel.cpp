// Scene workspace, bottom left: transport controls and the run's time track
// (docs/design/ui/02-scene/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr double kSeekIntervalSeconds = 1.0 / 30.0;  // while dragging the playhead
constexpr float kGutter = 64.0F;                     // row labels, left of the track
// Where the playhead sits in a moving window, as a fraction of its width (as in the design).
constexpr double kPlayheadFraction = 0.6;

struct Choice {
    const char* label;
    double value;
};
constexpr std::array<Choice, 3> kSteps{{{"1 ms", 1e-3}, {"10 ms", 1e-2}, {"100 ms", 1e-1}}};
constexpr std::array<Choice, 6> kSpeeds{
    {{"0.1x", 0.1}, {"0.25x", 0.25}, {"0.5x", 0.5}, {"1x", 1.0}, {"2x", 2.0}, {"4x", 4.0}}};
constexpr std::array<const char*, 4> kWindows{"5 s", "10 s", "60 s", "all"};
constexpr std::array<double, 4> kWindowSeconds{5.0, 10.0, 60.0, 0.0};

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// A tick spacing of 1, 2 or 5 times a power of ten that gives about `target` ticks.
double tickSpacing(double span, double target) {
    const double raw = span / std::max(target, 1.0);
    const double magnitude = std::pow(10.0, std::floor(std::log10(std::max(raw, 1e-9))));
    const double scaled = raw / magnitude;
    return (scaled <= 1.0 ? 1.0 : (scaled <= 2.0 ? 2.0 : (scaled <= 5.0 ? 5.0 : 10.0))) * magnitude;
}

class TimelinePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Timeline"; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const bool ready = context.connection.state == ConnectionState::Ready;
        const double begin = static_cast<double>(context.status.replayBeginNs) * 1e-9;
        const double end = static_cast<double>(context.status.replayEndNs) * 1e-9;
        // While the user drags the playhead it shows their time, not the server's.
        const double now =
            dragging_ ? dragTime_ : static_cast<double>(context.status.simTimeNs) * 1e-9;

        ImGui::BeginDisabled(!ready);
        transport(context, begin, now, end > begin);
        ImGui::EndDisabled();
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        if (end <= begin) {
            widgets::Caption("%s", ready ? "This run has no recorded range to show."
                                         : "Not connected. The timeline shows the last run "
                                           "when a server is reachable.");
        } else {
            track(context, begin, end, now, ready);
        }
        ImGui::End();
    }

private:
    void transport(PanelContext& context, double begin, double now, bool replay) {
        const bool running = context.status.runState == RunState::Running;
        ImGui::BeginDisabled(!context.connection.canSeek);
        if (widgets::Button("##start", widgets::ButtonKind::Default, widgets::Icon::JumpStart)) {
            context.client.seek(static_cast<std::int64_t>(begin * 1e9));
        }
        ImGui::SetItemTooltip("Jump to the start");
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, px(4.0F));
        if (widgets::Button("##play", widgets::ButtonKind::Primary,
                            running ? widgets::Icon::Pause : widgets::Icon::Play)) {
            if (running) {
                context.client.pause();
            } else {
                context.client.play();
            }
        }
        ImGui::SameLine(0.0F, px(4.0F));
        ImGui::BeginDisabled(!context.connection.canStep || running);
        if (widgets::Button("##step", widgets::ButtonKind::Default, widgets::Icon::Step)) {
            context.client.step(static_cast<std::int64_t>(kSteps[step_].value * 1e9));
        }
        ImGui::EndDisabled();

        ImGui::SameLine(0.0F, px(12.0F));
        widgets::Mono(c::t1, "%.3f s", now);
        ImGui::SameLine(0.0F, px(10.0F));
        // The mock server replays a recording; a simulation server will report live runs.
        widgets::Pill(replay ? "REPLAY" : "LIVE",
                      replay ? widgets::PillKind::Info : widgets::PillKind::Ok);

        ImGui::SameLine(0.0F, px(10.0F));
        ImGui::TextDisabled("step");
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(px(74.0F));
        if (ImGui::BeginCombo("##step", kSteps[step_].label)) {
            for (std::size_t i = 0; i < kSteps.size(); ++i) {
                if (ImGui::Selectable(kSteps[i].label, i == step_)) {
                    step_ = i;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopFont();

        ImGui::SameLine(0.0F, px(10.0F));
        ImGui::TextDisabled("speed");
        ImGui::SameLine(0.0F, px(6.0F));
        char speed[24];
        std::snprintf(speed, sizeof(speed), "%gx", context.status.realTimeFactorTarget);
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(px(64.0F));
        if (ImGui::BeginCombo("##speed", speed)) {
            for (const Choice& choice : kSpeeds) {
                if (ImGui::Selectable(choice.label,
                                      choice.value == context.status.realTimeFactorTarget)) {
                    context.client.setRealTimeFactor(choice.value);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopFont();

        // The time window of the track. It moves to its own line when the panel is narrow.
        ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
        float needed = 0.0F;
        for (const char* option : kWindows) {
            needed += ImGui::CalcTextSize(option).x + px(16.0F);
        }
        ImGui::PopFont();
        ImGui::SameLine(0.0F, px(10.0F));
        if (ImGui::GetContentRegionAvail().x < needed) {
            ImGui::NewLine();
        }
        widgets::Segmented("##window", kWindows.data(), static_cast<int>(kWindows.size()),
                           &window_);
    }

    void track(PanelContext& context, double begin, double end, double now, bool ready) {
        // The visible time window: the whole run, or a window that follows the playhead.
        double from = begin;
        double to = end;
        const double width = kWindowSeconds[static_cast<std::size_t>(window_)];
        if (width > 0.0 && width < end - begin) {
            from = std::clamp(now - width * kPlayheadFraction, begin, end - width);
            to = from + width;
        }

        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size(ImGui::GetContentRegionAvail().x,
                          std::max(ImGui::GetContentRegionAvail().y, px(56.0F)));
        ImGui::InvisibleButton("##track", size);
        const float left = origin.x + px(kGutter);
        const float right = origin.x + size.x - px(6.0F);
        const float span = std::max(right - left, 1.0F);
        const auto toX = [&](double t) {
            return left + static_cast<float>((t - from) / (to - from)) * span;
        };

        // Dragging anywhere on the track moves the playhead; the server follows at 30 Hz.
        if (ready && context.connection.canSeek && ImGui::IsItemActive()) {
            const double fraction =
                std::clamp(static_cast<double>(ImGui::GetIO().MousePos.x - left) / span, 0.0, 1.0);
            dragTime_ = from + fraction * (to - from);
            dragging_ = true;
            const double clock = ImGui::GetTime();
            if (clock - lastSeek_ >= kSeekIntervalSeconds) {
                context.client.seek(static_cast<std::int64_t>(dragTime_ * 1e9));
                lastSeek_ = clock;
            }
        } else if (dragging_) {
            context.client.seek(static_cast<std::int64_t>(dragTime_ * 1e9));
            dragging_ = false;
        }

        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImFont* mono = theme::fonts().mono;
        ImFont* sans = theme::fonts().sans;
        const float small = px(theme::kCaptionSize);
        const float rulerY = origin.y + px(16.0F);

        // Ruler.
        draw->AddLine(ImVec2(left, rulerY), ImVec2(right, rulerY), u32(c::lineStrong));
        const double spacing = tickSpacing(to - from, static_cast<double>(span) / px(70.0F));
        for (double t = std::ceil(from / spacing) * spacing; t <= to + 1e-9; t += spacing) {
            const float x = toX(t);
            draw->AddLine(ImVec2(x, rulerY - px(4.0F)), ImVec2(x, rulerY), u32(c::lineStrong));
            char label[24];
            std::snprintf(label, sizeof(label), spacing < 1.0 ? "%.1f" : "%.0f", t);
            draw->AddText(mono, small, ImVec2(x + px(3.0F), origin.y), u32(c::t3), label);
        }

        // The run: what is recorded and can be shown again.
        const float rowTop = rulerY + px(8.0F);
        const float rowBottom = rowTop + px(20.0F);
        draw->AddText(sans, px(theme::kLabelSize), ImVec2(origin.x + px(2.0F), rowTop + px(3.0F)),
                      u32(c::t3), "run");
        const float recordedFrom = std::max(toX(begin), left);
        const float recordedTo = std::min(toX(end), right);
        draw->AddRectFilled(ImVec2(recordedFrom, rowTop), ImVec2(recordedTo, rowBottom),
                            u32(c::bg3), px(2.0F));
        char recorded[48];
        std::snprintf(recorded, sizeof(recorded), "recorded %.1f s", end - begin);
        draw->PushClipRect(ImVec2(recordedFrom, rowTop), ImVec2(recordedTo, rowBottom), true);
        draw->AddText(sans, small, ImVec2(recordedFrom + px(8.0F), rowTop + px(4.0F)), u32(c::t2),
                      recorded);
        draw->PopClipRect();

        // Event lanes: one per domain that has events, in the domains' usual order. Spans are
        // bars, instants are marks; errors are red whatever the domain.
        const std::shared_ptr<const EventLog> log = context.client.events();
        static constexpr std::array<std::string_view, 10> kLaneOrder{
            "rigid_body", "electrical", "thermal", "aerodynamics", "weather",
            "emi",        "comms",      "sensors", "radiation",    "traffic"};
        float laneTop = rowBottom + px(6.0F);
        const float laneHeight = px(16.0F);
        const EventInfo* hoveredEvent = nullptr;
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        for (const std::string_view domain : kLaneOrder) {
            bool any = false;
            for (const EventInfo& event : log->events) {
                if (event.domain != domain) {
                    continue;
                }
                if (!any) {
                    any = true;
                    if (laneTop + laneHeight > origin.y + size.y) {
                        break;  // no room for another lane in this panel
                    }
                    const std::string_view name = widgets::domainShortLabel(domain);
                    draw->AddText(sans, small, ImVec2(origin.x + px(2.0F), laneTop + px(2.0F)),
                                  u32(c::t3), name.data(), name.data() + name.size());
                }
                const double b = static_cast<double>(event.beginNs) * 1e-9;
                const double e = static_cast<double>(event.endNs) * 1e-9;
                if (e < from || b > to) {
                    continue;
                }
                const ImVec4 color = event.severity == EventInfo::Severity::Error
                                         ? c::error
                                         : widgets::domainColor(domain);
                const float x0 = std::max(toX(b), left);
                const float x1 = std::max(std::min(toX(e), right), x0 + px(3.0F));
                const ImVec2 min(x0, laneTop + px(2.0F));
                const ImVec2 max(x1, laneTop + laneHeight - px(2.0F));
                draw->AddRectFilled(min, max, u32(color), px(2.0F));
                if (mouse.x >= min.x - px(2.0F) && mouse.x <= max.x + px(2.0F) &&
                    mouse.y >= laneTop && mouse.y <= laneTop + laneHeight) {
                    hoveredEvent = &event;
                }
            }
            if (any) {
                laneTop += laneHeight;
            }
        }
        if (hoveredEvent != nullptr && ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s · %s\n%.3f s to %.3f s", hoveredEvent->kind.c_str(),
                              hoveredEvent->text.c_str(),
                              static_cast<double>(hoveredEvent->beginNs) * 1e-9,
                              static_cast<double>(hoveredEvent->endNs) * 1e-9);
        }

        // Bookmarks: small flags on the ruler.
        for (const std::int64_t bookmark : *context.bookmarks) {
            const double t = static_cast<double>(bookmark) * 1e-9;
            if (t < from || t > to) {
                continue;
            }
            const float x = toX(t);
            draw->AddLine(ImVec2(x, rulerY - px(8.0F)), ImVec2(x, rowBottom), u32(c::info));
            draw->AddTriangleFilled(ImVec2(x, rulerY - px(8.0F)),
                                    ImVec2(x + px(6.0F), rulerY - px(5.0F)),
                                    ImVec2(x, rulerY - px(2.0F)), u32(c::info));
        }

        // Playhead: an accent line with a triangle head on the ruler.
        if (now >= from && now <= to) {
            const float x = toX(now);
            draw->AddLine(ImVec2(x, rulerY), ImVec2(x, origin.y + size.y), u32(c::accent),
                          px(1.5F));
            draw->AddTriangleFilled(ImVec2(x - px(5.0F), rulerY - px(7.0F)),
                                    ImVec2(x + px(5.0F), rulerY - px(7.0F)), ImVec2(x, rulerY),
                                    u32(c::accent));
        }
    }

    std::size_t step_{1};  // 10 ms
    int window_{3};        // all
    bool dragging_{false};
    double dragTime_{0.0};
    double lastSeek_{0.0};
};

/// Times the user marked, next to the timeline. Kept for the session only.
class BookmarksPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Bookmarks"; }

    void draw(PanelContext& context) override {
        std::vector<std::int64_t>& bookmarks = *context.bookmarks;
        char label[48];
        std::snprintf(label, sizeof(label), "Bookmarks · %zu###Bookmarks", bookmarks.size());
        if (!ImGui::Begin(label)) {
            ImGui::End();
            return;
        }
        const bool ready = context.connection.state == ConnectionState::Ready;
        char add[48];
        std::snprintf(add, sizeof(add), "Add at %.3f s",
                      static_cast<double>(context.status.simTimeNs) * 1e-9);
        ImGui::BeginDisabled(!ready);
        if (widgets::Button(add, widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            const std::int64_t now = context.status.simTimeNs;
            const auto at = std::lower_bound(bookmarks.begin(), bookmarks.end(), now);
            if (at == bookmarks.end() || *at != now) {
                bookmarks.insert(at, now);
            }
        }
        ImGui::EndDisabled();
        if (bookmarks.empty()) {
            ImGui::SameLine(0.0F, px(10.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Caption("No bookmarks. Add one at the current time.");
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        std::size_t remove = bookmarks.size();
        for (std::size_t i = 0; i < bookmarks.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            char time[32];
            std::snprintf(time, sizeof(time), "%.3f s", static_cast<double>(bookmarks[i]) * 1e-9);
            ImGui::BeginDisabled(!ready || !context.connection.canSeek);
            if (widgets::Button("Go to", widgets::ButtonKind::Ghost)) {
                context.client.seek(bookmarks[i]);
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Mono(c::t1, "%s", time);
            ImGui::SameLine();
            widgets::AlignRight(widgets::ButtonWidth("Remove"));
            if (widgets::Button("Remove", widgets::ButtonKind::Ghost)) {
                remove = i;
            }
            ImGui::PopID();
        }
        if (remove < bookmarks.size()) {
            bookmarks.erase(bookmarks.begin() + static_cast<std::ptrdiff_t>(remove));
        }
        ImGui::End();
    }
};

}  // namespace

std::unique_ptr<Panel> createBookmarksPanel() {
    return std::make_unique<BookmarksPanel>();
}

std::unique_ptr<Panel> createTimelinePanel() {
    return std::make_unique<TimelinePanel>();
}

}  // namespace lagrange::viewer
