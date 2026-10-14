// Scene workspace, bottom right: the server's log (Console) and its warnings and errors
// (Problems) (docs/design/ui/02-scene/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <array>
#include <cfloat>
#include <cstdio>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

enum class Filter : int { All = 0, Info = 1, Warn = 2, Error = 3 };

bool passes(const LogEntry& entry, Filter filter, bool problemsOnly) {
    if (problemsOnly && entry.level < LogLevel::Warn) {
        return false;
    }
    switch (filter) {
        case Filter::Info:
            return entry.level <= LogLevel::Info;
        case Filter::Warn:
            return entry.level == LogLevel::Warn;
        case Filter::Error:
            return entry.level == LogLevel::Error;
        case Filter::All:
            break;
    }
    return true;
}

class ConsolePanel final : public Panel {
public:
    /// `window` names the window when the console is used outside the Scene workspace.
    explicit ConsolePanel(bool problemsOnly, const char* window = nullptr,
                          Workspace workspace = Workspace::Scene)
        : problemsOnly_(problemsOnly), window_(window), workspace_(workspace) {}
    [[nodiscard]] Workspace workspace() const override { return workspace_; }

    [[nodiscard]] const char* title() const override {
        return problemsOnly_ ? "Problems" : "Console";
    }

    void draw(PanelContext& context) override {
        const std::shared_ptr<const LogBook> book = context.client.logs();
        char label[48];
        if (problemsOnly_) {
            // Design: "Problems · 2". The part after ### keeps the window's identity.
            std::snprintf(label, sizeof(label), "Problems · %u###Problems",
                          book->warnings + book->errors);
        } else {
            std::snprintf(label, sizeof(label), "%s", window_ != nullptr ? window_ : "Console");
        }
        if (!ImGui::Begin(label)) {
            ImGui::End();
            return;
        }
        if (!problemsOnly_) {
            // Level filter, right-aligned, with the counts the design shows.
            char warn[24];
            char error[24];
            std::snprintf(warn, sizeof(warn), "Warn %u", book->warnings);
            std::snprintf(error, sizeof(error), "Error %u", book->errors);
            const std::array<const char*, 4> options{"All", "Info", warn, error};
            int index = static_cast<int>(filter_);
            if (widgets::Segmented("##level", options.data(), 4, &index)) {
                filter_ = static_cast<Filter>(index);
                shown_ = nullptr;
            }
        }
        if (book != shown_) {
            // New lines or a new filter: rebuild the list of visible lines once.
            visible_.clear();
            for (std::uint32_t i = 0; i < book->entries.size(); ++i) {
                if (passes(book->entries[i], filter_, problemsOnly_)) {
                    visible_.push_back(i);
                }
            }
            shown_ = book;
        }

        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##lines", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Borders);
        ImGui::PopStyleColor();
        if (visible_.empty()) {
            widgets::Caption("%s", problemsOnly_
                                       ? "No warnings or errors."
                                       : (book->entries.empty() ? "No log lines yet."
                                                                : "No log line at this level."));
        }
        const bool atBottom = ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - px(4.0F);
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(visible_.size()));
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                line(book->entries[visible_[static_cast<std::size_t>(i)]]);
            }
        }
        ImGui::PopFont();
        if (atBottom && book->total != followed_) {
            ImGui::SetScrollHereY(1.0F);  // keep following the newest line
        }
        followed_ = book->total;
        ImGui::EndChild();
        ImGui::End();
    }

private:
    static void line(const LogEntry& entry) {
        char time[24];
        format::clock(entry.simTimeNs, time, sizeof(time));
        const char* level = "INFO ";
        ImVec4 color = c::info;
        switch (entry.level) {
            case LogLevel::Debug:
                level = "DEBUG";
                color = c::t3;
                break;
            case LogLevel::Warn:
                level = "WARN ";
                color = c::warn;
                break;
            case LogLevel::Error:
                level = "ERROR";
                color = c::error;
                break;
            case LogLevel::Info:
                break;
        }
        ImGui::TextColored(c::t3, "%s", time);
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::TextColored(color, "%s", level);
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::TextColored(c::t2, "%s", entry.source.c_str());
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::TextUnformatted(entry.text.c_str());
    }

    bool problemsOnly_;
    const char* window_;
    Workspace workspace_;
    Filter filter_{Filter::All};
    std::shared_ptr<const LogBook> shown_;
    std::vector<std::uint32_t> visible_;
    std::uint64_t followed_{0};
};

}  // namespace

std::unique_ptr<Panel> createConsolePanel() {
    return std::make_unique<ConsolePanel>(false);
}

std::unique_ptr<Panel> createRosConsolePanel() {
    return std::make_unique<ConsolePanel>(false, "Console###RosConsole", Workspace::Ros2);
}

std::unique_ptr<Panel> createProblemsPanel() {
    return std::make_unique<ConsolePanel>(true);
}

}  // namespace lagrange::viewer
