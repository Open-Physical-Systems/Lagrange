// The Builder's Problems tab (docs/design/ui/12-builder/SPEC.md "Problems"): the builder's own
// validation (RFC 0016 §8.1), and after a Test the loader's messages and `lagrange check`.
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <cfloat>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using builder::Level;
using builder::Problem;
using widgets::px;

widgets::PillKind pillOf(Level level) {
    switch (level) {
        case Level::Error:
            return widgets::PillKind::Error;
        case Level::Warning:
            return widgets::PillKind::Warn;
        case Level::Hint:
            return widgets::PillKind::Info;
    }
    return widgets::PillKind::Neutral;
}

class ProblemsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return title_.c_str(); }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        BuilderSession& session = *context.builder;
        const BuilderSession::Problems& problems = session.problems;
        const std::size_t count = problems.builder.size() + problems.loader.size() +
                                  problems.check.size() + problems.files.size() +
                                  (session.catalogError.empty() ? 0 : 1);
        const int errors = problems.errors() + (session.catalogError.empty() ? 0 : 1);
        title_ = "Problems · " + std::to_string(count) + "###BuilderProblems";
        // State 3 (Invalid): Problems comes to the front once when errors appear, not on every
        // edit.
        if (errors > 0 && shownErrors_ == 0) {
            ImGui::SetNextWindowFocus();
        }
        shownErrors_ = errors;
        if (!ImGui::Begin(title_.c_str())) {
            ImGui::End();
            return;
        }
        if (count == 0) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            widgets::Pill("no problems", widgets::PillKind::Ok);
            ImGui::SameLine();
            widgets::Caption("the builder's checks found nothing");
        }
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(px(8.0F), 0.0F));
        if (!session.catalogError.empty()) {
            section("Sensor catalog", 1);
            row(session,
                Problem{Level::Error, "", "the catalog was not read: " + session.catalogError});
        }
        // The newest outcomes first (WP-F): a failed Test, Check loads, a file; then the
        // builder's own findings, which also mark the tree.
        list(session, "Loader · after Test", problems.loader);
        list(session, "Check loads", problems.check);
        if (!problems.checkTable.empty()) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Mono(c::t2, "%s", problems.checkTable.c_str());
            ImGui::PopTextWrapPos();
        }
        list(session, "Files", problems.files);
        list(session, "Builder", problems.builder);
        ImGui::PopStyleVar();
        ImGui::End();
    }

private:
    void list(BuilderSession& session, const char* heading, const std::vector<Problem>& items) {
        if (items.empty()) {
            return;
        }
        if (!section(heading, items.size())) {
            return;
        }
        for (std::size_t i = 0; i < items.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            row(session, items[i]);
            ImGui::PopID();
        }
    }

    static bool section(const char* heading, std::size_t count) {
        const std::string meta = std::to_string(count);
        return widgets::SectionHeader(heading, meta.c_str());
    }

    /// level (pill) · the item (mono, a link that selects it) · the message.
    static void row(BuilderSession& session, const Problem& problem) {
        ImGui::Dummy(ImVec2(0.0F, px(3.0F)));
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        const std::string level(builder::toString(problem.level));
        widgets::Pill(level.c_str(), pillOf(problem.level));
        ImGui::SameLine(px(80.0F));
        const char* item =
            problem.item.empty() ? session.document().robot.c_str() : problem.item.c_str();
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::PushStyleColor(ImGuiCol_Text, c::info);
        ImGui::TextUnformatted(item);
        ImGui::PopStyleColor();
        ImGui::PopFont();
        if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }
        if (ImGui::IsItemClicked()) {
            (void)selectByName(session, problem.item);
        }
        ImGui::SameLine();
        ImGui::PushTextWrapPos(0.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        ImGui::TextUnformatted(problem.message.c_str());
        ImGui::PopStyleColor();
        ImGui::PopTextWrapPos();
    }

    std::string title_ = "Problems###BuilderProblems";
    int shownErrors_{0};
};

}  // namespace

std::unique_ptr<Panel> createProblemsPanel() {
    return std::make_unique<ProblemsPanel>();
}

}  // namespace lagrange::viewer::builder_ui
