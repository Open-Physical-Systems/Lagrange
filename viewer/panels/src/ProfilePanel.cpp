// Domains workspace, right panel: the profile as YAML with the pending changes marked, the
// server's verdict on it, and Apply (docs/design/ui/03-domains/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <cfloat>
#include <charconv>
#include <cstdio>
#include <string_view>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr float kLineHeight = 18.0F;  // design: mono 12 on an 18 px line
constexpr float kGutter = 44.0F;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

ImVec4 withAlpha(ImVec4 color, float alpha) {
    color.w = alpha;
    return color;
}

bool isNumber(std::string_view word) {
    double value = 0.0;
    const auto [end, error] = std::from_chars(word.data(), word.data() + word.size(), value);
    return error == std::errc{} && end == word.data() + word.size();
}

/// Draws one line of profile text with the design's syntax colours: keys, strings, numbers,
/// booleans, comments. A lexer for the canonical layout, not a YAML parser.
void drawYaml(ImDrawList* draw, ImVec2 at, std::string_view line) {
    ImFont* font = theme::fonts().mono;
    const float size = px(theme::kMonoSize);
    const auto emit = [&](std::string_view text, const ImVec4& color) {
        const char* begin = text.data();
        const char* end = begin + text.size();
        draw->AddText(font, size, at, u32(color), begin, end);
        at.x += font->CalcTextSizeA(size, FLT_MAX, 0.0F, begin, end).x;
    };
    const auto isBreak = [&](std::size_t i) {
        const char ch = line[i];
        if (ch == ':') {  // a colon ends a word only as "key: " (not in "tcp://host")
            return i + 1 == line.size() || line[i + 1] == ' ';
        }
        return ch == ' ' || ch == ',' || ch == '{' || ch == '}' || ch == '[' || ch == ']';
    };
    std::size_t i = 0;
    while (i < line.size()) {
        const char ch = line[i];
        if (ch == '#') {
            emit(line.substr(i), c::t3);
            return;
        }
        if (ch == '"' || ch == '\'') {
            std::size_t end = line.find(ch, i + 1);
            end = end == std::string_view::npos ? line.size() : end + 1;
            emit(line.substr(i, end - i), c::ok);
            i = end;
            continue;
        }
        if (isBreak(i)) {
            std::size_t end = i + 1;
            while (end < line.size() && isBreak(end) && line[end] != '#') {
                ++end;
            }
            emit(line.substr(i, end - i), c::t2);
            i = end;
            continue;
        }
        std::size_t end = i;
        while (end < line.size() && !isBreak(end)) {
            ++end;
        }
        const std::string_view word = line.substr(i, end - i);
        const bool key = end < line.size() && line[end] == ':';
        if (key) {
            emit(word, c::info);
        } else if (word == "true" || word == "false" || word == "null") {
            emit(word, c::accent);
        } else if (isNumber(word)) {
            emit(word, c::warn);
        } else {
            emit(word, c::ok);
        }
        i = end;
    }
}

enum class LineKind { Same, Removed, Added };

void drawLine(std::string_view text, int number, LineKind kind) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, px(kLineHeight));
    ImGui::Dummy(size);
    if (!ImGui::IsItemVisible()) {
        return;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (kind != LineKind::Same) {
        const ImVec4 tint = kind == LineKind::Removed ? c::error : c::ok;
        draw->AddRectFilled(min, ImVec2(min.x + size.x, min.y + size.y),
                            u32(withAlpha(tint, 0.12F)));
    }
    ImFont* font = theme::fonts().mono;
    const float fontSize = px(theme::kMonoSize);
    const float y = min.y + (size.y - fontSize) * 0.5F;
    char gutter[16];
    if (kind == LineKind::Removed) {
        std::snprintf(gutter, sizeof(gutter), "-");
    } else {
        std::snprintf(gutter, sizeof(gutter), "%d", number);
    }
    const float width = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0F, gutter).x;
    draw->AddText(font, fontSize, ImVec2(min.x + px(kGutter - 14.0F) - width, y),
                  u32(kind == LineKind::Removed ? c::error : c::t3), gutter);
    drawYaml(draw, ImVec2(min.x + px(kGutter), y), text);
}

class ProfilePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Profile"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Domains; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        ProfileSession& session = *context.profileSession;
        const std::shared_ptr<const ProfileInfo>& profile = session.profile();
        const bool connected = context.connection.state == ConnectionState::Ready;
        if (!profile) {
            widgets::Caption("%s", connected ? "Waiting for the server's profile ..."
                                             : "Not connected. The profile appears when a "
                                               "simulation server is reachable.");
            ImGui::End();
            return;
        }

        header(session, *profile);
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0F + px(6.0F);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##yaml", ImVec2(0.0F, -footer), ImGuiChildFlags_Borders);
        ImGui::PopStyleColor();
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        const ProfileDraft& draft = session.draft();
        for (std::size_t i = 0; i < draft.lines().size(); ++i) {
            const int number = static_cast<int>(i) + 1;
            if (draft.lineChanged(i)) {
                if (const std::string* before = draft.appliedLine(i)) {
                    drawLine(*before, number, LineKind::Removed);
                }
                drawLine(draft.lines()[i], number, LineKind::Added);
            } else {
                drawLine(draft.lines()[i], number, LineKind::Same);
            }
        }
        ImGui::PopStyleVar();
        issues(session);
        ImGui::EndChild();

        // Footer: what Apply does, then the actions. Apply is the one primary action here.
        widgets::Dot(c::warn, false);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Applying rebuilds the wiring and restarts the run at t = 0.");
        ImGui::BeginDisabled(!draft.dirty());
        if (ImGui::Button("Revert")) {
            session.revert();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        const float width =
            ImGui::CalcTextSize("Apply and restart").x + ImGui::GetStyle().FramePadding.x * 2.0F;
        widgets::AlignRight(width);
        ImGui::BeginDisabled(!connected || !session.canApply());
        if (widgets::PrimaryButton("Apply and restart")) {
            session.apply(context.client);
        }
        ImGui::EndDisabled();
        ImGui::End();
    }

private:
    static void header(const ProfileSession& session, const ProfileInfo& profile) {
        ImGui::AlignTextToFramePadding();
        const std::size_t changes = session.draft().changedLineCount();
        if (changes == 0) {
            widgets::Mono(c::t1, "applied profile");
        } else {
            widgets::Mono(c::t1, "applied profile + %zu %s", changes,
                          changes == 1 ? "change" : "changes");
        }
        if (profile.restartRequired) {
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill("restart required", widgets::PillKind::Warn);
        }
        char text[48];
        widgets::PillKind kind = widgets::PillKind::Neutral;
        switch (session.validity()) {
            case ProfileSession::Validity::Valid:
                std::snprintf(text, sizeof(text), "valid · schema v%u", profile.schemaVersion);
                kind = widgets::PillKind::Ok;
                break;
            case ProfileSession::Validity::Invalid: {
                std::size_t errors = 0;
                for (const ProfileIssue& issue : session.issues()) {
                    errors += issue.error ? 1U : 0U;
                }
                std::snprintf(text, sizeof(text), "%zu %s", errors,
                              errors == 1 ? "error" : "errors");
                kind = widgets::PillKind::Error;
                break;
            }
            default:
                std::snprintf(text, sizeof(text), "checking ...");
                break;
        }
        ImGui::SameLine();
        widgets::AlignRight(widgets::PillWidth(text));
        widgets::Pill(text, kind);
    }

    static void issues(const ProfileSession& session) {
        if (session.issues().empty()) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(8.0F)));
        ImGui::Indent(px(10.0F));
        for (const ProfileIssue& issue : session.issues()) {
            widgets::Pill(issue.error ? "error" : "warning",
                          issue.error ? widgets::PillKind::Error : widgets::PillKind::Warn);
            if (!issue.pointer.empty()) {
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::Mono(c::t1, "%s", issue.pointer.c_str());
            }
            ImGui::PushTextWrapPos(0.0F);
            ImGui::TextUnformatted(issue.message.c_str());
            ImGui::PopTextWrapPos();
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        ImGui::Unindent(px(10.0F));
    }
};

}  // namespace

std::unique_ptr<Panel> createProfilePanel() {
    return std::make_unique<ProfilePanel>();
}

}  // namespace lagrange::viewer
