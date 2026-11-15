// Domains workspace, right panel: the profile as YAML with the pending changes marked, the
// server's verdict on it, and Apply (docs/design/ui/03-domains/SPEC.md).
#include "PanelFormat.hpp"
#include "YamlText.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

using yaml_text::kGutter;
using yaml_text::kLineHeight;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

ImVec4 withAlpha(ImVec4 color, float alpha) {
    color.w = alpha;
    return color;
}

enum class LineKind { Same, Removed, Added };

void drawLine(std::string_view text, int number, LineKind kind) {
    char gutter[16];
    if (kind == LineKind::Removed) {
        std::snprintf(gutter, sizeof(gutter), "-");
    } else {
        std::snprintf(gutter, sizeof(gutter), "%d", number);
    }
    yaml_text::codeLine(text, gutter,
                        kind == LineKind::Removed ? yaml_text::Tint::Removed
                        : kind == LineKind::Added ? yaml_text::Tint::Added
                                                  : yaml_text::Tint::None);
}

/// The draft line a validation issue is about: the domain's line for "/domains/<name>/...",
/// the key's line for a top-level key ("/stage"); none otherwise.
std::optional<std::size_t> issueLine(const std::vector<std::string>& lines,
                                     std::string_view pointer) {
    if (pointer.empty() || pointer.front() != '/') {
        return std::nullopt;
    }
    std::string_view rest = pointer.substr(1);
    const std::string_view top = rest.substr(0, rest.find('/'));
    std::string key;
    std::size_t indent = 0;
    if (top == "domains" && rest.size() > top.size() + 1) {
        rest = rest.substr(top.size() + 1);
        key = std::string(rest.substr(0, rest.find('/')));
        indent = 2;
    } else {
        key = std::string(top);
    }
    const std::string prefix = std::string(indent, ' ') + key + ":";
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].starts_with(prefix)) {
            return i;
        }
    }
    return std::nullopt;
}

/// ImGui's text editing on a std::string, growing it as the user types.
int resizeString(ImGuiInputTextCallbackData* data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto* text = static_cast<std::string*>(data->UserData);
        text->resize(static_cast<std::size_t>(data->BufTextLen));
        data->Buf = text->data();
    }
    return 0;
}

/// Where Save writes when no file was chosen yet: the viewer's own profiles folder, named
/// after the stage ("<config>/profiles/amr_diffdrive.yaml").
std::string defaultProfilePath(const std::string& configDirectory,
                               const std::vector<std::string>& lines) {
    std::string stem = "profile";
    for (const std::string& line : lines) {
        if (line.starts_with("stage:")) {
            std::string stage = line.substr(6);
            stage.erase(0, stage.find_first_not_of(" \"'"));
            stage.erase(stage.find_last_not_of(" \"'") + 1);
            const std::string name = std::filesystem::path(stage).stem().string();
            if (!name.empty() && name != "none") {
                stem = name;
            }
            break;
        }
    }
    const std::filesystem::path folder = configDirectory.empty()
                                             ? std::filesystem::current_path()
                                             : std::filesystem::path(configDirectory) / "profiles";
    return (folder / (stem + ".yaml")).string();
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
        const ProfileDraft& draft = session.draft();
        const bool saveAs = savingAs_;
        const float footer = ImGui::GetFrameHeightWithSpacing() * (saveAs ? 3.0F : 2.0F) +
                             px(6.0F) + (message_.empty() ? 0.0F : ImGui::GetTextLineHeight());
        ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg0);
        ImGui::BeginChild("##yaml", ImVec2(0.0F, -footer), ImGuiChildFlags_Borders);
        ImGui::PopStyleColor();
        if (editing_) {
            editor(session, draft);
        } else {
            viewer(session, draft);
        }
        ImGui::EndChild();

        // Footer: what Apply does, then the actions. Apply is the one primary action here.
        widgets::Dot(c::warn, false);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::AlignTextToFramePadding();
        // Cut with an ellipsis in a narrow panel, whole in a tooltip (QA3 F6).
        fit::text("Applying rebuilds the wiring and restarts the run at t = 0.",
                  ImGui::GetContentRegionAvail().x);
        ImGui::BeginDisabled(!draft.dirty());
        if (widgets::Button("Revert")) {
            session.revert();
            editing_ = false;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (widgets::Button("Save as…")) {
            startSaveAs(context, draft);
        }
        ImGui::SameLine();
        // Save, with its key in the button (design: .btn with a .kbd inside).
        const float kbdWidth =
            px(10.0F) + theme::fonts().mono->CalcTextSizeA(px(10.0F), FLT_MAX, 0.0F, "Ctrl S").x;
        // Room for the key after the word: as many spaces as it is wide (the button's font).
        ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
        const float space = std::max(ImGui::CalcTextSize(" ").x, 1.0F);
        ImGui::PopFont();
        std::string saveLabel = "Save";
        saveLabel.append(static_cast<std::size_t>(std::ceil((kbdWidth + px(6.0F)) / space)), ' ');
        saveLabel += "##save";
        const bool saveClicked = widgets::Button(saveLabel.c_str());
        {
            const ImVec2 max = ImGui::GetItemRectMax();
            const ImVec2 min = ImGui::GetItemRectMin();
            const ImVec2 box(kbdWidth, px(14.0F));
            const ImVec2 at(max.x - box.x - px(6.0F), min.y + (max.y - min.y - box.y) * 0.5F);
            ImDrawList* list = ImGui::GetWindowDrawList();
            list->AddRect(at, ImVec2(at.x + box.x, at.y + box.y), u32(c::lineStrong), px(2.0F));
            list->AddText(theme::fonts().mono, px(10.0F),
                          ImVec2(at.x + px(5.0F), at.y + (box.y - px(10.0F)) * 0.5F - px(1.0F)),
                          u32(c::t3), "Ctrl S");
        }
        if (saveClicked || ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_S)) {
            if (path_.empty()) {
                startSaveAs(context, draft);
            } else {
                save(draft, path_);
            }
        }
        ImGui::SameLine();
        // A narrow panel (1024 px wide) has no room for "Apply and restart": the button says
        // "Apply" and its tooltip the rest (QA3 F6), never a word cut in half.
        const bool roomy =
            ImGui::GetContentRegionAvail().x >= widgets::ButtonWidth("Apply and restart");
        const char* applyLabel = roomy ? "Apply and restart" : "Apply##restart";
        widgets::AlignRight(widgets::ButtonWidth(applyLabel));
        ImGui::BeginDisabled(!connected || !session.canApply());
        if (widgets::PrimaryButton(applyLabel)) {
            session.apply(context.client);
            editing_ = false;
        }
        ImGui::EndDisabled();
        if (!roomy && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip(
                "Apply and restart: rebuilds the wiring and restarts the run at t = 0");
        }
        if (savingAs_) {
            // Save as: the file on this machine (the viewer's, not the server's).
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x -
                                    widgets::ButtonWidth("Save") - widgets::ButtonWidth("Cancel") -
                                    px(16.0F));
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            if (focusSavePath_) {
                ImGui::SetKeyboardFocusHere();  // Enter saves at once
                focusSavePath_ = false;
            }
            const bool enter = ImGui::InputText("##save-path", savePath_.data(), savePath_.size(),
                                                ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::PopFont();
            ImGui::SameLine();
            if (widgets::Button("Save##as") || enter) {
                path_ = savePath_.data();
                save(draft, path_);
                savingAs_ = false;
            }
            ImGui::SameLine();
            if (widgets::Button("Cancel")) {
                savingAs_ = false;
            }
        }
        if (!message_.empty()) {
            widgets::Caption("%s", message_.c_str());
        }
        ImGui::End();
    }

private:
    /// The profile with its changes marked; a click puts it into the editor.
    void viewer(ProfileSession& session, const ProfileDraft& draft) {
        // Each issue under its line (SPEC 03: "show the error inline at the line").
        std::vector<std::vector<const ProfileIssue*>> byLine(draft.lines().size());
        std::vector<const ProfileIssue*> loose;
        for (const ProfileIssue& issue : session.issues()) {
            if (const auto line = issueLine(draft.lines(), issue.pointer)) {
                byLine[*line].push_back(&issue);
            } else {
                loose.push_back(&issue);
            }
        }
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
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
            for (const ProfileIssue* issue : byLine[i]) {
                inlineIssue(*issue);
            }
        }
        ImGui::PopStyleVar();
        // A click on the text edits it (the window's own empty space too).
        if (ImGui::IsWindowHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_TextInput);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                startEditing(draft);
            }
        }
        issues(loose);
    }

    /// The YAML as text to type in; the server checks it a moment after each change.
    void editor(ProfileSession& session, const ProfileDraft& draft) {
        if (draft.revision() != shownRevision_ && !ImGui::IsAnyItemActive()) {
            text_ = draft.text();  // changed elsewhere (the Domains table, Revert)
            shownRevision_ = draft.revision();
        }
        std::vector<const ProfileIssue*> all;
        for (const ProfileIssue& issue : session.issues()) {
            all.push_back(&issue);
        }
        const float issuesHeight =
            all.empty()
                ? 0.0F
                : std::min(px(120.0F), (ImGui::GetTextLineHeightWithSpacing() * 2.0F + px(8.0F)) *
                                           static_cast<float>(all.size()));
        // As the read view (yaml_text): 12 px mono on 19 px lines, through the code face.
        ImGui::PushFont(theme::fonts().code, theme::kCodeLine);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, c::bg0);
        if (focusEditor_) {
            ImGui::SetKeyboardFocusHere();
            focusEditor_ = false;
        }
        text_.reserve(text_.size() + 1);
        const bool changed = ImGui::InputTextMultiline(
            "##profile-text", text_.data(), text_.capacity() + 1,
            ImVec2(-1.0F, ImGui::GetContentRegionAvail().y - issuesHeight),
            ImGuiInputTextFlags_AllowTabInput | ImGuiInputTextFlags_CallbackResize, resizeString,
            &text_);
        ImGui::PopStyleColor();
        ImGui::PopFont();
        if (changed) {
            session.setText(text_);
            shownRevision_ = session.draft().revision();
        }
        // Leaving the editor shows the profile with its changes marked again.
        if (ImGui::IsItemDeactivated() || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            editing_ = false;
        }
        issues(all);
    }

    void startEditing(const ProfileDraft& draft) {
        editing_ = true;
        focusEditor_ = true;
        text_ = draft.text();
        shownRevision_ = draft.revision();
    }

    void startSaveAs(const PanelContext& context, const ProfileDraft& draft) {
        savingAs_ = true;
        focusSavePath_ = true;
        const std::string suggested =
            path_.empty() ? defaultProfilePath(context.configDirectory, draft.lines()) : path_;
        std::snprintf(savePath_.data(), savePath_.size(), "%s", suggested.c_str());
    }

    void save(const ProfileDraft& draft, const std::string& path) {
        std::error_code ec;
        const std::filesystem::path file(path);
        if (file.has_parent_path()) {
            std::filesystem::create_directories(file.parent_path(), ec);
        }
        std::ofstream out(file);
        out << draft.text();
        out.close();
        message_ = out ? "Saved " + path + " (on this machine)" : "Could not write " + path;
    }

    static void inlineIssue(const ProfileIssue& issue) {
        // Under the line, indented past the gutter, on the issue's tint.
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float text = width - px(kGutter) - px(8.0F);
        const float height =
            widgets::ParagraphHeight(ImGui::GetFont(), ImGui::GetFontSize(), issue.message, text) +
            px(6.0F);
        const ImVec4 tint = issue.error ? c::error : c::warn;
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(min, ImVec2(min.x + width, min.y + height),
                            u32(withAlpha(tint, 0.12F)));
        draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), min.y + height), u32(tint));
        ImGui::SetCursorScreenPos(ImVec2(min.x + px(kGutter), min.y + px(3.0F)));
        ImGui::PushStyleColor(ImGuiCol_Text, tint);
        widgets::Paragraph(issue.message, 0.0F, text);
        ImGui::PopStyleColor();
        ImGui::SetCursorScreenPos(ImVec2(min.x, min.y + height));
        ImGui::Dummy(ImVec2(width, 0.0F));
    }

    static void header(const ProfileSession& session, const ProfileInfo& profile,
                       const std::string& path) {
        ImGui::AlignTextToFramePadding();
        const std::size_t changes = session.draft().changedLineCount();
        // The file's name; its whole path in the tooltip and the Save line.
        const std::string file = std::filesystem::path(path).filename().string();
        const char* name = path.empty() ? "applied profile" : file.c_str();
        if (changes == 0) {
            widgets::Mono(c::t1, "%s", name);
        } else {
            widgets::Mono(c::t1, "%s + %zu %s", name, changes, changes == 1 ? "change" : "changes");
        }
        if (!path.empty()) {
            ImGui::SetItemTooltip("%s", path.c_str());
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
    void header(const ProfileSession& session, const ProfileInfo& profile) const {
        header(session, profile, path_);
    }

    static void issues(const std::vector<const ProfileIssue*>& list) {
        if (list.empty()) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(8.0F)));
        ImGui::Indent(px(10.0F));
        for (const ProfileIssue* issue : list) {
            widgets::Pill(issue->error ? "error" : "warning",
                          issue->error ? widgets::PillKind::Error : widgets::PillKind::Warn);
            if (!issue->pointer.empty()) {
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::Mono(c::t1, "%s", issue->pointer.c_str());
            }
            widgets::Paragraph(issue->message);
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        ImGui::Unindent(px(10.0F));
    }

    bool editing_{false};
    bool focusEditor_{false};
    std::string text_;
    std::uint64_t shownRevision_{0};
    std::string path_;  ///< The file Save writes, once one was chosen; on the viewer's machine.
    bool savingAs_{false};
    bool focusSavePath_{false};
    std::array<char, 512> savePath_{};
    std::string message_;
};

}  // namespace

std::unique_ptr<Panel> createProfilePanel() {
    return std::make_unique<ProfilePanel>();
}

}  // namespace lagrange::viewer
