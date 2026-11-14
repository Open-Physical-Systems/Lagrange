// The Builder's Joints and Poses tabs (docs/design/ui/12-builder/SPEC.md "Joints"). The
// preview poses the robot by forward kinematics only: it is view state, never an undo step.
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using builder::Joint;
using widgets::px;

/// The slider's range. Limits when both are given; otherwise a view range only, never written
/// to the document (no invented limits): ±180° for angles, ±1 m for a prismatic joint.
std::pair<double, double> previewRange(const Joint& joint) {
    if (joint.type == Joint::Type::Continuous) {
        return {-std::numbers::pi, std::numbers::pi};
    }
    if (joint.lower && joint.upper && *joint.lower < *joint.upper) {
        return {*joint.lower, *joint.upper};
    }
    return joint.type == Joint::Type::Prismatic ? std::pair{-1.0, 1.0}
                                                : std::pair{-std::numbers::pi, std::numbers::pi};
}

bool angular(const Joint& joint) {
    return joint.type != Joint::Type::Prismatic;
}

class JointsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return title_.c_str(); }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        BuilderSession& session = *context.builder;
        const builder::Document& document = session.document();
        int moving = 0;
        int fixed = 0;
        for (const Joint& joint : document.joints) {
            (builder::isMoving(joint.type) ? moving : fixed) += 1;
        }
        title_ = "Joints · " + std::to_string(moving) + " moving###BuilderJoints";
        if (!ImGui::Begin(title_.c_str())) {
            ImGui::End();
            return;
        }
        // Tools (design: deg | rad, Zero all).
        const char* units[] = {"deg", "rad"};
        const float zeroWidth = widgets::ButtonWidth("Zero all");
        if (widgets::BeginPanelTools("##joint-tools", px(80.0F) + zeroWidth)) {
            int index = session.showDegrees ? 0 : 1;
            if (widgets::Segmented("##units", units, 2, &index)) {
                session.showDegrees = index == 0;
            }
            ImGui::SameLine(0.0F, px(4.0F));
            if (widgets::Button("Zero all", widgets::ButtonKind::Ghost)) {
                if (session.mode == BuilderSession::Mode::Testing) {
                    // Testing: every moving joint is told to go to zero (RFC 0016 §7.3).
                    for (const Joint& joint : document.joints) {
                        if (builder::isMoving(joint.type) && !joint.mimic) {
                            session.jointTargets[joint.name] = 0.0;
                        }
                    }
                } else {
                    session.preview.clear();
                }
            }
            widgets::EndPanelTools();
        }
        if (moving == 0) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "No moving joints yet: add a link with a revolute, continuous or "
                "prismatic joint");
            ImGui::PopTextWrapPos();
        } else {
            grid(session);
        }
        // Footer (design: a rule, then "2 fixed joints not shown" and "Preview only").
        const float footer = px(30.0F);
        if (ImGui::GetContentRegionAvail().y > footer) {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + ImGui::GetContentRegionAvail().y -
                                 footer);
        }
        const ImVec2 at = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(at,
                                            ImVec2(at.x + ImGui::GetContentRegionAvail().x, at.y),
                                            ImGui::ColorConvertFloat4ToU32(c::line));
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        if (fixed > 0) {
            widgets::Caption("%d fixed joint%s not shown", fixed, fixed == 1 ? "" : "s");
            ImGui::SameLine();
        }
        // Testing: values are the run's; a slider commands its joint's position (§7.3).
        if (session.mode == BuilderSession::Mode::Testing && !session.jointDriveError.empty()) {
            // One line: cut with an ellipsis to the room left; the tooltip has all of it.
            std::string note = "Not driven: " + session.jointDriveError;
            const float room = ImGui::GetContentRegionAvail().x - px(4.0F);
            if (ImGui::CalcTextSize(note.c_str()).x * 0.92F > room) {
                while (note.size() > 12 &&
                       ImGui::CalcTextSize((note + "…").c_str()).x * 0.92F > room) {
                    // A whole UTF-8 character: its continuation bytes, then its lead byte.
                    unsigned char last = 0;
                    do {
                        last = static_cast<unsigned char>(note.back());
                        note.pop_back();
                    } while (!note.empty() && (last & 0xC0U) == 0x80U);
                }
                note += "…";
            }
            widgets::AlignRight(ImGui::CalcTextSize(note.c_str()).x * 0.92F);
            ImGui::PushStyleColor(ImGuiCol_Text, c::error);
            widgets::Caption("%s", note.c_str());
            ImGui::PopStyleColor();
            ImGui::SetItemTooltip("%s", session.jointDriveError.c_str());
        } else {
            const char* note = session.mode == BuilderSession::Mode::Build
                                   ? "Preview only: no physics"
                               : session.mode == BuilderSession::Mode::Testing
                                   ? "Live from the run · a slider commands its position"
                                   : "Starting the test…";
            widgets::AlignRight(ImGui::CalcTextSize(note).x * 0.92F);
            widgets::Caption("%s", note);
        }
        ImGui::End();
    }

private:
    void grid(BuilderSession& session) {
        const builder::Document& document = session.document();
        // Design (.jg): 128 px joint, 78 px type, the slider, 72 px value, 92 px limits.
        const float total = ImGui::GetContentRegionAvail().x;
        const float gap = px(10.0F);
        const float nameW = px(128.0F);
        const float typeW = px(78.0F);
        const float valueW = px(72.0F);
        const float limitsW = px(92.0F);
        const float sliderW =
            std::max(total - nameW - typeW - valueW - limitsW - gap * 4.0F, px(60.0F));
        const float x0 = ImGui::GetCursorPosX();
        const auto header = [&](const char* text, float x) {
            ImGui::SameLine(x0 + x);
            widgets::Caption("%s", text);
        };
        ImGui::Dummy(ImVec2(0.0F, 0.0F));
        header("joint", 0.0F);
        header("type", nameW + gap);
        const bool testing = session.mode == BuilderSession::Mode::Testing;
        header(testing ? "position · drag to command" : "preview position",
               nameW + typeW + gap * 2.0F);
        header("value", nameW + typeW + sliderW + gap * 3.0F + valueW - px(30.0F));
        header("limits", nameW + typeW + sliderW + valueW + gap * 4.0F);

        // Build: sliders pose the preview. Testing: they show the run's positions and, dragged
        // or typed, command them (RFC 0016 §7.3). Starting: neither.
        const bool live = session.mode == BuilderSession::Mode::Build || testing;
        for (const Joint& joint : document.joints) {
            if (!builder::isMoving(joint.type)) {
                continue;
            }
            ImGui::PushID(joint.name.c_str());
            const float rowY = ImGui::GetCursorPosY();
            const float height = px(30.0F);
            const auto cell = [&](float x, float h) {
                ImGui::SetCursorPos(ImVec2(x0 + x, rowY + (height - h) * 0.5F));
            };
            cell(0.0F, ImGui::GetTextLineHeight());
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            ImGui::TextColored(c::t1, "%s", joint.name.c_str());
            ImGui::PopFont();
            cell(nameW + gap, px(18.0F));
            widgets::Pill(std::string(builder::toString(joint.type)).c_str(),
                          widgets::PillKind::Neutral);

            // A mimic joint follows its leader and shows no knob.
            const bool mimic = joint.mimic.has_value();
            double value = 0.0;
            const auto liveValue = session.liveJoints.find(joint.name);
            const bool hasLive = testing && liveValue != session.liveJoints.end();
            if (testing) {
                // The knob follows the run, except while it is dragged: then it is the command.
                const auto target = session.jointTargets.find(joint.name);
                value = dragging_ == joint.name && target != session.jointTargets.end()
                            ? target->second
                        : hasLive                              ? liveValue->second
                        : target != session.jointTargets.end() ? target->second
                                                               : 0.0;
            } else if (mimic) {
                const auto& [leader, multiplier, offset] = *joint.mimic;
                const auto found = session.preview.find(leader);
                value =
                    multiplier * (found != session.preview.end() ? found->second : 0.0) + offset;
            } else if (const auto found = session.preview.find(joint.name);
                       found != session.preview.end()) {
                value = found->second;
            }
            const auto [lower, upper] = previewRange(joint);
            cell(nameW + typeW + gap * 2.0F, px(30.0F));
            if (widgets::JointSlider("##slider", &value, lower, upper,
                                     joint.type == Joint::Type::Continuous, !mimic && live,
                                     sliderW)) {
                (testing ? session.jointTargets : session.preview)[joint.name] = value;
            }
            if (testing) {
                if (ImGui::IsItemActive()) {
                    dragging_ = joint.name;
                } else if (dragging_ == joint.name) {
                    dragging_.clear();
                }
            }
            if (mimic) {
                ImGui::SetItemTooltip("Follows %s", std::get<0>(*joint.mimic).c_str());
            }

            const bool degrees = angular(joint) && session.showDegrees;
            cell(nameW + typeW + sliderW + gap * 3.0F, px(22.0F));
            // Testing: the value field is the run's position (typing commands it).
            const double fieldValue = hasLive ? liveValue->second : value;
            const double shownValue = degrees ? builder::toDegrees(fieldValue) : fieldValue;
            char text[32];
            std::snprintf(text, sizeof(text), "%.1f", shownValue);
            if (!degrees) {
                std::snprintf(text, sizeof(text), "%.3f", shownValue);
            }
            std::string field = testing && !hasLive ? std::string("—") : std::string(text);
            ImGui::BeginDisabled(mimic || !live);
            if (widgets::CommitField("##value", &field,
                                     angular(joint) ? (degrees ? "°" : "rad") : "m", false,
                                     valueW)) {
                if (const auto typed = builder::parseNumber(field)) {
                    (testing ? session.jointTargets : session.preview)[joint.name] =
                        degrees ? builder::toRadians(*typed) : *typed;
                }
            }
            ImGui::EndDisabled();

            cell(nameW + typeW + sliderW + valueW + gap * 4.0F, ImGui::GetTextLineHeight());
            if (joint.lower && joint.upper) {
                const double lo = degrees ? builder::toDegrees(*joint.lower) : *joint.lower;
                const double hi = degrees ? builder::toDegrees(*joint.upper) : *joint.upper;
                widgets::Mono(c::t2, "%s … %s%s", builder::formatNumber(lo, 4).c_str(),
                              builder::formatNumber(hi, 4).c_str(),
                              angular(joint) ? (degrees ? "°" : " rad") : " m");
            } else {
                widgets::Caption("none");
            }
            ImGui::SetCursorPos(ImVec2(x0, rowY + height));
            ImGui::Dummy(ImVec2(0.0F, 0.0F));
            ImGui::PopID();
        }
    }

    std::string title_ = "Joints###BuilderJoints";
    std::string dragging_;  ///< Testing: the joint whose slider is held (its knob is the command)
};

/// Named sets of preview positions (SPEC "Poses", RFC 0016 open question 10). The document has
/// no place for them yet (its structs are the RFC's), so they last for this session only.
class PosesPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kPoses; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float button = widgets::ButtonWidth("Save pose");
        (void)widgets::CommitField("##pose-name", &name_, nullptr, false,
                                   ImGui::GetContentRegionAvail().x - button - px(6.0F), false);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(name_.empty());
        if (widgets::Button("Save pose")) {
            bool replaced = false;
            for (auto& [name, preview] : poses_) {
                if (name == name_) {
                    preview = session.preview;
                    replaced = true;
                }
            }
            if (!replaced) {
                poses_.emplace_back(name_, session.preview);
            }
        }
        ImGui::EndDisabled();
        if (poses_.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("No poses yet: set the joints, type a name and save the pose.");
            ImGui::PopTextWrapPos();
        }
        for (std::size_t i = 0; i < poses_.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            widgets::Mono(c::t1, "%s", poses_[i].first.c_str());
            ImGui::SameLine();
            widgets::Caption("%zu joints", poses_[i].second.size());
            ImGui::SameLine();
            widgets::AlignRight(widgets::ButtonWidth("Show") + widgets::ButtonWidth("Remove") +
                                px(4.0F));
            if (widgets::Button("Show")) {
                session.preview = poses_[i].second;
            }
            ImGui::SameLine(0.0F, px(4.0F));
            if (widgets::Button("Remove", widgets::ButtonKind::Ghost)) {
                poses_.erase(poses_.begin() + static_cast<std::ptrdiff_t>(i));
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(
            "Kept for this session: the document has no poses yet (RFC 0016 open "
            "question 10).");
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    std::string name_;
    std::vector<std::pair<std::string, builder::Preview>> poses_;
};

}  // namespace

std::unique_ptr<Panel> createJointsPanel() {
    return std::make_unique<JointsPanel>();
}

std::unique_ptr<Panel> createPosesPanel() {
    return std::make_unique<PosesPanel>();
}

}  // namespace lagrange::viewer::builder_ui
