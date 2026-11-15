// The Builder's Test layout (docs/design/ui/12-builder/SPEC.md state 4): Drive, the ROS 2 topics
// the test robot takes with a copyable command line each, and Console, the test server's log.
// Both are drawn only while a Test runs; in Build they submit nothing.
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <string>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using widgets::px;

/// One topic: its name and type, what it does, and a command line with a Copy button.
void topic(const char* id, const char* name, const char* type, const char* what,
           const std::string& command) {
    ImGui::PushID(id);
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    widgets::Mono(c::t1, "%s", name);
    ImGui::SameLine();
    widgets::Pill(type, widgets::PillKind::Neutral);
    widgets::Caption("%s", what);
    ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x - px(64.0F));
    widgets::Mono(c::t2, "%s", command.c_str());
    ImGui::PopTextWrapPos();
    ImGui::SameLine();
    widgets::AlignRight(px(56.0F));
    if (widgets::Button("Copy", widgets::ButtonKind::Ghost)) {
        ImGui::SetClipboardText(command.c_str());
    }
    ImGui::SetItemTooltip("Copy the command line");
    ImGui::PopID();
}

class DrivePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kDrive; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        const BuilderSession& session = *context.builder;
        if (session.mode == BuilderSession::Mode::Build) {
            return;  // the Test layout only
        }
        // A new Test: Drive in front of the catalog. Twice, as Console appears in the same
        // dock node on the first frame and would take the front.
        if (session.testSerial != shownSerial_ || focusAgain_) {
            focusAgain_ = session.testSerial != shownSerial_;
            shownSerial_ = session.testSerial;
            ImGui::SetNextWindowFocus();
        }
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const builder::Document& document = session.document();
        std::string domain;
        if (session.testIntoRun) {
            // RFC 0016 §7.2: the robot is in the run this viewer is connected to.
            widgets::Caption("in the connected run as");
            ImGui::SameLine();
            widgets::Mono(c::accent, "%s",
                          session.testRobotName.empty() ? "…" : session.testRobotName.c_str());
            ImGui::SameLine();
            widgets::Mono(c::t2, "%s", session.testEndpoint.c_str());
            if (session.testPassive) {
                widgets::Pill("passive", widgets::PillKind::Warn);
                ImGui::SameLine();
                widgets::Caption("the run already has a robot: nothing drives this one");
                ImGui::End();
                return;
            }
            widgets::Caption("ROS 2 on the run's own domain");
        } else {
            widgets::Caption("test server");
            ImGui::SameLine();
            widgets::Mono(c::t2, "%s", session.testEndpoint.c_str());
            if (session.testRosDomain.empty()) {
                widgets::Pill("no ROS", widgets::PillKind::Neutral);
                ImGui::SameLine();
                widgets::Caption("started with --no-ros (LAGRANGE_BUILDER_ROS_DOMAIN_ID=off)");
                ImGui::End();
                return;
            }
            // Plan R8: the test server has a ROS domain of its own, so a teleop on the user's
            // domain does not drive it, nor this one the user's run.
            domain = "ROS_DOMAIN_ID=" + session.testRosDomain + " ";
            widgets::Caption("ROS 2 on its own domain");
            ImGui::SameLine();
            widgets::Mono(c::accent, "%s", session.testRosDomain.c_str());
        }
        std::string joint;
        for (const builder::Joint& j : document.joints) {
            if (builder::isMoving(j.type) && !j.mimic) {
                joint = j.name;
                break;
            }
        }
        topic("cmd_vel", "/cmd_vel", "geometry_msgs/TwistStamped",
              "a wheeled base or a multicopter, when the server drives it",
              domain +
                  "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p "
                  "stamped:=true");
        if (!joint.empty()) {
            topic("joint_commands", "/joint_commands", "sensor_msgs/JointState",
                  "every hinge and slide: position, velocity or effort (--joint-control)",
                  domain +
                      "ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
                      "\"{name: [" +
                      joint + "], position: [0.5]}\"");
        }
        topic("joint_states", "/joint_states", "sensor_msgs/JointState", "what the joints do",
              domain + "ros2 topic echo /joint_states");
        ImGui::End();
    }

private:
    std::uint64_t shownSerial_{0};
    bool focusAgain_{false};
};

class TestConsolePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kTestConsole; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        const BuilderSession& session = *context.builder;
        if (session.mode == BuilderSession::Mode::Build) {
            return;
        }
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        widgets::Caption("the test server's output (stdout and stderr)");
        ImGui::BeginChild("##log", ImVec2(0.0F, 0.0F), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::TextUnformatted(session.testLog.data(),
                               session.testLog.data() + session.testLog.size());
        ImGui::PopFont();
        if (session.testLog.size() != shownBytes_) {
            shownBytes_ = session.testLog.size();
            ImGui::SetScrollHereY(1.0F);  // follow the newest line
        }
        ImGui::EndChild();
        ImGui::End();
    }

private:
    std::size_t shownBytes_{0};
};

}  // namespace

std::unique_ptr<Panel> createDrivePanel() {
    return std::make_unique<DrivePanel>();
}

std::unique_ptr<Panel> createTestConsolePanel() {
    return std::make_unique<TestConsolePanel>();
}

}  // namespace lagrange::viewer::builder_ui
