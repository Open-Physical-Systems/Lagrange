// The launcher: the first screen, shown until the viewer is connected. System check on the
// left, what to open in the middle, which server on the right
// (docs/design/ui/01-launcher/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <imgui_internal.h>  // BeginViewportSideBar
#include <string>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr ImGuiWindowFlags kFixed =
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

void text(ImFont* font, float size, const ImVec4& color, const char* string) {
    ImGui::PushFont(font, size);
    ImGui::TextColored(color, "%s", string);
    ImGui::PopFont();
}

bool containsNoCase(std::string_view haystack, std::string_view needle) {
    const auto it = std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(),
                                [](char a, char b) {
                                    return std::tolower(static_cast<unsigned char>(a)) ==
                                           std::tolower(static_cast<unsigned char>(b));
                                });
    return it != haystack.end();
}

// --- left column: what this is, and the system check ---------------------------------------

/// One row of the system check: a mark, a line, and a detail line under it.
void check(bool ok, const char* line, const char* detail) {
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float icon = px(12.0F);
    if (ok) {
        widgets::DrawIcon(ImGui::GetWindowDrawList(), widgets::Icon::Check,
                          ImVec2(at.x, at.y + px(4.0F)), icon, u32(c::ok), u32(c::bg1));
    } else {
        ImGui::GetWindowDrawList()->AddText(theme::fonts().sansSemiBold, px(theme::kBodySize),
                                            ImVec2(at.x + px(3.0F), at.y), u32(c::warn), "!");
    }
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(26.0F));
    ImGui::BeginGroup();
    text(theme::fonts().sans, theme::kBodySize, c::t1, line);
    ImGui::PushTextWrapPos(0.0F);
    text(theme::fonts().sans, theme::kLabelSize, c::t3, detail);
    ImGui::PopTextWrapPos();
    ImGui::EndGroup();
    ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
}

void about(const LauncherState& state) {
    const SystemInfo& system = state.system;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    widgets::DrawLogo(draw, at, px(52.0F));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(66.0F));
    ImGui::BeginGroup();
    text(theme::fonts().sansSemiBold, 28.0F, c::t1, "lagrange");
    widgets::Mono(c::t3, "0.1.0-dev  ·  Apache-2.0");
    ImGui::EndGroup();
    ImGui::Dummy(ImVec2(0.0F, px(18.0F)));
    ImGui::PushTextWrapPos(0.0F);
    text(theme::fonts().sans, 14.0F, c::t2,
         "Coupled multi-physics robot simulation. Turn each physics domain on, off, or replace "
         "it with a substitute, and watch every signal live.");
    ImGui::PopTextWrapPos();
    ImGui::Dummy(ImVec2(0.0F, px(20.0F)));
    widgets::Caption("SYSTEM CHECK");
    ImGui::Dummy(ImVec2(0.0F, px(6.0F)));

    char line[160];
    char detail[200];
    // 1. GPU and the tier it gives.
    std::snprintf(detail, sizeof(detail), "%s → %s%s", system.gpuDetail.c_str(),
                  system.tier.c_str(), system.softwareGpu ? " · software renderer, slow" : "");
    check(!system.gpu.empty() && !system.softwareGpu,
          system.gpu.empty() ? "No Vulkan device" : system.gpu.c_str(), detail);
    // 2. OS, session, window backend, scale.
    std::snprintf(line, sizeof(line), "%s · %s session",
                  system.os.empty() ? "Linux" : system.os.c_str(),
                  system.session.empty() ? "unknown" : system.session.c_str());
    std::snprintf(detail, sizeof(detail), "window backend SDL3 · HiDPI %.2fx",
                  static_cast<double>(system.scale));
    check(true, line, detail);
    // 3. ROS 2.
    if (system.rosDistro.empty()) {
        check(false, "ROS 2 is not sourced", "source /opt/ros/lyrical/setup.bash");
    } else {
        std::snprintf(line, sizeof(line), "ROS 2 %s sourced", system.rosDistro.c_str());
        std::snprintf(detail, sizeof(detail), "%s · ROS_DOMAIN_ID %s",
                      system.rmw.empty() ? "default RMW" : system.rmw.c_str(),
                      system.rosDomain.empty() ? "0" : system.rosDomain.c_str());
        check(true, line, detail);
    }
    // 4. CUDA: the viewer draws with Vulkan only; a CUDA path belongs to the simulation server.
    check(false, "CUDA fast path not used",
          "The viewer draws with Vulkan. Batched GPU physics is a server feature.");
}

// --- centre: what to open -----------------------------------------------------------------

/// An action tile (design: .act): icon box, title, what it takes, and its shortcut.
bool action(const char* id, widgets::Icon icon, const char* title, const char* detail,
            const char* shortcut, float width) {
    const float height = px(64.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(width, height));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(ImGui::IsItemHovered() ? c::bg2 : c::bg1), px(4.0F));
    draw->AddRect(min, max, u32(c::line), px(4.0F));
    const float box = px(36.0F);
    const ImVec2 boxMin(min.x + px(12.0F), min.y + (height - box) * 0.5F);
    draw->AddRectFilled(boxMin, ImVec2(boxMin.x + box, boxMin.y + box), u32(c::bg3), px(4.0F));
    draw->AddRect(boxMin, ImVec2(boxMin.x + box, boxMin.y + box), u32(c::lineStrong), px(4.0F));
    const float iconSize = px(18.0F);
    widgets::DrawIcon(
        draw, icon, ImVec2(boxMin.x + (box - iconSize) * 0.5F, boxMin.y + (box - iconSize) * 0.5F),
        iconSize, u32(c::t1), u32(c::bg3));
    const theme::Fonts& fonts = theme::fonts();
    const float x = boxMin.x + box + px(12.0F);
    draw->PushClipRect(min, ImVec2(max.x - px(64.0F), max.y), true);
    draw->AddText(fonts.sansMedium, px(14.0F), ImVec2(x, min.y + px(14.0F)), u32(c::t1), title);
    draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(x, min.y + px(36.0F)), u32(c::t3),
                  detail);
    draw->PopClipRect();
    // Shortcut hint at the right.
    const float size = px(10.0F);
    const ImVec2 extent = fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, shortcut);
    const ImVec2 kbdMin(max.x - extent.x - px(22.0F), min.y + (height - px(16.0F)) * 0.5F);
    draw->AddRect(kbdMin, ImVec2(kbdMin.x + extent.x + px(10.0F), kbdMin.y + px(16.0F)),
                  u32(c::lineStrong), px(2.0F));
    draw->AddText(fonts.mono, size, ImVec2(kbdMin.x + px(5.0F), kbdMin.y + px(3.0F)), u32(c::t3),
                  shortcut);
    return clicked;
}

struct RobotClass {
    const char* name;
    const char* description;
    const char* integrations;
    std::array<theme::Domain, 5> domains;
    int domainCount;
    bool available;  ///< The mock server has a sample of this class.
};

using D = theme::Domain;
constexpr std::array<RobotClass, 8> kClasses{{
    {"Mobile robot / AMR",
     "diff-drive in a warehouse · lidar, camera, IMU",
     "Nav2 · ros2_control",
     {D::RigidBody, D::Electrical, D::Thermal, D::Emi, D::Sensors},
     5,
     true},
    {"Robot arm",
     "6-DoF arm on a table · gripper contact",
     "MoveIt 2 · ros2_control",
     {D::RigidBody, D::Electrical, D::Thermal, D::RigidBody, D::RigidBody},
     3,
     false},
    {"Drone",
     "quadrotor · gusts, battery sag, GNSS",
     "PX4 · ArduPilot SITL",
     {D::RigidBody, D::Electrical, D::Aerodynamics, D::Weather, D::Sensors},
     5,
     false},
    {"Car",
     "Pacejka tires, suspension, EV powertrain",
     "ros2_control · CAN",
     {D::RigidBody, D::Electrical, D::Thermal, D::Weather, D::RigidBody},
     4,
     false},
    {"Autonomous car",
     "city block · traffic, full sensor suite",
     "Autoware · SUMO",
     {D::RigidBody, D::Electrical, D::Weather, D::Emi, D::Sensors},
     5,
     false},
    {"Huge world",
     "geospatial terrain · tile streaming, fleets",
     "3D Tiles · OSM · DEM",
     {D::RigidBody, D::Aerodynamics, D::Weather, D::Sensors, D::RigidBody},
     4,
     false},
    {"Electrical bench",
     "physics off · replay a real run through the power train",
     "MCAP replay",
     {D::Electrical, D::Thermal, D::RigidBody, D::RigidBody, D::RigidBody},
     2,
     false},
    {"Empty stage",
     "ground plane and sun · add robots later",
     "-",
     {D::RigidBody, D::RigidBody, D::RigidBody, D::RigidBody, D::RigidBody},
     1,
     false},
}};

/// The line art of a robot class, from the design's SVG (64 x 40 units, 1.6 unit strokes).
void art(ImDrawList* draw, int index, const ImVec2& centre, ImU32 ink, ImU32 hole) {
    const float s = px(1.0F);
    const ImVec2 o(centre.x - 32.0F * s, centre.y - 20.0F * s);
    const float w = 1.6F * s;
    const auto at = [&](float x, float y) { return ImVec2(o.x + x * s, o.y + y * s); };
    const auto wheel = [&](float x, float y, float r) {
        draw->AddCircleFilled(at(x, y), r * s, hole);
        draw->AddCircle(at(x, y), r * s, ink, 0, w);
    };
    const auto ellipse = [&](float x, float y) {
        draw->AddEllipse(at(x, y), ImVec2(8.0F * s, 2.5F * s), ink, 0.0F, 0, w);
    };
    switch (index) {
        case 0:  // AMR
            draw->AddRect(at(10, 12), at(54, 28), ink, 2.0F * s, w);
            draw->AddRect(at(28, 7), at(36, 12), ink, 1.0F * s, w);
            wheel(20, 30, 5);
            wheel(44, 30, 5);
            break;
        case 1:  // arm
            widgets::StrokeSvgPath(draw,
                                   "M14 36h20 M24 36V26 M24 26l12-12 M36 14l12 6 M48 20l4 5 "
                                   "M48 20l6-2",
                                   o, s, ink, w);
            wheel(24, 26, 3);
            wheel(36, 14, 3);
            wheel(48, 20, 2.5F);
            break;
        case 2:  // drone
            widgets::StrokeSvgPath(draw, "M22 14l20 12 M42 14L22 26", o, s, ink, w);
            draw->AddRectFilled(at(27, 16), at(37, 24), hole, 2.0F * s);
            draw->AddRect(at(27, 16), at(37, 24), ink, 2.0F * s, w);
            ellipse(18, 12);
            ellipse(46, 12);
            ellipse(18, 28);
            ellipse(46, 28);
            break;
        case 3:  // car
            widgets::StrokeSvgPath(draw, "M6 28v-6l8-2 7-8h18l9 8 8 2v6z", o, s, ink, w);
            wheel(18, 29, 5);
            wheel(46, 29, 5);
            break;
        case 4:  // autonomous car
            widgets::StrokeSvgPath(draw, "M6 30v-6l8-2 7-8h18l9 8 8 2v6z", o, s, ink, w);
            draw->AddRect(at(28, 9), at(36, 14), ink, 1.0F * s, w);
            widgets::StrokeSvgPath(draw, "M24 6a10 10 0 0 1 16 0 M20 3a15 15 0 0 1 24 0", o, s, ink,
                                   w);
            wheel(18, 31, 5);
            wheel(46, 31, 5);
            break;
        case 5:  // huge world
            widgets::StrokeSvgPath(draw, "M4 32l14-14 10 8 12-14 20 20z", o, s, ink, w);
            for (float x = 4.0F; x < 60.0F; x += 6.0F) {
                draw->AddLine(at(x, 36), at(std::min(x + 3.0F, 60.0F), 36), ink, w);
            }
            draw->AddCircle(at(46, 10), 3.0F * s, ink, 0, w);
            break;
        case 6:  // electrical bench
            widgets::StrokeSvgPath(draw, "M32 6l14 7v14l-14 7-14-7V13z M18 13l14 7 14-7 M32 20v14",
                                   o, s, ink, w);
            break;
        default:  // empty stage
            draw->AddLine(at(32, 12), at(32, 28), ink, w);
            draw->AddLine(at(24, 20), at(40, 20), ink, w);
            break;
    }
}

/// A robot-class card (design: .tpl). Returns true when clicked.
bool card(int index, const RobotClass& item, bool selected, const ImVec2& size) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImGui::PushID(index);
    const bool clicked = ImGui::InvisibleButton("##class", size);
    ImGui::PopID();
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(hovered && item.available ? c::bg2 : c::bg1), px(4.0F));
    draw->AddRect(min, max, u32(selected ? c::accent : c::line), px(4.0F),
                  selected ? px(2.0F) : 1.0F);
    const float pad = px(12.0F);
    const ImVec2 artMin(min.x + pad, min.y + pad);
    const ImVec2 artMax(max.x - pad, artMin.y + px(56.0F));
    draw->AddRectFilled(artMin, artMax, u32(c::bg0), px(3.0F));
    draw->AddRect(artMin, artMax, u32(c::line), px(3.0F));
    const ImVec4 ink = item.available ? c::t2 : c::t3;
    art(draw, index, ImVec2((artMin.x + artMax.x) * 0.5F, (artMin.y + artMax.y) * 0.5F), u32(ink),
        u32(c::bg0));

    const theme::Fonts& fonts = theme::fonts();
    draw->PushClipRect(ImVec2(min.x + pad, min.y), ImVec2(max.x - pad, max.y), true);
    float y = artMax.y + px(10.0F);
    draw->AddText(fonts.sansMedium, px(14.0F), ImVec2(min.x + pad, y),
                  u32(item.available ? c::t1 : c::t2), item.name);
    y += px(24.0F);
    draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(min.x + pad, y), u32(c::t3),
                  item.description, nullptr, size.x - pad * 2.0F);
    // Bottom line: the domains the class turns on, and what it integrates with.
    const float bottom = max.y - pad - px(14.0F);
    for (int d = 0; d < item.domainCount; ++d) {
        draw->AddCircleFilled(
            ImVec2(min.x + pad + px(4.0F) + px(11.0F) * static_cast<float>(d), bottom + px(7.0F)),
            px(3.5F),
            u32(theme::kDomainColor[static_cast<int>(
                item.domains.at(static_cast<std::size_t>(d)))]));
    }
    const float small = px(theme::kLabelSize);
    const float width = fonts.sans->CalcTextSizeA(small, FLT_MAX, 0.0F, item.integrations).x;
    draw->AddText(fonts.sans, small, ImVec2(max.x - pad - width, bottom), u32(c::t3),
                  item.integrations);
    draw->PopClipRect();
    if (hovered && !item.available) {
        ImGui::SetTooltip("No sample of this robot class yet");
    }
    return clicked && item.available;
}

void start(PanelContext& context, LauncherState& state) {
    const float width = ImGui::GetContentRegionAvail().x;
    const float gap = px(8.0F);
    const float tile = (width - gap * 2.0F) / 3.0F;
    // The file is read by the server, so a tile only chooses what to open after connecting.
    const bool canPick = context.canOpenFiles;
    ImGui::BeginDisabled(!canPick);
    if (action("##open-stage", widgets::Icon::Folder, "Open stage", ".usd · .usda · .usdc",
               "Ctrl O", tile)) {
        context.openFileRequested = true;
    }
    ImGui::SameLine(0.0F, gap);
    if (action("##import-robot", widgets::Icon::Download, "Import robot",
               "URDF / xacro · SDFormat · MJCF", "Ctrl I", tile)) {
        context.openFileRequested = true;
    }
    ImGui::SameLine(0.0F, gap);
    if (action("##open-recording", widgets::Icon::Replay, "Open recording", ".mcap → replay mode",
               "Ctrl R", tile)) {
        context.openFileRequested = true;
    }
    ImGui::EndDisabled();
    if (canPick && (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_I) ||
                    ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_R))) {
        context.openFileRequested = true;
    }

    ImGui::Dummy(ImVec2(0.0F, px(16.0F)));
    text(theme::fonts().sansSemiBold, 14.0F, c::t1, "Start from a robot class");
    ImGui::SameLine(0.0F, px(8.0F));
    ImGui::AlignTextToFramePadding();
    text(theme::fonts().sans, theme::kBodySize, c::t3, "sample robot, world and sim profile");
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    const ImVec2 cardSize((width - gap * 3.0F) / 4.0F, px(178.0F));
    for (int i = 0; i < static_cast<int>(kClasses.size()); ++i) {
        if (i % 4 != 0) {
            ImGui::SameLine(0.0F, gap);
        }
        if (card(i, kClasses.at(static_cast<std::size_t>(i)), state.robotClass == i, cardSize)) {
            state.robotClass = i;
            state.openTarget.clear();  // the sample the server already serves
        }
        if (i % 4 == 3) {
            ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        }
    }

    // Recent.
    ImGui::Dummy(ImVec2(0.0F, px(16.0F)));
    const ImVec2 heading = ImGui::GetCursorPos();
    text(theme::fonts().sansSemiBold, 14.0F, c::t1, "Recent");
    ImGui::SetCursorPos(ImVec2(heading.x + width - px(220.0F), heading.y - px(3.0F)));
    widgets::SearchField("##recent-filter", "Filter recent", state.recentFilter,
                         sizeof(state.recentFilter), px(220.0F));
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg1);
    ImGui::BeginChild("##recent", ImVec2(0.0F, 0.0F), ImGuiChildFlags_Borders);
    ImGui::PopStyleColor();
    const std::vector<RecentEntry>& recent = state.recent.entries();
    if (recent.empty()) {
        widgets::Caption("Nothing opened yet. What you open is listed here.");
    } else if (ImGui::BeginTable("##recent-table", 5,
                                 ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                     ImGuiTableFlags_NoPadOuterX)) {
        ImGui::TableSetupColumn("stage", ImGuiTableColumnFlags_None, 2.2F);
        ImGui::TableSetupColumn("robot", ImGuiTableColumnFlags_None, 1.3F);
        ImGui::TableSetupColumn("profile", ImGuiTableColumnFlags_None, 1.2F);
        ImGui::TableSetupColumn("server", ImGuiTableColumnFlags_None, 2.4F);
        ImGui::TableSetupColumn("opened", ImGuiTableColumnFlags_None, 1.4F);
        ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
        for (const char* name : {"stage", "robot", "profile", "server", "opened"}) {
            ImGui::TableNextColumn();
            widgets::Caption("  %s", name);
        }
        for (std::size_t i = 0; i < recent.size(); ++i) {
            const RecentEntry& entry = recent[i];
            if (state.recentFilter[0] != '\0' && !containsNoCase(entry.stage, state.recentFilter) &&
                !containsNoCase(entry.robot, state.recentFilter) &&
                !containsNoCase(entry.endpoint, state.recentFilter)) {
                continue;
            }
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow(ImGuiTableRowFlags_None, px(23.0F));
            ImGui::TableNextColumn();
            // Double-click: the same server and the same thing again.
            if (ImGui::Selectable(
                    "##row", false,
                    ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick) &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                state.connectRequested = true;
                state.connectEndpoint = entry.endpoint;
                // A stage that is a file is opened again; a sample comes from the server.
                state.openTarget = entry.stage.find('/') == 0 ? entry.stage : std::string{};
            }
            ImGui::SameLine(px(8.0F));
            ImGui::TextUnformatted(entry.stage.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.robot.c_str());
            ImGui::TableNextColumn();
            widgets::Mono(c::t1, "%s", entry.profile.c_str());
            ImGui::TableNextColumn();
            widgets::Mono(c::t2, "%s", entry.endpoint.c_str());
            ImGui::TableNextColumn();
            widgets::Mono(c::t2, "%s", entry.opened.c_str());
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
}

// --- right column: which server -----------------------------------------------------------

/// A server row (design: .srv) with a radio mark, two lines and a status pill.
bool serverRow(const char* id, const char* name, const char* detail, const char* pill,
               widgets::PillKind kind, bool selected, bool enabled) {
    const float width = ImGui::GetContentRegionAvail().x;
    const float height = px(52.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    ImGui::BeginDisabled(!enabled);
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(width, height));
    ImGui::EndDisabled();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(selected ? theme::hex(0xE8772E, 0.08F) : c::bg1), px(4.0F));
    draw->AddRect(min, max, u32(selected ? c::accent : c::line), px(4.0F));
    const ImVec2 radio(min.x + px(18.0F), min.y + height * 0.5F);
    draw->AddCircle(radio, px(6.5F), u32(selected ? c::accent : c::lineStrong), 0, px(1.5F));
    if (selected) {
        draw->AddCircleFilled(radio, px(3.0F), u32(c::accent));
    }
    const theme::Fonts& fonts = theme::fonts();
    const float x = min.x + px(36.0F);
    draw->PushClipRect(min, ImVec2(max.x - px(96.0F), max.y), true);
    draw->AddText(fonts.sansMedium, px(14.0F), ImVec2(x, min.y + px(9.0F)),
                  u32(enabled ? c::t1 : c::t2), name);
    draw->AddText(fonts.mono, px(theme::kLabelSize), ImVec2(x, min.y + px(30.0F)), u32(c::t3),
                  detail);
    draw->PopClipRect();
    const float pillWidth = widgets::PillWidth(pill);
    ImGui::SetCursorScreenPos(ImVec2(max.x - pillWidth - px(10.0F), min.y + px(17.0F)));
    widgets::Pill(pill, kind);
    ImGui::SetCursorScreenPos(ImVec2(min.x, max.y + px(8.0F)));
    ImGui::Dummy(ImVec2(0.0F, 0.0F));
    return clicked && enabled;
}

/// The dashed drop zone at the bottom of the server column.
void dropZone(const LauncherState& state) {
    const float width = ImGui::GetContentRegionAvail().x;
    const float height = px(96.0F);
    ImGui::SetCursorPosY(
        std::max(ImGui::GetCursorPosY(), ImGui::GetWindowHeight() - height - px(24.0F)));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    ImGui::Dummy(ImVec2(width, height));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 line = u32(c::lineStrong);
    for (float x = min.x; x < max.x; x += px(7.0F)) {
        draw->AddLine(ImVec2(x, min.y), ImVec2(std::min(x + px(4.0F), max.x), min.y), line);
        draw->AddLine(ImVec2(x, max.y), ImVec2(std::min(x + px(4.0F), max.x), max.y), line);
    }
    for (float y = min.y; y < max.y; y += px(7.0F)) {
        draw->AddLine(ImVec2(min.x, y), ImVec2(min.x, std::min(y + px(4.0F), max.y)), line);
        draw->AddLine(ImVec2(max.x, y), ImVec2(max.x, std::min(y + px(4.0F), max.y)), line);
    }
    const float icon = px(22.0F);
    widgets::DrawIcon(draw, widgets::Icon::Download,
                      ImVec2((min.x + max.x - icon) * 0.5F, min.y + px(14.0F)), icon, u32(c::t2),
                      u32(c::bg1));
    const theme::Fonts& fonts = theme::fonts();
    const float size = px(theme::kBodySize);
    const char* first = state.openTarget.empty() ? "Drop a URDF, xacro or MCAP file"
                                                 : "Will open after connecting:";
    const std::string second =
        state.openTarget.empty() ? "anywhere in this window" : state.openTarget;
    const float a = fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, first).x;
    draw->AddText(fonts.sans, size, ImVec2((min.x + max.x - a) * 0.5F, min.y + px(44.0F)),
                  u32(c::t2), first);
    draw->PushClipRect(min, max, true);
    const float b = fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, second.c_str()).x;
    draw->AddText(fonts.sans, size,
                  ImVec2(std::max((min.x + max.x - b) * 0.5F, min.x + px(8.0F)), min.y + px(64.0F)),
                  u32(state.openTarget.empty() ? c::t2 : c::t1), second.c_str());
    draw->PopClipRect();
}

void servers(LauncherState& state) {
    text(theme::fonts().sansSemiBold, 14.0F, c::t1, "Simulation server");
    ImGui::SameLine();
    widgets::AlignRight(widgets::ButtonWidth("+ Add server"));
    if (widgets::Button("+ Add server",
                        state.showOther ? widgets::ButtonKind::On : widgets::ButtonKind::Ghost)) {
        state.showOther = !state.showOther;
        state.server = state.showOther ? 2 : 1;
    }
    ImGui::Dummy(ImVec2(0.0F, px(6.0F)));

    // Starting one is pointless while a server already listens on the default socket.
    const bool canStart = !state.localServer.empty() && !state.defaultAvailable;
    if (serverRow("##local", "Start local simulator",
                  state.localServer.empty() ? "lagrange_server is not installed"
                                            : "launches lagrange_server on this machine",
                  state.localServer.empty() ? "not installed"
                                            : (state.defaultAvailable ? "running" : "available"),
                  canStart ? widgets::PillKind::Ok : widgets::PillKind::Neutral, state.server == 0,
                  canStart)) {
        state.server = 0;
    }
    if (state.server == 0 && !canStart) {
        state.server = 1;
    }
    char detail[220];
    std::snprintf(detail, sizeof(detail), "%s", state.defaultEndpoint.c_str());
    if (serverRow("##mock", "Server on the default socket", detail,
                  state.defaultAvailable ? "ready" : "not running",
                  state.defaultAvailable ? widgets::PillKind::Ok : widgets::PillKind::Warn,
                  state.server == 1, true)) {
        state.server = 1;
    }
    if (state.showOther) {
        if (serverRow("##other", "Other server",
                      state.otherEndpoint[0] != '\0' ? state.otherEndpoint
                                                     : "unix:/path/to.sock or tcp://host:port",
                      "typed", widgets::PillKind::Neutral, state.server == 2, true)) {
            state.server = 2;
        }
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputTextWithHint("##endpoint", "tcp://host:7447", state.otherEndpoint,
                                     sizeof(state.otherEndpoint))) {
            state.server = 2;
        }
        ImGui::PopFont();
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
    }

    // What connecting will do.
    const std::string endpoint = state.server == 2 ? state.otherEndpoint : state.defaultEndpoint;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(12.0F), px(10.0F)));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg1);
    ImGui::BeginChild("##detail", ImVec2(0.0F, 0.0F),
                      ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    widgets::PropertyRow("protocol", "0.1 · this viewer", nullptr);
    widgets::PropertyRow("transport",
                         endpoint.rfind("tcp", 0) == 0 ? "TCP" : "Unix socket on this machine",
                         nullptr);
    widgets::PropertyRow(
        "open with",
        state.openTarget.empty() ? "what the server is showing" : state.openTarget.c_str(),
        nullptr);
    ImGui::EndChild();
    ImGui::Dummy(ImVec2(0.0F, px(8.0F)));

    // The one primary action of the screen.
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, px(34.0F));
    ImGui::BeginDisabled(endpoint.empty());
    const bool clicked = ImGui::InvisibleButton("##connect", size);
    const bool hovered = ImGui::IsItemHovered();
    ImGui::EndDisabled();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(
        min, ImVec2(min.x + size.x, min.y + size.y),
        u32(endpoint.empty() ? c::bg3 : (hovered ? theme::hex(0xF08A45) : c::accent)), px(3.0F));
    const char* label = "Connect and open";
    ImFont* font = theme::fonts().sansSemiBold;
    const float fontSize = px(14.0F);
    const ImVec2 extent = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0F, label);
    draw->AddText(font, fontSize,
                  ImVec2(min.x + (size.x - extent.x) * 0.5F, min.y + (size.y - extent.y) * 0.5F),
                  u32(endpoint.empty() ? c::t3 : c::onAccent), label);
    if (clicked || (!endpoint.empty() && !ImGui::GetIO().WantTextInput &&
                    ImGui::IsKeyPressed(ImGuiKey_Enter, false))) {
        state.connectRequested = true;
        state.startLocalRequested = state.server == 0;
        state.connectEndpoint = endpoint;
    }
    if (!state.error.empty()) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::Pill("error", widgets::PillKind::Error);
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::PushTextWrapPos(0.0F);
        ImGui::TextUnformatted(state.error.c_str());
        ImGui::PopTextWrapPos();
    }
    if (!state.defaultAvailable && state.server == 1) {
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        widgets::Caption("Start one in a terminal, the simulator or the recorded sample:");
        widgets::Mono(c::t2, "ros2 run lagrange_server lagrange_server");
        widgets::Mono(c::t2, "ros2 run lagrange_mock_server lagrange_mock_server --sample");
    }
    dropZone(state);
}

void column(const char* id, const ImVec2& position, const ImVec2& size, const ImVec4& background,
            const ImVec2& padding) {
    ImGui::SetNextWindowPos(position);
    ImGui::SetNextWindowSize(size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, padding);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, background);
    ImGui::Begin(id, nullptr, kFixed);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}

}  // namespace

void drawLauncher(PanelContext& context) {
    LauncherState& state = *context.launcher;
    ImGuiViewport* viewport = ImGui::GetMainViewport();

    // Menu bar: the mark and name, File and Help, the Ctrl K hint (design: no workspace bar
    // and no domain strip on this screen).
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(8.0F), px(7.5F)));
    if (ImGui::BeginMainMenuBar()) {
        ImGui::PopStyleVar();
        const float bar = ImGui::GetWindowHeight();
        const float mark = px(16.0F);
        widgets::DrawLogo(
            ImGui::GetWindowDrawList(),
            ImVec2(ImGui::GetCursorScreenPos().x, ImGui::GetWindowPos().y + (bar - mark) * 0.5F),
            mark);
        ImGui::Dummy(ImVec2(mark, 1.0F));
        ImGui::SameLine(0.0F, px(7.0F));
        text(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, "Lagrange");
        ImGui::SameLine(0.0F, px(16.0F));
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        if (ImGui::BeginMenu("File")) {
            ImGui::PushStyleColor(ImGuiCol_Text, c::t1);
            if (ImGui::MenuItem("Open robot or recording...", "Ctrl+O", false,
                                context.canOpenFiles)) {
                context.openFileRequested = true;
            }
            ImGui::PopStyleColor();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            ImGui::MenuItem("Lagrange viewer, v0.1 in development", nullptr, false, false);
            ImGui::MenuItem("Apache-2.0 · fonts: IBM Plex, SIL OFL 1.1", nullptr, false, false);
            ImGui::EndMenu();
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        widgets::AlignRight(px(44.0F));
        widgets::Kbd("Ctrl K");
        ImGui::EndMainMenuBar();
    } else {
        ImGui::PopStyleVar();
    }
    if (context.canOpenFiles && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) {
        context.openFileRequested = true;
    }

    // Status bar.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(10.0F), px(4.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    if (ImGui::BeginViewportSideBar("##launcher-status", viewport, ImGuiDir_Down, px(24.0F),
                                    kFixed | ImGuiWindowFlags_NoNav)) {
        const SystemInfo& system = state.system;
        widgets::Dot(c::t3);
        ImGui::SameLine(0.0F, px(6.0F));
        text(theme::fonts().sans, theme::kCaptionSize, c::t1, "Not connected");
        if (!system.rosDistro.empty()) {
            ImGui::SameLine(0.0F, px(16.0F));
            widgets::Caption("ROS 2");
            ImGui::SameLine(0.0F, px(6.0F));
            char ros[120];
            std::snprintf(ros, sizeof(ros), "%s · domain %s%s%s", system.rosDistro.c_str(),
                          system.rosDomain.empty() ? "0" : system.rosDomain.c_str(),
                          system.rmw.empty() ? "" : " · ", system.rmw.c_str());
            text(theme::fonts().mono, theme::kCaptionSize, c::t2, ros);
        }
        char right[320];
        std::snprintf(right, sizeof(right), "GPU  %s · %s     config  %s", system.gpu.c_str(),
                      system.tier.c_str(), system.configDirectory.c_str());
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        const float width = ImGui::CalcTextSize(right).x;
        ImGui::PopFont();
        ImGui::SameLine();
        widgets::AlignRight(width);
        text(theme::fonts().mono, theme::kCaptionSize, c::t2, right);
    }
    ImGui::End();
    ImGui::PopStyleVar(2);

    // Three fixed columns (design: 380 px about, flexible start, 400 px server).
    const ImVec2 origin = viewport->WorkPos;
    const ImVec2 size = viewport->WorkSize;
    const float left = std::min(px(380.0F), size.x * 0.28F);
    const float right = std::min(px(400.0F), size.x * 0.30F);
    column("##launcher-about", origin, ImVec2(left, size.y), c::bg1, ImVec2(px(28.0F), px(36.0F)));
    about(state);
    ImGui::End();
    column("##launcher-start", ImVec2(origin.x + left, origin.y),
           ImVec2(size.x - left - right, size.y), c::bg0, ImVec2(px(28.0F), px(28.0F)));
    start(context, state);
    ImGui::End();
    column("##launcher-server", ImVec2(origin.x + size.x - right, origin.y), ImVec2(right, size.y),
           c::bg1, ImVec2(px(24.0F), px(32.0F)));
    servers(state);
    ImGui::End();
    // Rules between the columns.
    ImDrawList* lines = ImGui::GetForegroundDrawList(viewport);
    lines->AddLine(ImVec2(origin.x + left, origin.y), ImVec2(origin.x + left, origin.y + size.y),
                   u32(c::line));
    lines->AddLine(ImVec2(origin.x + size.x - right, origin.y),
                   ImVec2(origin.x + size.x - right, origin.y + size.y), u32(c::line));
}

}  // namespace lagrange::viewer
