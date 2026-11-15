// The launcher: the first screen, shown until the viewer is connected. System check on the
// left, what to open in the middle, which server on the right
// (docs/design/ui/01-launcher/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <imgui_internal.h>  // BeginViewportSideBar
#include <string>
#include <utility>
#include <vector>

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

// Where the links of the left column and the Help menu go. They open in the system's browser
// (xdg-open, by the application), never inside the viewer (CLAUDE.md invariant 13).
constexpr const char* kDocumentation = "https://open-physical-systems.github.io/Lagrange/";
constexpr const char* kUrdfGuide =
    "https://open-physical-systems.github.io/Lagrange/guide/cli.html#importing-a-robot";
constexpr const char* kGpuIssue =
    "https://github.com/Open-Physical-Systems/Lagrange/issues/new?template=bug.yml";

/// A text link (design: .lnk): info colour, underlined while hovered. True when clicked.
bool link(const char* label, const char* url) {
    ImFont* font = theme::fonts().sans;
    const float size = px(theme::kLabelSize);
    const ImVec2 extent = font->CalcTextSizeA(size, FLT_MAX, 0.0F, label);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(label, extent);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 color = u32(hovered ? c::t1 : c::info);
    draw->AddText(font, size, at, color, label);
    if (hovered) {
        draw->AddLine(ImVec2(at.x, at.y + extent.y), ImVec2(at.x + extent.x, at.y + extent.y),
                      color);
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        ImGui::SetTooltip("%s", url);
    }
    return clicked;
}

/// A dashed outline (design: border-style dashed), 4 px dashes with 3 px gaps.
void dashedRect(ImDrawList* draw, const ImVec2& min, const ImVec2& max, ImU32 color) {
    const float dash = px(4.0F);
    const float period = px(7.0F);
    for (float x = min.x; x < max.x; x += period) {
        const float to = std::min(x + dash, max.x);
        draw->AddLine(ImVec2(x, min.y), ImVec2(to, min.y), color);
        draw->AddLine(ImVec2(x, max.y), ImVec2(to, max.y), color);
    }
    for (float y = min.y; y < max.y; y += period) {
        const float to = std::min(y + dash, max.y);
        draw->AddLine(ImVec2(min.x, y), ImVec2(min.x, to), color);
        draw->AddLine(ImVec2(max.x, y), ImVec2(max.x, to), color);
    }
}

/// "2026-10-14 23:12" as the Recent table shows it (design): "today 23:12", "yesterday",
/// "Oct 14", or "Oct 14 2025" in another year. Anything else is shown as it is.
std::string openedWhen(const std::string& opened) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    if (std::sscanf(opened.c_str(), "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute) != 5 ||
        month < 1 || month > 12) {
        return opened;
    }
    const std::time_t now = std::time(nullptr);
    std::tm today{};
    localtime_r(&now, &today);
    today.tm_hour = 12;  // noon, so that a change of daylight saving time is no whole day
    today.tm_min = 0;
    today.tm_sec = 0;
    today.tm_isdst = -1;
    std::tm when{};
    when.tm_year = year - 1900;
    when.tm_mon = month - 1;
    when.tm_mday = day;
    when.tm_hour = 12;
    when.tm_isdst = -1;
    const long days = std::lround(std::difftime(std::mktime(&today), std::mktime(&when)) / 86400.0);
    char buffer[32];
    if (days == 0) {
        std::snprintf(buffer, sizeof(buffer), "today %02d:%02d", hour, minute);
        return buffer;
    }
    if (days == 1) {
        return "yesterday";
    }
    static constexpr std::array<const char*, 12> kMonths{"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    if (year == today.tm_year + 1900) {
        std::snprintf(buffer, sizeof(buffer), "%s %02d",
                      kMonths.at(static_cast<std::size_t>(month - 1)), day);
    } else {
        std::snprintf(buffer, sizeof(buffer), "%s %02d %d",
                      kMonths.at(static_cast<std::size_t>(month - 1)), day, year);
    }
    return buffer;
}

/// A server as the Recent table names it (design): "localhost · default socket",
/// "localhost · unix socket", or the host of a TCP or TLS address.
std::string serverName(const std::string& endpoint, const std::string& defaultEndpoint) {
    if (endpoint.rfind("unix:", 0) == 0) {
        return endpoint == defaultEndpoint ? "localhost · default socket"
                                           : "localhost · unix socket";
    }
    const std::size_t scheme = endpoint.find("://");
    std::string host = scheme == std::string::npos ? endpoint : endpoint.substr(scheme + 3);
    const std::size_t port = host.rfind(':');
    if (port != std::string::npos && host.find(']') == std::string::npos) {
        host.resize(port);
    }
    return host.empty() ? endpoint : host;
}

/// A stage as the Recent table names it: a file by its name, a relative path or sample as it is.
std::string stageName(const std::string& stage) {
    if (stage.empty() || stage.front() != '/') {
        return stage;
    }
    return stage.substr(stage.rfind('/') + 1);
}

// --- left column: what this is, and the system check ---------------------------------------

/// One row of the system check (design: 12 px line, 11 px detail under it): a mark, a line, and
/// a detail line ending in `accent` (the tier) when given.
void check(bool ok, const char* line, const char* detail, const char* accent = nullptr) {
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float icon = px(12.0F);
    if (ok) {
        widgets::DrawIcon(ImGui::GetWindowDrawList(), widgets::Icon::Check,
                          ImVec2(at.x, at.y + px(3.0F)), icon, u32(c::ok), u32(c::bg1));
    } else {
        ImGui::GetWindowDrawList()->AddText(theme::fonts().sansSemiBold, px(theme::kLabelSize),
                                            ImVec2(at.x + px(3.0F), at.y), u32(c::warn), "!");
    }
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(26.0F));
    ImGui::BeginGroup();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, px(1.0F)));
    text(theme::fonts().sans, theme::kLabelSize, c::t1, line);
    // The accent follows the detail on its line when both fit; else it gets a line of its own
    // (after a wrapped detail, SameLine would put it past the column's edge).
    const float caption = px(theme::kCaptionSize);
    const float room = ImGui::GetContentRegionAvail().x;
    const float detailWidth = theme::fonts().sans->CalcTextSizeA(caption, FLT_MAX, 0.0F, detail).x;
    const bool oneLine =
        accent != nullptr &&
        detailWidth + theme::fonts().sans->CalcTextSizeA(caption, FLT_MAX, 0.0F, accent).x <= room;
    ImGui::PushTextWrapPos(0.0F);
    text(theme::fonts().sans, theme::kCaptionSize, c::t3, detail);
    ImGui::PopTextWrapPos();
    if (accent != nullptr) {
        if (oneLine) {
            ImGui::SameLine(0.0F, 0.0F);
        }
        text(theme::fonts().sans, theme::kCaptionSize, c::accent, accent);
    }
    ImGui::PopStyleVar();
    ImGui::EndGroup();
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
}

/// What a tier means, after its name (CLAUDE.md invariant 9).
const char* tierMeaning(const std::string& tier) {
    if (tier == "Tier 2") {
        return "Tier 2 (full)";
    }
    if (tier == "Tier 1") {
        return "Tier 1 (raster)";
    }
    if (tier == "Tier 0") {
        return "Tier 0 (CPU)";
    }
    return tier.c_str();
}

void about(LauncherState& state) {
    const SystemInfo& system = state.system;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    // Mark (64 px) and wordmark (30 px over the 11 px version line), 16 px apart (design).
    const ImVec2 at = ImGui::GetCursorScreenPos();
    widgets::DrawLogo(draw, at, px(64.0F));
    ImGui::SetCursorScreenPos(ImVec2(at.x + px(80.0F), at.y + px(7.0F)));
    ImGui::BeginGroup();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, px(2.0F)));
    text(theme::fonts().sansSemiBold, 30.0F, c::t1, "lagrange");
    char version[64];
    std::snprintf(version, sizeof(version), "%s · Apache-2.0", LAGRANGE_VIEWER_VERSION);
    text(theme::fonts().mono, theme::kCaptionSize, c::t3, version);
    ImGui::PopStyleVar();
    ImGui::EndGroup();
    ImGui::SetCursorScreenPos(ImVec2(at.x, at.y + px(64.0F)));
    ImGui::Dummy(ImVec2(0.0F, px(14.0F)));
    ImGui::PushTextWrapPos(0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, px(5.0F)));
    text(theme::fonts().sans, theme::kBodySize, c::t2,
         "Coupled multi-physics robot simulation. Turn each physics domain on, off, or replace "
         "it with a substitute, and watch every signal live.");
    ImGui::PopStyleVar();
    ImGui::PopTextWrapPos();
    ImGui::Dummy(ImVec2(0.0F, px(14.0F)));
    widgets::Caption("SYSTEM CHECK");
    ImGui::Dummy(ImVec2(0.0F, px(2.0F)));

    char line[160];
    char detail[200];
    // 1. GPU and the tier it gives.
    std::snprintf(detail, sizeof(detail), "%s%s → ", system.gpuDetail.c_str(),
                  system.softwareGpu ? " · software renderer, slow" : "");
    check(!system.gpu.empty() && !system.softwareGpu,
          system.gpu.empty() ? "No Vulkan device" : system.gpu.c_str(), detail,
          tierMeaning(system.tier));
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

    // Links at the bottom of the column (design), 6 px apart.
    const float lineHeight = px(theme::kLabelSize) * 1.25F;
    const float links = lineHeight * 3.0F + px(6.0F) * 2.0F;
    ImGui::SetCursorPosY(
        std::max(ImGui::GetCursorPosY() + px(16.0F),
                 ImGui::GetWindowHeight() - ImGui::GetStyle().WindowPadding.y - links));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, px(6.0F)));
    for (const auto& [label, url] : {std::pair{"Documentation and tutorials", kDocumentation},
                                     std::pair{"Bring your own URDF — guide", kUrdfGuide},
                                     std::pair{"Report a GPU or driver issue", kGpuIssue}}) {
        if (link(label, url)) {
            state.openPageRequested = url;
        }
    }
    ImGui::PopStyleVar();
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
    // Shortcut hint at the right; the title and what it takes are cut before it (ellipsis).
    // In a narrow tile the title comes first: the hint goes (it stays in the tooltip).
    const float size = px(10.0F);
    const ImVec2 extent = fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, shortcut);
    const ImVec2 kbdMin(max.x - extent.x - px(22.0F), min.y + (height - px(16.0F)) * 0.5F);
    const float titleWidth = fonts.sansMedium->CalcTextSizeA(px(14.0F), FLT_MAX, 0.0F, title).x;
    const bool showShortcut = kbdMin.x - px(8.0F) - x >= titleWidth;
    const float room = (showShortcut ? kbdMin.x - px(8.0F) : max.x - px(12.0F)) - x;
    const bool cutTitle = fit::draw(draw, fonts.sansMedium, px(14.0F), ImVec2(x, min.y + px(14.0F)),
                                    u32(c::t1), title, room);
    const bool cutDetail = fit::draw(draw, fonts.sans, px(theme::kLabelSize),
                                     ImVec2(x, min.y + px(36.0F)), u32(c::t3), detail, room);
    if ((cutTitle || cutDetail || !showShortcut) && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s  (%s)\n%s", title, shortcut, detail);
    }
    if (!showShortcut) {
        return clicked;
    }
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
    /// What lagrange_server is started with for this class's sample: arguments separated by
    /// spaces, "@" before a path in lagrange_assets' share. "" is the server's default (the
    /// sample AMR, which any server shows), nullptr no sample yet.
    const char* sample;
    const char* openWith;  ///< The detail box's "open with" for the sample.
};

using D = theme::Domain;
// The samples are those of the `lagrange` command (tools/cli/lagrange, `lagrange samples`).
constexpr std::array<RobotClass, 8> kClasses{{
    {"Mobile robot / AMR",
     "diff-drive in a warehouse · lidar, camera, IMU",
     "Nav2 · ros2_control",
     {D::RigidBody, D::Electrical, D::Thermal, D::Emi, D::Sensors},
     5,
     "",
     "the sample AMR · warehouse"},
    {"Robot arm",
     "6-DoF arm on a table · gripper contact",
     "MoveIt 2 · ros2_control",
     {D::RigidBody, D::Electrical, D::Thermal, D::RigidBody, D::RigidBody},
     3,
     "--robot @robots/arm6/arm6.urdf --overlay @robots/arm6/arm6.overlay.yaml "
     "--world @worlds/workcell_arm6/workcell_arm6.urdf",
     "arm6.urdf · workcell_arm6.urdf"},
    {"Drone",
     "quadrotor · gusts, battery sag, GNSS",
     "PX4 · ArduPilot SITL",
     {D::RigidBody, D::Electrical, D::Aerodynamics, D::Weather, D::Sensors},
     5,
     "--robot @robots/quad_x500/quad_x500.usda --world @worlds/open_field/open_field.usda "
     "--air-density 1.225 --geo-origin 47.397742,8.545594,488",
     "quad_x500.usda · open_field.usda"},
    {"Car",
     "Pacejka tires, suspension, EV powertrain",
     "ros2_control · CAN",
     {D::RigidBody, D::Electrical, D::Thermal, D::Weather, D::RigidBody},
     4,
     "--robot @robots/sedan/sedan.usda --world @worlds/city_block/city_block.usda",
     "sedan.usda · city_block.usda"},
    {"Autonomous car",
     "city block · traffic, full sensor suite",
     "Autoware · SUMO",
     {D::RigidBody, D::Electrical, D::Weather, D::Emi, D::Sensors},
     5,
     "--robot @robots/sedan/sedan.usda --world @worlds/town/town.usda --spawn 30,-1.75,0.6,0 "
     "--no-demo --traffic vehicles=24,pedestrians=36,seed=1",
     "sedan.usda · town.usda · traffic"},
    {"Huge world",
     "geospatial terrain · tile streaming, fleets",
     "3D Tiles · OSM · DEM",
     {D::RigidBody, D::Aerodynamics, D::Weather, D::Sensors, D::RigidBody},
     4,
     "--world @worlds/tiled_town/tiled_town.usda --spawn 1036,1036",
     "the sample AMR · tiled_town.usda"},
    {"Electrical bench",
     "physics off · replay a real run through the power train",
     "MCAP replay",
     {D::Electrical, D::Thermal, D::RigidBody, D::RigidBody, D::RigidBody},
     2,
     nullptr,  // needs a recording of a real run; none ships (Import or drop an .mcap)
     ""},
    {"Empty stage",
     "ground plane and sun · add robots later",
     "—",
     {D::RigidBody, D::RigidBody, D::RigidBody, D::RigidBody, D::RigidBody},
     1,
     "--robot none --world none",
     "the ground only"},
}};

/// Can this class's sample be shown? The default sample by any server; the others by starting
/// lagrange_server here, which needs it and lagrange_assets installed and the default socket
/// free.
bool canRun(const LauncherState& state, const RobotClass& item) {
    if (item.sample == nullptr) {
        return false;
    }
    if (*item.sample == '\0') {
        return true;
    }
    return !state.localServer.empty() && !state.assetDirectory.empty() && !state.defaultAvailable;
}

/// Why a class cannot run now (tooltip), or nullptr.
const char* whyNot(const LauncherState& state, const RobotClass& item) {
    if (item.sample == nullptr) {
        return "No sample of this robot class yet: open a recording of a real run (Ctrl R)";
    }
    if (*item.sample == '\0') {
        return nullptr;
    }
    if (state.localServer.empty()) {
        return "Needs lagrange_server installed to start its sample";
    }
    if (state.assetDirectory.empty()) {
        return "Needs lagrange_assets installed (the sample robots and worlds)";
    }
    if (state.defaultAvailable) {
        return "A server already runs on the default socket: stop it to start this sample";
    }
    return nullptr;
}

/// lagrange_server's arguments for a sample: split at spaces, "@" made a path in the assets.
std::vector<std::string> sampleArguments(const LauncherState& state, const RobotClass& item) {
    std::vector<std::string> arguments;
    if (item.sample == nullptr) {
        return arguments;
    }
    const std::string_view sample = item.sample;
    std::size_t from = 0;
    while (from < sample.size()) {
        std::size_t end = sample.find(' ', from);
        end = end == std::string_view::npos ? sample.size() : end;
        const std::string_view word = sample.substr(from, end - from);
        if (!word.empty()) {
            arguments.push_back(word.front() == '@'
                                    ? state.assetDirectory + "/" + std::string(word.substr(1))
                                    : std::string(word));
        }
        from = end + 1;
    }
    return arguments;
}

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
        default:  // empty stage: a 20 px icon, M8 3v10M3 8h10 in a 16 unit box
            draw->AddLine(at(32, 13.75F), at(32, 26.25F), ink, 1.5F * s);
            draw->AddLine(at(25.75F, 20), at(38.25F, 20), ink, 1.5F * s);
            break;
    }
}

/// A robot-class card (design: .tpl): 1 = clicked, 2 = double-clicked, 0 = neither.
int card(int index, const RobotClass& item, const char* unavailable, bool selected,
         const ImVec2& size) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImGui::PushID(index);
    const bool clicked = ImGui::InvisibleButton("##class", size);
    ImGui::PopID();
    const bool hovered = ImGui::IsItemHovered();
    const bool twice = hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
    const bool available = unavailable == nullptr;
    const bool empty = index == static_cast<int>(kClasses.size()) - 1;  // dashed (design)
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(hovered && available ? c::bg2 : c::bg1), px(4.0F));
    if (selected) {
        draw->AddRect(min, max, u32(c::accent), px(4.0F), 0, px(2.0F));
    } else if (empty) {
        dashedRect(draw, min, max, u32(c::line));
    } else {
        draw->AddRect(min, max, u32(c::line), px(4.0F));
    }
    const float pad = px(12.0F);
    const ImVec2 artMin(min.x + pad, min.y + pad);
    const ImVec2 artMax(max.x - pad, artMin.y + px(56.0F));
    draw->AddRectFilled(artMin, artMax, u32(c::bg0), px(3.0F));
    if (empty) {
        dashedRect(draw, artMin, artMax, u32(c::line));
    } else {
        draw->AddRect(artMin, artMax, u32(c::line), px(3.0F));
    }
    const ImVec4 ink = available ? c::t2 : c::t3;
    art(draw, index, ImVec2((artMin.x + artMax.x) * 0.5F, (artMin.y + artMax.y) * 0.5F), u32(ink),
        u32(c::bg0));

    const theme::Fonts& fonts = theme::fonts();
    const float inner = size.x - pad * 2.0F;
    float y = artMax.y + px(10.0F);
    const bool cutName =
        fit::draw(draw, fonts.sansMedium, px(theme::kBodySize), ImVec2(min.x + pad, y),
                  u32(available ? c::t1 : c::t2), item.name, inner);
    y += px(24.0F);
    // The description wraps; what does not fit above the bottom line is cut (all of it is in
    // the tooltip).
    const float bottom = max.y - pad - px(14.0F);
    const float small = px(theme::kCaptionSize);
    const ImVec2 wrapped = fonts.sans->CalcTextSizeA(small, FLT_MAX, inner, item.description);
    const bool cutDescription = y + wrapped.y > bottom - px(4.0F);
    draw->PushClipRect(ImVec2(min.x + pad, y), ImVec2(max.x - pad, bottom - px(4.0F)), true);
    draw->AddText(fonts.sans, small, ImVec2(min.x + pad, y), u32(c::t3), item.description, nullptr,
                  inner);
    draw->PopClipRect();
    // Bottom line: the domains the class turns on, and what it integrates with after them.
    for (int d = 0; d < item.domainCount; ++d) {
        draw->AddCircleFilled(
            ImVec2(min.x + pad + px(4.0F) + px(11.0F) * static_cast<float>(d), bottom + px(7.0F)),
            px(3.5F),
            u32(theme::kDomainColor[static_cast<int>(
                item.domains.at(static_cast<std::size_t>(d)))]));
    }
    const float dots = px(11.0F) * static_cast<float>(item.domainCount) + px(6.0F);
    const std::string integrations =
        fit::cut(fonts.sans, small, item.integrations, std::max(inner - dots, 0.0F));
    const float width = fonts.sans->CalcTextSizeA(small, FLT_MAX, 0.0F, integrations.c_str()).x;
    draw->AddText(fonts.sans, small, ImVec2(max.x - pad - width, bottom), u32(c::t3),
                  integrations.c_str());
    const bool cut = cutName || cutDescription || integrations != item.integrations;
    if (hovered && (!available || cut)) {
        ImGui::SetTooltip("%s\n%s\n%s%s%s", item.name, item.description, item.integrations,
                          available ? "" : "\n\n", available ? "" : unavailable);
    }
    if (!available) {
        return 0;
    }
    return twice ? 2 : (clicked ? 1 : 0);
}

/// Picks a robot class; its sample is started here, so Start local simulator is chosen when it
/// can be. `connect`: and connect at once (double-click, Enter).
void pickClass(LauncherState& state, int index, bool connect) {
    const RobotClass& item = kClasses.at(static_cast<std::size_t>(index));
    state.robotClass = index;
    state.openTarget.clear();  // the class's sample, not a file chosen before
    if (item.sample != nullptr && *item.sample != '\0' && canRun(state, item)) {
        state.server = 0;
    }
    if (connect) {
        state.connectRequested = true;
        state.startLocalRequested = state.server == 0;
        state.startArguments =
            state.server == 0 ? sampleArguments(state, item) : std::vector<std::string>{};
        state.connectEndpoint =
            state.server == 2 ? std::string(state.otherEndpoint) : state.defaultEndpoint;
    }
}

/// Opens a Recent entry again: the same server and the same thing (double-click, File menu).
void reopen(LauncherState& state, const RecentEntry& entry) {
    state.connectRequested = true;
    state.startLocalRequested = false;
    state.startArguments.clear();
    state.connectEndpoint = entry.endpoint;
    // A stage that is a file is opened again; a sample comes from the server.
    state.openTarget = entry.stage.find('/') == 0 ? entry.stage : std::string{};
}

/// The action tiles: the design's three in a row, or, with Build a robot (RFC 0016 open
/// question 6), four two by two (four in one row would cut their titles at the design's window
/// width). The owner has not reviewed the two-by-two arrangement yet: false here puts the
/// design's row back, and Build a robot stays in the File menu and on Ctrl B.
constexpr bool kBuildTile = true;

void start(PanelContext& context, LauncherState& state) {
    const float width = ImGui::GetContentRegionAvail().x;
    const float gap = px(10.0F);
    const int perRow = kBuildTile ? 2 : 3;
    const float tile = (width - gap * static_cast<float>(perRow - 1)) / static_cast<float>(perRow);
    // The file is read by the server, so a tile only chooses what to open after connecting.
    const bool canPick = context.canOpenFiles;
    const ImVec2 firstRow = ImGui::GetCursorScreenPos();
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
    if (kBuildTile) {
        ImGui::SetCursorScreenPos(ImVec2(firstRow.x, firstRow.y + px(64.0F) + gap));
    } else {
        ImGui::SameLine(0.0F, gap);
    }
    if (action("##open-recording", widgets::Icon::Replay, "Open recording", ".mcap → replay mode",
               "Ctrl R", tile)) {
        context.openFileRequested = true;
    }
    ImGui::EndDisabled();
    // Building needs no server: it opens the Builder workspace without connecting (R1).
    if (kBuildTile) {
        ImGui::SameLine(0.0F, gap);
        if (action("##build-robot", widgets::Icon::Builder, "Build a robot",
                   "links · joints · sensors, no server", "Ctrl B", tile)) {
            state.builderRequested = true;
        }
    }

    ImGui::Dummy(ImVec2(0.0F, px(14.0F)));
    text(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, "Start from a robot class");
    ImGui::SameLine(0.0F, px(6.0F));
    text(theme::fonts().sans, theme::kLabelSize, c::t3, "sample robot, world and sim profile");
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    // Four cards a row (design); three when four would be too narrow for their names.
    const int cardsPerRow = (width - gap * 3.0F) / 4.0F < px(128.0F) ? 3 : 4;
    const ImVec2 cardSize(
        (width - gap * static_cast<float>(cardsPerRow - 1)) / static_cast<float>(cardsPerRow),
        px(178.0F));
    for (int i = 0; i < static_cast<int>(kClasses.size()); ++i) {
        if (i % cardsPerRow != 0) {
            ImGui::SameLine(0.0F, gap);
        }
        const RobotClass& item = kClasses.at(static_cast<std::size_t>(i));
        const int pressed = card(i, item, whyNot(state, item), state.robotClass == i, cardSize);
        if (pressed != 0) {
            pickClass(state, i, pressed == 2);
        }
        if (i % cardsPerRow == cardsPerRow - 1) {
            ImGui::Dummy(ImVec2(0.0F, gap - ImGui::GetStyle().ItemSpacing.y));
        }
    }

    // Recent.
    ImGui::Dummy(ImVec2(0.0F, px(10.0F)));
    const ImVec2 heading = ImGui::GetCursorPos();
    text(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, "Recent");
    ImGui::SetCursorPos(ImVec2(heading.x + width - px(220.0F), heading.y - px(4.0F)));
    widgets::SearchField("##recent-filter", "Filter recent", state.recentFilter,
                         sizeof(state.recentFilter), px(220.0F));
    ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, c::bg1);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    // The rest of the column, but never less than a few rows: in a small window the column
    // scrolls to it instead.
    const float recentHeight = std::max(ImGui::GetContentRegionAvail().y, px(150.0F));
    ImGui::BeginChild("##recent", ImVec2(0.0F, recentHeight), ImGuiChildFlags_Borders);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    const std::vector<RecentEntry>& recent = state.recent.entries();
    if (recent.empty()) {
        ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
        widgets::Caption("Nothing opened yet. What you open is listed here.");
    } else if (ImGui::BeginTable("##recent-table", 5,
                                 ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
                                     ImGuiTableFlags_PadOuterX)) {
        ImGui::TableSetupColumn("stage", ImGuiTableColumnFlags_None, 1.6F);
        ImGui::TableSetupColumn("robot", ImGuiTableColumnFlags_None, 1.1F);
        ImGui::TableSetupColumn("profile", ImGuiTableColumnFlags_None, 2.7F);
        ImGui::TableSetupColumn("server", ImGuiTableColumnFlags_None, 1.6F);
        ImGui::TableSetupColumn("opened", ImGuiTableColumnFlags_None, 1.1F);
        ImGui::TableNextRow(ImGuiTableRowFlags_None, px(24.0F));
        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::bg2));
        for (const char* name : {"stage", "robot", "profile", "server", "opened"}) {
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            if (std::strcmp(name, "opened") == 0) {
                widgets::AlignRight(ImGui::CalcTextSize(name).x);
            }
            widgets::Caption("%s", name);
        }
        const float rowHeight = px(23.0F);
        for (std::size_t i = 0; i < recent.size(); ++i) {
            const RecentEntry& entry = recent[i];
            if (state.recentFilter[0] != '\0' && !containsNoCase(entry.stage, state.recentFilter) &&
                !containsNoCase(entry.robot, state.recentFilter) &&
                !containsNoCase(entry.profile, state.recentFilter) &&
                !containsNoCase(entry.endpoint, state.recentFilter)) {
                continue;
            }
            const bool selected = state.recentRow == static_cast<int>(i);
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow(ImGuiTableRowFlags_None, rowHeight);
            if (selected) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, u32(c::accentWeak));
            }
            ImGui::TableNextColumn();
            const ImVec2 rowMin = ImGui::GetCursorScreenPos();
            // Click selects; double-click: the same server and the same thing again.
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0, 0, 0, 0));
            if (ImGui::Selectable(
                    "##row", false,
                    ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
                    ImVec2(0.0F, rowHeight - px(4.0F)))) {
                state.recentRow = static_cast<int>(i);
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    reopen(state, entry);
                }
            }
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered() && !entry.stage.empty()) {
                ImGui::SetTooltip("%s\n%s\nopened %s", entry.stage.c_str(), entry.endpoint.c_str(),
                                  entry.opened.c_str());
            }
            if (selected) {  // the design's 2 px accent bar at the row's left
                const float pad = ImGui::GetStyle().CellPadding.x;
                ImGui::GetWindowDrawList()->AddRectFilled(
                    ImVec2(rowMin.x - pad, rowMin.y - ImGui::GetStyle().CellPadding.y),
                    ImVec2(rowMin.x - pad + px(2.0F), rowMin.y + rowHeight), u32(c::accent));
            }
            ImGui::SameLine();
            ImGui::SetCursorScreenPos(ImVec2(rowMin.x, ImGui::GetCursorScreenPos().y));
            fit::text(stageName(entry.stage), ImGui::GetContentRegionAvail().x, true);
            ImGui::TableNextColumn();
            fit::text(entry.robot, ImGui::GetContentRegionAvail().x);
            ImGui::TableNextColumn();
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            fit::text(entry.profile, ImGui::GetContentRegionAvail().x);
            ImGui::PopFont();
            ImGui::TableNextColumn();
            fit::text(serverName(entry.endpoint, state.defaultEndpoint),
                      ImGui::GetContentRegionAvail().x);
            ImGui::TableNextColumn();
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            const std::string when = openedWhen(entry.opened);
            widgets::AlignRight(ImGui::CalcTextSize(when.c_str()).x);
            ImGui::TextUnformatted(when.c_str());
            ImGui::PopFont();
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
    const float pillWidth = widgets::PillWidth(pill);
    const float room = max.x - pillWidth - px(18.0F) - x;
    const bool cutName =
        fit::draw(draw, fonts.sansMedium, px(theme::kBodySize), ImVec2(x, min.y + px(10.0F)),
                  u32(enabled ? c::t1 : c::t2), name, room);
    const bool cutDetail = fit::draw(draw, fonts.mono, px(theme::kCaptionSize),
                                     ImVec2(x, min.y + px(29.0F)), u32(c::t3), detail, room, true);
    if ((cutName || cutDetail) && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s\n%s", name, detail);
    }
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
    dashedRect(draw, min, max, u32(c::lineStrong));
    const float icon = px(22.0F);
    widgets::DrawIcon(draw, widgets::Icon::Download,
                      ImVec2((min.x + max.x - icon) * 0.5F, min.y + px(14.0F)), icon, u32(c::t2),
                      u32(c::bg1));
    const theme::Fonts& fonts = theme::fonts();
    const float size = px(theme::kBodySize);
    const char* first = state.openTarget.empty() ? "Drop a URDF, SDF, MJCF, USD or MCAP file"
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
    text(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, "Simulation server");
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

    // What connecting will do (design: bg0 box, 110 px keys in t3, mono values).
    const std::string endpoint = state.server == 2 ? state.otherEndpoint : state.defaultEndpoint;
    const RobotClass& chosen = kClasses.at(static_cast<std::size_t>(state.robotClass));
    char protocol[48];
    std::snprintf(protocol, sizeof(protocol), "%u.%u · this viewer", kProtocolMajor,
                  kProtocolMinor);
    const bool local = endpoint.rfind("unix:", 0) == 0;
    char tier[64];
    // A server on this machine draws on this GPU; a remote one tells its tier on connecting.
    std::snprintf(tier, sizeof(tier), "%s", local ? state.system.tier.c_str() : "on connecting");
    std::string openWith = state.openTarget;
    if (openWith.empty()) {
        openWith = state.server == 0 && canRun(state, chosen) ? chosen.openWith
                                                              : "what the server is showing";
    }
    const std::array<std::pair<const char*, const char*>, 4> rows{{
        {"protocol", protocol},
        {"transport", local ? "unix socket · this machine" : "TCP"},
        {"server tier", tier},
        {"open with", openWith.c_str()},
    }};
    {
        const theme::Fonts& fonts = theme::fonts();
        const float pad = px(10.0F);
        const float rowHeight = px(16.0F);
        const float rowGap = px(6.0F);
        const float boxWidth = ImGui::GetContentRegionAvail().x;
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const ImVec2 max(min.x + boxWidth, min.y + pad * 2.0F + rowHeight * 4.0F + rowGap * 3.0F);
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(min, max, u32(c::bg0), px(4.0F));
        draw->AddRect(min, max, u32(c::line), px(4.0F));
        const float size = px(theme::kLabelSize);
        const float valueX = min.x + pad + px(110.0F);
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const float y = min.y + pad + (rowHeight + rowGap) * static_cast<float>(i) + px(1.0F);
            draw->AddText(fonts.sans, size, ImVec2(min.x + pad, y), u32(c::t3), rows.at(i).first);
            ImFont* font = i == 3 ? fonts.sans : fonts.mono;  // "open with" is in sans (design)
            fit::draw(draw, font, size, ImVec2(valueX, y), u32(c::t1), rows.at(i).second,
                      max.x - pad - valueX, i == 3);
        }
        ImGui::Dummy(ImVec2(boxWidth, max.y - min.y));
    }
    ImGui::Dummy(ImVec2(0.0F, px(6.0F)));

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
    ImFont* font = theme::fonts().sansMedium;
    const float fontSize = px(theme::kBodySize);
    const ImVec2 extent = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0F, label);
    draw->AddText(font, fontSize,
                  ImVec2(min.x + (size.x - extent.x) * 0.5F, min.y + (size.y - extent.y) * 0.5F),
                  u32(endpoint.empty() ? c::t3 : c::onAccent), label);
    if (clicked || (!endpoint.empty() && !ImGui::GetIO().WantTextInput &&
                    ImGui::IsKeyPressed(ImGuiKey_Enter, false))) {
        state.connectRequested = true;
        state.startLocalRequested = state.server == 0;
        state.startArguments = state.server == 0 && canRun(state, chosen)
                                   ? sampleArguments(state, chosen)
                                   : std::vector<std::string>{};
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
            const ImVec2& padding, ImGuiWindowFlags flags = kFixed) {
    ImGui::SetNextWindowPos(position);
    ImGui::SetNextWindowSize(size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, padding);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, background);
    ImGui::Begin(id, nullptr, flags);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}

/// The system check as text, for a bug report (Tools → Copy system check).
std::string systemReport(const SystemInfo& system) {
    char buffer[1024];
    std::snprintf(buffer, sizeof(buffer),
                  "Lagrange viewer %s\nGPU: %s (%s) -> %s%s\nOS: %s, %s session, HiDPI %.2f\n"
                  "ROS 2: %s, RMW %s, ROS_DOMAIN_ID %s\n",
                  LAGRANGE_VIEWER_VERSION, system.gpu.empty() ? "none" : system.gpu.c_str(),
                  system.gpuDetail.c_str(), system.tier.c_str(),
                  system.softwareGpu ? " (software renderer)" : "",
                  system.os.empty() ? "Linux" : system.os.c_str(),
                  system.session.empty() ? "unknown" : system.session.c_str(),
                  static_cast<double>(system.scale),
                  system.rosDistro.empty() ? "not sourced" : system.rosDistro.c_str(),
                  system.rmw.empty() ? "default" : system.rmw.c_str(),
                  system.rosDomain.empty() ? "0" : system.rosDomain.c_str());
    return buffer;
}

/// The launcher's menu bar (design: mark and name, a rule, File, View, Tools, Help, and the
/// Ctrl K hint at the right). Only what works before connecting is in it.
void drawMenuBar(PanelContext& context, LauncherState& state) {
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(8.0F), px(7.5F)));
    if (!ImGui::BeginMainMenuBar()) {
        ImGui::PopStyleVar();
        return;
    }
    ImGui::PopStyleVar();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float bar = ImGui::GetWindowHeight();
    const float mark = px(16.0F);
    widgets::DrawLogo(
        draw, ImVec2(ImGui::GetCursorScreenPos().x, ImGui::GetWindowPos().y + (bar - mark) * 0.5F),
        mark);
    ImGui::Dummy(ImVec2(mark, 1.0F));
    ImGui::SameLine(0.0F, px(7.0F));
    text(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, "Lagrange");
    ImGui::SameLine(0.0F, px(12.0F));
    const ImVec2 rule = ImGui::GetCursorScreenPos();
    const float top = ImGui::GetWindowPos().y + (bar - px(18.0F)) * 0.5F;
    draw->AddLine(ImVec2(rule.x, top), ImVec2(rule.x, top + px(18.0F)), u32(c::line));
    ImGui::Dummy(ImVec2(px(4.0F), 1.0F));
    ImGui::SameLine(0.0F, 0.0F);

    ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(px(8.0F), px(4.0F)));
    const auto menu = [](const char* label) {
        const bool shown = ImGui::BeginMenu(label);
        if (shown) {
            ImGui::PushStyleColor(ImGuiCol_Text, c::t1);  // items inside are in primary text
        }
        return shown;
    };
    const auto endMenu = [] {
        ImGui::PopStyleColor();
        ImGui::EndMenu();
    };
    const bool canStart = !state.localServer.empty() && !state.defaultAvailable;
    if (menu("File")) {
        // One file dialog takes all three; what was chosen decides (stage, robot, recording).
        if (ImGui::MenuItem("Open stage…", "Ctrl+O", false, context.canOpenFiles)) {
            context.openFileRequested = true;
        }
        if (ImGui::MenuItem("Import robot…", "Ctrl+I", false, context.canOpenFiles)) {
            context.openFileRequested = true;
        }
        if (ImGui::MenuItem("Open recording…", "Ctrl+R", false, context.canOpenFiles)) {
            context.openFileRequested = true;
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Build a robot", "Ctrl+B")) {
            state.builderRequested = true;
        }
        ImGui::Separator();
        const std::vector<RecentEntry>& recent = state.recent.entries();
        if (ImGui::BeginMenu("Open recent", !recent.empty())) {
            for (std::size_t i = 0; i < recent.size(); ++i) {
                const RecentEntry& entry = recent[i];
                char label[256];
                std::snprintf(label, sizeof(label), "%s  ·  %s  ·  %s##recent%zu",
                              stageName(entry.stage).c_str(),
                              serverName(entry.endpoint, state.defaultEndpoint).c_str(),
                              openedWhen(entry.opened).c_str(), i);
                if (ImGui::MenuItem(label)) {
                    state.recentRow = static_cast<int>(i);
                    reopen(state, entry);
                }
            }
            ImGui::EndMenu();
        }
        endMenu();
    }
    if (menu("View")) {
        // The robot classes, as the cards pick them.
        for (int i = 0; i < static_cast<int>(kClasses.size()); ++i) {
            const RobotClass& item = kClasses.at(static_cast<std::size_t>(i));
            if (ImGui::MenuItem(item.name, nullptr, state.robotClass == i,
                                whyNot(state, item) == nullptr)) {
                pickClass(state, i, false);
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Server address field", nullptr, state.showOther)) {
            state.showOther = !state.showOther;
            state.server = state.showOther ? 2 : 1;
        }
        if (ImGui::MenuItem("Builder workspace", "Ctrl+B")) {
            state.builderRequested = true;
        }
        endMenu();
    }
    if (menu("Tools")) {
        if (ImGui::MenuItem("Command palette", "Ctrl+K")) {
            openCommandPalette();
        }
        if (ImGui::MenuItem("Start local simulator", nullptr, false, canStart)) {
            state.server = 0;
            const RobotClass& chosen = kClasses.at(static_cast<std::size_t>(state.robotClass));
            state.connectRequested = true;
            state.startLocalRequested = true;
            state.startArguments =
                canRun(state, chosen) ? sampleArguments(state, chosen) : std::vector<std::string>{};
            state.connectEndpoint = state.defaultEndpoint;
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Copy system check")) {
            ImGui::SetClipboardText(systemReport(state.system).c_str());
        }
        if (ImGui::MenuItem("Open configuration folder", nullptr, false,
                            !state.system.configDirectory.empty())) {
            state.openPageRequested = state.system.configDirectory;
        }
        endMenu();
    }
    if (menu("Help")) {
        if (ImGui::MenuItem("Documentation and tutorials")) {
            state.openPageRequested = kDocumentation;
        }
        if (ImGui::MenuItem("Bring your own URDF — guide")) {
            state.openPageRequested = kUrdfGuide;
        }
        if (ImGui::MenuItem("Report a GPU or driver issue")) {
            state.openPageRequested = kGpuIssue;
        }
        ImGui::Separator();
        ImGui::MenuItem("Lagrange viewer " LAGRANGE_VIEWER_VERSION, nullptr, false, false);
        ImGui::MenuItem("Apache-2.0 · fonts: IBM Plex, SIL OFL 1.1", nullptr, false, false);
        endMenu();
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    ImGui::SameLine();
    widgets::AlignRight(px(44.0F));
    widgets::Kbd("Ctrl K");
    ImGui::EndMainMenuBar();
}

}  // namespace

void drawLauncher(PanelContext& context) {
    LauncherState& state = *context.launcher;
    ImGuiViewport* viewport = ImGui::GetMainViewport();

    drawMenuBar(context, state);
    // Shortcuts of the tiles and the File menu.
    if (context.canOpenFiles && (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O) ||
                                 ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_I) ||
                                 ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_R))) {
        context.openFileRequested = true;
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_B)) {
        state.builderRequested = true;
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
        // The configuration folder under the home folder as ~/... (design).
        std::string config = system.configDirectory;
        const char* home = std::getenv("HOME");
        if (home != nullptr && *home != '\0' && config.rfind(home, 0) == 0 &&
            (config.size() == std::strlen(home) || config[std::strlen(home)] == '/')) {
            config = "~" + config.substr(std::strlen(home));
        }
        char right[320];
        std::snprintf(right, sizeof(right), "GPU  %s · %s     config  %s/", system.gpu.c_str(),
                      system.tier.c_str(), config.c_str());
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
    column("##launcher-about", origin, ImVec2(left, size.y), c::bg1, ImVec2(px(28.0F), px(32.0F)));
    about(state);
    ImGui::End();
    // The middle column scrolls when a small window cannot hold its tiles, cards and Recent.
    column("##launcher-start", ImVec2(origin.x + left, origin.y),
           ImVec2(size.x - left - right, size.y), c::bg0, ImVec2(px(28.0F), px(28.0F)),
           kFixed & ~ImGuiWindowFlags_NoScrollbar);
    start(context, state);
    ImGui::End();
    column("##launcher-server", ImVec2(origin.x + size.x - right, origin.y), ImVec2(right, size.y),
           c::bg1, ImVec2(px(24.0F), px(28.0F)));
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
