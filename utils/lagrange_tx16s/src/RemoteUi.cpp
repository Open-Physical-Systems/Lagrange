#include "RemoteUi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include <imgui.h>

#include <lagrange/viewer/lagrange_theme.hpp>

namespace lagrange::tx16s {

namespace {

namespace c = theme::c;

ImU32 col(const ImVec4& v, float alpha = 1.0F) {
    return ImGui::GetColorU32(ImVec4(v.x, v.y, v.z, v.w * alpha));
}

// The radio's own greys: darker than the panels, as a plastic case is.
const ImVec4 kCase = theme::hex(0x23272E);
const ImVec4 kCaseEdge = theme::hex(0x3A404A);
const ImVec4 kGimbal = theme::hex(0x15181D);
const ImVec4 kMetal = theme::hex(0xB8C0CC);

ImFont* gMono = nullptr;

void monoText(ImDrawList* dl, ImVec2 at, ImU32 colour, const char* text, float size = 0.0F) {
    ImFont* font = gMono != nullptr ? gMono : ImGui::GetFont();
    dl->AddText(font, size > 0.0F ? size : ImGui::GetFontSize(), at, colour, text);
}

void centredText(ImDrawList* dl, ImVec2 centre, ImU32 colour, const char* text, float size = 0.0F) {
    ImFont* font = gMono != nullptr ? gMono : ImGui::GetFont();
    const float s = size > 0.0F ? size : ImGui::GetFontSize();
    const ImVec2 extent = font->CalcTextSizeA(s, 1e9F, 0.0F, text);
    dl->AddText(font, s, ImVec2(centre.x - extent.x / 2, centre.y - extent.y / 2), colour, text);
}

/// A gimbal: a square well with a stick in it. Dragging moves the stick; let go, it springs to
/// the centre on the next keyboard step (the keyboard model does the spring).
bool gimbal(const char* id, ImVec2 centre, float half, double& x, double& y, bool canDrag,
            const char* label) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 a(centre.x - half, centre.y - half);
    const ImVec2 b(centre.x + half, centre.y + half);
    dl->AddRectFilled(a, b, col(kGimbal), half * 0.25F);
    dl->AddRect(a, b, col(kCaseEdge), half * 0.25F, 0, 2.0F);
    dl->AddLine(ImVec2(centre.x - half * 0.85F, centre.y),
                ImVec2(centre.x + half * 0.85F, centre.y), col(c::lineStrong), 1.0F);
    dl->AddLine(ImVec2(centre.x, centre.y - half * 0.85F),
                ImVec2(centre.x, centre.y + half * 0.85F), col(c::lineStrong), 1.0F);
    dl->AddCircle(centre, half * 0.42F, col(c::lineStrong), 48, 1.0F);
    bool dragged = false;
    ImGui::SetCursorScreenPos(a);
    ImGui::InvisibleButton(id, ImVec2(half * 2, half * 2));
    if (canDrag && ImGui::IsItemActive()) {
        const ImVec2 m = ImGui::GetIO().MousePos;
        x = std::clamp(static_cast<double>((m.x - centre.x) / (half * 0.8F)), -1.0, 1.0);
        y = std::clamp(static_cast<double>((centre.y - m.y) / (half * 0.8F)), -1.0, 1.0);
        dragged = true;
    }
    const ImVec2 knob(centre.x + static_cast<float>(x) * half * 0.8F,
                      centre.y - static_cast<float>(y) * half * 0.8F);
    dl->AddLine(centre, knob, col(kMetal, 0.8F), half * 0.09F);
    dl->AddCircleFilled(knob, half * 0.17F, col(theme::hex(0x0E1013)));
    dl->AddCircle(knob, half * 0.17F, col(kMetal), 32, 2.0F);
    dl->AddCircleFilled(knob, half * 0.06F, col(c::accent));
    centredText(dl, ImVec2(centre.x, b.y + 14), col(c::t3), label, 11.0F);
    return dragged;
}

/// A switch drawn from the front: a slot, its lever up (+1), middle (0) or down (−1). A click
/// steps it; SH, momentary, is up only while pressed.
void toggleSwitch(Radio& radio, Control which, ImVec2 centre, bool canClick) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float w = 14.0F;
    const float h = 40.0F;
    const ImVec2 a(centre.x - w / 2, centre.y - h / 2);
    const ImVec2 b(centre.x + w / 2, centre.y + h / 2);
    ImGui::SetCursorScreenPos(ImVec2(a.x - 8, a.y - 4));
    const std::string id = std::string("##sw") + std::string(name(which));
    ImGui::InvisibleButton(id.c_str(), ImVec2(w + 16, h + 26));
    double& v = radio[which];
    if (canClick) {
        if (which == Control::SH) {
            v = ImGui::IsItemActive() ? 1.0 : -1.0;
        } else if (ImGui::IsItemClicked()) {
            Keyboard::toggle(which, radio);
        }
    }
    const bool hot = ImGui::IsItemHovered();
    dl->AddRectFilled(a, b, col(kGimbal), 4.0F);
    dl->AddRect(a, b, col(hot ? c::accentBorder : kCaseEdge), 4.0F, 0, 1.5F);
    const float lever = centre.y - static_cast<float>(v) * (h / 2 - 7);
    dl->AddRectFilled(ImVec2(centre.x - 3, centre.y), ImVec2(centre.x + 3, lever), col(kMetal));
    dl->AddCircleFilled(ImVec2(centre.x, lever), 6.0F, col(v > 0.5 ? c::accent : kMetal));
    centredText(dl, ImVec2(centre.x, b.y + 12), col(c::t2), std::string(name(which)).c_str(),
                11.0F);
}

/// A pot: a knob turned −1 to +1 (−135° to +135°), dragged sideways.
void pot(Radio& radio, Control which, ImVec2 centre, float r, bool canDrag) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::SetCursorScreenPos(ImVec2(centre.x - r, centre.y - r));
    const std::string id = std::string("##pot") + std::string(name(which));
    ImGui::InvisibleButton(id.c_str(), ImVec2(r * 2, r * 2));
    double& v = radio[which];
    if (canDrag && ImGui::IsItemActive()) {
        v = std::clamp(v + ImGui::GetIO().MouseDelta.x / 80.0, -1.0, 1.0);
    }
    if (canDrag && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        v = 0.0;
    }
    constexpr float kSweep = 2.356F;  // rad: ±135°
    const float angle = -1.5708F + static_cast<float>(v) * kSweep;
    dl->AddCircleFilled(centre, r, col(kGimbal), 32);
    dl->PathArcTo(centre, r - 3, -1.5708F - kSweep, -1.5708F + kSweep, 32);
    dl->PathStroke(col(c::lineStrong), 0, 2.0F);
    dl->PathArcTo(centre, r - 3, -1.5708F, angle, 32);
    dl->PathStroke(col(c::accent), 0, 2.5F);
    dl->AddLine(centre,
                ImVec2(centre.x + std::cos(angle) * (r - 5), centre.y + std::sin(angle) * (r - 5)),
                col(kMetal), 2.0F);
    centredText(dl, ImVec2(centre.x, centre.y + r + 11), col(c::t2),
                std::string(name(which)).c_str(), 11.0F);
}

/// A side slider: a vertical track, −1 at the bottom, dragged up and down.
void slider(Radio& radio, Control which, ImVec2 top, float length, bool canDrag) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::SetCursorScreenPos(ImVec2(top.x - 10, top.y));
    const std::string id = std::string("##sl") + std::string(name(which));
    ImGui::InvisibleButton(id.c_str(), ImVec2(20, length));
    double& v = radio[which];
    if (canDrag && ImGui::IsItemActive()) {
        v = std::clamp(1.0 - 2.0 * (ImGui::GetIO().MousePos.y - top.y) / length, -1.0, 1.0);
    }
    dl->AddRectFilled(ImVec2(top.x - 3, top.y), ImVec2(top.x + 3, top.y + length), col(kGimbal),
                      3.0F);
    dl->AddLine(ImVec2(top.x - 7, top.y + length / 2), ImVec2(top.x + 7, top.y + length / 2),
                col(c::lineStrong));
    const float y = top.y + static_cast<float>(1.0 - v) / 2.0F * length;
    dl->AddRectFilled(ImVec2(top.x - 9, y - 6), ImVec2(top.x + 9, y + 6), col(kMetal), 3.0F);
    centredText(dl, ImVec2(top.x, top.y + length + 12), col(c::t2),
                std::string(name(which)).c_str(), 11.0F);
}

/// The radio's screen: arm state, rate, mode, what it sends and the robot's telemetry.
void screen(ImVec2 a, ImVec2 b, const Radio& radio, const Mixer& mixer, const UiState& ui,
            const Telemetry& t, const std::optional<Command>& command) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(ImVec2(a.x - 6, a.y - 6), ImVec2(b.x + 6, b.y + 6), col(kGimbal), 8.0F);
    dl->AddRectFilled(a, b, col(c::bg0), 4.0F);
    dl->AddRect(a, b, col(c::line), 4.0F);
    const float pad = 10.0F;
    const float line = 17.0F;
    float y = a.y + pad;
    // Title bar: the model and the arm state.
    monoText(dl, ImVec2(a.x + pad, y), col(c::t1), "LAGRANGE", 13.0F);
    const bool armed = mixer.armed(radio);
    const char* state = armed ? "ARMED" : "DISARMED";
    const ImVec2 pillSize(armed ? 62.0F : 82.0F, 18.0F);
    const ImVec2 pa(b.x - pad - pillSize.x, y - 1);
    dl->AddRectFilled(pa, ImVec2(pa.x + pillSize.x, pa.y + pillSize.y),
                      col(armed ? c::ok : c::error, 0.18F), 9.0F);
    dl->AddRect(pa, ImVec2(pa.x + pillSize.x, pa.y + pillSize.y),
                col(armed ? c::ok : c::error, 0.6F), 9.0F);
    centredText(dl, ImVec2(pa.x + pillSize.x / 2, pa.y + pillSize.y / 2),
                col(armed ? c::ok : c::error), state, 11.0F);
    y += line + 6;
    dl->AddLine(ImVec2(a.x + pad, y), ImVec2(b.x - pad, y), col(c::line));
    y += 6;
    char text[96];
    const double r = mixer.rate(radio);
    std::snprintf(text, sizeof text, "%-9s%s  rate %3.0f%%",
                  ui.source == Source::Joystick ? "USB" : "KEYBOARD",
                  mixer.mode == StickMode::Mode2 ? "M2" : "M1", r * 100.0);
    monoText(dl, ImVec2(a.x + pad, y), col(c::t2), text, 12.0F);
    y += line;
    std::snprintf(text, sizeof text, "-> %s", ui.topic.c_str());
    monoText(dl, ImVec2(a.x + pad, y), col(c::t3), text, 12.0F);
    y += line + 4;
    const Command cmd = command.value_or(Command{});
    const ImU32 value = col(command ? c::t1 : c::t3);
    std::snprintf(text, sizeof text, "fwd %+5.2f  left %+5.2f m/s", cmd.forward, cmd.left);
    monoText(dl, ImVec2(a.x + pad, y), value, text, 12.0F);
    y += line;
    std::snprintf(text, sizeof text, "up  %+5.2f  yaw  %+5.2f rad/s", cmd.up, cmd.yawRate);
    monoText(dl, ImVec2(a.x + pad, y), value, text, 12.0F);
    y += line + 4;
    dl->AddLine(ImVec2(a.x + pad, y), ImVec2(b.x - pad, y), col(c::line));
    y += 6;
    if (t.have) {
        std::snprintf(text, sizeof text, "ALT %6.2f m   SPD %5.2f m/s", t.height, t.speed);
        monoText(dl, ImVec2(a.x + pad, y), col(c::info), text, 12.0F);
        y += line;
        std::snprintf(text, sizeof text, "VS  %+6.2f m/s HDG %+6.1f deg", t.climb, t.heading);
        monoText(dl, ImVec2(a.x + pad, y), col(c::info), text, 12.0F);
    } else {
        monoText(dl, ImVec2(a.x + pad, y), col(c::t3), "no telemetry (/odom)", 12.0F);
    }
}

}  // namespace

void setMonoFont(ImFont* font) {
    gMono = font;
}

void drawRemote(Radio& radio, const Mixer& mixer, const UiState& ui, const Telemetry& telemetry,
                const std::optional<Command>& command, float width, float height) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    // The case: a wide rounded body with a handle on top, sized to the space it has.
    const float w = std::min(width - 20.0F, height * 1.45F);
    const float h = w / 1.45F;
    const ImVec2 o(origin.x + (width - w) / 2, origin.y + (height - h) / 2 + 20);
    const ImVec2 bodyA(o.x, o.y + h * 0.12F);
    const ImVec2 bodyB(o.x + w, o.y + h);
    const ImVec2 mid((bodyA.x + bodyB.x) / 2, (bodyA.y + bodyB.y) / 2);
    // Handle.
    dl->AddRect(ImVec2(mid.x - w * 0.18F, o.y - h * 0.02F), ImVec2(mid.x + w * 0.18F, bodyA.y + 12),
                col(kCaseEdge), 18.0F, ImDrawFlags_RoundCornersTop, 10.0F);
    // Shoulders and body.
    dl->AddRectFilled(ImVec2(bodyA.x + w * 0.02F, bodyA.y - h * 0.06F),
                      ImVec2(bodyA.x + w * 0.30F, bodyA.y + 30), col(kCase), 16.0F);
    dl->AddRectFilled(ImVec2(bodyB.x - w * 0.30F, bodyA.y - h * 0.06F),
                      ImVec2(bodyB.x - w * 0.02F, bodyA.y + 30), col(kCase), 16.0F);
    dl->AddRectFilled(bodyA, bodyB, col(kCase), w * 0.07F);
    dl->AddRect(bodyA, bodyB, col(kCaseEdge), w * 0.07F, 0, 2.0F);
    // Grips at the lower corners.
    dl->AddRectFilled(ImVec2(bodyA.x + w * 0.03F, bodyB.y - h * 0.30F),
                      ImVec2(bodyA.x + w * 0.16F, bodyB.y - h * 0.04F), col(kGimbal, 0.55F), 24.0F);
    dl->AddRectFilled(ImVec2(bodyB.x - w * 0.16F, bodyB.y - h * 0.30F),
                      ImVec2(bodyB.x - w * 0.03F, bodyB.y - h * 0.04F), col(kGimbal, 0.55F), 24.0F);

    const bool byHand = true;  // the mouse works whatever the source; a joystick overwrites it
    // Switches: SE SF on the left shoulder, SA SB at the front left; SC SD at the front right,
    // SG SH on the right shoulder (as on the TX16S).
    const float sy = bodyA.y - h * 0.005F;
    const float sx = w * 0.075F;
    toggleSwitch(radio, Control::SE, ImVec2(bodyA.x + w * 0.08F, sy), byHand);
    toggleSwitch(radio, Control::SF, ImVec2(bodyA.x + w * 0.08F + sx, sy), byHand);
    toggleSwitch(radio, Control::SA, ImVec2(bodyA.x + w * 0.08F + 2 * sx, sy + 16), byHand);
    toggleSwitch(radio, Control::SB, ImVec2(bodyA.x + w * 0.08F + 3 * sx, sy + 16), byHand);
    toggleSwitch(radio, Control::SC, ImVec2(bodyB.x - w * 0.08F - 3 * sx, sy + 16), byHand);
    toggleSwitch(radio, Control::SD, ImVec2(bodyB.x - w * 0.08F - 2 * sx, sy + 16), byHand);
    toggleSwitch(radio, Control::SG, ImVec2(bodyB.x - w * 0.08F - sx, sy), byHand);
    toggleSwitch(radio, Control::SH, ImVec2(bodyB.x - w * 0.08F, sy), byHand);
    // Pots either side of the screen's top, sliders on the sides.
    pot(radio, Control::S1, ImVec2(mid.x - w * 0.20F, bodyA.y + h * 0.13F), 15.0F, byHand);
    pot(radio, Control::S2, ImVec2(mid.x + w * 0.20F, bodyA.y + h * 0.13F), 15.0F, byHand);
    slider(radio, Control::LS, ImVec2(bodyA.x + w * 0.035F, bodyA.y + h * 0.10F), h * 0.32F,
           byHand);
    slider(radio, Control::RS, ImVec2(bodyB.x - w * 0.035F, bodyA.y + h * 0.10F), h * 0.32F,
           byHand);
    // The screen.
    screen(ImVec2(mid.x - w * 0.16F, bodyA.y + h * 0.07F),
           ImVec2(mid.x + w * 0.16F, bodyA.y + h * 0.44F), radio, mixer, ui, telemetry, command);
    // The gimbals, below the screen to either side.
    std::array<double, 4> s = mixer.sticks(radio);
    const float half = w * 0.12F;
    const ImVec2 left(bodyA.x + w * 0.25F, bodyA.y + h * 0.58F);
    const ImVec2 right(bodyB.x - w * 0.25F, bodyA.y + h * 0.58F);
    const bool mode2 = mixer.mode == StickMode::Mode2;
    if (gimbal("##left", left, half, s[0], s[1], byHand,
               mode2 ? "yaw · throttle" : "yaw · pitch")) {
        radio[Control::Yaw] = s[0];
        radio[mode2 ? Control::Throttle : Control::Pitch] = s[1];
    }
    if (gimbal("##right", right, half, s[2], s[3], byHand,
               mode2 ? "roll · pitch" : "roll · throttle")) {
        radio[Control::Roll] = s[2];
        radio[mode2 ? Control::Pitch : Control::Throttle] = s[3];
    }
    // The six buttons under the screen (6POS): the lit one is the position; a click presses it.
    const int lit = sixPosButton(radio[Control::SixPos]);
    for (int k = 0; k < kSixPositions; ++k) {
        const ImVec2 bc(mid.x + (static_cast<float>(k) - 2.5F) * w * 0.045F, bodyA.y + h * 0.53F);
        const ImVec2 lo(bc.x - 15, bc.y - 11);
        const ImVec2 hi(bc.x + 15, bc.y + 11);
        ImGui::SetCursorScreenPos(lo);
        const std::string id = "##6pos" + std::to_string(k);
        ImGui::InvisibleButton(id.c_str(), ImVec2(30, 22));
        if (ImGui::IsItemClicked()) {
            Keyboard::press(k, radio);
        }
        const bool on = k == lit;
        dl->AddRectFilled(lo, hi, col(on ? c::accent : kGimbal), 6.0F);
        dl->AddRect(lo, hi, col(ImGui::IsItemHovered() ? c::accentBorder : kCaseEdge), 6.0F);
        const std::string label = std::to_string(k + 1);
        centredText(dl, bc, col(on ? c::onAccent : c::t2), label.c_str(), 12.0F);
    }
    centredText(dl, ImVec2(mid.x, bodyA.y + h * 0.53F + 22), col(c::t3), "6POS", 10.0F);
    // The menu keys beside the screen, under the pots, for the look only.
    constexpr std::array<std::array<const char*, 2>, 2> kKeys{{{"SYS", "TELE"}, {"MDL", "RTN"}}};
    for (std::size_t row = 0; row < kKeys.size(); ++row) {
        const float y = bodyA.y + h * (0.24F + 0.09F * static_cast<float>(row));
        for (std::size_t side = 0; side < 2; ++side) {
            const float x = mid.x + (side == 0 ? -1.0F : 1.0F) * w * 0.205F;
            dl->AddRectFilled(ImVec2(x - 18, y - 9), ImVec2(x + 18, y + 9), col(kGimbal), 9.0F);
            centredText(dl, ImVec2(x, y), col(c::t3), kKeys.at(row).at(side), 10.0F);
        }
    }
    centredText(dl, ImVec2(mid.x, bodyB.y - h * 0.08F), col(c::t3),
                "TX16S-style  ·  lagrange_tx16s", 11.0F);
    ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + height));
    ImGui::Dummy(ImVec2(width, 0));
}

void drawPanel(Radio& radio, Mixer& mixer, UiState& ui, const std::optional<Command>& command) {
    (void)command;
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 6));
    ImGui::TextColored(c::t3, "INPUT");
    int source = ui.source == Source::Joystick ? 1 : 0;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##source", &source, "Keyboard and mouse\0USB radio (joystick)\0")) {
        ui.source = source == 1 ? Source::Joystick : Source::Keyboard;
    }
    if (ui.source == Source::Joystick) {
        if (ui.joystickName.empty()) {
            ImGui::TextColored(c::warn, "No joystick. Plug the TX16S in by USB and pick");
            ImGui::TextColored(c::warn, "\"USB Joystick (HID)\" on the radio.");
        } else {
            ImGui::TextColored(c::ok, "%s", ui.joystickName.c_str());
            for (std::size_t i = 0; i < ui.joystickAxes.size(); ++i) {
                const float f = static_cast<float>(ui.joystickAxes[i]) / 32767.0F;
                char overlay[32];
                std::snprintf(overlay, sizeof overlay, "axis %zu  %+.2f", i, f);
                ImGui::ProgressBar((f + 1.0F) / 2.0F, ImVec2(-1, 14), overlay);
            }
        }
    } else if (ui.keyboardHelp) {
        ImGui::TextColored(c::t2, "W/S throttle  A/D yaw");
        ImGui::TextColored(c::t2, "arrows or IJKL pitch and roll");
        ImGui::TextColored(c::t2, "Space arm (SF)  R rate (SA)");
        ImGui::TextColored(c::t2, "1-8 switches SA-SH  M stick mode");
        ImGui::TextColored(c::t2, "F1-F6 the six buttons (6POS)");
        ImGui::TextColored(c::t3, "or drag the sticks with the mouse");
    }

    ImGui::Spacing();
    // The map matters only with a radio plugged in: closed until then.
    ImGui::SetNextItemOpen(ui.source == Source::Joystick, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Joystick map") &&
        ImGui::BeginTable("map", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("control", ImGuiTableColumnFlags_WidthStretch, 1.0F);
        ImGui::TableSetupColumn("axis", ImGuiTableColumnFlags_WidthStretch, 1.2F);
        ImGui::TableSetupColumn("inv", ImGuiTableColumnFlags_WidthFixed, 28.0F);
        const int axes = std::max<int>(8, static_cast<int>(ui.joystickAxes.size()));
        for (std::size_t i = 0; i < kControls; ++i) {
            const auto control = static_cast<Control>(i);
            AxisMap& m = ui.map.controls.at(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(std::string(name(control)).c_str());
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-1);
            char preview[16];
            std::snprintf(preview, sizeof preview, m.axis < 0 ? "-" : "axis %d", m.axis);
            const std::string id = "##axis" + std::to_string(i);
            if (ImGui::BeginCombo(id.c_str(), preview)) {
                if (ImGui::Selectable("-", m.axis < 0)) {
                    m.axis = -1;
                }
                for (int a = 0; a < axes; ++a) {
                    char item[16];
                    std::snprintf(item, sizeof item, "axis %d", a);
                    if (ImGui::Selectable(item, m.axis == a)) {
                        m.axis = a;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::TableNextColumn();
            const std::string inv = "##inv" + std::to_string(i);
            ImGui::Checkbox(inv.c_str(), &m.invert);
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::TextColored(c::t3, "FLIGHT");
    int mode = mixer.mode == StickMode::Mode2 ? 1 : 0;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##mode", &mode, "Mode 1 (throttle right)\0Mode 2 (throttle left)\0")) {
        mixer.mode = mode == 1 ? StickMode::Mode2 : StickMode::Mode1;
    }
    float h = static_cast<float>(mixer.maxHorizontal);
    float v = static_cast<float>(mixer.maxVertical);
    float yaw = static_cast<float>(mixer.maxYawRate);
    ImGui::SetNextItemWidth(-90);
    if (ImGui::SliderFloat("m/s fwd", &h, 0.5F, 10.0F, "%.1f")) {
        mixer.maxHorizontal = h;
    }
    ImGui::SetNextItemWidth(-90);
    if (ImGui::SliderFloat("m/s up", &v, 0.2F, 5.0F, "%.1f")) {
        mixer.maxVertical = v;
    }
    ImGui::SetNextItemWidth(-90);
    if (ImGui::SliderFloat("rad/s yaw", &yaw, 0.2F, 3.0F, "%.1f")) {
        mixer.maxYawRate = yaw;
    }
    bool armed = mixer.armed(radio);
    if (ImGui::Checkbox("Armed (SF up)", &armed)) {
        radio[mixer.armSwitch] = armed ? 1.0 : -1.0;
    }

    ImGui::Spacing();
    if (ImGui::Button("Save settings", ImVec2(-1, 0))) {
        ui.saveRequested = true;
    }
    if (!ui.savedTo.empty()) {
        ImGui::TextColored(c::t3, "saved to %s", ui.savedTo.c_str());
    }
    ImGui::PopStyleVar();
}

}  // namespace lagrange::tx16s
