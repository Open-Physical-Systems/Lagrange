#pragma once

// Dear ImGui drawing of the TX16S-style radio and its side panel. UI thread only.

#include "lagrange/tx16s/Radio.hpp"

#include <optional>
#include <string>
#include <vector>

struct ImFont;

namespace lagrange::tx16s {

/// What the radio's screen shows besides its own state: the robot's telemetry from /odom.
struct Telemetry {
    bool have{false};
    double height{0.0};   // m above where it started
    double speed{0.0};    // m/s, horizontal
    double climb{0.0};    // m/s
    double heading{0.0};  // deg, counter-clockwise from +x
};

enum class Source { Keyboard, Joystick };

/// Everything the panel edits and shows.
struct UiState {
    Source source{Source::Keyboard};
    std::string joystickName;       // empty: none connected
    std::vector<int> joystickAxes;  // raw, −32768 to 32767
    JoystickMap map{JoystickMap::edgetxClassic()};
    std::string topic{"/cmd_vel"};
    std::string savedTo;  // where Save wrote the settings, for the panel to say
    bool saveRequested{false};
    bool keyboardHelp{true};
};

/// The face for numbers and names on the radio (IBM Plex Mono); ImGui's default when null.
void setMonoFont(ImFont* font);

/// Draws the radio filling the current window's left part. Sticks, switches, pots and sliders
/// can be dragged and clicked; the sticks spring back when let go.
void drawRemote(Radio& radio, const Mixer& mixer, const UiState& ui, const Telemetry& telemetry,
                const std::optional<Command>& command, float width, float height);

/// The side panel: input source, joystick axes and mapping, flight settings, output.
void drawPanel(Radio& radio, Mixer& mixer, UiState& ui, const std::optional<Command>& command);

}  // namespace lagrange::tx16s
