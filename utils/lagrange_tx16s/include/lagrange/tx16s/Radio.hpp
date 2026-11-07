#pragma once

// The radio's state and what it commands, free of SDL, ImGui and ROS so that it can be tested:
// the sticks, switches, pots and sliders of a RadioMaster TX16S-style transmitter, filled from a
// USB joystick (EdgeTX's USB Joystick mode) or from the keyboard, and the velocity command they
// make for Lagrange's built-in controllers (/cmd_vel: forward, left, up, yaw rate).

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::tx16s {

/// Every control of the radio, in the order of `Radio::value`.
enum class Control : std::size_t {
    Roll,      ///< aileron: right stick, left/right (Mode 2)
    Pitch,     ///< elevator: right stick, up/down (Mode 2)
    Throttle,  ///< left stick, up/down (Mode 2); does not spring back on a real radio
    Yaw,       ///< rudder: left stick, left/right (Mode 2)
    SA,        ///< 3-position switches: −1, 0, +1
    SB,
    SC,
    SD,
    SE,
    SF,  ///< 2-position: −1, +1
    SG,
    SH,  ///< 2-position, momentary on the real radio
    S1,  ///< pots, −1 to +1
    S2,
    LS,  ///< side sliders, −1 to +1
    RS,
    /// The six buttons under the screen, one six-position switch (EdgeTX's 6POS): the one
    /// pressed last stays lit, the others go out. −1 to +1 in six steps (button k at −1 + 0.4·k,
    /// k = 0–5: EdgeTX's −100, −60, −20, 20, 60, 100).
    SixPos,
    Count,
};
inline constexpr std::size_t kControls = static_cast<std::size_t>(Control::Count);

[[nodiscard]] std::string_view name(Control c);
[[nodiscard]] std::optional<Control> controlNamed(std::string_view name);
/// The positions a switch has: 3 for SA–SE and SG, 2 for SF and SH, 6 for the six buttons, 0 for
/// the others.
[[nodiscard]] int positions(Control c);
inline constexpr int kSixPositions = 6;
/// Which of the six buttons (0–5) a 6POS value is, the nearest one; and a button's value.
[[nodiscard]] int sixPosButton(double value);
[[nodiscard]] double sixPosValue(int button);

/// The position of every control, each from −1 to +1 (a stick's up and right are +1).
struct Radio {
    std::array<double, kControls> value{};
    [[nodiscard]] double operator[](Control c) const { return value[static_cast<std::size_t>(c)]; }
    double& operator[](Control c) { return value[static_cast<std::size_t>(c)]; }
};

/// Which stick carries the throttle: Mode 2 (left, the usual one) or Mode 1 (right). In Mode 1
/// the right stick's up/down is throttle and the left one's is pitch.
enum class StickMode { Mode1, Mode2 };

/// The keyboard as a radio. Held keys push a stick towards its end at `kStickRate` per second;
/// released, the sticks spring back to the centre (the throttle too: there it means "hold the
/// height"). Switches step one position per press.
class Keyboard {
public:
    /// A key, as the app maps SDL's keys onto them.
    enum class Key : std::size_t {
        ThrottleUp,    // W
        ThrottleDown,  // S
        YawLeft,       // A
        YawRight,      // D
        PitchUp,       // Up arrow or I
        PitchDown,     // Down arrow or K
        RollLeft,      // Left arrow or J
        RollRight,     // Right arrow or L
        Count,
    };
    static constexpr double kStickRate = 3.0;   // per second, towards a held end
    static constexpr double kSpringRate = 5.0;  // per second, back to the centre

    void set(Key k, bool down) { held_[static_cast<std::size_t>(k)] = down; }
    [[nodiscard]] bool held(Key k) const { return held_[static_cast<std::size_t>(k)]; }
    /// Moves the sticks of `radio` for `dt` seconds of the held keys (the right stick's axes are
    /// pitch and roll, the left one's throttle and yaw, whatever the mode: the keys name what
    /// they do).
    void step(double dt, Radio& radio) const;
    /// One step of a switch: −1 → 0 → +1 → −1 for three positions, −1 ↔ +1 for two, the next
    /// button (after the sixth, the first) for the six buttons.
    static void toggle(Control c, Radio& radio);
    /// Presses one of the six buttons (0–5).
    static void press(int button, Radio& radio);

private:
    std::array<bool, static_cast<std::size_t>(Key::Count)> held_{};
};

/// Where a control comes from on a USB joystick: an axis (−32768 to 32767 in SDL), turned
/// round with `invert`, or nothing.
struct AxisMap {
    int axis{-1};  ///< −1: not mapped
    bool invert{false};
};

/// A joystick's axes onto the radio's controls. The default is EdgeTX's USB Joystick "classic"
/// mode, where the model's channels 1–8 are the HID axes X, Y, Z, Rx, Ry, Rz, slider and dial,
/// which SDL numbers 0–7, with a model whose channels are AETR, then SF (arm), SA (rate), SB, SC.
/// TODO(verify): the axis order on a real TX16S with EdgeTX; the app's mapping panel shows the
/// raw axes and saves a different map.
struct JoystickMap {
    std::array<AxisMap, kControls> controls{};
    double deadband{0.03};  ///< of full scale, around the centre of each stick

    [[nodiscard]] static JoystickMap edgetxClassic();
    /// The radio from the raw axes: mapped controls set, the others left as they are.
    void apply(const std::vector<int>& axes, Radio& radio) const;
};

/// The velocity command a radio makes, in m/s and rad/s, for /cmd_vel.
struct Command {
    double forward{0.0};
    double left{0.0};
    double up{0.0};
    double yawRate{0.0};  ///< counter-clockwise, seen from above
};

/// How Lagrange's multicopter flies the sticks, sent on /flight_mode ("position", "angle").
/// Position: the sticks are speeds over the ground and it holds its place, leaning into the
/// wind (GPS position mode). Angle: the right stick tilts it (full stick 30°) and the wind
/// carries it; the throttle still climbs and holds the height.
enum class FlightMode { Position, Angle };
[[nodiscard]] std::string_view flightModeName(FlightMode mode);

/// How the radio flies: the speeds at full stick (Lagrange's multicopter controller limits are
/// 5 m/s, 2 m/s and 1.5 rad/s), the rates of switch SA (low, middle, high: a share of full),
/// the arm switch (SF by default: up, +1, is armed), the flight mode switch (SB by default: up or
/// middle position, down angle) and the stick mode.
struct Mixer {
    double maxHorizontal{5.0};  // m/s
    double maxVertical{2.0};    // m/s
    double maxYawRate{1.5};     // rad/s
    std::array<double, 3> rates{0.3, 0.6, 1.0};
    Control rateSwitch{Control::SA};
    Control armSwitch{Control::SF};
    Control modeSwitch{Control::SB};
    StickMode mode{StickMode::Mode2};

    [[nodiscard]] bool armed(const Radio& radio) const { return radio[armSwitch] > 0.0; }
    [[nodiscard]] FlightMode flightMode(const Radio& radio) const {
        return radio[modeSwitch] < -0.5 ? FlightMode::Angle : FlightMode::Position;
    }
    /// The rate SA picks: rates[0] at −1, [1] in the middle, [2] at +1.
    [[nodiscard]] double rate(const Radio& radio) const;
    /// The command of the sticks, or nothing while disarmed (then nothing is sent, and the
    /// drone holds where it is, or stays on the ground).
    [[nodiscard]] std::optional<Command> command(const Radio& radio) const;
    /// The right and left sticks as drawn, (x, y) each, from the radio's controls and the mode.
    [[nodiscard]] std::array<double, 4> sticks(const Radio& radio) const;
};

/// The radio's channels as an EdgeTX model sends them: 1–4 AETR, then SA–SH, S1, S2, LS, RS, 6POS,
/// each −1 to +1 (for sensor_msgs/Joy).
[[nodiscard]] std::array<double, kControls> channels(const Radio& radio);

}  // namespace lagrange::tx16s
