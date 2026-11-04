#include "lagrange/tx16s/Radio.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::tx16s {

namespace {

constexpr std::array<std::string_view, kControls> kNames{
    "roll", "pitch", "throttle", "yaw", "SA", "SB", "SC", "SD",
    "SE",   "SF",    "SG",       "SH",  "S1", "S2", "LS", "RS",
};
constexpr double kAxisFull = 32767.0;  // SDL's axis range is −32768 to 32767

double toward(double from, double to, double step) {
    return from < to ? std::min(from + step, to) : std::max(from - step, to);
}

}  // namespace

std::string_view name(Control c) {
    return kNames.at(static_cast<std::size_t>(c));
}

std::optional<Control> controlNamed(std::string_view text) {
    for (std::size_t i = 0; i < kControls; ++i) {
        if (kNames.at(i) == text) {
            return static_cast<Control>(i);
        }
    }
    return std::nullopt;
}

int positions(Control c) {
    switch (c) {
        case Control::SA:
        case Control::SB:
        case Control::SC:
        case Control::SD:
        case Control::SE:
        case Control::SG:
            return 3;
        case Control::SF:
        case Control::SH:
            return 2;
        default:
            return 0;
    }
}

void Keyboard::step(double dt, Radio& radio) const {
    const auto axis = [&](Control c, Key plus, Key minus) {
        const double want = (held(plus) ? 1.0 : 0.0) - (held(minus) ? 1.0 : 0.0);
        double& v = radio[c];
        // Towards a held end at the stick rate; released, back to the centre on the spring.
        const double rate = want == 0.0 ? kSpringRate : kStickRate;
        v = toward(v, want, rate * dt);
    };
    axis(Control::Throttle, Key::ThrottleUp, Key::ThrottleDown);
    axis(Control::Yaw, Key::YawRight, Key::YawLeft);
    axis(Control::Pitch, Key::PitchUp, Key::PitchDown);
    axis(Control::Roll, Key::RollRight, Key::RollLeft);
}

void Keyboard::toggle(Control c, Radio& radio) {
    double& v = radio[c];
    if (positions(c) == 3) {
        v = v < -0.5 ? 0.0 : (v < 0.5 ? 1.0 : -1.0);
    } else if (positions(c) == 2) {
        v = v > 0.0 ? -1.0 : 1.0;
    }
}

JoystickMap JoystickMap::edgetxClassic() {
    JoystickMap map;
    // Channels 1–8 on axes 0–7: AETR, then the arm switch SF on 5 (where most drone models put
    // it), the rate switch SA on 6, then SB and SC.
    constexpr std::array<Control, 8> kOrder{Control::Roll, Control::Pitch, Control::Throttle,
                                            Control::Yaw,  Control::SF,    Control::SA,
                                            Control::SB,   Control::SC};
    for (std::size_t i = 0; i < kOrder.size(); ++i) {
        map.controls.at(static_cast<std::size_t>(kOrder.at(i))).axis = static_cast<int>(i);
    }
    return map;
}

void JoystickMap::apply(const std::vector<int>& axes, Radio& radio) const {
    for (std::size_t c = 0; c < kControls; ++c) {
        const AxisMap& m = controls.at(c);
        if (m.axis < 0 || static_cast<std::size_t>(m.axis) >= axes.size()) {
            continue;
        }
        double v = std::clamp(axes[static_cast<std::size_t>(m.axis)] / kAxisFull, -1.0, 1.0);
        if (m.invert) {
            v = -v;
        }
        const auto control = static_cast<Control>(c);
        if (positions(control) == 3) {
            v = v < -0.33 ? -1.0 : (v > 0.33 ? 1.0 : 0.0);
        } else if (positions(control) == 2) {
            v = v > 0.0 ? 1.0 : -1.0;
        } else if (control <= Control::Yaw && std::abs(v) < deadband) {
            v = 0.0;  // the sticks rest near, not at, the centre
        }
        radio.value.at(c) = v;
    }
}

double Mixer::rate(const Radio& radio) const {
    const double s = radio[rateSwitch];
    return s < -0.5 ? rates[0] : (s > 0.5 ? rates[2] : rates[1]);
}

std::optional<Command> Mixer::command(const Radio& radio) const {
    if (!armed(radio)) {
        return std::nullopt;
    }
    const double r = rate(radio);
    Command c;
    c.forward = radio[Control::Pitch] * maxHorizontal * r;
    c.left = -radio[Control::Roll] * maxHorizontal * r;  // roll right is +1, left is +y
    c.up = radio[Control::Throttle] * maxVertical * r;
    c.yawRate = -radio[Control::Yaw] * maxYawRate * r;  // rudder right turns clockwise
    return c;
}

std::array<double, 4> Mixer::sticks(const Radio& radio) const {
    // {left x, left y, right x, right y}
    if (mode == StickMode::Mode1) {
        return {radio[Control::Yaw], radio[Control::Pitch], radio[Control::Roll],
                radio[Control::Throttle]};
    }
    return {radio[Control::Yaw], radio[Control::Throttle], radio[Control::Roll],
            radio[Control::Pitch]};
}

std::array<double, kControls> channels(const Radio& radio) {
    return radio.value;
}

}  // namespace lagrange::tx16s
