// The radio's model: the keyboard's sticks and springs, the switches, the joystick map and the
// command the sticks make.
#include "lagrange/tx16s/Radio.hpp"

#include <gtest/gtest.h>

namespace lagrange::tx16s {
namespace {

using Key = Keyboard::Key;

TEST(Keyboard, AHeldKeyPushesTheStickAndItSpringsBack) {
    Radio radio;
    Keyboard keys;
    keys.set(Key::PitchUp, true);
    keys.step(0.1, radio);
    EXPECT_NEAR(radio[Control::Pitch], 0.1 * Keyboard::kStickRate, 1e-12);
    for (int i = 0; i < 20; ++i) {
        keys.step(0.1, radio);
    }
    EXPECT_DOUBLE_EQ(radio[Control::Pitch], 1.0);  // at the end, no further
    keys.set(Key::PitchUp, false);
    keys.step(0.1, radio);
    EXPECT_NEAR(radio[Control::Pitch], 1.0 - 0.1 * Keyboard::kSpringRate, 1e-12);
    keys.step(1.0, radio);
    EXPECT_DOUBLE_EQ(radio[Control::Pitch], 0.0);  // back at the centre, not past it
    // Both ways at once cancel; the throttle springs back too (the centre holds the height).
    keys.set(Key::YawLeft, true);
    keys.set(Key::YawRight, true);
    keys.set(Key::ThrottleUp, true);
    keys.step(0.2, radio);
    EXPECT_DOUBLE_EQ(radio[Control::Yaw], 0.0);
    EXPECT_GT(radio[Control::Throttle], 0.5);
    keys.set(Key::ThrottleUp, false);
    keys.step(1.0, radio);
    EXPECT_DOUBLE_EQ(radio[Control::Throttle], 0.0);
}

TEST(Keyboard, SwitchesStepThroughTheirPositions) {
    Radio radio;
    radio[Control::SA] = -1.0;
    Keyboard::toggle(Control::SA, radio);
    EXPECT_DOUBLE_EQ(radio[Control::SA], 0.0);
    Keyboard::toggle(Control::SA, radio);
    EXPECT_DOUBLE_EQ(radio[Control::SA], 1.0);
    Keyboard::toggle(Control::SA, radio);
    EXPECT_DOUBLE_EQ(radio[Control::SA], -1.0);
    radio[Control::SF] = -1.0;
    Keyboard::toggle(Control::SF, radio);
    EXPECT_DOUBLE_EQ(radio[Control::SF], 1.0);
    Keyboard::toggle(Control::SF, radio);
    EXPECT_DOUBLE_EQ(radio[Control::SF], -1.0);
    radio[Control::S1] = 0.4;
    Keyboard::toggle(Control::S1, radio);  // a pot is not a switch
    EXPECT_DOUBLE_EQ(radio[Control::S1], 0.4);
}

TEST(SixPos, OneButtonOfSixIsLit) {
    Radio radio;
    for (int k = 0; k < kSixPositions; ++k) {
        Keyboard::press(k, radio);
        EXPECT_EQ(sixPosButton(radio[Control::SixPos]), k);
    }
    EXPECT_DOUBLE_EQ(sixPosValue(0), -1.0);  // EdgeTX's −100 … +100 in steps of 40
    EXPECT_DOUBLE_EQ(sixPosValue(5), 1.0);
    EXPECT_NEAR(sixPosValue(2), -0.2, 1e-12);
    Keyboard::toggle(Control::SixPos, radio);  // after the sixth, the first
    EXPECT_EQ(sixPosButton(radio[Control::SixPos]), 0);
    Keyboard::toggle(Control::SixPos, radio);
    EXPECT_EQ(sixPosButton(radio[Control::SixPos]), 1);
    // From a joystick axis: the nearest of the six levels.
    JoystickMap map;
    map.controls.at(static_cast<std::size_t>(Control::SixPos)).axis = 0;
    map.apply({static_cast<int>(0.21 * 32767)}, radio);  // between 3 (−0.2) and 4 (+0.2)
    EXPECT_EQ(sixPosButton(radio[Control::SixPos]), 3);
    EXPECT_NEAR(radio[Control::SixPos], 0.2, 1e-12);
    map.apply({-32768}, radio);
    EXPECT_EQ(sixPosButton(radio[Control::SixPos]), 0);
}

TEST(JoystickMap, EdgetxClassicReadsAetrAndTheSwitches) {
    const JoystickMap map = JoystickMap::edgetxClassic();
    Radio radio;
    // Axes 0–7: aileron, elevator, throttle, rudder, SF, SA, SB, SC.
    map.apply({16384, -32768, 32767, 500, 30000, -30000, 0, 12000}, radio);
    EXPECT_NEAR(radio[Control::Roll], 0.5, 1e-3);
    EXPECT_DOUBLE_EQ(radio[Control::Pitch], -1.0);
    EXPECT_DOUBLE_EQ(radio[Control::Throttle], 1.0);
    EXPECT_DOUBLE_EQ(radio[Control::Yaw], 0.0);  // 1.5 % of full: inside the deadband
    EXPECT_DOUBLE_EQ(radio[Control::SF], 1.0);   // armed
    EXPECT_DOUBLE_EQ(radio[Control::SA], -1.0);
    EXPECT_DOUBLE_EQ(radio[Control::SB], 0.0);
    EXPECT_DOUBLE_EQ(radio[Control::SC], 1.0);  // 0.37: past a third, the top position
}

TEST(JoystickMap, InvertsLeavesUnmappedAloneAndIgnoresMissingAxes) {
    JoystickMap map;
    map.controls.at(static_cast<std::size_t>(Control::Pitch)) = {1, true};
    map.controls.at(static_cast<std::size_t>(Control::SF)) = {9, false};  // not on this stick
    Radio radio;
    radio[Control::SF] = 1.0;
    radio[Control::Roll] = 0.7;
    map.apply({0, 16384}, radio);
    EXPECT_NEAR(radio[Control::Pitch], -0.5, 1e-3);
    EXPECT_DOUBLE_EQ(radio[Control::SF], 1.0);
    EXPECT_DOUBLE_EQ(radio[Control::Roll], 0.7);
}

TEST(Mixer, SendsNothingDisarmedAndTheSticksArmed) {
    Mixer mixer;
    Radio radio;
    radio[Control::SF] = -1.0;
    radio[Control::Pitch] = 1.0;
    EXPECT_FALSE(mixer.command(radio));
    radio[Control::SF] = 1.0;
    radio[Control::SA] = 1.0;    // full rate
    radio[Control::Roll] = 0.5;  // right
    radio[Control::Throttle] = -1.0;
    radio[Control::Yaw] = 1.0;  // right: clockwise
    const auto c = mixer.command(radio);
    ASSERT_TRUE(c);
    EXPECT_DOUBLE_EQ(c->forward, mixer.maxHorizontal);
    EXPECT_DOUBLE_EQ(c->left, -0.5 * mixer.maxHorizontal);
    EXPECT_DOUBLE_EQ(c->up, -mixer.maxVertical);
    EXPECT_DOUBLE_EQ(c->yawRate, -mixer.maxYawRate);
    radio[Control::SA] = -1.0;  // low rate
    EXPECT_DOUBLE_EQ(mixer.command(radio)->forward, mixer.rates[0] * mixer.maxHorizontal);
    radio[Control::SA] = 0.0;
    EXPECT_DOUBLE_EQ(mixer.rate(radio), mixer.rates[1]);
}

TEST(Mixer, TheModePutsTheThrottleOnItsStick) {
    Mixer mixer;
    Radio radio;
    radio[Control::Throttle] = 0.8;
    radio[Control::Pitch] = -0.2;
    mixer.mode = StickMode::Mode2;
    EXPECT_DOUBLE_EQ(mixer.sticks(radio)[1], 0.8);  // left stick, up/down
    EXPECT_DOUBLE_EQ(mixer.sticks(radio)[3], -0.2);
    mixer.mode = StickMode::Mode1;
    EXPECT_DOUBLE_EQ(mixer.sticks(radio)[1], -0.2);
    EXPECT_DOUBLE_EQ(mixer.sticks(radio)[3], 0.8);  // right stick, up/down
}

// SB is the flight mode switch: up and middle (where it starts) position, down angle, named as
// /flight_mode says them.
TEST(Mixer, SbPicksTheFlightMode) {
    const Mixer mixer;
    Radio radio;
    EXPECT_EQ(mixer.flightMode(radio), FlightMode::Position);
    radio[Control::SB] = 1.0;
    EXPECT_EQ(mixer.flightMode(radio), FlightMode::Position);
    radio[Control::SB] = -1.0;
    EXPECT_EQ(mixer.flightMode(radio), FlightMode::Angle);
    EXPECT_EQ(flightModeName(FlightMode::Angle), "angle");
    EXPECT_EQ(flightModeName(FlightMode::Position), "position");
}

TEST(Controls, HaveNamesBothWays) {
    for (std::size_t i = 0; i < kControls; ++i) {
        const auto c = static_cast<Control>(i);
        EXPECT_EQ(controlNamed(name(c)), c);
    }
    EXPECT_FALSE(controlNamed("SZ"));
}

}  // namespace
}  // namespace lagrange::tx16s
