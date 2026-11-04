// A USB radio as SDL sees it: SDL's virtual joystick with a TX16S's eight axes (EdgeTX's USB
// Joystick, classic mode), read the way the app reads it, through the default map.
#include "lagrange/tx16s/Radio.hpp"

#include <vector>

#include <SDL3/SDL.h>
#include <gtest/gtest.h>

namespace lagrange::tx16s {
namespace {

TEST(Joystick, AVirtualRadioFliesThroughTheDefaultMap) {
    SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "dummy");
    ASSERT_TRUE(SDL_Init(SDL_INIT_JOYSTICK)) << SDL_GetError();
    SDL_VirtualJoystickDesc desc;
    SDL_INIT_INTERFACE(&desc);
    desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
    desc.naxes = 8;
    desc.name = "TX16S virtual";
    const SDL_JoystickID id = SDL_AttachVirtualJoystick(&desc);
    ASSERT_NE(id, 0U) << SDL_GetError();
    SDL_Joystick* stick = SDL_OpenJoystick(id);
    ASSERT_NE(stick, nullptr) << SDL_GetError();
    EXPECT_EQ(SDL_GetNumJoystickAxes(stick), 8);
    // Disarmed (SF down on channel 5): nothing to send.
    ASSERT_TRUE(SDL_SetJoystickVirtualAxis(stick, 4, -32768));
    SDL_UpdateJoysticks();
    std::vector<int> axes(8);
    const auto read = [&] {
        for (int a = 0; a < 8; ++a) {
            axes[static_cast<std::size_t>(a)] = SDL_GetJoystickAxis(stick, a);
        }
    };
    read();
    Radio radio;
    const JoystickMap map = JoystickMap::edgetxClassic();
    map.apply(axes, radio);
    const Mixer mixer;
    EXPECT_FALSE(mixer.command(radio));
    // Armed, right stick full forward, throttle half up, SA (channel 6) at the top.
    ASSERT_TRUE(SDL_SetJoystickVirtualAxis(stick, 4, 32767));
    ASSERT_TRUE(SDL_SetJoystickVirtualAxis(stick, 1, 32767));
    ASSERT_TRUE(SDL_SetJoystickVirtualAxis(stick, 2, 16384));
    ASSERT_TRUE(SDL_SetJoystickVirtualAxis(stick, 5, 32767));
    SDL_UpdateJoysticks();
    read();
    map.apply(axes, radio);
    const auto c = mixer.command(radio);
    ASSERT_TRUE(c);
    EXPECT_DOUBLE_EQ(c->forward, mixer.maxHorizontal);  // SA up: full rate
    EXPECT_NEAR(c->up, 0.5 * mixer.maxVertical, 1e-3);
    EXPECT_DOUBLE_EQ(c->left, 0.0);
    SDL_CloseJoystick(stick);
    SDL_DetachVirtualJoystick(id);
    SDL_Quit();
}

}  // namespace
}  // namespace lagrange::tx16s
