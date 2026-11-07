// lagrange_tx16s: a TX16S-style radio on screen, flown from a real RadioMaster TX16S over USB
// (EdgeTX's USB Joystick mode) or from the keyboard and mouse. It sends /cmd_vel (TwistStamped
// by default: forward, left, up, yaw rate), which Lagrange's built-in multicopter, car and
// wheeled-robot controllers take, while the arm switch (SF) is up, and /tx16s/joy (sensor_msgs/Joy)
// with every channel always. The radio's screen shows the robot's telemetry from /odom.
//
//   ros2 run lagrange_tx16s lagrange_tx16s [--headless --frames N --screenshot FILE.bmp --demo]
//       [--ros-args -p mode:=1 -p topic:=/cmd_vel -p stamped:=true ...]
//
// Its settings (stick mode, speeds, joystick map) are saved by the panel's Save button to
// $XDG_CONFIG_HOME/lagrange/tx16s.yaml (default ~/.config), a ROS parameter file read at start.

#include "RemoteUi.hpp"
#include "lagrange/tx16s/Radio.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <SDL3/SDL.h>
#include <imgui.h>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <lagrange/viewer/lagrange_theme.hpp>

namespace {

using namespace lagrange::tx16s;
using namespace std::chrono_literals;

constexpr int kWidth = 1280;
constexpr int kHeight = 760;
constexpr float kPanelWidth = 330.0F;
constexpr double kPublishHz = 50.0;

std::filesystem::path settingsFile() {
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    const char* home = std::getenv("HOME");
    const std::filesystem::path base =
        xdg != nullptr && *xdg != '\0'
            ? std::filesystem::path(xdg)
            : std::filesystem::path(home != nullptr ? home : ".") / ".config";
    return base / "lagrange" / "tx16s.yaml";
}

/// What the ROS side and the UI share, under one lock.
struct Shared {
    std::mutex lock;
    Radio radio;
    Mixer mixer;
    Telemetry telemetry;
    std::optional<double> ground;
};

class RadioNode : public rclcpp::Node {
public:
    RadioNode(Shared& shared, const rclcpp::NodeOptions& options)
        : Node("tx16s", options), shared_(shared) {
        Mixer& m = shared_.mixer;
        m.mode = declare_parameter<int>("mode", 2) == 1 ? StickMode::Mode1 : StickMode::Mode2;
        m.maxHorizontal = declare_parameter<double>("max_horizontal", m.maxHorizontal);
        m.maxVertical = declare_parameter<double>("max_vertical", m.maxVertical);
        m.maxYawRate = declare_parameter<double>("max_yaw_rate", m.maxYawRate);
        topic_ = declare_parameter<std::string>("topic", "/cmd_vel");
        const bool stamped = declare_parameter<bool>("stamped", true);
        frame_ = declare_parameter<std::string>("frame_id", "base_link");
        const std::string odom = declare_parameter<std::string>("odom_topic", "/odom");
        map_.deadband = declare_parameter<double>("map.deadband", map_.deadband);
        const JoystickMap defaults = JoystickMap::edgetxClassic();
        for (std::size_t i = 0; i < kControls; ++i) {
            const std::string key = "map." + std::string(name(static_cast<Control>(i)));
            map_.controls.at(i).axis = static_cast<int>(
                declare_parameter<int>(key + ".axis", defaults.controls.at(i).axis));
            map_.controls.at(i).invert =
                declare_parameter<bool>(key + ".invert", defaults.controls.at(i).invert);
        }
        // The arm switch starts off and the others in the middle or up, as a radio is checked
        // before a flight.
        for (std::size_t i = 0; i < kControls; ++i) {
            const auto control = static_cast<Control>(i);
            if (positions(control) == 2) {
                shared_.radio[control] = -1.0;
            }
        }
        shared_.radio[Control::SA] = 0.0;   // the middle rate
        Keyboard::press(0, shared_.radio);  // button 1
        if (stamped) {
            stampedOut_ = create_publisher<geometry_msgs::msg::TwistStamped>(topic_, 10);
        } else {
            plainOut_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
        }
        joy_ = create_publisher<sensor_msgs::msg::Joy>("/tx16s/joy", 10);
        odom_ = create_subscription<nav_msgs::msg::Odometry>(
            odom, 10, [this](const nav_msgs::msg::Odometry& o) { onOdom(o); });
        // Lagrange's multicopter: the flight mode (latched, sent when SB moves) and the air
        // past it.
        modeOut_ = create_publisher<std_msgs::msg::String>(
            "/flight_mode", rclcpp::QoS(1).transient_local().reliable());
        airspeed_ = create_subscription<std_msgs::msg::Float64>(
            "/airspeed", 10, [this](const std_msgs::msg::Float64& air) {
                const std::lock_guard<std::mutex> hold(shared_.lock);
                shared_.telemetry.airspeed = air.data;
            });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / kPublishHz),
                                   [this] { publish(); });
    }

    [[nodiscard]] const std::string& topic() const { return topic_; }
    [[nodiscard]] const JoystickMap& map() const { return map_; }

private:
    void publish() {
        std::optional<Command> command;
        Radio radio;
        FlightMode mode{FlightMode::Position};
        {
            const std::lock_guard<std::mutex> hold(shared_.lock);
            radio = shared_.radio;
            command = shared_.mixer.command(radio);
            mode = shared_.mixer.flightMode(radio);
        }
        if (mode != sentMode_) {
            std_msgs::msg::String name;
            name.data = std::string(flightModeName(mode));
            modeOut_->publish(name);
            sentMode_ = mode;
        }
        const rclcpp::Time now = get_clock()->now();
        // While armed, the sticks' command, 50 times a second. Disarmed, one zero command (so
        // the robot holds), then nothing: Lagrange's controllers hold where they are.
        if (command || wasArmed_) {
            const Command c = command.value_or(Command{});
            geometry_msgs::msg::Twist twist;
            twist.linear.x = c.forward;
            twist.linear.y = c.left;
            twist.linear.z = c.up;
            twist.angular.z = c.yawRate;
            if (stampedOut_) {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = now;
                msg.header.frame_id = frame_;
                msg.twist = twist;
                stampedOut_->publish(msg);
            } else {
                plainOut_->publish(twist);
            }
        }
        wasArmed_ = command.has_value();
        sensor_msgs::msg::Joy joy;
        joy.header.stamp = now;
        joy.header.frame_id = "tx16s";
        for (const double v : channels(radio)) {
            joy.axes.push_back(static_cast<float>(v));
        }
        // Buttons: each 2- and 3-position switch at its top position (SA–SH), then the six
        // buttons, the lit one 1.
        for (std::size_t i = 0; i < kControls; ++i) {
            const int n = positions(static_cast<Control>(i));
            if (n == 2 || n == 3) {
                joy.buttons.push_back(radio.value.at(i) > 0.5 ? 1 : 0);
            }
        }
        const int lit = sixPosButton(radio[Control::SixPos]);
        for (int k = 0; k < kSixPositions; ++k) {
            joy.buttons.push_back(k == lit ? 1 : 0);
        }
        joy_->publish(joy);
    }

    void onOdom(const nav_msgs::msg::Odometry& o) {
        const auto& p = o.pose.pose.position;
        const auto& q = o.pose.pose.orientation;
        const auto& v = o.twist.twist.linear;
        const std::lock_guard<std::mutex> hold(shared_.lock);
        if (!shared_.ground) {
            shared_.ground = p.z;
        }
        Telemetry& t = shared_.telemetry;
        t.have = true;
        t.height = p.z - *shared_.ground;
        t.climb = v.z;
        t.heading = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) *
                    180.0 / 3.141592653589793;
        // The body's up axis against the world's: its z component is 1 − 2(x² + y²).
        t.tilt = std::acos(std::clamp(1.0 - 2.0 * (q.x * q.x + q.y * q.y), -1.0, 1.0)) * 180.0 /
                 3.141592653589793;
        // The ground speed from the body-frame twist turned to the world: the odometry gives
        // it in the child frame, which tilts.
        const double w = q.w, x = q.x, y = q.y, z = q.z;
        const double vx =
            (1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y - w * z) * v.y + 2 * (x * z + w * y) * v.z;
        const double vy =
            2 * (x * y + w * z) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z - w * x) * v.z;
        t.speed = std::hypot(vx, vy);
    }

    Shared& shared_;
    std::string topic_;
    std::string frame_;
    JoystickMap map_;
    bool wasArmed_{false};
    std::optional<FlightMode> sentMode_;  // what /flight_mode last said
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr modeOut_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr airspeed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stampedOut_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr plainOut_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/// Writes the settings as a ROS parameter file for the next start.
bool save(const std::filesystem::path& file, const Mixer& m, const UiState& ui) {
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    std::ofstream out(file);
    if (!out) {
        return false;
    }
    out << "# lagrange_tx16s settings, written by its Save button; a ROS 2 parameter file.\n"
        << "tx16s:\n  ros__parameters:\n"
        << "    mode: " << (m.mode == StickMode::Mode1 ? 1 : 2) << "\n"
        << "    max_horizontal: " << m.maxHorizontal << "\n"
        << "    max_vertical: " << m.maxVertical << "\n"
        << "    max_yaw_rate: " << m.maxYawRate << "\n"
        << "    map:\n      deadband: " << ui.map.deadband << "\n";
    for (std::size_t i = 0; i < kControls; ++i) {
        const AxisMap& a = ui.map.controls.at(i);
        out << "      " << name(static_cast<Control>(i)) << ": {axis: " << a.axis
            << ", invert: " << (a.invert ? "true" : "false") << "}\n";
    }
    return static_cast<bool>(out);
}

std::optional<Keyboard::Key> stickKey(SDL_Scancode s) {
    switch (s) {
        case SDL_SCANCODE_W:
            return Keyboard::Key::ThrottleUp;
        case SDL_SCANCODE_S:
            return Keyboard::Key::ThrottleDown;
        case SDL_SCANCODE_A:
            return Keyboard::Key::YawLeft;
        case SDL_SCANCODE_D:
            return Keyboard::Key::YawRight;
        case SDL_SCANCODE_UP:
        case SDL_SCANCODE_I:
            return Keyboard::Key::PitchUp;
        case SDL_SCANCODE_DOWN:
        case SDL_SCANCODE_K:
            return Keyboard::Key::PitchDown;
        case SDL_SCANCODE_LEFT:
        case SDL_SCANCODE_J:
            return Keyboard::Key::RollLeft;
        case SDL_SCANCODE_RIGHT:
        case SDL_SCANCODE_L:
            return Keyboard::Key::RollRight;
        default:
            return std::nullopt;
    }
}

struct Options {
    bool headless{false};
    int frames{-1};
    std::string screenshot;
    bool demo{false};  // armed, sticks held part way: for screenshots
};

}  // namespace

int main(int argc, char** argv) {
    // Our own options first; the rest (--ros-args ...) go to ROS.
    Options options;
    std::vector<char*> rosArgs{argv[0]};
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") {
            options.headless = true;
        } else if (a == "--frames" && i + 1 < argc) {
            options.frames = std::atoi(argv[++i]);
        } else if (a == "--screenshot" && i + 1 < argc) {
            options.screenshot = argv[++i];
        } else if (a == "--demo") {
            options.demo = true;
        } else if (a == "--help" || a == "-h") {
            std::puts(
                "lagrange_tx16s [--headless --frames N --screenshot FILE.bmp] [--ros-args ...]\n"
                "  a TX16S-style radio: a real one over USB (EdgeTX USB Joystick) or the "
                "keyboard;\n"
                "  sends /cmd_vel while armed (SF up, or Space) and /tx16s/joy\n"
                "  parameters: mode (1|2), topic, stamped, frame_id, odom_topic, max_horizontal,\n"
                "  max_vertical, max_yaw_rate, map.<control>.axis|invert, map.deadband");
            return 0;
        } else {
            rosArgs.push_back(argv[i]);
        }
    }
    rclcpp::init(static_cast<int>(rosArgs.size()), rosArgs.data());
    rclcpp::NodeOptions nodeOptions;
    const std::filesystem::path file = settingsFile();
    if (std::filesystem::exists(file)) {
        // Saved settings first; the command line's own parameters, after them, still win.
        std::vector<std::string> arguments{"--ros-args", "--params-file", file.string()};
        arguments.insert(arguments.end(), rosArgs.begin() + 1, rosArgs.end());
        nodeOptions.arguments(arguments);
    }
    Shared shared;
    auto node = std::make_shared<RadioNode>(shared, nodeOptions);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor] { executor.spin(); });

    if (options.headless) {
        SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "offscreen");
    }
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK)) {
        std::fprintf(stderr, "lagrange_tx16s: SDL: %s\n", SDL_GetError());
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("Lagrange TX16S", kWidth, kHeight,
                                          SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, options.headless ? "software" : nullptr);
    if (window == nullptr || renderer == nullptr) {
        std::fprintf(stderr, "lagrange_tx16s: window: %s\n", SDL_GetError());
        SDL_Quit();
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    const float scale = options.headless ? 1.0F : SDL_GetWindowDisplayScale(window);
    lagrange::theme::applyStyle(scale);
    // The design's fonts, from the viewer's package; ImGui's own if they are not there.
    try {
        const std::string fonts =
            (ament_index_cpp::get_package_share_path("lagrange_viewer") / "fonts/").string();
        ImFont* sans =
            io.Fonts->AddFontFromFileTTF((fonts + "IBMPlexSans-Regular.ttf").c_str(), 13.0F);
        ImFont* mono =
            io.Fonts->AddFontFromFileTTF((fonts + "IBMPlexMono-Medium.ttf").c_str(), 12.0F);
        if (sans != nullptr) {
            io.FontDefault = sans;
        }
        setMonoFont(mono);
    } catch (const std::exception&) {
    }
    ImGui::GetStyle().FontScaleDpi = scale;
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    UiState ui;
    ui.map = node->map();
    ui.topic = node->topic();
    Keyboard keyboard;
    SDL_Joystick* joystick = nullptr;
    const auto openJoystick = [&]() {
        int count = 0;
        SDL_JoystickID* ids = SDL_GetJoysticks(&count);
        if (ids != nullptr && count > 0 && joystick == nullptr) {
            joystick = SDL_OpenJoystick(ids[0]);
            if (joystick != nullptr) {
                const char* n = SDL_GetJoystickName(joystick);
                ui.joystickName = n != nullptr ? n : "joystick";
                std::fprintf(stderr, "[tx16s] joystick: %s, %d axes\n", ui.joystickName.c_str(),
                             SDL_GetNumJoystickAxes(joystick));
                if (!options.demo) {
                    ui.source = Source::Joystick;
                }
            }
        }
        SDL_free(ids);
    };
    openJoystick();
    if (options.demo) {
        const std::lock_guard<std::mutex> hold(shared.lock);
        shared.radio[Control::SF] = 1.0;
        shared.radio[Control::Throttle] = 0.4;
        shared.radio[Control::Yaw] = -0.3;
        shared.radio[Control::Pitch] = 0.6;
        shared.radio[Control::Roll] = 0.25;
        shared.radio[Control::S1] = 0.5;
        shared.radio[Control::LS] = -0.4;
    }

    auto last = std::chrono::steady_clock::now();
    int frame = 0;
    bool running = true;
    while (running && rclcpp::ok()) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);
            if (e.type == SDL_EVENT_QUIT || e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
                running = false;
            } else if (e.type == SDL_EVENT_JOYSTICK_ADDED) {
                openJoystick();
                if (options.demo) {
                    const std::lock_guard<std::mutex> hold(shared.lock);
                    shared.radio[Control::SF] = 1.0;
                    shared.radio[Control::Throttle] = 0.4;
                    shared.radio[Control::Yaw] = -0.3;
                    shared.radio[Control::Pitch] = 0.6;
                    shared.radio[Control::Roll] = 0.25;
                    shared.radio[Control::S1] = 0.5;
                    shared.radio[Control::LS] = -0.4;
                }
            } else if (e.type == SDL_EVENT_JOYSTICK_REMOVED && joystick != nullptr &&
                       e.jdevice.which == SDL_GetJoystickID(joystick)) {
                SDL_CloseJoystick(joystick);
                joystick = nullptr;
                ui.joystickName.clear();
                ui.joystickAxes.clear();
            } else if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat && !io.WantTextInput) {
                const std::lock_guard<std::mutex> hold(shared.lock);
                const SDL_Scancode s = e.key.scancode;
                if (s == SDL_SCANCODE_SPACE) {
                    Keyboard::toggle(shared.mixer.armSwitch, shared.radio);
                } else if (s == SDL_SCANCODE_R) {
                    Keyboard::toggle(shared.mixer.rateSwitch, shared.radio);
                } else if (s == SDL_SCANCODE_G) {
                    // The flight mode switch: down is angle, the middle position.
                    const Control sb = shared.mixer.modeSwitch;
                    shared.radio[sb] =
                        shared.mixer.flightMode(shared.radio) == FlightMode::Angle ? 0.0 : -1.0;
                } else if (s == SDL_SCANCODE_M) {
                    shared.mixer.mode =
                        shared.mixer.mode == StickMode::Mode2 ? StickMode::Mode1 : StickMode::Mode2;
                } else if (s >= SDL_SCANCODE_1 && s <= SDL_SCANCODE_8) {
                    Keyboard::toggle(static_cast<Control>(static_cast<std::size_t>(Control::SA) +
                                                          (s - SDL_SCANCODE_1)),
                                     shared.radio);
                } else if (s >= SDL_SCANCODE_F1 && s <= SDL_SCANCODE_F6) {
                    Keyboard::press(static_cast<int>(s - SDL_SCANCODE_F1), shared.radio);
                } else if (s == SDL_SCANCODE_ESCAPE) {
                    shared.radio[shared.mixer.armSwitch] = -1.0;  // disarm at once
                }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::min(std::chrono::duration<double>(now - last).count(), 0.1);
        last = now;

        // Input: the keyboard moves the sticks (and springs them back); a joystick, when it is
        // the source, then sets the controls it maps.
        const bool* keys = SDL_GetKeyboardState(nullptr);
        for (int s = 0; s < SDL_SCANCODE_COUNT; ++s) {
            if (const auto k = stickKey(static_cast<SDL_Scancode>(s))) {
                keyboard.set(*k, false);
            }
        }
        if (ui.source == Source::Keyboard && !io.WantTextInput) {
            for (int s = 0; s < SDL_SCANCODE_COUNT; ++s) {
                if (keys[s]) {
                    if (const auto k = stickKey(static_cast<SDL_Scancode>(s))) {
                        keyboard.set(*k, true);
                    }
                }
            }
        }
        if (joystick != nullptr) {
            ui.joystickAxes.resize(
                static_cast<std::size_t>(std::max(0, SDL_GetNumJoystickAxes(joystick))));
            for (std::size_t a = 0; a < ui.joystickAxes.size(); ++a) {
                ui.joystickAxes[a] = SDL_GetJoystickAxis(joystick, static_cast<int>(a));
            }
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        {
            const std::lock_guard<std::mutex> hold(shared.lock);
            const bool dragging = ImGui::IsMouseDown(0) && ImGui::IsAnyItemActive();
            if (ui.source == Source::Keyboard && !dragging && !options.demo) {
                keyboard.step(dt, shared.radio);
            }
            if (ui.source == Source::Joystick) {
                ui.map.apply(ui.joystickAxes, shared.radio);
            }
            const std::optional<Command> command = shared.mixer.command(shared.radio);
            const ImGuiViewport* vp = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(vp->WorkPos);
            ImGui::SetNextWindowSize(vp->WorkSize);
            ImGui::Begin("##root", nullptr,
                         ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoBringToFrontOnFocus |
                             ImGuiWindowFlags_NoSavedSettings);
            const float remoteWidth = vp->WorkSize.x - kPanelWidth * scale - 24.0F;
            ImGui::BeginChild("##remote", ImVec2(remoteWidth, 0), ImGuiChildFlags_None,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            drawRemote(shared.radio, shared.mixer, ui, shared.telemetry, command, remoteWidth,
                       vp->WorkSize.y - 40.0F);
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("##panel", ImVec2(0, 0), ImGuiChildFlags_Borders);
            drawPanel(shared.radio, shared.mixer, ui, command);
            ImGui::EndChild();
            ImGui::End();
            if (ui.saveRequested) {
                ui.saveRequested = false;
                ui.savedTo = save(file, shared.mixer, ui) ? file.string() : "(could not write)";
            }
        }
        ImGui::Render();
        const ImVec4 bg = lagrange::theme::c::bg1;
        SDL_SetRenderDrawColorFloat(renderer, bg.x, bg.y, bg.z, 1.0F);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        ++frame;
        if (options.frames > 0 && frame >= options.frames) {
            if (!options.screenshot.empty()) {
                if (SDL_Surface* shot = SDL_RenderReadPixels(renderer, nullptr)) {
                    SDL_SaveBMP(shot, options.screenshot.c_str());
                    SDL_DestroySurface(shot);
                }
            }
            running = false;
        }
        SDL_RenderPresent(renderer);
        if (options.headless) {
            std::this_thread::sleep_for(16ms);
        }
    }

    if (joystick != nullptr) {
        SDL_CloseJoystick(joystick);
    }
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
