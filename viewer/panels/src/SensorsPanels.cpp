// Sensors workspace: the sensor list, the lidar bird's-eye view, the IMU scopes and a summary
// of the selected sensor (docs/design/ui/06-sensors/SPEC.md).
//
// Sensors are the stage's prims of a sensor type; their frames and signals are named after
// those prims. Nothing here knows a particular robot.
#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr double kRateWindowSeconds = 1.0;  // frame rates are measured over this long
// Returns weaker than this are drawn as "attenuated" (design: two tones of the sensor colour).
constexpr float kStrongIntensity = 0.12F;

const ImVec4 kStrong = theme::kDomainColor[static_cast<int>(theme::Domain::Sensors)];
const ImVec4 kAttenuated = theme::hex(0x2A8F76);

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

enum class Kind : std::uint8_t { Lidar, Camera, Imu, Other };

Kind kindOf(const std::string& typeName) {
    if (typeName == "Lidar") {
        return Kind::Lidar;
    }
    if (typeName == "Camera") {
        return Kind::Camera;
    }
    if (typeName == "Imu") {
        return Kind::Imu;
    }
    return Kind::Other;
}

bool isSensorType(const std::string& typeName) {
    return typeName == "Lidar" || typeName == "Camera" || typeName == "Imu" ||
           typeName == "Magnetometer" || typeName == "Gnss" || typeName == "Radar";
}

struct Sensor {
    std::uint32_t prim{kNone};
    Kind kind{Kind::Other};
    std::shared_ptr<SensorStream> stream;  // sensors that send frames (lidar, camera)
    std::size_t signals{0};                // sensors that send signals (IMU)
    std::int64_t signalDtNs{0};
    // Frame rate, measured.
    std::uint64_t countedFrames{0};
    double countedAt{0.0};
    double rateHz{0.0};
};

/// What the panels share: the stage's sensors, their streams and the IMU scopes.
class SensorsState {
public:
    void follow(PanelContext& context) {
        const std::uint64_t scene = context.scene ? context.scene->statics->generation : 0;
        const std::uint64_t catalog = context.catalog ? context.catalog->generation : 0;
        if (scene != scene_ || catalog != catalog_) {
            scene_ = scene;
            catalog_ = catalog;
            discover(context);
        }
        if (const int frame = ImGui::GetFrameCount(); frame != frame_) {
            frame_ = frame;
            scopes.update();
            const double now = ImGui::GetTime();
            for (Sensor& sensor : sensors) {
                if (sensor.stream && now - sensor.countedAt >= kRateWindowSeconds) {
                    const std::uint64_t frames = sensor.stream->frames();
                    sensor.rateHz = static_cast<double>(frames - sensor.countedFrames) /
                                    (now - sensor.countedAt);
                    sensor.countedFrames = frames;
                    sensor.countedAt = now;
                }
            }
        }
        // The selection is global: selecting a sensor prim anywhere selects it here.
        for (std::size_t i = 0; i < sensors.size(); ++i) {
            if (sensors[i].prim == context.selectedPrim) {
                selected = i;
            }
        }
        selected = sensors.empty() ? 0 : std::min(selected, sensors.size() - 1);
    }

    /// The first sensor of `kind`, preferring the selected one. Null if the stage has none.
    [[nodiscard]] const Sensor* first(Kind kind) const {
        if (selected < sensors.size() && sensors[selected].kind == kind) {
            return &sensors[selected];
        }
        for (const Sensor& sensor : sensors) {
            if (sensor.kind == kind) {
                return &sensor;
            }
        }
        return nullptr;
    }

    std::vector<Sensor> sensors;
    std::size_t selected{0};
    ScopeSession scopes;  // the IMU's signals
    ScopePlotter plotter;

private:
    void discover(PanelContext& context) {
        sensors.clear();
        while (!scopes.traces().empty()) {
            scopes.remove(context.client, scopes.traces().size() - 1);
        }
        if (!context.scene) {
            return;
        }
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        for (std::uint32_t i = 0; i < prims.size(); ++i) {
            if (!isSensorType(prims[i].typeName)) {
                continue;
            }
            Sensor sensor;
            sensor.prim = i;
            sensor.kind = kindOf(prims[i].typeName);
            sensor.countedAt = ImGui::GetTime();
            if (sensor.kind == Kind::Lidar || sensor.kind == Kind::Camera) {
                sensor.stream = context.client.watchSensor(prims[i].path);
                sensor.countedFrames = sensor.stream->frames();
            }
            if (context.catalog) {
                for (const SignalInfo& info : context.catalog->signals) {
                    // Scalars, and the axes of vectors (an IMU's specific force and rate).
                    if (info.prim != i || (info.components != 1 && info.components != 3)) {
                        continue;
                    }
                    ++sensor.signals;
                    sensor.signalDtNs = info.dtNs;
                    if (sensor.kind == Kind::Imu) {
                        for (std::uint32_t c = 0; c < info.components; ++c) {
                            scopes.add(context.client, *context.catalog, info.name, c);
                        }
                    }
                }
            }
            sensors.push_back(std::move(sensor));
        }
    }

    std::uint64_t scene_{0};
    std::uint64_t catalog_{0};
    int frame_{-1};
};

bool emptyState(const PanelContext& context, const SensorsState& state) {
    if (!context.scene) {
        widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                   ? "Waiting for the scene ..."
                                   : "Not connected. Sensors appear when a simulation server "
                                     "is reachable.");
        return true;
    }
    if (state.sensors.empty()) {
        widgets::Caption("This stage has no sensors.");
        return true;
    }
    return false;
}

void rate(std::int64_t dtNs, char* out, std::size_t size) {
    const double hz = 1e9 / static_cast<double>(std::max<std::int64_t>(dtNs, 1));
    if (hz >= 1000.0) {
        std::snprintf(out, size, "%g kHz", hz / 1000.0);
    } else {
        std::snprintf(out, size, "%g Hz", hz);
    }
}

// --- list ---------------------------------------------------------------------------------

class SensorListPanel final : public Panel {
public:
    explicit SensorListPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Sensors"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        // The id after ### is what the dock layout refers to.
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Sensors###SensorList");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (!context.scene || state_->sensors.empty()) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), px(8.0F)));
            emptyState(context, *state_);
            ImGui::End();
            return;
        }
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        for (std::size_t i = 0; i < state_->sensors.size(); ++i) {
            const Sensor& sensor = state_->sensors[i];
            const bool receiving =
                sensor.stream ? sensor.stream->frames() != 0 : sensor.signals != 0;
            char detail[96];
            if (sensor.stream && sensor.kind == Kind::Camera) {
                const std::shared_ptr<const SensorImage> image = sensor.stream->latestImage();
                std::snprintf(detail, sizeof(detail), "%u×%u  ·  %.1f Hz",
                              image ? image->width : 0U, image ? image->height : 0U, sensor.rateHz);
            } else if (sensor.stream) {
                const std::shared_ptr<const PointCloud> cloud = sensor.stream->latest();
                std::snprintf(detail, sizeof(detail), "%.1f Hz  ·  %u pts", sensor.rateHz,
                              cloud ? cloud->count : 0U);
            } else if (sensor.signals != 0) {
                char hz[24];
                rate(sensor.signalDtNs, hz, sizeof(hz));
                std::snprintf(detail, sizeof(detail), "%s  ·  %zu signals", hz, sensor.signals);
            } else {
                std::snprintf(detail, sizeof(detail), "no data from this server");
            }
            if (row(static_cast<int>(i), prims[sensor.prim].name.c_str(), detail,
                    receiving ? "ok" : "no data", receiving, state_->selected == i)) {
                state_->selected = i;
                context.selectedPrim = sensor.prim;
            }
        }
        ImGui::End();
    }

private:
    /// Design: status dot, name, a status pill, and a mono line with the configuration.
    static bool row(int index, const char* name, const char* detail, const char* pill, bool ok,
                    bool selected) {
        const float width = ImGui::GetContentRegionAvail().x;
        const float height = px(52.0F);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const ImVec2 max(min.x + width, min.y + height);
        ImGui::PushID(index);
        const bool clicked = ImGui::InvisibleButton("##sensor", ImVec2(width, height));
        ImGui::PopID();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        if (selected) {
            draw->AddRectFilled(min, max, u32(c::accentWeak));
            draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), max.y), u32(c::accent));
        } else if (ImGui::IsItemHovered()) {
            draw->AddRectFilled(min, max, u32(c::bg3));
        }
        draw->AddLine(ImVec2(min.x, max.y), max, u32(c::line));
        draw->AddCircleFilled(ImVec2(min.x + px(14.0F), min.y + height * 0.5F), px(4.0F),
                              u32(ok ? c::ok : c::warn));
        const theme::Fonts& fonts = theme::fonts();
        draw->AddText(fonts.sansMedium, px(theme::kBodySize),
                      ImVec2(min.x + px(28.0F), min.y + px(8.0F)), u32(c::t1), name);
        draw->AddText(fonts.mono, px(theme::kCaptionSize),
                      ImVec2(min.x + px(28.0F), min.y + px(30.0F)), u32(c::t3), detail);
        const float pillWidth = widgets::PillWidth(pill);
        const ImVec2 after = ImGui::GetCursorScreenPos();
        ImGui::SetCursorScreenPos(ImVec2(max.x - pillWidth - px(10.0F), min.y + px(8.0F)));
        widgets::Pill(pill, ok ? widgets::PillKind::Ok : widgets::PillKind::Warn);
        ImGui::SetCursorScreenPos(after);
        ImGui::Dummy(ImVec2(0.0F, 0.0F));
        return clicked;
    }

    std::shared_ptr<SensorsState> state_;
};

// --- lidar --------------------------------------------------------------------------------

class LidarPanel final : public Panel {
public:
    explicit LidarPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Lidar"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Lidar · bird's-eye###Lidar");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (widgets::BeginPanelTools("##lidar-tools", px(120.0F))) {
            static constexpr std::array<const char*, 2> kModes{"intensity", "range"};
            widgets::Segmented("##mode", kModes.data(), 2, &mode_);
            widgets::EndPanelTools();
        }
        const Sensor* lidar = state_->first(Kind::Lidar);
        if (lidar == nullptr) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context, *state_)) {
                widgets::Caption("This stage has no lidar.");
            }
            ImGui::End();
            return;
        }
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton("##canvas", size);
        if (ImGui::IsItemHovered() && ImGui::GetIO().MouseWheel != 0.0F) {
            radius_ = std::clamp(radius_ * std::pow(0.88F, ImGui::GetIO().MouseWheel), 2.0F, 60.0F);
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), u32(c::bg0));

        // The sensor sits low in the view, looking up the screen: x forward is up, y left is
        // left, as a driver would see it from above.
        const ImVec2 centre(origin.x + size.x * 0.5F, origin.y + size.y * 0.68F);
        const float scale = size.y * 0.62F / radius_;  // pixels per metre
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        const float step = radius_ > 30.0F ? 10.0F : 5.0F;
        for (float r = step; r <= radius_ * 1.8F; r += step) {
            draw->AddCircle(centre, r * scale, u32(c::lineStrong), 96);
            char label[16];
            std::snprintf(label, sizeof(label), "%.0f m", static_cast<double>(r));
            draw->AddText(fonts.mono, small,
                          ImVec2(centre.x + r * scale * 0.7071F + px(4.0F),
                                 centre.y - r * scale * 0.7071F - small),
                          u32(c::t3), label);
        }
        draw->AddLine(ImVec2(centre.x, origin.y), ImVec2(centre.x, origin.y + size.y),
                      u32(c::line));
        draw->AddLine(ImVec2(origin.x, centre.y), ImVec2(origin.x + size.x, centre.y),
                      u32(c::line));

        const std::shared_ptr<const PointCloud> cloud = lidar->stream->latest();
        std::uint32_t strong = 0;
        if (cloud) {
            const float dot = std::max(px(1.6F), 1.0F);
            const ImU32 strongColor = u32(kStrong);
            const ImU32 weakColor = u32(kAttenuated);
            for (std::uint32_t i = 0; i < cloud->count; ++i) {
                const float* p = &cloud->points[static_cast<std::size_t>(i) * 4];
                const ImVec2 at(centre.x - p[1] * scale, centre.y - p[0] * scale);
                if (at.x < origin.x || at.x > origin.x + size.x || at.y < origin.y ||
                    at.y > origin.y + size.y) {
                    continue;
                }
                ImU32 color = 0;
                if (mode_ == 0) {
                    const bool isStrong = p[3] >= kStrongIntensity;
                    strong += isStrong ? 1U : 0U;
                    color = isStrong ? strongColor : weakColor;
                } else {
                    color = rangeColor(std::hypot(p[0], p[1]) / (radius_ * 1.5F));
                }
                draw->AddRectFilled(ImVec2(at.x - dot * 0.5F, at.y - dot * 0.5F),
                                    ImVec2(at.x + dot * 0.5F, at.y + dot * 0.5F), color);
            }
        }
        // The robot: an accent arrow pointing forward.
        const float a = px(7.0F);
        draw->AddTriangleFilled(ImVec2(centre.x, centre.y - a * 1.4F),
                                ImVec2(centre.x - a, centre.y + a),
                                ImVec2(centre.x + a, centre.y + a), u32(c::accent));
        draw->PopClipRect();

        // Overlays: what is drawn, and how much of it.
        const ImVec2 bottomLeft(origin.x + px(10.0F), origin.y + size.y - px(36.0F));
        if (!cloud) {
            box(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)),
                "no frames from this sensor yet", c::t2);
        } else {
            char count[64];
            std::snprintf(count, sizeof(count), "%u pts  ·  %.1f Hz", cloud->count, lidar->rateHz);
            const float width =
                fonts.mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, count).x + px(16.0F);
            box(draw, ImVec2(origin.x + size.x - width - px(10.0F), bottomLeft.y), count, c::t1);
            if (mode_ == 0) {
                legend(draw, bottomLeft, strong, cloud->count);
            } else {
                box(draw, bottomLeft, "colour: range, near to far", c::t2);
            }
        }
        if (const DomainView* view = context.profileSession->find("sensors");
            view != nullptr && !view->enabled) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)));
            widgets::Pill("Sensors domain is off in the profile · this scan is synthetic",
                          widgets::PillKind::Info);
        }
        ImGui::End();
    }

private:
    /// The "field" colour map of the design tokens (viridis), by fraction 0..1.
    static ImU32 rangeColor(float fraction) {
        static const std::array<ImVec4, 5> kStops{theme::hex(0x440154), theme::hex(0x3B528B),
                                                  theme::hex(0x21918C), theme::hex(0x5EC962),
                                                  theme::hex(0xFDE725)};
        const float t = std::clamp(fraction, 0.0F, 1.0F) * 4.0F;
        const auto index = std::min(static_cast<std::size_t>(t), std::size_t{3});
        const float blend = t - static_cast<float>(index);
        const ImVec4& a = kStops.at(index);
        const ImVec4& b = kStops.at(index + 1);
        return u32(ImVec4(a.x + (b.x - a.x) * blend, a.y + (b.y - a.y) * blend,
                          a.z + (b.z - a.z) * blend, 1.0F));
    }

    static void box(ImDrawList* draw, const ImVec2& min, const char* text, const ImVec4& color) {
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kMonoSize);
        const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, text).x;
        const ImVec2 max(min.x + width + px(16.0F), min.y + px(26.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        draw->AddText(font, size, ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(color), text);
    }

    static void legend(ImDrawList* draw, const ImVec2& min, std::uint32_t strong,
                       std::uint32_t total) {
        char text[96];
        std::snprintf(text, sizeof(text), "strong %u     attenuated %u", strong, total - strong);
        ImFont* font = theme::fonts().sans;
        const float size = px(theme::kLabelSize);
        const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, text).x;
        const ImVec2 max(min.x + width + px(44.0F), min.y + px(26.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        draw->AddCircleFilled(ImVec2(min.x + px(12.0F), min.y + px(13.0F)), px(3.5F), u32(kStrong));
        draw->AddCircleFilled(ImVec2(min.x + px(24.0F), min.y + px(13.0F)), px(3.5F),
                              u32(kAttenuated));
        draw->AddText(font, size, ImVec2(min.x + px(34.0F), min.y + px(5.0F)), u32(c::t2), text);
    }

    std::shared_ptr<SensorsState> state_;
    int mode_{0};
    float radius_{9.0F};  // metres from the sensor to the top of the view
};

// --- IMU ----------------------------------------------------------------------------------

class ImuPanel final : public Panel {
public:
    explicit ImuPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "IMU"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin("IMU###Imu")) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        ScopeSession& scopes = state_->scopes;
        if (scopes.traces().empty()) {
            if (!emptyState(context, *state_)) {
                widgets::Caption("%s", state_->first(Kind::Imu) != nullptr
                                           ? "The IMU of this stage sends no signals."
                                           : "This stage has no IMU.");
            }
            ImGui::End();
            return;
        }
        static constexpr std::array<const char*, 3> kWindows{"1 s", "10 s", "60 s"};
        scopes.window = std::clamp(scopes.window, 0, 2);
        widgets::Segmented("##window", kWindows.data(), 3, &scopes.window);
        ImGui::SameLine(0.0F, px(12.0F));
        ImGui::AlignTextToFramePadding();
        char hz[24];
        rate(scopes.traces().front().buffer->dtNs(), hz, sizeof(hz));
        widgets::Caption("%zu signals at %s  ·  one plot per unit", scopes.traces().size(), hz);
        state_->plotter.draw(scopes);
        ImGui::End();
    }

private:
    std::shared_ptr<SensorsState> state_;
};

// --- camera -------------------------------------------------------------------------------

class CameraPanel final : public Panel {
public:
    explicit CameraPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Camera"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        state_->follow(context);
        const Sensor* camera = state_->first(Kind::Camera);
        // Design: the tab names the camera.
        const std::string name = camera != nullptr && context.scene
                                     ? context.scene->statics->prims[camera->prim].name
                                     : std::string("Camera");
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin((name + "###Camera").c_str());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        if (widgets::BeginPanelTools("##camera-tools", px(150.0F))) {
            static constexpr std::array<const char*, 3> kModes{"rgb", "depth", "labels"};
            widgets::Segmented("##mode", kModes.data(), 3, &mode_);
            widgets::EndPanelTools();
        }
        const std::shared_ptr<const SensorImage> image =
            camera != nullptr && camera->stream ? camera->stream->latestImage() : nullptr;
        if (camera == nullptr || !image || mode_ != 0) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context, *state_)) {
                ImGui::PushTextWrapPos(0.0F);
                widgets::Caption("%s", camera == nullptr ? "This stage has no camera."
                                       : mode_ != 0
                                           ? "This server sends the colour image only."
                                           : "Waiting for the first image (the sensors domain "
                                             "may be on ground truth, which renders nothing).");
                ImGui::PopTextWrapPos();
            }
            ImGui::End();
            return;
        }
        // Design: the image across the panel's width at its own aspect, then a mono footer.
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const float footer = px(28.0F);
        const float aspect = static_cast<float>(image->height) / static_cast<float>(image->width);
        float width = available.x;
        float height = width * aspect;
        if (height > available.y - footer) {
            height = std::max(available.y - footer, 1.0F);
            width = height / aspect;
        }
        const std::uint64_t texture =
            context.sensorTextures != nullptr ? context.sensorTextures->texture(name, image) : 0;
        const ImVec2 at = ImGui::GetCursorScreenPos();
        if (texture != 0) {
            ImGui::SetCursorScreenPos(ImVec2(at.x + (available.x - width) * 0.5F, at.y));
            ImGui::Image(static_cast<ImTextureID>(texture), ImVec2(width, height));
        } else {
            ImGui::Dummy(ImVec2(width, height));
        }
        ImGui::SetCursorScreenPos(ImVec2(at.x + px(10.0F), at.y + height + px(8.0F)));
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        ImGui::TextColored(c::t2, "%u×%u", image->width, image->height);
        ImGui::SameLine(0.0F, px(14.0F));
        ImGui::TextColored(c::t2, "%.1f Hz", camera->rateHz);
        ImGui::SameLine(0.0F, px(14.0F));
        ImGui::TextColored(c::t2, "t %.3f s", static_cast<double>(image->simTimeNs) * 1e-9);
        ImGui::SameLine(0.0F, px(14.0F));
        ImGui::TextColored(c::t3, "raster · no sensor model");
        ImGui::PopFont();
        ImGui::End();
    }

private:
    std::shared_ptr<SensorsState> state_;
    int mode_{0};
};

// --- summary of the selected sensor -------------------------------------------------------

class DegradationPanel final : public Panel {
public:
    explicit DegradationPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Degradation"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (emptyState(context, *state_)) {
            ImGui::End();
            return;
        }
        const Sensor& sensor = state_->sensors[state_->selected];
        const PrimData& prim = context.scene->statics->prims[sensor.prim];
        ImGui::PushFont(theme::fonts().sansSemiBold, theme::kTitleSize);
        ImGui::TextUnformatted(prim.name.c_str());
        ImGui::PopFont();
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::TextDisabled("%s", prim.typeName.c_str());
        widgets::Mono(c::t3, "%s", prim.path.c_str());
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));

        const std::shared_ptr<const PointCloud> cloud =
            sensor.stream ? sensor.stream->latest() : nullptr;
        if (cloud && widgets::SectionHeader("This frame", "measured")) {
            // Measured from the points themselves: what the frame holds, not a model of why.
            float nearest = FLT_MAX;
            float farthest = 0.0F;
            double intensity = 0.0;
            for (std::uint32_t i = 0; i < cloud->count; ++i) {
                const float* p = &cloud->points[static_cast<std::size_t>(i) * 4];
                const float range = std::hypot(p[0], p[1], p[2]);
                nearest = std::min(nearest, range);
                farthest = std::max(farthest, range);
                intensity += static_cast<double>(p[3]);
            }
            char text[48];
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%u", cloud->count);
            widgets::PropertyRow("points", text, nullptr);
            std::snprintf(text, sizeof(text), "%.2f", cloud->count != 0 ? nearest : 0.0F);
            widgets::PropertyRow("nearest return", text, "m");
            std::snprintf(text, sizeof(text), "%.2f", farthest);
            widgets::PropertyRow("farthest return", text, "m");
            std::snprintf(text, sizeof(text), "%.3f",
                          cloud->count != 0 ? intensity / cloud->count : 0.0);
            widgets::PropertyRow("mean intensity", text, nullptr);
            std::snprintf(text, sizeof(text), "%.1f", sensor.rateHz);
            widgets::PropertyRow("frame rate", text, "Hz");
            std::snprintf(text, sizeof(text), "%.3f", static_cast<double>(cloud->simTimeNs) * 1e-9);
            widgets::PropertyRow("frame time", text, "s");
            std::snprintf(text, sizeof(text), "%llu",
                          static_cast<unsigned long long>(sensor.stream->rejected()));
            widgets::PropertyRow("rejected frames", text, nullptr);
            ImGui::Unindent(px(8.0F));
        } else if (sensor.signals != 0 && widgets::SectionHeader("Signals", "live")) {
            char text[48];
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%zu", sensor.signals);
            widgets::PropertyRow("signals", text, nullptr);
            rate(sensor.signalDtNs, text, sizeof(text));
            widgets::PropertyRow("rate", text, nullptr);
            ImGui::Unindent(px(8.0F));
        }
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(
            "What degrades this sensor (fog, rain, EMI, noise and timing) is not "
            "sent by this server.");
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    std::shared_ptr<SensorsState> state_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createSensorsPanels() {
    auto state = std::make_shared<SensorsState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<SensorListPanel>(state));
    panels.push_back(std::make_unique<LidarPanel>(state));
    panels.push_back(std::make_unique<CameraPanel>(state));
    panels.push_back(std::make_unique<ImuPanel>(state));
    panels.push_back(std::make_unique<DegradationPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
