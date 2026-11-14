// Sensors workspace: the sensor list, the lidar bird's-eye view, the radar's range-azimuth view,
// the IMU scopes and a summary of the selected sensor (docs/design/ui/06-sensors/SPEC.md).
//
// Sensors are the stage's prims of a sensor type; their frames and signals are named after
// those prims. Nothing here knows a particular robot.
#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/RadarScale.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <numbers>
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

enum class Kind : std::uint8_t { Lidar, Camera, Imu, Radar, Other };

Kind kindOf(const std::string& typeName) {
    if (typeName == "Lidar") {
        return Kind::Lidar;
    }
    if (typeName == "Radar") {
        return Kind::Radar;
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
           typeName == "Magnetometer" || typeName == "Gnss" || typeName == "Radar" ||
           typeName == "CustomSensor";
}

struct Sensor {
    std::uint32_t prim{kNone};
    Kind kind{Kind::Other};
    std::shared_ptr<SensorStream> stream;  // sensors that send frames (lidar, camera, radar)
    // A custom sensor's images (a built-in kind's left, right, depth, ...) by signal name, and
    // their quantities ("left") for the Camera panel's modes; a camera's depth and instance.
    std::vector<std::string> images;
    std::vector<std::string> imageLabels;
    bool custom{false};
    std::size_t signals{0};  // sensors that send signals (IMU)
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
    /// Whether the catalog has sensor/<name>/<quantity>.
    static bool hasSignal(const PanelContext& context, const std::string& name,
                          const char* quantity) {
        if (!context.catalog) {
            return false;
        }
        const std::string wanted = "sensor/" + name + "/" + quantity;
        return std::any_of(context.catalog->signals.begin(), context.catalog->signals.end(),
                           [&wanted](const SignalInfo& info) { return info.name == wanted; });
    }

    void discover(PanelContext& context) {
        sensors.clear();
        std::vector<std::uint32_t> radarPrims;
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
            sensor.custom = prims[i].typeName == "CustomSensor";
            sensor.countedAt = ImGui::GetTime();
            if (context.catalog) {
                // Its images, by name: what a custom sensor shows (sensors.md §9 "Images of
                // kinds"), and a camera's other images. Not the delayed copies.
                for (const SignalInfo& info : context.catalog->signals) {
                    if (info.prim != i || !info.image ||
                        info.name.find("/delayed/") != std::string::npos) {
                        continue;
                    }
                    const std::size_t at = info.name.find('/', std::string("sensor/").size());
                    sensor.images.push_back(info.name);
                    sensor.imageLabels.push_back(
                        at == std::string::npos ? info.name : info.name.substr(at + 1));
                }
            }
            if (sensor.custom && !sensor.images.empty()) {
                sensor.kind = Kind::Camera;  // drawn in the Camera panel, by its images
                sensor.stream = context.client.watchSensor(sensor.images.front());
                sensor.countedFrames = sensor.stream->frames();
            } else if (sensor.custom && hasSignal(context, prims[i].name, "detections")) {
                // A custom sensor with detections and no image: an imaging radar, whose frames
                // the server sends by its prim (a kind it does not send for sends nothing).
                sensor.kind = Kind::Radar;
            }
            if (sensor.kind == Kind::Lidar || sensor.kind == Kind::Camera ||
                sensor.kind == Kind::Radar) {
                if (!sensor.stream) {
                    sensor.stream = context.client.watchSensor(prims[i].path);
                    sensor.countedFrames = sensor.stream->frames();
                }
            }
            if (sensor.kind == Kind::Radar) {
                radarPrims.push_back(i);
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
        if (!radarPrims.empty()) {
            context.client.requestAttributesOf(radarPrims);  // their fields of view
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
            const bool receiving = (sensor.stream && sensor.stream->frames() != 0) ||
                                   (!sensor.stream && sensor.signals != 0);
            char detail[96];
            if (sensor.stream && sensor.kind == Kind::Camera) {
                const std::shared_ptr<const SensorImage> image = sensor.stream->latestImage();
                std::snprintf(detail, sizeof(detail), "%u×%u  ·  %.1f Hz",
                              image ? image->width : 0U, image ? image->height : 0U, sensor.rateHz);
            } else if (sensor.stream) {
                const std::shared_ptr<const PointCloud> cloud = sensor.stream->latest();
                std::snprintf(detail, sizeof(detail), "%.1f Hz  ·  %u %s", sensor.rateHz,
                              cloud ? cloud->count : 0U,
                              sensor.kind == Kind::Radar ? "det" : "pts");
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

// --- radar --------------------------------------------------------------------------------

/// A radar's field of view and range from the attributes the server reports for its prim
/// (RadarAPI_1's sensor:radar:*, an imaging radar's sensor:custom:param:*); 0 when not known.
struct RadarView {
    float rangeM{0.0F};
    float azimuthFov{0.0F};
};

RadarView radarViewOf(const PanelContext& context, const Sensor& sensor) {
    RadarView view;
    const auto attributes = context.client.attributesOf();
    const auto it = attributes->find(sensor.prim);
    if (it == attributes->end()) {
        return view;
    }
    for (const AttributeInfo& a : it->second.attributes) {
        if (a.numbers.empty()) {
            continue;
        }
        const auto value = static_cast<float>(a.numbers.front());
        if (a.name == "sensor:radar:rangeMax" || a.name == "sensor:custom:param:max_range") {
            view.rangeM = value;
        } else if (a.name == "sensor:radar:azimuthFov" ||
                   a.name == "sensor:custom:param:azimuth_fov") {
            view.azimuthFov = value;
        }
    }
    return view;
}

ImU32 velocityColour(float velocity, float span) {
    const std::array<float, 4> c = radarVelocityColour(velocity, span);
    return u32(ImVec4(c[0], c[1], c[2], 1.0F));
}

/// Design 06-sensors, a sensor view per kind: the radar's detections over range and azimuth,
/// coloured by radial velocity (RadarScale.hpp), each dot sized by its RCS. Two modes: "fan",
/// the field of view from above with the sensor at the bottom (x forward up the screen, as the
/// lidar view), and "r–az", range up and azimuth across (a B-scope).
class RadarPanel final : public Panel {
public:
    explicit RadarPanel(std::shared_ptr<SensorsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Radar"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Sensors; }

    void draw(PanelContext& context) override {
        state_->follow(context);
        const Sensor* radar = state_->first(Kind::Radar);
        const std::string name = radar != nullptr && context.scene
                                     ? context.scene->statics->prims[radar->prim].name
                                     : std::string("Radar");
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin((name + " · range-azimuth###Radar").c_str());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        if (widgets::BeginPanelTools("##radar-tools", px(100.0F))) {
            static constexpr std::array<const char*, 2> kModes{"fan", "r–az"};
            widgets::Segmented("##mode", kModes.data(), 2, &mode_);
            widgets::EndPanelTools();
        }
        if (radar == nullptr) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context, *state_)) {
                widgets::Caption("This stage has no radar.");
            }
            ImGui::End();
            return;
        }
        const RadarView view = radarViewOf(context, *radar);
        const std::shared_ptr<const PointCloud> cloud =
            radar->stream ? radar->stream->latest() : nullptr;
        const bool have = cloud && cloud->radar();
        // What the view spans: the radar's own range and field of view, else what it detects.
        float fov = view.azimuthFov > 0.0F ? view.azimuthFov : std::numbers::pi_v<float> * 0.5F;
        float range = view.rangeM > 0.0F ? view.rangeM : 50.0F;
        float fastest = 0.0F;
        float farthest = 0.0F;
        for (std::uint32_t i = 0; have && i < cloud->count; ++i) {
            const float* p = &cloud->points[std::size_t{i} * kRadarFloats];
            farthest = std::max(farthest, std::hypot(p[0], p[1], p[2]));
            if (view.rangeM <= 0.0F) {
                range = std::max(range, farthest);
            }
            if (view.azimuthFov <= 0.0F) {
                fov = std::max(fov, 2.0F * std::abs(std::atan2(p[1], p[0])));
            }
            if (std::isfinite(p[3])) {
                fastest = std::max(fastest, std::abs(p[3]));
            }
        }
        const float span = radarVelocitySpan(fastest);
        if (zoomOf_ != radar->prim) {
            zoomOf_ = radar->prim;
            zoom_ = 1.0F;
        }
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton("##canvas", ImVec2(std::max(size.x, 1.0F), std::max(size.y, 1.0F)));
        if (ImGui::IsItemHovered() && ImGui::GetIO().MouseWheel != 0.0F) {
            zoom_ = std::clamp(zoom_ * std::pow(0.88F, ImGui::GetIO().MouseWheel), 0.05F, 1.0F);
        }
        // Metres to the top of the view: what the scan reaches (in round steps, so that the view
        // keeps still from scan to scan), up to the radar's range; the wheel zooms in from there.
        constexpr float kLeastShownM = 20.0F;
        const float reach = std::min(range, std::max(kLeastShownM, niceStep(farthest * 1.1F)));
        const float shown = reach * zoom_;
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const ImVec2 end(origin.x + size.x, origin.y + size.y);
        draw->PushClipRect(origin, end, true);
        draw->AddRectFilled(origin, end, u32(c::bg0));
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        const float half = fov * 0.5F;
        const float step = niceStep(shown / 4.0F);
        // Where a detection at (range, azimuth) is drawn, and the frame of each mode.
        ImVec2 sensorAt;
        float scale = 1.0F;  // pixels per metre (fan) or per metre of range (r–az)
        // Room for the count box above, and the legend and the azimuth labels below.
        const float plotTop = origin.y + px(mode_ == 0 ? 28.0F : 46.0F);
        const float plotBottom = end.y - px(mode_ == 0 ? 44.0F : 64.0F);
        const float plotLeft = origin.x + px(44.0F);
        const float plotRight = end.x - px(16.0F);
        if (mode_ == 0) {
            const float width = 2.0F * std::sin(std::min(half, std::numbers::pi_v<float> * 0.5F));
            scale = std::min((plotBottom - plotTop) / shown,
                             (plotRight - plotLeft) / std::max(width * shown, 1e-3F));
            sensorAt = ImVec2((plotLeft + plotRight) * 0.5F, plotBottom);
        } else {
            scale = (plotBottom - plotTop) / shown;
        }
        const auto place = [&](float r, float azimuth) {
            if (mode_ == 0) {
                return ImVec2(sensorAt.x - r * std::sin(azimuth) * scale,
                              sensorAt.y - r * std::cos(azimuth) * scale);
            }
            // Azimuth left (positive) on the left, as seen from behind the sensor.
            const float t = (half - azimuth) / std::max(fov, 1e-3F);
            return ImVec2(plotLeft + t * (plotRight - plotLeft), plotBottom - r * scale);
        };
        // Range marks, and the field of view's edges.
        for (float r = step; r <= shown + 1e-3F; r += step) {
            char label[16];
            std::snprintf(label, sizeof(label), "%.0f m", static_cast<double>(r));
            if (mode_ == 0) {
                constexpr int kSegments = 48;
                for (int k = 0; k < kSegments; ++k) {
                    const float a0 = -half + fov * static_cast<float>(k) / kSegments;
                    const float a1 = -half + fov * static_cast<float>(k + 1) / kSegments;
                    draw->AddLine(place(r, a0), place(r, a1), u32(c::lineStrong));
                }
                // On the left edge, where the view has room for it.
                const ImVec2 at = place(r, half);
                const float w = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, label).x;
                draw->AddText(fonts.mono, small, ImVec2(at.x - w - px(4.0F), at.y - small * 0.5F),
                              u32(c::t3), label);
            } else {
                const float y = plotBottom - r * scale;
                draw->AddLine(ImVec2(plotLeft, y), ImVec2(plotRight, y), u32(c::line));
                draw->AddText(fonts.mono, small, ImVec2(origin.x + px(6.0F), y - small * 0.5F),
                              u32(c::t3), label);
            }
        }
        if (mode_ == 0) {
            constexpr int kSegments = 48;  // the view's far edge
            for (int k = 0; k < kSegments; ++k) {
                draw->AddLine(place(shown, -half + fov * static_cast<float>(k) / kSegments),
                              place(shown, -half + fov * static_cast<float>(k + 1) / kSegments),
                              u32(c::line));
            }
            for (const float a : {-half, 0.0F, half}) {
                draw->AddLine(sensorAt, place(shown, a), u32(a == 0.0F ? c::line : c::lineStrong));
            }
        } else {
            draw->AddRect(ImVec2(plotLeft, plotTop), ImVec2(plotRight, plotBottom),
                          u32(c::lineStrong));
            const float degrees = 180.0F / std::numbers::pi_v<float>;
            for (const float a : {half, 0.0F, -half}) {
                char label[16];
                std::snprintf(label, sizeof(label), "%+.0f°", static_cast<double>(a * degrees));
                const ImVec2 at = place(0.0F, a);
                const float w = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, label).x;
                draw->AddLine(ImVec2(at.x, plotTop), ImVec2(at.x, plotBottom), u32(c::line));
                draw->AddText(fonts.mono, small, ImVec2(at.x - w * 0.5F, plotBottom + px(3.0F)),
                              u32(c::t3), label);
            }
        }
        // The detections, smallest first so that strong ones stay on top.
        std::uint32_t closing = 0;
        std::uint32_t opening = 0;
        if (have) {
            order_.resize(cloud->count);
            for (std::uint32_t i = 0; i < cloud->count; ++i) {
                order_[i] = i;
            }
            std::sort(order_.begin(), order_.end(), [&cloud](std::uint32_t a, std::uint32_t b) {
                return cloud->points[std::size_t{a} * kRadarFloats + 4] <
                       cloud->points[std::size_t{b} * kRadarFloats + 4];
            });
            for (const std::uint32_t i : order_) {
                const float* p = &cloud->points[std::size_t{i} * kRadarFloats];
                const float r = std::hypot(p[0], p[1], p[2]);
                const float azimuth = std::atan2(p[1], p[0]);
                const ImVec2 at = place(r, azimuth);
                if (r > shown || !std::isfinite(at.x) || !std::isfinite(at.y)) {
                    continue;
                }
                const float radius = px(2.0F) + px(5.0F) * radarRcsFraction(p[4]);
                draw->AddCircleFilled(at, radius, velocityColour(p[3], span), 12);
                closing += p[3] < 0.0F ? 1U : 0U;
                opening += p[3] > 0.0F ? 1U : 0U;
            }
        }
        if (mode_ == 0) {
            const float a = px(7.0F);  // the sensor: an accent arrow pointing forward
            draw->AddTriangleFilled(ImVec2(sensorAt.x, sensorAt.y - a * 1.4F),
                                    ImVec2(sensorAt.x - a, sensorAt.y + a),
                                    ImVec2(sensorAt.x + a, sensorAt.y + a), u32(c::accent));
        }
        draw->PopClipRect();

        // Overlays: the colour scale bottom left, the count bottom right.
        const ImVec2 bottomLeft(origin.x + px(10.0F), end.y - px(36.0F));
        if (!have) {
            box(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)),
                "no frames from this sensor yet", c::t2);
        } else {
            char count[96];
            std::snprintf(count, sizeof(count), "%u det  ·  %u closing  ·  %u away  ·  %.1f Hz",
                          cloud->count, closing, opening, radar->rateHz);
            const float width =
                fonts.mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, count).x + px(16.0F);
            box(draw, ImVec2(end.x - width - px(10.0F), origin.y + px(10.0F)), count, c::t1);
            legend(draw, bottomLeft, span);
        }
        if (const DomainView* domain = context.profileSession->find("sensors");
            domain != nullptr && !domain->enabled) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)));
            widgets::Pill("Sensors domain is off in the profile · ground truth sends no detections",
                          widgets::PillKind::Info);
        }
        ImGui::End();
    }

private:
    /// 1, 2 or 5 × 10ⁿ at or above `at least`.
    static float niceStep(float atLeast) {
        const float decade = std::pow(10.0F, std::floor(std::log10(std::max(atLeast, 1e-3F))));
        for (const float k : {1.0F, 2.0F, 5.0F}) {
            if (k * decade >= atLeast) {
                return k * decade;
            }
        }
        return 10.0F * decade;
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

    /// The diverging scale: closing (blue) through at rest to moving away (orange), ± span.
    static void legend(ImDrawList* draw, const ImVec2& min, float span) {
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        const float barWidth = px(120.0F);
        char low[24];
        char high[24];
        std::snprintf(low, sizeof(low), "%+.0f", -static_cast<double>(span));
        std::snprintf(high, sizeof(high), "%+.0f m/s", static_cast<double>(span));
        const float lowWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, low).x;
        const float highWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, high).x;
        const ImVec2 max(min.x + lowWidth + barWidth + highWidth + px(32.0F), min.y + px(26.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        draw->AddText(fonts.mono, small, ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(c::t2),
                      low);
        const float x = min.x + px(16.0F) + lowWidth;
        constexpr int kSteps = 24;
        for (int k = 0; k < kSteps; ++k) {
            const float t = (static_cast<float>(k) + 0.5F) / kSteps;
            draw->AddRectFilled(
                ImVec2(x + barWidth * static_cast<float>(k) / kSteps, min.y + px(9.0F)),
                ImVec2(x + barWidth * static_cast<float>(k + 1) / kSteps, min.y + px(17.0F)),
                velocityColour((t * 2.0F - 1.0F) * span, span));
        }
        draw->AddText(fonts.mono, small, ImVec2(x + barWidth + px(8.0F), min.y + px(6.0F)),
                      u32(c::t2), high);
    }

    std::shared_ptr<SensorsState> state_;
    int mode_{0};
    float zoom_{1.0F};  // share of the radar's range shown
    std::uint32_t zoomOf_{kNone};
    std::vector<std::uint32_t> order_;
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
        // Modes: a camera's rgb, depth and labels; a custom sensor's images by quantity.
        static constexpr std::array<const char*, 3> kModes{"rgb", "depth", "labels"};
        std::vector<const char*> modes(kModes.begin(), kModes.end());
        if (camera != nullptr && camera->custom) {
            modes.clear();
            for (const std::string& label : camera->imageLabels) {
                modes.push_back(label.c_str());
            }
        }
        if (widgets::BeginPanelTools("##camera-tools",
                                     px(50.0F) * static_cast<float>(modes.size()))) {
            widgets::Segmented("##mode", modes.data(), static_cast<int>(modes.size()), &mode_);
            widgets::EndPanelTools();
        }
        mode_ = std::clamp(mode_, 0, std::max(static_cast<int>(modes.size()) - 1, 0));
        const std::shared_ptr<SensorStream> stream = streamOf(context, camera);
        const std::shared_ptr<const SensorImage> image = stream ? stream->latestImage() : nullptr;
        if (camera == nullptr || !image) {
            ImGui::SetCursorPos(ImVec2(px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context, *state_)) {
                ImGui::PushTextWrapPos(0.0F);
                widgets::Caption("%s", camera == nullptr ? "This stage has no camera."
                                       : !stream
                                           ? "This server does not send this image."
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
            context.sensorTextures != nullptr
                ? context.sensorTextures->texture(name + "/" + std::to_string(mode_), image)
                : 0;
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
        if (camera->custom) {
            ImGui::TextColored(c::t3, "%s · %s", modes[static_cast<std::size_t>(mode_)],
                               image->encoding.c_str());
        } else {
            ImGui::TextColored(c::t3, "raster · no sensor model");
        }
        ImGui::PopFont();
        ImGui::End();
    }

private:
    /// The stream of the shown mode: the sensor's own for the first, else the image signal of
    /// the mode by name (watched while shown). Null when the server has no such image.
    std::shared_ptr<SensorStream> streamOf(PanelContext& context, const Sensor* camera) {
        std::string wanted;
        if (camera != nullptr && mode_ > 0) {
            if (camera->custom) {
                wanted = camera->images.at(static_cast<std::size_t>(mode_));
            } else {
                const char* quantity = mode_ == 1 ? "/depth" : "/instance";
                for (const std::string& name : camera->images) {
                    if (name.ends_with(quantity)) {
                        wanted = name;
                    }
                }
            }
        }
        if (wanted != watched_) {
            if (!watched_.empty()) {
                context.client.unwatchSensor(watched_);
            }
            named_ = wanted.empty() ? nullptr : context.client.watchSensor(wanted);
            watched_ = wanted;
        }
        if (camera == nullptr) {
            return nullptr;
        }
        return mode_ == 0 ? camera->stream : named_;
    }

    std::shared_ptr<SensorsState> state_;
    int mode_{0};
    std::string watched_;                  // the image signal watched for this mode, if any
    std::shared_ptr<SensorStream> named_;  // its stream
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
        if (cloud && cloud->radar() && widgets::SectionHeader("This scan", "measured")) {
            // A radar's detections, as they came: how many, how far, how fast, how strong.
            float nearest = FLT_MAX;
            float farthest = 0.0F;
            float closing = 0.0F;
            float away = 0.0F;
            double snr = 0.0;
            for (std::uint32_t i = 0; i < cloud->count; ++i) {
                const float* p = &cloud->points[std::size_t{i} * kRadarFloats];
                const float range = std::hypot(p[0], p[1], p[2]);
                nearest = std::min(nearest, range);
                farthest = std::max(farthest, range);
                closing = std::min(closing, p[3]);
                away = std::max(away, p[3]);
                snr += static_cast<double>(p[5]);
            }
            char text[48];
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%u", cloud->count);
            widgets::PropertyRow("detections", text, nullptr);
            std::snprintf(text, sizeof(text), "%.2f", cloud->count != 0 ? nearest : 0.0F);
            widgets::PropertyRow("nearest", text, "m");
            std::snprintf(text, sizeof(text), "%.2f", farthest);
            widgets::PropertyRow("farthest", text, "m");
            std::snprintf(text, sizeof(text), "%.2f", static_cast<double>(closing));
            widgets::PropertyRow("fastest closing", text, "m/s");
            std::snprintf(text, sizeof(text), "%.2f", static_cast<double>(away));
            widgets::PropertyRow("fastest away", text, "m/s");
            std::snprintf(
                text, sizeof(text), "%.1f",
                cloud->count != 0 && snr > 0.0 ? 10.0 * std::log10(snr / cloud->count) : 0.0);
            widgets::PropertyRow("mean SNR", text, "dB");
            std::snprintf(text, sizeof(text), "%.1f", sensor.rateHz);
            widgets::PropertyRow("frame rate", text, "Hz");
            std::snprintf(text, sizeof(text), "%.3f", static_cast<double>(cloud->simTimeNs) * 1e-9);
            widgets::PropertyRow("frame time", text, "s");
            ImGui::Unindent(px(8.0F));
        } else if (cloud && widgets::SectionHeader("This frame", "measured")) {
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
    panels.push_back(std::make_unique<RadarPanel>(state));
    panels.push_back(std::make_unique<CameraPanel>(state));
    panels.push_back(std::make_unique<ImuPanel>(state));
    panels.push_back(std::make_unique<DegradationPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
