// Environment workspace: what the environment is, a slice of one of its fields with a probe,
// and the list of fields that can be shown (docs/design/ui/07-environment/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// The "field" colour map of the design tokens (viridis), by fraction 0..1.
ImVec4 viridis(float fraction) {
    static const std::array<ImVec4, 5> kStops{theme::hex(0x440154), theme::hex(0x3B528B),
                                              theme::hex(0x21918C), theme::hex(0x5EC962),
                                              theme::hex(0xFDE725)};
    const float t = std::clamp(fraction, 0.0F, 1.0F) * 4.0F;
    const auto index = std::min(static_cast<std::size_t>(t), std::size_t{3});
    const float blend = t - static_cast<float>(index);
    const ImVec4& a = kStops.at(index);
    const ImVec4& b = kStops.at(index + 1);
    return {a.x + (b.x - a.x) * blend, a.y + (b.y - a.y) * blend, a.z + (b.z - a.z) * blend, 1.0F};
}

/// The air's speed: wind, gusts and the rotors' wakes (server kAirField, rotor_aero.md §5). The
/// only field with vertical slices and the airflow particles.
constexpr const char* kAirField = "aero/wind_speed";

/// What the three panels share: which field is shown and how.
struct EnvironmentView {
    std::string field;  // name of the field shown; empty for none
    std::shared_ptr<FieldStream> stream;
    double heightM{0.1};
    float opacity{0.85F};
    SliceOrientation orientation{SliceOrientation::Horizontal};

    /// Chooses the first available field when none is chosen, and keeps the stream open.
    void follow(PanelContext& context) {
        const std::shared_ptr<const EnvironmentInfo> info = context.client.environment();
        if (!info) {
            return;
        }
        const auto available = [&](const std::string& name) {
            return std::ranges::any_of(info->fields, [&](const EnvironmentInfo::Field& f) {
                return f.name == name && f.available;
            });
        };
        if (!field.empty() && !available(field)) {
            field.clear();
            stream.reset();
        }
        if (field.empty()) {
            for (const EnvironmentInfo::Field& candidate : info->fields) {
                if (candidate.available && candidate.components == 1) {
                    field = candidate.name;
                    break;
                }
            }
        }
        if (field != kAirField) {
            orientation = SliceOrientation::Horizontal;  // only the air is sliced upright
        }
        if (!field.empty()) {
            // The app's particles watch the air's XZ slice with volumes: ask for the same.
            const bool volume = field == kAirField && orientation == SliceOrientation::VerticalXZ &&
                                context.renderOptions != nullptr && context.renderOptions->airflow;
            stream = context.client.watchField(field, heightM, orientation, volume);
        }
    }
};

bool emptyState(const PanelContext& context) {
    if (context.client.environment()) {
        return false;
    }
    widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                               ? "This server does not describe its environment."
                               : "Not connected. The environment appears when a simulation "
                                 "server is reachable.");
    return true;
}

void row(const char* label, const char* unit, const char* format, double value) {
    char text[48];
    std::snprintf(text, sizeof(text), format, value);
    widgets::PropertyRow(label, text, unit);
}

// --- what the environment is --------------------------------------------------------------

/// A field of the Environment panel that changes the site while the run goes on
/// (Server::setSite): typing does not send; Enter or leaving the field does. The text follows the
/// server's value whenever that value changes, so what is typed stays until it is sent.
struct LiveField {
    std::array<char, 32> text{};
    double lastShown{std::nan("")};
    bool editing{false};  // typed in on the last frame: never refilled under the cursor

    /// Refills the text from `shown` (nullopt: unknown, empty) when the server's value moved.
    void follow(std::optional<double> shown, const char* format) {
        if (editing) {
            return;  // lastShown stays, so the server's value is shown once typing ends
        }
        const double value = shown.value_or(std::nan(""));
        const bool same = (std::isnan(value) && std::isnan(lastShown)) || value == lastShown;
        if (same) {
            return;
        }
        lastShown = value;
        if (shown) {
            std::snprintf(text.data(), text.size(), format, *shown);
        } else {
            text[0] = '\0';
        }
    }
    /// The number typed, when there is one.
    [[nodiscard]] std::optional<double> number() const {
        char* end = nullptr;
        const double v = std::strtod(text.data(), &end);
        if (end == text.data() || !std::isfinite(v)) {
            return std::nullopt;
        }
        return v;
    }
};

void sendSite(PanelContext& context, const char* key, double value) {
    context.client.setAttribute(kNone, std::string("site:") + key, value, false);
}

/// One editable row: returns the number when it was applied this frame.
std::optional<double> liveRow(const char* label, LiveField& field, const char* unit,
                              const char* tooltip) {
    (void)widgets::PropertyEdit(label, field.text.data(), field.text.size(), unit, false, tooltip);
    field.editing = ImGui::IsItemActive();
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        return field.number();
    }
    return std::nullopt;
}

/// What this viewer sent of the site that the server does not report back (the air the weather
/// needs), and the preset applied last; shared by the Environment and Presets tabs.
struct SentSite {
    std::optional<double> pressureHpa;
    std::optional<double> humidityPercent;
    std::string preset;  // the preset applied last; cleared by a value typed by hand
};

class EnvironmentPanel final : public Panel {
public:
    explicit EnvironmentPanel(std::shared_ptr<SentSite> sent) : sent_(std::move(sent)) {}
    [[nodiscard]] const char* title() const override { return "Environment"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Environment###EnvironmentState");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        if (!context.client.environment()) {
            ImGui::SetCursorPos(
                ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            emptyState(context);
            ImGui::End();
            return;
        }
        const EnvironmentInfo& info = *context.client.environment();
        ImGui::SetCursorPos(
            ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(6.0F)));
        widgets::Mono(c::t1, "%s",
                      info.layer.empty() ? "no environment layer" : info.layer.c_str());
        if (info.synthetic) {
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill("not modelled by this server", widgets::PillKind::Warn);
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float pad = px(8.0F);
        const auto domainOn = [&](const char* name) {
            const DomainView* view = context.profileSession->find(name);
            return view == nullptr || view->enabled;
        };

        // Every field below with a number in it changes the site live (Server::setSite):
        // Enter or leaving the field sends it, and the run goes on with it.
        const bool weatherRuns = std::ranges::any_of(
            info.fields,
            [](const EnvironmentInfo::Field& f) { return f.domain == "weather" && f.available; });
        const bool ambientKnown = info.ambientTemperatureK > 0.0;

        const ImVec4 thermal = widgets::domainColor("thermal");
        if (widgets::SectionHeader("Time and sun", nullptr, true, &thermal)) {
            ImGui::Indent(pad);
            char time[16];
            const int minutes = static_cast<int>(std::lround(info.localTimeHours * 60.0)) % 1440;
            std::snprintf(time, sizeof(time), "%02d:%02d", minutes / 60, minutes % 60);
            widgets::PropertyRow("local time", time, nullptr);
            row("sun elevation", "°", "%.1f", info.sunElevationDeg);
            ambient_.follow(
                ambientKnown ? std::optional(info.ambientTemperatureK - 273.15) : std::nullopt,
                "%.1f");
            if (const auto v =
                    liveRow("ambient", ambient_, "°C", "the air's temperature at the site")) {
                sendSite(context, "temperature", *v + 273.15);
                sent_->preset.clear();
            }
            ImGui::Unindent(pad);
        }

        const ImVec4 weather = widgets::domainColor("weather");
        if (widgets::SectionHeader("Weather", weatherRuns ? nullptr : "domain off", true,
                                   &weather)) {
            ImGui::Indent(pad);
            // The air the weather needs: not sent back by the server, so the panel holds what
            // it sent.
            // While it runs, a change goes at once; before, Start sends them together.
            pressure_.follow(sent_->pressureHpa, "%.1f");
            humidity_.follow(sent_->humidityPercent, "%.0f");
            if (const auto v = liveRow("pressure", pressure_, "hPa", "sea level is about 1013");
                v && weatherRuns) {
                sendSite(context, "pressure", *v * 100.0);
                sent_->pressureHpa = *v;
                sent_->preset.clear();
            }
            if (const auto v = liveRow("humidity", humidity_, "%", "relative humidity, 0 to 100");
                v && weatherRuns) {
                sendSite(context, "relative_humidity", *v / 100.0);
                sent_->humidityPercent = *v;
                sent_->preset.clear();
            }
            if (!weatherRuns) {
                widgets::Caption("The weather runs once the site has its air: an ambient");
                widgets::Caption("temperature (above), a pressure and a humidity.");
                const auto hPa = pressure_.number();
                const auto percent = humidity_.number();
                const bool ready = ambientKnown && hPa && percent;
                ImGui::BeginDisabled(!ready);
                if (widgets::PrimaryButton("Start weather")) {
                    sendSite(context, "pressure", *hPa * 100.0);
                    sendSite(context, "relative_humidity", *percent / 100.0);
                    sent_->pressureHpa = *hPa;
                    sent_->humidityPercent = *percent;
                }
                widgets::Caption("Or pick a preset in the Presets tab.");
                ImGui::EndDisabled();
            } else {
                rain_.follow(info.rainMmPerH, "%.1f");
                if (const auto v = liveRow("rain rate", rain_, "mm/h", "0 for none")) {
                    sendSite(context, "rain_rate", *v / kMmPerHourPerMps);
                    sent_->preset.clear();
                }
                fog_.follow(
                    info.fogVisibilityM > 0.0 ? std::optional(info.fogVisibilityM) : std::nullopt,
                    "%.0f");
                if (const auto v = liveRow("fog visibility", fog_, "m",
                                           "how far you can see; 10000 or more is clear air")) {
                    sendSite(context, "visibility", *v);
                    sent_->preset.clear();
                }
                row("dust", "mg/m³", "%.1f", info.dustMgPerM3);
                // Surface wetness, 0 dry to 1 standing water, as a meter.
                ImGui::AlignTextToFramePadding();
                ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
                ImGui::TextUnformatted("surface wetness");
                ImGui::PopStyleColor();
                char wet[16];
                std::snprintf(wet, sizeof(wet), "%.2f", info.surfaceWetness);
                ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
                const float width = ImGui::CalcTextSize(wet).x;
                ImGui::PopFont();
                ImGui::SameLine();
                widgets::AlignRight(px(120.0F) + px(8.0F) + width + pad);
                widgets::Meter(static_cast<float>(info.surfaceWetness), weather, px(120.0F));
                ImGui::SameLine(0.0F, px(8.0F));
                widgets::Mono(c::t1, "%s", wet);
            }
            ImGui::Unindent(pad);
        }
        const ImVec4 aero = widgets::domainColor("aerodynamics");
        if (widgets::SectionHeader("Wind", domainOn("aerodynamics") ? nullptr : "domain off", true,
                                   &aero)) {
            ImGui::Indent(pad);
            windSpeed_.follow(info.windSpeed, "%.1f");
            windFrom_.follow(info.windFromDeg, "%.0f");
            const int applied = widgets::PropertyEditPair(
                "mean",
                {windSpeed_.text.data(), windSpeed_.text.size(), "m/s", false,
                 "mean speed, 10 m above the ground"},
                {windFrom_.text.data(), windFrom_.text.size(), "° from", false,
                 "where it comes from: 0 north, 90 east, 180 south, 270 west"});
            windSpeed_.editing = (applied & 4) != 0;
            windFrom_.editing = (applied & 8) != 0;
            if ((applied & 1) != 0) {
                if (const auto v = windSpeed_.number()) {
                    sendSite(context, "wind_speed", *v);
                    sent_->preset.clear();
                }
            }
            if ((applied & 2) != 0) {
                if (const auto v = windFrom_.number()) {
                    sendSite(context, "wind_from", std::fmod(std::fmod(*v, 360.0) + 360.0, 360.0));
                }
            }
            row("gust intensity", nullptr, "%.2f", info.gustIntensity);
            widgets::PropertyRow(
                "field", info.windField.empty() ? "none — uniform wind" : info.windField.c_str(),
                nullptr);
            ImGui::Unindent(pad);
        }

        if (widgets::SectionHeader("Site", nullptr, false, nullptr)) {
            ImGui::Indent(pad);
            gravity_.follow(info.gravity, "%.3f");
            if (const auto v = liveRow("gravity", gravity_, "m/s²", "downward; Earth 9.807")) {
                sendSite(context, "gravity", *v);
            }
            ImGui::Unindent(pad);
        }
        const ImVec4 emi = widgets::domainColor("emi");
        char count[32];
        std::snprintf(count, sizeof(count), "%zu", info.emiSources.size());
        if (widgets::SectionHeader("EMI sources", count, true, &emi)) {
            ImGui::Indent(pad);
            if (info.emiSources.empty()) {
                widgets::Caption("No EMI source in this stage.");
            }
            for (const EnvironmentInfo::EmiSource& source : info.emiSources) {
                // Name, then frequency and current or power; the frequency is a parameter of
                // the site or the device and is shown as given.
                char numbers[96];
                char frequency[32];
                if (source.frequencyHz >= 1e9) {
                    std::snprintf(frequency, sizeof(frequency), "%g GHz", source.frequencyHz / 1e9);
                } else if (source.frequencyHz >= 1e3) {
                    std::snprintf(frequency, sizeof(frequency), "%g kHz", source.frequencyHz / 1e3);
                } else {
                    std::snprintf(frequency, sizeof(frequency), "%g Hz", source.frequencyHz);
                }
                if (source.currentA != 0.0) {
                    std::snprintf(numbers, sizeof(numbers), "%s · %g A", frequency,
                                  source.currentA);
                } else if (source.powerW != 0.0) {
                    std::snprintf(numbers, sizeof(numbers), "%s · %g W", frequency, source.powerW);
                } else {
                    std::snprintf(numbers, sizeof(numbers), "%s", frequency);
                }
                ImGui::PushID(source.name.c_str());
                if (ImGui::Selectable("##source", false, ImGuiSelectableFlags_AllowOverlap,
                                      ImVec2(0.0F, px(40.0F))) &&
                    source.prim != kNone) {
                    context.selectedPrim = source.prim;
                }
                ImGui::PopID();
                const ImVec2 min = ImGui::GetItemRectMin();
                const ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                const theme::Fonts& fonts = theme::fonts();
                draw->AddText(fonts.sansMedium, px(theme::kBodySize),
                              ImVec2(min.x, min.y + px(3.0F)), u32(c::t1), source.name.c_str());
                const float mono = px(theme::kMonoSize);
                const float width = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, numbers).x;
                draw->AddText(fonts.mono, mono, ImVec2(max.x - width - pad, min.y + px(4.0F)),
                              u32(c::t1), numbers);
                draw->AddText(fonts.mono, px(theme::kCaptionSize), ImVec2(min.x, min.y + px(23.0F)),
                              u32(c::t3), source.detail.c_str());
            }
            ImGui::Unindent(pad);
        }
        const ImVec4 radiation = widgets::domainColor("radiation");
        widgets::SectionHeader("Radiation", domainOn("radiation") ? "no sources" : "domain off",
                               false, &radiation);
        ImGui::End();
    }

private:
    static constexpr double kMmPerHourPerMps = 3.6e6;  // mm/h in 1 m/s

    std::shared_ptr<SentSite> sent_;
    LiveField ambient_, pressure_, humidity_, rain_, fog_, windSpeed_, windFrom_, gravity_;
};

// --- presets: a whole weather in one click -----------------------------------------------

/// A weather: every site value it sets (the wind's direction stays as it is). The numbers are
/// examples of each kind of weather, picked inside published bands, not measurements.
struct WeatherPreset {
    const char* name;
    const char* description;
    double temperatureC;
    double pressureHpa;
    double humidityPercent;
    double rainMmPerH;
    double visibilityM;  // fog: how far one can see
    double snowMmPerH;   // as water
    double snowCover;    // 0 bare ground .. 1 covered
    double windMps;      // mean, 10 m up
};

// Sources of the bands: wind, the Beaufort scale (WMO No. 306 / Met Office): force 4 "moderate
// breeze" 5.5–7.9 m/s, force 6 "strong breeze" 10.8–13.8, force 8 "gale" 17.2–20.7. Rain, the
// AMS Glossary of Meteorology: light under 2.5 mm/h, heavy over 7.6 mm/h; a thunderstorm's
// downpour 50 mm/h is illustrative. Fog, WMO No. 8: visibility under 1000 m; 200 m is thick fog,
// illustrative. Clear air is given 20 km of visibility. Snow 1 mm/h as water, the --season
// winter value. Pressure 1013.25 hPa is the ICAO standard atmosphere at sea level; temperatures
// and humidities are illustrative for each weather.
constexpr std::array<WeatherPreset, 8> kPresets{{
    {"Clear and calm", "20 °C, dry, no wind", 20.0, 1013.25, 50.0, 0.0, 20000.0, 0.0, 0.0, 0.0},
    {"Breezy", "7 m/s wind (Beaufort 4), dry", 20.0, 1013.25, 50.0, 0.0, 20000.0, 0.0, 0.0, 7.0},
    {"Strong wind", "12 m/s wind (Beaufort 6), dry", 18.0, 1010.0, 55.0, 0.0, 20000.0, 0.0, 0.0,
     12.0},
    {"Light rain", "2 mm/h, 3 m/s wind, 15 °C", 15.0, 1008.0, 90.0, 2.0, 10000.0, 0.0, 0.0, 3.0},
    {"Heavy rain", "25 mm/h, 6 m/s wind, 15 °C", 15.0, 1004.0, 95.0, 25.0, 5000.0, 0.0, 0.0, 6.0},
    {"Thunderstorm", "50 mm/h, 18 m/s wind (gale)", 18.0, 1000.0, 95.0, 50.0, 3000.0, 0.0, 0.0,
     18.0},
    {"Fog", "200 m visibility, 1 m/s wind, 10 °C", 10.0, 1018.0, 100.0, 0.0, 200.0, 0.0, 0.0, 1.0},
    {"Snow", "1 mm/h snow, ground covered, −3 °C", -3.0, 1015.0, 85.0, 0.0, 8000.0, 1.0, 1.0, 3.0},
}};

class PresetsPanel final : public Panel {
public:
    explicit PresetsPanel(std::shared_ptr<SentSite> sent) : sent_(std::move(sent)) {}
    [[nodiscard]] const char* title() const override { return "Presets"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        const bool open = ImGui::Begin("Presets###EnvironmentPresets");
        if (!open) {
            ImGui::End();
            return;
        }
        if (!context.client.environment()) {
            emptyState(context);
            ImGui::End();
            return;
        }
        widgets::Caption("A whole weather in one click: the temperature, the air, rain, fog,");
        widgets::Caption("snow and the wind's speed. The wind keeps its direction.");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const float width = ImGui::GetContentRegionAvail().x;
        for (const WeatherPreset& preset : kPresets) {
            if (widgets::OptionCard(preset.name, preset.name, preset.description,
                                    sent_->preset == preset.name, ImVec2(width, px(58.0F)))) {
                apply(context, preset);
            }
            ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        }
        if (!sent_->preset.empty()) {
            widgets::Caption("Values typed in the Environment tab change it from there.");
        }
        ImGui::End();
    }

private:
    static constexpr double kMmPerHourPerMps = 3.6e6;  // mm/h in 1 m/s

    void apply(PanelContext& context, const WeatherPreset& p) {
        // The air first, so that the weather runs before rain and fog are given to it.
        sendSite(context, "temperature", p.temperatureC + 273.15);
        sendSite(context, "pressure", p.pressureHpa * 100.0);
        sendSite(context, "relative_humidity", p.humidityPercent / 100.0);
        sendSite(context, "rain_rate", p.rainMmPerH / kMmPerHourPerMps);
        sendSite(context, "visibility", p.visibilityM);
        sendSite(context, "snow_rate", p.snowMmPerH / kMmPerHourPerMps);
        sendSite(context, "snow_cover", p.snowCover);
        sendSite(context, "wind_speed", p.windMps);
        sent_->pressureHpa = p.pressureHpa;
        sent_->humidityPercent = p.humidityPercent;
        sent_->preset = p.name;
    }

    std::shared_ptr<SentSite> sent_;
};

// --- a slice of the chosen field ----------------------------------------------------------

class FieldSlicePanel final : public Panel {
public:
    explicit FieldSlicePanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Slice 2D"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Slice 2D###FieldSlice");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        // --slice: a field and plane asked for on the command line.
        if (context.fieldSlice != nullptr && !context.fieldSlice->empty() &&
            context.client.environment()) {
            std::string asked = *context.fieldSlice;
            SliceOrientation plane = SliceOrientation::Horizontal;
            if (const std::size_t colon = asked.rfind(':'); colon != std::string::npos) {
                const std::string name = asked.substr(colon + 1);
                plane = name == "xz"   ? SliceOrientation::VerticalXZ
                        : name == "yz" ? SliceOrientation::VerticalYZ
                                       : SliceOrientation::Horizontal;
                asked.resize(colon);
            }
            view_->field = asked;
            view_->orientation = plane;
            view_->stream.reset();
            context.fieldSlice->clear();
        }
        view_->follow(context);
        // Header: the plane (XY, XZ, YZ: design 07's slice choices; upright only for the air)
        // and the height of a horizontal slice.
        if (widgets::BeginPanelTools("##slice-tools", px(150.0F))) {
            static constexpr std::array<const char*, 3> kPlanes{"XY", "XZ", "YZ"};
            int plane = static_cast<int>(view_->orientation);
            ImGui::BeginDisabled(view_->field != kAirField);
            if (widgets::Segmented("##plane", kPlanes.data(), static_cast<int>(kPlanes.size()),
                                   &plane)) {
                view_->orientation = static_cast<SliceOrientation>(plane);
                view_->stream.reset();
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(12.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Caption("height");
            ImGui::SameLine(0.0F, px(6.0F));
            ImGui::BeginDisabled(view_->orientation != SliceOrientation::Horizontal);
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            ImGui::SetNextItemWidth(px(160.0F));
            // A slider from the ground to 10 m; Ctrl+click types any height.
            double height = view_->heightM;
            constexpr double kLowest = 0.0;
            constexpr double kHighest = 10.0;
            if (ImGui::SliderScalar("##height", ImGuiDataType_Double, &height, &kLowest, &kHighest,
                                    "%.2f m") &&
                std::isfinite(height)) {
                view_->heightM = std::clamp(height, -100.0, 1000.0);
            }
            ImGui::PopFont();
            ImGui::EndDisabled();
            widgets::EndPanelTools();
        }
        if (!context.client.environment() || view_->field.empty()) {
            ImGui::SetCursorPos(
                ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context)) {
                widgets::Caption("No field of this run can be shown. The Overlays panel says why.");
            }
            ImGui::End();
            return;
        }
        const std::shared_ptr<const FieldSliceData> slice =
            view_->stream ? view_->stream->latest() : nullptr;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton("##canvas", size);
        const bool hovered = ImGui::IsItemHovered();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
                            u32(theme::hex(0x091120)));
        if (!slice) {
            draw->AddText(theme::fonts().sans, px(theme::kLabelSize),
                          ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), u32(c::t3),
                          "Waiting for the first slice ...");
            ImGui::End();
            return;
        }
        // Fit the slice into the panel, +X to the right and +Y up, as on a map.
        const double spanX = slice->cellM * slice->width;
        const double spanY = slice->cellM * slice->height;
        const float scale = std::min((size.x - px(40.0F)) / static_cast<float>(spanX),
                                     (size.y - px(40.0F)) / static_cast<float>(spanY));
        const double left = slice->originX - slice->cellM * 0.5;
        const double bottom = slice->originY - slice->cellM * 0.5;
        const ImVec2 base(origin.x + (size.x - static_cast<float>(spanX) * scale) * 0.5F,
                          origin.y + (size.y + static_cast<float>(spanY) * scale) * 0.5F);
        const auto toScreen = [&](double x, double y) {
            return ImVec2(base.x + static_cast<float>(x - left) * scale,
                          base.y - static_cast<float>(y - bottom) * scale);
        };
        const float range = std::max(slice->valueMax - slice->valueMin, 1e-12F);
        const float cell = static_cast<float>(slice->cellM) * scale;
        for (std::uint32_t r = 0; r < slice->height; ++r) {
            for (std::uint32_t col = 0; col < slice->width; ++col) {
                const float value =
                    slice->values[(static_cast<std::size_t>(r) * slice->width + col) *
                                  slice->components];
                ImVec4 color = viridis((value - slice->valueMin) / range);
                color.w = view_->opacity;
                const ImVec2 min(base.x + static_cast<float>(col) * cell,
                                 base.y - static_cast<float>(r + 1) * cell);
                draw->AddRectFilled(min, ImVec2(min.x + cell + 0.5F, min.y + cell + 0.5F),
                                    u32(color));
            }
        }
        draw->AddRect(toScreen(left, bottom + spanY), toScreen(left + spanX, bottom),
                      u32(c::lineStrong));

        // What is in the slice's area: the robots, at their positions.
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        if (context.scene) {
            const std::vector<PrimData>& prims = context.scene->statics->prims;
            for (std::uint32_t i = 0; i < prims.size(); ++i) {
                if (prims[i].parent != kNone) {
                    continue;
                }
                const bool robot = std::ranges::any_of(
                    prims, [i](const PrimData& other) { return other.parent == i; });
                if (!robot) {
                    continue;
                }
                const auto& position = context.scene->transforms[i].position;
                ImVec2 at = toScreen(position[0], position[1]);
                if (slice->orientation == SliceOrientation::VerticalXZ) {
                    at = toScreen(position[0], position[2]);
                } else if (slice->orientation == SliceOrientation::VerticalYZ) {
                    at = toScreen(position[1], position[2]);
                }
                draw->AddCircleFilled(at, px(4.0F), u32(c::t1));
                draw->AddCircle(at, px(7.0F), u32(c::accent), 0, px(1.5F));
                draw->AddText(fonts.sans, small, ImVec2(at.x + px(10.0F), at.y - small * 0.5F),
                              u32(c::t1), prims[i].name.c_str());
            }
        }

        const EnvironmentInfo& info = *context.client.environment();
        const EnvironmentInfo::Field* field = nullptr;
        for (const EnvironmentInfo::Field& candidate : info.fields) {
            if (candidate.name == view_->field) {
                field = &candidate;
            }
        }
        const char* unit = field != nullptr ? field->unit.c_str() : "";
        const char* label = field != nullptr ? field->label.c_str() : view_->field.c_str();

        // Top left: what the slice is.
        char text[160];
        const char* plane = slice->orientation == SliceOrientation::VerticalXZ   ? "y"
                            : slice->orientation == SliceOrientation::VerticalYZ ? "x"
                                                                                 : "z";
        std::snprintf(text, sizeof(text), "%s  ·  %u × %u cells of %.2f m  ·  %s %.2f m", label,
                      slice->width, slice->height, slice->cellM, plane, slice->heightM);
        box(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), text, c::t1);

        // Probe: the value under the cursor (design: crosshair with a tooltip).
        if (hovered) {
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const double x = left + static_cast<double>(mouse.x - base.x) / scale;
            const double y = bottom + static_cast<double>(base.y - mouse.y) / scale;
            const float value = slice->sample(x, y);
            if (!std::isnan(value)) {
                draw->AddLine(ImVec2(mouse.x - px(8.0F), mouse.y),
                              ImVec2(mouse.x + px(8.0F), mouse.y), u32(c::t1));
                draw->AddLine(ImVec2(mouse.x, mouse.y - px(8.0F)),
                              ImVec2(mouse.x, mouse.y + px(8.0F)), u32(c::t1));
                ImGui::BeginTooltip();
                if (slice->orientation == SliceOrientation::VerticalXZ) {
                    widgets::Mono(c::t3, "probe · x %.2f  y %.2f  z %.2f m", x, slice->heightM, y);
                } else if (slice->orientation == SliceOrientation::VerticalYZ) {
                    widgets::Mono(c::t3, "probe · x %.2f  y %.2f  z %.2f m", slice->heightM, x, y);
                } else {
                    widgets::Mono(c::t3, "probe · x %.2f  y %.2f  z %.2f m", x, y, slice->heightM);
                }
                widgets::Dot(field != nullptr ? widgets::domainColor(field->domain) : c::t2);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "%s  %.3g %s", label, static_cast<double>(value), unit);
                ImGui::EndTooltip();
            }
        }

        // Bottom right: the legend.
        const ImVec2 legendMin(origin.x + size.x - px(250.0F), origin.y + size.y - px(74.0F));
        const ImVec2 legendMax(origin.x + size.x - px(10.0F), origin.y + size.y - px(10.0F));
        draw->AddRectFilled(legendMin, legendMax, u32(theme::hex(0x091120, 0.92F)), px(4.0F));
        draw->AddRect(legendMin, legendMax, u32(c::lineStrong), px(4.0F));
        draw->AddText(fonts.sansSemiBold, px(theme::kLabelSize),
                      ImVec2(legendMin.x + px(10.0F), legendMin.y + px(7.0F)), u32(c::t1), label);
        const float unitWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, unit).x;
        draw->AddText(fonts.mono, small,
                      ImVec2(legendMax.x - unitWidth - px(10.0F), legendMin.y + px(8.0F)),
                      u32(c::t3), unit);
        const float barLeft = legendMin.x + px(10.0F);
        const float barRight = legendMax.x - px(10.0F);
        const float barTop = legendMin.y + px(28.0F);
        constexpr int kSteps = 48;
        for (int i = 0; i < kSteps; ++i) {
            const float a = static_cast<float>(i) / kSteps;
            draw->AddRectFilled(ImVec2(barLeft + (barRight - barLeft) * a, barTop),
                                ImVec2(barLeft + (barRight - barLeft) * (a + 1.0F / kSteps) + 0.5F,
                                       barTop + px(8.0F)),
                                u32(viridis(a)));
        }
        std::snprintf(text, sizeof(text), "%.3g", static_cast<double>(slice->valueMin));
        draw->AddText(fonts.mono, small, ImVec2(barLeft, barTop + px(12.0F)), u32(c::t3), text);
        std::snprintf(text, sizeof(text), "%.3g", static_cast<double>(slice->valueMax));
        const float maxWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
        draw->AddText(fonts.mono, small, ImVec2(barRight - maxWidth, barTop + px(12.0F)),
                      u32(c::t3), text);
        ImGui::End();
    }

private:
    static void box(ImDrawList* draw, const ImVec2& min, const char* text, const ImVec4& color) {
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kMonoSize);
        const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, text).x;
        const ImVec2 max(min.x + width + px(16.0F), min.y + px(26.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        draw->AddText(font, size, ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(color), text);
    }

    std::shared_ptr<EnvironmentView> view_;
};

// --- which fields can be shown ------------------------------------------------------------

class OverlaysPanel final : public Panel {
public:
    explicit OverlaysPanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Overlays"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (emptyState(context)) {
            ImGui::End();
            return;
        }
        view_->follow(context);
        const EnvironmentInfo& info = *context.client.environment();
        // One scalar slice at a time: choosing a field replaces the one shown.
        for (const EnvironmentInfo::Field& field : info.fields) {
            ImGui::PushID(field.name.c_str());
            bool on = view_->field == field.name;
            ImGui::BeginDisabled(!field.available || field.components != 1);
            if (widgets::Checkbox("##show", &on)) {
                view_->field = on ? field.name : std::string{};
                view_->stream.reset();
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Swatch(widgets::domainColor(field.domain));
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushStyleColor(ImGuiCol_Text, field.available ? c::t1 : c::t3);
            ImGui::TextUnformatted(field.label.c_str());
            ImGui::PopStyleColor();
            // On the right: the unit, or why the field cannot be shown.
            const char* note =
                field.available ? field.unit.c_str() : field.unavailableReason.c_str();
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            const float width = ImGui::CalcTextSize(note).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(width);
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::TextColored(c::t3, "%s", note);
            ImGui::PopFont();
            ImGui::PopID();
        }
        // Vectors combine with the slice (design 07): the air's motion as moving particles.
        const auto air = std::ranges::find_if(
            info.fields, [](const EnvironmentInfo::Field& f) { return f.name == kAirField; });
        if (air != info.fields.end()) {
            ImGui::PushID("airflow");
            bool& on = context.renderOptions->airflow;
            ImGui::BeginDisabled(!air->available);
            if (!air->available) {
                on = false;
            }
            widgets::Checkbox("##show", &on);
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Swatch(widgets::domainColor(air->domain));
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushStyleColor(ImGuiCol_Text, air->available ? c::t1 : c::t3);
            ImGui::TextUnformatted("airflow · particles");
            ImGui::PopStyleColor();
            const char* note = air->available ? "m/s" : air->unavailableReason.c_str();
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            const float width = ImGui::CalcTextSize(note).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(width);
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::TextColored(c::t3, "%s", note);
            ImGui::PopFont();
            ImGui::PopID();
        }
        const std::shared_ptr<const FieldSliceData> slice =
            view_->stream ? view_->stream->latest() : nullptr;
        if (slice && widgets::SectionHeader("Selected layer", nullptr)) {
            ImGui::Indent(px(8.0F));
            char text[64];
            std::snprintf(text, sizeof(text), "%u × %u", slice->width, slice->height);
            widgets::PropertyRow("grid", text, "cells");
            row("cell", "m", "%.2f", slice->cellM);
            std::snprintf(text, sizeof(text), "%.1f × %.1f", slice->cellM * slice->width,
                          slice->cellM * slice->height);
            widgets::PropertyRow("bounds", text, "m");
            row("lowest", nullptr, "%.4g", static_cast<double>(slice->valueMin));
            row("highest", nullptr, "%.4g", static_cast<double>(slice->valueMax));
            std::snprintf(text, sizeof(text), "%llu",
                          static_cast<unsigned long long>(view_->stream->slices()));
            widgets::PropertyRow("slices received", text, nullptr);
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted("opacity");
            ImGui::PopStyleColor();
            ImGui::SameLine(px(8.0F) + px(112.0F));
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::SliderFloat("##opacity", &view_->opacity, 0.1F, 1.0F, "%.2f");
            widgets::PropertyRow("colormap", "viridis", nullptr);
            ImGui::Unindent(px(8.0F));
        }
        if (info.synthetic) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "This server has no environment model. A field shown here is an "
                "estimate it derives from the recording.");
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }

private:
    std::shared_ptr<EnvironmentView> view_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createEnvironmentPanels() {
    auto view = std::make_shared<EnvironmentView>();
    std::vector<std::unique_ptr<Panel>> panels;
    auto sent = std::make_shared<SentSite>();
    panels.push_back(std::make_unique<EnvironmentPanel>(sent));
    panels.push_back(std::make_unique<PresetsPanel>(sent));
    panels.push_back(std::make_unique<FieldSlicePanel>(view));
    panels.push_back(std::make_unique<OverlaysPanel>(view));
    return panels;
}

}  // namespace lagrange::viewer
