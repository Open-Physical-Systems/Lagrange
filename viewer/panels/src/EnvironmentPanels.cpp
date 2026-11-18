// Environment workspace: what the environment is, a slice of one of its fields with a probe,
// and the list of fields that can be shown (docs/design/ui/07-environment/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Camera.hpp"
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
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
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
constexpr const char* kSprayField = "aero/spray";  // the spray off water (downwash.md §5b)
constexpr const char* kWaterSurfaceField = "water/surface";  // its moving surface (water.md)
/// |B| of the EMI sources; sliced upright through the robot too (RFC 0013: its own wiring).
constexpr const char* kFluxField = "emi/flux_density";

/// Whether the server slices this field upright as well as level.
bool slicedUpright(const std::string& field) {
    return field == kAirField || field == kFluxField;
}

/// What the three panels share: which field is shown and how.
struct EnvironmentView {
    std::string field;  // name of the field shown; empty for none
    std::shared_ptr<FieldStream> stream;
    double heightM{0.1};
    float opacity{0.85F};
    SliceOrientation orientation{SliceOrientation::Horizontal};
    /// How the layer's values are coloured (the Overlays panel's "contrast"): as they are, on a
    /// symmetric log scale over four decades below the largest |value|, or high-passed (less
    /// their mean over kHighPassCells round each cell), so that sub-millimetre ripples show beside
    /// a dimple of centimetres (water/surface).
    enum class Contrast : int { Linear = 0, SymmetricLog = 1, HighPass = 2 };
    Contrast contrast{Contrast::Linear};
    static constexpr int kHighPassCells = 3;  // the mean's half width, cells

    /// The slice to draw: the stream's latest, high-passed when asked (made once per slice).
    std::shared_ptr<const FieldSliceData> shown() {
        std::shared_ptr<const FieldSliceData> latest = stream ? stream->latest() : nullptr;
        if (!latest || contrast != Contrast::HighPass || latest->components != 1U) {
            return latest;
        }
        if (latest != passedOf_) {
            passedOf_ = latest;
            passed_ = highPassed(*latest);
        }
        return passed_;
    }

    /// `slice` less the mean of its finite values within kHighPassCells cells round each cell.
    static std::shared_ptr<const FieldSliceData> highPassed(const FieldSliceData& slice) {
        auto out = std::make_shared<FieldSliceData>(slice);
        const auto w = static_cast<int>(slice.width);
        const auto h = static_cast<int>(slice.height);
        float lo = 0.0F;
        float hi = 0.0F;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const float v = slice.values[static_cast<std::size_t>(y) * w + x];
                if (!std::isfinite(v)) {
                    continue;
                }
                double sum = 0.0;
                int count = 0;
                for (int dy = -kHighPassCells; dy <= kHighPassCells; ++dy) {
                    for (int dx = -kHighPassCells; dx <= kHighPassCells; ++dx) {
                        const int xx = x + dx;
                        const int yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= w || yy >= h) {
                            continue;
                        }
                        const float n = slice.values[static_cast<std::size_t>(yy) * w + xx];
                        if (std::isfinite(n)) {
                            sum += n;
                            ++count;
                        }
                    }
                }
                const auto passed = static_cast<float>(v - sum / count);
                out->values[static_cast<std::size_t>(y) * w + x] = passed;
                lo = std::min(lo, passed);
                hi = std::max(hi, passed);
            }
        }
        const float most = std::max(-lo, hi);  // symmetric about 0
        out->valueMin = -most;
        out->valueMax = most;
        return out;
    }

    /// Chooses the first available field when none is chosen, and keeps the stream open.
    void follow(PanelContext& context) {
        // --slice: a field and plane asked for on the command line.
        if (context.fieldSlice != nullptr && !context.fieldSlice->empty() &&
            context.client.environment()) {
            std::string asked = *context.fieldSlice;
            // An optional last part: the contrast (water/surface:xy:highpass).
            for (const auto& [word, mode] : {std::pair{":linear", Contrast::Linear},
                                             std::pair{":symlog", Contrast::SymmetricLog},
                                             std::pair{":highpass", Contrast::HighPass}}) {
                if (asked.ends_with(word)) {
                    contrast = mode;
                    asked.resize(asked.size() - std::string_view(word).size());
                }
            }
            SliceOrientation plane = SliceOrientation::Horizontal;
            if (const std::size_t colon = asked.rfind(':'); colon != std::string::npos) {
                const std::string name = asked.substr(colon + 1);
                plane = name == "xz"   ? SliceOrientation::VerticalXZ
                        : name == "yz" ? SliceOrientation::VerticalYZ
                                       : SliceOrientation::Horizontal;
                asked.resize(colon);
            }
            field = asked;
            orientation = plane;
            stream.reset();
            context.fieldSlice->clear();
        }
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
        // Design: |B| is the layer shown first when the run has EMI; else the first one.
        if (field.empty() && available(kFluxField)) {
            field = kFluxField;
        }
        if (field.empty()) {
            for (const EnvironmentInfo::Field& candidate : info->fields) {
                if (candidate.available && candidate.components == 1) {
                    field = candidate.name;
                    break;
                }
            }
        }
        if (!slicedUpright(field)) {
            orientation = SliceOrientation::Horizontal;  // only these are sliced upright
        }
        if (!field.empty()) {
            // The app's particles watch the air's XZ slice with volumes: ask for the same.
            // The water's surface and the spray are always asked with their boxes: the
            // realistic view and the spray overlay draw from them.
            const bool volume =
                (field == kAirField && orientation == SliceOrientation::VerticalXZ &&
                 context.renderOptions != nullptr && context.renderOptions->airflow) ||
                field == kWaterSurfaceField || field == kSprayField;
            stream = context.client.watchField(field, heightM, orientation, volume);
        }
    }

private:
    std::shared_ptr<const FieldSliceData> passedOf_, passed_;
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

/// Whether a domain runs now, by the profile the server applied; true before it said.
bool domainRuns(const PanelContext& context, std::string_view name) {
    const std::shared_ptr<const ProfileInfo>& profile = context.profileSession->profile();
    const DomainInfo* info = profile ? profile->find(name) : nullptr;
    return info == nullptr || info->enabled;
}

/// Wind with the aerodynamics domain off moves nothing: say so where the wind is set.
void windWarning() {
    widgets::Pill("aerodynamics off", widgets::PillKind::Warn);
    ImGui::SameLine(0.0F, px(6.0F));
    ImGui::PushTextWrapPos(0.0F);
    widgets::Caption(
        "The wind pushes nothing: the aerodynamics domain does not run (it needs "
        "AeroAPI_1 links or rotors and the site's air density).");
    ImGui::PopTextWrapPos();
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
        // Design: "Swap..." picks another environment; the server takes whole weathers as the
        // presets (there is no layer to load by name over the protocol), so it opens them.
        ImGui::SameLine();
        widgets::AlignRight(widgets::ButtonWidth("Swap…") + px(8.0F));
        if (widgets::Button("Swap…", widgets::ButtonKind::Ghost)) {
            ImGui::SetWindowFocus("Presets###EnvironmentPresets");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Pick another weather from the presets");
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float pad = px(8.0F);
        // Whether a domain runs now: the server's word on the applied profile, not the draft
        // being edited in the Domains workspace.
        const auto domainOn = [&](const char* name) { return domainRuns(context, name); };

        // Every field below with a number in it changes the site live (Server::setSite):
        // Enter or leaving the field sends it, and the run goes on with it.
        const bool weatherRuns = std::ranges::any_of(
            info.fields,
            [](const EnvironmentInfo::Field& f) { return f.domain == "weather" && f.available; });
        const bool ambientKnown = info.ambientTemperatureK > 0.0;

        const ImVec4 thermal = widgets::domainColor("thermal");
        if (widgets::SectionHeader("Time and sun", nullptr, true, &thermal)) {
            ImGui::Indent(pad);
            // A server without a time of day sends NaN (an older one, 0 and 0): a dash, not
            // midnight with the sun on the horizon.
            const bool modelled =
                std::isfinite(info.localTimeHours) && std::isfinite(info.sunElevationDeg) &&
                (info.synthetic || info.localTimeHours != 0.0 || info.sunElevationDeg != 0.0);
            constexpr const char* kNoSun =
                "not modelled: this run has no time of day and no sun position";
            if (modelled) {
                char time[16];
                const int minutes =
                    static_cast<int>(std::lround(info.localTimeHours * 60.0)) % 1440;
                std::snprintf(time, sizeof(time), "%02d:%02d", minutes / 60, minutes % 60);
                widgets::PropertyRow("local time", time, nullptr);
                row("sun elevation", "°", "%.1f", info.sunElevationDeg);
            } else {
                widgets::PropertyRow("local time", "—", nullptr, false, kNoSun);
                widgets::PropertyRow("sun elevation", "—", "°", false, kNoSun);
            }
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
            if (!domainOn("aerodynamics") && info.windSpeed > 0.0) {
                windWarning();
            }
            row("gust intensity", nullptr, "%.2f", info.gustIntensity);
            widgets::PropertyRow(
                "field", info.windField.empty() ? "none — uniform wind" : info.windField.c_str(),
                nullptr);
            robotInTheWind(context);
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

    /// The robot in the wind: its speed over the ground, the air's speed past it and its tilt,
    /// for the first body with an airspeed (body/<b>/airspeed, the aerodynamics domain's; the
    /// robot ROS drives comes first). A drone holding its ground speed leans further into the
    /// wind instead of slowing; these rows show it.
    void robotInTheWind(PanelContext& context) {
        const auto catalog = context.client.catalog();
        if (catalog != robotCatalog_) {
            robotCatalog_ = catalog;
            for (auto& b : robotStreams_) {
                if (b) {
                    context.client.unsubscribe(b);
                }
                b.reset();
            }
            robotBody_.clear();
            if (catalog) {
                for (const SignalInfo& s : catalog->signals) {
                    constexpr std::string_view kPrefix = "body/";
                    constexpr std::string_view kSuffix = "/airspeed";
                    const std::string_view n = s.name;
                    if (n.starts_with(kPrefix) && n.ends_with(kSuffix) &&
                        n.size() > kPrefix.size() + kSuffix.size()) {
                        robotBody_ = std::string(
                            n.substr(kPrefix.size(), n.size() - kPrefix.size() - kSuffix.size()));
                        break;
                    }
                }
            }
            if (!robotBody_.empty()) {
                const std::string base = "body/" + robotBody_ + "/";
                robotStreams_ = {context.client.subscribe(base + "airspeed"),
                                 context.client.subscribe(base + "twist", 0),
                                 context.client.subscribe(base + "twist", 1),
                                 context.client.subscribe(base + "pose", 4),
                                 context.client.subscribe(base + "pose", 5)};
            }
        }
        if (robotBody_.empty()) {
            return;
        }
        // A stream older than a second (paused long, or gone) shows a dash.
        const std::int64_t now = context.scene ? context.scene->simTimeNs : 0;
        const auto value = [&](std::size_t i) -> std::optional<double> {
            const auto& b = robotStreams_.at(i);
            if (!b || b->newestTimeNs() < now - 1'000'000'000) {
                return std::nullopt;
            }
            const double v = b->latest();
            return std::isfinite(v) ? std::optional<double>(v) : std::nullopt;
        };
        const auto show = [](const char* label, std::optional<double> v, const char* format,
                             const char* unit) {
            char text[48];
            if (v) {
                std::snprintf(text, sizeof(text), format, *v);
            } else {
                std::snprintf(text, sizeof(text), "—");
            }
            widgets::PropertyRow(label, text, unit);
        };
        const auto air = value(0);
        const auto vx = value(1);
        const auto vy = value(2);
        const auto qx = value(3);
        const auto qy = value(4);
        show("robot ground speed",
             vx && vy ? std::optional<double>(std::hypot(*vx, *vy)) : std::nullopt, "%.2f", "m/s");
        show("robot airspeed", air, "%.2f", "m/s");
        std::optional<double> tilt;
        if (qx && qy) {
            // The body's up axis against the vertical: its z component is 1 − 2(x² + y²).
            tilt = std::acos(std::clamp(1.0 - 2.0 * (*qx * *qx + *qy * *qy), -1.0, 1.0)) * 180.0 /
                   std::numbers::pi;
        }
        show("robot tilt", tilt, "%.1f", "°");
    }

    std::shared_ptr<const SignalCatalog> robotCatalog_;
    std::string robotBody_;
    std::array<std::shared_ptr<ScopeBuffer>, 5> robotStreams_{};

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
            const auto info = context.client.environment();
            if (!domainRuns(context, "aerodynamics") && info && info->windSpeed > 0.0) {
                windWarning();
            }
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

/// The header of both field views: the plane (XY, XZ, YZ: design 07's slice choices; upright
/// only for the air and |B|) and the height of a level slice.
void sliceTools(EnvironmentView& view, const char* id) {
    // Wide enough for all of it: "slice", three planes, "height" and a 130 px field.
    if (!widgets::BeginPanelTools(id, px(340.0F))) {
        return;
    }
    ImGui::AlignTextToFramePadding();
    widgets::Caption("slice");
    ImGui::SameLine(0.0F, px(6.0F));
    static constexpr std::array<const char*, 3> kPlanes{"XY", "XZ", "YZ"};
    int plane = static_cast<int>(view.orientation);
    ImGui::BeginDisabled(!slicedUpright(view.field));
    if (widgets::Segmented("##plane", kPlanes.data(), static_cast<int>(kPlanes.size()), &plane)) {
        view.orientation = static_cast<SliceOrientation>(plane);
        view.stream.reset();
    }
    ImGui::EndDisabled();
    ImGui::SameLine(0.0F, px(12.0F));
    ImGui::AlignTextToFramePadding();
    widgets::Caption("height");
    ImGui::SameLine(0.0F, px(6.0F));
    ImGui::BeginDisabled(view.orientation != SliceOrientation::Horizontal);
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    ImGui::SetNextItemWidth(px(130.0F));
    // A slider from the ground to 10 m; Ctrl+click types any height.
    double height = view.heightM;
    constexpr double kLowest = 0.0;
    constexpr double kHighest = 10.0;
    if (ImGui::SliderScalar("##height", ImGuiDataType_Double, &height, &kLowest, &kHighest,
                            "%.2f m") &&
        std::isfinite(height)) {
        view.heightM = std::clamp(height, -100.0, 1000.0);
    }
    ImGui::PopFont();
    ImGui::EndDisabled();
    widgets::EndPanelTools();
}

/// How a slice's values map onto the colour map: |B| spans decades (it falls as 1/r to 1/r³), so
/// it is coloured on a log scale over its top four decades; the rest linearly.
struct SliceScale {
    bool logScale{false};
    float logLow{0.0F};
    float logHigh{0.0F};
    float lowest{0.0F};
    float range{1.0F};
    /// The symmetric log scale (EnvironmentView::Contrast): 0 in the middle, ±most at the ends.
    bool symmetricLog{false};
    float most{1.0F};

    SliceScale(const std::string& field, const FieldSliceData& slice,
               EnvironmentView::Contrast contrast = EnvironmentView::Contrast::Linear)
        : logScale(field == kFluxField && slice.valueMax > 0.0F &&
                   contrast == EnvironmentView::Contrast::Linear)
        , logHigh(logScale ? std::log10(slice.valueMax) : 0.0F)
        , lowest(slice.valueMin)
        , range(std::max(slice.valueMax - slice.valueMin, 1e-12F))
        , symmetricLog(contrast == EnvironmentView::Contrast::SymmetricLog)
        , most(std::max({std::abs(slice.valueMin), std::abs(slice.valueMax), 1e-30F})) {
        logLow = logScale ? std::max(std::log10(std::max(slice.valueMin, 1e-30F)), logHigh - 4.0F)
                          : 0.0F;
    }
    [[nodiscard]] float fraction(float value) const {
        if (symmetricLog) {
            // sign(v)·log10(1 + |v|/v0)/log10(1 + most/v0), v0 four decades below the most.
            const float v0 = most * 1e-4F;
            const float scaled =
                std::log10(1.0F + std::abs(value) / v0) / std::log10(1.0F + most / v0);
            return 0.5F + 0.5F * std::copysign(scaled, value);
        }
        if (logScale) {
            return (std::log10(std::max(value, 1e-30F)) - logLow) /
                   std::max(logHigh - logLow, 1e-6F);
        }
        return (value - lowest) / range;
    }
};

/// The field the view shows, as the server describes it; null when none.
const EnvironmentInfo::Field* shownField(const PanelContext& context, const EnvironmentView& view) {
    const std::shared_ptr<const EnvironmentInfo> info = context.client.environment();
    if (!info) {
        return nullptr;
    }
    for (const EnvironmentInfo::Field& candidate : info->fields) {
        if (candidate.name == view.field) {
            return &candidate;
        }
    }
    return nullptr;
}

/// Design: the legend at bottom right of a field view: the field's name and unit, the colour
/// bar and its range; `wind` adds the air's line when its particles are drawn.
void sliceLegend(ImDrawList* draw, const ImVec2& origin, const ImVec2& size, const char* label,
                 const char* unit, const std::string& field, const FieldSliceData& slice,
                 const char* wind,
                 EnvironmentView::Contrast contrast = EnvironmentView::Contrast::Linear) {
    const theme::Fonts& fonts = theme::fonts();
    const float small = px(theme::kCaptionSize);
    const float tall = wind != nullptr ? px(96.0F) : px(74.0F);
    const ImVec2 legendMin(origin.x + size.x - px(250.0F), origin.y + size.y - tall);
    const ImVec2 legendMax(origin.x + size.x - px(10.0F), origin.y + size.y - px(10.0F));
    draw->AddRectFilled(legendMin, legendMax, u32(theme::hex(0x091120, 0.92F)), px(4.0F));
    draw->AddRect(legendMin, legendMax, u32(c::lineStrong), px(4.0F));
    draw->AddText(fonts.sansSemiBold, px(theme::kLabelSize),
                  ImVec2(legendMin.x + px(10.0F), legendMin.y + px(7.0F)), u32(c::t1), label);
    const float unitWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, unit).x;
    draw->AddText(fonts.mono, small,
                  ImVec2(legendMax.x - unitWidth - px(10.0F), legendMin.y + px(8.0F)), u32(c::t3),
                  unit);
    const float barLeft = legendMin.x + px(10.0F);
    const float barRight = legendMax.x - px(10.0F);
    const float barTop = legendMin.y + px(28.0F);
    constexpr int kSteps = 48;
    for (int i = 0; i < kSteps; ++i) {
        const float a = static_cast<float>(i) / kSteps;
        draw->AddRectFilled(
            ImVec2(barLeft + (barRight - barLeft) * a, barTop),
            ImVec2(barLeft + (barRight - barLeft) * (a + 1.0F / kSteps) + 0.5F, barTop + px(8.0F)),
            u32(viridis(a)));
    }
    const bool symmetric = contrast != EnvironmentView::Contrast::Linear;
    const bool logLegend = field == kFluxField && slice.valueMax > 0.0F && !symmetric;
    const double most = std::max(std::abs(static_cast<double>(slice.valueMin)),
                                 std::abs(static_cast<double>(slice.valueMax)));
    const double lowest = logLegend   ? std::max(static_cast<double>(slice.valueMin),
                                                 static_cast<double>(slice.valueMax) * 1e-4)
                          : symmetric ? -most
                                      : static_cast<double>(slice.valueMin);
    char text[64];
    std::snprintf(text, sizeof(text),
                  logLegend                                             ? "%.3g · log"
                  : contrast == EnvironmentView::Contrast::SymmetricLog ? "%.3g · symlog"
                  : contrast == EnvironmentView::Contrast::HighPass     ? "%.3g · high-pass"
                                                                        : "%.3g",
                  lowest);
    draw->AddText(fonts.mono, small, ImVec2(barLeft, barTop + px(12.0F)), u32(c::t3), text);
    std::snprintf(text, sizeof(text), "%.3g",
                  symmetric ? most : static_cast<double>(slice.valueMax));
    const float maxWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
    draw->AddText(fonts.mono, small, ImVec2(barRight - maxWidth, barTop + px(12.0F)), u32(c::t3),
                  text);
    if (wind != nullptr) {
        const float y = barTop + px(36.0F);
        draw->AddLine(ImVec2(barLeft, y), ImVec2(barLeft + px(18.0F), y),
                      u32(widgets::domainColor("aerodynamics")), px(2.0F));
        draw->AddText(fonts.sans, small, ImVec2(barLeft + px(26.0F), y - small * 0.5F), u32(c::t2),
                      wind);
    }
}

/// A caption in a box over a field view (its top left: what is shown).
void viewBox(ImDrawList* draw, const ImVec2& min, const char* text, const ImVec4& color) {
    ImFont* font = theme::fonts().mono;
    const float size = px(theme::kMonoSize);
    const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, text).x;
    const ImVec2 max(min.x + width + px(16.0F), min.y + px(26.0F));
    draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
    draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
    draw->AddText(font, size, ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(color), text);
}

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
        view_->follow(context);
        sliceTools(*view_, "##slice-tools");
        if (!context.client.environment() || view_->field.empty()) {
            ImGui::SetCursorPos(
                ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context)) {
                widgets::Caption("No field of this run can be shown. The Overlays panel says why.");
            }
            ImGui::End();
            return;
        }
        const std::shared_ptr<const FieldSliceData> slice = view_->shown();
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
        const SliceScale colours(view_->field, *slice, view_->contrast);
        const auto fraction = [&colours](float value) { return colours.fraction(value); };
        const float cell = static_cast<float>(slice->cellM) * scale;
        for (std::uint32_t r = 0; r < slice->height; ++r) {
            for (std::uint32_t col = 0; col < slice->width; ++col) {
                const float value =
                    slice->values[(static_cast<std::size_t>(r) * slice->width + col) *
                                  slice->components];
                ImVec4 color = viridis(fraction(value));
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
        viewBox(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), text, c::t1);

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
        sliceLegend(draw, origin, size, label, unit, view_->field, *slice, nullptr,
                    view_->contrast);
        ImGui::End();
    }

private:
    std::shared_ptr<EnvironmentView> view_;
};

// --- the field in the 3D view --------------------------------------------------------------

/// A point of a slice's plane (its two axes u, v) in the region frame.
std::array<double, 3> slicePoint(const FieldSliceData& slice, double u, double v) {
    switch (slice.orientation) {
        case SliceOrientation::VerticalXZ:
            return {u, slice.heightM, v};
        case SliceOrientation::VerticalYZ:
            return {slice.heightM, u, v};
        case SliceOrientation::Horizontal:
            break;
    }
    return {u, v, slice.heightM};
}

/// "Viewport · fields" (design 07): the scene as the Scene workspace draws it, with the chosen
/// field's slice laid in it where it is, the EMI sources marked and named, a probe under the
/// cursor and the legend. Dragging orbits, the right button pans, the wheel zooms.
class FieldViewportPanel final : public Panel {
public:
    explicit FieldViewportPanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Viewport · fields"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open =
            ImGui::Begin("Viewport · fields###FieldViewport", nullptr,
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        view_->follow(context);
        sliceTools(*view_, "##viewport-slice-tools");
        if (!context.scene) {
            ImGui::SetCursorPos(
                ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context)) {
                widgets::Caption("Waiting for the scene ...");
            }
            ImGui::End();
            return;
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        context.requestedViewportWidth =
            static_cast<std::uint32_t>(std::max(available.x * scale.x, 1.0F));
        context.requestedViewportHeight =
            static_cast<std::uint32_t>(std::max(available.y * scale.y, 1.0F));
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        if (context.viewportTexture != 0) {
            ImGui::Image(static_cast<ImTextureID>(context.viewportTexture), available);
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::InvisibleButton(
            "##fields", available,
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const bool hovered = ImGui::IsItemHovered();
        camera(context, available);

        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(origin, ImVec2(origin.x + available.x, origin.y + available.y), true);
        const double aspect = static_cast<double>(available.x) / std::max(available.y, 1.0F);
        const Mat4 matrix = context.camera.viewProjection(aspect);
        const std::array<double, 3> eye = context.camera.eye();
        const auto project = [&](const std::array<double, 3>& p) -> std::optional<ImVec2> {
            const auto x = static_cast<float>(p[0] - eye[0]);
            const auto y = static_cast<float>(p[1] - eye[1]);
            const auto z = static_cast<float>(p[2] - eye[2]);
            const float w = matrix[12] * x + matrix[13] * y + matrix[14] * z;
            if (w <= 0.01F) {
                return std::nullopt;  // behind the camera
            }
            const float nx = (matrix[0] * x + matrix[1] * y + matrix[2] * z) / w;
            const float ny = (matrix[4] * x + matrix[5] * y + matrix[6] * z) / w;
            return ImVec2(origin.x + (nx + 1.0F) * 0.5F * available.x,
                          origin.y + (ny + 1.0F) * 0.5F * available.y);
        };

        const EnvironmentInfo::Field* field = shownField(context, *view_);
        const char* unit = field != nullptr ? field->unit.c_str() : "";
        const char* label = field != nullptr ? field->label.c_str() : view_->field.c_str();
        const std::shared_ptr<const FieldSliceData> slice = view_->shown();
        if (slice && slice->width > 0 && slice->height > 0) {
            drawSlice(draw, *slice, project);
        }
        sources(context, draw, project);

        // Top left: what the view shows.
        char text[200];
        if (view_->field.empty()) {
            std::snprintf(text, sizeof(text), "Fields  ·  no field shown");
        } else if (!slice) {
            std::snprintf(text, sizeof(text), "Fields  ·  %s  ·  waiting for the first slice",
                          label);
        } else {
            std::snprintf(text, sizeof(text), "Fields  ·  %s  ·  %.2f m cells%s", label,
                          slice->cellM, context.renderOptions->airflow ? "  ·  airflow shown" : "");
        }
        viewBox(draw, ImVec2(origin.x + px(12.0F), origin.y + px(12.0F)), text, c::t1);

        if (hovered && slice) {
            probe(context, *slice, origin, available, label, unit, field);
        }
        if (slice) {
            char wind[64];
            std::snprintf(wind, sizeof(wind), "airflow · particles move with the air");
            sliceLegend(draw, origin, available, label, unit, view_->field, *slice,
                        context.renderOptions->airflow ? wind : nullptr, view_->contrast);
        }
        draw->PopClipRect();
        if (!emptyState(context) && view_->field.empty()) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + px(12.0F), origin.y + px(46.0F)));
            widgets::Caption("No field of this run can be shown. The Overlays panel says why.");
        }
        ImGui::End();
    }

private:
    static constexpr double kOrbitRadiansPerPixel = 0.005;
    static constexpr double kZoomPerWheelStep = 0.9;
    static constexpr std::uint32_t kMostQuadsPerSide = 96;  // larger slices are merged cells

    static void camera(PanelContext& context, const ImVec2& size) {
        const ImGuiIO& io = ImGui::GetIO();
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            context.camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                                 io.MouseDelta.y * kOrbitRadiansPerPixel);
        }
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            context.camera.pan(-io.MouseDelta.x / std::max(size.y, 1.0F),
                               io.MouseDelta.y / std::max(size.y, 1.0F));
        }
        if (ImGui::IsItemHovered() && io.MouseWheel != 0.0F) {
            context.camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
        }
    }

    /// The slice where it is: a quad per cell (or per block of cells on a large slice), in the
    /// field colour map at the Overlays panel's opacity.
    template <typename Project>
    void drawSlice(ImDrawList* draw, const FieldSliceData& slice, const Project& project) const {
        const SliceScale colours(view_->field, slice, view_->contrast);
        const std::uint32_t step = std::max<std::uint32_t>(
            1, (std::max(slice.width, slice.height) + kMostQuadsPerSide - 1) / kMostQuadsPerSide);
        const double half = slice.cellM * 0.5;
        for (std::uint32_t r = 0; r < slice.height; r += step) {
            for (std::uint32_t col = 0; col < slice.width; col += step) {
                const float value = slice.values[(static_cast<std::size_t>(r) * slice.width + col) *
                                                 slice.components];
                if (std::isnan(value)) {
                    continue;
                }
                const std::uint32_t lastCol = std::min(col + step, slice.width) - 1;
                const std::uint32_t lastRow = std::min(r + step, slice.height) - 1;
                const double u0 = slice.originX + slice.cellM * col - half;
                const double u1 = slice.originX + slice.cellM * lastCol + half;
                const double v0 = slice.originY + slice.cellM * r - half;
                const double v1 = slice.originY + slice.cellM * lastRow + half;
                const auto a = project(slicePoint(slice, u0, v0));
                const auto b = project(slicePoint(slice, u1, v0));
                const auto cc = project(slicePoint(slice, u1, v1));
                const auto d = project(slicePoint(slice, u0, v1));
                if (!a || !b || !cc || !d) {
                    continue;
                }
                ImVec4 colour = viridis(colours.fraction(value));
                colour.w = view_->opacity;
                draw->AddQuadFilled(*a, *b, *cc, *d, u32(colour));
            }
        }
        // The slice's edge.
        const double u0 = slice.originX - half;
        const double v0 = slice.originY - half;
        const double u1 = u0 + slice.cellM * slice.width;
        const double v1 = v0 + slice.cellM * slice.height;
        const auto a = project(slicePoint(slice, u0, v0));
        const auto b = project(slicePoint(slice, u1, v0));
        const auto cc = project(slicePoint(slice, u1, v1));
        const auto d = project(slicePoint(slice, u0, v1));
        if (a && b && cc && d) {
            draw->AddQuad(*a, *b, *cc, *d, u32(c::lineStrong));
        }
    }

    /// Design: the EMI sources in the scene, each a mark in the EMI colour with its name and
    /// numbers.
    template <typename Project>
    static void sources(const PanelContext& context, ImDrawList* draw, const Project& project) {
        const std::shared_ptr<const EnvironmentInfo> info = context.client.environment();
        if (!info || !context.scene) {
            return;
        }
        const ImVec4 emi = widgets::domainColor("emi");
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        for (const EnvironmentInfo::EmiSource& source : info->emiSources) {
            // The server may not say which prim a source is (prim none): its name is the
            // prim's instance name (emi::Source::name), so find it by that.
            std::uint32_t prim = source.prim;
            for (std::uint32_t i = 0; prim == kNone && i < prims.size(); ++i) {
                if (prims[i].name == source.name) {
                    prim = i;
                }
            }
            if (prim >= context.scene->transforms.size()) {
                continue;
            }
            const auto& p = context.scene->transforms[prim].position;
            const auto at = project({p[0], p[1], p[2]});
            if (!at) {
                continue;
            }
            char text[160];
            if (source.currentA != 0.0) {
                std::snprintf(text, sizeof(text), "%s  ·  %g A  ·  %g Hz", source.name.c_str(),
                              source.currentA, source.frequencyHz);
            } else {
                std::snprintf(text, sizeof(text), "%s  ·  %g Hz", source.name.c_str(),
                              source.frequencyHz);
            }
            draw->AddCircleFilled(*at, px(4.0F), u32(emi));
            draw->AddCircle(*at, px(8.0F), u32(emi), 0, px(1.5F));
            draw->AddText(fonts.sans, small, ImVec2(at->x + px(12.0F), at->y - small - px(2.0F)),
                          u32(emi), text);
        }
    }

    /// Design: a crosshair under the cursor where its ray meets the slice's plane, with a
    /// tooltip of the field there and the site's wind and air (what the server reports of them;
    /// the other fields are not streamed while one is shown).
    void probe(const PanelContext& context, const FieldSliceData& slice, const ImVec2& origin,
               const ImVec2& size, const char* label, const char* unit,
               const EnvironmentInfo::Field* field) const {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const double aspect = static_cast<double>(size.x) / std::max(size.y, 1.0F);
        const Ray ray = context.camera.ray((mouse.x - origin.x) / size.x * 2.0 - 1.0,
                                           (mouse.y - origin.y) / size.y * 2.0 - 1.0, aspect);
        const std::size_t axis = slice.orientation == SliceOrientation::VerticalXZ   ? 1
                                 : slice.orientation == SliceOrientation::VerticalYZ ? 0
                                                                                     : 2;
        if (std::abs(ray.direction.at(axis)) < 1e-9) {
            return;
        }
        const double t = (slice.heightM - ray.origin.at(axis)) / ray.direction.at(axis);
        if (t <= 0.0) {
            return;
        }
        std::array<double, 3> hit{};
        for (std::size_t k = 0; k < 3; ++k) {
            hit.at(k) = ray.origin.at(k) + ray.direction.at(k) * t;
        }
        const double u = axis == 0 ? hit[1] : hit[0];
        const double v = axis == 2 ? hit[1] : hit[2];
        const float value = slice.sample(u, v);
        if (std::isnan(value)) {
            return;
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddCircle(mouse, px(7.0F), u32(c::t1), 0, px(1.5F));
        draw->AddLine(ImVec2(mouse.x - px(11.0F), mouse.y), ImVec2(mouse.x + px(11.0F), mouse.y),
                      u32(c::t1));
        draw->AddLine(ImVec2(mouse.x, mouse.y - px(11.0F)), ImVec2(mouse.x, mouse.y + px(11.0F)),
                      u32(c::t1));
        ImGui::BeginTooltip();
        widgets::Mono(c::t3, "probe · x %.2f  y %.2f  z %.2f m", hit[0], hit[1], hit[2]);
        widgets::Dot(field != nullptr ? widgets::domainColor(field->domain) : c::t2);
        ImGui::SameLine(0.0F, px(6.0F));
        const std::shared_ptr<const EnvironmentInfo> info = context.client.environment();
        if (view_->field == kFluxField && info && info->emiSources.size() == 1) {
            // One source: its field is at its frequency (design: "|B| 29.4 µT · 60 Hz").
            widgets::Mono(c::t1, "%s  %.3g %s · %g Hz", label, static_cast<double>(value), unit,
                          info->emiSources.front().frequencyHz);
        } else {
            widgets::Mono(c::t1, "%s  %.3g %s", label, static_cast<double>(value), unit);
        }
        if (info) {
            if (view_->field != kAirField && info->windSpeed > 0.0) {
                widgets::Dot(widgets::domainColor("aerodynamics"));
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "wind %.1f m/s · from %.0f° (site mean)", info->windSpeed,
                              info->windFromDeg);
            }
            if (info->fogVisibilityM > 0.0 && info->fogVisibilityM < kClearAirM &&
                (field == nullptr || field->domain != "weather")) {
                widgets::Dot(widgets::domainColor("weather"));
                ImGui::SameLine(0.0F, px(6.0F));
                // Koschmieder: visibility 3.912 / sigma at a 2 % contrast threshold.
                widgets::Mono(c::t1, "σ_fog %.3g /m", kKoschmieder / info->fogVisibilityM);
            }
            if (info->ambientTemperatureK > 0.0) {
                widgets::Dot(widgets::domainColor("thermal"));
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "air %.1f °C (site)", info->ambientTemperatureK - 273.15);
            }
        }
        ImGui::EndTooltip();
    }

    // Koschmieder's constant, -ln(0.02): visibility at a 2 % contrast threshold (WMO).
    static constexpr double kKoschmieder = 3.912;
    static constexpr double kClearAirM = 10'000.0;  // the Environment panel's clear air

    std::shared_ptr<EnvironmentView> view_;
};

// --- what the field shown is ----------------------------------------------------------------

/// "Field info" (design 07, beside Overlays): the field shown and its last slice, how much
/// arrives, and every field of the run with why it can or cannot be shown.
class FieldInfoPanel final : public Panel {
public:
    explicit FieldInfoPanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Field info"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin("Field info###FieldInfo")) {
            ImGui::End();
            return;
        }
        if (emptyState(context)) {
            ImGui::End();
            return;
        }
        view_->follow(context);
        const EnvironmentInfo& info = *context.client.environment();
        const EnvironmentInfo::Field* field = shownField(context, *view_);
        const std::shared_ptr<const FieldSliceData> slice =
            view_->stream ? view_->stream->latest() : nullptr;
        char text[96];
        if (widgets::SectionHeader("Shown", field != nullptr ? field->label.c_str() : "none")) {
            ImGui::Indent(px(8.0F));
            if (field == nullptr) {
                widgets::Caption("No field shown: pick one in Overlays.");
            } else {
                widgets::PropertyRow("name", field->name.c_str(), nullptr);
                widgets::PropertyRow(
                    "domain", std::string(widgets::domainLabel(field->domain)).c_str(), nullptr);
                widgets::PropertyRow("unit", field->unit.c_str(), nullptr);
                std::snprintf(text, sizeof(text), "%u", field->components);
                widgets::PropertyRow("components", text, nullptr);
            }
            ImGui::Unindent(px(8.0F));
        }
        if (field != nullptr && widgets::SectionHeader("Last slice", slice ? nullptr : "waiting")) {
            ImGui::Indent(px(8.0F));
            if (!slice) {
                widgets::Caption("Waiting for the first slice ...");
            } else {
                const char* plane = slice->orientation == SliceOrientation::VerticalXZ   ? "XZ"
                                    : slice->orientation == SliceOrientation::VerticalYZ ? "YZ"
                                                                                         : "XY";
                const char* across = slice->orientation == SliceOrientation::VerticalXZ   ? "y"
                                     : slice->orientation == SliceOrientation::VerticalYZ ? "x"
                                                                                          : "z";
                std::snprintf(text, sizeof(text), "%s at %s = %.2f", plane, across, slice->heightM);
                widgets::PropertyRow("plane", text, "m");
                std::snprintf(text, sizeof(text), "%u × %u", slice->width, slice->height);
                widgets::PropertyRow("grid", text, "cells");
                row("cell", "m", "%.3f", slice->cellM);
                std::snprintf(text, sizeof(text), "%.2f, %.2f", slice->originX, slice->originY);
                widgets::PropertyRow("first cell", text, "m");
                std::snprintf(text, sizeof(text), "%.1f × %.1f", slice->cellM * slice->width,
                              slice->cellM * slice->height);
                widgets::PropertyRow("bounds", text, "m");
                std::snprintf(text, sizeof(text), "%.4g … %.4g",
                              static_cast<double>(slice->valueMin),
                              static_cast<double>(slice->valueMax));
                widgets::PropertyRow("range", text, field->unit.c_str());
                const double kib =
                    static_cast<double>(slice->values.size() * sizeof(float)) / 1024.0;
                row("size", "KiB", "%.1f", kib);
                std::snprintf(text, sizeof(text), "%llu",
                              static_cast<unsigned long long>(view_->stream->slices()));
                widgets::PropertyRow("slices received", text, nullptr);
                row("sim time", "s", "%.2f", static_cast<double>(slice->simTimeNs) * 1e-9);
                const double age =
                    static_cast<double>(context.status.simTimeNs - slice->simTimeNs) * 1e-9;
                row("age", "s", "%.2f", std::max(age, 0.0));
                widgets::PropertyRow("colour scale",
                                     SliceScale(view_->field, *slice).logScale
                                         ? "viridis · log, top 4 decades"
                                         : "viridis · linear",
                                     nullptr);
            }
            ImGui::Unindent(px(8.0F));
        }
        std::snprintf(text, sizeof(text), "%zu", info.fields.size());
        if (widgets::SectionHeader("Fields of this run", text)) {
            ImGui::Indent(px(8.0F));
            for (const EnvironmentInfo::Field& f : info.fields) {
                widgets::Dot(widgets::domainColor(f.domain), f.available);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(f.available ? c::t1 : c::t3, "%s", f.name.c_str());
                if (!f.available) {
                    ImGui::PushTextWrapPos(0.0F);
                    widgets::Caption("    %s", f.unavailableReason.c_str());
                    ImGui::PopTextWrapPos();
                }
            }
            ImGui::Unindent(px(8.0F));
        }
        if (info.synthetic) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "This server has no environment model: a field shown is an estimate it derives "
                "from the recording.");
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }

private:
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
            // Cut with an ellipsis when the panel is narrow; whole in a tooltip (QA3 F6).
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::PushStyleColor(ImGuiCol_Text, c::t3);
            fit::textRight(note);
            ImGui::PopStyleColor();
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
            // Cut with an ellipsis when the panel is narrow; whole in a tooltip (QA3 F6).
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::PushStyleColor(ImGuiCol_Text, c::t3);
            fit::textRight(note);
            ImGui::PopStyleColor();
            ImGui::PopFont();
            ImGui::PopID();
        }
        // The spray off water (downwash.md §5b): its drops, drawn where the physics has them.
        const auto spray = std::ranges::find_if(
            info.fields, [](const EnvironmentInfo::Field& f) { return f.name == kSprayField; });
        if (spray != info.fields.end()) {
            ImGui::PushID("spray");
            bool& on = context.renderOptions->spray;
            ImGui::BeginDisabled(!spray->available);
            if (!spray->available) {
                on = false;
            }
            widgets::Checkbox("##show", &on);
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Swatch(widgets::domainColor(spray->domain));
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushStyleColor(ImGuiCol_Text, spray->available ? c::t1 : c::t3);
            ImGui::TextUnformatted("spray · drops");
            ImGui::PopStyleColor();
            const char* note = spray->available ? "kg/m³" : spray->unavailableReason.c_str();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::PushStyleColor(ImGuiCol_Text, c::t3);
            fit::textRight(note);
            ImGui::PopStyleColor();
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
            // Contrast: sub-millimetre ripples beside a dimple of centimetres (water/surface).
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted("contrast");
            ImGui::PopStyleColor();
            ImGui::SameLine(px(8.0F) + px(112.0F));
            static constexpr std::array<const char*, 3> kContrasts{"linear", "symlog", "high-pass"};
            int contrast = static_cast<int>(view_->contrast);
            ImGui::BeginDisabled(slice->components != 1U);
            if (widgets::Segmented("##contrast", kContrasts.data(),
                                   static_cast<int>(kContrasts.size()), &contrast)) {
                view_->contrast = static_cast<EnvironmentView::Contrast>(contrast);
            }
            ImGui::EndDisabled();
            widgets::PropertyRow("colormap", "viridis", nullptr);
            ImGui::Unindent(px(8.0F));
        }
        coupledInto(context);
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
    /// One downstream effect: what it is, and the signals its value is made from.
    struct Effect {
        enum class Kind { Compass, Can, Radio };
        Kind kind{Kind::Compass};
        std::string label;
        std::shared_ptr<ScopeBuffer> a, b;
    };

    /// "Coupled into" (design 07-environment, Overlays): the downstream effects of the fields
    /// now: each magnetometer's heading error and its share of the Earth's field (sensor/<n>/
    /// heading_error, interference; RFC 0013), coloured by ArduPilot's compassmot bands (under
    /// 30 % fine, over 60 % move the compass), each CAN bus's bit-error rate and each radio
    /// link's SNR and packet loss.
    void coupledInto(PanelContext& context) {
        const auto catalog = context.client.catalog();
        if (catalog != effectsCatalog_) {
            effectsCatalog_ = catalog;
            for (const Effect& e : effects_) {
                for (const auto& buffer : {e.a, e.b}) {
                    if (buffer) {
                        context.client.unsubscribe(buffer);
                    }
                }
            }
            effects_.clear();
            const auto middle = [](std::string_view name, std::string_view prefix,
                                   std::string_view suffix) -> std::string {
                if (name.starts_with(prefix) && name.ends_with(suffix) &&
                    name.size() > prefix.size() + suffix.size()) {
                    return std::string(
                        name.substr(prefix.size(), name.size() - prefix.size() - suffix.size()));
                }
                return {};
            };
            for (const SignalInfo& info : catalog ? catalog->signals : std::vector<SignalInfo>{}) {
                if (const std::string n = middle(info.name, "sensor/", "/heading_error");
                    !n.empty()) {
                    effects_.push_back({Effect::Kind::Compass, n,
                                        context.client.subscribe(info.name),
                                        context.client.subscribe("sensor/" + n + "/interference")});
                } else if (const std::string c = middle(info.name, "emi/", "/bit_error_rate");
                           !c.empty()) {
                    effects_.push_back(
                        {Effect::Kind::Can, c, context.client.subscribe(info.name), nullptr});
                } else if (const std::string l = middle(info.name, "link/", "/snr"); !l.empty()) {
                    effects_.push_back(
                        {Effect::Kind::Radio, l, context.client.subscribe(info.name),
                         context.client.subscribe("link/" + l + "/packet_error_rate")});
                }
            }
        }
        if (effects_.empty() || !widgets::SectionHeader("Coupled into", nullptr)) {
            return;
        }
        ImGui::Indent(px(8.0F));
        const theme::Fonts& fonts = theme::fonts();
        for (const Effect& e : effects_) {
            const double a = e.a ? e.a->latest() : NAN;
            const double b = e.b ? e.b->latest() : NAN;
            char value[64];
            ImVec4 colour = c::t1;
            switch (e.kind) {
                case Effect::Kind::Compass: {
                    constexpr double kDegPerRad = 180.0 / std::numbers::pi;
                    std::snprintf(value, sizeof(value), "%+.1f° heading · %.0f %%", a * kDegPerRad,
                                  100.0 * b);
                    colour = b > 0.6 ? c::error : b > 0.3 ? c::warn : c::t1;
                    break;
                }
                case Effect::Kind::Can:
                    std::snprintf(value, sizeof(value), "BER %.1e", a);
                    break;
                case Effect::Kind::Radio:
                    std::snprintf(value, sizeof(value), "SNR %.0f dB · %.1f %% loss",
                                  a > 0.0 ? 10.0 * std::log10(a) : -INFINITY, 100.0 * b);
                    break;
            }
            if (std::isnan(a)) {
                std::snprintf(value, sizeof(value), "—");
                colour = c::t3;
            }
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted(e.label.c_str());
            ImGui::PopStyleColor();
            const float mono = px(theme::kMonoSize);
            const float width = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, value).x;
            ImGui::SameLine(ImGui::GetContentRegionMax().x - width - px(8.0F));
            ImGui::PushFont(fonts.mono, theme::kMonoSize);
            ImGui::TextColored(colour, "%s", value);
            ImGui::PopFont();
        }
        ImGui::Unindent(px(8.0F));
    }

    std::shared_ptr<EnvironmentView> view_;
    std::shared_ptr<const SignalCatalog> effectsCatalog_;
    std::vector<Effect> effects_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createEnvironmentPanels() {
    auto view = std::make_shared<EnvironmentView>();
    std::vector<std::unique_ptr<Panel>> panels;
    auto sent = std::make_shared<SentSite>();
    panels.push_back(std::make_unique<EnvironmentPanel>(sent));
    panels.push_back(std::make_unique<PresetsPanel>(sent));
    panels.push_back(std::make_unique<FieldViewportPanel>(view));
    panels.push_back(std::make_unique<FieldSlicePanel>(view));
    panels.push_back(std::make_unique<OverlaysPanel>(view));
    panels.push_back(std::make_unique<FieldInfoPanel>(view));
    return panels;
}

}  // namespace lagrange::viewer
