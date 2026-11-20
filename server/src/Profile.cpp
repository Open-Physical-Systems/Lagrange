#include "lagrange/sensors/Faults.hpp"
#include "lagrange/server/Profile.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonschema/jsonschema.hpp>
#include <limits>
#include <numbers>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace lagrange::server {

namespace {

/// Why a domain the profile enables cannot run here, and what to do: most domains have a module
/// in this server but nothing to run on in this robot and world (QA3 F5: a power profile on a
/// robot without its overlay).
std::string whyNotRunnable(const std::string& domain) {
    static const std::map<std::string, std::string, std::less<>> kNeeds{
        {"electrical",
         "the robot has no power system (no motors or battery: ElectricalMotorAPI_1, "
         "BatteryAPI_1); give its power-system overlay (--overlay FILE, a run file's overlay:, "
         "Python Simulation(..., overlay=FILE)) or a robot file with one, or disable electrical "
         "and name a substitute (ideal)"},
        {"thermal",
         "the robot has no thermal nodes or materials (ThermalAPI_1, ThermalMaterialAPI_1, or "
         "an overlay's thermal entries); give them or disable thermal and name a substitute "
         "(ambient)"},
        {"sensors", "the robot has no sensors; disable sensors and name a substitute"},
        {"rotor_aero",
         "the robot has no rotors (RotorAPI_1, an overlay's rotors:); disable rotor_aero and "
         "name a substitute"},
        {"aerodynamics",
         "nothing in the stage has drag (AeroAPI_1) or rotors; disable aerodynamics and name a "
         "substitute (zero)"},
        {"emi",
         "the stage has no EMI sources or victims (EmiSourceAPI_1, EmiSusceptibilityAPI_1, "
         "--emi-motor-leads); disable emi and name a substitute (zero)"},
        {"comms",
         "the robot has no radio links or CAN buses (RadioLinkAPI_1, CanBusAPI_1); disable "
         "comms and name a substitute (ideal)"},
        {"vehicle_dynamics",
         "the robot is not a road vehicle (VehicleAPI_1); disable vehicle_dynamics and name a "
         "substitute"},
        {"traffic",
         "the world has no traffic (a world with RoadLaneAPI_1 lanes, --traffic); disable "
         "traffic and name a substitute"},
        {"humans",
         "the stage has no people (HumanAPI_1, --humans); disable humans and name a substitute"},
        {"hydrodynamics",
         "the stage has no water (WaterAPI_1, --water); disable hydrodynamics and name a "
         "substitute"},
        {"tether",
         "the stage has no tethers (TetherAPI_1, --tether); disable tether and name a "
         "substitute (zero)"},
        {"fire", "the stage has no fires (FireAPI_1); disable fire and name a substitute (zero)"},
        {"lights",
         "the stage has no lamps (LampAPI_1); disable lights and name a substitute (zero)"},
        {"actuators",
         "the stage has no grippers, nozzles or servos (SuctionGripperAPI_1, "
         "MagnetGripperAPI_1, NozzleAPI_1, ServoAPI_1); disable actuators and name a substitute "
         "(zero, constant or ideal)"},
    };
    if (const auto found = kNeeds.find(domain); found != kNeeds.end()) {
        return domain + " cannot run here: " + found->second;
    }
    return domain + " has no module in this server yet; disable it and name a substitute";
}

namespace jsonschema = jsoncons::jsonschema;

constexpr double kNsPerS = 1e9;
constexpr std::int64_t kMinPhysicsDtNs = 100'000;     // 0.1 ms, as DomainState advertises
constexpr std::int64_t kMaxPhysicsDtNs = 10'000'000;  // 10 ms
// The electrical network's step range (domains/electrical.md §4), as DomainState advertises.
constexpr std::int64_t kMinElectricalDtNs = 10'000;     // 10 µs
constexpr std::int64_t kMaxElectricalDtNs = 1'000'000;  // 1 ms
// The thermal network's step range (domains/thermal.md §4).
constexpr std::int64_t kMinThermalDtNs = 1'000'000;       // 1 ms
constexpr std::int64_t kMaxThermalDtNs = 10'000'000'000;  // 10 s

/// A YAML document as JSON, for the schema. Scalars keep YAML 1.2 core meanings: true/false,
/// null and ~, integers, floats; anything else is a string.
jsoncons::json toJson(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Map: {
            jsoncons::json out(jsoncons::json_object_arg);
            for (const auto& item : node) {
                out.insert_or_assign(item.first.as<std::string>(), toJson(item.second));
            }
            return out;
        }
        case YAML::NodeType::Sequence: {
            jsoncons::json out(jsoncons::json_array_arg);
            for (const auto& item : node) {
                out.push_back(toJson(item));
            }
            return out;
        }
        case YAML::NodeType::Scalar: {
            const std::string text = node.Scalar();
            if (node.Tag() == "!") {  // quoted: always a string
                return jsoncons::json(text);
            }
            if (text == "true" || text == "false") {
                return jsoncons::json(text == "true");
            }
            if (text == "null" || text == "~" || text.empty()) {
                return jsoncons::json::null();
            }
            std::size_t used = 0;
            try {
                const long long integer = std::stoll(text, &used);
                if (used == text.size()) {
                    return jsoncons::json(static_cast<std::int64_t>(integer));
                }
                const double real = std::stod(text, &used);
                if (used == text.size()) {
                    return jsoncons::json(real);
                }
            } catch (const std::exception&) {
            }
            return jsoncons::json(text);
        }
        case YAML::NodeType::Null:
        case YAML::NodeType::Undefined:
            break;
    }
    return jsoncons::json::null();
}

void error(ProfileCheck& check, std::string pointer, std::string message) {
    check.issues.push_back({true, std::move(pointer), std::move(message)});
}

void warning(ProfileCheck& check, std::string pointer, std::string message) {
    check.issues.push_back({false, std::move(pointer), std::move(message)});
}

}  // namespace

ProfileCheck checkProfile(const std::string& yaml, const std::string& schemaJson,
                          const ProfileContext& context) {
    ProfileCheck check;
    jsoncons::json profile;
    try {
        const YAML::Node document = YAML::Load(yaml);
        if (!document.IsMap()) {
            error(check, "", "a profile must be a mapping with 'stage' and 'domains'");
            return check;
        }
        profile = toJson(document);
    } catch (const YAML::Exception& e) {
        error(check, "", "line " + std::to_string(e.mark.line + 1) + ": " + e.msg);
        return check;
    }

    if (!schemaJson.empty()) {
        try {
            const jsoncons::json schemaDocument = jsoncons::json::parse(schemaJson);
            const auto schema = jsonschema::make_json_schema(schemaDocument);
            // A domain this server runs that the installed schema does not know: the schema
            // (package lagrange_profiles) is older than the server, as in an overlay that rebuilt
            // lagrange_server alone. Say that, once per domain, instead of the schema's
            // "additional property"; the rest of the profile is still checked.
            jsoncons::json checked = profile;
            if (checked.contains("domains") && checked["domains"].is_object() &&
                schemaDocument.contains("properties") &&
                schemaDocument["properties"].contains("domains") &&
                schemaDocument["properties"]["domains"].contains("properties")) {
                const jsoncons::json& known = schemaDocument["properties"]["domains"]["properties"];
                std::vector<std::string> unknown;
                for (const auto& member : checked["domains"].object_range()) {
                    const std::string name(member.key());
                    if (!known.contains(name) && context.substitutes.contains(name)) {
                        unknown.push_back(name);
                    }
                }
                for (const std::string& name : unknown) {
                    error(check, "/domains/" + name,
                          "the profile schema (package lagrange_profiles) is older than this "
                          "server and does not know the domain '" +
                              name + "'; build and source lagrange_profiles with lagrange_server");
                    checked["domains"].erase(name);
                }
            }
            schema.validate(
                checked,
                [&check](const jsonschema::validation_message& message) -> jsonschema::walk_result {
                    std::string text = message.message();
                    if (text.size() > 200) {
                        text = text.substr(0, 197) + "...";
                    }
                    error(check, message.instance_location().string(), text);
                    return jsonschema::walk_result::advance;
                });
        } catch (const std::exception& e) {
            error(check, "", std::string("the profile schema could not be used: ") + e.what());
        }
    }

    ProfileSettings settings;
    if (profile.contains("stage") && profile["stage"].is_string() &&
        profile["stage"].as<std::string>() != context.stage) {
        error(check, "/stage",
              "this server runs " + context.stage +
                  "; to run another stage, start the server with it");
    }
    if (profile.contains("domains") && profile["domains"].is_object()) {
        for (const auto& member : profile["domains"].object_range()) {
            const std::string name(member.key());
            const jsoncons::json& domain = member.value();
            const std::string at = "/domains/" + name;
            if (!domain.is_object() || !domain.contains("enabled") ||
                !domain["enabled"].is_bool()) {
                continue;  // the schema reported it
            }
            const bool enabled = domain["enabled"].as<bool>();
            if (name == "electrical" && domain.contains("speed_prediction") &&
                domain["speed_prediction"].is_bool()) {
                settings.speedPrediction = domain["speed_prediction"].as<bool>();
            }
            if (name == "thermal" && domain.contains("ambient_temperature") &&
                domain["ambient_temperature"].is_number()) {
                settings.ambientTemperatureK = domain["ambient_temperature"].as<double>();
            }
            if (name == "aerodynamics") {
                for (const char* key : {"air_density", "wind_speed", "wind_from",
                                        "wind_reference_height", "roughness_length"}) {
                    if (domain.contains(key) && domain[key].is_number()) {
                        settings.aerodynamics[key] = domain[key].as<double>();
                    }
                }
                if (domain.contains("turbulence") && domain["turbulence"].is_string()) {
                    settings.aerodynamics["turbulence"] = domain["turbulence"].as<std::string>();
                }
                for (const char* key : {"downwash", "downwash_water", "downwash_spray"}) {
                    if (domain.contains(key) && domain[key].is_bool()) {
                        settings.aerodynamics[key] = domain[key].as<bool>();
                    }
                }
                if (domain.contains("downwash_model") && domain["downwash_model"].is_string()) {
                    settings.aerodynamics["downwash_model"] =
                        domain["downwash_model"].as<std::string>();
                }
            }
            if (name == "rotor_aero" && domain.contains("air_density") &&
                domain["air_density"].is_number()) {
                settings.airDensityKgM3 = domain["air_density"].as<double>();
            }
            if (name == "weather") {
                for (const char* key :
                     {"temperature", "pressure", "relative_humidity", "visibility", "rain_rate",
                      "snow_rate", "snow_cover", "dust_concentration", "dust_median_diameter",
                      "solar_irradiance", "sky_temperature", "ground_dust_diameter",
                      "ground_dust_concentration"}) {
                    if (domain.contains(key) && domain[key].is_number()) {
                        settings.weather[key] = domain[key].as<double>();
                    }
                }
                for (const char* key : {"fog_field", "wind_field"}) {
                    if (domain.contains(key) && domain[key].is_string()) {
                        settings.weather[key] = domain[key].as<std::string>();
                    }
                }
                if (domain.contains("effects") && domain["effects"].is_array()) {
                    std::vector<std::string> effects;
                    for (const auto& e : domain["effects"].array_range()) {
                        effects.push_back(e.as<std::string>());
                    }
                    settings.weather["effects"] = effects;
                }
                if (enabled && context.modules.contains("weather")) {
                    for (const char* key : {"temperature", "pressure", "relative_humidity"}) {
                        const bool temperature =
                            std::string_view(key) == "temperature" &&
                            (settings.ambientTemperatureK || context.ambientKnown);
                        if (!settings.weather.contains(key) &&
                            !context.weatherKnown.contains(key) && !temperature) {
                            error(check, at + "/" + key,
                                  std::string("the weather needs the site's ") + key +
                                      "; it has no default");
                        }
                    }
                }
            }
            if (name == "vehicle_dynamics" && domain.contains("tire_model") &&
                domain["tire_model"].is_string()) {
                settings.tireModel = domain["tire_model"].as<std::string>();
            }
            if (name == "emi" && domain.contains("coupling") && domain["coupling"].is_array()) {
                std::vector<std::string> coupling;
                for (const auto& c : domain["coupling"].array_range()) {
                    coupling.push_back(c.as<std::string>());
                }
                settings.emiCoupling = coupling;
            }
            if (name == "thermal") {
                for (const char* key : {"cell", "max_cells", "surface_coefficient"}) {
                    if (domain.contains(key) && domain[key].is_number()) {
                        settings.thermal[key] = domain[key].as<double>();
                    }
                }
                if (domain.contains("fidelity") && domain["fidelity"].is_string()) {
                    const std::string fidelity = domain["fidelity"].as<std::string>();
                    if (fidelity != "lumped" && fidelity != "conduction") {
                        error(check, at + "/fidelity",
                              "the thermal fidelity is lumped or conduction");
                    } else {
                        settings.thermal["fidelity"] = fidelity;
                    }
                }
            }
            if (name == "rotor_aero") {
                for (const char* key : {"inflow_time_constant", "speed_of_sound"}) {
                    if (domain.contains(key) && domain[key].is_number()) {
                        settings.rotorAero[key] = domain[key].as<double>();
                    }
                }
                if (domain.contains("fidelity") && domain["fidelity"].is_string()) {
                    settings.rotorAero["fidelity"] = domain["fidelity"].as<std::string>();
                }
                if (domain.contains("ground_effect") && domain["ground_effect"].is_bool()) {
                    settings.rotorAero["ground_effect"] = domain["ground_effect"].as<bool>();
                }
            }
            if (name == "rotor_aero" && context.modules.contains("rotor_aero") &&
                !settings.airDensityKgM3 && !context.airDensityKnown) {
                error(check, at + "/air_density",
                      "the rotors need the site's air density (kg/m³); it has no default");
            }
            if (name == "thermal" && enabled && context.modules.contains("thermal") &&
                !settings.ambientTemperatureK && !context.ambientKnown) {
                error(check, at + "/ambient_temperature",
                      "the thermal domain needs the site's ambient temperature (K); it has no "
                      "default");
            }
            if (name == "rigid_body" && domain.contains("substeps") &&
                domain["substeps"].is_int64()) {
                settings.rigidBodySubsteps = domain["substeps"].as<std::int64_t>();
            }
            if (name == "rigid_body" && domain.contains("noslip_iterations") &&
                domain["noslip_iterations"].is_int64()) {
                settings.rigidBodyNoslip = domain["noslip_iterations"].as<std::int64_t>();
                const bool mujoco = !domain.contains("backend") ||
                                    (domain["backend"].is_string() &&
                                     domain["backend"].as<std::string>() == "mujoco");
                if (*settings.rigidBodyNoslip > 0 && !mujoco) {
                    warning(check, at + "/noslip_iterations",
                            "MuJoCo's noslip iterations; other backends ignore them");
                }
            }
            if (domain.contains("dt") && domain["dt"].is_number()) {
                const double ns = domain["dt"].as<double>() * kNsPerS;
                if (std::abs(ns - std::round(ns)) > 1e-6) {
                    error(check, at + "/dt", "dt must be a whole number of nanoseconds");
                } else if (name == "rigid_body") {
                    const auto step = static_cast<std::int64_t>(std::llround(ns));
                    if (step < kMinPhysicsDtNs || step > kMaxPhysicsDtNs) {
                        error(check, at + "/dt", "the MuJoCo step must be from 0.0001 to 0.01 s");
                    } else {
                        settings.rigidBodyDtNs = step;
                    }
                } else if (name == "thermal") {
                    const auto step = static_cast<std::int64_t>(std::llround(ns));
                    if (step < kMinThermalDtNs || step > kMaxThermalDtNs) {
                        error(check, at + "/dt", "the thermal step must be from 0.001 to 10 s");
                    } else {
                        settings.thermalDtNs = step;
                    }
                } else if (name == "electrical") {
                    const auto step = static_cast<std::int64_t>(std::llround(ns));
                    if (step < kMinElectricalDtNs || step > kMaxElectricalDtNs) {
                        error(check, at + "/dt",
                              "the electrical step must be from 0.00001 to 0.001 s");
                    } else {
                        settings.electricalDtNs = step;
                    }
                }
            }
            if (enabled) {
                if (!context.modules.contains(name)) {
                    error(check, at + "/enabled", whyNotRunnable(name));
                }
                if (name == "rigid_body" && domain.contains("backend")) {
                    const std::string backend = domain["backend"].as<std::string>();
                    if (!context.rigidBodyBackends.contains(backend)) {
                        std::string choices;
                        for (const std::string& option : context.rigidBodyBackends) {
                            choices += (choices.empty() ? "" : ", ") + option;
                        }
                        error(check, at + "/backend",
                              "this server's rigid body backends are " + choices +
                                  " (load another with --plugin)");
                    } else {
                        settings.rigidBodyBackend = backend;
                    }
                }
                if (name == "electrical" && domain.contains("fidelity")) {
                    const std::string fidelity = domain["fidelity"].as<std::string>();
                    if (fidelity != "averaged" && fidelity != "fmu") {
                        error(check, at + "/fidelity",
                              "this server's electrical fidelities are averaged and fmu");
                    } else {
                        settings.electricalFidelity = fidelity;
                    }
                }
                if (name == "sensors" && domain.contains("only")) {
                    warning(check, at + "/only", "not applied yet: every sensor is simulated");
                }
                continue;
            }
            if (context.modules.contains(name)) {
                settings.substituted.insert(name);
            }
            if (name == "rigid_body") {
                // A replay names its recording on the domain or in the substitute object.
                const jsoncons::json* source = nullptr;
                if (domain.contains("source")) {
                    source = &domain["source"];
                } else if (domain.contains("substitute") && domain["substitute"].is_object() &&
                           domain["substitute"].contains("source")) {
                    source = &domain["substitute"]["source"];
                }
                if (source != nullptr && source->is_string()) {
                    const std::string file = source->as<std::string>();
                    if (!std::filesystem::exists(file)) {
                        error(check, at + "/source", "the recording " + file + " does not exist");
                    } else {
                        settings.rigidBodyReplay = file;
                    }
                }
            }
            if (domain.contains("substitute")) {
                const jsoncons::json& substitute = domain["substitute"];
                const std::string token =
                    substitute.is_string()
                        ? substitute.as<std::string>()
                        : (substitute.contains("kind") ? substitute["kind"].as<std::string>()
                                                       : std::string());
                settings.substituteTokens[name] = token;
                const auto allowed = context.substitutes.find(name);
                if (allowed != context.substitutes.end() && !allowed->second.contains(token)) {
                    std::string choices;
                    for (const std::string& option : allowed->second) {
                        choices += (choices.empty() ? "" : ", ") + option;
                    }
                    error(check, at + "/substitute",
                          "'" + token + "' cannot replace " + name + " here; choose " + choices);
                }
            }
        }
    }
    if (profile.contains("time") && profile["time"].is_object() &&
        profile["time"].contains("real_time_factor") &&
        profile["time"]["real_time_factor"].is_number()) {
        settings.realTimeFactor = profile["time"]["real_time_factor"].as<double>();
        if (profile["time"].contains("end")) {
            warning(check, "/time/end", "not applied yet: the run does not stop by itself");
        }
    }
    if (profile.contains("world") && profile["world"].is_object()) {
        const jsoncons::json& world = profile["world"];
        if (world.contains("geo_origin") && world["geo_origin"].is_object()) {
            const jsoncons::json& g = world["geo_origin"];
            if (g.contains("latitude") && g.contains("longitude") && g.contains("height")) {
                constexpr double kDegree = std::numbers::pi / 180.0;
                settings.geoOrigin = coords::Geodetic{g["latitude"].as<double>() * kDegree,
                                                      g["longitude"].as<double>() * kDegree,
                                                      g["height"].as<double>()};
            }
        }
        if (world.contains("streaming") && world["streaming"].is_object()) {
            const jsoncons::json& streaming = world["streaming"];
            if (streaming.contains("load_radius")) {
                settings.streamingLoadRadius = streaming["load_radius"].as<double>();
            }
            if (streaming.contains("unload_radius")) {
                settings.streamingUnloadRadius = streaming["unload_radius"].as<double>();
            }
            if (streaming.contains("collision_radius")) {
                settings.streamingCollisionRadius = streaming["collision_radius"].as<double>();
            }
            for (const auto& member : streaming.object_range()) {
                if (member.key() != "load_radius" && member.key() != "unload_radius" &&
                    member.key() != "collision_radius") {
                    warning(check, "/world/streaming/" + std::string(member.key()),
                            "not applied by this server yet");
                }
            }
        }
        for (const auto& member : world.object_range()) {
            if (member.key() != "geo_origin" && member.key() != "streaming") {
                warning(check, "/world/" + std::string(member.key()),
                        "not applied by this server yet");
            }
        }
    }
    if (profile.contains("plugins") && profile["plugins"].is_array()) {
        std::size_t index = 0;
        for (const jsoncons::json& entry : profile["plugins"].array_range()) {
            const std::string at = "/plugins/" + std::to_string(index++);
            ProfileSettings::Plugin plugin;
            plugin.library = entry["library"].as<std::string>();
            if (!context.pluginLibraries.contains(plugin.library)) {
                error(check, at + "/library",
                      plugin.library +
                          " is not loaded: plugins are loaded when the server starts "
                          "(--plugin, or `lagrange run` with this profile), never by a viewer");
            }
            if (entry.contains("config")) {
                for (const auto& member : entry["config"].object_range()) {
                    const jsoncons::json& value = member.value();
                    const std::string key(member.key());
                    if (value.is_bool()) {
                        plugin.config[key] = value.as<bool>();
                    } else if (value.is_int64() || value.is_uint64()) {
                        plugin.config[key] = value.as<std::int64_t>();
                    } else if (value.is_number()) {
                        plugin.config[key] = value.as<double>();
                    } else if (value.is_string()) {
                        plugin.config[key] = value.as<std::string>();
                    } else {
                        warning(check, at + "/config/" + key,
                                "only numbers, booleans and strings reach a plugin");
                    }
                }
            }
            settings.plugins.push_back(std::move(plugin));
        }
    }
    if (profile.contains("sensor_faults") && profile["sensor_faults"].is_array()) {
        // Checked as --sensor-fault checks them; whether the sensor and quantity exist is known
        // only once the run is wired, and a profile naming one that does not is refused then.
        std::vector<std::string> faults;
        std::size_t index = 0;
        for (const jsoncons::json& entry : profile["sensor_faults"].array_range()) {
            const std::string at = "/sensor_faults/" + std::to_string(index++);
            sensors::SensorFault fault;
            fault.sensor = entry["sensor"].as<std::string>();
            fault.quantity = entry.get_value_or<std::string>("quantity", "*");
            const std::string mode = entry["mode"].as<std::string>();
            for (const sensors::FaultMode m :
                 {sensors::FaultMode::Dropout, sensors::FaultMode::Stuck, sensors::FaultMode::Bias,
                  sensors::FaultMode::Drift, sensors::FaultMode::Noise,
                  sensors::FaultMode::Disconnect}) {
                if (sensors::to_string(m) == mode) {
                    fault.mode = m;
                }
            }
            fault.at = entry.get_value_or<double>("at", 0.0);
            if (entry.contains("for")) {
                fault.duration = entry["for"].as<double>();
            }
            fault.probability = entry.get_value_or<double>("probability", 1.0);
            fault.nan = entry.get_value_or<bool>("nan", false);
            fault.component = entry.get_value_or<int>("component", -1);
            // The size each mode needs: the schema requires it, so the default never applies.
            const double nan = std::numeric_limits<double>::quiet_NaN();
            fault.value = entry.get_value_or<double>("value", nan);
            fault.rate = entry.get_value_or<double>("rate", nan);
            fault.sigma = entry.get_value_or<double>("sigma", nan);
            if (const std::string problem = sensors::checkSensorFault(fault); !problem.empty()) {
                error(check, at, problem);
                continue;
            }
            // Written as text and read back: the same checks as the command line's.
            const std::string text = sensors::to_string(fault);
            if (auto parsed = sensors::parseSensorFault(text); !parsed) {
                error(check, at, parsed.error());
                continue;
            }
            faults.push_back(text);
        }
        settings.sensorFaults = std::move(faults);
    }
    for (const char* key : {"environment", "seed", "deterministic", "tier", "batch", "bridges",
                            "scenario", "record"}) {
        if (profile.contains(key)) {
            warning(check, std::string("/") + key,
                    std::string("not applied by this server yet") +
                        (std::string(key) == "record" ? "; start it with --record" : ""));
        }
    }
    check.valid = std::none_of(check.issues.begin(), check.issues.end(),
                               [](const ProfileIssue& issue) { return issue.error; });
    if (check.valid) {
        check.settings = std::move(settings);
    }
    return check;
}

}  // namespace lagrange::server
