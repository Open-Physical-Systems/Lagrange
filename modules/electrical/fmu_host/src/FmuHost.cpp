// FMI 3.0 co-simulation FMUs on the bus; rules on FmuComponent in FmuHost.hpp.
#include "lagrange/electrical/FmuHost.hpp"
#include "lagrange/services/Log.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fmilib.h>
#include <optional>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace lagrange::electrical {

namespace {

constexpr std::string_view kSource = "electrical/fmu";

/// One mapped variable.
struct Mapped {
    std::string variable;
    std::string signal;     // with the placeholders filled in
    std::string attribute;  // parameters
    Unit unit;
    std::optional<double> fallback;  // inputs: read while the signal is not written
    fmi3_value_reference_t reference{0};
    SignalId id;
};

/// FMI Library's messages go to the run log; the last error is kept for the status.
void logLibrary(jm_callbacks* callbacks, jm_string module, jm_log_level_enu_t level,
                jm_string message) {
    if (level <= jm_log_level_error && callbacks->context != nullptr) {
        *static_cast<std::string*>(callbacks->context) = std::string(module) + ": " + message;
    }
    if (level <= jm_log_level_warning) {
        services::log(
            level <= jm_log_level_error ? services::LogLevel::Error : services::LogLevel::Warning,
            kSource, std::string(module) + ": " + message);
    }
}

/// The FMU's own messages (fmi3LogMessageCallback).
void logFmu(fmi3_instance_environment_t, fmi3_status_t status, fmi3_string_t category,
            fmi3_string_t message) {
    services::log(status == fmi3_status_ok ? services::LogLevel::Info : services::LogLevel::Warning,
                  kSource,
                  std::string(category != nullptr ? category : "") + ": " +
                      (message != nullptr ? message : ""));
}

std::string fill(std::string text, const std::map<std::string, std::string>& values,
                 std::string& problem) {
    for (std::size_t open = text.find('{'); open != std::string::npos; open = text.find('{')) {
        const std::size_t close = text.find('}', open);
        if (close == std::string::npos) {
            problem = "unclosed { in " + text;
            return text;
        }
        const std::string key = text.substr(open + 1, close - open - 1);
        const auto found = values.find(key);
        if (found == values.end()) {
            problem = "no value for {" + key + "} in " + text;
            return text;
        }
        text.replace(open, close - open + 1, found->second);
    }
    return text;
}

}  // namespace

std::string fmuMapPath(const std::string& fmuPath) {
    return fmuPath + ".map.yaml";
}

struct FmuComponent::Impl {
    Status loadStatus;  // why load() failed; reported by declare
    YAML::Node map;
    jm_callbacks callbacks{};
    std::string lastError;
    fmi_import_context_t* context{nullptr};
    char* directory{nullptr};
    fmi3_import_t* fmu{nullptr};
    bool loaded{false};        // the binary is loaded
    bool instantiated{false};  // an instance exists
    bool deterministic{false};
    Duration step{};
    std::vector<Mapped> inputs, outputs, parameters;
    std::vector<fmi3_value_reference_t> inputRefs, outputRefs;
    std::vector<double> inputValues, outputValues;

    ~Impl() {
        if (instantiated) {
            fmi3_import_terminate(fmu);
            fmi3_import_free_instance(fmu);
        }
        if (loaded) {
            fmi3_import_destroy_dllfmu(fmu);
        }
        if (fmu != nullptr) {
            fmi3_import_free(fmu);
        }
        if (context != nullptr) {
            fmi_import_free_context(context);
        }
        if (directory != nullptr) {
            fmi_import_rmdir(&callbacks, directory);
            callbacks.free(directory);
        }
    }

    [[nodiscard]] Status error(const std::string& text) const {
        return Status::error(StatusCode::InvalidConfig,
                             lastError.empty() ? text : text + " (" + lastError + ")");
    }
};

FmuComponent::FmuComponent(FmuBinding binding)
    : binding_(std::move(binding)), moduleName_("fmu_" + binding_.name) {}

FmuComponent::~FmuComponent() = default;

ModuleInfo FmuComponent::info() const {
    return {"electrical", moduleName_, 0, 1, false, load().deterministic, CapabilityTier::Tier0};
}

Duration FmuComponent::preferredStep() const {
    const Impl& f = load();
    return f.step.ns > 0 ? f.step : Duration{100'000};
}

FmuComponent::Impl& FmuComponent::load() const {
    if (impl_) {
        return *impl_;
    }
    impl_ = std::make_unique<Impl>();
    Impl& f = *impl_;
    const std::string where = binding_.fmuPath;
    const auto fail = [&f](const std::string& text) -> Impl& {
        f.loadStatus = f.error(text);
        f.deterministic = false;
        return f;
    };
    try {
        f.map = YAML::LoadFile(fmuMapPath(binding_.fmuPath));
    } catch (const std::exception& e) {
        return fail(where + ": its map " + fmuMapPath(binding_.fmuPath) + ": " + e.what());
    }
    if (!f.map["fmi_version"] || f.map["fmi_version"].as<std::string>() != "3.0") {
        return fail(fmuMapPath(binding_.fmuPath) + ": fmi_version must be \"3.0\"");
    }
    const double step = f.map["step"] ? f.map["step"].as<double>(0.0) : 0.0;
    if (step <= 0.0) {
        return fail(fmuMapPath(binding_.fmuPath) + ": step must be a positive time in s");
    }
    f.step = Duration{static_cast<std::int64_t>(std::llround(step * 1e9))};

    // The FMU, unpacked and parsed, its binary loaded.
    f.callbacks = *jm_get_default_callbacks();
    f.callbacks.logger = logLibrary;
    f.callbacks.log_level = jm_log_level_warning;
    f.callbacks.context = &f.lastError;
    f.context = fmi_import_allocate_context(&f.callbacks);
    f.directory = fmi_import_mk_temp_dir(&f.callbacks, nullptr, "lagrange_fmu_");
    if (f.context == nullptr || f.directory == nullptr) {
        return fail(where + ": cannot make a directory to unpack it");
    }
    if (fmi_import_get_fmi_version(f.context, binding_.fmuPath.c_str(), f.directory) !=
        fmi_version_3_0_enu) {
        return fail(where + ": not an FMI 3.0 FMU");
    }
    f.fmu = fmi3_import_parse_xml(f.context, f.directory, nullptr);
    if (f.fmu == nullptr) {
        return fail(where + ": its modelDescription.xml cannot be read");
    }
    if ((static_cast<int>(fmi3_import_get_fmu_kind(f.fmu)) & fmi3_fmu_kind_cs) == 0) {
        return fail(where + ": not a co-simulation FMU (model exchange is not hosted)");
    }
    if (fmi3_import_create_dllfmu(f.fmu, fmi3_fmu_kind_cs, nullptr, logFmu) != jm_status_success) {
        return fail(where + ": its binary for this platform cannot be loaded");
    }
    f.loaded = true;
    f.deterministic = fmi3_import_get_capability(f.fmu, fmi3_cs_canGetAndSetFMUState) != 0 &&
                      fmi3_import_get_capability(f.fmu, fmi3_cs_canSerializeFMUState) != 0;
    return f;
}

Status FmuComponent::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) {
    Impl& f = load();
    if (!f.loadStatus.ok()) {
        return f.loadStatus;
    }
    f.inputs.clear();
    f.outputs.clear();
    f.parameters.clear();
    f.inputRefs.clear();
    f.outputRefs.clear();
    const std::string where = binding_.fmuPath;
    const YAML::Node& map = f.map;

    // The variables: Float64, the right causality, units of the map's dimension.
    std::string problems;
    const auto variable = [&](const YAML::Node& entry, fmi3_causality_enu_t causality,
                              const char* kind) -> std::optional<Mapped> {
        Mapped m;
        m.variable = entry["variable"].as<std::string>("");
        fmi3_import_variable_t* v = fmi3_import_get_variable_by_name(f.fmu, m.variable.c_str());
        if (v == nullptr) {
            problems += "\n  " + where + ": no variable " + m.variable;
            return std::nullopt;
        }
        if (fmi3_import_get_variable_base_type(v) != fmi3_base_type_float64 ||
            fmi3_import_get_variable_causality(v) != causality) {
            problems += "\n  " + where + ": " + m.variable + " is not a Float64 " + kind;
            return std::nullopt;
        }
        m.reference = fmi3_import_get_variable_vr(v);
        if (!entry["unit"]) {
            return m;
        }
        const UnitParse parsed = parseUnit(entry["unit"].as<std::string>());
        if (!parsed.unit) {
            problems += "\n  " + where + ": " + m.variable + ": " + parsed.error;
            return std::nullopt;
        }
        m.unit = *parsed.unit;
        // The FMU's unit from its BaseUnit exponents (kg m s A K mol cd rad), SI only.
        Dimension fmuDimension;
        double factor = 1.0;
        double offset = 0.0;
        if (fmi3_import_unit_t* unit =
                fmi3_import_get_float64_variable_unit(fmi3_import_get_variable_as_float64(v))) {
            const int* e = fmi3_import_get_SI_unit_exponents(unit);
            fmuDimension = {static_cast<std::int8_t>(e[1]), static_cast<std::int8_t>(e[0]),
                            static_cast<std::int8_t>(e[2]), static_cast<std::int8_t>(e[3]),
                            static_cast<std::int8_t>(e[4]), static_cast<std::int8_t>(e[5]),
                            static_cast<std::int8_t>(e[6]), static_cast<std::int8_t>(e[7])};
            factor = fmi3_import_get_SI_unit_factor(unit);
            offset = fmi3_import_get_SI_unit_offset(unit);
        }
        if (!(fmuDimension == m.unit.dim) || factor != m.unit.scale || offset != 0.0) {
            problems += "\n  " + where + ": " + m.variable + " is in " +
                        to_string(Unit{fmuDimension, factor}) + " but the map says " +
                        to_string(m.unit);
            return std::nullopt;
        }
        return m;
    };
    const auto signalOf = [&](const YAML::Node& entry, Mapped& m) {
        std::string problem;
        m.signal = fill(entry["signal"].as<std::string>(""), binding_.placeholders, problem);
        if (!problem.empty()) {
            problems += "\n  " + where + ": " + problem;
        }
    };
    for (const auto& entry : map["inputs"]) {
        if (auto m = variable(entry, fmi3_causality_enu_input, "input")) {
            signalOf(entry, *m);
            if (entry["optional"]) {
                m->fallback = entry["optional"].as<double>();
            }
            f.inputs.push_back(std::move(*m));
        }
    }
    for (const auto& entry : map["outputs"]) {
        if (auto m = variable(entry, fmi3_causality_enu_output, "output")) {
            signalOf(entry, *m);
            f.outputs.push_back(std::move(*m));
        }
    }
    for (const auto& entry : map["parameters"]) {
        if (auto m = variable(entry, fmi3_causality_enu_parameter, "parameter")) {
            m->attribute = entry["attribute"].as<std::string>("");
            f.parameters.push_back(std::move(*m));
        }
    }
    if (!problems.empty()) {
        return Status::error(StatusCode::InvalidConfig, "the FMU cannot be hosted:" + problems);
    }

    for (Mapped& m : f.inputs) {
        SignalSpec spec;
        spec.name = m.signal;
        spec.unit = m.unit;
        spec.optional = m.fallback.has_value();
        spec.defaultValue = m.fallback.value_or(0.0);
        // The FMU's inputs come from modules that read its outputs (the network reads the bus
        // current and writes the bus voltage): feedback breaks that loop.
        spec.feedback = true;
        m.id = d.input(spec);
        f.inputRefs.push_back(m.reference);
    }
    for (Mapped& m : f.outputs) {
        SignalSpec spec;
        spec.name = m.signal;
        spec.unit = m.unit;
        m.id = d.output(spec);
        f.outputRefs.push_back(m.reference);
    }
    f.inputValues.assign(f.inputs.size(), 0.0);
    f.outputValues.assign(f.outputs.size(), 0.0);
    return {};
}

Status FmuComponent::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    Impl& f = *impl_;
    if (f.instantiated) {
        fmi3_import_terminate(f.fmu);
        fmi3_import_free_instance(f.fmu);
        f.instantiated = false;
    }
    if (fmi3_import_instantiate_co_simulation(f.fmu, binding_.name.c_str(), nullptr, fmi3_false,
                                              fmi3_false, fmi3_false, fmi3_false, nullptr, 0,
                                              nullptr) != jm_status_success) {
        return f.error(binding_.fmuPath + ": fmi3InstantiateCoSimulation failed");
    }
    f.instantiated = true;
    // Parameters from the component's schema attributes; unauthored ones keep the FMU's start.
    for (const Mapped& m : f.parameters) {
        const auto value = binding_.attributes.find(m.attribute);
        if (value == binding_.attributes.end()) {
            continue;
        }
        if (fmi3_import_set_float64(f.fmu, &m.reference, 1, &value->second, 1) != fmi3_status_ok) {
            return f.error(binding_.fmuPath + ": setting " + m.variable + " failed");
        }
    }
    if (fmi3_import_enter_initialization_mode(f.fmu, fmi3_false, 0.0, 0.0, fmi3_false, 0.0) !=
            fmi3_status_ok ||
        fmi3_import_exit_initialization_mode(f.fmu) != fmi3_status_ok) {
        return f.error(binding_.fmuPath + ": initialisation failed");
    }
    if (!f.outputRefs.empty() &&
        fmi3_import_get_float64(f.fmu, f.outputRefs.data(), f.outputRefs.size(),
                                f.outputValues.data(), f.outputValues.size()) != fmi3_status_ok) {
        return f.error(binding_.fmuPath + ": reading its outputs failed");
    }
    for (std::size_t i = 0; i < f.outputs.size(); ++i) {
        bus.write(f.outputs[i].id, f.outputValues[i]);
    }
    return {};
}

Status FmuComponent::step(const StepContext& context, SignalBus& bus) {
    Impl& f = *impl_;
    for (std::size_t i = 0; i < f.inputs.size(); ++i) {
        f.inputValues[i] = bus.read(f.inputs[i].id);
    }
    if (!f.inputRefs.empty() &&
        fmi3_import_set_float64(f.fmu, f.inputRefs.data(), f.inputRefs.size(), f.inputValues.data(),
                                f.inputValues.size()) != fmi3_status_ok) {
        return Status::error(StatusCode::Internal, binding_.fmuPath + ": setting inputs failed");
    }
    fmi3_boolean_t event = fmi3_false;
    fmi3_boolean_t terminate = fmi3_false;
    fmi3_boolean_t early = fmi3_false;
    fmi3_float64_t reached = 0.0;
    const fmi3_status_t status =
        fmi3_import_do_step(f.fmu, context.t.seconds(), context.dt.seconds(), fmi3_true, &event,
                            &terminate, &early, &reached);
    if ((status != fmi3_status_ok && status != fmi3_status_warning) || terminate) {
        return Status::error(StatusCode::Diverged,
                             binding_.fmuPath + ": fmi3DoStep failed or asked to terminate");
    }
    if (!f.outputRefs.empty() &&
        fmi3_import_get_float64(f.fmu, f.outputRefs.data(), f.outputRefs.size(),
                                f.outputValues.data(), f.outputValues.size()) != fmi3_status_ok) {
        return Status::error(StatusCode::Internal, binding_.fmuPath + ": reading outputs failed");
    }
    for (std::size_t i = 0; i < f.outputs.size(); ++i) {
        bus.write(f.outputs[i].id, f.outputValues[i]);
    }
    return {};
}

void FmuComponent::shutdown() {
    if (impl_ && impl_->instantiated) {
        fmi3_import_terminate(impl_->fmu);
        fmi3_import_free_instance(impl_->fmu);
        impl_->instantiated = false;
    }
}

Status FmuComponent::saveState(StateWriter& writer) const {
    if (!impl_ || !impl_->deterministic || !impl_->instantiated) {
        return Status::error(StatusCode::Unsupported,
                             binding_.fmuPath + " cannot get and serialize its state");
    }
    fmi3_FMU_state_t state = nullptr;
    size_t size = 0;
    if (fmi3_import_get_fmu_state(impl_->fmu, &state) != fmi3_status_ok ||
        fmi3_import_serialized_fmu_state_size(impl_->fmu, state, &size) != fmi3_status_ok) {
        return Status::error(StatusCode::Internal, binding_.fmuPath + ": saving its state failed");
    }
    std::vector<fmi3_byte_t> bytes(size);
    const fmi3_status_t serialized =
        fmi3_import_serialize_fmu_state(impl_->fmu, state, bytes.data(), size);
    fmi3_import_free_fmu_state(impl_->fmu, &state);
    if (serialized != fmi3_status_ok) {
        return Status::error(StatusCode::Internal, binding_.fmuPath + ": saving its state failed");
    }
    writer.put(static_cast<std::uint64_t>(size));
    writer.putBytes(bytes.data(), size);
    return {};
}

Status FmuComponent::restoreState(StateReader& reader) {
    std::uint64_t size = 0;
    if (!impl_ || !impl_->instantiated || !reader.get(size)) {
        return Status::error(StatusCode::InvalidConfig, binding_.fmuPath + ": no state to restore");
    }
    std::vector<fmi3_byte_t> bytes(size);
    if (!reader.getBytes(bytes.data(), size)) {
        return Status::error(StatusCode::InvalidConfig, binding_.fmuPath + ": state truncated");
    }
    fmi3_FMU_state_t state = nullptr;
    if (fmi3_import_de_serialize_fmu_state(impl_->fmu, bytes.data(), size, &state) !=
            fmi3_status_ok ||
        fmi3_import_set_fmu_state(impl_->fmu, state) != fmi3_status_ok) {
        fmi3_import_free_fmu_state(impl_->fmu, &state);
        return Status::error(StatusCode::Internal, binding_.fmuPath + ": restoring failed");
    }
    fmi3_import_free_fmu_state(impl_->fmu, &state);
    return {};
}

std::vector<std::string> FmuComponent::inputNames() const {
    std::vector<std::string> out;
    if (impl_) {
        for (const Mapped& m : impl_->inputs) {
            out.push_back(m.signal);
        }
    }
    return out;
}

std::vector<std::string> FmuComponent::outputNames() const {
    std::vector<std::string> out;
    if (impl_) {
        for (const Mapped& m : impl_->outputs) {
            out.push_back(m.signal);
        }
    }
    return out;
}

}  // namespace lagrange::electrical
