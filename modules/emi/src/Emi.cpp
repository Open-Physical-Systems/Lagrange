// The EMI domain: sources, victims and the module (Emi.hpp, domains/emi.md).
#include "lagrange/emi/Emi.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::emi {

using namespace literals;

namespace {

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {},
                bool feedback = false) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    s.feedback = feedback;
    return s;
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 plus(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 minus(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 scaled(const Vec3& a, double k) {
    return {a[0] * k, a[1] * k, a[2] * k};
}
double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

/// The square root of 1 + Σh² (the RMS of the waveform per unit fundamental RMS) and of
/// 1 + Σ(k·h_k)² (the same for its derivative over ω).
double waveformRms(const std::vector<double>& harmonics, bool derivative) {
    double sum = 1.0;
    for (std::size_t i = 0; i < harmonics.size(); ++i) {
        const double k = derivative ? static_cast<double>(i + 2) : 1.0;
        sum += k * k * harmonics[i] * harmonics[i];
    }
    return std::sqrt(sum);
}

/// The spatial shape of a source's B at its RMS current (T), or, for a source with a field map,
/// the map scaled to that current.
Vec3 fieldShape(const Source& s, const SourceState& state, const Vec3& at) {
    const Pose& pose = state.pose;
    if (s.fieldMap) {
        if (!s.mapIsB) {
            return {};
        }
        const Vec3 local = rotate(conjugate(pose.orientation), minus(at, pose.position));
        const Vec3 b = rotate(pose.orientation, s.fieldMap->sampleVector(local));
        return scaled(b, s.current > 0.0 ? state.current / s.current : 0.0);
    }
    const Vec3 axis = rotate(pose.orientation, s.direction);
    switch (s.type) {
        case Source::Type::PowerLine:
            return lineField(pose.position, axis, state.current, at);
        case Source::Type::MotorDrive:
        case Source::Type::Welder:
            return dipoleField(pose.position, scaled(axis, s.momentPerAmpere * state.current), at);
        case Source::Type::Wire: {
            // Each segment, and the return path shifted by its offset carrying it back.
            const Vec3 back = rotate(pose.orientation, s.returnOffset);
            const bool paired = norm(s.returnOffset) > 0.0;
            Vec3 sum{0.0, 0.0, 0.0};
            for (std::size_t k = 0; k + 1 < s.path.size(); ++k) {
                const Vec3 a = plus(pose.position, rotate(pose.orientation, s.path[k]));
                const Vec3 b = plus(pose.position, rotate(pose.orientation, s.path[k + 1]));
                sum = plus(sum, segmentField(a, b, state.current, at));
                if (paired) {
                    sum = plus(sum, segmentField(plus(a, back), plus(b, back), -state.current, at));
                }
            }
            return sum;
        }
        case Source::Type::Transmitter:
        case Source::Type::Custom:
            return {};
    }
    return {};
}

std::optional<double> real(const SchemaInstance& s, const char* attr) {
    return s.real(attr);
}

}  // namespace

Vec3 lineField(const Vec3& point, const Vec3& direction, double current, const Vec3& at) {
    const Vec3 r = minus(at, point);
    const Vec3 radial = minus(r, scaled(direction, dot(r, direction)));
    const double d = std::max(norm(radial), kMinDistance);
    const double magnitude = kMu0 * current / (2.0 * std::numbers::pi * d);
    return scaled(cross(direction, radial), magnitude / std::max(norm(radial), 1e-300));
}

Vec3 dipoleField(const Vec3& centre, const Vec3& moment, const Vec3& at) {
    const Vec3 r = minus(at, centre);
    const double length = norm(r);
    const double d = std::max(length, kMinDistance);
    const Vec3 unit = length > 0.0 ? scaled(r, 1.0 / length) : Vec3{1.0, 0.0, 0.0};
    const double k = kMu0 / (4.0 * std::numbers::pi * d * d * d);
    const Vec3 along = scaled(unit, 3.0 * dot(moment, unit));
    return scaled(minus(along, moment), k);
}

Vec3 segmentField(const Vec3& a, const Vec3& b, double current, const Vec3& at) {
    const Vec3 ab = minus(b, a);
    const double length = norm(ab);
    if (length <= 0.0 || current == 0.0) {
        return {0.0, 0.0, 0.0};
    }
    const Vec3 along = scaled(ab, 1.0 / length);
    const Vec3 r = minus(at, a);
    const double z1 = -dot(r, along);  // the ends' positions along the line, seen from at
    const double z2 = length + z1;
    const Vec3 radial = minus(r, scaled(along, dot(r, along)));
    const double radialLength = norm(radial);
    const double d = std::max(radialLength, kMinDistance);
    const double sines = z2 / std::sqrt(z2 * z2 + d * d) - z1 / std::sqrt(z1 * z1 + d * d);
    const double magnitude = kMu0 * current / (4.0 * std::numbers::pi * d) * sines;
    if (radialLength <= 0.0) {
        return {0.0, 0.0, 0.0};  // on the line: no direction (and no victim sits in a wire)
    }
    return scaled(cross(along, radial), magnitude / radialLength);
}

double farField(double eirp, double distance) {
    return std::sqrt(30.0 * std::max(eirp, 0.0)) / std::max(distance, kMinDistance);
}

double glitchProbability(double field, double field50) {
    if (field <= 0.0 || field50 <= 0.0) {
        return 0.0;
    }
    return 1.0 / (1.0 + std::pow(field50 / field, 4));
}

double bitErrorRate(double field, double referenceField, double rateAtReference) {
    if (referenceField <= 0.0) {
        return 0.0;
    }
    const double x = field / referenceField;
    return std::min(0.5, rateAtReference * x * x);
}

double bandOverlap(double sourceFrequency, double sourceBandwidth, double receiverFrequency,
                   double receiverBandwidth) {
    const double rxLo = receiverFrequency - receiverBandwidth / 2.0;
    const double rxHi = receiverFrequency + receiverBandwidth / 2.0;
    if (sourceBandwidth <= 0.0) {
        return sourceFrequency >= rxLo && sourceFrequency <= rxHi ? 1.0 : 0.0;
    }
    const double lo = std::max(sourceFrequency - sourceBandwidth / 2.0, rxLo);
    const double hi = std::min(sourceFrequency + sourceBandwidth / 2.0, rxHi);
    return std::clamp((hi - lo) / sourceBandwidth, 0.0, 1.0);
}

double receivedPower(double field, double frequency) {
    constexpr double kSpeedOfLight = 299792458.0;  // m/s (exact, SI)
    if (frequency <= 0.0) {
        return 0.0;
    }
    const double lambda = kSpeedOfLight / frequency;
    return field * field / kFreeSpaceImpedance * lambda * lambda / (4.0 * std::numbers::pi);
}

Expected<std::vector<Source>, std::string> sourcesOf(const WorldView& world) {
    std::vector<Source> out;
    for (const SchemaInstance* inst : world.instancesOf("EmiSourceAPI_1")) {
        Source s;
        s.name = std::string(world.signalInstanceName(inst->prim));
        const std::string where = std::string(world.primPath(inst->prim)) + ": ";
        const std::string_view type = inst->token("emi:sourceType").value_or("");
        if (type == "power_line") {
            s.type = Source::Type::PowerLine;
        } else if (type == "motor_drive") {
            s.type = Source::Type::MotorDrive;
        } else if (type == "transmitter") {
            s.type = Source::Type::Transmitter;
        } else if (type == "welder") {
            s.type = Source::Type::Welder;
        } else if (type == "custom") {
            s.type = Source::Type::Custom;
        } else if (type == "wire") {
            s.type = Source::Type::Wire;
        } else {
            return Unexpected{where + "emi:sourceType is required"};
        }
        s.body = world.primBody(inst->prim);
        s.poseInBody = world.primPoseInBody(inst->prim);
        const auto frequency = real(*inst, "emi:frequency");
        s.frequency = frequency.value_or(0.0);
        s.current = real(*inst, "emi:current").value_or(0.0);
        s.radiatedPower = real(*inst, "emi:radiatedPower").value_or(0.0);
        s.bandwidth = real(*inst, "emi:bandwidth").value_or(0.0);
        s.dutyCycle = real(*inst, "emi:dutyCycle").value_or(1.0);
        s.momentPerAmpere = real(*inst, "emi:dipoleMomentPerAmpere").value_or(0.0);
        const std::span<const double> harmonics = inst->realArray("emi:harmonics");
        s.harmonics.assign(harmonics.begin(), harmonics.end());
        if (const std::span<const double> d = inst->realArray("emi:direction"); d.size() == 3) {
            const double n = norm({d[0], d[1], d[2]});
            if (n <= 0.0) {
                return Unexpected{where + "emi:direction must not be zero"};
            }
            s.direction = {d[0] / n, d[1] / n, d[2] / n};
        }
        if (s.dutyCycle < 0.0 || s.dutyCycle > 1.0) {
            return Unexpected{where + "emi:dutyCycle must be within 0 to 1"};
        }
        if (s.frequency < 0.0 || s.current < 0.0 || s.radiatedPower < 0.0) {
            return Unexpected{where +
                              "emi:frequency, emi:current and emi:radiatedPower must "
                              "not be negative"};
        }
        // Site rule: a power line's or welder's frequency has no default (50 or 60 Hz mains).
        if ((s.type == Source::Type::PowerLine || s.type == Source::Type::Welder) && !frequency) {
            return Unexpected{where +
                              "emi:frequency is required: the site's grid frequency "
                              "(50 or 60 Hz), never a default; 0 for direct current"};
        }
        if (s.type == Source::Type::Transmitter && (!frequency || s.radiatedPower <= 0.0)) {
            return Unexpected{where + "a transmitter needs emi:frequency and emi:radiatedPower"};
        }
        if (const auto map = inst->assetPath("emi:fieldMap")) {
            auto field = fields::Field::load(std::string(*map), world.raw().region.worldEnu);
            if (!field) {
                return Unexpected{where + field.error()};
            }
            if ((*field)->kind() != fields::Field::Kind::Vector) {
                return Unexpected{where + "emi:fieldMap must be a vector grid"};
            }
            s.fieldMap = *field;
            s.mapIsB = inst->token("emi:fieldMapQuantity").value_or("B") == "B";
            if (s.mapIsB && s.current <= 0.0) {
                return Unexpected{where +
                                  "a B field map needs emi:current, the current it was "
                                  "made at"};
            }
        }
        if (const std::span<const PrimId> driver = inst->rel("emi:drivenBy"); !driver.empty()) {
            s.drivenBy = std::string(world.signalInstanceName(driver[0]));
        }
        if (const auto signal = inst->token("emi:currentSignal"); signal && !signal->empty()) {
            s.currentSignal = std::string(*signal);
        }
        if (s.type == Source::Type::Wire) {
            const std::span<const double> path = inst->realArray("emi:path");
            if (path.size() < 6 || path.size() % 3 != 0) {
                return Unexpected{where + "a wire needs emi:path, x, y, z of two points or more"};
            }
            for (std::size_t k = 0; k < path.size(); k += 3) {
                s.path.push_back({path[k], path[k + 1], path[k + 2]});
            }
            if (const std::span<const double> o = inst->realArray("emi:returnOffset");
                o.size() == 3) {
                s.returnOffset = {o[0], o[1], o[2]};
            }
            if (s.currentSignal.empty() && s.current <= 0.0) {
                return Unexpected{where + "a wire needs emi:currentSignal or emi:current"};
            }
        }
        if (s.type == Source::Type::PowerLine && !s.fieldMap && s.current <= 0.0) {
            return Unexpected{where + "a power line needs emi:current"};
        }
        if ((s.type == Source::Type::MotorDrive || s.type == Source::Type::Welder) && !s.fieldMap &&
            s.momentPerAmpere <= 0.0) {
            return Unexpected{where + "a dipole source needs emi:dipoleMomentPerAmpere"};
        }
        out.push_back(std::move(s));
    }
    return out;
}

Vec3 magneticField(const Source& s, const SourceState& state, const Vec3& at, double t) {
    const Vec3 shape = fieldShape(s, state, at);
    double waveform = std::sqrt(s.dutyCycle);
    if (s.frequency > 0.0) {
        const double w = 2.0 * std::numbers::pi * s.frequency * t;
        double sum = std::sin(w);
        for (std::size_t i = 0; i < s.harmonics.size(); ++i) {
            sum += s.harmonics[i] * std::sin(static_cast<double>(i + 2) * w);
        }
        waveform *= std::numbers::sqrt2 * sum;
    }
    return scaled(shape, waveform);
}

FieldRms magneticFieldRms(const Source& s, const SourceState& state, const Vec3& at) {
    const double shape = norm(fieldShape(s, state, at)) * std::sqrt(s.dutyCycle);
    if (s.frequency <= 0.0) {
        return {shape, 0.0};
    }
    return {shape * waveformRms(s.harmonics, false),
            shape * s.frequency * waveformRms(s.harmonics, true)};
}

double electricFieldRms(const Source& s, const SourceState& state, const Vec3& at) {
    if (s.fieldMap && !s.mapIsB) {
        const Pose& pose = state.pose;
        const Vec3 local = rotate(conjugate(pose.orientation), minus(at, pose.position));
        return norm(s.fieldMap->sampleVector(local)) * std::sqrt(s.dutyCycle);
    }
    if (s.radiatedPower <= 0.0) {
        return 0.0;
    }
    return farField(s.radiatedPower * s.dutyCycle, norm(minus(at, state.pose.position)));
}

Expected<std::vector<Victim>, std::string> victimsOf(const WorldView& world) {
    std::vector<Victim> out;
    for (const SchemaInstance* inst : world.instancesOf("EmiSusceptibilityAPI_1")) {
        Victim v;
        v.name = std::string(world.signalInstanceName(inst->prim));
        v.prim = inst->prim;
        const std::string where = std::string(world.primPath(inst->prim)) + ": ";
        const std::string_view type = inst->token("emi:victimType").value_or("");
        if (type == "magnetometer") {
            v.type = Victim::Type::Magnetometer;
        } else if (type == "imu_adc") {
            v.type = Victim::Type::ImuAdc;
        } else if (type == "encoder") {
            v.type = Victim::Type::Encoder;
        } else if (type == "can_bus") {
            v.type = Victim::Type::CanBus;
        } else if (type == "radio") {
            v.type = Victim::Type::Radio;
        } else {
            return Unexpected{where + "emi:victimType is required"};
        }
        v.body = world.primBody(inst->prim);
        v.poseInBody = world.primPoseInBody(inst->prim);
        v.magneticGain = real(*inst, "emi:magneticGain").value_or(1.0);
        v.accelNoisePerField = real(*inst, "emi:accelNoisePerField").value_or(0.0);
        v.gyroNoisePerField = real(*inst, "emi:gyroNoisePerField").value_or(0.0);
        v.glitchField = real(*inst, "emi:encoderGlitchField").value_or(0.0);
        v.canErrorField = real(*inst, "emi:canErrorField").value_or(0.0);
        v.canErrorRate = real(*inst, "emi:canErrorRateAtField").value_or(1e-5);
        v.loopArea = real(*inst, "emi:couplingLoopArea").value_or(0.0);
        v.shielding = real(*inst, "emi:shieldingEffectiveness").value_or(1.0);
        if (v.shielding < 1.0) {
            return Unexpected{where + "emi:shieldingEffectiveness must be at least 1"};
        }
        if (v.type == Victim::Type::Radio) {
            for (const SchemaInstance* radio : world.instancesOf("RadioLinkAPI_1")) {
                if (radio->prim == inst->prim) {
                    v.rxFrequency = radio->real("comms:frequency").value_or(0.0);
                    v.rxBandwidth = radio->real("comms:bandwidth").value_or(0.0);
                }
            }
            if (v.rxFrequency <= 0.0) {
                return Unexpected{where +
                                  "a radio victim needs RadioLinkAPI_1 on the same prim, "
                                  "with comms:frequency and comms:bandwidth"};
            }
        }
        out.push_back(std::move(v));
    }
    return out;
}

const char* couplingName(Victim::Type type) {
    switch (type) {
        case Victim::Type::Magnetometer:
            return "magnetometer";
        case Victim::Type::ImuAdc:
            return "imu";
        case Victim::Type::Encoder:
            return "encoder";
        case Victim::Type::CanBus:
            return "can_bus";
        case Victim::Type::Radio:
            return "wifi";
    }
    return "";
}

ModuleInfo Emi::info() const {
    return zero_ ? ModuleInfo{"emi", "zero", 0, 1, true, true, CapabilityTier::Tier0}
                 : ModuleInfo{"emi", "analytic", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Emi::preferredStep() const {
    return 1_ms;
}

Status Emi::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    auto victims = victimsOf(world);
    if (!victims) {
        return Status::error(StatusCode::InvalidConfig, victims.error());
    }
    sources_.clear();
    sourceAt_.clear();
    sourceCurrent_.clear();
    victims_.clear();
    const World& raw = world.raw();
    const std::vector<Pose> rest = restPoses(raw);
    const std::vector<bool> moving = movingBodies(raw);
    const auto place = [&](BodyId body, const Pose& mount) {
        Placed p;
        p.mount = mount;
        if (body == kWorldBody) {
            return p;
        }
        p.rest = rest[body.index];
        if (!zero_ && moving[body.index]) {
            p.pose = d.input(spec("body/" + raw.bodies.names[body.index] + "/pose",
                                  SignalType::Pose, units::m, "world_local", true));
        }
        return p;
    };
    if (!zero_) {
        auto sources = sourcesOf(world);
        if (!sources) {
            return Status::error(StatusCode::InvalidConfig, sources.error());
        }
        sources_ = std::move(*sources);
        for (const Source& s : sources_) {
            sourceAt_.push_back(place(s.body, s.poseInBody));
            SignalId current;
            if (!s.currentSignal.empty()) {
                // Any current (RFC 0013): a motor's bus current, a battery's; signed.
                SignalSpec in = spec(s.currentSignal, SignalType::Scalar, units::A, {}, true);
                in.optional = true;
                in.defaultValue = s.current;
                current = d.input(in);
            } else if (!s.drivenBy.empty()) {
                // The motor's current when the electrical network runs, else the source's own.
                SignalSpec in = spec("motor/" + s.drivenBy + "/current_q", SignalType::Scalar,
                                     units::A, {}, true);
                in.optional = true;
                in.defaultValue = s.current;
                current = d.input(in);
            }
            sourceCurrent_.push_back(current);
        }
    }
    const auto coupling = config.get<std::vector<std::string>>("coupling");
    for (Victim& v : *victims) {
        VictimOut o;
        o.at = place(v.body, v.poseInBody);
        o.coupled = !coupling || std::ranges::count(*coupling, couplingName(v.type)) > 0;
        const std::string base = "emi/" + v.name + "/";
        o.b = d.output(spec(base + "b_field", SignalType::Vec3, units::T, "sensor"));
        o.e = d.output(spec(base + "e_field_rms", SignalType::Scalar, units::V_per_m));
        switch (v.type) {
            case Victim::Type::ImuAdc:
                o.accel = d.output(spec(base + "accel_noise", SignalType::Scalar, units::m_per_s2));
                o.gyro = d.output(spec(base + "gyro_noise", SignalType::Scalar, units::rad_per_s));
                break;
            case Victim::Type::Encoder:
                o.glitch =
                    d.output(spec(base + "glitch_probability", SignalType::Scalar, units::one));
                break;
            case Victim::Type::CanBus:
                o.ber = d.output(spec(base + "bit_error_rate", SignalType::Scalar, units::one));
                break;
            case Victim::Type::Radio:
                o.interference =
                    d.output(spec(base + "interference_power", SignalType::Scalar, units::W));
                break;
            case Victim::Type::Magnetometer:
                break;
        }
        if (v.loopArea > 0.0) {
            o.induced = d.output(spec(base + "induced_voltage", SignalType::Scalar, units::V));
        }
        o.v = std::move(v);
        victims_.push_back(std::move(o));
    }
    return {};
}

Status Emi::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    writeZero(bus);
    return {};
}

void Emi::writeZero(SignalBus& bus) const {
    const std::array<double, 3> zero{0.0, 0.0, 0.0};
    for (const VictimOut& o : victims_) {
        bus.writeN(o.b, zero);
        for (const SignalId id :
             {o.e, o.accel, o.gyro, o.glitch, o.ber, o.interference, o.induced}) {
            if (id.valid()) {
                bus.write(id, 0.0);
            }
        }
    }
}

Pose Emi::worldPose(const Placed& p, const SignalBus& bus) const {
    if (!p.pose.valid()) {
        return compose(p.rest, p.mount);
    }
    const std::span<const double> v = bus.readN(p.pose);
    return compose(Pose{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}}, p.mount);
}

Status Emi::step(const StepContext& context, SignalBus& bus) {
    if (zero_) {
        writeZero(bus);
        return {};
    }
    const double t = context.t.seconds();
    std::vector<SourceState>& states = states_;
    states.resize(sources_.size());
    for (std::size_t i = 0; i < sources_.size(); ++i) {
        states[i].pose = worldPose(sourceAt_[i], bus);
        const Source& s = sources_[i];
        if (!sourceCurrent_[i].valid()) {
            states[i].current = s.current;
        } else if (!s.currentSignal.empty() && s.frequency <= 0.0) {
            states[i].current = bus.read(sourceCurrent_[i]);  // direct: its direction counts
        } else {
            states[i].current = std::abs(bus.read(sourceCurrent_[i]));
        }
    }
    for (const VictimOut& o : victims_) {
        if (!o.coupled) {
            continue;  // stays zero from init
        }
        const Victim& v = o.v;
        const Pose pose = worldPose(o.at, bus);
        Vec3 b{};
        double e2 = 0.0;
        double interference = 0.0;
        double induced2 = 0.0;
        for (std::size_t i = 0; i < sources_.size(); ++i) {
            const Source& s = sources_[i];
            const Vec3 bi = magneticField(s, states[i], pose.position, t);
            b = {b[0] + bi[0], b[1] + bi[1], b[2] + bi[2]};
            const double e = electricFieldRms(s, states[i], pose.position);
            e2 += e * e;  // incoherent sources add in power
            if (v.type == Victim::Type::Radio && e > 0.0) {
                interference += receivedPower(e / v.shielding, s.frequency) *
                                bandOverlap(s.frequency, s.bandwidth, v.rxFrequency, v.rxBandwidth);
            }
            if (v.loopArea > 0.0) {
                const double emf = 2.0 * std::numbers::pi * v.loopArea *
                                   magneticFieldRms(s, states[i], pose.position).bTimesFrequency;
                induced2 += emf * emf;
            }
        }
        const double gain = v.type == Victim::Type::Magnetometer ? v.magneticGain : 1.0;
        const Vec3 local = rotate(conjugate(pose.orientation), b);
        bus.writeN(o.b, scaled(local, gain / v.shielding));
        const double e = std::sqrt(e2) / v.shielding;
        bus.write(o.e, e);
        if (o.accel.valid()) {
            bus.write(o.accel, v.accelNoisePerField * e);
            bus.write(o.gyro, v.gyroNoisePerField * e);
        }
        if (o.glitch.valid()) {
            bus.write(o.glitch, glitchProbability(e, v.glitchField));
        }
        if (o.ber.valid()) {
            bus.write(o.ber, bitErrorRate(e, v.canErrorField, v.canErrorRate));
        }
        if (o.interference.valid()) {
            bus.write(o.interference, interference);
        }
        if (o.induced.valid()) {
            bus.write(o.induced, std::sqrt(induced2) / v.shielding);
        }
    }
    return {};
}

}  // namespace lagrange::emi
