// The comms domain: link budgets, packet errors and CAN fault confinement (Comms.hpp,
// domains/comms.md).
#include "lagrange/comms/Comms.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::comms {

using namespace literals;

namespace {

constexpr double kWallStep = 1e-6;  // m past a wall's entry or exit before the next ray
constexpr int kMaxWalls = 64;

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

SignalSpec optional(std::string name, Unit unit) {
    SignalSpec s = spec(std::move(name), SignalType::Scalar, unit);
    s.optional = true;
    return s;
}

double distance(const Vec3& a, const Vec3& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                     (a[2] - b[2]) * (a[2] - b[2]));
}

}  // namespace

double freeSpacePathLoss(double distance, double frequency) {
    const double lambda = kSpeedOfLight / frequency;
    const double x = 4.0 * std::numbers::pi * distance / lambda;
    return x * x;
}

double pathLoss(double distance, double frequency, double referenceDistance, double exponent) {
    if (distance <= referenceDistance) {
        return freeSpacePathLoss(distance, frequency);
    }
    return freeSpacePathLoss(referenceDistance, frequency) *
           std::pow(distance / referenceDistance, exponent);
}

double thermalNoise(double bandwidth, double noiseFigure) {
    return kBoltzmann * kNoiseReferenceK * bandwidth * noiseFigure;
}

int bitsPerSymbol(Modulation m) {
    switch (m) {
        case Modulation::Bpsk:
            return 1;
        case Modulation::Qpsk:
            return 2;
        case Modulation::Qam16:
            return 4;
        case Modulation::Qam64:
            return 6;
    }
    return 1;
}

double gaussianTail(double x) {
    return 0.5 * std::erfc(x / std::numbers::sqrt2);
}

double bitErrorRate(Modulation m, double snr) {
    if (snr <= 0.0) {
        return 0.5;
    }
    const int k = bitsPerSymbol(m);
    const double ebn0 = snr / k;
    if (m == Modulation::Bpsk || m == Modulation::Qpsk) {
        return gaussianTail(std::sqrt(2.0 * ebn0));
    }
    const double points = std::pow(2.0, k);
    const double ber = 4.0 / k * (1.0 - 1.0 / std::sqrt(points)) *
                       gaussianTail(std::sqrt(3.0 * k / (points - 1.0) * ebn0));
    return std::min(ber, 0.5);
}

double packetErrorRate(double ber, double bits) {
    if (ber <= 0.0) {
        return 0.0;
    }
    if (ber >= 1.0) {
        return 1.0;
    }
    return -std::expm1(bits * std::log1p(-ber));
}

void CanFaultConfinement::transmitError() {
    if (state_ == State::BusOff) {
        return;
    }
    tec_ += kErrorIncrement;
    update();
}

void CanFaultConfinement::transmitSuccess() {
    if (state_ == State::BusOff) {
        return;
    }
    tec_ = std::max(0, tec_ - 1);
    update();
}

std::int64_t CanFaultConfinement::idle(std::int64_t bits) {
    if (state_ != State::BusOff) {
        return bits;
    }
    if (bits < recovery_) {
        recovery_ -= bits;
        return 0;
    }
    const std::int64_t left = bits - recovery_;
    recovery_ = 0;
    tec_ = 0;
    state_ = State::ErrorActive;
    return left;
}

void CanFaultConfinement::restore(State state, int tec, std::int64_t recovery) {
    state_ = state;
    tec_ = tec;
    recovery_ = recovery;
}

void CanFaultConfinement::update() {
    if (tec_ > kBusOffAbove) {
        state_ = State::BusOff;
        recovery_ = kRecoveryBits;
    } else if (tec_ > kPassiveAbove) {
        state_ = State::ErrorPassive;
    } else {
        state_ = State::ErrorActive;
    }
}

Expected<std::vector<Link>, std::string> linksOf(const WorldView& world) {
    std::vector<Link> out;
    for (const SchemaInstance* inst : world.instancesOf("RadioLinkAPI_1")) {
        Link l;
        l.name = std::string(world.signalInstanceName(inst->prim));
        l.rx = inst->prim;
        const std::string where = std::string(world.primPath(inst->prim)) + ": ";
        const std::span<const PrimId> tx = inst->rel("comms:transmitter");
        if (tx.empty()) {
            return Unexpected{where + "comms:transmitter is required"};
        }
        l.tx = tx[0];
        const auto frequency = inst->real("comms:frequency");
        const auto bandwidth = inst->real("comms:bandwidth");
        const auto power = inst->real("comms:txPower");
        if (!frequency || !bandwidth || !power || *frequency <= 0.0 || *bandwidth <= 0.0 ||
            *power <= 0.0) {
            return Unexpected{where +
                              "comms:frequency, comms:bandwidth and comms:txPower are "
                              "required and positive"};
        }
        l.frequency = *frequency;
        l.bandwidth = *bandwidth;
        l.txPower = *power;
        l.txGain = inst->real("comms:txAntennaGain").value_or(1.0);
        l.rxGain = inst->real("comms:rxAntennaGain").value_or(1.0);
        l.exponent = inst->real("comms:pathLossExponent").value_or(2.0);
        l.referenceDistance = inst->real("comms:referenceDistance").value_or(1.0);
        l.wallLoss = inst->real("comms:wallLoss").value_or(1.0);
        l.shadowingDb = inst->real("comms:shadowingStd").value_or(0.0);
        l.correlationDistance = inst->real("comms:shadowingCorrelationDistance").value_or(0.0);
        l.noiseFigure = inst->real("comms:noiseFigure").value_or(1.0);
        l.packetBits = static_cast<double>(inst->integer("comms:packetBits").value_or(12000));
        l.maxRetries = static_cast<int>(inst->integer("comms:maxRetries").value_or(0));
        const std::string_view modulation = inst->token("comms:modulation").value_or("qpsk");
        if (modulation == "bpsk") {
            l.modulation = Modulation::Bpsk;
        } else if (modulation == "qam16") {
            l.modulation = Modulation::Qam16;
        } else if (modulation == "qam64") {
            l.modulation = Modulation::Qam64;
        }
        if (l.referenceDistance <= 0.0 || l.wallLoss < 1.0 || l.noiseFigure < 1.0 ||
            l.packetBits <= 0.0 || l.maxRetries < 0 || l.shadowingDb < 0.0) {
            return Unexpected{where +
                              "comms:referenceDistance and comms:packetBits must be "
                              "positive, comms:wallLoss and comms:noiseFigure at least 1, "
                              "comms:maxRetries and comms:shadowingStd not negative"};
        }
        out.push_back(std::move(l));
    }
    return out;
}

Expected<std::vector<CanBus>, std::string> canBusesOf(const WorldView& world) {
    std::vector<CanBus> out;
    for (const SchemaInstance* inst : world.instancesOf("CanBusAPI_1")) {
        CanBus c;
        c.name = std::string(world.signalInstanceName(inst->prim));
        const auto rate = inst->real("can:bitRate");
        if (!rate || *rate <= 0.0) {
            return Unexpected{std::string(world.primPath(inst->prim)) +
                              ": can:bitRate is required and positive"};
        }
        c.bitRate = *rate;
        c.frameBits = static_cast<double>(inst->integer("can:frameBits").value_or(108));
        out.push_back(std::move(c));
    }
    return out;
}

LinkQuality linkQuality(const Link& link, double distance, int walls, double shadowDb,
                        double interference) {
    LinkQuality q;
    const double loss =
        pathLoss(std::max(distance, 1e-3), link.frequency, link.referenceDistance, link.exponent) *
        std::pow(link.wallLoss, walls) * std::pow(10.0, -shadowDb / 10.0);
    q.rxPower = link.txPower * link.txGain * link.rxGain / loss;
    q.snr = q.rxPower / (thermalNoise(link.bandwidth, link.noiseFigure) + interference);
    const double per = packetErrorRate(bitErrorRate(link.modulation, q.snr), link.packetBits);
    const int attempts = link.maxRetries + 1;
    q.packetLoss = std::pow(per, attempts);
    // Expected transmissions per packet: 1 + PER + … + PER^r.
    const double tries = per < 1.0 ? (1.0 - q.packetLoss) / (1.0 - per) : attempts;
    const double rate = link.bandwidth * bitsPerSymbol(link.modulation);  // bit/s
    q.latency = tries * link.packetBits / rate;
    q.throughput = rate * (1.0 - per);
    q.up = q.packetLoss < 0.5;
    return q;
}

int Comms::countWalls(const services::SceneQuery& query, const Vec3& from, const Vec3& to) {
    const double length = distance(from, to);
    if (length <= 0.0) {
        return 0;
    }
    const Vec3 dir{(to[0] - from[0]) / length, (to[1] - from[1]) / length,
                   (to[2] - from[2]) / length};
    int walls = 0;
    double travelled = 0.0;
    while (walls < kMaxWalls) {
        const Vec3 at{from[0] + dir[0] * travelled, from[1] + dir[1] * travelled,
                      from[2] + dir[2] * travelled};
        const services::RayHit entry = query.ray(at, dir, length - travelled);
        if (!entry.hit()) {
            break;
        }
        ++walls;
        travelled += entry.distance + kWallStep;
        // From inside, a ray reports where it leaves the shape.
        const Vec3 inside{from[0] + dir[0] * travelled, from[1] + dir[1] * travelled,
                          from[2] + dir[2] * travelled};
        const services::RayHit exit = query.ray(inside, dir, length - travelled);
        if (!exit.hit()) {
            break;  // the receiver is inside it
        }
        travelled += exit.distance + kWallStep;
        if (travelled >= length) {
            break;
        }
    }
    return walls;
}

ModuleInfo Comms::info() const {
    return ideal_ ? ModuleInfo{"comms", "ideal", 0, 1, true, true, CapabilityTier::Tier0}
                  : ModuleInfo{"comms", "link_budget", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Comms::preferredStep() const {
    return 10_ms;
}

Status Comms::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) {
    auto links = linksOf(world);
    if (!links) {
        return Status::error(StatusCode::InvalidConfig, links.error());
    }
    auto buses = canBusesOf(world);
    if (!buses) {
        return Status::error(StatusCode::InvalidConfig, buses.error());
    }
    links_.clear();
    buses_.clear();
    const World& raw = world.raw();
    const std::vector<Pose> rest = restPoses(raw);
    const std::vector<bool> moving = movingBodies(raw);
    const auto place = [&](PrimId prim) {
        Placed p;
        p.mount = world.primPoseInBody(prim);
        const BodyId body = world.primBody(prim);
        if (body == kWorldBody) {
            return p;
        }
        p.rest = rest[body.index];
        if (!ideal_ && moving[body.index]) {
            SignalSpec pose = spec("body/" + raw.bodies.names[body.index] + "/pose",
                                   SignalType::Pose, units::m, "world_local");
            pose.feedback = true;
            p.pose = d.input(pose);
        }
        return p;
    };
    for (Link& l : *links) {
        LinkState s;
        s.tx = place(l.tx);
        s.rx = place(l.rx);
        const std::string base = "link/" + l.name + "/";
        if (!ideal_) {
            s.interference = d.input(optional("emi/" + l.name + "/interference_power", units::W));
        }
        s.rxPower = d.output(spec(base + "rx_power", SignalType::Scalar, units::W));
        s.snr = d.output(spec(base + "snr", SignalType::Scalar, units::one));
        s.per = d.output(spec(base + "packet_error_rate", SignalType::Scalar, units::one));
        s.latency = d.output(spec(base + "latency", SignalType::Scalar, units::s));
        s.throughput = d.output(spec(base + "throughput", SignalType::Scalar, units::bit_per_s));
        s.up = d.output(spec(base + "up", SignalType::Bool, units::one));
        s.link = std::move(l);
        links_.push_back(std::move(s));
    }
    for (CanBus& c : *buses) {
        BusState s;
        const std::string base = "can/" + c.name + "/";
        if (!ideal_) {
            s.ber = d.input(optional("emi/" + c.name + "/bit_error_rate", units::one));
        }
        s.fer = d.output(spec(base + "frame_error_rate", SignalType::Scalar, units::one));
        s.state = d.output(spec(base + "state", SignalType::Int, units::one));
        s.tec = d.output(spec(base + "tx_error_count", SignalType::Int, units::one));
        s.bus = std::move(c);
        buses_.push_back(std::move(s));
    }
    return {};
}

Status Comms::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (LinkState& l : links_) {
        l.started = false;
        l.shadow = 0.0;
    }
    for (BusState& b : buses_) {
        b.node = CanFaultConfinement{};
        b.pendingFrames = 0.0;
    }
    writeIdeal(bus);
    return {};
}

void Comms::writeIdeal(SignalBus& bus) const {
    for (const LinkState& l : links_) {
        bus.write(l.rxPower, kIdealRxPower);
        bus.write(l.snr, kIdealSnr);
        bus.write(l.per, 0.0);
        bus.write(l.latency, 0.0);
        bus.write(l.throughput, l.link.bandwidth * bitsPerSymbol(l.link.modulation));
        bus.write(l.up, 1.0);
    }
    for (const BusState& b : buses_) {
        bus.write(b.fer, 0.0);
        bus.write(b.state, 0.0);
        bus.write(b.tec, 0.0);
    }
}

Pose Comms::worldPose(const Placed& p, const SignalBus& bus) const {
    if (!p.pose.valid()) {
        return compose(p.rest, p.mount);
    }
    const std::span<const double> v = bus.readN(p.pose);
    return compose(Pose{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}}, p.mount);
}

Status Comms::step(const StepContext& context, SignalBus& bus) {
    if (ideal_) {
        writeIdeal(bus);
        return {};
    }
    for (std::size_t i = 0; i < links_.size(); ++i) {
        LinkState& s = links_[i];
        const Link& l = s.link;
        const Vec3 tx = worldPose(s.tx, bus).position;
        const Vec3 rx = worldPose(s.rx, bus).position;
        if (l.shadowingDb > 0.0) {
            // Gudmundson: an AR(1) process over the receiver's travel.
            random::Stream rng(context.seed, context.stepIndex, static_cast<std::uint32_t>(i));
            const double z = rng.normal();
            if (!s.started) {
                s.shadow = l.shadowingDb * z;
            } else {
                const double rho = l.correlationDistance > 0.0
                                       ? std::exp(-distance(rx, s.lastRx) / l.correlationDistance)
                                       : 0.0;
                s.shadow = rho * s.shadow + std::sqrt(1.0 - rho * rho) * l.shadowingDb * z;
            }
        }
        s.started = true;
        s.lastRx = rx;
        const int walls = walls_ != nullptr && l.wallLoss > 1.0 ? countWalls(*walls_, tx, rx) : 0;
        const double interference = s.interference.valid() ? bus.read(s.interference) : 0.0;
        const LinkQuality q = linkQuality(l, distance(tx, rx), walls, s.shadow, interference);
        bus.write(s.rxPower, q.rxPower);
        bus.write(s.snr, q.snr);
        bus.write(s.per, q.packetLoss);
        bus.write(s.latency, q.latency);
        bus.write(s.throughput, q.throughput);
        bus.write(s.up, q.up ? 1.0 : 0.0);
    }
    const double dt = context.dt.seconds();
    for (std::size_t i = 0; i < buses_.size(); ++i) {
        BusState& b = buses_[i];
        const double ber = b.ber.valid() ? std::clamp(bus.read(b.ber), 0.0, 1.0) : 0.0;
        const double fer = packetErrorRate(ber, b.bus.frameBits);
        // A frame after a frame: the bus's bit times this step, in frames, while not bus off.
        random::Stream rng(context.seed, context.stepIndex,
                           static_cast<std::uint32_t>(links_.size() + i));
        auto bits = static_cast<std::int64_t>(std::llround(b.bus.bitRate * dt));
        bits = b.node.idle(bits);
        b.pendingFrames += static_cast<double>(bits) / b.bus.frameBits;
        while (b.pendingFrames >= 1.0) {
            b.pendingFrames -= 1.0;
            if (b.node.state() == CanFaultConfinement::State::BusOff) {
                // Bus off mid-step: the rest of the step's bits count towards recovery.
                const auto rest = static_cast<std::int64_t>(b.pendingFrames * b.bus.frameBits);
                b.pendingFrames = static_cast<double>(b.node.idle(rest)) / b.bus.frameBits;
                continue;
            }
            if (rng.uniform() < fer) {
                b.node.transmitError();
            } else {
                b.node.transmitSuccess();
            }
        }
        bus.write(b.fer, fer);
        bus.write(b.state, static_cast<double>(b.node.state()));
        bus.write(b.tec, static_cast<double>(b.node.transmitErrorCount()));
    }
    return {};
}

Status Comms::saveState(StateWriter& writer) const {
    for (const LinkState& l : links_) {
        writer.put(l.shadow);
        writer.put(l.started);
        writer.putDoubles(l.lastRx);
    }
    for (const BusState& b : buses_) {
        writer.put(static_cast<std::uint8_t>(b.node.state()));
        writer.put(b.node.transmitErrorCount());
        writer.put(b.node.recoveryLeft());
        writer.put(b.pendingFrames);
    }
    return {};
}

Status Comms::restoreState(StateReader& reader) {
    for (LinkState& l : links_) {
        if (!reader.get(l.shadow) || !reader.get(l.started) || !reader.getDoubles(l.lastRx)) {
            return Status::error(StatusCode::Internal, "comms: truncated state");
        }
    }
    for (BusState& b : buses_) {
        std::uint8_t state = 0;
        int tec = 0;
        std::int64_t recovery = 0;
        if (!reader.get(state) || !reader.get(tec) || !reader.get(recovery) ||
            !reader.get(b.pendingFrames)) {
            return Status::error(StatusCode::Internal, "comms: truncated state");
        }
        b.node.restore(static_cast<CanFaultConfinement::State>(state), tec, recovery);
    }
    return {};
}

}  // namespace lagrange::comms
