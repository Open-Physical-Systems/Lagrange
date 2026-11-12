// Built-in kind `wifi_scanner` (sensors.md §9): the received signal strength of every access
// point the robot hears, as a Wi-Fi scan reports it.
//
// Access points, in this order (the order of sensor/<n>/rssi):
//   1. every RadioLinkAPI_1 whose receiver prim is on the sensor's body: its transmitter. It
//      reads link/<rx>/rx_power from the comms domain (walls, shadowing, comms.md); with the
//      comms domain off (unconnected, or the ideal substitute's 1 W sentinel) the link's own
//      attributes go through the log-distance and wall model below;
//   2. the points of the text parameter `access_points` ("x y z; x y z", m, region-local);
//   3. every world prim or body whose name starts with `wifi_ap`, in load order.
// 2 and 3 use the parameters tx_power, frequency, path_loss_exponent, reference_distance and
// wall_loss_db; walls are the colliders of bodies that never move (fixed scenery), the ground
// left out.
#include "KindModule.hpp"
#include "ObstacleScene.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kSpeedOfLight = 299792458.0;  // m/s (exact, SI)
/// What comms' ideal substitute writes as rx_power (comms::kIdealRxPower): no real link.
constexpr double kIdealRxPowerW = 1.0;
/// Minimum input sensitivity of the OFDM PHY at 6 Mb/s, 20 MHz channel (IEEE 802.11-2020,
/// Table 17-18): a beacon weaker than this is not decoded, so the scan does not list the AP.
constexpr double kDefaultSensitivityDbm = -82.0;
/// Time of one full active scan, s. A scan dwells MinChannelTime to MaxChannelTime on every
/// channel (IEEE 802.11-2020 §11.1.4.3.2; drivers use some 20–100 ms) over the 13 channels of
/// 2.4 GHz and some 25 of 5 GHz: one to four seconds. TODO(verify) against a measured adapter.
constexpr double kDefaultScanTimeS = 2.0;
constexpr double kDefaultTxPowerW = 0.1;         // 20 dBm, a typical access point
constexpr double kDefaultFrequencyHz = 2.437e9;  // 2.4 GHz channel 6
constexpr double kDefaultExponent = 3.0;         // indoor, obstructed (as the sample link)
constexpr double kDefaultWallLossDb = 10.0;      // per wall (as the sample link), illustrative

double toDbm(double watts) {
    return 10.0 * std::log10(watts / 1e-3);
}

class WifiScanner final : public KindModule {
public:
    using KindModule::KindModule;

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        aps_.clear();
        poses_.reset(world);
        poses_.watch(d, body_);
        sensitivity_ = kindSetting(world, sensor_, "sensitivity", kDefaultSensitivityDbm);
        scanTime_ = kindSetting(world, sensor_, "scan_time", kDefaultScanTimeS);
        noise_ = kindSetting(world, sensor_, "noise", 0.0);
        const double txPower = kindSetting(world, sensor_, "tx_power", kDefaultTxPowerW);
        const double frequency = kindSetting(world, sensor_, "frequency", kDefaultFrequencyHz);
        const double exponent = kindSetting(world, sensor_, "path_loss_exponent", kDefaultExponent);
        const double reference = kindSetting(world, sensor_, "reference_distance", 1.0);
        const double wallLossDb = kindSetting(world, sensor_, "wall_loss_db", kDefaultWallLossDb);
        if (txPower <= 0.0 || frequency <= 0.0 || reference <= 0.0 || scanTime_ < 0.0 ||
            noise_ < 0.0 || wallLossDb < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "wifi_scanner " + name_ +
                                     ": tx_power, frequency and reference_distance must be "
                                     "positive, scan_time, noise and wall_loss_db not negative");
        }
        Ap model;
        model.txPower = txPower;
        model.frequency = frequency;
        model.exponent = exponent;
        model.reference = reference;
        model.wallLossDb = wallLossDb;
        // 1. links received on this body.
        for (const SchemaInstance* link : view.instancesOf("RadioLinkAPI_1")) {
            const std::span<const PrimId> tx = link->rel("comms:transmitter");
            if (view.primBody(link->prim).index != body_ || tx.empty()) {
                continue;
            }
            Ap ap;
            ap.txPower = link->real("comms:txPower").value_or(txPower);
            ap.frequency = link->real("comms:frequency").value_or(frequency);
            ap.exponent = link->real("comms:pathLossExponent").value_or(2.0);
            ap.reference = link->real("comms:referenceDistance").value_or(1.0);
            ap.wallLossDb =
                10.0 * std::log10(std::max(1.0, link->real("comms:wallLoss").value_or(1.0)));
            ap.gain = link->real("comms:txAntennaGain").value_or(1.0) *
                      link->real("comms:rxAntennaGain").value_or(1.0);
            ap.body = view.primBody(tx[0]).index;
            ap.local = view.primPoseInBody(tx[0]).position;
            ap.fixed = view.primBody(tx[0]) == kWorldBody;
            ap.rxPower = d.input(optionalKindSpec(
                "link/" + std::string(view.signalInstanceName(link->prim)) + "/rx_power",
                SignalType::Scalar, units::W));
            if (!ap.fixed) {
                poses_.watch(d, ap.body);
            }
            aps_.push_back(ap);
        }
        // 2. points given as a parameter.
        std::vector<Vec3> points;
        if (!parsePoints(kindText(world, sensor_, "access_points"), points)) {
            return Status::error(
                StatusCode::InvalidConfig,
                "wifi_scanner " + name_ + ": access_points is \"x y z; x y z; ...\" in metres");
        }
        for (const Vec3& p : points) {
            Ap ap = model;
            ap.local = p;
            ap.fromText = true;
            aps_.push_back(ap);
        }
        // 3. prims and bodies named wifi_ap*; a body's own prim stands for the body.
        for (const NamedPlace& place : namedPlaces(world, "wifi_ap")) {
            Ap ap = model;
            ap.fixed = place.fixed;
            ap.body = place.body;
            ap.local = place.local;
            if (!ap.fixed) {
                poses_.watch(d, ap.body);
            }
            aps_.push_back(ap);
        }
        // Walls: what never moves, the ground, the scanner's body and the access points' own
        // bodies left out.
        const std::vector<bool> moving = movingBodies(world);
        std::vector<bool> skip(world.bodies.names.size(), false);
        if (body_ < skip.size()) {
            skip[body_] = true;
        }
        for (const Ap& ap : aps_) {
            if (!ap.fixed) {
                skip[ap.body] = true;
            }
        }
        walls_.declare(world, poses_, d, [&](std::uint32_t c) {
            const BodyId owner = world.colliders.body[c];
            const ShapeType type = world.colliders.shape[c];
            return type != ShapeType::Plane && type != ShapeType::HeightField &&
                   (owner == kWorldBody || (!moving[owner.index] && !skip[owner.index]));
        });
        const auto n = static_cast<std::uint32_t>(std::max<std::size_t>(aps_.size(), 1));
        SignalSpec rssi = kindSpec(base() + "/rssi", SignalType::ScalarArray, units::one);
        rssi.arrayLength = n;
        rssiOut_ = d.output(rssi);
        SignalSpec power = kindSpec(base() + "/power", SignalType::ScalarArray, units::W);
        power.arrayLength = n;
        powerOut_ = d.output(power);
        countOut_ = d.output(kindSpec(base() + "/count", SignalType::Int, units::one));
        validOut_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        rssi_.assign(n, kNaN);
        power_.assign(n, kNaN);
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        nextScan_ = 0.0;
        std::fill(rssi_.begin(), rssi_.end(), kNaN);
        std::fill(power_.begin(), power_.end(), kNaN);
        publish(bus, 0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const double now = (context.t + context.dt).seconds();
        if (now + 1e-12 < nextScan_) {
            bus.hold(rssiOut_);
            bus.hold(powerOut_);
            bus.hold(countOut_);
            bus.hold(validOut_);
            return {};
        }
        nextScan_ = now + scanTime_;
        walls_.place(bus, poses_);
        const Vec3 at = compose(poses_.pose(bus, body_), mount_).position;
        random::Stream rng(context.seed, context.stepIndex, 0);
        int heard = 0;
        for (std::size_t i = 0; i < aps_.size(); ++i) {
            const Ap& ap = aps_[i];
            double watts = ap.rxPower.valid() ? bus.read(ap.rxPower) : kNaN;
            if (!std::isfinite(watts) || watts >= kIdealRxPowerW) {
                const Vec3 from =
                    ap.fixed
                        ? ap.local
                        : compose(poses_.pose(bus, ap.body), Pose{ap.local, {1, 0, 0, 0}}).position;
                watts = modelPower(ap, from, at);
            }
            double dbm = watts > 0.0 ? toDbm(watts) : -std::numeric_limits<double>::infinity();
            if (!groundTruth_) {
                if (noise_ > 0.0) {
                    dbm += noise_ * rng.normal();
                }
                dbm = std::round(dbm);  // scans report whole dBm
            }
            if (!groundTruth_ && dbm < sensitivity_) {
                rssi_[i] = kNaN;
                power_[i] = kNaN;
                continue;
            }
            rssi_[i] = dbm;
            power_[i] = 1e-3 * std::pow(10.0, dbm / 10.0);
            ++heard;
        }
        publish(bus, heard);
        return {};
    }

    Status saveState(StateWriter& writer) const override {
        writer.put(nextScan_);
        writer.putDoubles(rssi_);
        writer.putDoubles(power_);
        return {};
    }
    Status restoreState(StateReader& reader) override {
        if (!reader.get(nextScan_) || !reader.getDoubles(rssi_) || !reader.getDoubles(power_)) {
            return Status::error(StatusCode::Internal,
                                 "wifi_scanner " + name_ + ": saved state does not fit");
        }
        return {};
    }
    /// A floating-origin rebase rebuilds the run from shifted world tables (named prims move
    /// with them) but reads `access_points` again as written: those move here (QA 2026-11-12
    /// #17).
    void shiftOrigin(const std::array<double, 3>& shift) override {
        for (Ap& ap : aps_) {
            if (ap.fromText) {
                for (std::size_t i = 0; i < 3; ++i) {
                    ap.local[i] -= shift[i];
                }
            }
        }
    }

private:
    struct Ap {
        double txPower{kDefaultTxPowerW};  // W
        double frequency{kDefaultFrequencyHz};
        double exponent{kDefaultExponent};
        double reference{1.0};  // m
        double wallLossDb{kDefaultWallLossDb};
        double gain{1.0};  // tx × rx antenna gain, linear
        bool fixed{true};  // `local` is region-local; else in body `body`
        std::uint32_t body{0};
        Vec3 local{};
        bool fromText{false};  // given by `access_points`, in the region frame the run began in
        SignalId rxPower;  // link/<rx>/rx_power, for a RadioLinkAPI_1
    };

    /// Log-distance path loss with walls (comms.md §2): free space (Friis) to the reference
    /// distance d0, then (d/d0)^n, and wall_loss_db per wall crossed.
    double modelPower(const Ap& ap, const Vec3& from, const Vec3& to) const {
        const double dx = to[0] - from[0], dy = to[1] - from[1], dz = to[2] - from[2];
        const double d = std::max(std::sqrt(dx * dx + dy * dy + dz * dz), 1e-3);
        const double lambda = kSpeedOfLight / ap.frequency;
        const auto friis = [&](double r) {
            const double x = 4.0 * std::numbers::pi * r / lambda;
            return x * x;
        };
        const double loss = d <= ap.reference
                                ? friis(d)
                                : friis(ap.reference) * std::pow(d / ap.reference, ap.exponent);
        const int walls = walls_.crossing(from, to).entered;
        return ap.txPower * ap.gain / loss * std::pow(10.0, -ap.wallLossDb * walls / 10.0);
    }

    void publish(SignalBus& bus, int heard) {
        if (aps_.empty()) {
            bus.writeN(rssiOut_, std::vector<double>{kNaN});
            bus.writeN(powerOut_, std::vector<double>{kNaN});
        } else {
            bus.writeN(rssiOut_, rssi_);
            bus.writeN(powerOut_, power_);
        }
        bus.write(countOut_, heard);
        bus.write(validOut_, heard > 0 ? 1.0 : 0.0);
    }

    double sensitivity_{kDefaultSensitivityDbm}, scanTime_{kDefaultScanTimeS}, noise_{0.0};
    double nextScan_{0.0};
    std::vector<Ap> aps_;
    std::vector<double> rssi_, power_;
    BodyPoses poses_;
    ObstacleScene walls_;
    SignalId rssiOut_, powerOut_, countOut_, validOut_;
};

}  // namespace

std::unique_ptr<IModule> createWifiScanner(const World& world, std::uint32_t sensor,
                                           bool groundTruth, const KindContext&) {
    return std::make_unique<WifiScanner>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
