#pragma once

// The comms domain (docs/design/domains/comms.md): the quality of radio links and CAN buses.
// The module computes it; bridges apply it to the messages they carry.

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/services/SceneQuery.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::comms {

using Vec3 = std::array<double, 3>;

/// Boltzmann constant, J/K (exact, SI 2019).
inline constexpr double kBoltzmann = 1.380649e-23;
/// The reference temperature of noise figures, K (T0 = 290 K, IEEE Std 100 / Friis 1944).
inline constexpr double kNoiseReferenceK = 290.0;
/// Speed of light in vacuum, m/s (exact, SI).
inline constexpr double kSpeedOfLight = 299792458.0;

/// Friis free-space path loss as a linear power ratio, (4π·d/λ)² (at least 1 is not enforced:
/// below λ/4π the far-field formula does not hold, and the caller keeps d ≥ d0).
[[nodiscard]] double freeSpacePathLoss(double distance, double frequency);
/// Log-distance path loss: free space up to d0, then FSPL(d0)·(d/d0)ⁿ. Linear.
[[nodiscard]] double pathLoss(double distance, double frequency, double referenceDistance,
                              double exponent);
/// Thermal noise power in a band, k·T0·B·F, W.
[[nodiscard]] double thermalNoise(double bandwidth, double noiseFigure);

enum class Modulation { Bpsk, Qpsk, Qam16, Qam64 };
[[nodiscard]] int bitsPerSymbol(Modulation m);
/// Q(x) = ½·erfc(x/√2), the Gaussian tail.
[[nodiscard]] double gaussianTail(double x);
/// Bit-error rate in additive white Gaussian noise at a signal-to-noise ratio `snr` (linear,
/// over the channel bandwidth, one symbol per hertz, so Eb/N0 = SNR / bits per symbol), Gray
/// coded: BPSK and QPSK Q(√(2·Eb/N0)) (exact), square M-QAM
/// (4/k)(1 − 1/√M)·Q(√(3k/(M − 1)·Eb/N0)) (the nearest-neighbour approximation; Proakis and
/// Salehi, Digital Communications, 5th ed., §4.3).
[[nodiscard]] double bitErrorRate(Modulation m, double snr);
/// The probability that a packet of `bits` has an error, 1 − (1 − BER)^bits.
[[nodiscard]] double packetErrorRate(double ber, double bits);

/// CAN fault confinement for one transmitting node (ISO 11898-1, the transmit error counter):
/// +8 per error, −1 per frame sent well; error passive above 127, bus off above 255; bus off
/// recovers to error active, counter 0, after 128 occurrences of 11 recessive bits.
class CanFaultConfinement {
public:
    enum class State : std::uint8_t { ErrorActive = 0, ErrorPassive = 1, BusOff = 2 };
    static constexpr int kErrorIncrement = 8;
    static constexpr int kPassiveAbove = 127;
    static constexpr int kBusOffAbove = 255;
    static constexpr std::int64_t kRecoveryBits = 128 * 11;

    void transmitError();
    void transmitSuccess();
    /// Bit times of an idle bus in bus off; returns the bits left over after recovering.
    std::int64_t idle(std::int64_t bits);
    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] int transmitErrorCount() const { return tec_; }
    [[nodiscard]] std::int64_t recoveryLeft() const { return recovery_; }
    void restore(State state, int tec, std::int64_t recovery);

private:
    void update();
    State state_{State::ErrorActive};
    int tec_{0};
    std::int64_t recovery_{0};  // bit times left before bus off recovers
};

/// One RadioLinkAPI_1 (on the receiving radio's prim), read once at load.
struct Link {
    std::string name;  // the receiver prim's instance name
    PrimId rx, tx;
    double frequency{0.0}, bandwidth{0.0}, txPower{0.0};
    double txGain{1.0}, rxGain{1.0};
    double exponent{2.0}, referenceDistance{1.0};
    double wallLoss{1.0};  // linear, per wall
    double shadowingDb{0.0}, correlationDistance{0.0};
    double noiseFigure{1.0};
    Modulation modulation{Modulation::Qpsk};
    double packetBits{12000.0};
    int maxRetries{0};
};
[[nodiscard]] Expected<std::vector<Link>, std::string> linksOf(const WorldView& world);

/// One CanBusAPI_1.
struct CanBus {
    std::string name;
    double bitRate{0.0};
    double frameBits{108.0};
};
[[nodiscard]] Expected<std::vector<CanBus>, std::string> canBusesOf(const WorldView& world);

/// What one link delivers.
struct LinkQuality {
    double rxPower{0.0};     // W
    double snr{0.0};         // linear
    double packetLoss{1.0};  // after retries
    double latency{0.0};     // s, the expected airtime with retries
    double throughput{0.0};  // bit/s
    bool up{false};
};
/// The quality of `link` at distance `distance` through `walls` walls, with shadowing
/// `shadowDb` (dB) and interference power `interference` (W) at the receiver.
[[nodiscard]] LinkQuality linkQuality(const Link& link, double distance, int walls, double shadowDb,
                                      double interference);

/// The ideal substitute's sentinels: a received power and SNR far above any real link.
inline constexpr double kIdealRxPower = 1.0;  // W
inline constexpr double kIdealSnr = 1e12;

/// The comms domain at its `link_budget` fidelity, every 10 ms (comms.md). Per link l it writes
/// link/<l>/rx_power (W), snr, packet_error_rate (packets lost after retries), latency (s),
/// throughput (bit/s) and up (delivered with probability at least ½); per CAN bus c
/// can/<c>/frame_error_rate, can/<c>/state (0 error active, 1 error passive, 2 bus off) and
/// can/<c>/tx_error_count. Inputs: the poses of the moving bodies carrying radios, and the
/// optional emi/<rx>/interference_power and emi/<c>/bit_error_rate (zero without EMI).
///
/// Walls: with a scene query over the fixed scenery, every shape the straight line from the
/// transmitter to the receiver crosses costs comms:wallLoss. Without one, walls are not
/// counted. Shadowing: log-normal, correlated over comms:shadowingCorrelationDistance of the
/// receiver's travel (Gudmundson 1991), its own random stream per link. A CAN bus is taken as
/// fully loaded by the node (a frame after a frame), the worst case for its error counter.
///
/// With `ideal` it is the ideal substitute: every packet and frame delivered, no latency.
class Comms final : public IModule {
public:
    explicit Comms(bool ideal = false, std::shared_ptr<const services::SceneQuery> walls = nullptr)
        : ideal_(ideal), walls_(std::move(walls)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Walls crossed by the segment from `from` to `to`, by repeated rays (at most 64).
    [[nodiscard]] static int countWalls(const services::SceneQuery& query, const Vec3& from,
                                        const Vec3& to);

private:
    struct Placed {
        SignalId pose;  // invalid for a body that never moves
        Pose rest;
        Pose mount;
    };
    struct LinkState {
        Link link;
        Placed tx, rx;
        SignalId interference;
        SignalId rxPower, snr, per, latency, throughput, up;
        double shadow{0.0};  // dB
        Vec3 lastRx{};
        bool started{false};
    };
    struct BusState {
        CanBus bus;
        SignalId ber;
        SignalId fer, state, tec;
        CanFaultConfinement node;
        double pendingFrames{0.0};
    };
    [[nodiscard]] Pose worldPose(const Placed& p, const SignalBus& bus) const;
    void writeIdeal(SignalBus& bus) const;

    bool ideal_;
    std::shared_ptr<const services::SceneQuery> walls_;
    std::vector<LinkState> links_;
    std::vector<BusState> buses_;
};

}  // namespace lagrange::comms
