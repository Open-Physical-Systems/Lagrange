#pragma once

// The EMI domain (docs/design/domains/emi.md): disturbance fields from sources in the scene and
// what they do to the robot's sensors, buses and radios. Behaviourally correct interference for
// testing robot software, not an electromagnetic compatibility tool.

#include "lagrange/fields/Field.hpp"
#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::emi {

using Vec3 = std::array<double, 3>;

/// Vacuum permeability μ0, N·A⁻² (CODATA 2018).
inline constexpr double kMu0 = 1.25663706212e-6;
/// The impedance of free space as 120π Ω (the far-field approximation behind E = √(30·P)/d;
/// CODATA 2018's μ0·c is 376.730 Ω, 0.07 % less).
inline constexpr double kFreeSpaceImpedance = 376.99111843077515;
/// Distances below this are taken as this (m): the analytic fields diverge at a conductor or
/// dipole, and no victim sits inside one.
inline constexpr double kMinDistance = 0.01;

/// B of an infinite straight conductor through `point` along the unit vector `direction`,
/// carrying `current` (A) in that direction, at `at`: μ0·I/(2π·d) along direction × r̂ (the
/// right-hand rule), with d the distance from the conductor. T.
[[nodiscard]] Vec3 lineField(const Vec3& point, const Vec3& direction, double current,
                             const Vec3& at);
/// B of a magnetic dipole of moment `moment` (A·m²) at `centre`, at `at`:
/// μ0/(4π)·(3·r̂(m·r̂) − m)/r³. T.
[[nodiscard]] Vec3 dipoleField(const Vec3& centre, const Vec3& moment, const Vec3& at);
/// RMS E of an isotropic radiator in free space, far field: √(30·EIRP)/d. V/m.
[[nodiscard]] double farField(double eirp, double distance);

/// Encoder: the probability that an edge is corrupted, 1/(1 + (E50/|E|)⁴) (behavioural).
[[nodiscard]] double glitchProbability(double field, double field50);
/// CAN: the bit-error rate BER0·(|E|/E_ref)², at most 0.5 (behavioural).
[[nodiscard]] double bitErrorRate(double field, double referenceField, double rateAtReference);
/// The fraction of a source's band [f − b/2, f + b/2] inside a receiver's band; a zero-width
/// band is a single frequency (1 inside, 0 outside).
[[nodiscard]] double bandOverlap(double sourceFrequency, double sourceBandwidth,
                                 double receiverFrequency, double receiverBandwidth);
/// The power an isotropic antenna receives from a field of RMS `field` at `frequency`:
/// S·λ²/(4π) with S = E²/Z0. W.
[[nodiscard]] double receivedPower(double field, double frequency);

/// One EmiSourceAPI_1 prim, read once at load.
struct Source {
    enum class Type { PowerLine, MotorDrive, Transmitter, Welder, Custom };
    std::string name;  // the prim's instance name
    Type type{Type::Custom};
    BodyId body{kWorldBody};  // where it is (World::Prims::body, poseInBody)
    Pose poseInBody;
    double frequency{0.0};          // Hz; 0 for a direct current
    double current{0.0};            // A, RMS; for a field map, the current it was made at
    double radiatedPower{0.0};      // W, EIRP
    double bandwidth{0.0};          // Hz
    double dutyCycle{1.0};          // the fraction of time on (an RMS weight)
    std::vector<double> harmonics;  // relative amplitudes of harmonics 2, 3, ...
    Vec3 direction{1.0, 0.0, 0.0};  // conductor or dipole axis, in the prim's frame (unit)
    double momentPerAmpere{0.0};    // m²
    std::shared_ptr<const fields::Field> fieldMap;  // in the prim's frame
    bool mapIsB{true};
    std::string drivenBy;  // a motor whose current scales it at runtime, or empty
};

/// The sources of `world`; an error for a missing or invalid required value (power lines need
/// their frequency, a site parameter without a default, and current).
[[nodiscard]] Expected<std::vector<Source>, std::string> sourcesOf(const WorldView& world);

/// One source as it is at one instant: its world pose and its current (A, RMS).
struct SourceState {
    Pose pose;
    double current{0.0};
};

/// The source's instantaneous B at `at` (region-local, T) at time `t` (s): the spatial shape
/// at RMS current times √2·(sin ωt + Σ h_k·sin kωt) for an alternating source, the shape for a
/// direct one; duty-weighted.
[[nodiscard]] Vec3 magneticField(const Source& s, const SourceState& state, const Vec3& at,
                                 double t);
/// The RMS of |B| over a cycle at `at`, T, and the RMS of dB/dt's magnitude over 2π (for the
/// induced voltage of a loop), T·Hz.
struct FieldRms {
    double b{0.0};
    double bTimesFrequency{0.0};
};
[[nodiscard]] FieldRms magneticFieldRms(const Source& s, const SourceState& state, const Vec3& at);
/// The source's RMS E at `at`, V/m (radiated power only; low-frequency E is not modelled).
[[nodiscard]] double electricFieldRms(const Source& s, const SourceState& state, const Vec3& at);

/// One EmiSusceptibilityAPI_1 prim.
struct Victim {
    enum class Type { Magnetometer, ImuAdc, Encoder, CanBus, Radio };
    std::string name;
    Type type{Type::Magnetometer};
    PrimId prim;
    BodyId body{kWorldBody};
    Pose poseInBody;
    double magneticGain{1.0};
    double accelNoisePerField{0.0};  // (m/s²)/(V/m)
    double gyroNoisePerField{0.0};   // (rad/s)/(V/m)
    double glitchField{0.0};         // V/m; 0: no glitches
    double canErrorField{0.0};       // V/m; 0: no bit errors
    double canErrorRate{1e-5};
    double loopArea{0.0};     // m²
    double shielding{1.0};    // at least 1
    double rxFrequency{0.0};  // Hz, a radio's band (RadioLinkAPI_1 on the same prim)
    double rxBandwidth{0.0};
};

[[nodiscard]] Expected<std::vector<Victim>, std::string> victimsOf(const WorldView& world);
/// The coupling name of a victim type in profiles (emi.coupling): magnetometer, imu, encoder,
/// can_bus, wifi.
[[nodiscard]] const char* couplingName(Victim::Type type);

/// The EMI domain at 1 ms (emi.md §4). For every victim p it writes emi/<p>/b_field (T, the
/// victim's frame, gain and shielding applied), emi/<p>/e_field_rms (V/m) and, by type,
/// emi/<p>/accel_noise and gyro_noise (IMU, 1 σ), glitch_probability (encoder), bit_error_rate
/// (CAN), interference_power (radio, W); emi/<p>/induced_voltage (V, RMS) for a victim with a
/// loop area. Victims whose coupling the config's "coupling" list leaves out read zero.
///
/// Inputs: the poses of moving bodies that carry sources or victims, and motor/<m>/current_q of
/// each motor a source is driven by (optional: without the electrical network, the source's own
/// emi:current). With `zero` it is the zero substitute: the same outputs, all zero.
class Emi final : public IModule {
public:
    explicit Emi(bool zero = false) : zero_(zero) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    struct Placed {
        SignalId pose;  // invalid for a body that never moves
        Pose rest;      // the body's (or the world's) rest pose
        Pose mount;
    };
    struct VictimOut {
        Victim v;
        Placed at;
        bool coupled{true};
        SignalId b, e, accel, gyro, glitch, ber, interference, induced;
    };
    [[nodiscard]] Pose worldPose(const Placed& p, const SignalBus& bus) const;
    void writeZero(SignalBus& bus) const;

    bool zero_;
    std::vector<Source> sources_;
    std::vector<Placed> sourceAt_;
    std::vector<SignalId> sourceCurrent_;  // invalid: the source's own current
    std::vector<SourceState> states_;      // per step, kept to avoid allocating
    std::vector<VictimOut> victims_;
};

}  // namespace lagrange::emi
