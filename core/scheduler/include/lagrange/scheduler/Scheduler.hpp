#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/module/ModuleConfig.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/module/Substitutes.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange {

/// A reason the scheduler refuses to start (docs/design/architecture.md §6). wire() collects
/// all of them rather than stopping at the first.
struct WiringError {
    enum class Kind : std::uint8_t {
        UnitMismatch,         // writer and reader units differ (dimension, scale or kind)
        TypeMismatch,         // type, array length, frame, or a hold the type cannot use
        MissingWriter,        // a required input that no module writes
        DuplicateWriter,      // two modules write the same signal
        Cycle,                // a loop with no input marked feedback
        ContractUnsatisfied,  // a module does not provide what its domain promises
        IncommensurateSteps,  // step <= 0, base tick < 1 µs, or hyperperiod > 10 s
        TierUnsupported,      // the module needs a higher capability tier than the run has
        NotDeterministic,     // a non-deterministic module in a deterministic run
        InvalidDeclaration,   // a malformed signal name or spec
        ModuleFailed,         // declare() or init() returned an error
    };
    Kind kind;
    std::string signal;  // "joint/left_wheel/torque", or "" when not about one signal
    std::string detail;  // "writer electrical/x: N·m (Torque); reader rigid_body/y: N (Force)"
};

[[nodiscard]] std::string_view to_string(WiringError::Kind kind);
/// One line for a startup error message: "UnitMismatch joint/a/torque: writer ...".
[[nodiscard]] std::string to_string(const WiringError& error);

/// Who writes a signal, for catalogs and tools.
struct SignalWriter {
    std::string_view label;  // "rigid_body/mujoco"
    Duration dt;
    bool substitute{false};
};

/// One wired input: who writes the signal, who reads it, with both declared units.
struct Connection {
    std::string writer;  // "rigid_body/mujoco"
    std::string reader;
    std::string signal;
    Unit writerUnit;
    Unit readerUnit;
    bool feedback{false};
};

/// Wall-clock cost of one module since the last takeCosts(), for the Performance workspace.
/// Timing is never part of the state, so it does not affect determinism.
struct ModuleCost {
    std::string label;
    std::string domain;
    Duration dt;
    std::uint64_t steps{0};
    std::int64_t totalNs{0};
    std::int64_t maxNs{0};
};

/// One module in the run, in execution order.
struct ScheduledModule {
    const IModule* module{nullptr};
    std::string label;  // "rigid_body/mujoco"
    Duration dt;
    std::uint32_t level{0};  // topological level: modules on one level do not read each other
    std::uint32_t order{0};  // position in the total order of a tick
    std::uint64_t seed{0};   // StepContext::seed
    /// Runs at the end of its step's window rather than at its start: it reads a flow from a
    /// faster module (architecture.md §8), so that the faster module has stepped through the
    /// window and the reader gets the average over its own step, not the one before.
    bool atWindowEnd{false};
};

struct SchedulerOptions {
    bool deterministic{false};  // refuse modules that are not deterministic
    std::uint64_t runSeed{0};   // profile `seed`
    /// Domain contracts to check at wiring; none are checked when null.
    const SubstituteRegistry* contracts{nullptr};
};

/// The multirate co-simulation master (invariant 4, RFC 0001).
///
/// Each module steps at its own dt; the base tick is their greatest common divisor. Within a
/// tick, due modules run in a fixed total order: topological over the signal graph (writers
/// before readers, feedback inputs excluded), ties broken by (dt, domain, name). Execution is
/// serial; the order is the same on every run, so results are deterministic when the modules
/// are.
class Scheduler {
public:
    explicit Scheduler(SchedulerOptions options = {});
    ~Scheduler();
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    Scheduler(Scheduler&&) = delete;
    Scheduler& operator=(Scheduler&&) = delete;

    /// Adds a module or substitute before wire().
    void add(std::unique_ptr<IModule> module, ModuleConfig config = {});

    /// Declares, checks and initialises every module. On failure nothing is initialised and
    /// every error found is returned.
    [[nodiscard]] Expected<void, std::vector<WiringError>> wire(const WorldView& world);

    [[nodiscard]] std::span<const ScheduledModule> plan() const { return plan_; }
    /// The module that writes an output signal; empty label for an invalid id.
    [[nodiscard]] SignalWriter writerOf(SignalId output) const;
    /// Every connected input after wire(), in declaration order. Unconnected optional inputs
    /// are not listed.
    [[nodiscard]] std::vector<Connection> connections() const;
    /// Step counts and wall time per module since the previous call, in execution order.
    [[nodiscard]] std::vector<ModuleCost> takeCosts();
    [[nodiscard]] Duration baseTick() const { return baseTick_; }
    [[nodiscard]] Duration hyperperiod() const { return hyperperiod_; }
    /// The wiring as a Graphviz digraph: modules, and one edge per connected signal
    /// (dashed for feedback inputs). Used by `lagrange graph`.
    [[nodiscard]] std::string dumpGraphDot() const;

    /// Advances by one base tick, stepping every module that is due.
    Status stepTick();
    /// Steps until now() >= t.
    Status stepUntil(SimTime t);
    [[nodiscard]] SimTime now() const { return now_; }
    /// Starts a freshly wired run at `t` instead of 0 (a whole number of base ticks), with every
    /// module's step count as if it had stepped since 0: a run rebuilt mid-way (robots spawned
    /// or deleted) keeps its clock. Only before the first step.
    Status startAt(SimTime t);
    [[nodiscard]] std::uint64_t tick() const { return tick_; }

    /// All modules' state, the bus and time. Fails if a module cannot save its state. The
    /// buffers that handle-typed signals (images, point clouds) refer to are saved with it, so a
    /// restored run reads its last image at once and publishes the same handles after.
    Status saveState(StateWriter& writer) const;
    Status restoreState(StateReader& reader);
    /// One module's state taken out of a run, to carry into a rebuilt one (robots spawned or
    /// deleted): `signature` is what it reads and writes, its step and its name.
    struct CarriedModule {
        std::string signature;
        std::vector<std::byte> state;
    };
    /// Every module's state by label; a module that cannot save its state is left out.
    [[nodiscard]] std::map<std::string, CarriedModule> carryModules() const;
    /// Restores the carried state into each module of this (freshly wired) run with the same
    /// label and signature, the same module reading and writing the same signals, and moves
    /// what it keeps by -shift when the run's region origin moved. A module of another
    /// signature starts afresh: its robots changed. Returns the labels restored.
    std::vector<std::string> restoreCarriedModules(
        const std::map<std::string, CarriedModule>& carried, const std::array<double, 3>& shift);
    /// Floating-origin rebase between ticks: moves the bus's region-local poses and every
    /// module's kept positions by -shift (IModule::shiftOrigin).
    void shiftOrigin(const std::array<double, 3>& shift);
    /// XXH3-64 of the saved state; equal hashes mean equal runs (determinism tests). A buffer's
    /// bytes are left out (its handle and sequence are in): a rendered image may differ by a bit
    /// on a GPU (invariant 7 holds for the CPU reference, not for a GPU's pixels).
    [[nodiscard]] Expected<std::uint64_t, Status> stateHash() const;

    [[nodiscard]] SignalBus& bus() { return bus_; }
    [[nodiscard]] const SignalBus& bus() const { return bus_; }

    /// Calls shutdown() on every initialised module; also done by the destructor.
    void shutdown();

private:
    Status saveStateImpl(StateWriter& writer, bool bufferBytes) const;
    [[nodiscard]] std::string signatureOf(std::uint32_t entry) const;
    struct Entry {
        std::unique_ptr<IModule> module;
        ModuleConfig config;
        ModuleInfo info;
        std::string label;
        Duration dt;
        std::vector<std::uint32_t> inputSlots;
        std::vector<std::uint32_t> outputSignals;
        std::uint64_t stepIndex{0};
        std::uint64_t seed{0};
        std::uint32_t level{0};
        bool atWindowEnd{false};  // ScheduledModule::atWindowEnd
        bool initialised{false};
        std::uint64_t costSteps{0};
        std::int64_t costTotalNs{0};
        std::int64_t costMaxNs{0};
        SplitStep* split{nullptr};           // the module as a SplitStep, if it is one
        std::vector<std::uint32_t> writers;  // entries whose outputs it reads (wire())
        SimTime pendingStart{};              // a split step in flight: its start, and the
        std::int64_t pendingNs{0};           // wall time its step() took
    };

    void checkSteps(std::vector<WiringError>& errors);
    void connect(std::vector<WiringError>& errors);
    void checkContracts(const WorldView& world, std::vector<WiringError>& errors);
    void order(std::vector<WiringError>& errors);
    /// Ends a split step in flight (SplitStep): finishStep() and the commit of its outputs.
    Status finishSplit(std::uint32_t index);
    void finishAllSplits();

    std::vector<std::uint32_t> inFlight_;  // entries whose split step has not finished

    SchedulerOptions options_;
    std::vector<Entry> entries_;        // in add() order; the bus refers to these indices
    std::vector<std::uint32_t> order_;  // indices into entries_, in execution order
    std::vector<ScheduledModule> plan_;
    SignalBus bus_;
    Duration baseTick_{};
    Duration hyperperiod_{};
    SimTime now_{};
    std::uint64_t tick_{0};
    bool wired_{false};
};

}  // namespace lagrange
