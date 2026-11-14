#pragma once

#include "lagrange/signals/Signal.hpp"
#include "lagrange/time/SimTime.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace lagrange {

class Scheduler;
class SignalDeclarer;

/// What a handle-typed signal (Image, PointCloud, Bytes, Field3D) refers to: bytes in host
/// memory with their shape. Immutable once published, so readers share it without copying.
struct Buffer {
    std::uint32_t width{0};   // pixels or elements per row
    std::uint32_t height{0};  // rows; 1 for one-dimensional data
    std::string encoding;     // "rgba8", "r32f" (one float per pixel), "r32u", "bytes"
    std::vector<std::byte> bytes;
};

/// The typed signal bus (docs/design/architecture.md §4).
///
/// Every signal has one writer. Writers write into a pending copy; when a module's step ends the
/// scheduler commits its outputs, which become visible to modules stepped later. Readers do not
/// read signals directly: each input is a slot that the scheduler fills before the reader steps,
/// applying the input's hold policy (zero-order, linear extrapolation, window average) or its
/// one-step feedback delay. So `read` on an input id costs one array access and never depends on
/// how fast the writer runs.
///
/// Values are stored per environment of a batch: index `[env * components + c]`.
class SignalBus {
public:
    // --- hot path, called from IModule::step ---------------------------------------------

    /// First component of an input (the held value) or an output (its last committed value).
    [[nodiscard]] double read(SignalId id, std::uint32_t env = 0) const;
    /// All components: Vec3, Quat, Pose, Wrench, ScalarArray.
    [[nodiscard]] std::span<const double> readN(SignalId id, std::uint32_t env = 0) const;
    /// Writes an output owned by the module being stepped.
    void write(SignalId id, double value, std::uint32_t env = 0);
    void writeN(SignalId id, std::span<const double> values, std::uint32_t env = 0);
    /// This step has no new value of an output of the module being stepped: it does not commit,
    /// so its stamp and readers stay as they are. For outputs that come irregularly (a delay line).
    void hold(SignalId id);
    /// Whether hold() was called on an output in this step: it will not commit. For wrappers that
    /// post-process the outputs of the module they step (sensors::withFaults).
    [[nodiscard]] bool held(SignalId id) const {
        return !id.isInput() && id.valid() && id.index < held_.size() && held_[id.index] != 0;
    }

    /// Whether an input has a writer: an optional input no module writes reads its default.
    /// Valid from init() on (after wiring); false for an output or an invalid id.
    [[nodiscard]] bool connected(SignalId input) const {
        return input.isInput() && input.slot() < inputs_.size() &&
               inputs_[input.slot()].signal != kNoModule;
    }

    [[nodiscard]] BufferHandle readBuffer(SignalId id, std::uint32_t env = 0) const;
    void writeBuffer(SignalId id, BufferHandle handle, std::uint32_t env = 0);
    /// Publishes `buffer` as the new value of a handle-typed output: writes a fresh handle and
    /// keeps the buffer while a reader can still hold that handle (pending, committed and
    /// previous values; publish at most once per step). Handles are numbered per signal in
    /// publishing order, so a deterministic run publishes the same handles.
    BufferHandle publishBuffer(SignalId id, std::shared_ptr<const Buffer> buffer,
                               std::uint32_t env = 0);
    /// The buffer a handle refers to; null for the empty handle and for one no longer kept.
    [[nodiscard]] std::shared_ptr<const Buffer> buffer(BufferHandle handle) const;

    /// Whole-batch access for vectorised modules: [env][component], row-major.
    [[nodiscard]] std::span<const double> readBatch(SignalId id) const;
    [[nodiscard]] std::span<double> writeBatch(SignalId id);

    /// Time of the value: for an output, the end of the writer step that committed it.
    [[nodiscard]] SimTime stamp(SignalId id) const;
    [[nodiscard]] std::uint32_t batchCount() const { return batchCount_; }

    // --- cold path: tools, recording, the protocol frontend ------------------------------

    /// The output id of a signal by name; invalid if there is none.
    [[nodiscard]] SignalId find(std::string_view name) const;
    [[nodiscard]] const SignalSpec& spec(SignalId id) const;
    [[nodiscard]] std::size_t signalCount() const { return signals_.size(); }
    /// Spec of the i-th signal; SignalId::output(i) addresses it.
    [[nodiscard]] const SignalSpec& specAt(std::size_t index) const {
        return signals_.at(index).spec;
    }

private:
    friend class Scheduler;
    friend class SignalDeclarer;

    static constexpr std::uint32_t kNoModule = 0xFFFF'FFFFU;

    struct SignalRecord {
        SignalSpec spec;
        std::size_t components{0};  // doubles per environment; 0 for handle types
        std::size_t offset{0};      // into the value arrays, or the handle arrays
        std::uint32_t writer{kNoModule};
    };
    /// A signal's commits, kept apart from its record: every commit writes them.
    struct Timing {
        SimTime stamp{};
        SimTime previousStamp{};
        std::uint64_t commitTick{~0ULL};  // tick of the last commit
        std::uint32_t commits{0};         // saturates at 2: enough to know if extrapolation works
    };

    struct InputSlot {
        SignalSpec spec;  // as declared by the reader
        std::uint32_t reader{kNoModule};
        std::uint32_t signal{kNoModule};  // kNoModule while unconnected (optional inputs)
        std::size_t components{0};
        std::size_t offset{0};       // into inputValues_ or inputHandles_
        double windowWeight{0.0};    // ns of writer steps accumulated (WindowAverage)
        Hold hold{Hold::ZeroOrder};  // spec.hold and spec.feedback, copied by allocate()
        bool feedback{false};
    };

    // Called by the scheduler.
    std::uint32_t addSignal(SignalSpec spec, std::uint32_t writer);
    /// Floating-origin rebase: moves every region-local Pose value (outputs, held inputs and
    /// window sums) by -shift, so that they are in the new region frame.
    void shiftOrigin(const std::array<double, 3>& shift);
    std::uint32_t addInput(SignalSpec spec, std::uint32_t reader);
    void allocate(std::uint32_t batchCount);
    void commit(std::uint32_t signal, SimTime stamp, std::uint64_t tick, Duration writerStep);
    /// commit() of each of `signals` (one module's outputs), with neighbours in storage copied
    /// together: the same result, fewer calls for a module with hundreds of outputs.
    void commitAll(std::span<const std::uint32_t> signals, SimTime stamp, std::uint64_t tick,
                   Duration writerStep);
    void prepareInput(std::uint32_t slot, SimTime readerTime, std::uint64_t tick);
    /// prepareInput() of each of `slots` (one module's inputs).
    void prepareInputs(std::span<const std::uint32_t> slots, SimTime readerTime,
                       std::uint64_t tick);

    /// Index of the value storage of `id`, or SIZE_MAX after flagging misuse.
    [[nodiscard]] std::size_t outputIndex(SignalId id, std::uint32_t env, bool forWrite) const;
    [[nodiscard]] std::size_t inputIndex(SignalId id, std::uint32_t env) const;
    void flagMisuse(std::string message) const;

    std::uint32_t batchCount_{1};
    std::vector<SignalRecord> signals_;
    std::vector<InputSlot> inputs_;
    std::vector<std::uint8_t> held_;  // per signal: hold() was called, until its commit
    std::vector<double> committed_, previous_, pending_;  // numeric outputs
    std::vector<std::uint64_t> committedHandles_, previousHandles_, pendingHandles_;
    struct KeptBuffer {
        std::uint64_t handle{0};
        std::shared_ptr<const Buffer> buffer;
    };
    static constexpr std::size_t kKeptBuffers = 4;  // pending, committed, previous, and a spare
    std::vector<std::array<KeptBuffer, kKeptBuffers>> kept_;  // per handle slot, by sequence
    std::vector<std::uint32_t> published_;                    // per handle slot: last sequence
    std::vector<double> inputValues_;                         // held input values
    std::vector<std::uint64_t> inputHandles_;
    std::vector<double> windowSums_;                         // per input component (WindowAverage)
    std::vector<std::vector<std::uint32_t>> windowReaders_;  // per signal: its WindowAverage slots
    std::uint32_t currentModule_{kNoModule};                 // the module being stepped
    mutable std::string misuse_;                             // first misuse in the current step
    // (Members added in 1.2.x stay last, so that the inline accessors above keep their layout.)
    std::vector<Timing> timing_;  // per signal
    // Per signal, what commitAll reads (allocate()): kept apart from the records, so that a
    // commit of a module's outputs reads 12 bytes a signal instead of a cache line each.
    struct CommitInfo {
        std::uint32_t offset{0};  // into the value arrays (SignalRecord::offset; 2³² is 32 GB)
        std::uint32_t count{0};   // values: components × batch; 0 for handle types
        std::uint8_t plain{0};    // values, read by no window average: commits as a copy
        // A reader uses its previous value (feedback, extrapolation). previous_ of the others is
        // not kept up to date by commitAll (nothing reads it).
        std::uint8_t needsPrevious{0};
    };
    std::vector<CommitInfo> commitInfo_;
    // Per input slot: the commit tick of the value it holds when that is a plain copy of the
    // committed value (zero-order hold), else kStale. A slot whose writer has not committed
    // since is not copied again: in the town MuJoCo reads 655 pose commands every 1 ms that
    // the traffic and the people write every 10 ms. Stale after a restore and an origin shift.
    static constexpr std::uint64_t kStale = ~0ULL - 1;
    std::vector<std::uint64_t> preparedTick_;
    std::vector<std::uint32_t> slotSignal_;  // per input slot: its signal (InputSlot::signal)
    // Per signal: written since its last commit, so its pending value may differ from the
    // committed one. A commit copies only these (the others are equal already); a writer that
    // leaves an unchanged value alone costs no copy. All set after allocate, a restore and an
    // origin shift.
    std::vector<std::uint8_t> dirty_;
    void markAllDirty() { std::fill(dirty_.begin(), dirty_.end(), std::uint8_t{1}); }
    void forgetPrepared() { std::fill(preparedTick_.begin(), preparedTick_.end(), kStale); }
    // Per writing module (by index): its outputs when they are plain values with consecutive ids,
    // one stretch of storage [valueFrom, valueTo); count 0 when they are not.
    struct DensePlan {
        std::uint32_t first{0};  // its first signal
        std::uint32_t count{0};  // its signals
        std::uint32_t valueFrom{0}, valueTo{0};
        std::uint32_t previousBegin{0}, previousEnd{0};  // into denseRanges_
        bool dense{true};
    };
    std::vector<DensePlan> densePlans_;
    // Stretches of values whose previous value is read, [from, to), per DensePlan.
    std::vector<std::pair<std::uint32_t, std::uint32_t>> denseRanges_;
    /// commitAll() of a module with a DensePlan that wrote every output and held none: one copy.
    /// False (nothing done) otherwise.
    bool commitDense(std::span<const std::uint32_t> signals, SimTime stamp, std::uint64_t tick);
};

}  // namespace lagrange
