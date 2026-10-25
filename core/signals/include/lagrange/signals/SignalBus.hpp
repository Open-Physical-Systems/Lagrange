#pragma once

#include "lagrange/signals/Signal.hpp"
#include "lagrange/time/SimTime.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
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
        std::size_t offset{0};     // into inputValues_ or inputHandles_
        double windowWeight{0.0};  // ns of writer steps accumulated (WindowAverage)
    };

    // Called by the scheduler.
    std::uint32_t addSignal(SignalSpec spec, std::uint32_t writer);
    std::uint32_t addInput(SignalSpec spec, std::uint32_t reader);
    void allocate(std::uint32_t batchCount);
    void commit(std::uint32_t signal, SimTime stamp, std::uint64_t tick, Duration writerStep);
    void prepareInput(std::uint32_t slot, SimTime readerTime, std::uint64_t tick);

    /// Index of the value storage of `id`, or SIZE_MAX after flagging misuse.
    [[nodiscard]] std::size_t outputIndex(SignalId id, std::uint32_t env, bool forWrite) const;
    [[nodiscard]] std::size_t inputIndex(SignalId id, std::uint32_t env) const;
    void flagMisuse(std::string message) const;

    std::uint32_t batchCount_{1};
    std::vector<SignalRecord> signals_;
    std::vector<InputSlot> inputs_;
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
};

}  // namespace lagrange
