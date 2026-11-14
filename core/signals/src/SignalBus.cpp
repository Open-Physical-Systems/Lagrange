#include "lagrange/signals/SignalBus.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

namespace lagrange {

namespace {

constexpr std::size_t kBad = std::numeric_limits<std::size_t>::max();
const std::vector<double> kNothing;

}  // namespace

std::size_t componentCount(SignalType type, std::uint32_t arrayLength) {
    switch (type) {
        case SignalType::Scalar:
        case SignalType::Bool:
        case SignalType::Int:
            return 1;
        case SignalType::Vec3:
            return 3;
        case SignalType::Quat:
            return 4;
        case SignalType::Wrench:
            return 6;
        case SignalType::Pose:
            return 7;
        case SignalType::ScalarArray:
            return arrayLength;
        case SignalType::Bytes:
        case SignalType::Field3D:
        case SignalType::Image:
        case SignalType::PointCloud:
            return 0;
    }
    return 0;
}

bool isHandle(SignalType type) {
    return componentCount(type, 1) == 0;
}

std::string_view to_string(SignalType type) {
    switch (type) {
        case SignalType::Scalar:
            return "Scalar";
        case SignalType::Vec3:
            return "Vec3";
        case SignalType::Quat:
            return "Quat";
        case SignalType::Pose:
            return "Pose";
        case SignalType::Wrench:
            return "Wrench";
        case SignalType::ScalarArray:
            return "ScalarArray";
        case SignalType::Bool:
            return "Bool";
        case SignalType::Int:
            return "Int";
        case SignalType::Bytes:
            return "Bytes";
        case SignalType::Field3D:
            return "Field3D";
        case SignalType::Image:
            return "Image";
        case SignalType::PointCloud:
            return "PointCloud";
    }
    return "Unknown";
}

std::string_view to_string(Hold hold) {
    switch (hold) {
        case Hold::ZeroOrder:
            return "ZeroOrder";
        case Hold::LinearExtrapolate:
            return "LinearExtrapolate";
        case Hold::WindowAverage:
            return "WindowAverage";
    }
    return "Unknown";
}

namespace {

/// Bytes of the UTF-8 sequence starting at `text[i]`; 0 if it is not a whole, valid one.
std::size_t utf8Length(std::string_view text, std::size_t i) {
    const auto lead = static_cast<unsigned char>(text[i]);
    const std::size_t n = lead < 0x80U              ? 1
                          : (lead & 0xE0U) == 0xC0U ? 2
                          : (lead & 0xF0U) == 0xE0U ? 3
                          : (lead & 0xF8U) == 0xF0U ? 4
                                                    : 0;
    if (n == 0 || i + n > text.size()) {
        return 0;
    }
    for (std::size_t k = 1; k < n; ++k) {
        if ((static_cast<unsigned char>(text[i + k]) & 0xC0U) != 0x80U) {
            return 0;
        }
    }
    return n;
}

/// `text` with every byte that is not part of valid UTF-8 written as \xNN, so that a message
/// quoting it is UTF-8 (Python decodes it).
std::string printable(std::string_view text) {
    std::string out;
    for (std::size_t i = 0; i < text.size();) {
        const std::size_t n = utf8Length(text, i);
        if (n == 0) {
            static constexpr char kHex[] = "0123456789abcdef";
            const auto byte = static_cast<unsigned char>(text[i]);
            out += "\\x";
            out += kHex[byte >> 4U];
            out += kHex[byte & 0xFU];
            ++i;
        } else {
            out.append(text.substr(i, n));
            i += n;
        }
    }
    return out;
}

}  // namespace

std::string checkSignalName(std::string_view name) {
    if (name.empty()) {
        return "empty name";
    }
    std::size_t segmentStart = 0;
    for (std::size_t i = 0; i <= name.size(); ++i) {
        if (i == name.size() || name[i] == '/') {
            if (i == segmentStart) {
                return "empty segment in \"" + std::string(name) + "\"";
            }
            segmentStart = i + 1;
            continue;
        }
        const char c = name[i];
        const bool allowed = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                             (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
        if (!allowed) {
            // The whole character, not one byte of a multi-byte one: the message stays UTF-8
            // (QA 2026-11-12 #14: Python raised UnicodeDecodeError instead of the error).
            const std::size_t n = std::max<std::size_t>(utf8Length(name, i), 1);
            return "character '" + printable(name.substr(i, n)) + "' not allowed in \"" +
                   printable(name) + "\" (use [A-Za-z0-9_.-] and '/')";
        }
    }
    return {};
}

// --- hot path ------------------------------------------------------------------------------

std::size_t SignalBus::outputIndex(SignalId id, std::uint32_t env, bool forWrite) const {
    if (!id.valid() || id.isInput() || id.index >= signals_.size() || env >= batchCount_) {
        flagMisuse("invalid output id or environment");
        return kBad;
    }
    const SignalRecord& record = signals_[id.index];
    if (forWrite && record.writer != currentModule_) {
        flagMisuse("write to \"" + record.spec.name + "\", which this module does not output");
        return kBad;
    }
    const std::size_t width = std::max<std::size_t>(record.components, 1);
    return record.offset + env * width;
}

std::size_t SignalBus::inputIndex(SignalId id, std::uint32_t env) const {
    if (!id.isInput() || id.slot() >= inputs_.size() || env >= batchCount_) {
        flagMisuse("invalid input id or environment");
        return kBad;
    }
    const InputSlot& slot = inputs_[id.slot()];
    if (currentModule_ != kNoModule && slot.reader != currentModule_) {
        flagMisuse("read of input \"" + slot.spec.name + "\" declared by another module");
        return kBad;
    }
    const std::size_t width = std::max<std::size_t>(slot.components, 1);
    return slot.offset + env * width;
}

void SignalBus::flagMisuse(std::string message) const {
    if (misuse_.empty()) {
        misuse_ = std::move(message);
    }
}

double SignalBus::read(SignalId id, std::uint32_t env) const {
    const std::span<const double> values = readN(id, env);
    return values.empty() ? 0.0 : values.front();
}

std::span<const double> SignalBus::readN(SignalId id, std::uint32_t env) const {
    if (id.isInput()) {
        const std::size_t index = inputIndex(id, env);
        if (index == kBad || inputs_[id.slot()].components == 0) {
            return kNothing;
        }
        return {inputValues_.data() + index, inputs_[id.slot()].components};
    }
    const std::size_t index = outputIndex(id, env, false);
    if (index == kBad || signals_[id.index].components == 0) {
        return kNothing;
    }
    return {committed_.data() + index, signals_[id.index].components};
}

void SignalBus::write(SignalId id, double value, std::uint32_t env) {
    writeN(id, std::span<const double>(&value, 1), env);
}

void SignalBus::writeN(SignalId id, std::span<const double> values, std::uint32_t env) {
    const std::size_t index = outputIndex(id, env, true);
    if (index == kBad) {
        return;
    }
    const std::size_t components = signals_[id.index].components;
    if (values.size() > components) {
        flagMisuse("write of " + std::to_string(values.size()) + " values to \"" +
                   signals_[id.index].spec.name + "\", which has " + std::to_string(components));
        return;
    }
    std::copy(values.begin(), values.end(), pending_.begin() + static_cast<std::ptrdiff_t>(index));
    dirty_[id.index] = 1;
}

BufferHandle SignalBus::readBuffer(SignalId id, std::uint32_t env) const {
    if (id.isInput()) {
        const std::size_t index = inputIndex(id, env);
        return index == kBad || inputs_[id.slot()].components != 0
                   ? BufferHandle{}
                   : BufferHandle{inputHandles_[index]};
    }
    const std::size_t index = outputIndex(id, env, false);
    return index == kBad || signals_[id.index].components != 0
               ? BufferHandle{}
               : BufferHandle{committedHandles_[index]};
}

void SignalBus::writeBuffer(SignalId id, BufferHandle handle, std::uint32_t env) {
    const std::size_t index = outputIndex(id, env, true);
    if (index == kBad) {
        return;
    }
    if (signals_[id.index].components != 0) {
        flagMisuse("buffer write to numeric signal \"" + signals_[id.index].spec.name + "\"");
        return;
    }
    pendingHandles_[index] = handle.id;
}

BufferHandle SignalBus::publishBuffer(SignalId id, std::shared_ptr<const Buffer> buffer,
                                      std::uint32_t env) {
    const std::size_t index = outputIndex(id, env, true);
    if (index == kBad) {
        return {};
    }
    if (signals_[id.index].components != 0) {
        flagMisuse("buffer write to numeric signal \"" + signals_[id.index].spec.name + "\"");
        return {};
    }
    // Slot in the high half, sequence in the low half; 0 stays the empty handle.
    const std::uint32_t sequence = ++published_[index];
    const BufferHandle handle{(std::uint64_t{index} + 1) << 32U | sequence};
    kept_[index][sequence % kKeptBuffers] = {handle.id, std::move(buffer)};
    pendingHandles_[index] = handle.id;
    return handle;
}

std::shared_ptr<const Buffer> SignalBus::buffer(BufferHandle handle) const {
    const std::uint64_t slot = handle.id >> 32U;
    if (slot == 0 || slot > kept_.size()) {
        return nullptr;
    }
    const KeptBuffer& kept = kept_[slot - 1][(handle.id & 0xFFFF'FFFFU) % kKeptBuffers];
    return kept.handle == handle.id ? kept.buffer : nullptr;
}

std::span<const double> SignalBus::readBatch(SignalId id) const {
    const std::size_t components =
        id.isInput()
            ? (id.slot() < inputs_.size() ? inputs_[id.slot()].components : 0)
            : (id.valid() && id.index < signals_.size() ? signals_[id.index].components : 0);
    const std::span<const double> first = readN(id, 0);
    if (first.empty()) {
        return kNothing;
    }
    return {first.data(), components * batchCount_};
}

std::span<double> SignalBus::writeBatch(SignalId id) {
    const std::size_t index = outputIndex(id, 0, true);
    if (index == kBad || signals_[id.index].components == 0) {
        return {};
    }
    dirty_[id.index] = 1;  // the caller writes through the span
    return {pending_.data() + index, signals_[id.index].components * batchCount_};
}

void SignalBus::hold(SignalId id) {
    if (!id.isInput() && id.valid() && id.index < held_.size()) {
        held_[id.index] = 1;
    }
}

SimTime SignalBus::stamp(SignalId id) const {
    if (id.isInput()) {
        if (id.slot() >= inputs_.size() || inputs_[id.slot()].signal == kNoModule) {
            return {};
        }
        return timing_[inputs_[id.slot()].signal].stamp;
    }
    return id.valid() && id.index < signals_.size() ? timing_[id.index].stamp : SimTime{};
}

// --- cold path -----------------------------------------------------------------------------

SignalId SignalBus::find(std::string_view name) const {
    for (std::size_t i = 0; i < signals_.size(); ++i) {
        if (signals_[i].spec.name == name) {
            return SignalId::output(static_cast<std::uint32_t>(i));
        }
    }
    return {};
}

const SignalSpec& SignalBus::spec(SignalId id) const {
    if (id.isInput()) {
        return inputs_.at(id.slot()).spec;
    }
    return signals_.at(id.index).spec;
}

// --- scheduler side ------------------------------------------------------------------------

std::uint32_t SignalBus::addSignal(SignalSpec spec, std::uint32_t writer) {
    SignalRecord record;
    record.components = componentCount(spec.type, spec.arrayLength);
    record.spec = std::move(spec);
    record.writer = writer;
    signals_.push_back(std::move(record));
    timing_.emplace_back();
    return static_cast<std::uint32_t>(signals_.size() - 1);
}

std::uint32_t SignalBus::addInput(SignalSpec spec, std::uint32_t reader) {
    InputSlot slot;
    slot.components = componentCount(spec.type, spec.arrayLength);
    slot.spec = std::move(spec);
    slot.reader = reader;
    inputs_.push_back(std::move(slot));
    return static_cast<std::uint32_t>(inputs_.size() - 1);
}

namespace {

// A Pose value is a position in the region-local frame unless its spec names a body frame.
bool regionPose(const SignalSpec& spec) {
    return spec.type == SignalType::Pose && (spec.frame.empty() || spec.frame == "world_local");
}

}  // namespace

void SignalBus::shiftOrigin(const std::array<double, 3>& shift) {
    for (const SignalRecord& record : signals_) {
        if (!regionPose(record.spec)) {
            continue;
        }
        for (std::uint32_t env = 0; env < batchCount_; ++env) {
            const std::size_t at = record.offset + env * record.components;
            for (std::size_t i = 0; i < 3; ++i) {
                committed_[at + i] -= shift.at(i);
                previous_[at + i] -= shift.at(i);
                pending_[at + i] -= shift.at(i);
            }
        }
    }
    forgetPrepared();
    markAllDirty();
    for (const InputSlot& slot : inputs_) {
        if (!regionPose(slot.spec) || slot.components == 0) {
            continue;
        }
        for (std::uint32_t env = 0; env < batchCount_; ++env) {
            const std::size_t at = slot.offset + env * slot.components;
            for (std::size_t i = 0; i < 3; ++i) {
                inputValues_[at + i] -= shift.at(i);
                windowSums_[at + i] -= shift.at(i) * slot.windowWeight;
            }
        }
    }
}

void SignalBus::allocate(std::uint32_t batchCount) {
    batchCount_ = std::max<std::uint32_t>(batchCount, 1);
    held_.assign(signals_.size(), 0);
    std::size_t values = 0;
    std::size_t handles = 0;
    for (SignalRecord& record : signals_) {
        if (record.components == 0) {
            record.offset = handles;
            handles += batchCount_;
        } else {
            record.offset = values;
            values += record.components * batchCount_;
        }
    }
    committed_.assign(values, 0.0);
    dirty_.assign(signals_.size(), 1);
    previous_.assign(values, 0.0);
    pending_.assign(values, 0.0);
    committedHandles_.assign(handles, 0);
    previousHandles_.assign(handles, 0);
    pendingHandles_.assign(handles, 0);
    kept_.assign(handles, {});
    published_.assign(handles, 0);

    std::size_t inputValues = 0;
    std::size_t inputHandles = 0;
    for (InputSlot& slot : inputs_) {
        if (slot.components == 0) {
            slot.offset = inputHandles;
            inputHandles += batchCount_;
        } else {
            slot.offset = inputValues;
            inputValues += slot.components * batchCount_;
        }
    }
    inputValues_.assign(inputValues, 0.0);
    preparedTick_.assign(inputs_.size(), kStale);
    slotSignal_.resize(inputs_.size());
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        slotSignal_[i] = inputs_[i].signal;
    }
    inputHandles_.assign(inputHandles, 0);
    windowSums_.assign(inputValues, 0.0);
    windowReaders_.assign(signals_.size(), {});
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        if (inputs_[i].signal != kNoModule && inputs_[i].spec.hold == Hold::WindowAverage &&
            !inputs_[i].spec.feedback) {
            windowReaders_[inputs_[i].signal].push_back(static_cast<std::uint32_t>(i));
        }
    }
    for (InputSlot& slot : inputs_) {
        slot.hold = slot.spec.hold;
        slot.feedback = slot.spec.feedback;
    }
    // What commitAll needs of each signal, in one small record: where its values are, whether
    // it commits as a plain copy, and whether a reader uses its previous value (a feedback
    // input, an extrapolation; previous_ of the others is not kept up to date).
    commitInfo_.assign(signals_.size(), {});
    for (std::size_t i = 0; i < signals_.size(); ++i) {
        const SignalRecord& record = signals_[i];
        CommitInfo& info = commitInfo_[i];
        info.offset = static_cast<std::uint32_t>(record.offset);
        info.count = static_cast<std::uint32_t>(record.components * batchCount_);
        info.plain = record.components != 0 && windowReaders_[i].empty() ? 1 : 0;
    }
    for (const InputSlot& slot : inputs_) {
        if (slot.signal != kNoModule &&
            (slot.spec.feedback || slot.spec.hold == Hold::LinearExtrapolate)) {
            commitInfo_[slot.signal].needsPrevious = 1;
        }
    }
    // Per writing module: whether its outputs are plain values with consecutive ids (and so
    // one stretch of storage), for commitDense; with the stretches of those whose previous
    // value is read, merged.
    densePlans_.clear();
    denseRanges_.clear();
    for (std::uint32_t i = 0; i < signals_.size(); ++i) {
        const std::uint32_t writer = signals_[i].writer;
        if (writer == kNoModule) {
            continue;
        }
        if (writer >= densePlans_.size()) {
            densePlans_.resize(writer + 1);
        }
        DensePlan& plan = densePlans_[writer];
        if (plan.count == 0) {
            plan.first = i;
            plan.valueFrom = commitInfo_[i].offset;
            plan.valueTo = plan.valueFrom;
        }
        if (plan.first + plan.count != i || commitInfo_[i].plain == 0 ||
            commitInfo_[i].offset != plan.valueTo) {
            plan.dense = false;
        }
        ++plan.count;
        plan.valueTo = commitInfo_[i].offset + commitInfo_[i].count;
    }
    for (DensePlan& plan : densePlans_) {
        if (!plan.dense) {
            plan = DensePlan{};  // count 0: never dense
            continue;
        }
        plan.previousBegin = static_cast<std::uint32_t>(denseRanges_.size());
        for (std::uint32_t i = plan.first; i < plan.first + plan.count; ++i) {
            const CommitInfo& info = commitInfo_[i];
            if (info.needsPrevious == 0) {
                continue;
            }
            if (denseRanges_.size() > plan.previousBegin &&
                denseRanges_.back().second == info.offset) {
                denseRanges_.back().second = info.offset + info.count;
            } else {
                denseRanges_.emplace_back(info.offset, info.offset + info.count);
            }
        }
        plan.previousEnd = static_cast<std::uint32_t>(denseRanges_.size());
    }
    // Unconnected optional inputs keep their default for the whole run.
    for (const InputSlot& slot : inputs_) {
        if (slot.signal == kNoModule && slot.components != 0) {
            std::fill_n(inputValues_.begin() + static_cast<std::ptrdiff_t>(slot.offset),
                        slot.components * batchCount_, slot.spec.defaultValue);
        }
    }
}

void SignalBus::commit(std::uint32_t signal, SimTime stamp, std::uint64_t tick,
                       Duration writerStep) {
    if (held_[signal] != 0) {
        held_[signal] = 0;
        return;
    }
    SignalRecord& record = signals_[signal];
    const auto begin = static_cast<std::ptrdiff_t>(record.offset);
    if (record.components == 0) {
        const auto count = static_cast<std::ptrdiff_t>(batchCount_);
        std::copy(committedHandles_.begin() + begin, committedHandles_.begin() + begin + count,
                  previousHandles_.begin() + begin);
        std::copy(pendingHandles_.begin() + begin, pendingHandles_.begin() + begin + count,
                  committedHandles_.begin() + begin);
    } else {
        const auto count = static_cast<std::ptrdiff_t>(record.components * batchCount_);
        std::copy(committed_.begin() + begin, committed_.begin() + begin + count,
                  previous_.begin() + begin);
        std::copy(pending_.begin() + begin, pending_.begin() + begin + count,
                  committed_.begin() + begin);
        dirty_[signal] = 0;
    }
    Timing& timing = timing_[signal];
    timing.previousStamp = timing.stamp;
    timing.stamp = stamp;
    timing.commitTick = tick;
    timing.commits = std::min<std::uint32_t>(timing.commits + 1, 2);

    // Window-average readers accumulate every commit, weighted by the writer's step, so the
    // mean they see conserves the integral of a flow (impulse, energy).
    if (record.components == 0) {
        return;
    }
    const double weight = static_cast<double>(writerStep.ns);
    const double* values = committed_.data() + record.offset;
    const std::size_t count = record.components * batchCount_;
    for (const std::uint32_t slotIndex : windowReaders_[signal]) {
        InputSlot& slot = inputs_[slotIndex];
        double* sums = windowSums_.data() + slot.offset;
        for (std::size_t i = 0; i < count; ++i) {
            sums[i] += values[i] * weight;
        }
        slot.windowWeight += weight;
    }
}

bool SignalBus::commitDense(std::span<const std::uint32_t> signals, SimTime stamp,
                            std::uint64_t tick) {
    // A module whose outputs are plain values in one stretch of storage (DensePlan, allocate())
    // and that wrote them all, none held: the electrical network at 10 to 20 kHz, the physics.
    // Its commit is a copy of the stretch and a pass over the timings, without the per-signal
    // checks of the general case (which gives the same result for it).
    if (signals.empty()) {
        return false;
    }
    const std::uint32_t writer = signals_[signals.front()].writer;
    if (writer >= densePlans_.size()) {
        return false;
    }
    const DensePlan& plan = densePlans_[writer];
    if (plan.count == 0 || plan.first != signals.front() || plan.count != signals.size()) {
        return false;
    }
    const std::uint8_t* const held = held_.data() + plan.first;
    std::uint8_t* const dirty = dirty_.data() + plan.first;
    unsigned allDirty = 1;
    unsigned anyHeld = 0;
    for (std::uint32_t k = 0; k < plan.count; ++k) {
        allDirty &= dirty[k];
        anyHeld |= held[k];
    }
    if (allDirty == 0 || anyHeld != 0) {
        return false;
    }
    for (std::uint32_t r = plan.previousBegin; r < plan.previousEnd; ++r) {
        const auto [from, to] = denseRanges_[r];
        std::copy(committed_.data() + from, committed_.data() + to, previous_.data() + from);
    }
    std::copy(pending_.data() + plan.valueFrom, pending_.data() + plan.valueTo,
              committed_.data() + plan.valueFrom);
    std::fill_n(dirty, plan.count, std::uint8_t{0});
    Timing* const timings = timing_.data() + plan.first;
    for (std::uint32_t k = 0; k < plan.count; ++k) {
        Timing& timing = timings[k];
        timing.previousStamp = timing.stamp;
        timing.stamp = stamp;
        timing.commitTick = tick;
        timing.commits = std::min<std::uint32_t>(timing.commits + 1, 2);
    }
    return true;
}

void SignalBus::commitAll(std::span<const std::uint32_t> signals, SimTime stamp, std::uint64_t tick,
                          Duration writerStep) {
    // One pass over the module's outputs, from the compact commitInfo_ (not the signal records,
    // a cache line each). A signal that is plain values, not held and read by no window average
    // commits as a copy of its pending value, and only when written since its last commit (the
    // others' pending values equal their committed ones). Written signals that sit next to each
    // other in storage copy at once, across a short clean stretch of this module's own plain
    // signals too (copying an equal value is harmless and saves a call); never across another
    // module's storage.
    if (commitDense(signals, stamp, tick)) {
        return;
    }
    constexpr std::size_t kMergeGap = 32;  // values
    constexpr std::size_t kNoRun = ~std::size_t{0};
    // Raw pointers in locals: the byte stores to dirty_ may alias anything, so members read
    // through `this` would be loaded again after each one.
    const CommitInfo* const infos = commitInfo_.data();
    const std::uint8_t* const held = held_.data();
    std::uint8_t* const dirty = dirty_.data();
    Timing* const timings = timing_.data();
    const double* const pending = pending_.data();
    double* const committed = committed_.data();
    double* const previous = previous_.data();
    std::size_t runEnd = kNoRun;  // end of the last plain signal's storage: the run goes on there
    std::size_t copyFrom = 0;
    std::size_t copyTo = 0;
    bool open = false;
    const auto flush = [pending, committed, &open, &copyFrom, &copyTo] {
        if (open) {
            std::copy(pending + copyFrom, pending + copyTo, committed + copyFrom);
            open = false;
        }
    };
    for (const std::uint32_t signal : signals) {
        const CommitInfo info = infos[signal];
        if (info.plain == 0 || held[signal] != 0) {
            runEnd = kNoRun;
            commit(signal, stamp, tick, writerStep);
            continue;
        }
        const std::size_t from = info.offset;
        const std::size_t to = from + info.count;
        if (from != runEnd) {
            flush();  // a new run of storage
        }
        runEnd = to;
        // The previous value only where a reader uses it (a feedback input, an extrapolation),
        // before the committed one is overwritten (the copy below is deferred to the flush).
        if (info.needsPrevious != 0) {
            std::copy(committed + from, committed + to, previous + from);
        }
        if (dirty[signal] != 0) {
            dirty[signal] = 0;
            if (open && from - copyTo <= kMergeGap) {
                copyTo = to;
            } else {
                flush();
                copyFrom = from;
                copyTo = to;
                open = true;
            }
        }
        Timing& timing = timings[signal];
        timing.previousStamp = timing.stamp;
        timing.stamp = stamp;
        timing.commitTick = tick;
        timing.commits = std::min<std::uint32_t>(timing.commits + 1, 2);
    }
    flush();
}

void SignalBus::prepareInput(std::uint32_t slotIndex, SimTime readerTime, std::uint64_t tick) {
    // The common case first, from two small arrays: unconnected, or a held copy whose writer
    // has not committed since (and not in this tick, when a feedback input reads the previous
    // value).
    const std::uint32_t signal = slotSignal_[slotIndex];
    if (signal == kNoModule) {
        return;
    }
    const std::uint64_t committedAt = timing_[signal].commitTick;
    if (preparedTick_[slotIndex] == committedAt && committedAt != tick) {
        return;
    }
    InputSlot& slot = inputs_[slotIndex];
    if (slot.signal == kNoModule) {
        return;  // optional and unconnected: the default stays
    }
    const SignalRecord& record = signals_[slot.signal];
    const Timing& timing = timing_[slot.signal];
    // A feedback input reads the writer's previous step when the writer already ran this tick.
    const bool usePrevious = slot.feedback && timing.commitTick == tick;
    if (record.components == 0) {
        const auto& source = usePrevious ? previousHandles_ : committedHandles_;
        std::copy_n(source.begin() + static_cast<std::ptrdiff_t>(record.offset), batchCount_,
                    inputHandles_.begin() + static_cast<std::ptrdiff_t>(slot.offset));
        return;
    }
    const std::size_t count = record.components * batchCount_;
    const double* committed = committed_.data() + record.offset;
    const double* previous = previous_.data() + record.offset;
    double* out = inputValues_.data() + slot.offset;

    std::uint64_t& prepared = preparedTick_[slotIndex];
    if (usePrevious) {
        std::copy_n(previous, count, out);
        prepared = kStale;
        return;
    }
    if (slot.hold != Hold::ZeroOrder) {
        prepared = kStale;
    }
    switch (slot.hold) {
        case Hold::ZeroOrder:
            // Already the committed value, unless its writer committed since.
            if (prepared != timing.commitTick) {
                std::copy_n(committed, count, out);
                prepared = timing.commitTick;
            }
            break;
        case Hold::LinearExtrapolate: {
            const std::int64_t span = timing.stamp.ns - timing.previousStamp.ns;
            if (timing.commits < 2 || span <= 0) {
                std::copy_n(committed, count, out);
                break;
            }
            const double factor =
                static_cast<double>(readerTime.ns - timing.stamp.ns) / static_cast<double>(span);
            for (std::size_t i = 0; i < count; ++i) {
                out[i] = committed[i] + (committed[i] - previous[i]) * factor;
            }
            break;
        }
        case Hold::WindowAverage: {
            double* sums = windowSums_.data() + slot.offset;
            if (slot.windowWeight > 0.0) {
                for (std::size_t i = 0; i < count; ++i) {
                    out[i] = sums[i] / slot.windowWeight;
                }
            } else {
                std::copy_n(committed, count, out);
            }
            std::fill_n(sums, count, 0.0);
            slot.windowWeight = 0.0;
            break;
        }
    }
}

void SignalBus::prepareInputs(std::span<const std::uint32_t> slots, SimTime readerTime,
                              std::uint64_t tick) {
    for (const std::uint32_t slot : slots) {
        prepareInput(slot, readerTime, tick);
    }
}

}  // namespace lagrange
