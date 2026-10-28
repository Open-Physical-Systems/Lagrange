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
            return "character '" + std::string(1, c) + "' not allowed in \"" + std::string(name) +
                   "\" (use [A-Za-z0-9_.-] and '/')";
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
    return {pending_.data() + index, signals_[id.index].components * batchCount_};
}

SimTime SignalBus::stamp(SignalId id) const {
    if (id.isInput()) {
        if (id.slot() >= inputs_.size() || inputs_[id.slot()].signal == kNoModule) {
            return {};
        }
        return signals_[inputs_[id.slot()].signal].stamp;
    }
    return id.valid() && id.index < signals_.size() ? signals_[id.index].stamp : SimTime{};
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
    inputHandles_.assign(inputHandles, 0);
    windowSums_.assign(inputValues, 0.0);
    windowReaders_.assign(signals_.size(), {});
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        if (inputs_[i].signal != kNoModule && inputs_[i].spec.hold == Hold::WindowAverage &&
            !inputs_[i].spec.feedback) {
            windowReaders_[inputs_[i].signal].push_back(static_cast<std::uint32_t>(i));
        }
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
    }
    record.previousStamp = record.stamp;
    record.stamp = stamp;
    record.commitTick = tick;
    record.commits = std::min<std::uint32_t>(record.commits + 1, 2);

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

void SignalBus::prepareInput(std::uint32_t slotIndex, SimTime readerTime, std::uint64_t tick) {
    InputSlot& slot = inputs_[slotIndex];
    if (slot.signal == kNoModule) {
        return;  // optional and unconnected: the default stays
    }
    const SignalRecord& record = signals_[slot.signal];
    // A feedback input reads the writer's previous step when the writer already ran this tick.
    const bool usePrevious = slot.spec.feedback && record.commitTick == tick;
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

    if (usePrevious) {
        std::copy_n(previous, count, out);
        return;
    }
    switch (slot.spec.hold) {
        case Hold::ZeroOrder:
            std::copy_n(committed, count, out);
            break;
        case Hold::LinearExtrapolate: {
            const std::int64_t span = record.stamp.ns - record.previousStamp.ns;
            if (record.commits < 2 || span <= 0) {
                std::copy_n(committed, count, out);
                break;
            }
            const double factor =
                static_cast<double>(readerTime.ns - record.stamp.ns) / static_cast<double>(span);
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

}  // namespace lagrange
