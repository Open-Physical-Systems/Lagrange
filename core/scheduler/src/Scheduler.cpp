#include "lagrange/scheduler/Scheduler.hpp"

// Header-only xxHash, as in proto/transport: no library to link.
#define XXH_INLINE_ALL
#include "lagrange/random/Philox.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <xxhash.h>

namespace lagrange {

namespace {

constexpr std::uint32_t kStateMagic = 0x5453474CU;  // "LGST"
constexpr std::uint32_t kStateVersion = 1;
constexpr std::int64_t kMinBaseTickNs = 1'000;              // 1 µs
constexpr std::int64_t kMaxHyperperiodNs = 10'000'000'000;  // 10 s
constexpr std::uint64_t kInitTick = ~0ULL;  // commit tick of initial values: before tick 0
constexpr std::uint32_t kNone = 0xFFFF'FFFFU;

std::string formatDuration(Duration d) {
    char text[32];
    if (d.ns % 1'000'000'000 == 0) {
        std::snprintf(text, sizeof(text), "%lld s", static_cast<long long>(d.ns / 1'000'000'000));
    } else if (d.ns % 1'000'000 == 0) {
        std::snprintf(text, sizeof(text), "%lld ms", static_cast<long long>(d.ns / 1'000'000));
    } else if (d.ns % 1'000 == 0) {
        std::snprintf(text, sizeof(text), "%lld µs", static_cast<long long>(d.ns / 1'000));
    } else {
        std::snprintf(text, sizeof(text), "%lld ns", static_cast<long long>(d.ns));
    }
    return text;
}

std::string formatTime(SimTime t) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.9g s", t.seconds());
    return text;
}

std::uint32_t moduleIdOf(std::string_view label) {
    return static_cast<std::uint32_t>(XXH3_64bits(label.data(), label.size()));
}

}  // namespace

std::string_view to_string(WiringError::Kind kind) {
    switch (kind) {
        case WiringError::Kind::UnitMismatch:
            return "UnitMismatch";
        case WiringError::Kind::TypeMismatch:
            return "TypeMismatch";
        case WiringError::Kind::MissingWriter:
            return "MissingWriter";
        case WiringError::Kind::DuplicateWriter:
            return "DuplicateWriter";
        case WiringError::Kind::Cycle:
            return "Cycle";
        case WiringError::Kind::ContractUnsatisfied:
            return "ContractUnsatisfied";
        case WiringError::Kind::IncommensurateSteps:
            return "IncommensurateSteps";
        case WiringError::Kind::TierUnsupported:
            return "TierUnsupported";
        case WiringError::Kind::NotDeterministic:
            return "NotDeterministic";
        case WiringError::Kind::InvalidDeclaration:
            return "InvalidDeclaration";
        case WiringError::Kind::ModuleFailed:
            return "ModuleFailed";
    }
    return "Unknown";
}

std::string to_string(const WiringError& error) {
    std::string out(to_string(error.kind));
    if (!error.signal.empty()) {
        out += " " + error.signal;
    }
    out += ": " + error.detail;
    return out;
}

Scheduler::Scheduler(SchedulerOptions options) : options_(options) {}

Scheduler::~Scheduler() {
    shutdown();
}

void Scheduler::add(std::unique_ptr<IModule> module, ModuleConfig config) {
    Entry entry;
    entry.module = std::move(module);
    entry.config = std::move(config);
    entries_.push_back(std::move(entry));
}

Expected<void, std::vector<WiringError>> Scheduler::wire(const WorldView& world) {
    using Kind = WiringError::Kind;
    std::vector<WiringError> errors;
    if (wired_) {
        errors.push_back({Kind::ModuleFailed, {}, "wire() was already called"});
        return Unexpected{std::move(errors)};
    }

    // 1. Each module describes itself and declares its signals.
    for (std::uint32_t i = 0; i < entries_.size(); ++i) {
        Entry& entry = entries_[i];
        entry.info = entry.module->info();
        entry.label = std::string(entry.info.domain) + "/" + std::string(entry.info.name);
        entry.dt = entry.config.dt().value_or(entry.module->preferredStep());
        entry.seed = random::moduleSeed(options_.runSeed, moduleIdOf(entry.label));
        if (entry.info.minimumTier > entry.config.tier()) {
            errors.push_back({Kind::TierUnsupported,
                              {},
                              entry.label + " needs tier " +
                                  std::to_string(static_cast<int>(entry.info.minimumTier)) +
                                  "; this run has tier " +
                                  std::to_string(static_cast<int>(entry.config.tier()))});
        }
        if (options_.deterministic && !entry.info.deterministic) {
            errors.push_back({Kind::NotDeterministic,
                              {},
                              entry.label + " is not deterministic; the profile asks for a "
                                            "deterministic run"});
        }
        SignalDeclarer declarer(bus_, i);
        const Status status = entry.module->declare(world, entry.config, declarer);
        if (!status.ok()) {
            errors.push_back({Kind::ModuleFailed,
                              {},
                              entry.label + " declare: " + std::string(to_string(status.code)) +
                                  ": " + status.message});
        }
        for (const std::string& problem : declarer.problems_) {
            errors.push_back({Kind::InvalidDeclaration, {}, entry.label + " " + problem});
        }
        entry.inputSlots = std::move(declarer.inputSlots_);
        entry.outputSignals = std::move(declarer.outputSignals_);
    }

    // 2. Static checks; all errors are collected.
    checkSteps(errors);
    connect(errors);
    checkContracts(world, errors);
    order(errors);
    if (!errors.empty()) {
        return Unexpected{std::move(errors)};
    }

    // 3. Storage, then initial values in execution order.
    bus_.allocate(world.batchCount());
    for (const std::uint32_t index : order_) {
        Entry& entry = entries_[index];
        bus_.currentModule_ = index;
        bus_.misuse_.clear();
        const Status status = entry.module->init(world, entry.config, bus_);
        bus_.currentModule_ = SignalBus::kNoModule;
        entry.initialised = true;
        if (!status.ok()) {
            errors.push_back({Kind::ModuleFailed,
                              {},
                              entry.label + " init: " + std::string(to_string(status.code)) + ": " +
                                  status.message});
        } else if (!bus_.misuse_.empty()) {
            errors.push_back({Kind::ModuleFailed, {}, entry.label + " init: " + bus_.misuse_});
        }
        for (const std::uint32_t signal : entry.outputSignals) {
            bus_.commit(signal, SimTime{0}, kInitTick, Duration{0});
        }
    }
    if (!errors.empty()) {
        shutdown();
        return Unexpected{std::move(errors)};
    }

    plan_.clear();
    for (std::uint32_t position = 0; position < order_.size(); ++position) {
        const Entry& entry = entries_[order_[position]];
        plan_.push_back(
            {entry.module.get(), entry.label, entry.dt, entry.level, position, entry.seed});
    }
    wired_ = true;
    now_ = SimTime{0};
    tick_ = 0;
    return {};
}

void Scheduler::checkSteps(std::vector<WiringError>& errors) {
    using Kind = WiringError::Kind;
    std::int64_t gcd = 0;
    std::int64_t lcm = 1;
    bool overflow = false;
    for (const Entry& entry : entries_) {
        if (entry.dt.ns <= 0) {
            errors.push_back({Kind::IncommensurateSteps,
                              {},
                              entry.label + " has step " + formatDuration(entry.dt) +
                                  "; a step must be positive"});
            continue;
        }
        gcd = std::gcd(gcd, entry.dt.ns);
        if (!overflow) {
            const std::int64_t factor = entry.dt.ns / std::gcd(lcm, entry.dt.ns);
            if (lcm > kMaxHyperperiodNs / factor) {
                overflow = true;
            } else {
                lcm *= factor;
            }
        }
    }
    if (gcd == 0) {
        return;
    }
    baseTick_ = Duration{gcd};
    hyperperiod_ = Duration{overflow ? 0 : lcm};
    if (gcd < kMinBaseTickNs) {
        errors.push_back({Kind::IncommensurateSteps,
                          {},
                          "base tick " + formatDuration(baseTick_) +
                              " is below 1 µs; check the profile's dt values for a typo"});
    }
    if (overflow || lcm > kMaxHyperperiodNs) {
        errors.push_back({Kind::IncommensurateSteps,
                          {},
                          "the steps repeat only after more than 10 s; check the profile's dt "
                          "values for a typo such as 0.00101"});
    }
}

void Scheduler::connect(std::vector<WiringError>& errors) {
    using Kind = WiringError::Kind;
    // Writers by name; the first writer keeps the name, others are errors.
    std::map<std::string, std::uint32_t, std::less<>> writers;
    for (std::uint32_t s = 0; s < bus_.signals_.size(); ++s) {
        const SignalBus::SignalRecord& record = bus_.signals_[s];
        const auto [it, inserted] = writers.emplace(record.spec.name, s);
        if (!inserted) {
            errors.push_back({Kind::DuplicateWriter, record.spec.name,
                              "written by " + entries_[bus_.signals_[it->second].writer].label +
                                  " and " + entries_[record.writer].label +
                                  "; a signal has one writer"});
        }
    }
    for (SignalBus::InputSlot& slot : bus_.inputs_) {
        const std::string& reader = entries_[slot.reader].label;
        const SignalSpec& want = slot.spec;
        const auto it = writers.find(want.name);
        if (it == writers.end()) {
            if (!want.optional) {
                errors.push_back({Kind::MissingWriter, want.name,
                                  "read by " + reader +
                                      ", but no enabled module or substitute "
                                      "writes it"});
            }
            continue;
        }
        const SignalBus::SignalRecord& record = bus_.signals_[it->second];
        const SignalSpec& have = record.spec;
        const std::string& writer = entries_[record.writer].label;
        bool ok = true;
        if (have.type != want.type || have.arrayLength != want.arrayLength) {
            errors.push_back({Kind::TypeMismatch, want.name,
                              "writer " + writer + ": " + std::string(to_string(have.type)) +
                                  (have.type == SignalType::ScalarArray
                                       ? "[" + std::to_string(have.arrayLength) + "]"
                                       : "") +
                                  "; reader " + reader + ": " + std::string(to_string(want.type)) +
                                  (want.type == SignalType::ScalarArray
                                       ? "[" + std::to_string(want.arrayLength) + "]"
                                       : "")});
            ok = false;
        }
        if (have.unit != want.unit) {
            errors.push_back({Kind::UnitMismatch, want.name,
                              "writer " + writer + ": " + to_string(have.unit) + "; reader " +
                                  reader + ": " + to_string(want.unit)});
            ok = false;
        }
        if (!have.frame.empty() && !want.frame.empty() && have.frame != want.frame) {
            errors.push_back({Kind::TypeMismatch, want.name,
                              "writer " + writer + " uses frame \"" + have.frame + "\"; reader " +
                                  reader + " expects \"" + want.frame + "\""});
            ok = false;
        }
        const bool numeric =
            !isHandle(want.type) && want.type != SignalType::Bool && want.type != SignalType::Int;
        if (want.hold != Hold::ZeroOrder && !numeric) {
            errors.push_back({Kind::TypeMismatch, want.name,
                              reader + " reads a " + std::string(to_string(want.type)) +
                                  " with hold " + std::string(to_string(want.hold)) +
                                  "; only ZeroOrder applies to this type"});
            ok = false;
        }
        if (ok) {
            slot.signal = it->second;
        }
    }
}

void Scheduler::checkContracts(const WorldView& world, std::vector<WiringError>& errors) {
    if (options_.contracts == nullptr) {
        return;
    }
    for (const Entry& entry : entries_) {
        const DomainContract* contract = options_.contracts->contract(entry.info.domain);
        if (contract == nullptr) {
            continue;
        }
        for (const ContractRule& rule : contract->provides) {
            std::vector<std::string> names;
            if (rule.schema.empty()) {
                names.push_back(rule.signalPattern);
            } else {
                for (const SchemaInstance* instance : world.instancesOf(rule.schema)) {
                    std::string name = rule.signalPattern;
                    const std::size_t at = name.find("{instance}");
                    if (at != std::string::npos) {
                        name.replace(at, 10, world.signalInstanceName(instance->prim));
                    }
                    names.push_back(std::move(name));
                }
            }
            for (const std::string& name : names) {
                const SignalSpec* provided = nullptr;
                for (const std::uint32_t signal : entry.outputSignals) {
                    if (bus_.signals_[signal].spec.name == name) {
                        provided = &bus_.signals_[signal].spec;
                    }
                }
                if (provided == nullptr) {
                    errors.push_back({WiringError::Kind::ContractUnsatisfied, name,
                                      entry.label + " does not write it, but the " +
                                          contract->domain + " contract promises it"});
                } else if (provided->type != rule.type || provided->unit != rule.unit) {
                    errors.push_back(
                        {WiringError::Kind::ContractUnsatisfied, name,
                         entry.label + " writes " + std::string(to_string(provided->type)) + " " +
                             to_string(provided->unit) + "; the " + contract->domain +
                             " contract promises " + std::string(to_string(rule.type)) + " " +
                             to_string(rule.unit)});
                }
            }
        }
    }
}

void Scheduler::order(std::vector<WiringError>& errors) {
    const auto count = static_cast<std::uint32_t>(entries_.size());
    // Edges writer -> reader, with the signals that carry them; feedback inputs are not edges.
    std::vector<std::map<std::uint32_t, std::vector<std::string>>> edges(count);
    std::vector<std::uint32_t> inDegree(count, 0);
    for (const SignalBus::InputSlot& slot : bus_.inputs_) {
        if (slot.signal == kNone || slot.spec.feedback) {
            continue;
        }
        const std::uint32_t writer = bus_.signals_[slot.signal].writer;
        auto& carried = edges[writer][slot.reader];
        if (carried.empty()) {
            ++inDegree[slot.reader];
        }
        carried.push_back(slot.spec.name);
    }

    // Kahn's algorithm, always taking the smallest (dt, domain, name, index) that is ready.
    const auto key = [this](std::uint32_t i) {
        const Entry& e = entries_[i];
        return std::make_tuple(e.dt.ns, std::string_view(e.info.domain),
                               std::string_view(e.info.name), i);
    };
    const auto before = [&key](std::uint32_t a, std::uint32_t b) { return key(a) < key(b); };
    std::set<std::uint32_t, decltype(before)> ready(before);
    for (std::uint32_t i = 0; i < count; ++i) {
        if (inDegree[i] == 0) {
            ready.insert(i);
        }
    }
    order_.clear();
    while (!ready.empty()) {
        const std::uint32_t next = *ready.begin();
        ready.erase(ready.begin());
        order_.push_back(next);
        for (const auto& [reader, signals] : edges[next]) {
            entries_[reader].level = std::max(entries_[reader].level, entries_[next].level + 1);
            if (--inDegree[reader] == 0) {
                ready.insert(reader);
            }
        }
    }
    if (order_.size() == count) {
        return;
    }

    // A cycle remains: walk from a blocked module through blocked modules until one repeats.
    std::uint32_t start = 0;
    while (inDegree[start] == 0) {
        ++start;
    }
    std::vector<std::uint32_t> path{start};
    std::vector<int> seenAt(count, -1);
    seenAt[start] = 0;
    for (;;) {
        // Any blocked predecessor keeps the walk inside the cycle's strongly connected part.
        std::uint32_t predecessor = kNone;
        for (std::uint32_t w = 0; w < count && predecessor == kNone; ++w) {
            if (inDegree[w] > 0 && edges[w].contains(path.back())) {
                predecessor = w;
            }
        }
        if (predecessor == kNone || seenAt[predecessor] >= 0) {
            if (predecessor != kNone) {
                path.erase(path.begin(), path.begin() + seenAt[predecessor]);
            }
            break;
        }
        seenAt[predecessor] = static_cast<int>(path.size());
        path.push_back(predecessor);
    }
    std::reverse(path.begin(), path.end());  // now in writer -> reader direction
    std::string detail;
    std::string firstSignal;
    for (std::size_t i = 0; i < path.size(); ++i) {
        const std::uint32_t from = path[i];
        const std::uint32_t to = path[(i + 1) % path.size()];
        detail += entries_[from].label + " -> ";
        const auto it = edges[from].find(to);
        if (it != edges[from].end()) {
            if (firstSignal.empty()) {
                firstSignal = it->second.front();
            }
            detail += "(";
            for (std::size_t s = 0; s < it->second.size(); ++s) {
                detail += (s > 0 ? ", " : "") + it->second[s];
            }
            detail += ") -> ";
        }
    }
    detail += entries_[path.front()].label +
              "; mark one of these inputs `feedback: true` to read the previous step";
    errors.push_back({WiringError::Kind::Cycle, firstSignal, detail});
}

Status Scheduler::stepTick() {
    if (!wired_) {
        return Status::error(StatusCode::Internal, "stepTick() before a successful wire()");
    }
    for (const std::uint32_t index : order_) {
        Entry& entry = entries_[index];
        if (now_.ns % entry.dt.ns != 0) {
            continue;
        }
        for (const std::uint32_t slot : entry.inputSlots) {
            bus_.prepareInput(slot, now_, tick_);
        }
        bus_.currentModule_ = index;
        bus_.misuse_.clear();
        const StepContext context{now_, entry.dt, entry.stepIndex, entry.seed};
        const auto started = std::chrono::steady_clock::now();
        const Status status = entry.module->step(context, bus_);
        const std::int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now() - started)
                                         .count();
        ++entry.costSteps;
        entry.costTotalNs += elapsed;
        entry.costMaxNs = std::max(entry.costMaxNs, elapsed);
        bus_.currentModule_ = SignalBus::kNoModule;
        if (!bus_.misuse_.empty()) {
            return Status::error(StatusCode::Internal,
                                 entry.label + " at " + formatTime(now_) + ": " + bus_.misuse_);
        }
        if (!status.ok()) {
            return Status::error(status.code,
                                 entry.label + " at " + formatTime(now_) + ": " + status.message);
        }
        for (const std::uint32_t signal : entry.outputSignals) {
            bus_.commit(signal, now_ + entry.dt, tick_, entry.dt);
        }
        ++entry.stepIndex;
    }
    now_ = now_ + baseTick_;
    ++tick_;
    return {};
}

Status Scheduler::stepUntil(SimTime t) {
    while (now_ < t) {
        if (Status status = stepTick(); !status.ok()) {
            return status;
        }
    }
    return {};
}

SignalWriter Scheduler::writerOf(SignalId output) const {
    if (!output.valid() || output.isInput() || output.index >= bus_.signals_.size()) {
        return {};
    }
    const Entry& entry = entries_[bus_.signals_[output.index].writer];
    return {entry.label, entry.dt, entry.info.isSubstitute};
}

std::vector<Connection> Scheduler::connections() const {
    std::vector<Connection> out;
    for (const SignalBus::InputSlot& slot : bus_.inputs_) {
        if (slot.signal == kNone) {
            continue;
        }
        const SignalBus::SignalRecord& record = bus_.signals_[slot.signal];
        out.push_back({entries_[record.writer].label, entries_[slot.reader].label, slot.spec.name,
                       record.spec.unit, slot.spec.unit, slot.spec.feedback});
    }
    return out;
}

std::vector<ModuleCost> Scheduler::takeCosts() {
    std::vector<ModuleCost> costs;
    for (const std::uint32_t index : order_) {
        Entry& entry = entries_[index];
        costs.push_back({entry.label, std::string(entry.info.domain), entry.dt, entry.costSteps,
                         entry.costTotalNs, entry.costMaxNs});
        entry.costSteps = 0;
        entry.costTotalNs = 0;
        entry.costMaxNs = 0;
    }
    return costs;
}

std::string Scheduler::dumpGraphDot() const {
    std::string out = "digraph lagrange {\n  rankdir=LR;\n  node [shape=box];\n";
    for (std::uint32_t i = 0; i < entries_.size(); ++i) {
        out += "  m" + std::to_string(i) + " [label=\"" + entries_[i].label + "\\n" +
               formatDuration(entries_[i].dt) + "\"];\n";
    }
    for (const SignalBus::InputSlot& slot : bus_.inputs_) {
        if (slot.signal == kNone) {
            continue;
        }
        out += "  m" + std::to_string(bus_.signals_[slot.signal].writer) + " -> m" +
               std::to_string(slot.reader) + " [label=\"" + slot.spec.name + "\"" +
               (slot.spec.feedback ? ", style=dashed" : "") + "];\n";
    }
    out += "}\n";
    return out;
}

// --- state ---------------------------------------------------------------------------------

namespace {

void putHandles(StateWriter& writer, const std::vector<std::uint64_t>& handles) {
    writer.put<std::uint64_t>(handles.size());
    writer.putBytes(handles.data(), handles.size() * sizeof(std::uint64_t));
}

bool getHandles(StateReader& reader, std::vector<std::uint64_t>& handles) {
    std::uint64_t size = 0;
    if (!reader.get(size) || size != handles.size()) {
        return false;
    }
    return reader.getBytes(handles.data(), handles.size() * sizeof(std::uint64_t));
}

}  // namespace

Status Scheduler::saveState(StateWriter& writer) const {
    if (!wired_) {
        return Status::error(StatusCode::Internal, "saveState() before a successful wire()");
    }
    writer.put(kStateMagic);
    writer.put(kStateVersion);
    writer.put(now_.ns);
    writer.put(tick_);
    writer.put<std::uint64_t>(order_.size());
    for (const std::uint32_t index : order_) {
        const Entry& entry = entries_[index];
        writer.putString(entry.label);
        writer.put(entry.stepIndex);
        StateWriter module;
        if (Status status = entry.module->saveState(module); !status.ok()) {
            return Status::error(status.code, entry.label + " saveState: " + status.message);
        }
        writer.put<std::uint64_t>(module.bytes().size());
        writer.putBytes(module.bytes().data(), module.bytes().size());
    }
    writer.putDoubles(bus_.committed_);
    writer.putDoubles(bus_.previous_);
    writer.putDoubles(bus_.pending_);
    putHandles(writer, bus_.committedHandles_);
    putHandles(writer, bus_.previousHandles_);
    putHandles(writer, bus_.pendingHandles_);
    for (const SignalBus::SignalRecord& record : bus_.signals_) {
        writer.put(record.stamp.ns);
        writer.put(record.previousStamp.ns);
        writer.put(record.commitTick);
        writer.put(record.commits);
    }
    writer.putDoubles(bus_.inputValues_);
    putHandles(writer, bus_.inputHandles_);
    writer.putDoubles(bus_.windowSums_);
    for (const SignalBus::InputSlot& slot : bus_.inputs_) {
        writer.put(slot.windowWeight);
    }
    return {};
}

Status Scheduler::restoreState(StateReader& reader) {
    const auto corrupt = [](std::string what) {
        return Status::error(StatusCode::InvalidConfig, "saved state does not match: " + what);
    };
    if (!wired_) {
        return Status::error(StatusCode::Internal, "restoreState() before a successful wire()");
    }
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    reader.get(magic);
    reader.get(version);
    if (magic != kStateMagic || version != kStateVersion) {
        return corrupt("not a scheduler state of version " + std::to_string(kStateVersion));
    }
    std::int64_t now = 0;
    std::uint64_t tick = 0;
    std::uint64_t modules = 0;
    reader.get(now);
    reader.get(tick);
    reader.get(modules);
    if (modules != order_.size()) {
        return corrupt("module count");
    }
    for (const std::uint32_t index : order_) {
        Entry& entry = entries_[index];
        std::string label;
        std::uint64_t stepIndex = 0;
        std::uint64_t size = 0;
        reader.getString(label);
        reader.get(stepIndex);
        reader.get(size);
        if (label != entry.label || reader.failed() || size > reader.remaining()) {
            return corrupt("module " + entry.label);
        }
        std::vector<std::byte> blob(size);
        reader.getBytes(blob.data(), blob.size());
        StateReader module(blob);
        if (Status status = entry.module->restoreState(module); !status.ok()) {
            return Status::error(status.code, entry.label + " restoreState: " + status.message);
        }
        entry.stepIndex = stepIndex;
    }
    bool ok = reader.getDoubles(bus_.committed_) && reader.getDoubles(bus_.previous_) &&
              reader.getDoubles(bus_.pending_) && getHandles(reader, bus_.committedHandles_) &&
              getHandles(reader, bus_.previousHandles_) && getHandles(reader, bus_.pendingHandles_);
    for (SignalBus::SignalRecord& record : bus_.signals_) {
        ok = ok && reader.get(record.stamp.ns) && reader.get(record.previousStamp.ns) &&
             reader.get(record.commitTick) && reader.get(record.commits);
    }
    ok = ok && reader.getDoubles(bus_.inputValues_) && getHandles(reader, bus_.inputHandles_) &&
         reader.getDoubles(bus_.windowSums_);
    for (SignalBus::InputSlot& slot : bus_.inputs_) {
        ok = ok && reader.get(slot.windowWeight);
    }
    if (!ok || reader.failed()) {
        return corrupt("signal bus layout");
    }
    now_ = SimTime{now};
    tick_ = tick;
    return {};
}

Expected<std::uint64_t, Status> Scheduler::stateHash() const {
    StateWriter writer;
    if (Status status = saveState(writer); !status.ok()) {
        return Unexpected{std::move(status)};
    }
    return XXH3_64bits(writer.bytes().data(), writer.bytes().size());
}

void Scheduler::shutdown() {
    for (auto it = order_.rbegin(); it != order_.rend(); ++it) {
        Entry& entry = entries_[*it];
        if (entry.initialised) {
            entry.module->shutdown();
            entry.initialised = false;
        }
    }
}

}  // namespace lagrange
