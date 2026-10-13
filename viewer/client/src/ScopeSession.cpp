#include "lagrange/viewer/ScopeSession.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <ostream>

namespace lagrange::viewer {

namespace {

constexpr std::size_t kChunk = 2048;  // samples read at a time for the trigger and export

}  // namespace

bool ScopeSession::add(ViewerClient& client, const SignalCatalog& catalog, const std::string& name,
                       std::uint32_t component) {
    const SignalInfo* info = catalog.find(name);
    if (info == nullptr || find(name, component) >= 0) {
        return false;
    }
    std::shared_ptr<ScopeBuffer> buffer = client.subscribe(name, component);
    if (!buffer) {
        return false;
    }
    ScopeTrace trace;
    trace.name = name;
    trace.component = component;
    trace.label = name;
    if (info->components > 1) {
        trace.label += component < info->componentNames.size()
                           ? "." + info->componentNames[component]
                           : "[" + std::to_string(component) + "]";
    }
    trace.unit = info->unit.empty() ? "1" : info->unit;
    trace.domain = info->writer.substr(0, info->writer.find('/'));
    trace.buffer = std::move(buffer);
    // The lowest colour no other trace uses, so colours do not shift when one is removed.
    for (int color = 0; color < kColors; ++color) {
        const bool used = std::ranges::any_of(
            traces_, [color](const ScopeTrace& other) { return other.color == color; });
        if (!used) {
            trace.color = color;
            break;
        }
        trace.color = static_cast<int>(traces_.size()) % kColors;
    }
    traces_.push_back(std::move(trace));
    rebuildUnits();
    return true;
}

void ScopeSession::adopt(std::string name, std::string unit, std::shared_ptr<ScopeBuffer> buffer) {
    ScopeTrace trace;
    trace.label = name;
    trace.name = std::move(name);
    trace.unit = std::move(unit);
    trace.color = static_cast<int>(traces_.size()) % kColors;
    trace.buffer = std::move(buffer);
    traces_.push_back(std::move(trace));
    rebuildUnits();
}

void ScopeSession::remove(ViewerClient& client, std::size_t index) {
    if (index >= traces_.size()) {
        return;
    }
    client.unsubscribe(traces_[index].buffer);
    traces_.erase(traces_.begin() + static_cast<std::ptrdiff_t>(index));
    // The trigger refers to a trace by index.
    const int removed = static_cast<int>(index);
    if (trigger_.trace == removed) {
        trigger_ = Trigger{};
    } else if (trigger_.trace > removed) {
        --trigger_.trace;
    }
    rebuildUnits();
}

int ScopeSession::find(std::string_view name, std::uint32_t component) const {
    for (std::size_t i = 0; i < traces_.size(); ++i) {
        if (traces_[i].name == name && traces_[i].component == component) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void ScopeSession::rebuildUnits() {
    units_.clear();
    for (const ScopeTrace& trace : traces_) {
        if (std::ranges::find(units_, trace.unit) == units_.end()) {
            units_.push_back(trace.unit);
        }
    }
}

void ScopeSession::update() {
    std::int64_t newest = -1;
    for (const ScopeTrace& trace : traces_) {
        newest = std::max(newest, trace.buffer->newestTimeNs());
    }
    if (newest >= 0) {
        // A replay that seeks back restarts the streams: follow it, whichever way time went.
        newestNs_ = newest;
    }
    evaluateTrigger();
}

ScopeSession::View ScopeSession::view() const {
    const std::int64_t end = paused_ ? frozenEndNs_ : newestNs_;
    const double seconds = kWindowSeconds.at(static_cast<std::size_t>(std::clamp(window, 0, 3)));
    if (seconds > 0.0) {
        return {end - static_cast<std::int64_t>(seconds * 1e9), end};
    }
    // The whole run: from the oldest sample any trace still holds.
    std::int64_t oldest = end;
    for (const ScopeTrace& trace : traces_) {
        const std::int64_t time = trace.buffer->oldestTimeNs();
        if (time >= 0) {
            oldest = std::min(oldest, time);
        }
    }
    // Never an empty range: plots need a width.
    return {std::min(oldest, end - 1'000'000), end};
}

void ScopeSession::setPaused(bool paused) {
    if (paused && !paused_) {
        frozenEndNs_ = newestNs_;
    }
    paused_ = paused;
}

void ScopeSession::showCursors(bool on) {
    if (on && !cursors) {
        const double span = view().seconds();
        cursorA = -0.66 * span;
        cursorB = -0.28 * span;
    }
    cursors = on;
}

std::int64_t ScopeSession::cursorTimeNs(double cursor) const {
    return view().endNs + static_cast<std::int64_t>(cursor * 1e9);
}

void ScopeSession::arm(int trace, bool rising, double level) {
    if (trace < 0 || trace >= static_cast<int>(traces_.size())) {
        return;
    }
    trigger_ = Trigger{true, trace, rising, level, -1};
    // Only crossings from now on count.
    const std::int64_t newest = traces_[static_cast<std::size_t>(trace)].buffer->newestTimeNs();
    checkedNs_ = newest < 0 ? 0 : newest + 1;
    hasPrevious_ = false;
    if (newest >= 0) {
        previous_ = traces_[static_cast<std::size_t>(trace)].buffer->latest();
        hasPrevious_ = previous_ == previous_;
    }
    paused_ = false;
}

void ScopeSession::disarm() {
    trigger_.armed = false;
}

void ScopeSession::evaluateTrigger() {
    if (!trigger_.armed || trigger_.trace < 0 ||
        trigger_.trace >= static_cast<int>(traces_.size())) {
        return;
    }
    const ScopeBuffer& buffer = *traces_[static_cast<std::size_t>(trigger_.trace)].buffer;
    const std::int64_t newest = buffer.newestTimeNs();
    if (newest < 0) {
        return;
    }
    if (newest + 1 < checkedNs_) {
        // The stream restarted (seek, new run): start looking again from what is there.
        checkedNs_ = std::max<std::int64_t>(buffer.oldestTimeNs(), 0);
        hasPrevious_ = false;
    }
    std::array<double, kChunk> times{};
    std::array<double, kChunk> values{};
    while (checkedNs_ <= newest) {
        const std::size_t count = buffer.readRaw(checkedNs_, newest + 1, times, values);
        if (count == 0) {
            // Nothing readable from there: the buffer moved on. Skip to what it still holds.
            const std::int64_t oldest = buffer.oldestTimeNs();
            if (oldest <= checkedNs_) {
                return;
            }
            checkedNs_ = oldest;
            hasPrevious_ = false;
            continue;
        }
        for (std::size_t i = 0; i < count; ++i) {
            const double value = values[i];
            if (value != value) {
                hasPrevious_ = false;  // a gap: no crossing across it
                continue;
            }
            const bool crossed =
                hasPrevious_ &&
                (trigger_.rising ? previous_ < trigger_.level && value >= trigger_.level
                                 : previous_ > trigger_.level && value <= trigger_.level);
            previous_ = value;
            hasPrevious_ = true;
            if (crossed) {
                trigger_.firedNs = static_cast<std::int64_t>(std::llround(times[i] * 1e9));
                trigger_.armed = false;  // single shot
                frozenEndNs_ = trigger_.firedNs;
                paused_ = true;
                return;
            }
        }
        checkedNs_ = static_cast<std::int64_t>(std::llround(times[count - 1] * 1e9)) + 1;
    }
}

std::size_t writeCsv(std::ostream& out, const ScopeSession& session, ScopeSession::View view) {
    out << "signal,unit,time_s,value\n";
    std::array<double, kChunk> times{};
    std::array<double, kChunk> values{};
    std::array<char, 64> text{};
    std::size_t rows = 0;
    const auto write = [&](std::to_chars_result result) {
        out.write(text.data(), result.ec == std::errc{} ? result.ptr - text.data() : 0);
    };
    // Values: the shortest text that reads back to the same double. Times: 12 significant
    // digits, which is finer than a nanosecond over any run and hides binary rounding.
    const auto number = [&](double value) {
        write(std::to_chars(text.data(), text.data() + text.size(), value));
    };
    const auto time = [&](double seconds) {
        write(std::to_chars(text.data(), text.data() + text.size(), seconds,
                            std::chars_format::general, 12));
    };
    for (const ScopeTrace& trace : session.traces()) {
        std::int64_t from = view.beginNs;
        while (from < view.endNs) {
            const std::size_t count = trace.buffer->readRaw(from, view.endNs, times, values);
            if (count == 0) {
                break;
            }
            for (std::size_t i = 0; i < count; ++i) {
                if (values[i] != values[i]) {
                    continue;  // gaps are left out
                }
                out << trace.label << ',' << trace.unit << ',';
                time(times[i]);
                out << ',';
                number(values[i]);
                out << '\n';
                ++rows;
            }
            from = static_cast<std::int64_t>(std::llround(times[count - 1] * 1e9)) + 1;
        }
    }
    return rows;
}

}  // namespace lagrange::viewer
