#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/record/v1/record.pb.h"
#include "lagrange/time/SimTime.hpp"
#include "lagrange/viewer/v1/scene.pb.h"
#include "lagrange/viewer/v1/signals.pb.h"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace lagrange::server {

/// Writes a run to an MCAP file in the recording format of docs/design/formats.md §7 (roadmap
/// 0.2-E4), the one the mock server replays: /lagrange/meta (RunMeta), /lagrange/catalog
/// (SignalCatalog), /lagrange/scene/snapshot (SceneSnapshot), /lagrange/signals/<dt>ns
/// (SignalBlock, 100 ms of samples each), /lagrange/state_hash (StateHash), zstd chunks.
///
/// The server thread adds samples; serialised messages go to a writer thread, so the disk never
/// blocks a tick. Every signal is uniformly sampled from t = 0: a sample that is missing (a
/// signal first written later than t = 0) is NaN.
class Recorder {
public:
    static Expected<std::unique_ptr<Recorder>, std::string> create(const std::string& path);
    ~Recorder();
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;
    Recorder(Recorder&&) = delete;
    Recorder& operator=(Recorder&&) = delete;

    /// Writes the header messages. The catalog's signal ids are 0..n-1, as add() uses them.
    void begin(const record::v1::RunMeta& meta, const viewer::v1::SignalCatalog& catalog,
               const viewer::v1::SceneSnapshot& snapshot);
    /// Sample `index` (time index x dt) of signal `id`. Indices only grow; a gap is NaN.
    void add(std::uint32_t id, std::uint64_t index, std::span<const double> values);
    void stateHash(SimTime t, std::uint64_t hash);
    /// Writes the blocks whose 100 ms span ended at or before `now`.
    void flush(SimTime now);
    /// Writes what is left and the final RunMeta (with the duration), then closes the file.
    Expected<void, std::string> finish(SimTime end, record::v1::RunMeta meta);

    [[nodiscard]] const std::string& path() const;

private:
    Recorder();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::server
