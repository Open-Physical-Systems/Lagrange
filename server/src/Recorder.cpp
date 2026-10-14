#include "Recorder.hpp"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <mcap/writer.hpp>

namespace lagrange::server {

namespace {

namespace v1 = viewer::v1;

constexpr std::int64_t kBlockNs = 100'000'000;  // samples per SignalBlock message: 100 ms

/// A FileDescriptorSet with the message's file and everything it imports, as MCAP's protobuf
/// encoding wants for a schema (the same as the mock server writes).
std::string descriptorSet(const google::protobuf::Descriptor* descriptor) {
    google::protobuf::FileDescriptorSet set;
    std::set<std::string> added;
    const auto add = [&](const auto& self, const google::protobuf::FileDescriptor* file) -> void {
        if (!added.insert(std::string(file->name())).second) {
            return;
        }
        for (int i = 0; i < file->dependency_count(); ++i) {
            self(self, file->dependency(i));
        }
        file->CopyTo(set.add_file());
    };
    add(add, descriptor->file());
    std::string out;
    set.SerializeToString(&out);
    return out;
}

}  // namespace

struct Recorder::Impl {
    std::string path;
    mcap::McapWriter writer;
    std::map<std::string, mcap::ChannelId> channels;  // by topic

    struct Item {
        mcap::ChannelId channel;
        std::int64_t timeNs;
        std::string bytes;
    };
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<Item> queue;
    bool closing{false};
    std::thread thread;
    std::string error;  // the first write error, from the writer thread
    std::uint32_t sequence{0};

    struct Signal {
        std::int64_t dtNs{1};
        std::uint32_t components{0};
        std::uint64_t next{0};  // index of the next sample to store
        std::uint64_t firstPending{0};
        std::vector<double> pending;
    };
    std::vector<Signal> signals;
    std::map<std::int64_t, std::vector<std::uint32_t>> groups;  // by step size
    std::int64_t flushedNs{0};  // blocks before this time are written

    mcap::ChannelId channel(const std::string& topic, const google::protobuf::Descriptor* type) {
        if (const auto it = channels.find(topic); it != channels.end()) {
            return it->second;
        }
        mcap::Schema schema(type->full_name(), "protobuf", descriptorSet(type));
        writer.addSchema(schema);
        mcap::Channel out(topic, "protobuf", schema.id);
        writer.addChannel(out);
        channels.emplace(topic, out.id);
        return out.id;
    }

    void push(mcap::ChannelId channel, std::int64_t timeNs, const google::protobuf::Message& m) {
        Item item{channel, timeNs, {}};
        m.SerializeToString(&item.bytes);
        {
            const std::lock_guard<std::mutex> lock(mutex);
            queue.push_back(std::move(item));
        }
        wake.notify_one();
    }

    void run() {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            wake.wait(lock, [this] { return closing || !queue.empty(); });
            if (queue.empty() && closing) {
                return;
            }
            Item item = std::move(queue.front());
            queue.pop_front();
            lock.unlock();
            mcap::Message message;
            message.channelId = item.channel;
            message.sequence = ++sequence;
            message.logTime = static_cast<mcap::Timestamp>(item.timeNs);
            message.publishTime = message.logTime;
            message.data = reinterpret_cast<const std::byte*>(item.bytes.data());
            message.dataSize = item.bytes.size();
            const mcap::Status status = writer.write(message);
            lock.lock();
            if (!status.ok() && error.empty()) {
                error = status.message;
            }
        }
    }

    /// One SignalBlock per step size with the pending samples, at time `t`.
    void writeBlocks(std::int64_t t) {
        for (const auto& [dt, members] : groups) {
            v1::SignalBlock block;
            block.set_catalog_generation(1);
            for (const std::uint32_t id : members) {
                Signal& s = signals[id];
                if (s.pending.empty()) {
                    continue;
                }
                v1::SignalRun& run = *block.add_runs();
                const auto count = static_cast<std::uint32_t>(s.pending.size() / s.components);
                run.set_signal(id);
                run.mutable_t0()->set_ns(static_cast<std::int64_t>(s.firstPending) * dt);
                run.set_dt_ns(dt);
                run.set_sample_count(count);
                run.set_components(s.components);
                run.set_encoding(v1::SAMPLE_ENCODING_F64_LE);
                run.set_first_index(s.firstPending);
                run.set_mode(v1::SignalSubscription::MODE_FULL);
                // The bytes of IEEE doubles on a little-endian host (amd64, arm64).
                run.set_samples(reinterpret_cast<const char*>(s.pending.data()),
                                s.pending.size() * sizeof(double));
                s.firstPending += count;
                s.pending.clear();
            }
            if (block.runs_size() > 0) {
                push(channels.at("/lagrange/signals/" + std::to_string(dt) + "ns"), t, block);
            }
        }
    }
};

Recorder::Recorder() : impl_(std::make_unique<Impl>()) {}

Recorder::~Recorder() {
    if (impl_->thread.joinable()) {
        {
            const std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->closing = true;
        }
        impl_->wake.notify_one();
        impl_->thread.join();
        impl_->writer.close();
    }
}

Expected<std::unique_ptr<Recorder>, std::string> Recorder::create(const std::string& path) {
    std::unique_ptr<Recorder> recorder(new Recorder());
    Impl& s = *recorder->impl_;
    s.path = path;
    mcap::McapWriterOptions options("");
    options.compression = mcap::Compression::Zstd;
    options.library = "lagrange_server";
    if (const mcap::Status status = s.writer.open(path, options); !status.ok()) {
        return Unexpected{"cannot record to " + path + ": " + status.message};
    }
    s.thread = std::thread([&s] { s.run(); });
    return recorder;
}

void Recorder::begin(const record::v1::RunMeta& meta, const v1::SignalCatalog& catalog,
                     const v1::SceneSnapshot& snapshot) {
    Impl& s = *impl_;
    s.signals.assign(static_cast<std::size_t>(catalog.signals_size()), {});
    for (const v1::SignalInfo& info : catalog.signals()) {
        Impl::Signal& signal = s.signals.at(info.id());
        signal.dtNs = std::max<std::int64_t>(info.native_dt_ns(), 1);
        signal.components = std::max<std::uint32_t>(info.components(), 1);
        s.groups[signal.dtNs].push_back(info.id());
    }
    // Every channel is registered here, before the writer thread gets a message: the MCAP
    // writer is not thread-safe, and after this only the writer thread touches it.
    const mcap::ChannelId metaChannel =
        s.channel("/lagrange/meta", record::v1::RunMeta::descriptor());
    const mcap::ChannelId catalogChannel =
        s.channel("/lagrange/catalog", v1::SignalCatalog::descriptor());
    const mcap::ChannelId snapshotChannel =
        s.channel("/lagrange/scene/snapshot", v1::SceneSnapshot::descriptor());
    for (const auto& [dt, members] : s.groups) {
        (void)members;
        s.channel("/lagrange/signals/" + std::to_string(dt) + "ns", v1::SignalBlock::descriptor());
    }
    s.channel("/lagrange/state_hash", record::v1::StateHash::descriptor());
    s.push(metaChannel, 0, meta);
    s.push(catalogChannel, 0, catalog);
    s.push(snapshotChannel, 0, snapshot);
}

void Recorder::add(std::uint32_t id, std::uint64_t index, std::span<const double> values) {
    Impl::Signal& s = impl_->signals.at(id);
    if (index < s.next || values.size() != s.components) {
        return;
    }
    // Missing samples (written later than expected) are NaN, so every run stays contiguous.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (; s.next < index; ++s.next) {
        s.pending.insert(s.pending.end(), s.components, nan);
    }
    s.pending.insert(s.pending.end(), values.begin(), values.end());
    ++s.next;
}

void Recorder::stateHash(SimTime t, std::uint64_t hash) {
    record::v1::StateHash message;
    message.mutable_sim_time()->set_ns(t.ns);
    message.set_hash(hash);
    impl_->push(impl_->channels.at("/lagrange/state_hash"), t.ns, message);
}

void Recorder::flush(SimTime now) {
    Impl& s = *impl_;
    while (now.ns >= s.flushedNs + kBlockNs) {
        s.writeBlocks(s.flushedNs);
        s.flushedNs += kBlockNs;
    }
}

Expected<void, std::string> Recorder::finish(SimTime end, record::v1::RunMeta meta) {
    Impl& s = *impl_;
    s.writeBlocks(s.flushedNs);
    meta.mutable_duration()->set_ns(end.ns);
    // A second RunMeta, now with the duration: readers keep the last one.
    s.push(s.channels.at("/lagrange/meta"), end.ns, meta);
    {
        const std::lock_guard<std::mutex> lock(s.mutex);
        s.closing = true;
    }
    s.wake.notify_one();
    s.thread.join();
    s.writer.close();
    if (!s.error.empty()) {
        return Unexpected{"recording " + s.path + ": " + s.error};
    }
    return {};
}

const std::string& Recorder::path() const {
    return impl_->path;
}

}  // namespace lagrange::server
