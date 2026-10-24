// Reading a recording and replaying one domain of it, on a small file written here as
// `lagrange_server --record` writes them (docs/design/formats.md §7).
#include "lagrange/record/v1/record.pb.h"
#include "lagrange/recording/Recording.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/viewer/v1/signals.pb.h"

#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mcap/writer.hpp>

namespace lagrange::recording {
namespace {

using namespace literals;
namespace v1 = lagrange::viewer::v1;

/// A recording of 50 ms: the joint speed (1 ms, rad/s, by rigid_body/mujoco) is 10·k at sample
/// k, its first two samples missing; a pose (5 ms); and a bus voltage by the electrical network,
/// which a rigid-body replay must leave out.
std::string writeRecording(const std::string& name) {
    const std::string path = (std::filesystem::path(::testing::TempDir()) / name).string();
    mcap::McapWriter writer;
    EXPECT_TRUE(writer.open(path, mcap::McapWriterOptions("")).ok());
    std::uint16_t next = 0;
    const auto channel = [&](const std::string& topic) {
        mcap::Schema schema("x", "protobuf", std::string());
        writer.addSchema(schema);
        mcap::Channel c(topic, "protobuf", schema.id);
        writer.addChannel(c);
        ++next;
        return c.id;
    };
    const auto put = [&](mcap::ChannelId id, const google::protobuf::Message& m) {
        const std::string bytes = m.SerializeAsString();
        mcap::Message message;
        message.channelId = id;
        message.data = reinterpret_cast<const std::byte*>(bytes.data());
        message.dataSize = bytes.size();
        EXPECT_TRUE(writer.write(message).ok());
    };
    record::v1::RunMeta meta;
    meta.set_stage_path("robots/test.urdf");
    meta.mutable_duration()->set_ns(50'000'000);
    put(channel("/lagrange/meta"), meta);

    v1::SignalCatalog catalog;
    const auto info = [&](std::uint32_t id, const char* signal, int type, int components,
                          std::int64_t dt, const char* writerName) {
        v1::SignalInfo* i = catalog.add_signals();
        i->set_id(id);
        i->set_name(signal);
        i->set_type(static_cast<v1::SignalType>(type + 1));
        i->set_components(static_cast<std::uint32_t>(components));
        i->set_native_dt_ns(dt);
        i->set_writer(writerName);
        return i->mutable_unit();
    };
    v1::Unit* speed = info(0, "joint/axle/velocity", 0, 1, 1'000'000, "rigid_body/mujoco");
    speed->set_time(-1);
    speed->set_angle(1);
    speed->set_kind(static_cast<std::uint32_t>(units::rad_per_s.kind));  // as the recorder sets it
    info(1, "body/base/pose", static_cast<int>(SignalType::Pose), 7, 5'000'000, "rigid_body/mujoco")
        ->set_length(1);
    v1::Unit* volts = info(2, "bus/dc/voltage", 0, 1, 100'000, "electrical/averaged_network");
    volts->set_mass(1);
    put(channel("/lagrange/catalog"), catalog);

    const mcap::ChannelId blocks = channel("/lagrange/signals/1000000ns");
    v1::SignalBlock block;
    v1::SignalRun* run = block.add_runs();
    run->set_signal(0);
    run->set_dt_ns(1'000'000);
    run->set_components(1);
    run->set_encoding(v1::SAMPLE_ENCODING_F64_LE);
    std::vector<double> values;
    for (int k = 0; k <= 50; ++k) {
        values.push_back(k < 2 ? std::numeric_limits<double>::quiet_NaN() : 10.0 * k);
    }
    run->set_sample_count(static_cast<std::uint32_t>(values.size()));
    run->set_samples(values.data(), values.size() * sizeof(double));
    run->mutable_t0()->set_ns(0);
    v1::SignalRun* pose = block.add_runs();
    pose->set_signal(1);
    pose->set_dt_ns(5'000'000);
    pose->set_components(7);
    pose->set_encoding(v1::SAMPLE_ENCODING_F64_LE);
    std::vector<double> poses;
    for (int k = 0; k <= 10; ++k) {
        poses.insert(poses.end(), {0.1 * k, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    }
    pose->set_sample_count(11);
    pose->set_samples(poses.data(), poses.size() * sizeof(double));
    pose->mutable_t0()->set_ns(0);
    put(blocks, block);
    writer.close();
    return path;
}

/// Reads the speed every millisecond.
class Reader final : public IModule {
public:
    std::vector<double> seen;
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "reader", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "joint/axle/velocity";
        spec.unit = units::rad_per_s;
        id_ = d.input(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        seen.push_back(bus.read(id_));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId id_;
};

TEST(Recording, ReadsTheCatalogAndTheSamples) {
    auto recording = Recording::open(writeRecording("read.mcap"));
    ASSERT_TRUE(recording) << recording.error();
    EXPECT_EQ(recording->stagePath, "robots/test.urdf");
    EXPECT_EQ(recording->duration.ns, 50'000'000);
    const RecordedSignal* speed = recording->find("joint/axle/velocity");
    ASSERT_NE(speed, nullptr);
    EXPECT_EQ(speed->unit, units::rad_per_s);
    EXPECT_EQ(speed->sampleCount(), 51U);
    EXPECT_EQ(speed->samples[7], 70.0);
    EXPECT_TRUE(std::isnan(speed->samples[0]));
    EXPECT_EQ(recording->find("body/base/pose")->components, 7U);
    EXPECT_EQ(recording->find("bus/dc/voltage")->writer, "electrical/averaged_network");
    EXPECT_FALSE(Recording::open("/nonexistent.mcap"));
}

TEST(ReplaySubstitute, ReplaysItsDomainSampleForSampleAndHoldsTheEnd) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto replay = std::make_unique<ReplaySubstitute>("rigid_body", writeRecording("replay.mcap"));
    ReplaySubstitute* r = replay.get();
    scheduler.add(std::move(replay));
    auto reader = std::make_unique<Reader>();
    Reader* seen = reader.get();
    scheduler.add(std::move(reader));
    auto wired = scheduler.wire(WorldView(World{}));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    EXPECT_TRUE(r->info().isSubstitute);
    EXPECT_EQ(r->preferredStep(), 1_ms);  // the gcd of 1 ms and 5 ms
    EXPECT_EQ(r->signalNames(),
              (std::vector<std::string>{"joint/axle/velocity", "body/base/pose"}));  // no bus
    ASSERT_TRUE(scheduler.stepUntil(SimTime{60'000'000}).ok());                      // past the end
    ASSERT_EQ(seen->seen.size(), 60U);
    // The step at t commits the sample for t + 1 ms; the reader runs after it in the same tick.
    EXPECT_EQ(seen->seen[0], 0.0);  // samples 0 and 1 are missing: nothing to hold yet
    for (std::size_t k = 2; k < 50; ++k) {
        EXPECT_EQ(seen->seen[k], 10.0 * static_cast<double>(k + 1)) << k;
    }
    EXPECT_EQ(seen->seen.back(), 500.0);  // held after the end
}

TEST(ReplaySubstitute, RefusesAFileWithNothingOfItsDomain) {
    Scheduler scheduler;
    scheduler.add(std::make_unique<ReplaySubstitute>("thermal", writeRecording("other.mcap")));
    auto wired = scheduler.wire(WorldView(World{}));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("no signals written by the thermal domain"),
              std::string::npos);
}

}  // namespace
}  // namespace lagrange::recording
