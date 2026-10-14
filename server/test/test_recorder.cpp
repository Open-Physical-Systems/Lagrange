// Recording a run (roadmap 0.2-E4): the file has the channels the mock server's reader needs,
// every signal is contiguous from t = 0, and the final RunMeta carries the duration.
#include "lagrange/record/v1/record.pb.h"
#include "lagrange/server/Server.hpp"
#include "lagrange/viewer/v1/scene.pb.h"
#include "lagrange/viewer/v1/signals.pb.h"

#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <mcap/reader.hpp>

namespace lagrange::server {
namespace {

namespace v1 = lagrange::viewer::v1;

TEST(Recorder, ARunIsRecordedAsTheMockServerReadsIt) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_recorder";
    std::filesystem::create_directories(dir);
    const std::filesystem::path file = dir / "run.mcap";
    std::filesystem::remove(file);
    {
        ServerOptions options;
        options.endpoint = proto::Endpoint::unixSocket((dir / "s.sock").string());
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.realTimeFactor = 0.0;  // as fast as possible
        options.recordPath = file.string();
        Server server(options);
        const Status started = server.start();
        ASSERT_TRUE(started.ok()) << started.message;
        while (server.simTime().ns < 1'250'000'000) {
            server.runOnce(0);
        }
    }  // stopping the server closes the recording

    mcap::McapReader reader;
    ASSERT_TRUE(reader.open(file.string()).ok());
    std::map<std::string, int> topics;
    record::v1::RunMeta meta;
    v1::SignalCatalog catalog;
    bool snapshot = false;
    std::map<std::uint32_t, std::uint64_t> next;  // signal -> next expected sample index
    int hashes = 0;
    for (const mcap::MessageView& view : reader.readMessages()) {
        const std::string& topic = view.channel->topic;
        ++topics[topic];
        const auto* data = reinterpret_cast<const char*>(view.message.data);
        const auto size = static_cast<int>(view.message.dataSize);
        if (topic == "/lagrange/meta") {
            ASSERT_TRUE(meta.ParseFromArray(data, size));
        } else if (topic == "/lagrange/catalog") {
            ASSERT_TRUE(catalog.ParseFromArray(data, size));
        } else if (topic == "/lagrange/scene/snapshot") {
            v1::SceneSnapshot message;
            ASSERT_TRUE(message.ParseFromArray(data, size));
            snapshot = message.prims_size() > 0;
        } else if (topic == "/lagrange/state_hash") {
            ++hashes;
        } else if (topic.rfind("/lagrange/signals/", 0) == 0) {
            v1::SignalBlock block;
            ASSERT_TRUE(block.ParseFromArray(data, size));
            for (const v1::SignalRun& run : block.runs()) {
                EXPECT_EQ(run.first_index(), next[run.signal()])
                    << "signal " << run.signal() << ": gap or overlap";
                EXPECT_EQ(run.samples().size(),
                          static_cast<std::size_t>(run.sample_count()) * run.components() * 8);
                next[run.signal()] += run.sample_count();
            }
        }
    }
    reader.close();

    EXPECT_EQ(topics["/lagrange/meta"], 2) << "at the start, and with the duration at the end";
    EXPECT_EQ(meta.format_version(), 1U);
    EXPECT_GE(meta.duration().ns(), 1'250'000'000);
    EXPECT_EQ(meta.stage_path(), LAGRANGE_TEST_AMR_URDF);
    EXPECT_TRUE(snapshot);
    EXPECT_GE(hashes, 2);  // t = 0 and t = 1 s
    ASSERT_GT(catalog.signals_size(), 0);
    std::set<std::string> names;
    for (const v1::SignalInfo& info : catalog.signals()) {
        names.insert(info.name());
        EXPECT_LE(info.components(), 1024U) << info.name() << " should have been left out";
        // Every recorded signal has its samples from t = 0 to the end of the run.
        const std::uint64_t expected =
            static_cast<std::uint64_t>(meta.duration().ns() / info.native_dt_ns()) + 1;
        EXPECT_NEAR(static_cast<double>(next[info.id()]), static_cast<double>(expected), 1.0)
            << info.name();
    }
    EXPECT_TRUE(names.contains("joint/left_wheel_joint/position"));
    EXPECT_FALSE(names.contains("sensor/top_lidar/ranges")) << "14400 components: left out";
    // Every moving prim is bound to a recorded pose signal, which is how a replay moves it.
    ASSERT_GT(meta.pose_bindings_size(), 0);
    for (const record::v1::PoseBinding& binding : meta.pose_bindings()) {
        EXPECT_TRUE(names.contains(binding.signal())) << binding.signal();
    }
}

/// The state hashes a recorded run wrote, in time order.
std::vector<std::pair<std::int64_t, std::uint64_t>> recordedHashes(
    const std::filesystem::path& file, std::int64_t untilNs) {
    {
        ServerOptions options;
        options.endpoint = proto::Endpoint::unixSocket(file.string() + ".sock");
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.realTimeFactor = 0.0;
        options.recordPath = file.string();
        Server server(options);
        EXPECT_TRUE(server.start().ok());
        while (server.simTime().ns < untilNs) {
            server.runOnce(0);
        }
    }
    std::vector<std::pair<std::int64_t, std::uint64_t>> out;
    mcap::McapReader reader;
    EXPECT_TRUE(reader.open(file.string()).ok());
    for (const mcap::MessageView& view : reader.readMessages()) {
        if (view.channel->topic == "/lagrange/state_hash") {
            record::v1::StateHash hash;
            EXPECT_TRUE(
                hash.ParseFromArray(view.message.data, static_cast<int>(view.message.dataSize)));
            out.emplace_back(hash.sim_time().ns(), hash.hash());
        }
    }
    reader.close();
    return out;
}

// v0.2 acceptance criterion 5: the demo (MuJoCo, both lidars with noise, the IMU, the drive
// controller) gives bit-identical state hashes on two runs for 10 simulated seconds. Without
// the ROS bridge the run takes no outside input, so the scheduler runs it in deterministic
// mode. (That a nondeterministic module is caught is tested in core/scheduler.)
TEST(Determinism, TheDemoRunsBitIdenticallyTwice) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_determinism";
    std::filesystem::create_directories(dir);
    const auto first = recordedHashes(dir / "a.mcap", 10'000'000'000);
    const auto second = recordedHashes(dir / "b.mcap", 10'000'000'000);
    ASSERT_GE(first.size(), 11U) << "a hash a second from t = 0";
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(second[i].first, first[i].first);
        EXPECT_EQ(second[i].second, first[i].second)
            << "diverged at t = " << first[i].first << " ns";
    }
    // And the run does something: the state changes over time.
    EXPECT_NE(first.front().second, first.back().second);
}

}  // namespace
}  // namespace lagrange::server
