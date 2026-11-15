// Comparing two recordings (Replay: Compare, Sim vs real, Validation): reading, resampling,
// alignment by cross-correlation, the metrics, suites and driving phases.
#include "lagrange/record/v1/record.pb.h"
#include "lagrange/viewer/CompareLog.hpp"
#include "lagrange/viewer/v1/signals.pb.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <numbers>
#include <string>

#include <gtest/gtest.h>
#include <mcap/writer.hpp>

namespace {

namespace v1 = lagrange::viewer::v1;
namespace cmp = lagrange::viewer::compare;

constexpr std::int64_t kMs = 1'000'000;

/// A recording with one scalar "wave" (1 kHz), a scalar "flag" (always 1), a 6-component
/// "body/base/twist" and one more scalar, `extra`; `wave(t)` gives the wave.
void writeRecording(const std::filesystem::path& path, const std::function<double(double)>& wave,
                    const std::string& extra, double seconds = 2.0) {
    mcap::McapWriter writer;
    mcap::McapWriterOptions options("");
    ASSERT_TRUE(writer.open(path.string(), options).ok());
    std::map<std::string, mcap::ChannelId> channels;
    const auto channel = [&](const std::string& topic) {
        if (!channels.contains(topic)) {
            mcap::Channel out(topic, "protobuf", 0);
            writer.addChannel(out);
            channels[topic] = out.id;
        }
        return channels[topic];
    };
    std::uint32_t sequence = 0;
    const auto put = [&](const std::string& topic, std::int64_t t,
                         const google::protobuf::Message& m) {
        const std::string bytes = m.SerializeAsString();
        mcap::Message message;
        message.channelId = channel(topic);
        message.sequence = ++sequence;
        message.logTime = static_cast<mcap::Timestamp>(t);
        message.publishTime = message.logTime;
        message.data = reinterpret_cast<const std::byte*>(bytes.data());
        message.dataSize = bytes.size();
        ASSERT_TRUE(writer.write(message).ok());
    };
    lagrange::record::v1::RunMeta meta;
    meta.set_format_version(1);
    meta.set_generator("test");
    meta.set_stage_path("robot.urdf");
    meta.mutable_duration()->set_ns(static_cast<std::int64_t>(seconds * 1e9));
    put("/lagrange/meta", 0, meta);
    v1::SignalCatalog catalog;
    const auto info = [&](std::uint32_t id, const std::string& name, std::uint32_t components) {
        v1::SignalInfo* s = catalog.add_signals();
        s->set_id(id);
        s->set_name(name);
        s->set_components(components);
        s->set_native_dt_ns(kMs);
        s->set_writer("test/writer");
        s->mutable_unit()->set_display("A");
        if (components == 6) {
            for (const char* c : {"x", "y", "z", "rx", "ry", "rz"}) {
                s->add_component_names(c);
            }
        }
    };
    info(0, "wave", 1);
    info(1, "flag", 1);
    info(2, "body/base/twist", 6);
    info(3, extra, 1);
    put("/lagrange/catalog", 0, catalog);
    const auto steps = static_cast<std::uint32_t>(seconds * 1000.0);
    v1::SignalBlock block;
    const auto run = [&](std::uint32_t id, std::uint32_t components,
                         const std::function<double(double, std::uint32_t)>& value) {
        v1::SignalRun* r = block.add_runs();
        r->set_signal(id);
        r->mutable_t0()->set_ns(0);
        r->set_dt_ns(kMs);
        r->set_sample_count(steps);
        r->set_components(components);
        r->set_encoding(v1::SAMPLE_ENCODING_F64_LE);
        std::string bytes;
        for (std::uint32_t k = 0; k < steps; ++k) {
            for (std::uint32_t c = 0; c < components; ++c) {
                const double v = value(static_cast<double>(k) * 1e-3, c);
                bytes.append(reinterpret_cast<const char*>(&v), sizeof(v));
            }
        }
        r->set_samples(bytes);
    };
    run(0, 1, [&](double t, std::uint32_t) { return wave(t); });
    run(1, 1, [](double, std::uint32_t) { return 1.0; });
    // Stands for 0.2 s, speeds up for 0.5 s, cruises at 1 m/s, stops after 1.5 s.
    run(2, 6, [](double t, std::uint32_t c) {
        if (c != 0 || t < 0.2) {
            return 0.0;
        }
        return t < 0.7 ? 2.0 * (t - 0.2) : (t < 1.5 ? 1.0 : std::max(0.0, 1.0 - 4.0 * (t - 1.5)));
    });
    run(3, 1, [](double t, std::uint32_t) { return t; });
    put("/lagrange/signals/1000000", 0, block);
    writer.close();
}

class CompareLogTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("lgr-compare-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                "-" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::create_directories(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }
    std::filesystem::path dir_;
};

double sine(double t) {
    return 10.0 * std::sin(2.0 * std::numbers::pi * 1.3 * t);
}

TEST_F(CompareLogTest, ReadsAndResamplesARecording) {
    writeRecording(dir_ / "sim.mcap", sine, "motor/left/current");
    const cmp::LoadResult loaded = cmp::loadLog((dir_ / "sim.mcap").string(), 10 * kMs);
    ASSERT_TRUE(loaded.log) << loaded.error;
    const cmp::LogFile& log = *loaded.log;
    EXPECT_EQ(log.stagePath, "robot.urdf");
    EXPECT_EQ(log.durationNs, 2'000'000'000);
    ASSERT_NE(log.find("wave"), nullptr);
    EXPECT_EQ(log.find("wave")->count(), 200U);  // 2 s at 100 Hz
    // Bin 0 is the mean of t = 0..9 ms.
    double mean = 0.0;
    for (int k = 0; k < 10; ++k) {
        mean += sine(k * 1e-3) / 10.0;
    }
    EXPECT_NEAR(log.find("wave")->at(0, 0), mean, 1e-12);
    EXPECT_EQ(log.find("body/base/twist")->componentNames.size(), 6U);

    const cmp::LoadResult missing = cmp::loadLog((dir_ / "none.mcap").string(), 10 * kMs);
    EXPECT_FALSE(missing.log);
    EXPECT_NE(missing.error.find("no such file"), std::string::npos);
}

TEST_F(CompareLogTest, FindsTheLagAndTheErrorOfAShiftedCopy) {
    writeRecording(dir_ / "sim.mcap", sine, "a");
    // The "real" run: the same wave 40 ms later, offset by 0.2.
    writeRecording(dir_ / "real.mcap", [](double t) { return sine(t - 0.04) + 0.2; }, "a");
    const auto sim = cmp::loadLog((dir_ / "sim.mcap").string(), 10 * kMs).log;
    const auto real = cmp::loadLog((dir_ / "real.mcap").string(), 10 * kMs).log;
    ASSERT_TRUE(sim && real);
    cmp::Method method;
    const cmp::Result r = cmp::compareSignals(*sim, *real, {"wave", "wave", 0, 0.05}, method, {});
    ASSERT_TRUE(r.valid);
    EXPECT_EQ(r.lagNs, 40 * kMs);
    EXPECT_NEAR(r.rmse, 0.2, 1e-9);  // only the offset remains once aligned
    EXPECT_NEAR(r.maxAbs, 0.2, 1e-9);
    EXPECT_GT(r.range, 19.0);
    EXPECT_NEAR(r.nrmse, 0.2 / r.range, 1e-12);
    EXPECT_TRUE(r.pass);

    // Without alignment the 40 ms shift is error too.
    method.align = false;
    const cmp::Result raw = cmp::compareSignals(*sim, *real, {"wave", "wave", 0, 0.01}, method, {});
    EXPECT_EQ(raw.lagNs, 0);
    EXPECT_GT(raw.rmse, 2.0);
    EXPECT_FALSE(raw.pass);

    // A window: only 0.5 s to 1.0 s.
    method.windowBeginNs = 500 * kMs;
    method.windowEndNs = 1000 * kMs;
    const cmp::Result window =
        cmp::compareSignals(*sim, *real, {"wave", "wave", 0, 0.05}, method, {});
    EXPECT_EQ(window.samples, 50U);
}

TEST_F(CompareLogTest, SuitesSharedAndFromAFile) {
    writeRecording(dir_ / "sim.mcap", sine, "motor/left_drive_motor/current_q");
    writeRecording(dir_ / "real.mcap", sine, "motor/left_drive_motor/iq");
    const auto sim = cmp::loadLog((dir_ / "sim.mcap").string(), 10 * kMs).log;
    const auto real = cmp::loadLog((dir_ / "real.mcap").string(), 10 * kMs).log;
    ASSERT_TRUE(sim && real);
    // Shared: the wave only. The flag is constant in both, the twist is not a scalar, the
    // motor currents have different names.
    const cmp::Suite shared = cmp::sharedSuite(*sim, *real, 0.05);
    ASSERT_EQ(shared.mappings.size(), 1U);
    EXPECT_EQ(shared.mappings[0].sim, "wave");
    const std::vector<cmp::Unmapped> rest = cmp::unmapped(*sim, *real, shared);
    bool suggested = false;
    for (const cmp::Unmapped& u : rest) {
        if (u.real == "motor/left_drive_motor/iq") {
            EXPECT_EQ(u.suggestion, "motor/left_drive_motor/current_q");
            suggested = true;
        }
    }
    EXPECT_TRUE(suggested);

    std::ofstream(dir_ / "suite.yaml") << "suite: drive\n"
                                          "method: {alignment: none, resample_hz: 50, "
                                          "window: [0.2, 1.8]}\n"
                                          "signals:\n"
                                          "  - {sim: motor/left_drive_motor/current_q, "
                                          "real: motor/left_drive_motor/iq, limit: 0.02}\n"
                                          "  - {sim: body/base/twist, component: 0, limit: 0.03}\n";
    const cmp::Suite suite = cmp::loadSuite((dir_ / "suite.yaml").string());
    ASSERT_TRUE(suite.error.empty()) << suite.error;
    EXPECT_EQ(suite.name, "drive");
    EXPECT_FALSE(suite.method.align);
    EXPECT_EQ(suite.periodNs, 20 * kMs);
    EXPECT_EQ(suite.method.windowEndNs, 1800 * kMs);
    ASSERT_EQ(suite.mappings.size(), 2U);
    EXPECT_EQ(suite.mappings[1].real, "body/base/twist");
    const cmp::Report report = cmp::runSuite(*sim, *real, suite);
    ASSERT_EQ(report.results.size(), 2U);
    EXPECT_EQ(report.results[1].sim, "body/base/twist.x");
    EXPECT_EQ(report.passed(), 2U);  // identical runs
    const std::string json = cmp::toJson(report, suite);
    EXPECT_NE(json.find("\"passed\": 2"), std::string::npos);
    EXPECT_NE(json.find("\"body/base/twist.x\""), std::string::npos);

    std::ofstream(dir_ / "bad.yaml") << "signals: []\n";
    EXPECT_FALSE(cmp::loadSuite((dir_ / "bad.yaml").string()).error.empty());
}

TEST_F(CompareLogTest, DrivingPhasesFollowTheBaseSpeed) {
    writeRecording(dir_ / "sim.mcap", sine, "a");
    const auto sim = cmp::loadLog((dir_ / "sim.mcap").string(), 10 * kMs).log;
    ASSERT_TRUE(sim);
    const std::vector<cmp::Phase> phases = cmp::drivingPhases(*sim);
    ASSERT_EQ(phases.size(), 200U);
    EXPECT_EQ(phases[5], cmp::Phase::Stopped);      // 0.05 s: standing
    EXPECT_EQ(phases[45], cmp::Phase::Accelerate);  // 0.45 s: 2 m/s²
    EXPECT_EQ(phases[100], cmp::Phase::Cruise);     // 1.0 s: 1 m/s
    EXPECT_EQ(phases[160], cmp::Phase::Brake);      // 1.6 s: −4 m/s²
    EXPECT_EQ(phases[195], cmp::Phase::Stopped);    // 1.95 s: stopped
}

}  // namespace
