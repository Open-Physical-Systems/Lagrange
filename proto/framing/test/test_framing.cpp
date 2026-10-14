#include "FrameReaderHarness.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::proto::appendFrame;
using lagrange::proto::FrameReader;
using lagrange::proto::kFrameHeaderSize;
using lagrange::proto::parsePayload;
using Status = lagrange::proto::FrameReader::Status;
namespace v1 = lagrange::viewer::v1;

std::vector<std::byte> bytes(std::string_view text) {
    std::vector<std::byte> out(text.size());
    std::transform(text.begin(), text.end(), out.begin(),
                   [](char c) { return static_cast<std::byte>(c); });
    return out;
}

std::string text(std::span<const std::byte> data) {
    std::string out(data.size(), '\0');
    std::transform(data.begin(), data.end(), out.begin(),
                   [](std::byte b) { return static_cast<char>(b); });
    return out;
}

TEST(AppendFrame, WritesLittleEndianLengthThenPayload) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("abc"), stream));
    ASSERT_EQ(stream.size(), kFrameHeaderSize + 3);
    EXPECT_EQ(stream[0], std::byte{3});
    EXPECT_EQ(stream[1], std::byte{0});
    EXPECT_EQ(stream[2], std::byte{0});
    EXPECT_EQ(stream[3], std::byte{0});
    EXPECT_EQ(text(std::span{stream}.subspan(kFrameHeaderSize)), "abc");
}

TEST(AppendFrame, LengthUsesAllFourBytes) {
    const std::vector<std::byte> payload(0x01'02'03, std::byte{0x5A});
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(payload, stream));
    EXPECT_EQ(stream[0], std::byte{0x03});
    EXPECT_EQ(stream[1], std::byte{0x02});
    EXPECT_EQ(stream[2], std::byte{0x01});
    EXPECT_EQ(stream[3], std::byte{0x00});
}

TEST(AppendFrame, RejectsOversizePayloadAndLeavesOutputUnchanged) {
    std::vector<std::byte> stream = bytes("keep");
    EXPECT_FALSE(appendFrame(bytes("12345"), stream, 4));
    EXPECT_EQ(text(stream), "keep");
    EXPECT_TRUE(appendFrame(bytes("1234"), stream, 4));
}

TEST(FrameReader, ReturnsNeedMoreOnEmptyInput) {
    FrameReader reader;
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    EXPECT_EQ(reader.bufferedBytes(), 0U);
}

TEST(FrameReader, ReadsSeveralFramesFromOneFeed) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("one"), stream));
    ASSERT_TRUE(appendFrame(bytes(""), stream));  // zero-length frames are valid
    ASSERT_TRUE(appendFrame(bytes("three"), stream));

    FrameReader reader;
    reader.feed(stream);

    auto r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    EXPECT_EQ(text(r.payload), "one");
    r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    EXPECT_TRUE(r.payload.empty());
    r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    EXPECT_EQ(text(r.payload), "three");
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    EXPECT_EQ(reader.bufferedBytes(), 0U);
}

TEST(FrameReader, HandlesStreamDeliveredOneByteAtATime) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("hello"), stream));
    ASSERT_TRUE(appendFrame(bytes("world!"), stream));

    FrameReader reader;
    std::vector<std::string> frames;
    for (const std::byte b : stream) {
        reader.feed(std::span{&b, 1});
        for (auto r = reader.next(); r.status == Status::Frame; r = reader.next()) {
            frames.push_back(text(r.payload));
        }
    }
    EXPECT_EQ(frames, (std::vector<std::string>{"hello", "world!"}));
}

TEST(FrameReader, WaitsWhenHeaderOrPayloadIsSplit) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("payload"), stream));
    const std::span<const std::byte> all{stream};

    FrameReader reader;
    reader.feed(all.first(2));  // half a header
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    reader.feed(all.subspan(2, 5));  // rest of header + 3 payload bytes
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    EXPECT_EQ(reader.bufferedBytes(), 7U);
    reader.feed(all.subspan(7));
    const auto r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    EXPECT_EQ(text(r.payload), "payload");
}

TEST(FrameReader, TruncatedStreamNeverYieldsAFrame) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("truncated"), stream));
    stream.pop_back();

    FrameReader reader;
    reader.feed(stream);
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    EXPECT_EQ(reader.next().status, Status::NeedMore);
    EXPECT_EQ(reader.error(), FrameReader::Error::None);
}

TEST(FrameReader, AcceptsFrameExactlyAtTheLimit) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("12345678"), stream, 8));
    FrameReader reader(8);
    reader.feed(stream);
    const auto r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    EXPECT_EQ(r.payload.size(), 8U);
}

TEST(FrameReader, OversizeHeaderIsAStickyErrorAndBuffersNothing) {
    // Header announcing 4 GiB - 1. The reader must fail from the header alone.
    const std::vector<std::byte> header(4, std::byte{0xFF});
    FrameReader reader(1024);
    reader.feed(header);
    EXPECT_EQ(reader.next().status, Status::Error);
    EXPECT_EQ(reader.error(), FrameReader::Error::Oversize);

    // Later input is ignored: no recovery, no unbounded buffering.
    std::vector<std::byte> valid;
    ASSERT_TRUE(appendFrame(bytes("ok"), valid));
    reader.feed(valid);
    EXPECT_EQ(reader.next().status, Status::Error);
    EXPECT_EQ(reader.bufferedBytes(), 0U);
}

TEST(FrameReader, OneOverTheLimitIsRejected) {
    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(bytes("123456789"), stream));  // 9 bytes, default limit
    FrameReader reader(8);
    reader.feed(stream);
    EXPECT_EQ(reader.next().status, Status::Error);
}

TEST(FrameReader, LongStreamWithRandomChunkingKeepsEveryFrameIntact) {
    // Enough data to trigger buffer compaction many times.
    std::mt19937 rng(20261013);  // fixed seed: the test is deterministic
    std::vector<std::string> sent;
    std::vector<std::byte> stream;
    for (int i = 0; i < 2000; ++i) {
        std::string payload(rng() % 700, '\0');
        for (char& c : payload) {
            c = static_cast<char>(rng());
        }
        ASSERT_TRUE(appendFrame(bytes(payload), stream));
        sent.push_back(std::move(payload));
    }

    FrameReader reader;
    std::vector<std::string> received;
    std::span<const std::byte> rest{stream};
    while (!rest.empty()) {
        const std::size_t n = std::min<std::size_t>(rng() % 5000 + 1, rest.size());
        reader.feed(rest.first(n));
        rest = rest.subspan(n);
        for (auto r = reader.next(); r.status == Status::Frame; r = reader.next()) {
            received.push_back(text(r.payload));
        }
    }
    EXPECT_EQ(received, sent);
    EXPECT_EQ(reader.bufferedBytes(), 0U);
}

TEST(Envelope, RoundTripsThroughAFrame) {
    v1::Envelope hello;
    hello.mutable_client_hello()->mutable_version()->set_minor(1);
    hello.mutable_client_hello()->set_client_name("lagrange_viewer 0.1.0");
    v1::Envelope ping;
    ping.mutable_ping()->set_nonce(77);
    const v1::Envelope empty;  // serialises to zero bytes

    std::vector<std::byte> stream;
    ASSERT_TRUE(appendFrame(hello, stream));
    ASSERT_TRUE(appendFrame(empty, stream));
    ASSERT_TRUE(appendFrame(ping, stream));

    FrameReader reader;
    reader.feed(stream);
    v1::Envelope parsed;

    auto r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    ASSERT_TRUE(parsePayload(r.payload, parsed));
    EXPECT_EQ(parsed.client_hello().client_name(), "lagrange_viewer 0.1.0");

    r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    ASSERT_TRUE(parsePayload(r.payload, parsed));
    EXPECT_EQ(parsed.body_case(), v1::Envelope::BODY_NOT_SET);

    r = reader.next();
    ASSERT_EQ(r.status, Status::Frame);
    ASSERT_TRUE(parsePayload(r.payload, parsed));
    EXPECT_EQ(parsed.ping().nonce(), 77U);
}

TEST(Envelope, MessageLargerThanLimitIsNotFramed) {
    v1::Envelope big;
    big.mutable_blob()->set_data(std::string(100, 'x'));
    std::vector<std::byte> stream;
    EXPECT_FALSE(appendFrame(big, stream, 50));
    EXPECT_TRUE(stream.empty());
}

TEST(ParsePayload, RejectsBytesThatAreNotAMessage) {
    v1::Envelope parsed;
    // Field 1, wire type 2 (length-delimited), announced length 100, but only 1 byte follows.
    const std::vector<std::byte> garbage = {std::byte{0x0A}, std::byte{0x64}, std::byte{0x00}};
    EXPECT_FALSE(parsePayload(garbage, parsed));
}

TEST(FrameReaderHarness, SurvivesPseudoRandomStreams) {
    // The fuzz harness on fixed-seed random input. Under ASan/UBSan this catches the same
    // classes of bug as the fuzzer, on every CI run, without needing Clang.
    std::mt19937 rng(42);
    std::vector<std::byte> data;
    for (int round = 0; round < 3000; ++round) {
        data.resize(rng() % 600);
        for (std::byte& b : data) {
            // Bias towards small values so that plausible headers appear often.
            b = static_cast<std::byte>(rng() % 4 == 0 ? rng() % 256 : rng() % 8);
        }
        lagrange::proto::fuzz::runFrameReader(data);
    }
    SUCCEED();
}

TEST(FrameReaderHarness, ExtractsFramesFromAValidStream) {
    std::vector<std::byte> stream = {std::byte{7}};  // chunk-size selector
    v1::Envelope ping;
    ping.mutable_ping()->set_nonce(1);
    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(appendFrame(ping, stream));
    }
    EXPECT_EQ(lagrange::proto::fuzz::runFrameReader(stream), 10U);
}

}  // namespace
