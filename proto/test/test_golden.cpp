// The committed golden frames, read by the C++ decoder.
//
// frames.bin holds one frame per Envelope body type with every field set (see
// golden/generate_golden.py). Parsing a frame and serialising it again must give the same
// bytes; together with the Python test doing the same, this shows that both languages decode
// every field alike.

#include "lagrange/proto/Framing.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <gtest/gtest.h>

namespace {

namespace v1 = lagrange::viewer::v1;
using lagrange::proto::FrameReader;

std::vector<std::string> goldenPayloads() {
    std::ifstream file(std::string(LAGRANGE_GOLDEN_DIR) + "/frames.bin", std::ios::binary);
    EXPECT_TRUE(file.good()) << "cannot open " << LAGRANGE_GOLDEN_DIR << "/frames.bin";
    const std::string data((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

    FrameReader reader;
    reader.feed(std::as_bytes(std::span{data}));
    std::vector<std::string> payloads;
    for (auto r = reader.next(); r.status == FrameReader::Status::Frame; r = reader.next()) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): bytes to chars.
        payloads.emplace_back(reinterpret_cast<const char*>(r.payload.data()), r.payload.size());
    }
    EXPECT_EQ(reader.error(), FrameReader::Error::None);
    EXPECT_EQ(reader.bufferedBytes(), 0U) << "trailing bytes after the last frame";
    return payloads;
}

TEST(GoldenFrames, EveryFrameReserialisesToTheSameBytes) {
    const std::vector<std::string> payloads = goldenPayloads();
    const auto* body = v1::Envelope::descriptor()->FindOneofByName("body");
    ASSERT_NE(body, nullptr);
    ASSERT_EQ(payloads.size(), static_cast<std::size_t>(body->field_count()) + 1);

    for (int i = 0; i < body->field_count(); ++i) {
        SCOPED_TRACE(body->field(i)->name());
        const std::string& payload = payloads[static_cast<std::size_t>(i)];
        v1::Envelope envelope;
        ASSERT_TRUE(envelope.ParseFromString(payload));
        EXPECT_EQ(static_cast<int>(envelope.body_case()), body->field(i)->number());
        EXPECT_EQ(envelope.SerializeAsString(), payload);
    }
    EXPECT_TRUE(payloads.back().empty());  // the Envelope with no body
}

TEST(GoldenFrames, KnownValues) {
    const std::vector<std::string> payloads = goldenPayloads();
    ASSERT_FALSE(payloads.empty());
    v1::Envelope envelope;
    ASSERT_TRUE(envelope.ParseFromString(payloads.front()));
    const v1::ClientHello& hello = envelope.client_hello();
    EXPECT_EQ(hello.version().major(), 11U);
    EXPECT_EQ(hello.version().minor(), 21U);
    EXPECT_EQ(hello.client_name(), "左車輪_21");
    ASSERT_EQ(hello.compression_size(), 3);
    EXPECT_EQ(hello.compression(2), "左車輪_43");
}

}  // namespace
