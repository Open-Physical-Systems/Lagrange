// Round-trip tests for the viewer protocol messages.
//
// Every message reachable from Envelope is filled through reflection with deterministic
// values, serialised, parsed back and compared. New fields and messages are covered without
// touching this file.

#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

namespace {

namespace pb = google::protobuf;
namespace v1 = lagrange::viewer::v1;

constexpr int kRepeatedCount = 3;

// Fills every field of `message` with a value derived from the field number, recursing into
// sub-messages. For a oneof, each member is set in turn, so the last one stays set.
void fill(pb::Message& message, int depth = 0) {
    ASSERT_LT(depth, 16) << "unexpected recursive message type";
    const pb::Descriptor* descriptor = message.GetDescriptor();
    const pb::Reflection* reflection = message.GetReflection();
    for (int i = 0; i < descriptor->field_count(); ++i) {
        const pb::FieldDescriptor* field = descriptor->field(i);
        const int n = field->number();
        const int count = field->is_repeated() ? kRepeatedCount : 1;
        for (int k = 0; k < count; ++k) {
            const int v = n * 10 + k + 1;
            switch (field->cpp_type()) {
                case pb::FieldDescriptor::CPPTYPE_INT32:
                    field->is_repeated() ? reflection->AddInt32(&message, field, -v)
                                         : reflection->SetInt32(&message, field, -v);
                    break;
                case pb::FieldDescriptor::CPPTYPE_INT64:
                    field->is_repeated()
                        ? reflection->AddInt64(&message, field, -(std::int64_t{v} << 40))
                        : reflection->SetInt64(&message, field, -(std::int64_t{v} << 40));
                    break;
                case pb::FieldDescriptor::CPPTYPE_UINT32:
                    field->is_repeated()
                        ? reflection->AddUInt32(&message, field, static_cast<std::uint32_t>(v))
                        : reflection->SetUInt32(&message, field, static_cast<std::uint32_t>(v));
                    break;
                case pb::FieldDescriptor::CPPTYPE_UINT64:
                    field->is_repeated()
                        ? reflection->AddUInt64(&message, field, std::uint64_t{1} << 63 | v)
                        : reflection->SetUInt64(&message, field, std::uint64_t{1} << 63 | v);
                    break;
                case pb::FieldDescriptor::CPPTYPE_DOUBLE:
                    field->is_repeated() ? reflection->AddDouble(&message, field, v + 0.1)
                                         : reflection->SetDouble(&message, field, v + 0.1);
                    break;
                case pb::FieldDescriptor::CPPTYPE_FLOAT:
                    field->is_repeated()
                        ? reflection->AddFloat(&message, field, static_cast<float>(v) + 0.5F)
                        : reflection->SetFloat(&message, field, static_cast<float>(v) + 0.5F);
                    break;
                case pb::FieldDescriptor::CPPTYPE_BOOL:
                    field->is_repeated() ? reflection->AddBool(&message, field, true)
                                         : reflection->SetBool(&message, field, true);
                    break;
                case pb::FieldDescriptor::CPPTYPE_ENUM: {
                    const pb::EnumDescriptor* e = field->enum_type();
                    const pb::EnumValueDescriptor* value = e->value(e->value_count() - 1);
                    field->is_repeated() ? reflection->AddEnum(&message, field, value)
                                         : reflection->SetEnum(&message, field, value);
                    break;
                }
                case pb::FieldDescriptor::CPPTYPE_STRING: {
                    // Bytes fields get a NUL and a non-UTF-8 byte; string fields get Japanese.
                    const std::string value = field->type() == pb::FieldDescriptor::TYPE_BYTES
                                                  ? std::string("\x00\xff", 2) + std::to_string(v)
                                                  : "左車輪_" + std::to_string(v);
                    field->is_repeated() ? reflection->AddString(&message, field, value)
                                         : reflection->SetString(&message, field, value);
                    break;
                }
                case pb::FieldDescriptor::CPPTYPE_MESSAGE: {
                    pb::Message* child = field->is_repeated()
                                             ? reflection->AddMessage(&message, field)
                                             : reflection->MutableMessage(&message, field);
                    fill(*child, depth + 1);
                    break;
                }
            }
        }
    }
}

TEST(Envelope, EveryBodyTypeRoundTrips) {
    const pb::Descriptor* descriptor = v1::Envelope::descriptor();
    const pb::OneofDescriptor* body = descriptor->FindOneofByName("body");
    ASSERT_NE(body, nullptr);
    ASSERT_GT(body->field_count(), 0);

    for (int i = 0; i < body->field_count(); ++i) {
        const pb::FieldDescriptor* field = body->field(i);
        SCOPED_TRACE(field->name());

        v1::Envelope original;
        fill(*original.GetReflection()->MutableMessage(&original, field));

        std::string wire;
        ASSERT_TRUE(original.SerializeToString(&wire));
        EXPECT_GT(wire.size(), 2U);

        v1::Envelope parsed;
        ASSERT_TRUE(parsed.ParseFromString(wire));
        EXPECT_EQ(parsed.body_case(), original.body_case());
        EXPECT_TRUE(pb::util::MessageDifferencer::Equals(original, parsed));
    }
}

TEST(Envelope, BodyFieldNumbersAreUnique) {
    // Guards against a copy-paste error when a new message is added to the oneof.
    const pb::OneofDescriptor* body = v1::Envelope::descriptor()->FindOneofByName("body");
    ASSERT_NE(body, nullptr);
    std::vector<int> numbers;
    for (int i = 0; i < body->field_count(); ++i) {
        numbers.push_back(body->field(i)->number());
    }
    std::sort(numbers.begin(), numbers.end());
    EXPECT_EQ(std::adjacent_find(numbers.begin(), numbers.end()), numbers.end());
}

TEST(SignalRun, RawSamplesSurviveBitExactly) {
    // Bulk numeric data travels as raw little-endian arrays in bytes fields.
    const std::vector<double> samples = {0.0, -0.0, 1e-300, 22.6, -51.3, 1.0 / 3.0};

    v1::SignalBlock block;
    block.set_catalog_generation(7);
    v1::SignalRun* run = block.add_runs();
    run->set_signal(42);
    run->mutable_t0()->set_ns(12'340'000'000);
    run->set_dt_ns(100'000);  // 10 kHz
    run->set_sample_count(static_cast<std::uint32_t>(samples.size()));
    run->set_components(1);
    run->set_encoding(v1::SAMPLE_ENCODING_F64_LE);
    run->set_first_index(123'400);
    run->set_samples(samples.data(), samples.size() * sizeof(double));

    std::string wire;
    ASSERT_TRUE(block.SerializeToString(&wire));
    v1::SignalBlock parsed;
    ASSERT_TRUE(parsed.ParseFromString(wire));

    ASSERT_EQ(parsed.runs_size(), 1);
    const std::string& bytes = parsed.runs(0).samples();
    ASSERT_EQ(bytes.size(), samples.size() * sizeof(double));
    EXPECT_EQ(std::memcmp(bytes.data(), samples.data(), bytes.size()), 0);
    EXPECT_EQ(parsed.runs(0).t0().ns(), 12'340'000'000);
    EXPECT_EQ(parsed.runs(0).first_index(), 123'400U);
}

TEST(SceneDelta, SequenceFieldsDetectLoss) {
    v1::SceneDelta delta;
    delta.set_sequence(101);
    delta.set_base_sequence(100);
    delta.add_transform_prims(3);
    delta.add_transform_prims(9);

    v1::SceneDelta parsed;
    ASSERT_TRUE(parsed.ParseFromString(delta.SerializeAsString()));
    EXPECT_EQ(parsed.sequence(), parsed.base_sequence() + 1);
    EXPECT_EQ(parsed.transform_prims_size(), 2);
}

TEST(Unit, CarriesDimensionScaleAndKind) {
    v1::Unit torque;  // N·m = kg·m²·s⁻²
    torque.set_length(2);
    torque.set_mass(1);
    torque.set_time(-2);
    torque.set_scale(1.0);
    torque.set_kind(1);
    torque.set_display("N·m");

    v1::Unit parsed;
    ASSERT_TRUE(parsed.ParseFromString(torque.SerializeAsString()));
    EXPECT_EQ(parsed.time(), -2);
    EXPECT_EQ(parsed.display(), "N·m");
}

}  // namespace
