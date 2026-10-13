#include "lagrange/proto/Framing.hpp"

#include <limits>

#include <google/protobuf/message_lite.h>

namespace lagrange::proto {

namespace {

// Consumed bytes are dropped from the front of the buffer once they are at least this many
// and at least half of it, so that the cost of moving the remainder stays amortised.
constexpr std::size_t kCompactThreshold = 64U * 1024U;

void writeLength(std::uint32_t length, std::byte* out) {
    out[0] = static_cast<std::byte>(length & 0xFFU);
    out[1] = static_cast<std::byte>((length >> 8U) & 0xFFU);
    out[2] = static_cast<std::byte>((length >> 16U) & 0xFFU);
    out[3] = static_cast<std::byte>((length >> 24U) & 0xFFU);
}

std::uint32_t readLength(const std::byte* in) {
    return std::to_integer<std::uint32_t>(in[0]) | (std::to_integer<std::uint32_t>(in[1]) << 8U) |
           (std::to_integer<std::uint32_t>(in[2]) << 16U) |
           (std::to_integer<std::uint32_t>(in[3]) << 24U);
}

}  // namespace

bool appendFrame(std::span<const std::byte> payload, std::vector<std::byte>& out,
                 std::uint32_t maxFrameSize) {
    if (payload.size() > maxFrameSize) {
        return false;
    }
    const std::size_t offset = out.size();
    out.resize(offset + kFrameHeaderSize);
    writeLength(static_cast<std::uint32_t>(payload.size()), out.data() + offset);
    out.insert(out.end(), payload.begin(), payload.end());
    return true;
}

bool appendFrame(const google::protobuf::MessageLite& message, std::vector<std::byte>& out,
                 std::uint32_t maxFrameSize) {
    const std::size_t size = message.ByteSizeLong();
    // SerializeToArray takes an int; protobuf itself cannot serialise more than 2 GiB.
    if (size > maxFrameSize || size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    const std::size_t offset = out.size();
    out.resize(offset + kFrameHeaderSize + size);
    writeLength(static_cast<std::uint32_t>(size), out.data() + offset);
    if (!message.SerializeToArray(out.data() + offset + kFrameHeaderSize, static_cast<int>(size))) {
        out.resize(offset);
        return false;
    }
    return true;
}

bool parsePayload(std::span<const std::byte> payload, google::protobuf::MessageLite& message) {
    if (payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    return message.ParseFromArray(payload.data(), static_cast<int>(payload.size()));
}

FrameReader::FrameReader(std::uint32_t maxFrameSize) : maxFrameSize_(maxFrameSize) {}

void FrameReader::feed(std::span<const std::byte> data) {
    if (error_ != Error::None) {
        return;  // the stream is dead; do not buffer anything more
    }
    compact();
    buffer_.insert(buffer_.end(), data.begin(), data.end());
}

FrameReader::Result FrameReader::next() {
    if (error_ != Error::None) {
        return {Status::Error, {}};
    }
    const std::size_t available = buffer_.size() - begin_;
    if (available < kFrameHeaderSize) {
        return {Status::NeedMore, {}};
    }
    const std::uint32_t length = readLength(buffer_.data() + begin_);
    if (length > maxFrameSize_) {
        error_ = Error::Oversize;
        buffer_.clear();
        buffer_.shrink_to_fit();
        begin_ = 0;
        return {Status::Error, {}};
    }
    if (available - kFrameHeaderSize < length) {
        return {Status::NeedMore, {}};
    }
    const std::span<const std::byte> payload{buffer_.data() + begin_ + kFrameHeaderSize, length};
    begin_ += kFrameHeaderSize + length;
    return {Status::Frame, payload};
}

void FrameReader::compact() {
    if (begin_ == buffer_.size()) {
        buffer_.clear();
        begin_ = 0;
    } else if (begin_ >= kCompactThreshold && begin_ >= buffer_.size() / 2) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(begin_));
        begin_ = 0;
    }
}

}  // namespace lagrange::proto
