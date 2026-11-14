#pragma once

// Shared by the libFuzzer target and by a unit test that runs it on pseudo-random input.
// Treats `data` as bytes arriving from an untrusted peer: the first byte chooses how the rest
// is cut into chunks (its low six bits) and the mode (its top bit), every chunk is fed to a
// FrameReader, and every frame that comes out is parsed as an Envelope. Nothing here may crash,
// hang or read out of bounds.
//
// The second mode is the server's handshake (RFC 0014 §1): the reader starts at a small limit
// and is raised to kHarnessMaxFrameSize after the first frame. It checks that no frame over
// the limit in force when it came out is ever returned, and that the reader never holds more
// than one fed chunk plus the limit plus a header; a breach aborts, which the fuzzer reports.

#include "lagrange/proto/Framing.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>

namespace lagrange::proto::fuzz {

inline constexpr std::uint32_t kHarnessMaxFrameSize = 4096;
/// The limit before the first frame in the second mode (a scaled kPreHandshakeMaxFrameSize).
inline constexpr std::uint32_t kHarnessSmallMaxFrameSize = 64;
/// The bit of the first byte that chooses the second mode.
inline constexpr unsigned char kRaiseLimitMode = 0x80;

/// Returns the number of frames extracted.
inline std::size_t runFrameReader(std::span<const std::byte> data) {
    if (data.empty()) {
        return 0;
    }
    const auto selector = std::to_integer<unsigned char>(data.front());
    const std::size_t chunk = selector % 64 + 1;
    const bool raiseAfterFirst = (selector & kRaiseLimitMode) != 0;
    data = data.subspan(1);

    FrameReader reader(raiseAfterFirst ? kHarnessSmallMaxFrameSize : kHarnessMaxFrameSize);
    lagrange::viewer::v1::Envelope envelope;
    std::size_t frames = 0;
    while (!data.empty()) {
        const std::size_t n = std::min(chunk, data.size());
        reader.feed(data.first(n));
        data = data.subspan(n);
        // Drained before this feed, the reader held less than a header plus the limit.
        if (reader.bufferedBytes() > n + kFrameHeaderSize + reader.maxFrameSize()) {
            std::abort();
        }
        for (;;) {
            const FrameReader::Result result = reader.next();
            if (result.status != FrameReader::Status::Frame) {
                break;
            }
            if (result.payload.size() > reader.maxFrameSize()) {
                std::abort();
            }
            ++frames;
            (void)parsePayload(result.payload, envelope);  // may fail; must not crash
            if (raiseAfterFirst && frames == 1) {
                reader.setMaxFrameSize(kHarnessMaxFrameSize);  // as the server after a hello
            }
        }
        if (reader.error() != FrameReader::Error::None) {
            break;
        }
    }
    return frames;
}

}  // namespace lagrange::proto::fuzz
