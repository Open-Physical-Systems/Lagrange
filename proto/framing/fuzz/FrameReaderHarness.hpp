#pragma once

// Shared by the libFuzzer target and by a unit test that runs it on pseudo-random input.
// Treats `data` as bytes arriving from an untrusted peer: the first byte chooses how the rest
// is cut into chunks, every chunk is fed to a FrameReader, and every frame that comes out is
// parsed as an Envelope. Nothing here may crash, hang or read out of bounds.

#include "lagrange/proto/Framing.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace lagrange::proto::fuzz {

inline constexpr std::uint32_t kHarnessMaxFrameSize = 4096;

/// Returns the number of frames extracted.
inline std::size_t runFrameReader(std::span<const std::byte> data) {
    if (data.empty()) {
        return 0;
    }
    const std::size_t chunk = std::to_integer<std::size_t>(data.front()) % 64 + 1;
    data = data.subspan(1);

    FrameReader reader(kHarnessMaxFrameSize);
    lagrange::viewer::v1::Envelope envelope;
    std::size_t frames = 0;
    while (!data.empty()) {
        const std::size_t n = std::min(chunk, data.size());
        reader.feed(data.first(n));
        data = data.subspan(n);
        for (;;) {
            const FrameReader::Result result = reader.next();
            if (result.status != FrameReader::Status::Frame) {
                break;
            }
            ++frames;
            (void)parsePayload(result.payload, envelope);  // may fail; must not crash
        }
        if (reader.error() != FrameReader::Error::None) {
            break;
        }
    }
    return frames;
}

}  // namespace lagrange::proto::fuzz
