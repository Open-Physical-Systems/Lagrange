#pragma once

// Shared by the libFuzzer target and by a unit test that runs it on mutated protocol frames.
// Treats `data` as one Envelope from an untrusted server: a snapshot replaces the scene, a
// delta updates it; the builder must reject what is malformed and never crash, hang or read
// out of bounds (docs/design/security.md).

#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <cstddef>
#include <span>
#include <string>

namespace lagrange::viewer::fuzz {

/// Applies `data` (a serialised Envelope) to `builder`. Returns true when it was a snapshot or
/// delta the builder accepted.
inline bool runSceneBuilder(SceneBuilder& builder, std::span<const std::byte> data) {
    v1::Envelope envelope;
    if (!envelope.ParseFromArray(data.data(), static_cast<int>(data.size()))) {
        return false;
    }
    if (envelope.has_snapshot()) {
        std::string error;
        return builder.applySnapshot(envelope.snapshot(), error);
    }
    if (envelope.has_delta()) {
        return builder.applyDelta(envelope.delta()) == SceneBuilder::DeltaResult::Applied;
    }
    return false;
}

}  // namespace lagrange::viewer::fuzz
