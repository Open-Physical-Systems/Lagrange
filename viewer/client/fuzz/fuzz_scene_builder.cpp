// libFuzzer entry point. Built only with Clang and -DLAGRANGE_BUILD_FUZZERS=ON.

#include "SceneBuilderHarness.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    // A fresh builder for each input, and the same one twice: a delta after a snapshot.
    lagrange::viewer::SceneBuilder builder;
    const auto bytes = std::as_bytes(std::span{data, size});
    (void)lagrange::viewer::fuzz::runSceneBuilder(builder, bytes);
    (void)lagrange::viewer::fuzz::runSceneBuilder(builder, bytes);
    return 0;
}
