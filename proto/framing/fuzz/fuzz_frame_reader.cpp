// libFuzzer entry point. Built only with Clang and -DLAGRANGE_BUILD_FUZZERS=ON.

#include "FrameReaderHarness.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    lagrange::proto::fuzz::runFrameReader(std::as_bytes(std::span{data, size}));
    return 0;
}
