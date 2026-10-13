# Script mode: turns a SPIR-V binary into a C++ header with a constexpr word array.
#   cmake -DINPUT=<file.spv> -DOUTPUT=<file.hpp> -DSYMBOL=<kName> -P EmbedSpirv.cmake
file(READ "${INPUT}" _hex HEX)
string(LENGTH "${_hex}" _length)
math(EXPR _remainder "${_length} % 8")
if(NOT _remainder EQUAL 0)
    message(FATAL_ERROR "${INPUT}: size is not a multiple of 4 bytes; not SPIR-V")
endif()
# SPIR-V words are little-endian: bytes b0 b1 b2 b3 become 0xb3b2b1b0.
string(
    REGEX REPLACE
    "([0-9a-f][0-9a-f])([0-9a-f][0-9a-f])([0-9a-f][0-9a-f])([0-9a-f][0-9a-f])"
    "0x\\4\\3\\2\\1U,"
    _words
    "${_hex}"
)
math(EXPR _count "${_length} / 8")
file(
    WRITE
    "${OUTPUT}"
    "// Generated from ${INPUT}. Do not edit.\n#pragma once\n\n#include <cstdint>\n\n"
    "inline constexpr std::uint32_t ${SYMBOL}[${_count}] = {${_words}};\n"
)
