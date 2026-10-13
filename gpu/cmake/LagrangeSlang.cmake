# Compiles a Slang entry point to SPIR-V at build time and embeds it in a generated header.
#
#   lagrange_add_slang_shader(<target>
#       SOURCE <file.slang> ENTRY <function> STAGE <vertex|fragment|compute> SYMBOL <kName>)
#
# The target can then `#include "shaders/<kName>.hpp"` and use `kName` as a
# std::span<const std::uint32_t>. GPU kernels are written once in Slang (invariant 8).

find_program(LAGRANGE_SLANGC slangc REQUIRED)
set(LAGRANGE_EMBED_SPIRV_SCRIPT
    "${CMAKE_CURRENT_LIST_DIR}/EmbedSpirv.cmake"
    CACHE INTERNAL
    "Script that embeds SPIR-V in a header"
)

function(lagrange_add_slang_shader target)
    cmake_parse_arguments(ARG "" "SOURCE;ENTRY;STAGE;SYMBOL" "" ${ARGN})
    foreach(_required SOURCE ENTRY STAGE SYMBOL)
        if(NOT ARG_${_required})
            message(FATAL_ERROR "lagrange_add_slang_shader: ${_required} is required")
        endif()
    endforeach()
    get_filename_component(_source "${ARG_SOURCE}" ABSOLUTE)
    set(_directory "${CMAKE_CURRENT_BINARY_DIR}/generated/${target}")
    set(_spirv "${_directory}/shaders/${ARG_SYMBOL}.spv")
    set(_header "${_directory}/shaders/${ARG_SYMBOL}.hpp")
    file(MAKE_DIRECTORY "${_directory}/shaders")
    add_custom_command(
        OUTPUT "${_header}"
        COMMAND
            "${LAGRANGE_SLANGC}" "${_source}" -target spirv -entry "${ARG_ENTRY}"
            -stage "${ARG_STAGE}" -o "${_spirv}"
        COMMAND
            "${CMAKE_COMMAND}" "-DINPUT=${_spirv}" "-DOUTPUT=${_header}"
            "-DSYMBOL=${ARG_SYMBOL}" -P "${LAGRANGE_EMBED_SPIRV_SCRIPT}"
        DEPENDS "${_source}" "${LAGRANGE_EMBED_SPIRV_SCRIPT}"
        COMMENT "Slang ${ARG_STAGE} shader ${ARG_SYMBOL}"
        VERBATIM
    )
    target_sources(${target} PRIVATE "${_header}")
    target_include_directories(${target} PRIVATE "${_directory}")
endfunction()
