# Compile options shared by every Lagrange target.
#
#   lagrange_target_defaults(<target>)
#   lagrange_test_private_tmpdir(<test target>...)
#   lagrange_test_ros_domain(<test target> <default domain>)
#
# Options:
#   LAGRANGE_WARNINGS_AS_ERRORS  Treat compiler warnings as errors.
#   LAGRANGE_DETERMINISTIC       Floating-point flags for bit-reproducible CPU results.
#   LAGRANGE_SANITIZER           Semicolon-separated sanitizers, e.g. "address;undefined".

option(LAGRANGE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(
    LAGRANGE_DETERMINISTIC
    "Floating-point flags for bit-reproducible CPU results"
    ON
)
set(LAGRANGE_SANITIZER
    ""
    CACHE STRING
    "Semicolon-separated list of sanitizers (address, undefined, thread)"
)

function(lagrange_target_defaults target)
    target_compile_features(${target} PUBLIC cxx_std_20)
    set_target_properties(
        ${target}
        PROPERTIES CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON
    )
    target_compile_options(
        ${target}
        PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
    if(LAGRANGE_WARNINGS_AS_ERRORS)
        target_compile_options(${target} PRIVATE -Werror)
    endif()
    if(LAGRANGE_DETERMINISTIC)
        # No fused multiply-add contraction: results must not depend on what the CPU supports.
        target_compile_options(${target} PRIVATE -ffp-contract=off)
    endif()
    if(LAGRANGE_SANITIZER)
        string(REPLACE ";" "," _lagrange_sanitizers "${LAGRANGE_SANITIZER}")
        target_compile_options(
            ${target}
            PRIVATE
                -fsanitize=${_lagrange_sanitizers}
                -fno-omit-frame-pointer
                # Jolt is built without RTTI, which UBSan's vptr check needs at link time.
                $<$<COMPILE_LANGUAGE:CXX>:-fno-sanitize=vptr>
                # GCC's flow analysis under sanitizers warns falsely inside std::regex.
                -Wno-error=maybe-uninitialized
                -Wno-error=uninitialized
        )
        target_link_options(
            ${target}
            PRIVATE -fsanitize=${_lagrange_sanitizers} -fno-sanitize=vptr
        )
    endif()
endfunction()

# Gives each run of a test executable a temporary directory of its own: before main() it makes
# $TMPDIR/lagrange-test-XXXXXX, points TMPDIR and TEST_TMPDIR at it (which
# ::testing::TempDir() and std::filesystem::temp_directory_path() read) and removes it at exit.
# Tests that write fixed names under the temporary directory can then run concurrently on one
# machine (several worktrees, CI jobs or agents) without overwriting each other's files.
function(lagrange_test_private_tmpdir)
    foreach(target IN LISTS ARGN)
        target_sources(
            ${target}
            PRIVATE
                "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/lagrange_test_tmpdir.cpp"
        )
    endforeach()
endfunction()

# A test that talks over DDS runs on $ROS_DOMAIN_ID when the environment sets it, else on
# <default domain>. (ament_add_gtest(... ENV ROS_DOMAIN_ID=N) would override the caller's.)
function(lagrange_test_ros_domain target default_domain)
    target_sources(
        ${target}
        PRIVATE
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/lagrange_test_ros_domain.cpp"
    )
    target_compile_definitions(
        ${target}
        PRIVATE LAGRANGE_TEST_ROS_DOMAIN_DEFAULT="${default_domain}"
    )
endfunction()

# For targets that use the Vulkan C API. Its structs are initialised as
# `VkFooCreateInfo info{VK_STRUCTURE_TYPE_FOO_CREATE_INFO};`, leaving the rest zero on purpose,
# which -Wextra reports as missing initialisers.
function(lagrange_target_vulkan_warnings target)
    target_compile_options(${target} PRIVATE -Wno-missing-field-initializers)
endfunction()
