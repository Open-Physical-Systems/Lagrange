include("${lagrange_cmake_DIR}/LagrangeCompileOptions.cmake")

# For tests that load a Vulkan driver or the window system:
#   ENV ASAN_OPTIONS=${LAGRANGE_GPU_TEST_ASAN_OPTIONS}
# Drivers are unloaded before LeakSanitizer runs, so their allocations are reported as leaks
# in "<unknown module>" and cannot be suppressed by name. Leak checking is therefore off for
# these tests only; every other AddressSanitizer check stays on, and tests that do not touch
# the GPU keep leak checking.
set(LAGRANGE_GPU_TEST_ASAN_OPTIONS "detect_leaks=0:strict_string_checks=1")
