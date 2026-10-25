# Makes the vendored Jolt findable: find_package(jolt_vendor), then link Jolt::Jolt.
get_filename_component(_JOLT_VENDOR_PREFIX "${jolt_vendor_DIR}/../../.." ABSOLUTE)
find_package(
    Jolt
    CONFIG
    REQUIRED
    PATHS "${_JOLT_VENDOR_PREFIX}/opt/jolt_vendor"
    NO_DEFAULT_PATH
)
