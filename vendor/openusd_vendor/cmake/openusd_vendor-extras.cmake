# Makes the vendored OpenUSD findable: find_package(openusd_vendor) then link
# openusd_vendor::usd, which carries OpenUSD's own targets plus the flags our -Werror builds need.
get_filename_component(
    _OPENUSD_VENDOR_PREFIX
    "${openusd_vendor_DIR}/../../.."
    ABSOLUTE
)
set(OPENUSD_VENDOR_ROOT "${_OPENUSD_VENDOR_PREFIX}/opt/openusd_vendor")
# The pxr Python package, for tools such as usdGenSchema run at build time.
file(
    GLOB OPENUSD_VENDOR_PYTHONPATH
    "${OPENUSD_VENDOR_ROOT}/lib/python3*/site-packages"
)

find_package(pxr CONFIG REQUIRED PATHS "${OPENUSD_VENDOR_ROOT}" NO_DEFAULT_PATH)

if(NOT TARGET openusd_vendor::usd)
    add_library(openusd_vendor::usd INTERFACE IMPORTED)
    set_target_properties(
        openusd_vendor::usd
        PROPERTIES
            INTERFACE_LINK_LIBRARIES "${PXR_LIBRARIES}"
            # OpenUSD headers, as system headers, so their warnings do not fail our builds.
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${PXR_INCLUDE_DIRS}"
            # pxr/base/tf/hash{set,map}.h include libstdc++'s deprecated <ext/hash_set>, whose
            # #warning passes even -isystem (OpenUSD spike, check 6).
            INTERFACE_COMPILE_DEFINITIONS "_GLIBCXX_PERMIT_BACKWARD_HASH"
            INTERFACE_LINK_OPTIONS "LINKER:-rpath,${OPENUSD_VENDOR_ROOT}/lib"
    )
endif()
set(openusd_vendor_LIBRARIES openusd_vendor::usd)
