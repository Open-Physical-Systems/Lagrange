# Makes the vendored FMI Library findable: find_package(fmilibrary3_vendor), then link
# fmilibrary3_vendor::fmilib. Its include directory also has the FMI 3.0 headers (FMI3/) that
# an FMU is compiled against.
get_filename_component(
    _FMIL_VENDOR_PREFIX
    "${fmilibrary3_vendor_DIR}/../../.."
    ABSOLUTE
)
set(_FMIL_ROOT "${_FMIL_VENDOR_PREFIX}/opt/fmilibrary3_vendor")
if(NOT TARGET fmilibrary3_vendor::fmilib)
    add_library(fmilibrary3_vendor::fmilib STATIC IMPORTED)
    set_target_properties(
        fmilibrary3_vendor::fmilib
        PROPERTIES
            IMPORTED_LOCATION "${_FMIL_ROOT}/lib/libfmilib.a"
            INTERFACE_INCLUDE_DIRECTORIES "${_FMIL_ROOT}/include"
            INTERFACE_LINK_LIBRARIES "${CMAKE_DL_LIBS}"
    )
endif()
set(fmilibrary3_vendor_INCLUDE_DIR "${_FMIL_ROOT}/include")
