cmake_minimum_required(VERSION 3.22)
project(lagrange_models VERSION 0.0.0 LANGUAGES C)

# Component behaviour models as FMI 3.0 co-simulation FMUs (CLAUDE.md §4). dc_drive is the
# FMU host's reference FMU, hand-written in C (Apache-2.0): open question Q8.
find_package(ament_cmake REQUIRED)
find_package(fmilibrary3_vendor REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The FMU binary directory for this machine (FMI 3.0 platform tuple).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
    set(FMU_PLATFORM x86_64-linux)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(FMU_PLATFORM aarch64-linux)
else()
    message(
        FATAL_ERROR
        "no FMI 3.0 platform tuple for ${CMAKE_SYSTEM_PROCESSOR}"
    )
endif()

set(stubs ${CMAKE_CURRENT_BINARY_DIR}/dc_drive/stubs.inc)
add_custom_command(
    OUTPUT ${stubs}
    COMMAND
        ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/dc_drive
    COMMAND
        Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/dc_drive/generate_stubs.py
        ${fmilibrary3_vendor_INCLUDE_DIR}/FMI3/fmi3FunctionTypes.h ${stubs}
    DEPENDS dc_drive/generate_stubs.py
    COMMENT "FMI 3.0 entry points of dc_drive"
    VERBATIM
)
add_library(dc_drive MODULE dc_drive/dc_drive.c ${stubs})
set_target_properties(
    dc_drive
    PROPERTIES
        PREFIX ""
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_VISIBILITY_PRESET hidden
)
target_include_directories(
    dc_drive
    PRIVATE
        ${fmilibrary3_vendor_INCLUDE_DIR}/FMI3
        ${CMAKE_CURRENT_BINARY_DIR}/dc_drive
)
target_compile_options(dc_drive PRIVATE -Wall -Wextra -Werror)
target_link_libraries(dc_drive PRIVATE m)

# The archive: modelDescription.xml and binaries/<platform>/dc_drive.so, zipped.
set(stage ${CMAKE_CURRENT_BINARY_DIR}/dc_drive_fmu)
set(fmu ${CMAKE_CURRENT_BINARY_DIR}/dc_drive.fmu)
add_custom_command(
    OUTPUT ${fmu}
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${stage}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${stage}/binaries/${FMU_PLATFORM}
    COMMAND
        ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/dc_drive/modelDescription.xml
        ${stage}/modelDescription.xml
    COMMAND
        ${CMAKE_COMMAND} -E copy $<TARGET_FILE:dc_drive>
        ${stage}/binaries/${FMU_PLATFORM}/dc_drive.so
    COMMAND
        ${CMAKE_COMMAND} -E chdir ${stage} ${CMAKE_COMMAND} -E tar cf ${fmu}
        --format=zip modelDescription.xml binaries
    DEPENDS dc_drive dc_drive/modelDescription.xml
    COMMENT "Packing dc_drive.fmu"
    VERBATIM
)
add_custom_target(dc_drive_fmu ALL DEPENDS ${fmu})

install(
    FILES ${fmu} dc_drive/dc_drive.fmu.map.yaml
    DESTINATION share/${PROJECT_NAME}/fmu
)

ament_package()
