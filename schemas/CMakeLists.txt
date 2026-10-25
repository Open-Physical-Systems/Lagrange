cmake_minimum_required(VERSION 3.22)
project(lagrange_schemas VERSION 0.0.0 LANGUAGES CXX) # CXX: pxrConfig finds TBB in multiarch paths

# Each schema library is generated codeless (RFC 0002 convention 7): usdGenSchema turns
# <library>/schema.usda into generatedSchema.usda and plugInfo.json, which USD loads through
# PXR_PLUGINPATH_NAME. The lagrangeUnit field is registered by the metadata plugin, which
# usdGenSchema needs to see too.
find_package(ament_cmake REQUIRED)
find_package(openusd_vendor REQUIRED)

set(LAGRANGE_SCHEMA_LIBRARIES sensor electrical thermal)
set(usd_env
    "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_SOURCE_DIR}/metadata"
    "PYTHONPATH=${OPENUSD_VENDOR_PYTHONPATH}"
    "LD_LIBRARY_PATH=${OPENUSD_VENDOR_ROOT}/lib"
)
set(generated_files)
foreach(library IN LISTS LAGRANGE_SCHEMA_LIBRARIES)
    set(out "${CMAKE_CURRENT_BINARY_DIR}/usd/${library}")
    add_custom_command(
        OUTPUT ${out}/generatedSchema.usda ${out}/plugInfo.json
        COMMAND ${CMAKE_COMMAND} -E make_directory ${out}
        COMMAND
            ${CMAKE_COMMAND} -E env ${usd_env} python3
            ${OPENUSD_VENDOR_ROOT}/bin/usdGenSchema
            ${CMAKE_CURRENT_SOURCE_DIR}/${library}/schema.usda ${out}
        COMMAND
            ${CMAKE_COMMAND} -DFILE=${out}/plugInfo.json -P
            ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fill_pluginfo.cmake
        DEPENDS
            ${library}/schema.usda
            metadata/plugInfo.json
            cmake/fill_pluginfo.cmake
        COMMENT "usdGenSchema ${library}"
        VERBATIM
    )
    list(
        APPEND generated_files
        ${out}/generatedSchema.usda
        ${out}/plugInfo.json
    )
    install(
        FILES
            ${out}/generatedSchema.usda
            ${out}/plugInfo.json
            ${library}/schema.usda
        DESTINATION share/${PROJECT_NAME}/usd/${library}
    )
endforeach()
add_custom_target(lagrange_schemas_generated ALL DEPENDS ${generated_files})
install(
    FILES metadata/plugInfo.json
    DESTINATION share/${PROJECT_NAME}/usd/metadata
)

ament_environment_hooks(env-hooks/lagrange_schemas.dsv.in)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    set(plugin_path "${CMAKE_CURRENT_SOURCE_DIR}/metadata")
    foreach(library IN LISTS LAGRANGE_SCHEMA_LIBRARIES)
        string(APPEND plugin_path ":${CMAKE_CURRENT_BINARY_DIR}/usd/${library}")
    endforeach()
    ament_add_pytest_test(
        test_schemas
        test/test_schemas.py
        ENV
            "PXR_PLUGINPATH_NAME=${plugin_path}"
            "PYTHONPATH=${OPENUSD_VENDOR_PYTHONPATH}"
            "LD_LIBRARY_PATH=${OPENUSD_VENDOR_ROOT}/lib"
            "LAGRANGE_SCHEMA_LIBRARIES=${LAGRANGE_SCHEMA_LIBRARIES}"
    )
endif()

ament_package()
