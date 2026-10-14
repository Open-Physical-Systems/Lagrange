cmake_minimum_required(VERSION 3.22)
project(lagrange_proto VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lagrange_cmake REQUIRED)
find_package(Protobuf REQUIRED)

set(LAGRANGE_PROTO_FILES
    lagrange/viewer/v1/common.proto
    lagrange/viewer/v1/session.proto
    lagrange/viewer/v1/scene.proto
    lagrange/viewer/v1/signals.proto
    lagrange/viewer/v1/command.proto
    lagrange/viewer/v1/profile.proto
    lagrange/viewer/v1/envelope.proto
    lagrange/record/v1/record.proto
)

set(_gen_cpp_dir ${CMAKE_CURRENT_BINARY_DIR}/gen/cpp)
set(_gen_python_dir ${CMAKE_CURRENT_BINARY_DIR}/gen/python)
file(MAKE_DIRECTORY ${_gen_cpp_dir} ${_gen_python_dir})

# C++ library. Generated code is not held to the project's warning level.
add_library(lagrange_proto ${LAGRANGE_PROTO_FILES})
add_library(lagrange::proto ALIAS lagrange_proto)
protobuf_generate(
    TARGET lagrange_proto
    LANGUAGE cpp
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    PROTOC_OUT_DIR ${_gen_cpp_dir}
)
target_compile_features(lagrange_proto PUBLIC cxx_std_20)
set_target_properties(
    lagrange_proto
    PROPERTIES CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON
)
target_include_directories(
    lagrange_proto
    SYSTEM
    PUBLIC
        $<BUILD_INTERFACE:${_gen_cpp_dir}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_proto PUBLIC protobuf::libprotobuf)

# Framing: hand-written code, held to the project's warning level and sanitizers.
add_library(lagrange_proto_framing framing/src/Framing.cpp)
add_library(lagrange::proto_framing ALIAS lagrange_proto_framing)
lagrange_target_defaults(lagrange_proto_framing)
target_include_directories(
    lagrange_proto_framing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/framing/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lagrange_proto_framing PUBLIC lagrange_proto)

# Transport: non-blocking Unix-socket/TCP connections and the shared-memory fast path.
find_path(LAGRANGE_XXHASH_INCLUDE_DIR xxhash.h REQUIRED)
add_library(
    lagrange_proto_transport
    transport/src/Connection.cpp
    transport/src/Endpoint.cpp
    transport/src/Shm.cpp
)
add_library(lagrange::proto_transport ALIAS lagrange_proto_transport)
lagrange_target_defaults(lagrange_proto_transport)
target_include_directories(
    lagrange_proto_transport
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/transport/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(
    lagrange_proto_transport
    SYSTEM
    PRIVATE ${LAGRANGE_XXHASH_INCLUDE_DIR}
)
target_link_libraries(lagrange_proto_transport PUBLIC lagrange_proto_framing)

# Python modules, generated into the build tree and installed as the namespace package
# lagrange.viewer.v1 (no __init__.py, so other packages can also provide lagrange.*).
set(_python_outputs "")
foreach(_proto IN LISTS LAGRANGE_PROTO_FILES)
    string(REGEX REPLACE "\\.proto$" "_pb2.py" _py ${_proto})
    list(APPEND _python_outputs ${_gen_python_dir}/${_py})
endforeach()
list(
    TRANSFORM LAGRANGE_PROTO_FILES
    PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/
    OUTPUT_VARIABLE _proto_paths
)
add_custom_command(
    OUTPUT ${_python_outputs}
    COMMAND
        protobuf::protoc --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
        --python_out=${_gen_python_dir} ${LAGRANGE_PROTO_FILES}
    DEPENDS ${_proto_paths}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating Python protobuf modules"
    VERBATIM
)
add_custom_target(lagrange_proto_python ALL DEPENDS ${_python_outputs})

install(
    DIRECTORY ${_gen_cpp_dir}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING
    PATTERN "*.pb.h"
)
install(
    DIRECTORY lagrange
    DESTINATION share/${PROJECT_NAME}/proto
    FILES_MATCHING
    PATTERN "*.proto"
)
# ament_cmake_python has no public helper for installing generated modules, so this uses the
# same internal macros as rosidl_generator_py to find the install directory and to register
# the PYTHONPATH environment hook.
_ament_cmake_python_get_python_install_dir()
_ament_cmake_python_register_environment_hook()
install(
    DIRECTORY ${_gen_python_dir}/lagrange
    DESTINATION ${PYTHON_INSTALL_DIR}
    FILES_MATCHING
    PATTERN "*_pb2.py"
)
install(
    DIRECTORY framing/include/ transport/include/
    DESTINATION include/${PROJECT_NAME}
)
install(
    TARGETS lagrange_proto lagrange_proto_framing lagrange_proto_transport
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_roundtrip test/test_roundtrip.cpp)
    target_link_libraries(test_roundtrip lagrange_proto)
    target_compile_options(test_roundtrip PRIVATE -Wall -Wextra -Wpedantic)

    ament_add_gtest(test_framing framing/test/test_framing.cpp)
    target_link_libraries(test_framing lagrange_proto_framing)
    target_include_directories(test_framing PRIVATE framing/fuzz)
    lagrange_target_defaults(test_framing)

    ament_add_gtest(test_golden test/test_golden.cpp)
    target_link_libraries(test_golden lagrange_proto_framing)
    target_compile_definitions(
        test_golden
        PRIVATE LAGRANGE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden"
    )
    lagrange_target_defaults(test_golden)

    find_package(Threads REQUIRED)
    ament_add_gtest(
        test_transport
        transport/test/test_transport.cpp
        transport/test/test_shm.cpp
    )
    target_link_libraries(
        test_transport
        lagrange_proto_transport
        Threads::Threads
    )
    lagrange_target_defaults(test_transport)

    find_package(ament_cmake_pytest REQUIRED)
    foreach(_test roundtrip golden)
        ament_add_pytest_test(
            test_${_test}_py
            test/test_${_test}.py
            APPEND_ENV
            PYTHONPATH=${_gen_python_dir}
        )
    endforeach()
endif()

option(LAGRANGE_BUILD_FUZZERS "Build libFuzzer targets (needs Clang)" OFF)
if(LAGRANGE_BUILD_FUZZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "LAGRANGE_BUILD_FUZZERS needs Clang (libFuzzer)")
    endif()
    add_executable(fuzz_frame_reader framing/fuzz/fuzz_frame_reader.cpp)
    target_link_libraries(fuzz_frame_reader PRIVATE lagrange_proto_framing)
    target_compile_options(fuzz_frame_reader PRIVATE -fsanitize=fuzzer)
    target_link_options(fuzz_frame_reader PRIVATE -fsanitize=fuzzer)
    lagrange_target_defaults(fuzz_frame_reader)
    install(TARGETS fuzz_frame_reader RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)
ament_package()
