# usdGenSchema leaves the placeholders that OpenUSD's own build fills in. A codeless schema
# library is a resource plugin found next to its plugInfo.json: no library, root and resources
# in the same directory.  Usage: cmake -DFILE=<plugInfo.json> -P fill_pluginfo.cmake
file(READ "${FILE}" text)
string(REPLACE "@PLUG_INFO_LIBRARY_PATH@" "" text "${text}")
string(REPLACE "@PLUG_INFO_RESOURCE_PATH@" "." text "${text}")
string(REPLACE "@PLUG_INFO_ROOT@" "." text "${text}")
file(WRITE "${FILE}" "${text}")
