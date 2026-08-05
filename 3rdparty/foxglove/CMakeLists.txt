# ============================================================================
# Foxglove WebSocket / MCAP SDK 配置
# 用于实时数据可视化、调试和录制
# ============================================================================

message(STATUS "    Configuring Foxglove SDK...")

set(FOXGLOVE_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(FOXGLOVE_LIBRARY "${FOXGLOVE_LIB_PATH}/libfoxglove.so")

if(NOT EXISTS "${FOXGLOVE_LIBRARY}")
    message(FATAL_ERROR "Foxglove library not found: ${FOXGLOVE_LIBRARY}")
endif()

# 将预编译动态库放入统一构建输出，可执行程序只依赖 $ORIGIN/../lib。
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
configure_file("${FOXGLOVE_LIBRARY}" "${CMAKE_BINARY_DIR}/lib/libfoxglove.so" COPYONLY)

add_library(foxglove_c SHARED IMPORTED GLOBAL)
set_target_properties(foxglove_c PROPERTIES
    IMPORTED_LOCATION "${CMAKE_BINARY_DIR}/lib/libfoxglove.so"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# libfoxglove.so 提供 C ABI；仓库内 src/ 是与头文件匹配的 C++ 封装层。
add_library(foxglove_sdk STATIC
    src/channel.cpp
    src/cloud_sink.cpp
    src/context.cpp
    src/error.cpp
    src/foxglove.cpp
    src/mcap.cpp
    src/schemas.cpp
    src/server.cpp
    src/server/connection_graph.cpp
    src/server/fetch_asset.cpp
    src/server/parameter.cpp
    src/server/service.cpp
)

target_include_directories(foxglove_sdk PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(foxglove_sdk PUBLIC foxglove_c)

set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH};$ORIGIN/../lib" PARENT_SCOPE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};$ORIGIN/../lib" PARENT_SCOPE)

message(STATUS "      ✓ Foxglove SDK include: ${CMAKE_CURRENT_SOURCE_DIR}/include")
message(STATUS "      ✓ Foxglove SDK library: ${CMAKE_BINARY_DIR}/lib/libfoxglove.so")
