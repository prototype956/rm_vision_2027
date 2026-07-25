# ============================================================================
# mv-hal-serial — 串口硬件抽象层
# ============================================================================
#
# 包含：
#   ISerial     — 纯虚接口（header-only）
#   UartSerial  — Linux termios 实现
#
# 仅依赖标准 POSIX API（termios、unistd），无需任何第三方库，
# 编译速度快，CI 无需任何额外依赖即可编译。
#
# 注意：串口协议层（帧格式/CRC/Kalman）在 Stage 3 的 mv-protocol 里实现，
# 此库只做纯粹的字节流 I/O。

add_library(mv-hal-serial STATIC
    uart_serial.cpp
    sim_serial.cpp
)

target_include_directories(mv-hal-serial
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv-hal-serial
    PUBLIC
        mv-core      # 依赖 Logger
        yaml-cpp
)

message(STATUS "    + mv-hal-serial (UartSerial + SimSerial)")
